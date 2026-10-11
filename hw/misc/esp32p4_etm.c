/*
 * ESP32-P4 event task matrix (SOC_ETM)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapter
 * 14 (Event Task Matrix), and ESP-IDF's soc_etm_reg.h for v3, which gives
 * the registers' layout, access and reset values, and soc_etm_source.h,
 * the events and tasks; its hal etm_ll.h and esp_hw_support's esp_etm.c
 * show how software drives the block.
 *
 * Fifty channels, each mapping the event SOC_ETM_CHn_EVT_ID selects to the
 * task SOC_ETM_CHn_TASK_ID selects while it is enabled: writing 1 to a bit
 * of CH_ENA_ADn_SET enables a channel, to CH_ENA_ADn_CLR disables it, and
 * CH_ENA_ADn shows which are. An ID of 0, or one above the last event
 * (234) or task (207), selects nothing, so that the channel maps nothing
 * (TRM 14.3.6: it is "disabled"), though CH_ENA_ADn still shows it as
 * enabled, the bit being one only the SET and CLR registers change.
 *
 * The ETM runs on SYS_CLK (TRM 14.3.5). An event is a pulse from its
 * peripheral; the ETM receives it in the SYS_CLK cycle it comes in, if
 * that clock runs and the block is out of reset, and sets its bit of
 * EVT_STn whether or not a channel maps it. In the same cycle each enabled
 * channel that maps it generates its task, a pulse to the peripheral that
 * takes it, setting the task's bit of TASK_STn. The TRM gives no latency
 * for the clock domain crossings of figure 14.3-2, so the model adds none:
 * the task comes at the virtual time of the event, and so does an event a
 * task's peripheral answers with, and its tasks. What the TRM does give is
 * that a peripheral's pulses closer than a SYS_CLK cycle are not each
 * received, and that a task several channels generate in the same cycle
 * is generated once: an event is received, and a task generated, at most
 * once in a cycle of SYS_CLK, counted from the time it last changed.
 * Events are received in the order they come, and the tasks of one event
 * generated in the order of their IDs. The time of an event is its
 * line's (esp32_line_time()), and its tasks' pulses carry it.
 *
 * The status bits are cleared by writing 1 to EVT_STn_CLR and TASK_STn_CLR.
 * SOC_ETM_CLK_EN, the register file's clock gating, has no effect software
 * can see and is held as written, as DATE is.
 *
 * RTC_EVT_TICK is every edge of the RTC timer's counting clock, which comes
 * in as a clock: while a channel maps it the ETM takes each edge as the
 * event, and otherwise only sets its status bit when software would see
 * it.
 *
 * The registers are clocked by ETM_APB_CLK; the machine maps them behind
 * it. RST_EN_ETM resets the whole block (TRM 14.3.6, step 6) and holds it
 * there: no event is received and writes are lost.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/gpio/esp32_line.h"
#include "hw/misc/esp32p4_etm.h"
#include "migration/vmstate.h"

#define A_CH_ENA_AD0        0x000
#define A_CH_ENA_AD0_SET    0x004
#define A_CH_ENA_AD0_CLR    0x008
#define A_CH_ENA_AD1        0x00c
#define A_CH_ENA_AD1_SET    0x010
#define A_CH_ENA_AD1_CLR    0x014
#define A_CH_ID0            0x018
#define A_CH_ID_LAST        (A_CH_ID0 + 8 * ESP32P4_ETM_CHANNELS - 4)
#define A_EVT_ST0           0x1a8
#define A_EVT_ST_LAST       (A_EVT_ST0 + 8 * ESP32P4_ETM_EVT_WORDS - 4)
#define A_TASK_ST0          0x1e8
#define A_TASK_ST_LAST      (A_TASK_ST0 + 8 * ESP32P4_ETM_TASK_WORDS - 4)
#define A_CLK_EN            0x220
#define A_DATE              0x224

#define CH_ENA_AD1_MASK     MAKE_64BIT_MASK(0, ESP32P4_ETM_CHANNELS - 32)
#define ID_MASK             0xff
#define CLK_EN_MASK         0x1
#define DATE_MASK           0x0fffffff
#define DATE_RESET          0x02303031

/* The bits of status word @k of @n bits, one per ID from 32 * k + 1. */
static uint32_t etm_st_mask(int k, int n)
{
    return k < (n - 1) / 32 ? UINT32_MAX : MAKE_64BIT_MASK(0, n - 32 * k);
}

static bool etm_running(ESP32P4EtmState *s)
{
    return !s->in_reset && clock_is_enabled(s->clk);
}

/* Whether times @last and @t fall in the same cycle of SYS_CLK. */
static bool etm_same_cycle(ESP32P4EtmState *s, int64_t last, int64_t t)
{
    if (last < s->clk_ns || t < s->clk_ns) {
        return false;
    }
    return clock_ns_to_ticks(s->clk, last - s->clk_ns) ==
           clock_ns_to_ticks(s->clk, t - s->clk_ns);
}

/*
 * [spec:nuos:req:emu.esp32p4.etm]
 * Task @id, for an event at time @t: the pulse carries @t, so that a
 * receiver timing it by esp32_line_time() acts at the event's time.
 */
static void etm_generate(ESP32P4EtmState *s, int id, int64_t t)
{
    if (etm_same_cycle(s, s->task_ns[id], t)) {
        return;
    }
    s->task_ns[id] = t;
    s->task_st[(id - 1) / 32] |= BIT((id - 1) % 32);
    esp32_line_set_at(s->task[id], 1, t);
    esp32_line_set_at(s->task[id], 0, t);
}

/*
 * [spec:nuos:req:emu.esp32p4.etm]
 * Event @id, at time @t, while the ETM runs: its status, and the tasks of
 * the enabled channels that map it.
 */
static void etm_receive(ESP32P4EtmState *s, int id, int64_t t)
{
    DECLARE_BITMAP(tasks, ESP32P4_ETM_TASKS);
    unsigned long task;
    int ch;

    if (etm_same_cycle(s, s->evt_ns[id], t)) {
        return;
    }
    s->evt_ns[id] = t;
    s->evt_st[(id - 1) / 32] |= BIT((id - 1) % 32);

    bitmap_zero(tasks, ESP32P4_ETM_TASKS);
    for (ch = 0; ch < ESP32P4_ETM_CHANNELS; ch++) {
        if (extract64(s->ch_ena, ch, 1) && s->evt_id[ch] == id &&
            s->task_id[ch] && s->task_id[ch] < ESP32P4_ETM_TASKS) {
            set_bit(s->task_id[ch], tasks);
        }
    }
    /* A task's peripheral may answer with an event, received in turn. */
    for (task = find_first_bit(tasks, ESP32P4_ETM_TASKS);
         task < ESP32P4_ETM_TASKS;
         task = find_next_bit(tasks, ESP32P4_ETM_TASKS, task + 1)) {
        etm_generate(s, task, t);
    }
}

/* [spec:nuos:req:emu.esp32p4.etm] */
static void etm_event_in(void *opaque, int id, int level)
{
    ESP32P4EtmState *s = opaque;

    if (!level || !id || !etm_running(s)) {
        return;
    }
    etm_receive(s, id, esp32_line_time());
}

/* Whether a channel maps RTC_EVT_TICK to a task while the ETM runs. */
static bool etm_tick_mapped(ESP32P4EtmState *s)
{
    int ch;

    if (!etm_running(s) || !clock_is_enabled(s->rtc_tick)) {
        return false;
    }
    for (ch = 0; ch < ESP32P4_ETM_CHANNELS; ch++) {
        if (extract64(s->ch_ena, ch, 1) &&
            s->evt_id[ch] == ESP32P4_ETM_RTC_EVT_TICK &&
            s->task_id[ch] && s->task_id[ch] < ESP32P4_ETM_TASKS) {
            return true;
        }
    }
    return false;
}

/* The time of RTC_EVT_TICK edge @n since @tick_ns, capped for timer_mod(). */
static int64_t etm_tick_edge_ns(ESP32P4EtmState *s, uint64_t n)
{
    uint64_t ns = clock_ticks_to_ns(s->rtc_tick, n);

    while (ns < INT64_MAX / 4 && clock_ns_to_ticks(s->rtc_tick, ns) < n) {
        ns++;
    }
    return s->tick_ns + MIN(ns, INT64_MAX / 4);
}

/*
 * [spec:nuos:req:emu.esp32p4.etm]
 * Take the RTC_EVT_TICK edges up to @now, as the ETM was configured over
 * them: each as the event while a channel maps it, otherwise once, for
 * its status, while the ETM runs; none while it does not.
 */
static void etm_tick_fold(ESP32P4EtmState *s, int64_t now)
{
    uint64_t edges;

    if (!clock_is_enabled(s->rtc_tick) || now <= s->tick_ns) {
        return;
    }
    edges = clock_ns_to_ticks(s->rtc_tick, now - s->tick_ns);
    if (edges <= s->tick_seen) {
        return;
    }
    if (!etm_running(s)) {
        s->tick_seen = edges;
        return;
    }
    if (!etm_tick_mapped(s)) {
        s->tick_seen = edges;
        etm_receive(s, ESP32P4_ETM_RTC_EVT_TICK,
                    etm_tick_edge_ns(s, edges));
        return;
    }
    while (s->tick_seen < edges) {
        s->tick_seen++;
        etm_receive(s, ESP32P4_ETM_RTC_EVT_TICK,
                    etm_tick_edge_ns(s, s->tick_seen));
    }
}

/* Time the next RTC_EVT_TICK while a channel maps it. */
static void etm_tick_schedule(ESP32P4EtmState *s)
{
    if (!etm_tick_mapped(s)) {
        timer_del(&s->tick_timer);
        return;
    }
    timer_mod(&s->tick_timer, etm_tick_edge_ns(s, s->tick_seen + 1));
}

static void etm_tick_cb(void *opaque)
{
    ESP32P4EtmState *s = opaque;

    etm_tick_fold(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    etm_tick_schedule(s);
}

/* [spec:nuos:req:emu.esp32p4.etm] */
static void etm_tick_update(void *opaque, ClockEvent event)
{
    ESP32P4EtmState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (event == ClockPreUpdate) {
        etm_tick_fold(s, now);
        return;
    }
    s->tick_ns = now;
    s->tick_seen = 0;
    etm_tick_schedule(s);
}

/* [spec:nuos:req:emu.esp32p4.etm] */
static void etm_clk_update(void *opaque, ClockEvent event)
{
    ESP32P4EtmState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (event == ClockPreUpdate) {
        etm_tick_fold(s, now);
        return;
    }
    s->clk_ns = now;
    etm_tick_schedule(s);
}

static void etm_reset_registers(ESP32P4EtmState *s)
{
    int i;

    s->ch_ena = 0;
    memset(s->evt_id, 0, sizeof(s->evt_id));
    memset(s->task_id, 0, sizeof(s->task_id));
    memset(s->evt_st, 0, sizeof(s->evt_st));
    memset(s->task_st, 0, sizeof(s->task_st));
    s->clk_en = 0;
    s->date = DATE_RESET;
    for (i = 0; i < ESP32P4_ETM_EVENTS; i++) {
        s->evt_ns[i] = INT64_MIN;
    }
    for (i = 0; i < ESP32P4_ETM_TASKS; i++) {
        s->task_ns[i] = INT64_MIN;
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.etm]
 * RST_EN_ETM: the block resets as it is asserted, and stays so.
 */
static void etm_set_reset(void *opaque, int n, int level)
{
    ESP32P4EtmState *s = opaque;

    if (!!level == s->in_reset) {
        return;
    }
    etm_tick_fold(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    s->in_reset = level;
    etm_reset_registers(s);
    etm_tick_schedule(s);
}

static bool etm_read_reg(ESP32P4EtmState *s, hwaddr addr, uint32_t *v)
{
    int k;

    switch (addr) {
    case A_CH_ENA_AD0:
        *v = (uint32_t)s->ch_ena;
        return true;
    case A_CH_ENA_AD1:
        *v = s->ch_ena >> 32;
        return true;
    case A_CH_ENA_AD0_SET:
    case A_CH_ENA_AD0_CLR:
    case A_CH_ENA_AD1_SET:
    case A_CH_ENA_AD1_CLR:
        *v = 0;
        return true;
    case A_CH_ID0 ... A_CH_ID_LAST:
        k = (addr - A_CH_ID0) / 8;
        *v = (addr & 4) ? s->task_id[k] : s->evt_id[k];
        return true;
    case A_EVT_ST0 ... A_EVT_ST_LAST:
        if (addr & 4) {
            *v = 0;
        } else {
            k = (addr - A_EVT_ST0) / 8;
            if (k == (ESP32P4_ETM_RTC_EVT_TICK - 1) / 32) {
                etm_tick_fold(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
                etm_tick_schedule(s);
            }
            *v = s->evt_st[k];
        }
        return true;
    case A_TASK_ST0 ... A_TASK_ST_LAST:
        *v = (addr & 4) ? 0 : s->task_st[(addr - A_TASK_ST0) / 8];
        return true;
    case A_CLK_EN:
        *v = s->clk_en;
        return true;
    case A_DATE:
        *v = s->date;
        return true;
    default:
        return false;
    }
}

static uint64_t etm_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4EtmState *s = opaque;
    uint32_t v;

    if (!etm_read_reg(s, addr & ~3, &v)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of reserved offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_ETM, addr);
        return 0;
    }
    return extract32(v, (addr & 3) * 8, size * 8);
}

/*
 * [spec:nuos:req:emu.esp32p4.etm]
 * Write the register at word offset @addr: @v is the register with the
 * bytes written replaced, @ones only the bytes written, for the
 * write-1-to-set and -clear registers.
 */
static void etm_write_reg(ESP32P4EtmState *s, hwaddr addr, uint32_t v,
                          uint32_t ones)
{
    int k;

    switch (addr) {
    case A_CH_ENA_AD0_SET:
        s->ch_ena |= ones;
        break;
    case A_CH_ENA_AD0_CLR:
        s->ch_ena &= ~(uint64_t)ones;
        break;
    case A_CH_ENA_AD1_SET:
        s->ch_ena |= (uint64_t)(ones & CH_ENA_AD1_MASK) << 32;
        break;
    case A_CH_ENA_AD1_CLR:
        s->ch_ena &= ~((uint64_t)(ones & CH_ENA_AD1_MASK) << 32);
        break;
    case A_CH_ID0 ... A_CH_ID_LAST:
        k = (addr - A_CH_ID0) / 8;
        if (addr & 4) {
            s->task_id[k] = v & ID_MASK;
            if (s->task_id[k] >= ESP32P4_ETM_TASKS) {
                qemu_log_mask(LOG_GUEST_ERROR, "%s: channel %d maps "
                              "task %u, which does not exist\n",
                              TYPE_ESP32P4_ETM, k, s->task_id[k]);
            }
        } else {
            s->evt_id[k] = v & ID_MASK;
            if (s->evt_id[k] >= ESP32P4_ETM_EVENTS) {
                qemu_log_mask(LOG_GUEST_ERROR, "%s: channel %d maps "
                              "event %u, which does not exist\n",
                              TYPE_ESP32P4_ETM, k, s->evt_id[k]);
            }
        }
        break;
    case A_EVT_ST0 ... A_EVT_ST_LAST:
        k = (addr - A_EVT_ST0) / 8;
        if (!(addr & 4)) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only "
                          "EVT_ST%d\n", TYPE_ESP32P4_ETM, k);
            break;
        }
        s->evt_st[k] &= ~(ones & etm_st_mask(k, ESP32P4_ETM_EVENTS - 1));
        break;
    case A_TASK_ST0 ... A_TASK_ST_LAST:
        k = (addr - A_TASK_ST0) / 8;
        if (!(addr & 4)) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only "
                          "TASK_ST%d\n", TYPE_ESP32P4_ETM, k);
            break;
        }
        s->task_st[k] &= ~(ones & etm_st_mask(k, ESP32P4_ETM_TASKS - 1));
        break;
    case A_CLK_EN:
        s->clk_en = v & CLK_EN_MASK;
        break;
    case A_DATE:
        s->date = v & DATE_MASK;
        break;
    case A_CH_ENA_AD0:
    case A_CH_ENA_AD1:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only CH_ENA_AD%d; "
                      "channels are enabled through CH_ENA_AD%d_SET and "
                      "disabled through _CLR\n", TYPE_ESP32P4_ETM,
                      addr == A_CH_ENA_AD1, addr == A_CH_ENA_AD1);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to reserved offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_ETM, addr);
        break;
    }
}

static void etm_write(void *opaque, hwaddr addr, uint64_t data,
                      unsigned size)
{
    ESP32P4EtmState *s = opaque;
    unsigned shift = (addr & 3) * 8;
    uint32_t lanes = MAKE_64BIT_MASK(shift, size * 8);
    uint32_t ones = (data << shift) & lanes;
    uint32_t old = 0;

    if (s->in_reset) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to 0x%" HWADDR_PRIx
                      " while the block is held in reset (RST_EN_ETM) is "
                      "lost\n", TYPE_ESP32P4_ETM, addr);
        return;
    }
    /* Channel changes take effect after the RTC_EVT_TICK edges before. */
    etm_tick_fold(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    etm_read_reg(s, addr & ~3, &old);
    etm_write_reg(s, addr & ~3, (old & ~lanes) | ones, ones);
    etm_tick_schedule(s);
}

static const MemoryRegionOps etm_ops = {
    .read = etm_read,
    .write = etm_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

/* [spec:nuos:req:emu.esp32p4.etm] */
void esp32p4_etm_connect_event(ESP32P4EtmState *s, int id, DeviceState *dev,
                               const char *name, int n)
{
    assert(id > 0 && id < ESP32P4_ETM_EVENTS);
    assert(!test_bit(id, s->evt_wired));
    set_bit(id, s->evt_wired);
    qdev_connect_gpio_out_named(dev, name, n,
                                qdev_get_gpio_in_named(DEVICE(s),
                                                       ESP32P4_ETM_EVENT,
                                                       id));
}

/* [spec:nuos:req:emu.esp32p4.etm] */
void esp32p4_etm_connect_task(ESP32P4EtmState *s, int id, DeviceState *dev,
                              const char *name, int n)
{
    assert(id > 0 && id < ESP32P4_ETM_TASKS);
    assert(!test_bit(id, s->task_wired));
    set_bit(id, s->task_wired);
    qdev_connect_gpio_out_named(DEVICE(s), ESP32P4_ETM_TASK, id,
                                qdev_get_gpio_in_named(dev, name, n));
}

/* [spec:nuos:req:emu.esp32p4.etm] */
static void esp32p4_etm_reset_hold(Object *obj, ResetType type)
{
    ESP32P4EtmState *s = ESP32P4_ETM(obj);

    etm_tick_fold(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    etm_reset_registers(s);
    timer_del(&s->tick_timer);
}

static void esp32p4_etm_reset_exit(Object *obj, ResetType type)
{
    etm_tick_schedule(ESP32P4_ETM(obj));
}

static void esp32p4_etm_init(Object *obj)
{
    ESP32P4EtmState *s = ESP32P4_ETM(obj);
    DeviceState *dev = DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &etm_ops, s, "esp32p4.etm",
                          ESP32P4_ETM_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    s->clk = qdev_init_clock_in(dev, "clk", etm_clk_update, s,
                                ClockPreUpdate | ClockUpdate);
    s->rtc_tick = qdev_init_clock_in(dev, "rtc-evt-tick", etm_tick_update,
                                     s, ClockPreUpdate | ClockUpdate);
    timer_init_ns(&s->tick_timer, QEMU_CLOCK_VIRTUAL, etm_tick_cb, s);
    qdev_init_gpio_in_named(dev, etm_event_in, ESP32P4_ETM_EVENT,
                            ESP32P4_ETM_EVENTS);
    qdev_init_gpio_out_named(dev, s->task, ESP32P4_ETM_TASK,
                             ESP32P4_ETM_TASKS);
    qdev_init_gpio_in_named(dev, etm_set_reset, ESP32P4_ETM_RESET, 1);
}

static const VMStateDescription vmstate_esp32p4_etm = {
    .name = TYPE_ESP32P4_ETM,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_CLOCK(clk, ESP32P4EtmState),
        VMSTATE_CLOCK(rtc_tick, ESP32P4EtmState),
        VMSTATE_BOOL(in_reset, ESP32P4EtmState),
        VMSTATE_UINT64(ch_ena, ESP32P4EtmState),
        VMSTATE_UINT8_ARRAY(evt_id, ESP32P4EtmState, ESP32P4_ETM_CHANNELS),
        VMSTATE_UINT8_ARRAY(task_id, ESP32P4EtmState, ESP32P4_ETM_CHANNELS),
        VMSTATE_UINT32_ARRAY(evt_st, ESP32P4EtmState, ESP32P4_ETM_EVT_WORDS),
        VMSTATE_UINT32_ARRAY(task_st, ESP32P4EtmState,
                             ESP32P4_ETM_TASK_WORDS),
        VMSTATE_UINT32(clk_en, ESP32P4EtmState),
        VMSTATE_UINT32(date, ESP32P4EtmState),
        VMSTATE_INT64(clk_ns, ESP32P4EtmState),
        VMSTATE_INT64_ARRAY(evt_ns, ESP32P4EtmState, ESP32P4_ETM_EVENTS),
        VMSTATE_INT64_ARRAY(task_ns, ESP32P4EtmState, ESP32P4_ETM_TASKS),
        VMSTATE_INT64(tick_ns, ESP32P4EtmState),
        VMSTATE_UINT64(tick_seen, ESP32P4EtmState),
        VMSTATE_TIMER(tick_timer, ESP32P4EtmState),
        VMSTATE_END_OF_LIST()
    },
};

static void esp32p4_etm_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->vmsd = &vmstate_esp32p4_etm;
    rc->phases.hold = esp32p4_etm_reset_hold;
    rc->phases.exit = esp32p4_etm_reset_exit;
}

/* [spec:nuos:req:emu.esp32p4.etm] */
static const TypeInfo esp32p4_etm_info = {
    .name          = TYPE_ESP32P4_ETM,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32P4EtmState),
    .instance_init = esp32p4_etm_init,
    .class_init    = esp32p4_etm_class_init,
};

static void esp32p4_etm_register_types(void)
{
    type_register_static(&esp32p4_etm_info);
}
type_init(esp32p4_etm_register_types)
