/*
 * ESP32-P4 pulse count controller (PCNT)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 TRM (v3.x silicon), "Pulse Count Controller
 * (PCNT)", with the field layout, access types and reset values of
 * ESP-IDF's v3 pcnt_reg.h and pcnt_struct.h, and the use ESP-IDF's PCNT
 * driver (esp_driver_pcnt) and hal/pcnt_ll.h make of them. The counting
 * follows the ESP32's PCNT (hw/misc/esp32_pcnt.c).
 *
 * Four units each keep a 16-bit signed counter driven by two channels. A
 * channel counts the edges of its signal input, up, down or not at all
 * per edge direction (POS_MODE, NEG_MODE), with the action kept,
 * inverted or inhibited by the level of its control input (HCTRL_MODE,
 * LCTRL_MODE). All four inputs of a unit pass through the unit's glitch
 * filter, clocked by APB_CLK: with FILTER_EN, a level change reaches the
 * counter once it has held for FILTER_THRES cycles, and a shorter pulse
 * is lost. Each unit also has a clear signal, which holds its counter at
 * 0 while high, as PULSE_CNT_RST does; it is not filtered.
 *
 * Five comparators watch each counter as it counts: the two thresholds
 * and zero match on equality, the high limit on count >= H_LIM and the
 * low limit on count <= L_LIM, and a limit match also returns the counter
 * to 0. Comparators are evaluated on a step of the counter only, not when
 * it is cleared or a comparator's value changes. A step that matches any
 * enabled comparator raises the unit's interrupt and latches which
 * comparators matched in the unit's STATUS register, replacing the
 * previous latch. THRES0 and THRES1 take effect at once; H_LIM and L_LIM,
 * as the TRM (51.4) and ESP-IDF's driver say, when the counter is cleared
 * and after a step matching an enabled comparator.
 *
 * Input changes are timed by the virtual time stamps of the wire's
 * changes (hw/gpio/esp32_line.h), not by when the model happens to run,
 * so a burst of edges delivered late, or a late timer, cannot lose an edge
 * or reorder edges between inputs; whenever software reads the registers,
 * the line sources are brought up to the present first.
 *
 * Behaviour the TRM leaves open, as modelled:
 * - The step comparator (PCNT_DELTA_CHANGE_EN_Un, PCNT_Un_CHANGE_CONF),
 *   which the P4's TRM does not describe and ESP-IDF does not use on the
 *   P4, works as on the ESP32-H2, whose register layout it shares and
 *   whose driver tests pin it down: a step to the last step point plus
 *   CNT_STEP is a step point, raising the unit's interrupt, and a step to
 *   CNT_STEP_LIM returns the counter to 0 as a limit does, raising it
 *   too; clearing the counter makes 0 the last step point. The P4's
 *   STATUS registers have no bits for these events, so they latch none.
 * - Units' CHANGE_CONF registers are in reverse order: unit 3's at 0x64,
 *   unit 0's at 0x70 (pcnt_struct.h).
 * - CLK_EN, the register clock gate's "always on", is kept as a plain bit.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/gpio/esp32_line.h"
#include "hw/misc/esp32p4_clk_period.h"
#include "hw/misc/esp32p4_pcnt.h"
#include "migration/vmstate.h"

#define NU                      ESP32P4_PCNT_UNITS
#define UI                      ESP32P4_PCNT_UNIT_INPUTS

#define A_CONF0(u)              (0x00 + 0x0c * (u))
#define A_CONF1(u)              (0x04 + 0x0c * (u))
#define A_CONF2(u)              (0x08 + 0x0c * (u))
#define A_CNT(u)                (0x30 + 0x04 * (u))
#define A_INT_RAW               0x40
#define A_INT_ST                0x44
#define A_INT_ENA               0x48
#define A_INT_CLR               0x4c
#define A_STATUS(u)             (0x50 + 0x04 * (u))
#define A_CTRL                  0x60
#define A_CHANGE_CONF(u)        (0x70 - 0x04 * (u))
#define A_DATE                  0xfc

#define CONF0_FILTER_THRES      0x3ffu
#define CONF0_FILTER_EN         (1u << 10)
#define CONF0_THR_ZERO_EN       (1u << 11)
#define CONF0_THR_H_LIM_EN      (1u << 12)
#define CONF0_THR_L_LIM_EN      (1u << 13)
#define CONF0_THR_THRES0_EN     (1u << 14)
#define CONF0_THR_THRES1_EN     (1u << 15)
/* Channel ch's NEG_MODE, POS_MODE, HCTRL_MODE and LCTRL_MODE, 2 bits each */
#define CONF0_CH_SHIFT(ch)      (16 + 8 * (ch))
#define CONF0_RESET             0x00003c10u

#define STATUS_ZERO_MODE        0x3u
#define STATUS_THRES1_LAT       (1u << 2)
#define STATUS_THRES0_LAT       (1u << 3)
#define STATUS_L_LIM_LAT        (1u << 4)
#define STATUS_H_LIM_LAT        (1u << 5)
#define STATUS_ZERO_LAT         (1u << 6)

/* ZERO_MODE: how the counter last stood relative to zero */
#define ZERO_MODE_POS_ZERO      0
#define ZERO_MODE_NEG_ZERO      1
#define ZERO_MODE_NEG           2
#define ZERO_MODE_POS           3

#define CTRL_CNT_RST(u)         (1u << (2 * (u)))
#define CTRL_CNT_PAUSE(u)       (1u << (2 * (u) + 1))
#define CTRL_DELTA_CHANGE_EN(u) (1u << (8 + (u)))
#define CTRL_CLK_EN             (1u << 16)
#define CTRL_MASK               0x00010fffu
#define CTRL_RESET              0x00000055u

#define INT_MASK                0xfu
#define DATE_RESET              0x02211301u

/* Edge modes and control modes */
#define EDGE_INC                1
#define EDGE_DEC                2
#define CTRL_KEEP               0
#define CTRL_INVERT             1

static void pcnt_update_irq(ESP32P4PcntState *s)
{
    qemu_set_irq(s->irq, (s->int_raw & s->int_ena) != 0);
}

/* Unit u's counter is held at 0, by PULSE_CNT_RST or its clear signal. */
static bool pcnt_held(ESP32P4PcntState *s, unsigned u)
{
    return (s->ctrl & CTRL_CNT_RST(u)) || s->raw[u * UI + ESP32P4_PCNT_RST];
}

/* Take CONF2's limits into effect. */
static void pcnt_take_limits(ESP32P4PcntUnit *un)
{
    un->h_lim = (int16_t)extract32(un->conf2, 0, 16);
    un->l_lim = (int16_t)extract32(un->conf2, 16, 16);
}

/* [spec:nuos:req:emu.esp32p4.rmt-pcnt] */
static void pcnt_clear(ESP32P4PcntState *s, unsigned u)
{
    ESP32P4PcntUnit *un = &s->unit[u];

    un->cnt = 0;
    un->status &= ~STATUS_ZERO_MODE;
    un->step_ref = 0;
    pcnt_take_limits(un);
}

/*
 * [spec:nuos:req:emu.esp32p4.rmt-pcnt]
 * Step unit u's counter by delta and run its comparators on the new value.
 */
static void pcnt_step(ESP32P4PcntState *s, unsigned u, int delta)
{
    ESP32P4PcntUnit *un = &s->unit[u];
    int16_t old = un->cnt;
    int16_t cnt = (int16_t)(uint16_t)(old + delta);
    int16_t thres0 = (int16_t)extract32(un->conf1, 0, 16);
    int16_t thres1 = (int16_t)extract32(un->conf1, 16, 16);
    int16_t step = (int16_t)extract32(un->change_conf, 0, 16);
    int16_t step_lim = (int16_t)extract32(un->change_conf, 16, 16);
    bool stepping = s->ctrl & CTRL_DELTA_CHANGE_EN(u);
    bool step_hit = false, step_lim_hit = false;
    uint32_t mode;
    uint32_t lat = 0;

    un->cnt = cnt;
    if (cnt == 0) {
        mode = old > 0 ? ZERO_MODE_POS_ZERO : ZERO_MODE_NEG_ZERO;
    } else {
        mode = cnt < 0 ? ZERO_MODE_NEG : ZERO_MODE_POS;
    }

    if ((un->conf0 & CONF0_THR_THRES1_EN) && cnt == thres1) {
        lat |= STATUS_THRES1_LAT;
    }
    if ((un->conf0 & CONF0_THR_THRES0_EN) && cnt == thres0) {
        lat |= STATUS_THRES0_LAT;
    }
    if ((un->conf0 & CONF0_THR_L_LIM_EN) && cnt <= un->l_lim) {
        lat |= STATUS_L_LIM_LAT;
    }
    if ((un->conf0 & CONF0_THR_H_LIM_EN) && cnt >= un->h_lim) {
        lat |= STATUS_H_LIM_LAT;
    }
    if ((un->conf0 & CONF0_THR_ZERO_EN) && cnt == 0) {
        lat |= STATUS_ZERO_LAT;
    }
    if (stepping) {
        if (step && cnt == (int16_t)(uint16_t)(un->step_ref + step)) {
            step_hit = true;
            un->step_ref = cnt;
        }
        step_lim_hit = cnt == step_lim;
    }

    /* A limit returns the counter to zero from the side it was reached */
    if (lat & STATUS_H_LIM_LAT) {
        un->cnt = 0;
        mode = ZERO_MODE_POS_ZERO;
    } else if (lat & STATUS_L_LIM_LAT) {
        un->cnt = 0;
        mode = ZERO_MODE_NEG_ZERO;
    } else if (step_lim_hit) {
        un->cnt = 0;
        mode = step_lim > 0 ? ZERO_MODE_POS_ZERO : ZERO_MODE_NEG_ZERO;
    }
    if (un->cnt == 0 && cnt != 0) {
        un->step_ref = 0;
    }

    if (lat || step_hit || step_lim_hit) {
        un->status = lat | mode;
        s->int_raw |= 1u << u;
        pcnt_update_irq(s);
    } else {
        un->status = (un->status & ~STATUS_ZERO_MODE) | mode;
    }
    if (lat) {
        pcnt_take_limits(un);
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.rmt-pcnt]
 * A filtered edge on unit u's input k. Only signal inputs count; a
 * control input's level is consulted when its channel's signal moves.
 */
static void pcnt_edge(ESP32P4PcntState *s, unsigned u, unsigned k,
                      bool rising)
{
    uint32_t conf0 = s->unit[u].conf0;
    unsigned ch, edge, lvl;
    int delta;

    if (k != ESP32P4_PCNT_SIG(0) && k != ESP32P4_PCNT_SIG(1)) {
        return;
    }
    if (pcnt_held(s, u) || (s->ctrl & CTRL_CNT_PAUSE(u))) {
        return;
    }
    ch = k;
    edge = extract32(conf0, CONF0_CH_SHIFT(ch) + (rising ? 2 : 0), 2);
    if (s->filt[u * UI + ESP32P4_PCNT_CTRL(ch)]) {
        lvl = extract32(conf0, CONF0_CH_SHIFT(ch) + 4, 2);
    } else {
        lvl = extract32(conf0, CONF0_CH_SHIFT(ch) + 6, 2);
    }

    switch (edge) {
    case EDGE_INC:
        delta = 1;
        break;
    case EDGE_DEC:
        delta = -1;
        break;
    default:
        return;
    }
    switch (lvl) {
    case CTRL_KEEP:
        break;
    case CTRL_INVERT:
        delta = -delta;
        break;
    default:
        return;
    }
    pcnt_step(s, u, delta);
}

/*
 * [spec:nuos:req:emu.esp32p4.rmt-pcnt]
 * Whether filtered input i has a change waiting, and when it matures.
 */
static bool pcnt_pending(ESP32P4PcntState *s, unsigned i, int64_t *due)
{
    uint32_t conf0 = s->unit[i / UI].conf0;
    uint32_t thres = conf0 & CONF0_FILTER_THRES;

    if (!s->clk_running || s->raw[i] == s->filt[i]) {
        return false;
    }
    *due = s->raw_since_ns[i];
    if (conf0 & CONF0_FILTER_EN) {
        *due += esp32p4_clk_cycles_ns(esp32p4_clk_exact(s->apb_clk), thres);
    }
    return true;
}

/* Pass every change that has matured by time t through its filter. */
static void pcnt_sync(ESP32P4PcntState *s, int64_t t)
{
    for (;;) {
        int best = -1;
        int64_t next = INT64_MAX, due;

        for (unsigned i = 0; i < ESP32P4_PCNT_INPUTS; i++) {
            if (i % UI != ESP32P4_PCNT_RST && pcnt_pending(s, i, &due) &&
                due < next) {
                best = i;
                next = due;
            }
        }
        if (best < 0 || next > t) {
            return;
        }
        s->filt[best] = s->raw[best];
        pcnt_edge(s, best / UI, best % UI, s->filt[best]);
    }
}

/* Arm the timer for the next change to mature. */
static void pcnt_schedule(ESP32P4PcntState *s)
{
    int64_t next = INT64_MAX, due;

    for (unsigned i = 0; i < ESP32P4_PCNT_INPUTS; i++) {
        if (i % UI != ESP32P4_PCNT_RST && pcnt_pending(s, i, &due)) {
            next = MIN(next, due);
        }
    }
    if (next == INT64_MAX) {
        timer_del(&s->filter_timer);
    } else {
        timer_mod_ns(&s->filter_timer, next);
    }
}

/* Bring the line sources, then the filters, up to the present. */
static void pcnt_catch_up(ESP32P4PcntState *s)
{
    esp32_line_sync();
    pcnt_sync(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
}

static void pcnt_filter_cb(void *opaque)
{
    ESP32P4PcntState *s = opaque;

    pcnt_catch_up(s);
    pcnt_schedule(s);
}

/* [spec:nuos:req:emu.esp32p4.rmt-pcnt] */
static void pcnt_set_input(void *opaque, int n, int level)
{
    ESP32P4PcntState *s = opaque;
    unsigned u = n / UI;
    int64_t t = esp32_line_time();

    level = !!level;
    if (s->raw[n] == level) {
        return;
    }
    /* Changes already held long enough happened before this one */
    pcnt_sync(s, t);
    s->raw[n] = level;
    s->raw_since_ns[n] = t;
    if (n % UI == ESP32P4_PCNT_RST) {
        if (pcnt_held(s, u)) {
            pcnt_clear(s, u);
        }
        return;
    }
    pcnt_sync(s, t);
    pcnt_schedule(s);
}

static uint64_t esp32p4_pcnt_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4PcntState *s = opaque;
    uint64_t v;

    pcnt_catch_up(s);
    switch (addr) {
    case A_CONF0(0) ... A_CONF2(NU - 1):
        if (addr % 0x0c == 0) {
            v = s->unit[addr / 0x0c].conf0;
        } else if (addr % 0x0c == 4) {
            v = s->unit[addr / 0x0c].conf1;
        } else {
            v = s->unit[addr / 0x0c].conf2;
        }
        break;
    case A_CNT(0) ... A_CNT(NU - 1):
        v = (uint16_t)s->unit[(addr - A_CNT(0)) / 4].cnt;
        break;
    case A_INT_RAW:
        v = s->int_raw;
        break;
    case A_INT_ST:
        v = s->int_raw & s->int_ena;
        break;
    case A_INT_ENA:
        v = s->int_ena;
        break;
    case A_INT_CLR:
        v = 0;
        break;
    case A_STATUS(0) ... A_STATUS(NU - 1):
        v = s->unit[(addr - A_STATUS(0)) / 4].status;
        break;
    case A_CTRL:
        v = s->ctrl;
        break;
    case A_CHANGE_CONF(NU - 1) ... A_CHANGE_CONF(0):
        v = s->unit[(A_CHANGE_CONF(0) - addr) / 4].change_conf;
        break;
    case A_DATE:
        v = s->date;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of reserved offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_PCNT, addr);
        v = 0;
        break;
    }
    pcnt_schedule(s);
    return v;
}

/* [spec:nuos:req:emu.esp32p4.rmt-pcnt] */
static void esp32p4_pcnt_write(void *opaque, hwaddr addr, uint64_t value,
                               unsigned size)
{
    ESP32P4PcntState *s = opaque;
    uint32_t v = value;
    unsigned u;

    /* Edges that matured before the write are counted under the old state */
    pcnt_catch_up(s);
    switch (addr) {
    case A_CONF0(0) ... A_CONF2(NU - 1):
        u = addr / 0x0c;
        if (addr % 0x0c == 0) {
            s->unit[u].conf0 = v;
            /* A new filter setting applies to changes already waiting */
            pcnt_sync(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
        } else if (addr % 0x0c == 4) {
            s->unit[u].conf1 = v;
        } else {
            s->unit[u].conf2 = v;
            if (pcnt_held(s, u)) {
                pcnt_take_limits(&s->unit[u]);
            }
        }
        break;
    case A_INT_RAW:
        s->int_raw &= ~(v & INT_MASK);
        pcnt_update_irq(s);
        break;
    case A_INT_ENA:
        s->int_ena = v & INT_MASK;
        pcnt_update_irq(s);
        break;
    case A_INT_CLR:
        s->int_raw &= ~(v & INT_MASK);
        pcnt_update_irq(s);
        break;
    case A_CTRL:
        s->ctrl = v & CTRL_MASK;
        for (u = 0; u < NU; u++) {
            if (pcnt_held(s, u)) {
                pcnt_clear(s, u);
            }
        }
        break;
    case A_CHANGE_CONF(NU - 1) ... A_CHANGE_CONF(0):
        s->unit[(A_CHANGE_CONF(0) - addr) / 4].change_conf = v;
        break;
    case A_DATE:
        s->date = v;
        break;
    case A_CNT(0) ... A_CNT(NU - 1):
    case A_INT_ST:
    case A_STATUS(0) ... A_STATUS(NU - 1):
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only register "
                      "0x%" HWADDR_PRIx "\n", TYPE_ESP32P4_PCNT, addr);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to reserved offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_PCNT, addr);
        break;
    }
    pcnt_schedule(s);
}

static const MemoryRegionOps esp32p4_pcnt_ops = {
    .read = esp32p4_pcnt_read,
    .write = esp32p4_pcnt_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/*
 * [spec:nuos:req:emu.esp32p4.rmt-pcnt]
 * Reset leaves the counters held by PULSE_CNT_RST. The input wires are
 * not part of the block: their levels survive, and the filters start out
 * agreeing with them, so a reset does not count an edge.
 */
static void esp32p4_pcnt_reset_hold(Object *obj, ResetType type)
{
    ESP32P4PcntState *s = ESP32P4_PCNT(obj);

    for (unsigned u = 0; u < NU; u++) {
        s->unit[u] = (ESP32P4PcntUnit) { .conf0 = CONF0_RESET };
    }
    s->int_raw = 0;
    s->int_ena = 0;
    s->ctrl = CTRL_RESET;
    s->date = DATE_RESET;
    timer_del(&s->filter_timer);
    s->clk_running = clock_is_enabled(s->apb_clk);
    for (unsigned i = 0; i < ESP32P4_PCNT_INPUTS; i++) {
        s->filt[i] = s->raw[i];
    }
}

static void esp32p4_pcnt_reset_exit(Object *obj, ResetType type)
{
    pcnt_update_irq(ESP32P4_PCNT(obj));
}

static void esp32p4_pcnt_clk_update(void *opaque, ClockEvent event)
{
    ESP32P4PcntState *s = opaque;
    bool running = clock_is_enabled(s->apb_clk);
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    /*
     * With the clock stopped the filter's flops hold their state; a change
     * waiting at the wire is filtered afresh once the clock restarts.
     */
    if (running && !s->clk_running) {
        for (unsigned i = 0; i < ESP32P4_PCNT_INPUTS; i++) {
            s->raw_since_ns[i] = now;
        }
    }
    s->clk_running = running;
    pcnt_sync(s, now);
    pcnt_schedule(s);
}

static void esp32p4_pcnt_init(Object *obj)
{
    ESP32P4PcntState *s = ESP32P4_PCNT(obj);

    memory_region_init_io(&s->iomem, obj, &esp32p4_pcnt_ops, s,
                          TYPE_ESP32P4_PCNT, ESP32P4_PCNT_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    qdev_init_gpio_in_named(DEVICE(obj), pcnt_set_input, ESP32P4_PCNT_INPUT,
                            ESP32P4_PCNT_INPUTS);
    s->apb_clk = qdev_init_clock_in(DEVICE(obj), "apb",
                                    esp32p4_pcnt_clk_update, s, ClockUpdate);
    timer_init_ns(&s->filter_timer, QEMU_CLOCK_VIRTUAL, pcnt_filter_cb, s);
}

static const VMStateDescription vmstate_esp32p4_pcnt_unit = {
    .name = "esp32p4-pcnt-unit",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(conf0, ESP32P4PcntUnit),
        VMSTATE_UINT32(conf1, ESP32P4PcntUnit),
        VMSTATE_UINT32(conf2, ESP32P4PcntUnit),
        VMSTATE_UINT32(change_conf, ESP32P4PcntUnit),
        VMSTATE_INT16(cnt, ESP32P4PcntUnit),
        VMSTATE_UINT32(status, ESP32P4PcntUnit),
        VMSTATE_INT16(h_lim, ESP32P4PcntUnit),
        VMSTATE_INT16(l_lim, ESP32P4PcntUnit),
        VMSTATE_INT16(step_ref, ESP32P4PcntUnit),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_esp32p4_pcnt = {
    .name = TYPE_ESP32P4_PCNT,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(unit, ESP32P4PcntState, NU, 1,
                             vmstate_esp32p4_pcnt_unit, ESP32P4PcntUnit),
        VMSTATE_UINT32(int_raw, ESP32P4PcntState),
        VMSTATE_UINT32(int_ena, ESP32P4PcntState),
        VMSTATE_UINT32(ctrl, ESP32P4PcntState),
        VMSTATE_UINT32(date, ESP32P4PcntState),
        VMSTATE_BOOL(clk_running, ESP32P4PcntState),
        VMSTATE_UINT8_ARRAY(raw, ESP32P4PcntState, ESP32P4_PCNT_INPUTS),
        VMSTATE_UINT8_ARRAY(filt, ESP32P4PcntState, ESP32P4_PCNT_INPUTS),
        VMSTATE_INT64_ARRAY(raw_since_ns, ESP32P4PcntState,
                            ESP32P4_PCNT_INPUTS),
        VMSTATE_TIMER(filter_timer, ESP32P4PcntState),
        VMSTATE_CLOCK(apb_clk, ESP32P4PcntState),
        VMSTATE_END_OF_LIST()
    }
};

static void esp32p4_pcnt_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32p4_pcnt_reset_hold;
    rc->phases.exit = esp32p4_pcnt_reset_exit;
    dc->vmsd = &vmstate_esp32p4_pcnt;
}

/* [spec:nuos:req:emu.esp32p4.rmt-pcnt] */
static const TypeInfo esp32p4_pcnt_info = {
    .name = TYPE_ESP32P4_PCNT,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32P4PcntState),
    .instance_init = esp32p4_pcnt_init,
    .class_init = esp32p4_pcnt_class_init,
};

static void esp32p4_pcnt_register_types(void)
{
    type_register_static(&esp32p4_pcnt_info);
}

type_init(esp32p4_pcnt_register_types)
