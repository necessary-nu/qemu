/*
 * ESP32 pulse count controller (PCNT)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32 TRM, "Pulse Count Controller (PCNT)", with the field
 * layout and reset values of ESP-IDF's soc/esp32 pcnt_reg.h and
 * pcnt_struct.h.
 *
 * Eight units each keep a 16-bit signed counter driven by two channels.
 * A channel counts the edges of its signal input, up, down or not at all
 * per edge direction (POS_MODE, NEG_MODE), with the action kept, inverted
 * or inhibited by the level of its control input (HCTRL_MODE,
 * LCTRL_MODE). All four inputs of a unit pass through the unit's glitch
 * filter, clocked by APB_CLK.
 *
 * Five comparators watch each counter as it counts: the two thresholds
 * and zero match on equality, the high limit on count >= H_LIM and the
 * low limit on count <= L_LIM, and a limit match also returns the counter
 * to 0. Comparators are evaluated on a step of the counter only, not when
 * software resets it or changes a comparator's value. A step that matches
 * any enabled comparator raises the unit's interrupt and latches which
 * comparators matched in the unit's STATUS register, replacing the
 * previous latch.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/misc/esp32_pcnt.h"
#include "migration/vmstate.h"

#define A_UNIT_CONF0(u)         (0x00 + 0x0c * (u))
#define A_UNIT_CNT(u)           (0x60 + 0x04 * (u))
#define A_INT_RAW               0x80
#define A_INT_ST                0x84
#define A_INT_ENA               0x88
#define A_INT_CLR               0x8c
#define A_UNIT_STATUS(u)        (0x90 + 0x04 * (u))
#define A_CTRL                  0xb0
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
#define CTRL_CLK_EN             (1u << 16)
#define CTRL_MASK               0x1ffffu
#define CTRL_RESET              0x5555u

#define INT_MASK                0xffu
#define DATE_RESET              0x14122600u

/* Inputs within a unit */
#define IN_SIG(ch)              (ch)
#define IN_CTRL(ch)             (2 + (ch))

/* Edge modes and control modes */
#define EDGE_INC                1
#define EDGE_DEC                2
#define CTRL_KEEP               0
#define CTRL_INVERT             1

static void pcnt_update_irq(Esp32PcntState *s)
{
    qemu_set_irq(s->irq, (s->int_raw & s->int_ena) != 0);
}

/*
 * [spec:nuos:req:emu.esp32.pcnt]
 * Step unit u's counter by delta and run its comparators on the new value.
 */
static void pcnt_step(Esp32PcntState *s, unsigned u, int delta)
{
    Esp32PcntUnit *un = &s->unit[u];
    int16_t old = un->cnt;
    int16_t cnt = (int16_t)(uint16_t)(old + delta);
    int16_t thres0 = (int16_t)extract32(un->conf1, 0, 16);
    int16_t thres1 = (int16_t)extract32(un->conf1, 16, 16);
    int16_t h_lim = (int16_t)extract32(un->conf2, 0, 16);
    int16_t l_lim = (int16_t)extract32(un->conf2, 16, 16);
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
    if ((un->conf0 & CONF0_THR_L_LIM_EN) && cnt <= l_lim) {
        lat |= STATUS_L_LIM_LAT;
    }
    if ((un->conf0 & CONF0_THR_H_LIM_EN) && cnt >= h_lim) {
        lat |= STATUS_H_LIM_LAT;
    }
    if ((un->conf0 & CONF0_THR_ZERO_EN) && cnt == 0) {
        lat |= STATUS_ZERO_LAT;
    }

    /* A limit returns the counter to zero from the side it was reached */
    if (lat & STATUS_H_LIM_LAT) {
        un->cnt = 0;
        mode = ZERO_MODE_POS_ZERO;
    } else if (lat & STATUS_L_LIM_LAT) {
        un->cnt = 0;
        mode = ZERO_MODE_NEG_ZERO;
    }

    if (lat) {
        un->status = lat | mode;
        s->int_raw |= 1u << u;
        pcnt_update_irq(s);
    } else {
        un->status = (un->status & ~STATUS_ZERO_MODE) | mode;
    }
}

/*
 * [spec:nuos:req:emu.esp32.pcnt]
 * A filtered edge on unit u's input k. Only signal inputs count; a
 * control input's level is consulted when its channel's signal moves.
 */
static void pcnt_edge(Esp32PcntState *s, unsigned u, unsigned k, bool rising)
{
    uint32_t conf0 = s->unit[u].conf0;
    unsigned ch, edge, lvl;
    int delta;

    if (k >= IN_CTRL(0)) {
        return;
    }
    if (s->ctrl & (CTRL_CNT_RST(u) | CTRL_CNT_PAUSE(u))) {
        return;
    }
    ch = k;
    edge = extract32(conf0, CONF0_CH_SHIFT(ch) + (rising ? 2 : 0), 2);
    if (s->filt[u * ESP32_PCNT_UNIT_INPUTS + IN_CTRL(ch)]) {
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
 * [spec:nuos:req:emu.esp32.pcnt]
 * The glitch filter: with FILTER_EN, a level change on an input reaches
 * the counter once it has held for FILTER_THRES APB_CLK cycles; a pulse
 * shorter than that is lost. Whether a change got through is decided from
 * the virtual time stamps of the wire's changes, not from when the
 * filter's timer happens to run, so a late timer cannot lose an edge or
 * reorder edges between inputs. Returns whether input i has a change
 * waiting, and when it matures.
 */
static bool pcnt_pending(Esp32PcntState *s, unsigned i, int64_t *due)
{
    uint32_t conf0 = s->unit[i / ESP32_PCNT_UNIT_INPUTS].conf0;
    uint32_t thres = conf0 & CONF0_FILTER_THRES;

    if (!s->clk_running || s->raw[i] == s->filt[i]) {
        return false;
    }
    *due = s->raw_since_ns[i];
    if (conf0 & CONF0_FILTER_EN) {
        *due += clock_ticks_to_ns(s->apb_clk, thres);
    }
    return true;
}

/*
 * Pass every change that has matured by now through its filter, oldest
 * first, and arm the timer for the next one.
 */
static void pcnt_sync(Esp32PcntState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int64_t due, next;
    int best;

    for (;;) {
        best = -1;
        next = INT64_MAX;
        for (unsigned i = 0; i < ESP32_PCNT_INPUT_COUNT; i++) {
            if (pcnt_pending(s, i, &due) && due < next) {
                best = i;
                next = due;
            }
        }
        if (best < 0 || next > now) {
            break;
        }
        s->filt[best] = s->raw[best];
        pcnt_edge(s, best / ESP32_PCNT_UNIT_INPUTS,
                  best % ESP32_PCNT_UNIT_INPUTS, s->filt[best]);
    }
    if (best < 0) {
        timer_del(&s->filter_timer);
    } else {
        timer_mod_ns(&s->filter_timer, next);
    }
}

static void pcnt_filter_cb(void *opaque)
{
    pcnt_sync(opaque);
}

static void pcnt_set_input(void *opaque, int n, int level)
{
    Esp32PcntState *s = opaque;

    level = !!level;
    if (s->raw[n] == level) {
        return;
    }
    /* A change already held long enough happened before this one */
    pcnt_sync(s);
    s->raw[n] = level;
    s->raw_since_ns[n] = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    pcnt_sync(s);
}

static uint64_t esp32_pcnt_read(void *opaque, hwaddr addr, unsigned size)
{
    Esp32PcntState *s = opaque;

    pcnt_sync(s);

    if (addr < A_UNIT_CNT(0)) {
        Esp32PcntUnit *un = &s->unit[addr / 0x0c];

        switch (addr % 0x0c) {
        case 0x0:
            return un->conf0;
        case 0x4:
            return un->conf1;
        default:
            return un->conf2;
        }
    }
    if (addr < A_INT_RAW) {
        return (uint16_t)s->unit[(addr - A_UNIT_CNT(0)) / 4].cnt;
    }
    if (addr >= A_UNIT_STATUS(0) && addr < A_CTRL) {
        return s->unit[(addr - A_UNIT_STATUS(0)) / 4].status;
    }

    switch (addr) {
    case A_INT_RAW:
        return s->int_raw;
    case A_INT_ST:
        return s->int_raw & s->int_ena;
    case A_INT_ENA:
        return s->int_ena;
    case A_INT_CLR:
        return 0;
    case A_CTRL:
        return s->ctrl;
    case A_DATE:
        return s->date;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_pcnt: read of reserved offset 0x%" HWADDR_PRIx
                      "\n", addr);
        return 0;
    }
}

static void esp32_pcnt_write(void *opaque, hwaddr addr, uint64_t value,
                             unsigned size)
{
    Esp32PcntState *s = opaque;

    /* Edges that matured before the write are counted under the old state */
    pcnt_sync(s);
    if (addr < A_UNIT_CNT(0)) {
        unsigned u = addr / 0x0c;
        Esp32PcntUnit *un = &s->unit[u];

        switch (addr % 0x0c) {
        case 0x0:
            un->conf0 = value;
            /* A new filter setting applies to changes already waiting */
            pcnt_sync(s);
            break;
        case 0x4:
            un->conf1 = value;
            break;
        default:
            un->conf2 = value;
            break;
        }
        return;
    }

    switch (addr) {
    case A_INT_ENA:
        s->int_ena = value & INT_MASK;
        pcnt_update_irq(s);
        return;
    case A_INT_CLR:
        s->int_raw &= ~(value & INT_MASK);
        pcnt_update_irq(s);
        return;
    case A_CTRL:
        /* A unit's counter is held at zero while its CNT_RST is set */
        s->ctrl = value & CTRL_MASK;
        for (unsigned u = 0; u < ESP32_PCNT_UNIT_COUNT; u++) {
            if (s->ctrl & CTRL_CNT_RST(u)) {
                s->unit[u].cnt = 0;
                s->unit[u].status &= ~STATUS_ZERO_MODE;
            }
        }
        return;
    case A_DATE:
        s->date = value;
        return;
    default:
        break;
    }

    if (addr < A_INT_RAW || addr == A_INT_RAW || addr == A_INT_ST ||
        (addr >= A_UNIT_STATUS(0) && addr < A_CTRL)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_pcnt: write to read-only register 0x%"
                      HWADDR_PRIx "\n", addr);
    } else {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_pcnt: write to reserved offset 0x%" HWADDR_PRIx
                      "\n", addr);
    }
}

static const MemoryRegionOps esp32_pcnt_ops = {
    .read = esp32_pcnt_read,
    .write = esp32_pcnt_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/*
 * [spec:nuos:req:emu.esp32.pcnt]
 * Reset leaves the counters held by CNT_RST. The input wires are not part
 * of the block: their levels survive, and the filters start out agreeing
 * with them, so a reset does not count an edge.
 */
static void esp32_pcnt_reset_hold(Object *obj, ResetType type)
{
    Esp32PcntState *s = ESP32_PCNT(obj);

    for (unsigned u = 0; u < ESP32_PCNT_UNIT_COUNT; u++) {
        s->unit[u] = (Esp32PcntUnit) { .conf0 = CONF0_RESET };
    }
    s->int_raw = 0;
    s->int_ena = 0;
    s->ctrl = CTRL_RESET;
    s->date = DATE_RESET;
    timer_del(&s->filter_timer);
    s->clk_running = clock_is_enabled(s->apb_clk);
    for (unsigned i = 0; i < ESP32_PCNT_INPUT_COUNT; i++) {
        s->filt[i] = s->raw[i];
    }
    pcnt_update_irq(s);
}

static void esp32_pcnt_clk_update(void *opaque, ClockEvent event)
{
    Esp32PcntState *s = opaque;
    bool running = clock_is_enabled(s->apb_clk);
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    /*
     * With the clock stopped the filter's flops hold their state; a change
     * waiting at the wire is filtered afresh once the clock restarts.
     */
    if (running && !s->clk_running) {
        for (unsigned i = 0; i < ESP32_PCNT_INPUT_COUNT; i++) {
            s->raw_since_ns[i] = now;
        }
    }
    s->clk_running = running;
    pcnt_sync(s);
}

static void esp32_pcnt_init(Object *obj)
{
    Esp32PcntState *s = ESP32_PCNT(obj);

    memory_region_init_io(&s->iomem, obj, &esp32_pcnt_ops, s,
                          TYPE_ESP32_PCNT, ESP32_PCNT_REG_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    qdev_init_gpio_in_named(DEVICE(obj), pcnt_set_input, ESP32_PCNT_INPUT,
                            ESP32_PCNT_INPUT_COUNT);
    s->apb_clk = qdev_init_clock_in(DEVICE(obj), "apb", esp32_pcnt_clk_update,
                                    s, ClockUpdate);
    timer_init_ns(&s->filter_timer, QEMU_CLOCK_VIRTUAL, pcnt_filter_cb, s);
}

static const VMStateDescription vmstate_esp32_pcnt_unit = {
    .name = "esp32-pcnt-unit",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(conf0, Esp32PcntUnit),
        VMSTATE_UINT32(conf1, Esp32PcntUnit),
        VMSTATE_UINT32(conf2, Esp32PcntUnit),
        VMSTATE_INT16(cnt, Esp32PcntUnit),
        VMSTATE_UINT32(status, Esp32PcntUnit),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_esp32_pcnt = {
    .name = TYPE_ESP32_PCNT,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(unit, Esp32PcntState, ESP32_PCNT_UNIT_COUNT, 1,
                             vmstate_esp32_pcnt_unit, Esp32PcntUnit),
        VMSTATE_UINT32(int_raw, Esp32PcntState),
        VMSTATE_UINT32(int_ena, Esp32PcntState),
        VMSTATE_UINT32(ctrl, Esp32PcntState),
        VMSTATE_UINT32(date, Esp32PcntState),
        VMSTATE_UINT8_ARRAY(raw, Esp32PcntState, ESP32_PCNT_INPUT_COUNT),
        VMSTATE_UINT8_ARRAY(filt, Esp32PcntState, ESP32_PCNT_INPUT_COUNT),
        VMSTATE_INT64_ARRAY(raw_since_ns, Esp32PcntState,
                            ESP32_PCNT_INPUT_COUNT),
        VMSTATE_TIMER(filter_timer, Esp32PcntState),
        VMSTATE_BOOL(clk_running, Esp32PcntState),
        VMSTATE_CLOCK(apb_clk, Esp32PcntState),
        VMSTATE_END_OF_LIST()
    }
};

static void esp32_pcnt_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32_pcnt_reset_hold;
    dc->vmsd = &vmstate_esp32_pcnt;
}

/* [spec:nuos:req:emu.esp32.pcnt] */
static const TypeInfo esp32_pcnt_info = {
    .name = TYPE_ESP32_PCNT,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32PcntState),
    .instance_init = esp32_pcnt_init,
    .class_init = esp32_pcnt_class_init,
};

static void esp32_pcnt_register_types(void)
{
    type_register_static(&esp32_pcnt_info);
}

type_init(esp32_pcnt_register_types)
