/*
 * ESP32-P4 system timer (SYSTIMER)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapter
 * 16 (System Timer), and ESP-IDF's systimer_reg.h for v3, which gives the
 * registers' layout, access and reset values; its hal systimer_ll.h and
 * esp_timer show how software drives the block.
 *
 * The block counts CNT_CLK, its function clock (SYSTIMER_CLK: XTAL_CLK or
 * RC_FAST_CLK, HP_SYS_CLKRST_SYSTIMER_CLK_SRC_SEL) divided by 2.5: over
 * every five cycles of the function clock CNT_CLK has two edges, after
 * three cycles and after two more, an average of 16 MHz from the 40 MHz
 * crystal (TRM figure 16.5-1). Two 52-bit counters, UNIT0 and UNIT1, count
 * its edges while SYSTIMER_TIMER_UNITn_WORK_EN is set and no core whose
 * UNITn_COREm_STALL_EN is set is stalled; nothing counts while the
 * function clock is stopped. Software reads a counter by writing
 * UNITn_UPDATE, which latches it into VALUE_HI:VALUE_LO and sets
 * VALUE_VALID, and loads one from LOAD_HI:LOAD_LO by writing UNITn_LOAD.
 * The latch completes at once: the TRM gives no time for the
 * synchronisation, so VALUE_VALID reads as set as soon as software can
 * poll it.
 *
 * Three comparators, COMP0 to COMP2, each compare the counter
 * TARGETx_TIMER_UNIT_SEL selects:
 *
 *   - Target mode: COMPx_LOAD takes the 52-bit alarm value from
 *     TARGETx_HI:TARGETx_LO. The comparator alarms once, when the counter
 *     reaches the value, or at once if the value has passed, the counter
 *     being 0 to 2^51 - 1 counts past it (TRM table 16.5-2); a value
 *     further ahead is reached after the counter wraps.
 *   - Period mode: COMPx_LOAD takes the 26-bit period from TARGETx_PERIOD
 *     and starts the alarms at the counter's value then plus the period,
 *     as setting PERIOD_MODE and enabling the comparator also do (the TRM
 *     has software clear and set PERIOD_MODE, and ESP-IDF enables the
 *     comparator after the load); each alarm moves the alarm value on by
 *     the period. A period of 0 is logged as a guest error and alarms no
 *     more.
 *
 * REAL_TARGETx shows the alarm value in either mode. A comparator compares
 * while TARGETx_WORK_EN is set; enabling it starts its comparison again,
 * with an alarm at once for a target already passed. An alarm sets
 * TARGETx_INT_RAW, a level interrupt that TARGETx_INT_ENA lets out
 * (sources 53 to 55 of the interrupt matrix), and while SYSTIMER_ETM_EN is
 * set it pulses the comparator's ETM event. The comparators need CNT_CLK
 * to run. An alarm happens at the time of the CNT_CLK edge that reaches
 * its value, however late the QEMU timer that runs it: register accesses
 * and clock and stall changes first run the alarms due before them, in
 * order, and the ETM event carries the alarm's time.
 *
 * Writes to TARGETx_HI, TARGETx_LO, TARGETx_PERIOD, LOAD_HI and LOAD_LO
 * take effect at the next COMPx_LOAD or UNITn_LOAD, as the TRM's
 * synchronisation requires; TIMER_UNIT_SEL and PERIOD_MODE take effect at
 * once. SYSTIMER_CLK_EN, the register file's clock gating, has no effect
 * software can see. The registers are clocked by SYSTIMER_APB_CLK; the
 * machine maps them behind it and the block's reset.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/gpio/esp32_line.h"
#include "hw/timer/esp32p4_systimer.h"
#include "migration/vmstate.h"

#define A_CONF              0x00
#define A_UNIT_OP(n)        (0x04 + 4 * (n))
#define A_UNIT_LOAD_HI(n)   (0x0c + 8 * (n))
#define A_UNIT_LOAD_LO(n)   (0x10 + 8 * (n))
#define A_TARGET_HI(x)      (0x1c + 8 * (x))
#define A_TARGET_LO(x)      (0x20 + 8 * (x))
#define A_TARGET_CONF(x)    (0x34 + 4 * (x))
#define A_UNIT_VALUE_HI(n)  (0x40 + 8 * (n))
#define A_UNIT_VALUE_LO(n)  (0x44 + 8 * (n))
#define A_COMP_LOAD(x)      (0x50 + 4 * (x))
#define A_UNIT_LOAD(n)      (0x5c + 4 * (n))
#define A_INT_ENA           0x64
#define A_INT_RAW           0x68
#define A_INT_CLR           0x6c
#define A_INT_ST            0x70
#define A_REAL_TARGET_LO(x) (0x74 + 8 * (x))
#define A_REAL_TARGET_HI(x) (0x78 + 8 * (x))
#define A_DATE              0xfc

#define CONF_ETM_EN         BIT(1)
#define CONF_TARGET_WORK_EN(x)      BIT(24 - (x))
#define CONF_UNIT_STALL_EN(n, core) BIT(28 - 2 * (n) - (core))
#define CONF_UNIT_WORK_EN(n)        BIT(30 - (n))
#define CONF_CLK_EN         BIT(31)
#define CONF_MASK           0xffc00002
#define CONF_RESET          0x46000000

#define OP_VALUE_VALID      BIT(29)
#define OP_UPDATE           BIT(30)

#define TCONF_PERIOD_MASK   0x03ffffff
#define TCONF_PERIOD_MODE   BIT(30)
#define TCONF_UNIT_SEL      BIT(31)
#define TCONF_MASK          0xc3ffffff

#define HI_MASK             0x000fffff
#define INT_MASK            0x7
#define DATE_RESET          0x02201073

#define COUNT_BITS          52
#define COUNT_MASK          MAKE_64BIT_MASK(0, COUNT_BITS)
#define PASSED_RANGE        (1ull << 51)

/* The least time in which @clk makes @n cycles. */
static uint64_t systimer_cycles_ns(Clock *clk, uint64_t n)
{
    uint64_t ns = clock_ticks_to_ns(clk, n);

    while (ns < INT64_MAX && clock_ns_to_ticks(clk, ns) < n) {
        ns++;
    }
    return ns;
}

/* CNT_CLK's edges up to @now. */
static uint64_t systimer_edges(ESP32P4SysTimerState *s, int64_t now)
{
    if (!clock_is_enabled(s->clk) || now <= s->clk_ns) {
        return s->clk_edges;
    }
    return s->clk_edges + clock_ns_to_ticks(s->clk, now - s->clk_ns) * 2 / 5;
}

/* When CNT_CLK edge @e comes, at its current rate; e > clk_edges. */
static int64_t systimer_edge_ns(ESP32P4SysTimerState *s, uint64_t e)
{
    uint64_t cycles = DIV_ROUND_UP((e - s->clk_edges) * 5, 2);
    uint64_t ns = systimer_cycles_ns(s->clk, cycles);

    return MIN(ns, INT64_MAX / 2 - s->clk_ns) + s->clk_ns;
}

static bool systimer_unit_should_run(ESP32P4SysTimerState *s, int n)
{
    int core;

    if (!(s->conf & CONF_UNIT_WORK_EN(n))) {
        return false;
    }
    for (core = 0; core < ESP32P4_SYSTIMER_CORES; core++) {
        if ((s->conf & CONF_UNIT_STALL_EN(n, core)) && s->stall[core]) {
            return false;
        }
    }
    return true;
}

static uint64_t systimer_unit_value(ESP32P4SysTimerState *s, int n,
                                    uint64_t edges)
{
    ESP32P4SysTimerUnit *u = &s->unit[n];

    if (!u->running) {
        return u->value;
    }
    return (u->value + edges - u->sync) & COUNT_MASK;
}

/* Fold the counts up to now into each counter, at CNT_CLK edge @edges. */
static void systimer_sync_units(ESP32P4SysTimerState *s, uint64_t edges)
{
    int n;

    for (n = 0; n < ESP32P4_SYSTIMER_UNITS; n++) {
        s->unit[n].value = systimer_unit_value(s, n, edges);
        s->unit[n].sync = edges;
    }
}

static void systimer_update_irqs(ESP32P4SysTimerState *s)
{
    uint32_t st = s->int_raw & s->int_ena;
    int x;

    for (x = 0; x < ESP32P4_SYSTIMER_COMPS; x++) {
        qemu_set_irq(s->irq[x], extract32(st, x, 1));
    }
}

static int systimer_comp_unit(ESP32P4SysTimerComp *c)
{
    return (c->conf & TCONF_UNIT_SEL) ? 1 : 0;
}

/* [spec:nuos:req:emu.esp32p4.timers] */
static void systimer_alarm(ESP32P4SysTimerState *s, int x, int64_t when)
{
    s->int_raw |= BIT(x);
    systimer_update_irqs(s);
    if (s->conf & CONF_ETM_EN) {
        esp32_line_set_at(s->etm_event[x], 1, when);
        esp32_line_set_at(s->etm_event[x], 0, when);
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.timers]
 * Compare comparator @x at CNT_CLK edge @edges, at time @now: alarm if
 * its value has been reached or passed, and schedule its next alarm
 * otherwise.
 */
static void systimer_comp_check(ESP32P4SysTimerState *s, int x,
                                uint64_t edges, int64_t now)
{
    ESP32P4SysTimerComp *c = &s->comp[x];
    int n = systimer_comp_unit(c);
    uint64_t v, ahead;

    timer_del(&c->timer);
    if (!c->armed || !(s->conf & CONF_TARGET_WORK_EN(x)) ||
        !clock_is_enabled(s->clk)) {
        return;
    }
    v = systimer_unit_value(s, n, edges);
    if (((v - c->real) & COUNT_MASK) < PASSED_RANGE) {
        systimer_alarm(s, x, now);
        if (!(c->conf & TCONF_PERIOD_MODE)) {
            c->armed = false;
            return;
        }
        if (!c->period) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: comparator %d in period "
                          "mode with a period of 0\n",
                          TYPE_ESP32P4_SYSTIMER, x);
            c->armed = false;
            return;
        }
        /* A counter loaded past several periods resumes from its value. */
        c->real = ((v == c->real ? c->real : v) + c->period) & COUNT_MASK;
    }
    if (!s->unit[n].running) {
        return;
    }
    ahead = (c->real - v) & COUNT_MASK;
    c->due_ns = systimer_edge_ns(s, edges + ahead);
    timer_mod(&c->timer, c->due_ns);
}

static void systimer_check_all(ESP32P4SysTimerState *s, uint64_t edges,
                               int64_t now)
{
    int x;

    for (x = 0; x < ESP32P4_SYSTIMER_COMPS; x++) {
        systimer_comp_check(s, x, edges, now);
    }
}

/* Start comparator @x's alarms afresh, as a load or an enable does. */
static void systimer_comp_start(ESP32P4SysTimerState *s, int x,
                                uint64_t edges)
{
    ESP32P4SysTimerComp *c = &s->comp[x];

    if (c->conf & TCONF_PERIOD_MODE) {
        c->real = (systimer_unit_value(s, systimer_comp_unit(c), edges) +
                   c->period) & COUNT_MASK;
    }
    c->armed = true;
}

/*
 * Re-evaluate which counters run, after folding their counts at @edges,
 * and the comparators against them, at time @now.
 */
static void systimer_update(ESP32P4SysTimerState *s, uint64_t edges,
                            int64_t now)
{
    int n;

    systimer_sync_units(s, edges);
    for (n = 0; n < ESP32P4_SYSTIMER_UNITS; n++) {
        s->unit[n].running = systimer_unit_should_run(s, n);
    }
    systimer_check_all(s, edges, now);
}

/*
 * Comparator @x's alarm falls due: compare it at its due time, the edge
 * that reaches its value, which a QEMU timer running late has passed.
 */
static void systimer_comp_due(ESP32P4SysTimerState *s, int x)
{
    ESP32P4SysTimerComp *c = &s->comp[x];

    timer_del(&c->timer);
    systimer_comp_check(s, x, systimer_edges(s, c->due_ns), c->due_ns);
}

/* Run the alarms that fell due by @now, in order. */
static void systimer_catch_up(ESP32P4SysTimerState *s, int64_t now)
{
    for (;;) {
        int next = -1;
        int x;

        for (x = 0; x < ESP32P4_SYSTIMER_COMPS; x++) {
            ESP32P4SysTimerComp *c = &s->comp[x];

            if (timer_pending(&c->timer) && c->due_ns <= now &&
                (next < 0 || c->due_ns < s->comp[next].due_ns)) {
                next = x;
            }
        }
        if (next < 0) {
            return;
        }
        systimer_comp_due(s, next);
    }
}

static void systimer_comp_cb(void *opaque)
{
    ESP32P4SysTimerComp *c = opaque;

    systimer_comp_due(c->s, c - c->s->comp);
}

/*
 * [spec:nuos:req:emu.esp32p4.timers]
 * The function clock changes rate, starts or stops: count CNT_CLK up to
 * the change at the old rate, then from the change at the new one. A
 * fraction of a CNT_CLK cycle under way at the change is lost.
 */
static void systimer_clk_update(void *opaque, ClockEvent event)
{
    ESP32P4SysTimerState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint64_t edges;

    if (event == ClockPreUpdate) {
        systimer_catch_up(s, now);
        edges = systimer_edges(s, now);
        systimer_sync_units(s, edges);
        s->clk_edges = edges;
        s->clk_ns = now;
        return;
    }
    s->clk_ns = now;
    systimer_update(s, s->clk_edges, now);
}

/* [spec:nuos:req:emu.esp32p4.timers] */
static void systimer_set_stall(void *opaque, int core, int level)
{
    ESP32P4SysTimerState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint64_t edges;

    systimer_catch_up(s, now);
    edges = systimer_edges(s, now);
    systimer_sync_units(s, edges);
    s->stall[core] = level;
    systimer_update(s, edges, now);
}

/* The register at word offset @addr; false for a reserved offset. */
static bool systimer_read_reg(ESP32P4SysTimerState *s, hwaddr addr,
                              uint32_t *v)
{
    switch (addr) {
    case A_CONF:
        *v = s->conf;
        return true;
    case A_UNIT_OP(0):
    case A_UNIT_OP(1):
        *v = s->unit[(addr - A_UNIT_OP(0)) / 4].valid ? OP_VALUE_VALID : 0;
        return true;
    case A_UNIT_LOAD_HI(0):
    case A_UNIT_LOAD_HI(1):
        *v = s->unit[(addr - A_UNIT_LOAD_HI(0)) / 8].load >> 32;
        return true;
    case A_UNIT_LOAD_LO(0):
    case A_UNIT_LOAD_LO(1):
        *v = (uint32_t)s->unit[(addr - A_UNIT_LOAD_LO(0)) / 8].load;
        return true;
    case A_TARGET_HI(0):
    case A_TARGET_HI(1):
    case A_TARGET_HI(2):
        *v = s->comp[(addr - A_TARGET_HI(0)) / 8].target >> 32;
        return true;
    case A_TARGET_LO(0):
    case A_TARGET_LO(1):
    case A_TARGET_LO(2):
        *v = (uint32_t)s->comp[(addr - A_TARGET_LO(0)) / 8].target;
        return true;
    case A_TARGET_CONF(0):
    case A_TARGET_CONF(1):
    case A_TARGET_CONF(2):
        *v = s->comp[(addr - A_TARGET_CONF(0)) / 4].conf;
        return true;
    case A_UNIT_VALUE_HI(0):
    case A_UNIT_VALUE_HI(1):
        *v = s->unit[(addr - A_UNIT_VALUE_HI(0)) / 8].latched >> 32;
        return true;
    case A_UNIT_VALUE_LO(0):
    case A_UNIT_VALUE_LO(1):
        *v = (uint32_t)s->unit[(addr - A_UNIT_VALUE_LO(0)) / 8].latched;
        return true;
    case A_COMP_LOAD(0):
    case A_COMP_LOAD(1):
    case A_COMP_LOAD(2):
    case A_UNIT_LOAD(0):
    case A_UNIT_LOAD(1):
    case A_INT_CLR:
        *v = 0;
        return true;
    case A_INT_ENA:
        *v = s->int_ena;
        return true;
    case A_INT_RAW:
        *v = s->int_raw;
        return true;
    case A_INT_ST:
        *v = s->int_raw & s->int_ena;
        return true;
    case A_REAL_TARGET_LO(0):
    case A_REAL_TARGET_LO(1):
    case A_REAL_TARGET_LO(2):
        *v = (uint32_t)s->comp[(addr - A_REAL_TARGET_LO(0)) / 8].real;
        return true;
    case A_REAL_TARGET_HI(0):
    case A_REAL_TARGET_HI(1):
    case A_REAL_TARGET_HI(2):
        *v = s->comp[(addr - A_REAL_TARGET_HI(0)) / 8].real >> 32;
        return true;
    case A_DATE:
        *v = s->date;
        return true;
    default:
        *v = 0;
        return false;
    }
}

static uint64_t systimer_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4SysTimerState *s = opaque;
    uint32_t v;

    systimer_catch_up(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    if (!systimer_read_reg(s, addr & ~3, &v)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of reserved offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_SYSTIMER, addr);
    }
    return extract32(v, (addr & 3) * 8, size * 8);
}

/* [spec:nuos:req:emu.esp32p4.timers] */
static void systimer_write_conf(ESP32P4SysTimerState *s, uint32_t value,
                                uint64_t edges, int64_t now)
{
    uint32_t old = s->conf;
    int x;

    systimer_sync_units(s, edges);
    s->conf = value & CONF_MASK;
    for (x = 0; x < ESP32P4_SYSTIMER_COMPS; x++) {
        if (s->conf & ~old & CONF_TARGET_WORK_EN(x)) {
            systimer_comp_start(s, x, edges);
        }
    }
    systimer_update(s, edges, now);
}

/*
 * [spec:nuos:req:emu.esp32p4.timers]
 * Write the register at word offset @addr: @value is its new value, the
 * written byte lanes merged into the old, for its R/W fields, and the bits
 * written as 1 for its write-trigger and write-1-to-clear ones.
 */
static void systimer_write_reg(ESP32P4SysTimerState *s, hwaddr addr,
                               uint32_t value, uint32_t ones, int64_t now)
{
    uint64_t edges = systimer_edges(s, now);
    ESP32P4SysTimerUnit *u;
    ESP32P4SysTimerComp *c;
    uint32_t old;
    int i;

    switch (addr) {
    case A_CONF:
        systimer_write_conf(s, value, edges, now);
        break;
    case A_UNIT_OP(0):
    case A_UNIT_OP(1):
        u = &s->unit[(addr - A_UNIT_OP(0)) / 4];
        if (ones & OP_VALUE_VALID) {
            u->valid = false;
        }
        if (ones & OP_UPDATE) {
            u->latched = systimer_unit_value(s, u - s->unit, edges);
            u->valid = true;
        }
        break;
    case A_UNIT_LOAD_HI(0):
    case A_UNIT_LOAD_HI(1):
        u = &s->unit[(addr - A_UNIT_LOAD_HI(0)) / 8];
        u->load = deposit64(u->load, 32, 32, value & HI_MASK);
        break;
    case A_UNIT_LOAD_LO(0):
    case A_UNIT_LOAD_LO(1):
        u = &s->unit[(addr - A_UNIT_LOAD_LO(0)) / 8];
        u->load = deposit64(u->load, 0, 32, value);
        break;
    case A_TARGET_HI(0):
    case A_TARGET_HI(1):
    case A_TARGET_HI(2):
        c = &s->comp[(addr - A_TARGET_HI(0)) / 8];
        c->target = deposit64(c->target, 32, 32, value & HI_MASK);
        break;
    case A_TARGET_LO(0):
    case A_TARGET_LO(1):
    case A_TARGET_LO(2):
        c = &s->comp[(addr - A_TARGET_LO(0)) / 8];
        c->target = deposit64(c->target, 0, 32, value);
        break;
    case A_TARGET_CONF(0):
    case A_TARGET_CONF(1):
    case A_TARGET_CONF(2):
        i = (addr - A_TARGET_CONF(0)) / 4;
        c = &s->comp[i];
        old = c->conf;
        c->conf = value & TCONF_MASK;
        if (c->conf & ~old & TCONF_PERIOD_MODE) {
            systimer_comp_start(s, i, edges);
        }
        systimer_comp_check(s, i, edges, now);
        break;
    case A_COMP_LOAD(0):
    case A_COMP_LOAD(1):
    case A_COMP_LOAD(2):
        if (ones & 1) {
            i = (addr - A_COMP_LOAD(0)) / 4;
            c = &s->comp[i];
            c->period = c->conf & TCONF_PERIOD_MASK;
            c->real = c->target;
            systimer_comp_start(s, i, edges);
            systimer_comp_check(s, i, edges, now);
        }
        break;
    case A_UNIT_LOAD(0):
    case A_UNIT_LOAD(1):
        if (ones & 1) {
            u = &s->unit[(addr - A_UNIT_LOAD(0)) / 4];
            u->value = u->load;
            u->sync = edges;
            systimer_check_all(s, edges, now);
        }
        break;
    case A_INT_ENA:
        s->int_ena = value & INT_MASK;
        systimer_update_irqs(s);
        break;
    case A_INT_RAW:
    case A_INT_CLR:
        s->int_raw &= ~(ones & INT_MASK);
        systimer_update_irqs(s);
        break;
    case A_DATE:
        s->date = value;
        break;
    case A_UNIT_VALUE_HI(0):
    case A_UNIT_VALUE_HI(1):
    case A_UNIT_VALUE_LO(0):
    case A_UNIT_VALUE_LO(1):
    case A_INT_ST:
    case A_REAL_TARGET_LO(0) ... A_REAL_TARGET_HI(2):
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only register at "
                      "0x%" HWADDR_PRIx "\n", TYPE_ESP32P4_SYSTIMER, addr);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to reserved offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_SYSTIMER, addr);
        break;
    }
}

static void systimer_write(void *opaque, hwaddr addr, uint64_t data,
                           unsigned size)
{
    ESP32P4SysTimerState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    unsigned shift = (addr & 3) * 8;
    uint32_t lanes = MAKE_64BIT_MASK(shift, size * 8);
    uint32_t ones = (data << shift) & lanes;
    uint32_t old;

    systimer_catch_up(s, now);
    systimer_read_reg(s, addr & ~3, &old);
    systimer_write_reg(s, addr & ~3, (old & ~lanes) | ones, ones, now);
}

static const MemoryRegionOps systimer_ops = {
    .read = systimer_read,
    .write = systimer_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

/* [spec:nuos:req:emu.esp32p4.timers] */
static void esp32p4_systimer_reset_hold(Object *obj, ResetType type)
{
    ESP32P4SysTimerState *s = ESP32P4_SYSTIMER(obj);
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint64_t edges = systimer_edges(s, now);
    int i;

    s->conf = CONF_RESET;
    s->int_ena = 0;
    s->int_raw = 0;
    s->date = DATE_RESET;
    for (i = 0; i < ESP32P4_SYSTIMER_UNITS; i++) {
        s->unit[i] = (ESP32P4SysTimerUnit) { .sync = edges };
    }
    for (i = 0; i < ESP32P4_SYSTIMER_COMPS; i++) {
        ESP32P4SysTimerComp *c = &s->comp[i];

        timer_del(&c->timer);
        c->target = 0;
        c->conf = 0;
        c->period = 0;
        c->real = 0;
        c->armed = false;
    }
    for (i = 0; i < ESP32P4_SYSTIMER_UNITS; i++) {
        s->unit[i].running = systimer_unit_should_run(s, i);
    }
}

static void esp32p4_systimer_reset_exit(Object *obj, ResetType type)
{
    systimer_update_irqs(ESP32P4_SYSTIMER(obj));
}

static void esp32p4_systimer_init(Object *obj)
{
    ESP32P4SysTimerState *s = ESP32P4_SYSTIMER(obj);
    DeviceState *dev = DEVICE(obj);
    int i;

    memory_region_init_io(&s->iomem, obj, &systimer_ops, s,
                          "esp32p4.systimer", ESP32P4_SYSTIMER_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    s->clk = qdev_init_clock_in(dev, "clk", systimer_clk_update, s,
                                ClockPreUpdate | ClockUpdate);
    for (i = 0; i < ESP32P4_SYSTIMER_COMPS; i++) {
        sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq[i]);
        s->comp[i].s = s;
        timer_init_ns(&s->comp[i].timer, QEMU_CLOCK_VIRTUAL,
                      systimer_comp_cb, &s->comp[i]);
    }
    qdev_init_gpio_in_named(dev, systimer_set_stall,
                            ESP32P4_SYSTIMER_CORE_STALL,
                            ESP32P4_SYSTIMER_CORES);
    qdev_init_gpio_out_named(dev, s->etm_event, ESP32P4_SYSTIMER_ETM_EVENT,
                             ESP32P4_SYSTIMER_COMPS);
}

static const VMStateDescription vmstate_esp32p4_systimer_unit = {
    .name = TYPE_ESP32P4_SYSTIMER "/unit",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT64(value, ESP32P4SysTimerUnit),
        VMSTATE_UINT64(sync, ESP32P4SysTimerUnit),
        VMSTATE_BOOL(running, ESP32P4SysTimerUnit),
        VMSTATE_UINT64(load, ESP32P4SysTimerUnit),
        VMSTATE_UINT64(latched, ESP32P4SysTimerUnit),
        VMSTATE_BOOL(valid, ESP32P4SysTimerUnit),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_esp32p4_systimer_comp = {
    .name = TYPE_ESP32P4_SYSTIMER "/comp",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT64(target, ESP32P4SysTimerComp),
        VMSTATE_UINT32(conf, ESP32P4SysTimerComp),
        VMSTATE_UINT32(period, ESP32P4SysTimerComp),
        VMSTATE_UINT64(real, ESP32P4SysTimerComp),
        VMSTATE_BOOL(armed, ESP32P4SysTimerComp),
        VMSTATE_INT64(due_ns, ESP32P4SysTimerComp),
        VMSTATE_TIMER(timer, ESP32P4SysTimerComp),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_esp32p4_systimer = {
    .name = TYPE_ESP32P4_SYSTIMER,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_CLOCK(clk, ESP32P4SysTimerState),
        VMSTATE_UINT32(conf, ESP32P4SysTimerState),
        VMSTATE_UINT32(int_ena, ESP32P4SysTimerState),
        VMSTATE_UINT32(int_raw, ESP32P4SysTimerState),
        VMSTATE_UINT32(date, ESP32P4SysTimerState),
        VMSTATE_STRUCT_ARRAY(unit, ESP32P4SysTimerState,
                             ESP32P4_SYSTIMER_UNITS, 1,
                             vmstate_esp32p4_systimer_unit,
                             ESP32P4SysTimerUnit),
        VMSTATE_STRUCT_ARRAY(comp, ESP32P4SysTimerState,
                             ESP32P4_SYSTIMER_COMPS, 1,
                             vmstate_esp32p4_systimer_comp,
                             ESP32P4SysTimerComp),
        VMSTATE_BOOL_ARRAY(stall, ESP32P4SysTimerState,
                           ESP32P4_SYSTIMER_CORES),
        VMSTATE_UINT64(clk_edges, ESP32P4SysTimerState),
        VMSTATE_INT64(clk_ns, ESP32P4SysTimerState),
        VMSTATE_END_OF_LIST()
    },
};

static void esp32p4_systimer_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->vmsd = &vmstate_esp32p4_systimer;
    rc->phases.hold = esp32p4_systimer_reset_hold;
    rc->phases.exit = esp32p4_systimer_reset_exit;
}

/* [spec:nuos:req:emu.esp32p4.timers] */
static const TypeInfo esp32p4_systimer_info = {
    .name          = TYPE_ESP32P4_SYSTIMER,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32P4SysTimerState),
    .instance_init = esp32p4_systimer_init,
    .class_init    = esp32p4_systimer_class_init,
};

static void esp32p4_systimer_register_types(void)
{
    type_register_static(&esp32p4_systimer_info);
}
type_init(esp32p4_systimer_register_types)
