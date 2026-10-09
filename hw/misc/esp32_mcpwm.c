/*
 * ESP32 motor control PWM (MCPWM)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32 TRM, "Motor Control PWM (MCPWM)", and ESP-IDF's
 * soc/mcpwm_reg.h, soc/mcpwm_struct.h and hal/mcpwm_ll.h.
 *
 * PWM_clk is PLL_F160M_CLK divided by CLK_PRESCALE + 1; each PWM timer
 * counts on PT_clk, PWM_clk divided by its own prescale. The three timers
 * count up (0..P), down (P..0) or up-down (0..P-1 up, P..1 down), and
 * signal TEZ and TEP when they reach 0 and the period P. Each of the three
 * operators follows a timer and signals TEA and TEB when the timer reaches
 * its compare values; its generators turn these events, the T0/T1 events
 * (fault or sync) and software forces into the levels of PWMxA and PWMxB,
 * which then pass through the dead-time generator, the carrier chopper and
 * the fault handler. Period, compare values, actions and dead times are
 * double-buffered, the active copies taking the shadows at the selected
 * events.
 *
 * Nothing is ticked. The timers and operators (Esp32McpwmCore) hold their
 * state as of a PWM_clk cycle and are run forward, from one event to the
 * next, to the current cycle whenever they are observed or changed. The
 * counters move monotonically between events, so a run costs a handful of
 * steps per PWM period, and a run whose state repeats exactly (the usual
 * case for a free-running configuration) skips whole repetitions at once.
 * A timer fires only at the moments something becomes visible outside the
 * block: an output edge or a rising interrupt line, found by running a copy
 * of the state ahead. Output edges closer together than the min-edge-ns
 * property are coalesced: the outputs then show their true level every
 * min-edge-ns. Interrupts are never coalesced.
 *
 * The capture timer counts APB_CLK and is computed in closed form; capture
 * inputs, fault inputs and GPIO sync inputs act at the moment their level
 * changes. The input synchronisers in the GPIO matrix and the fault and
 * capture sampling are not modelled: an input takes effect in the PWM_clk
 * or APB_CLK cycle in which it changes.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/esp32_mcpwm.h"
#include "migration/vmstate.h"

#define A_CLK_CFG               0x000
#define A_TIMER_BASE            0x004
#define TIMER_STRIDE            0x10
#define TIMER_CFG0              0x0
#define TIMER_CFG1              0x4
#define TIMER_SYNC              0x8
#define TIMER_STATUS            0xc
#define A_SYNCI_CFG             0x034
#define A_TIMERSEL              0x038
#define A_OP_BASE               0x03c
#define OP_STRIDE               0x38
#define OP_STMP_CFG             0x00
#define OP_TSTMP_A              0x04
#define OP_TSTMP_B              0x08
#define OP_GEN_CFG0             0x0c
#define OP_FORCE                0x10
#define OP_GEN_A                0x14
#define OP_GEN_B                0x18
#define OP_DT_CFG               0x1c
#define OP_DT_FED               0x20
#define OP_DT_RED               0x24
#define OP_CARRIER              0x28
#define OP_FH_CFG0              0x2c
#define OP_FH_CFG1              0x30
#define OP_FH_STATUS            0x34
#define A_FAULT_DETECT          0x0e4
#define A_CAP_TIMER_CFG         0x0e8
#define A_CAP_TIMER_PHASE       0x0ec
#define A_CAP_CH_CFG            0x0f0
#define A_CAP_CH                0x0fc
#define A_CAP_STATUS            0x108
#define A_UPDATE_CFG            0x10c
#define A_INT_ENA               0x110
#define A_INT_RAW               0x114
#define A_INT_ST                0x118
#define A_INT_CLR               0x11c
#define A_CLK                   0x120
#define A_VERSION               0x124

#define ESP32_MCPWM_REGION_SIZE 0x1000

#define CFG0_MASK               0x03ffffffu
#define CFG0_RESET              0x0000ff00u
#define CFG0_PRESCALE(v)        ((v) & 0xff)
#define CFG0_PERIOD(v)          (((v) >> 8) & 0xffff)
#define CFG0_UPMETHOD(v)        (((v) >> 24) & 3)
#define CFG1_MASK               0x1fu
#define CFG1_START(v)           ((v) & 7)
#define CFG1_MOD(v)             (((v) >> 3) & 3)
#define SYNC_MASK               0x1fffffu
#define SYNC_SYNCI_EN           (1u << 0)
#define SYNC_SW                 (1u << 1)
#define SYNC_SYNCO_SEL(v)       (((v) >> 2) & 3)
#define SYNC_PHASE(v)           (((v) >> 4) & 0xffff)
#define SYNC_PHASE_DIR          (1u << 20)

enum {
    MOD_FREEZE,
    MOD_UP,
    MOD_DOWN,
    MOD_UPDOWN,
};

enum {
    START_STOP_TEZ,
    START_STOP_TEP,
    START_RUN,
    START_ONCE_TEZ,
    START_ONCE_TEP,
};

#define SYNCO_SYNCI             0
#define SYNCO_TEZ               1
#define SYNCO_TEP               2

#define SYNCI_CFG_MASK          0xfffu
/* Sync input selections: 1-3 timer sync_out, 4-6 GPIO SYNC0-2 */
#define SYNCI_SEL(cfg, i)       (((cfg) >> (3 * (i))) & 7)
#define SYNCI_SEL_TIMER(i)      (1 + (i))
#define SYNCI_SEL_GPIO(n)       (4 + (n))
#define SYNCI_INVERT(n)         (1u << (9 + (n)))
#define TIMERSEL_MASK           0x3fu

#define STMP_UPMETHOD(v, n)     (((v) >> (4 * (n))) & 0xf)
#define STMP_SHDW_FULL(n)       (1u << (8 + (n)))
#define GEN_CFG0_MASK           0x3ffu
#define GEN_CFG0_UPMETHOD(v)    ((v) & 0xf)
#define GEN_CFG0_TSEL(v, n)     (((v) >> (4 + 3 * (n))) & 7)
#define TSEL_SYNC               3
#define FORCE_MASK              0xffffu
#define FORCE_RESET             0x20u
#define FORCE_CNTU_UPMETHOD(v)  ((v) & 0x3f)
#define FORCE_CNTU_MODES(v)     (((v) >> 6) & 0xf)
#define FORCE_NCI(ch)           (1u << (10 + 3 * (ch)))
#define FORCE_NCI_MODE(v, ch)   (((v) >> (11 + 3 * (ch))) & 3)
#define GEN_MASK                0xffffffu
#define DT_CFG_MASK             0x3ffffu
#define DT_CFG_RESET            0x18000u
#define DT_FED_UPMETHOD(v)      ((v) & 0xf)
#define DT_RED_UPMETHOD(v)      (((v) >> 4) & 0xf)
#define DT_DEB_MODE             (1u << 8)
#define DT_A_OUTSWAP            (1u << 9)
#define DT_B_OUTSWAP            (1u << 10)
#define DT_RED_INSEL            (1u << 11)
#define DT_FED_INSEL            (1u << 12)
#define DT_RED_OUTINVERT        (1u << 13)
#define DT_FED_OUTINVERT        (1u << 14)
#define DT_A_OUTBYPASS          (1u << 15)
#define DT_B_OUTBYPASS          (1u << 16)
#define DT_CLK_SEL              (1u << 17)
#define DT_DELAY_MASK           0xffffu
#define CARRIER_MASK            0x3fffu
#define CARRIER_EN              (1u << 0)
#define CARRIER_PRESCALE(v)     (((v) >> 1) & 0xf)
#define CARRIER_DUTY(v)         (((v) >> 5) & 7)
#define CARRIER_OSHTWTH(v)      (((v) >> 8) & 0xf)
#define CARRIER_OUT_INVERT      (1u << 12)
#define CARRIER_IN_INVERT       (1u << 13)
#define FH_CFG0_MASK            0xffffffu
#define FH_SW_CBC               (1u << 0)
#define FH_F_CBC(n)             (1u << (3 - (n)))
#define FH_SW_OST               (1u << 4)
#define FH_F_OST(n)             (1u << (7 - (n)))
#define FH_CFG1_MASK            0x1fu
#define FH_CLR_OST              (1u << 0)
#define FH_CBCPULSE(v)          (((v) >> 1) & 3)
#define FH_FORCE_CBC            (1u << 3)
#define FH_FORCE_OST            (1u << 4)
#define FAULT_DETECT_MASK       0x3fu
#define FAULT_EN(n)             (1u << (n))
#define FAULT_POLE(n)           (1u << (3 + (n)))
#define CAP_TIMER_CFG_MASK      0x1fu
#define CAP_TIMER_EN            (1u << 0)
#define CAP_SYNCI_EN            (1u << 1)
#define CAP_SYNCI_SEL(v)        (((v) >> 2) & 7)
#define CAP_SYNC_SW             (1u << 5)
#define CAP_CH_CFG_MASK         0xfffu
#define CAP_EN                  (1u << 0)
#define CAP_MODE_NEG            (1u << 1)
#define CAP_MODE_POS            (1u << 2)
#define CAP_PRESCALE(v)         (((v) >> 3) & 0xff)
#define CAP_IN_INVERT           (1u << 11)
#define CAP_SW                  (1u << 12)
#define UPDATE_CFG_MASK         0xffu
#define UPDATE_CFG_RESET        0x55u
#define UP_GLOBAL_EN            (1u << 0)
#define UP_GLOBAL_FORCE         (1u << 1)
#define UP_OP_EN(k)             (1u << (2 + 2 * (k)))
#define UP_OP_FORCE(k)          (1u << (3 + 2 * (k)))
#define INT_MASK                0x3fffffffu
#define INT_TIMER_STOP(i)       (1u << (i))
#define INT_TIMER_TEZ(i)        (1u << (3 + (i)))
#define INT_TIMER_TEP(i)        (1u << (6 + (i)))
#define INT_FAULT(n)            (1u << (9 + (n)))
#define INT_FAULT_CLR(n)        (1u << (12 + (n)))
#define INT_OP_TEA(k)           (1u << (15 + (k)))
#define INT_OP_TEB(k)           (1u << (18 + (k)))
#define INT_FH_CBC(k)           (1u << (21 + (k)))
#define INT_FH_OST(k)           (1u << (24 + (k)))
#define INT_CAP(n)              (1u << (27 + (n)))
#define VERSION_MASK            0x0fffffffu
#define VERSION_RESET           0x02107230u

/* Timing events seen by an operator */
#define EV_TEZ                  (1u << 0)
#define EV_TEP                  (1u << 1)
#define EV_TEA                  (1u << 2)
#define EV_TEB                  (1u << 3)
#define EV_SYNC                 (1u << 4)
#define EV_T0                   (1u << 5)
#define EV_T1                   (1u << 6)

/* An operator's shadow registers awaiting transfer */
#define OP_PEND_A               (1u << 0)
#define OP_PEND_B               (1u << 1)
#define OP_PEND_GEN             (1u << 2)
#define OP_PEND_FED             (1u << 3)
#define OP_PEND_RED             (1u << 4)
#define OP_PEND_CNTU            (1u << 5)
#define OP_PEND_ALL             0x3fu

/* Generator and fault handler actions */
enum {
    ACT_NONE,
    ACT_LOW,
    ACT_HIGH,
    ACT_TOGGLE,
};

#define NO_EVENT                UINT64_MAX

/*
 * Periodicity detection: a snapshot of the normalised state is taken after
 * SNAP_AFTER events of a run, and the run looks for it to recur for up to
 * SNAP_WINDOW events before taking a fresh one, waiting twice as long each
 * time, up to SNAP_WINDOW_MAX.
 */
#define SNAP_AFTER              8
#define SNAP_WINDOW             4096
#define SNAP_WINDOW_MAX         (1u << 20)
/* The most overdue timer callbacks an access runs before its own work */
#define MAX_LATE_CALLBACKS      64
/* The most events a look-ahead run takes before giving up for now. */
#define WATCH_MAX_EVENTS        65536

/* What a look-ahead run is looking for. */
typedef struct McpwmWatch {
    const uint8_t *driven;      /* output levels as driven */
    bool irq;                   /* whether a rising interrupt counts */
    uint64_t floor;             /* output changes count from this cycle */
} McpwmWatch;

typedef enum McpwmSeen {
    SEEN_NOTHING,               /* nothing visible will ever happen */
    SEEN_HORIZON,               /* nothing visible yet; look again later */
    SEEN_OUTPUT,
    SEEN_IRQ,
} McpwmSeen;

/* Timers */

static unsigned timer_mode(const Esp32McpwmTimer *t)
{
    return CFG1_MOD(t->cfg1);
}

static bool timer_counting(const Esp32McpwmTimer *t)
{
    return t->running && timer_mode(t) != MOD_FREEZE;
}

/* One PT_clk tick. Up-down counts 0..P-1 up and P..1 down. */
static void timer_tick(Esp32McpwmTimer *t)
{
    uint32_t p = t->period;

    switch (timer_mode(t)) {
    case MOD_UP:
        t->value = t->value >= p ? 0 : t->value + 1;
        break;
    case MOD_DOWN:
        t->value = t->value == 0 ? p : t->value - 1;
        break;
    case MOD_UPDOWN:
        if (!t->down) {
            if (t->value + 1 >= p) {
                t->value = p;
                t->down = true;
            } else {
                t->value++;
            }
        } else {
            if (t->value <= 1) {
                t->value = 0;
                t->down = false;
            } else {
                t->value--;
            }
        }
        break;
    }
}

/*
 * Up-down mode, from position pos of the 2P-long cycle (pos < P counting
 * up from pos, pos >= P counting down from 2P - pos): ticks until the
 * counter next reads v.
 */
static uint64_t updown_ticks(uint32_t p, uint32_t pos, uint32_t v)
{
    uint64_t best = NO_EVENT;
    uint32_t q[2];
    int n = 0;

    if (v < p) {
        q[n++] = v;
    }
    if (v >= 1 && v <= p) {
        q[n++] = 2 * p - v;
    }
    for (int i = 0; i < n; i++) {
        uint64_t d = (q[i] + 2 * p - pos) % (2 * p);

        best = MIN(best, d ? d : 2 * p);
    }
    return best;
}

/* PT_clk ticks until the counter next reads v, or NO_EVENT. */
static uint64_t timer_ticks_to(const Esp32McpwmTimer *t, uint32_t v)
{
    uint32_t p = t->period;
    uint32_t x = t->value;

    switch (timer_mode(t)) {
    case MOD_UP:
        if (v > p) {
            return NO_EVENT;
        }
        if (x >= p) {
            return 1 + (uint64_t)v;
        }
        if (v > x) {
            return v - x;
        }
        return (uint64_t)(p - x) + 1 + v;
    case MOD_DOWN:
        if (x == 0) {
            return v <= p ? 1 + (uint64_t)(p - v) : NO_EVENT;
        }
        if (v < x) {
            return x - v;
        }
        return v <= p ? (uint64_t)x + 1 + (p - v) : NO_EVENT;
    case MOD_UPDOWN:
        if (p == 0) {
            return v == 0 ? 1 : NO_EVENT;
        }
        if (!t->down) {
            if (x < p) {
                return updown_ticks(p, x, v);
            }
            /* The next tick turns round at P. */
            return v == p ? 1 : 1 + updown_ticks(p, p, v);
        }
        if (x == 0) {
            return v == 0 ? 1 : 1 + updown_ticks(p, 0, v);
        }
        if (x <= p) {
            return updown_ticks(p, 2 * p - x, v);
        }
        if (v < x && v >= p) {
            return x - v;
        }
        return (x - p) + updown_ticks(p, p, v);
    default:
        return NO_EVENT;
    }
}

/*
 * Count `ticks` ticks known to cross no event value: the counter moves
 * monotonically in its current direction.
 */
static void timer_move(Esp32McpwmTimer *t, uint64_t ticks)
{
    bool down = timer_mode(t) == MOD_DOWN ||
                (timer_mode(t) == MOD_UPDOWN && t->down);

    t->value = down ? t->value - ticks : t->value + ticks;
}

/* Operators */

static int op_timer(const Esp32McpwmCore *c, int k)
{
    int i = (c->timersel >> (2 * k)) & 3;

    return i < ESP32_MCPWM_TIMERS ? i : -1;
}

static bool op_timer_down(const Esp32McpwmCore *c, int k)
{
    int i = op_timer(c, k);

    return i >= 0 && c->timer[i].down;
}

/* Whether an operator's compare value v produces TEA/TEB events. */
static bool op_cmp_live(const Esp32McpwmCore *c, int i, uint32_t v)
{
    return v <= c->timer[i].period;
}

static bool op_update_enabled(const Esp32McpwmCore *c, int k)
{
    return (c->update_cfg & UP_GLOBAL_EN) && (c->update_cfg & UP_OP_EN(k));
}

/* The update method of each pending shadow register. */
static uint32_t op_method(const Esp32McpwmOp *o, uint32_t pend)
{
    switch (pend) {
    case OP_PEND_A:
        return STMP_UPMETHOD(o->stmp_cfg, 0);
    case OP_PEND_B:
        return STMP_UPMETHOD(o->stmp_cfg, 1);
    case OP_PEND_GEN:
        return GEN_CFG0_UPMETHOD(o->gen_cfg0);
    case OP_PEND_FED:
        return DT_FED_UPMETHOD(o->dt_cfg);
    case OP_PEND_RED:
        return DT_RED_UPMETHOD(o->dt_cfg);
    case OP_PEND_CNTU:
    default:
        return FORCE_CNTU_UPMETHOD(o->force);
    }
}

/*
 * Whether update method m takes the shadow at events ev. 0 is immediate;
 * the four-bit methods are TEZ, TEP, sync and disable; the continuous
 * force's six-bit method is TEZ, TEP, TEA, TEB, sync and disable.
 */
static bool op_method_hit(uint32_t pend, uint32_t m, uint32_t ev)
{
    if (m == 0) {
        return true;
    }
    if (pend == OP_PEND_CNTU) {
        return !(m & 0x20) &&
               (((m & 0x01) && (ev & EV_TEZ)) ||
                ((m & 0x02) && (ev & EV_TEP)) ||
                ((m & 0x04) && (ev & EV_TEA)) ||
                ((m & 0x08) && (ev & EV_TEB)) ||
                ((m & 0x10) && (ev & EV_SYNC)));
    }
    return !(m & 0x8) &&
           (((m & 0x1) && (ev & EV_TEZ)) ||
            ((m & 0x2) && (ev & EV_TEP)) ||
            ((m & 0x4) && (ev & EV_SYNC)));
}

static void op_apply_force(Esp32McpwmOp *o)
{
    for (int ch = 0; ch < 2; ch++) {
        switch ((o->cntu_act >> (2 * ch)) & 3) {
        case ACT_LOW:
            o->gen_out[ch] = false;
            break;
        case ACT_HIGH:
            o->gen_out[ch] = true;
            break;
        }
    }
}

static void op_transfer(Esp32McpwmOp *o, uint32_t which)
{
    which &= o->pending;
    if (which & OP_PEND_A) {
        o->cmp[0] = o->tstmp[0];
    }
    if (which & OP_PEND_B) {
        o->cmp[1] = o->tstmp[1];
    }
    if (which & OP_PEND_GEN) {
        o->gen_act[0] = o->gen[0];
        o->gen_act[1] = o->gen[1];
    }
    if (which & OP_PEND_FED) {
        o->fed_act = o->dt_fed;
    }
    if (which & OP_PEND_RED) {
        o->red_act = o->dt_red;
    }
    if (which & OP_PEND_CNTU) {
        o->cntu_act = FORCE_CNTU_MODES(o->force);
    }
    o->pending &= ~which;
}

/* Transfer the shadows whose update method takes them at events ev. */
static void op_shadow_events(Esp32McpwmCore *c, int k, uint32_t ev)
{
    Esp32McpwmOp *o = &c->op[k];
    uint32_t which = 0;

    if (!op_update_enabled(c, k)) {
        return;
    }
    for (uint32_t pend = 1; pend & OP_PEND_ALL; pend <<= 1) {
        if ((o->pending & pend) &&
            op_method_hit(pend, op_method(o, pend), ev)) {
            which |= pend;
        }
    }
    op_transfer(o, which);
}

/* Transfer the shadows whose update method is immediate. */
static void op_immediate(Esp32McpwmCore *c, int k)
{
    Esp32McpwmOp *o = &c->op[k];
    uint32_t which = 0;

    if (!op_update_enabled(c, k)) {
        return;
    }
    for (uint32_t pend = 1; pend & OP_PEND_ALL; pend <<= 1) {
        if ((o->pending & pend) && op_method(o, pend) == 0) {
            which |= pend;
        }
    }
    op_transfer(o, which);
}

/*
 * The generators' response to simultaneous events: the action of the
 * highest-priority event that has one (TRM tables 29.3-3 and 29.3-4). The
 * continuous software force, when on, holds the output instead.
 */
static void op_generate(Esp32McpwmOp *o, uint32_t ev, bool down)
{
    /* Field offsets in GENx_A/B, highest priority first */
    static const uint8_t up_order[] = { 2, 8, 10, 6, 4, 0 };
    static const uint8_t down_order[] = { 0, 8, 10, 6, 4, 2 };
    static const uint32_t field_ev[] = {
        [0] = EV_TEZ, [2] = EV_TEP, [4] = EV_TEA, [6] = EV_TEB,
        [8] = EV_T0, [10] = EV_T1,
    };
    const uint8_t *order = down ? down_order : up_order;

    for (int ch = 0; ch < 2; ch++) {
        for (int i = 0; i < ARRAY_SIZE(up_order); i++) {
            unsigned f = order[i];
            unsigned act;

            if (!(ev & field_ev[f])) {
                continue;
            }
            act = (o->gen_act[ch] >> (f + (down ? 12 : 0))) & 3;
            if (act == ACT_NONE) {
                continue;
            }
            if (act == ACT_TOGGLE) {
                o->gen_out[ch] = !o->gen_out[ch];
            } else {
                o->gen_out[ch] = act == ACT_HIGH;
            }
            break;
        }
    }
    op_apply_force(o);
}

/* Whether a fault handler source of each kind is active. */
static void fh_sources(const Esp32McpwmCore *c, int k, bool *ost, bool *cbc)
{
    uint32_t f = c->op[k].fh_cfg0;

    *ost = false;
    *cbc = false;
    for (int n = 0; n < ESP32_MCPWM_FAULTS; n++) {
        if (c->fault_ev[n]) {
            *ost |= !!(f & FH_F_OST(n));
            *cbc |= !!(f & FH_F_CBC(n));
        }
    }
}

/*
 * Trip the fault handler of operator k on its active sources, and on the
 * software triggers given.
 */
static void fh_trip(Esp32McpwmCore *c, int k, bool sw_ost, bool sw_cbc)
{
    Esp32McpwmOp *o = &c->op[k];
    bool ost, cbc;

    fh_sources(c, k, &ost, &cbc);
    ost |= sw_ost && (o->fh_cfg0 & FH_SW_OST);
    cbc |= sw_cbc && (o->fh_cfg0 & FH_SW_CBC);
    if (ost && !o->ost_on) {
        o->ost_on = true;
        c->int_raw |= INT_FH_OST(k);
    }
    if (cbc && !o->cbc_on) {
        o->cbc_on = true;
        c->int_raw |= INT_FH_CBC(k);
    }
}

/* Operator k's response to timing events ev of its timer. */
static void op_timer_events(Esp32McpwmCore *c, int k, int i, uint32_t ev)
{
    Esp32McpwmOp *o = &c->op[k];
    Esp32McpwmTimer *t = &c->timer[i];
    uint32_t pulse = FH_CBCPULSE(o->fh_cfg1);

    if (t->value == o->cmp[0] && op_cmp_live(c, i, o->cmp[0])) {
        ev |= EV_TEA;
        c->int_raw |= INT_OP_TEA(k);
    }
    if (t->value == o->cmp[1] && op_cmp_live(c, i, o->cmp[1])) {
        ev |= EV_TEB;
        c->int_raw |= INT_OP_TEB(k);
    }
    if (ev & (EV_TEZ | EV_TEP | EV_TEA | EV_TEB)) {
        op_generate(o, ev, t->down);
    }
    op_shadow_events(c, k, ev);
    op_apply_force(o);

    /*
     * A cycle-by-cycle trip ends at the selected refresh event once no
     * source of it is active.
     */
    if (((pulse & 1) && (ev & EV_TEZ)) || ((pulse & 2) && (ev & EV_TEP))) {
        bool ost, cbc;

        fh_sources(c, k, &ost, &cbc);
        o->cbc_on = cbc;
    }
}

/* A sync taken by timer i: the T0/T1 sync events of its operators. */
static void op_sync_taken(Esp32McpwmCore *c, int i)
{
    for (int k = 0; k < ESP32_MCPWM_OPS; k++) {
        Esp32McpwmOp *o = &c->op[k];
        uint32_t ev = EV_SYNC;

        if (op_timer(c, k) != i) {
            continue;
        }
        if (GEN_CFG0_TSEL(o->gen_cfg0, 0) == TSEL_SYNC) {
            ev |= EV_T0;
        }
        if (GEN_CFG0_TSEL(o->gen_cfg0, 1) == TSEL_SYNC) {
            ev |= EV_T1;
        }
        if (ev & (EV_T0 | EV_T1)) {
            op_generate(o, ev, c->timer[i].down);
        }
        op_shadow_events(c, k, ev);
        op_apply_force(o);
    }
}

static void timer_transfer(Esp32McpwmCore *c, int i, bool force)
{
    Esp32McpwmTimer *t = &c->timer[i];

    if (t->period_pending && (force || (c->update_cfg & UP_GLOBAL_EN))) {
        t->period = CFG0_PERIOD(t->cfg0);
        t->period_pending = false;
    }
}

static void timer_sync_out(Esp32McpwmCore *c, int i, unsigned visited);

/*
 * Timer j takes a sync: it reloads with the phase and its prescaler
 * restarts. Up-down mode takes the direction from PHASE_DIRECTION.
 */
static void timer_take_sync(Esp32McpwmCore *c, int j)
{
    Esp32McpwmTimer *t = &c->timer[j];

    t->value = SYNC_PHASE(t->sync);
    switch (timer_mode(t)) {
    case MOD_UP:
        t->down = false;
        break;
    case MOD_DOWN:
        t->down = true;
        break;
    default:
        t->down = !!(t->sync & SYNC_PHASE_DIR);
        break;
    }
    t->pre_cnt = 0;
    if (CFG0_UPMETHOD(t->cfg0) & 2) {
        timer_transfer(c, j, false);
    }
    op_sync_taken(c, j);
}

/* A sync event arriving at timer j's input. */
static void timer_sync_in(Esp32McpwmCore *c, int j, unsigned visited)
{
    Esp32McpwmTimer *t = &c->timer[j];

    if (visited & (1u << j)) {
        return;
    }
    visited |= 1u << j;
    if (t->sync & SYNC_SYNCI_EN) {
        timer_take_sync(c, j);
    }
    if (SYNC_SYNCO_SEL(t->sync) == SYNCO_SYNCI) {
        timer_sync_out(c, j, visited);
    }
}

/* Timer i's sync_out: to the timers and capture timer that select it. */
static void timer_sync_out(Esp32McpwmCore *c, int i, unsigned visited)
{
    for (int j = 0; j < ESP32_MCPWM_TIMERS; j++) {
        if (SYNCI_SEL(c->synci_cfg, j) == SYNCI_SEL_TIMER(i)) {
            timer_sync_in(c, j, visited);
        }
    }
    if ((c->cap_timer_cfg & CAP_SYNCI_EN) &&
        CAP_SYNCI_SEL(c->cap_timer_cfg) == SYNCI_SEL_TIMER(i)) {
        c->cap_sync_hit = true;
        c->cap_sync = c->now;
    }
}

/* The events of a tick of timer i, which has just ticked. */
static void timer_events(Esp32McpwmCore *c, int i)
{
    Esp32McpwmTimer *t = &c->timer[i];
    uint32_t ev = 0;
    unsigned start = CFG1_START(t->cfg1);
    unsigned synco = SYNC_SYNCO_SEL(t->sync);

    if (t->value == 0) {
        ev |= EV_TEZ;
        c->int_raw |= INT_TIMER_TEZ(i);
    }
    if (t->value == t->period) {
        ev |= EV_TEP;
        c->int_raw |= INT_TIMER_TEP(i);
    }
    for (int k = 0; k < ESP32_MCPWM_OPS; k++) {
        if (op_timer(c, k) == i) {
            op_timer_events(c, k, i, ev);
        }
    }
    if ((ev & EV_TEZ) && (CFG0_UPMETHOD(t->cfg0) & 1)) {
        timer_transfer(c, i, false);
    }
    if (((start == START_STOP_TEZ || start == START_ONCE_TEZ) &&
         (ev & EV_TEZ)) ||
        ((start == START_STOP_TEP || start == START_ONCE_TEP) &&
         (ev & EV_TEP))) {
        /* The one-shot starts read back as the stopped state they reach. */
        t->running = false;
        c->int_raw |= INT_TIMER_STOP(i);
        if (start == START_ONCE_TEZ) {
            t->cfg1 = (t->cfg1 & ~7u) | START_STOP_TEZ;
        } else if (start == START_ONCE_TEP) {
            t->cfg1 = (t->cfg1 & ~7u) | START_STOP_TEP;
        }
    }
    if ((synco == SYNCO_TEZ && (ev & EV_TEZ)) ||
        (synco == SYNCO_TEP && (ev & EV_TEP))) {
        timer_sync_out(c, i, 1u << i);
    }
}

/* Dead time and carrier */

/* An operator's dead time, RED or FED register value v, in PWM_clk cycles */
static uint64_t dt_delay(const Esp32McpwmCore *c, int k, uint32_t v)
{
    int i = op_timer(c, k);
    uint64_t unit = 1;

    /*
     * The delay is the register value plus one DT_clk cycles, as
     * ESP-IDF's mcpwm_ll programs it; the TRM's formula omits the one.
     */
    if ((c->op[k].dt_cfg & DT_CLK_SEL) && i >= 0) {
        unit = c->timer[i].prescale + 1;
    }
    return ((uint64_t)v + 1) * unit;
}

static void delay_input(Esp32McpwmDelay *d, bool in, bool rising,
                        uint64_t delay, uint64_t now)
{
    if (in == d->in) {
        return;
    }
    d->in = in;
    if (in == rising) {
        d->due = now + delay;
    } else {
        d->due = NO_EVENT;
        d->out = in;
    }
}

/* The dead-time generator's outputs (TRM figure 29.3-21, S0-S8). */
static bool op_dt_out(const Esp32McpwmOp *o, int ch)
{
    uint32_t d = o->dt_cfg;
    bool red_path = o->red.out ^ !!(d & DT_RED_OUTINVERT);
    bool fed_path = o->fed.out ^ !!(d & DT_FED_OUTINVERT);
    bool a, b;

    if (d & DT_A_OUTBYPASS) {
        a = o->gen_out[0];
    } else {
        /* Dual-edge B mode: A duplicates the B path. */
        a = (d & DT_DEB_MODE) ? fed_path : red_path;
    }
    b = (d & DT_B_OUTBYPASS) ? o->gen_out[1] : fed_path;
    if (ch == 0) {
        return (d & DT_A_OUTSWAP) ? b : a;
    }
    return (d & DT_B_OUTSWAP) ? a : b;
}

/* The carrier's period and first pulse, in PWM_clk cycles */
static uint64_t car_unit(const Esp32McpwmOp *o)
{
    return CARRIER_PRESCALE(o->carrier) + 1;
}

static uint64_t car_first(const Esp32McpwmOp *o)
{
    return 8 * car_unit(o) * (CARRIER_OSHTWTH(o->carrier) + 1);
}

/*
 * The carrier at `age` cycles after its input rose: the one-shot first
 * pulse, then pulses of DUTY/8 of each 8-PC_clk period.
 */
static bool car_wave(const Esp32McpwmOp *o, uint64_t age)
{
    uint64_t unit = car_unit(o);
    uint64_t first = car_first(o);

    if (age < first) {
        return true;
    }
    return (age - first) % (8 * unit) < CARRIER_DUTY(o->carrier) * unit;
}

/* The age at which the carrier next changes after `age`, or NO_EVENT. */
static uint64_t car_next(const Esp32McpwmOp *o, uint64_t age)
{
    uint64_t unit = car_unit(o);
    uint64_t first = car_first(o);
    uint64_t high = CARRIER_DUTY(o->carrier) * unit;
    uint64_t e, base;

    if (age < first) {
        return high ? first + high : first;
    }
    if (!high) {
        return NO_EVENT;
    }
    e = (age - first) % (8 * unit);
    base = age - e;
    return e < high ? base + high : base + 8 * unit;
}

/* The age normalised to the carrier's repetition. */
static uint64_t car_norm(const Esp32McpwmOp *o, uint64_t age)
{
    uint64_t first = car_first(o);

    if (age < first) {
        return age;
    }
    return first + (age - first) % (8 * car_unit(o));
}

/*
 * Propagate operator k's generator outputs into its dead-time delays and
 * carrier, at the current cycle.
 */
static void op_paths(Esp32McpwmCore *c, int k)
{
    Esp32McpwmOp *o = &c->op[k];
    uint32_t d = o->dt_cfg;
    bool red_in = o->gen_out[(d & DT_RED_INSEL) ? 1 : 0];
    bool fed_in;

    delay_input(&o->red, red_in, true, dt_delay(c, k, o->red_act), c->now);
    if (d & DT_DEB_MODE) {
        fed_in = o->red.out;
    } else {
        fed_in = o->gen_out[(d & DT_FED_INSEL) ? 1 : 0];
    }
    delay_input(&o->fed, fed_in, false, dt_delay(c, k, o->fed_act), c->now);

    for (int ch = 0; ch < 2; ch++) {
        bool x = false;

        if (o->carrier & CARRIER_EN) {
            x = op_dt_out(o, ch) ^ !!(o->carrier & CARRIER_IN_INVERT);
        }
        if (x && !o->car_in[ch]) {
            o->car_rise[ch] = c->now;
        }
        o->car_in[ch] = x;
    }
}

/*
 * Timers linked by sync, with the operators that follow them, evolve
 * independently of the others: a group, as a mask of its timers. An
 * operator that selects no timer goes with timer 0's group.
 */
#define ALL_TIMERS ((1u << ESP32_MCPWM_TIMERS) - 1)

static bool op_in(const Esp32McpwmCore *c, int k, unsigned group)
{
    int i = op_timer(c, k);

    return group & (1u << (i >= 0 ? i : 0));
}

static int core_groups(const Esp32McpwmCore *c, unsigned *groups)
{
    unsigned grp[ESP32_MCPWM_TIMERS];
    int n = 0;

    for (int i = 0; i < ESP32_MCPWM_TIMERS; i++) {
        grp[i] = 1u << i;
    }
    /* Merge each timer with the one whose sync_out it selects. */
    for (int pass = 0; pass < ESP32_MCPWM_TIMERS; pass++) {
        for (int j = 0; j < ESP32_MCPWM_TIMERS; j++) {
            unsigned sel = SYNCI_SEL(c->synci_cfg, j);

            if (sel >= SYNCI_SEL_TIMER(0) &&
                sel <= SYNCI_SEL_TIMER(ESP32_MCPWM_TIMERS - 1)) {
                unsigned m = grp[j] | grp[sel - SYNCI_SEL_TIMER(0)];

                for (int i = 0; i < ESP32_MCPWM_TIMERS; i++) {
                    if (m & (1u << i)) {
                        grp[i] = m;
                    }
                }
            }
        }
    }
    for (int i = 0; i < ESP32_MCPWM_TIMERS; i++) {
        if (grp[i] & ((1u << i) - 1)) {
            continue;
        }
        groups[n++] = grp[i];
    }
    return n;
}

static void core_settle_group(Esp32McpwmCore *c, unsigned group)
{
    for (int k = 0; k < ESP32_MCPWM_OPS; k++) {
        if (op_in(c, k, group)) {
            op_apply_force(&c->op[k]);
            op_paths(c, k);
        }
    }
}

static void core_settle(Esp32McpwmCore *c)
{
    core_settle_group(c, ALL_TIMERS);
}

/* The fault handler's action on channel ch of operator k. */
static unsigned fh_action(const Esp32McpwmCore *c, int k, int ch)
{
    const Esp32McpwmOp *o = &c->op[k];
    unsigned shift;

    if (!o->ost_on && !o->cbc_on) {
        return ACT_NONE;
    }
    /* One-shot trips take precedence over cycle-by-cycle ones. */
    shift = 8 + 8 * ch + (o->ost_on ? 4 : 0) +
            (op_timer_down(c, k) ? 0 : 2);
    return (o->fh_cfg0 >> shift) & 3;
}

/* PWMxA (ch 0) or PWMxB (ch 1) of operator k at the current cycle. */
static bool op_out(const Esp32McpwmCore *c, int k, int ch)
{
    const Esp32McpwmOp *o = &c->op[k];
    bool v = op_dt_out(o, ch);

    if (o->carrier & CARRIER_EN) {
        v = o->car_in[ch] && car_wave(o, c->now - o->car_rise[ch]);
        v ^= !!(o->carrier & CARRIER_OUT_INVERT);
    }
    switch (fh_action(c, k, ch)) {
    case ACT_LOW:
        return false;
    case ACT_HIGH:
        return true;
    case ACT_TOGGLE:
        return !v;
    default:
        return v;
    }
}

/* The next cycle after now at which a carrier can change an output. */
static uint64_t core_next_carrier(const Esp32McpwmCore *c, unsigned group)
{
    uint64_t next = NO_EVENT;

    for (int k = 0; k < ESP32_MCPWM_OPS; k++) {
        const Esp32McpwmOp *o = &c->op[k];

        if (!op_in(c, k, group) || !(o->carrier & CARRIER_EN)) {
            continue;
        }
        for (int ch = 0; ch < 2; ch++) {
            unsigned act = fh_action(c, k, ch);
            uint64_t age;

            if (!o->car_in[ch] || act == ACT_LOW || act == ACT_HIGH) {
                continue;
            }
            age = car_next(o, c->now - o->car_rise[ch]);
            if (age != NO_EVENT) {
                next = MIN(next, o->car_rise[ch] + age);
            }
        }
    }
    return next;
}

/* Running the core */

static uint64_t timer_next(const Esp32McpwmCore *c, int i)
{
    const Esp32McpwmTimer *t = &c->timer[i];
    uint64_t pre = t->prescale + 1;
    uint64_t k;

    if (!timer_counting(t)) {
        return NO_EVENT;
    }
    k = MIN(timer_ticks_to(t, 0), timer_ticks_to(t, t->period));
    for (int n = 0; n < ESP32_MCPWM_OPS; n++) {
        const Esp32McpwmOp *o = &c->op[n];

        if (op_timer(c, n) != i) {
            continue;
        }
        for (int cmp = 0; cmp < 2; cmp++) {
            if (op_cmp_live(c, i, o->cmp[cmp])) {
                k = MIN(k, timer_ticks_to(t, o->cmp[cmp]));
            }
        }
    }
    if (k == NO_EVENT) {
        return NO_EVENT;
    }
    return c->now + (pre - t->pre_cnt) + (k - 1) * pre;
}

static uint64_t core_next(const Esp32McpwmCore *c, uint64_t *tdue,
                          unsigned group)
{
    uint64_t next = NO_EVENT;

    for (int i = 0; i < ESP32_MCPWM_TIMERS; i++) {
        tdue[i] = (group & (1u << i)) ? timer_next(c, i) : NO_EVENT;
        next = MIN(next, tdue[i]);
    }
    for (int k = 0; k < ESP32_MCPWM_OPS; k++) {
        if (op_in(c, k, group)) {
            next = MIN(next, c->op[k].red.due);
            next = MIN(next, c->op[k].fed.due);
        }
    }
    return next;
}

/*
 * Move to cycle `to`, at or before the next event. Timers due at `to` take
 * their tick; the mask of them is returned.
 */
static unsigned core_advance(Esp32McpwmCore *c, uint64_t to,
                             const uint64_t *tdue, unsigned group)
{
    uint64_t dc = to - c->now;
    unsigned due = 0;

    for (int i = 0; i < ESP32_MCPWM_TIMERS; i++) {
        Esp32McpwmTimer *t = &c->timer[i];
        uint64_t pre = t->prescale + 1;
        uint64_t total;
        bool tick = tdue && tdue[i] == to;

        if (!(group & (1u << i)) || !timer_counting(t) || !dc) {
            continue;
        }
        total = t->pre_cnt + dc - (tick ? 1 : 0);
        timer_move(t, total / pre);
        t->pre_cnt = total % pre;
        if (tick) {
            t->pre_cnt = 0;
            timer_tick(t);
            due |= 1u << i;
        }
    }
    c->now = to;
    return due;
}

/* Process the events at cycle `to`. */
static void core_step(Esp32McpwmCore *c, uint64_t to, const uint64_t *tdue,
                      unsigned group)
{
    unsigned due = core_advance(c, to, tdue, group);

    for (int k = 0; k < ESP32_MCPWM_OPS; k++) {
        Esp32McpwmOp *o = &c->op[k];

        if (!op_in(c, k, group)) {
            continue;
        }
        if (o->red.due == to) {
            o->red.out = o->red.in;
            o->red.due = NO_EVENT;
        }
        if (o->fed.due == to) {
            o->fed.out = o->fed.in;
            o->fed.due = NO_EVENT;
        }
    }
    for (int i = 0; i < ESP32_MCPWM_TIMERS; i++) {
        if (due & (1u << i)) {
            timer_events(c, i);
        }
    }
    core_settle_group(c, group);
}

/*
 * A group's state with its times made relative to now, and everything
 * outside it cleared, for comparison: equal normalised states evolve
 * alike.
 */
static void core_normalise(Esp32McpwmCore *n, const Esp32McpwmCore *c,
                           unsigned group)
{
    memcpy(n, c, sizeof(*n));
    n->now = 0;
    n->cap_sync_hit = false;
    n->cap_sync = 0;
    for (int i = 0; i < ESP32_MCPWM_TIMERS; i++) {
        if (!(group & (1u << i))) {
            memset(&n->timer[i], 0, sizeof(n->timer[i]));
        }
    }
    for (int k = 0; k < ESP32_MCPWM_OPS; k++) {
        Esp32McpwmOp *o = &n->op[k];

        if (!op_in(c, k, group)) {
            memset(o, 0, sizeof(*o));
            continue;
        }
        if (o->red.due != NO_EVENT) {
            o->red.due -= c->now;
        }
        if (o->fed.due != NO_EVENT) {
            o->fed.due -= c->now;
        }
        for (int ch = 0; ch < 2; ch++) {
            if ((o->carrier & CARRIER_EN) && o->car_in[ch]) {
                o->car_rise[ch] = car_norm(o, c->now - o->car_rise[ch]);
            } else {
                o->car_rise[ch] = 0;
            }
        }
    }
}

/* Move a group's state, unchanged, `d` cycles later. */
static void core_shift(Esp32McpwmCore *c, uint64_t d, uint64_t since,
                       unsigned group)
{
    c->now += d;
    for (int k = 0; k < ESP32_MCPWM_OPS; k++) {
        Esp32McpwmOp *o = &c->op[k];

        if (!op_in(c, k, group)) {
            continue;
        }
        if (o->red.due != NO_EVENT) {
            o->red.due += d;
        }
        if (o->fed.due != NO_EVENT) {
            o->fed.due += d;
        }
        o->car_rise[0] += d;
        o->car_rise[1] += d;
    }
    if (c->cap_sync_hit && c->cap_sync >= since) {
        c->cap_sync += d;
    }
}

static bool core_outputs_differ(const Esp32McpwmCore *c,
                                const uint8_t *driven, unsigned group)
{
    for (int n = 0; n < ESP32_MCPWM_OUTS; n++) {
        if (op_in(c, n / 2, group) && op_out(c, n / 2, n % 2) != driven[n]) {
            return true;
        }
    }
    return false;
}

/*
 * Run a group to cycle `target`. Without a watch, the run always reaches
 * it. With one, it stops at the first visible change and says what it
 * was: the run then serves only to look ahead.
 */
static McpwmSeen core_run_group(Esp32McpwmCore *c, uint64_t target,
                                const McpwmWatch *w, unsigned group)
{
    g_autofree Esp32McpwmCore *snap = NULL;
    g_autofree Esp32McpwmCore *cur = NULL;
    uint64_t tdue[ESP32_MCPWM_TIMERS];
    uint64_t snap_now = 0;
    unsigned events = 0, since = 0, window = SNAP_WINDOW;
    bool have_snap = false;
    uint32_t raw0 = c->int_raw & c->int_ena;

    while (c->now < target) {
        uint64_t next = core_next(c, tdue, group);

        if (w) {
            uint64_t check;

            if (c->now < w->floor) {
                check = w->floor;
            } else {
                check = core_next_carrier(c, group);
            }
            if (check < next && check <= target) {
                core_advance(c, check, NULL, group);
                if (c->now >= w->floor &&
                    core_outputs_differ(c, w->driven, group)) {
                    return SEEN_OUTPUT;
                }
                continue;
            }
            if (next == NO_EVENT) {
                return SEEN_NOTHING;
            }
        }
        if (next > target) {
            core_advance(c, target, NULL, group);
            break;
        }
        core_step(c, next, tdue, group);
        events++;

        if (w) {
            if (w->irq && (c->int_raw & c->int_ena & ~raw0)) {
                return SEEN_IRQ;
            }
            if (c->now >= w->floor &&
                core_outputs_differ(c, w->driven, group)) {
                return SEEN_OUTPUT;
            }
            if (events >= WATCH_MAX_EVENTS) {
                return SEEN_HORIZON;
            }
            if (c->now < w->floor) {
                continue;
            }
        }

        if (!have_snap) {
            if (events < SNAP_AFTER) {
                continue;
            }
            if (!snap) {
                snap = g_new(Esp32McpwmCore, 1);
                cur = g_new(Esp32McpwmCore, 1);
            }
            core_normalise(snap, c, group);
            snap_now = c->now;
            have_snap = true;
            since = 0;
            continue;
        }
        core_normalise(cur, c, group);
        if (memcmp(cur, snap, sizeof(*cur)) == 0) {
            uint64_t d = c->now - snap_now;

            if (w) {
                /* A whole repetition passed with nothing visible. */
                return SEEN_NOTHING;
            }
            core_shift(c, (target - c->now) / d * d, snap_now, group);
            have_snap = false;
            events = 0;
        } else if (++since > window) {
            /* Look for a longer repetition from a later state. */
            have_snap = false;
            window = MIN(window * 2, SNAP_WINDOW_MAX);
        }
    }
    return w ? SEEN_HORIZON : SEEN_NOTHING;
}

/* Run the core to cycle `target`, group by group. */
/* [spec:nuos:req:emu.esp32.mcpwm] */
static void core_run(Esp32McpwmCore *c, uint64_t target)
{
    unsigned groups[ESP32_MCPWM_TIMERS];
    int n = core_groups(c, groups);
    uint64_t start = c->now;
    bool hit = c->cap_sync_hit;
    uint64_t cap_sync = c->cap_sync;

    for (int g = 0; g < n; g++) {
        c->now = start;
        c->cap_sync_hit = false;
        core_run_group(c, target, NULL, groups[g]);
        if (c->cap_sync_hit && (!hit || c->cap_sync > cap_sync)) {
            hit = true;
            cap_sync = c->cap_sync;
        }
    }
    c->now = target;
    c->cap_sync_hit = hit;
    c->cap_sync = cap_sync;
}

/*
 * Look ahead from the core's state for the first visible change: the
 * earliest of each group's. Returns what it is, with *when its cycle.
 */
static McpwmSeen core_look_ahead(const Esp32McpwmCore *c, const McpwmWatch *w,
                                 uint64_t *when)
{
    g_autofree Esp32McpwmCore *ahead = g_new(Esp32McpwmCore, 1);
    unsigned groups[ESP32_MCPWM_TIMERS];
    int n = core_groups(c, groups);
    McpwmSeen best = SEEN_NOTHING;

    *when = NO_EVENT;
    for (int g = 0; g < n; g++) {
        McpwmSeen seen;

        memcpy(ahead, c, sizeof(*ahead));
        seen = core_run_group(ahead, NO_EVENT - 1, w, groups[g]);
        if (seen == SEEN_NOTHING) {
            continue;
        }
        if (ahead->now < *when || (ahead->now == *when && seen == SEEN_IRQ)) {
            *when = ahead->now;
            best = seen;
        }
    }
    return best;
}

/* Fault detection: fault_eventN from FAULTN, its enable and polarity. */
static void core_faults(Esp32McpwmCore *c)
{
    for (int n = 0; n < ESP32_MCPWM_FAULTS; n++) {
        bool ev = (c->fault_detect & FAULT_EN(n)) &&
                  c->fault_in[n] == !!(c->fault_detect & FAULT_POLE(n));

        if (ev == c->fault_ev[n]) {
            continue;
        }
        c->fault_ev[n] = ev;
        if (!ev) {
            c->int_raw |= INT_FAULT_CLR(n);
            continue;
        }
        c->int_raw |= INT_FAULT(n);
        /* The generators' T0/T1 events selected from this fault */
        for (int k = 0; k < ESP32_MCPWM_OPS; k++) {
            Esp32McpwmOp *o = &c->op[k];
            uint32_t tev = 0;

            if (GEN_CFG0_TSEL(o->gen_cfg0, 0) == n) {
                tev |= EV_T0;
            }
            if (GEN_CFG0_TSEL(o->gen_cfg0, 1) == n) {
                tev |= EV_T1;
            }
            if (tev) {
                op_generate(o, tev, op_timer_down(c, k));
            }
        }
    }
    for (int k = 0; k < ESP32_MCPWM_OPS; k++) {
        fh_trip(c, k, false, false);
    }
    core_settle(c);
}

/* The device */

static uint32_t clk_div(Esp32McpwmState *s)
{
    return (s->clk_cfg & 0xff) + 1;
}

static uint64_t mcpwm_cycle_at(Esp32McpwmState *s, int64_t ns)
{
    if (ns <= s->base_ns) {
        return s->base_cycle;
    }
    return s->base_cycle +
           clock_ns_to_ticks(s->f160m, ns - s->base_ns) / clk_div(s);
}

/* The first virtual time at which PWM_clk has reached `cycle`. */
static int64_t mcpwm_ns_at(Esp32McpwmState *s, uint64_t cycle)
{
    uint64_t ticks, ns;

    if (cycle <= s->base_cycle) {
        return s->base_ns;
    }
    if (cycle - s->base_cycle > UINT64_MAX / 2 / clk_div(s)) {
        return INT64_MAX / 2;
    }
    ticks = (cycle - s->base_cycle) * clk_div(s);
    ns = clock_ticks_to_ns(s->f160m, ticks);
    while (clock_ns_to_ticks(s->f160m, ns) < ticks) {
        ns++;
    }
    return s->base_ns + MIN(ns, (uint64_t)INT64_MAX / 2);
}

/*
 * The present. While the timer's callback runs it is the moment the timer
 * was due: the callback itself can run a little late, and the edges it
 * drives, and anything they cause in this block (a capture of its own
 * output through the GPIO matrix), belong to the due time. Time never
 * goes back from a moment the block has already acted at.
 */
static int64_t mcpwm_now(Esp32McpwmState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (s->in_timer_cb) {
        now = MIN(s->due_ns, now);
    }
    s->seen_ns = MAX(s->seen_ns, now);
    return s->seen_ns;
}

static uint32_t cap_timer_now(Esp32McpwmState *s)
{
    if (!(s->core.cap_timer_cfg & CAP_TIMER_EN)) {
        return s->cap_anchor;
    }
    return s->cap_anchor + (uint32_t)clock_ns_to_ticks(s->apb, mcpwm_now(s) -
                                                       s->cap_anchor_ns);
}

static void cap_reanchor(Esp32McpwmState *s)
{
    s->cap_anchor = cap_timer_now(s);
    s->cap_anchor_ns = mcpwm_now(s);
}

static void cap_timer_sync(Esp32McpwmState *s)
{
    s->cap_anchor = s->cap_phase;
    s->cap_anchor_ns = mcpwm_now(s);
}

/* Apply the last sync a PWM timer's sync_out gave the capture timer. */
static void mcpwm_cap_sync_hit(Esp32McpwmState *s)
{
    Esp32McpwmCore *c = &s->core;

    if (c->cap_sync_hit) {
        s->cap_anchor = s->cap_phase;
        s->cap_anchor_ns = mcpwm_ns_at(s, c->cap_sync);
        c->cap_sync_hit = false;
    }
}

static void mcpwm_timer_cb(void *opaque);

/*
 * Run the core to the present. The timer's work that is already due is
 * done first, at its due time, so that an access racing a late callback
 * sees the edges driven when they happened.
 */
static void mcpwm_sync(Esp32McpwmState *s)
{
    Esp32McpwmCore *c = &s->core;
    uint64_t target;

    for (int n = 0; n < MAX_LATE_CALLBACKS && !s->in_timer_cb &&
         timer_pending(s->timer) &&
         s->due_ns <= qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL); n++) {
        timer_del(s->timer);
        mcpwm_timer_cb(s);
    }
    target = mcpwm_cycle_at(s, mcpwm_now(s));

    if (target > c->now) {
        core_run(c, target);
    }
    mcpwm_cap_sync_hit(s);
}

static void mcpwm_schedule(Esp32McpwmState *s)
{
    McpwmWatch w;
    McpwmSeen seen;
    int64_t floor_ns = s->last_out_ns + s->min_edge_ns;
    uint64_t when;
    int64_t due;

    if (!clock_is_enabled(s->f160m)) {
        timer_del(s->timer);
        return;
    }
    w.driven = s->out_level;
    w.irq = !s->irq_level;
    w.floor = MAX(mcpwm_cycle_at(s, floor_ns), s->core.now);
    seen = core_look_ahead(&s->core, &w, &when);
    if (seen == SEEN_NOTHING) {
        timer_del(s->timer);
        return;
    }
    due = mcpwm_ns_at(s, when);
    if (seen == SEEN_OUTPUT) {
        due = MAX(due, floor_ns);
    }
    s->due_ns = due;
    timer_mod(s->timer, due);
}

/* Drive the outputs and the interrupt from the core's state. */
static void mcpwm_update(Esp32McpwmState *s)
{
    Esp32McpwmCore *c = &s->core;

    if (s->updating) {
        s->update_pending = true;
        return;
    }
    mcpwm_cap_sync_hit(s);
    s->updating = true;
    do {
        uint8_t irq_level;

        s->update_pending = false;
        for (int n = 0; n < ESP32_MCPWM_OUTS; n++) {
            uint8_t level = op_out(c, n / 2, n % 2);

            /*
             * Record the level before driving it: through the GPIO matrix
             * an output can come back into this device as an input.
             */
            if (level != s->out_level[n]) {
                s->out_level[n] = level;
                s->last_out_ns = mcpwm_now(s);
                qemu_set_irq(s->out[n], level);
            }
        }
        irq_level = !!(c->int_raw & c->int_ena);
        if (irq_level != s->irq_level) {
            s->irq_level = irq_level;
            qemu_set_irq(s->irq, irq_level);
        }
    } while (s->update_pending);
    s->updating = false;
    mcpwm_schedule(s);
}

static void mcpwm_timer_cb(void *opaque)
{
    Esp32McpwmState *s = opaque;

    s->in_timer_cb = true;
    mcpwm_sync(s);
    mcpwm_update(s);
    s->in_timer_cb = false;
}

/* Capture */

static void cap_capture(Esp32McpwmState *s, int n, bool neg)
{
    s->cap_val[n] = cap_timer_now(s);
    s->cap_status = (s->cap_status & ~(1u << n)) | ((uint32_t)neg << n);
    s->core.int_raw |= INT_CAP(n);
}

/*
 * A change of capture channel n's input as seen after the inversion. The
 * prescaler is a toggle divider: with PRESCALE N > 0 its output toggles on
 * every Nth rising edge of its input, so that its rising edges come every
 * 2N input rising edges (ESP-IDF's mcpwm_ll; the TRM's "PRESCALE + 1"
 * describes neither the bypass at 0 nor the halving).
 */
/* [spec:nuos:req:emu.esp32.mcpwm] */
static void cap_input(Esp32McpwmState *s, int n)
{
    uint32_t cfg = s->cap_cfg[n];
    bool x = s->cap_in[n] ^ !!(cfg & CAP_IN_INVERT);
    unsigned pre = CAP_PRESCALE(cfg);
    bool pos;

    if (x == s->cap_x[n]) {
        return;
    }
    s->cap_x[n] = x;
    if (!(cfg & CAP_EN)) {
        return;
    }
    if (pre == 0) {
        s->cap_pre_out[n] = x;
    } else {
        if (!x || ++s->cap_pre_cnt[n] < pre) {
            return;
        }
        s->cap_pre_cnt[n] = 0;
        s->cap_pre_out[n] = !s->cap_pre_out[n];
    }
    pos = s->cap_pre_out[n];
    if (cfg & (pos ? CAP_MODE_POS : CAP_MODE_NEG)) {
        cap_capture(s, n, !pos);
    }
}

/* Register access */

/* Effective GPIO SYNCn level, after its inversion */
static bool sync_level(Esp32McpwmState *s, int n)
{
    return s->sync_in[n] ^ !!(s->core.synci_cfg & SYNCI_INVERT(n));
}

/* A rising edge of GPIO SYNCn. */
static void gpio_sync(Esp32McpwmState *s, int n)
{
    Esp32McpwmCore *c = &s->core;

    for (int j = 0; j < ESP32_MCPWM_TIMERS; j++) {
        if (SYNCI_SEL(c->synci_cfg, j) == SYNCI_SEL_GPIO(n)) {
            timer_sync_in(c, j, 0);
        }
    }
    if ((c->cap_timer_cfg & CAP_SYNCI_EN) &&
        CAP_SYNCI_SEL(c->cap_timer_cfg) == SYNCI_SEL_GPIO(n)) {
        cap_timer_sync(s);
    }
    core_settle(c);
}

static uint64_t op_read(Esp32McpwmCore *c, int k, hwaddr reg)
{
    Esp32McpwmOp *o = &c->op[k];

    switch (reg) {
    case OP_STMP_CFG:
        return o->stmp_cfg |
               ((o->pending & OP_PEND_A) ? STMP_SHDW_FULL(0) : 0) |
               ((o->pending & OP_PEND_B) ? STMP_SHDW_FULL(1) : 0);
    case OP_TSTMP_A:
        return o->tstmp[0];
    case OP_TSTMP_B:
        return o->tstmp[1];
    case OP_GEN_CFG0:
        return o->gen_cfg0;
    case OP_FORCE:
        return o->force;
    case OP_GEN_A:
        return o->gen[0];
    case OP_GEN_B:
        return o->gen[1];
    case OP_DT_CFG:
        return o->dt_cfg;
    case OP_DT_FED:
        return o->dt_fed;
    case OP_DT_RED:
        return o->dt_red;
    case OP_CARRIER:
        return o->carrier;
    case OP_FH_CFG0:
        return o->fh_cfg0;
    case OP_FH_CFG1:
        return o->fh_cfg1;
    case OP_FH_STATUS:
    default:
        return (o->cbc_on ? 1 : 0) | (o->ost_on ? 2 : 0);
    }
}

static void op_write(Esp32McpwmCore *c, int k, hwaddr reg, uint32_t v)
{
    Esp32McpwmOp *o = &c->op[k];
    uint32_t old;

    switch (reg) {
    case OP_STMP_CFG:
        /* Writing 1 to a SHDW_FULL bit drops the pending shadow. */
        if (v & STMP_SHDW_FULL(0)) {
            o->pending &= ~OP_PEND_A;
        }
        if (v & STMP_SHDW_FULL(1)) {
            o->pending &= ~OP_PEND_B;
        }
        o->stmp_cfg = v & 0xff;
        break;
    case OP_TSTMP_A:
    case OP_TSTMP_B: {
        int n = reg == OP_TSTMP_B;

        o->tstmp[n] = v & 0xffff;
        o->pending |= n ? OP_PEND_B : OP_PEND_A;
        break;
    }
    case OP_GEN_CFG0:
        o->gen_cfg0 = v & GEN_CFG0_MASK;
        break;
    case OP_FORCE:
        old = o->force;
        o->force = v & FORCE_MASK;
        o->pending |= OP_PEND_CNTU;
        /*
         * Toggling NCIFORCE is a non-continuous immediate force: the
         * output takes the forced level until the next action.
         */
        for (int ch = 0; ch < 2; ch++) {
            if ((old ^ o->force) & FORCE_NCI(ch)) {
                switch (FORCE_NCI_MODE(o->force, ch)) {
                case ACT_LOW:
                    o->gen_out[ch] = false;
                    break;
                case ACT_HIGH:
                    o->gen_out[ch] = true;
                    break;
                }
            }
        }
        break;
    case OP_GEN_A:
    case OP_GEN_B:
        o->gen[reg == OP_GEN_B] = v & GEN_MASK;
        o->pending |= OP_PEND_GEN;
        break;
    case OP_DT_CFG:
        o->dt_cfg = v & DT_CFG_MASK;
        break;
    case OP_DT_FED:
        o->dt_fed = v & DT_DELAY_MASK;
        o->pending |= OP_PEND_FED;
        break;
    case OP_DT_RED:
        o->dt_red = v & DT_DELAY_MASK;
        o->pending |= OP_PEND_RED;
        break;
    case OP_CARRIER:
        o->carrier = v & CARRIER_MASK;
        break;
    case OP_FH_CFG0:
        o->fh_cfg0 = v & FH_CFG0_MASK;
        fh_trip(c, k, false, false);
        break;
    case OP_FH_CFG1:
        old = o->fh_cfg1;
        o->fh_cfg1 = v & FH_CFG1_MASK;
        /* A rising edge of CLR_OST ends a one-shot trip. */
        if (!(old & FH_CLR_OST) && (o->fh_cfg1 & FH_CLR_OST)) {
            o->ost_on = false;
        }
        fh_trip(c, k, (old ^ o->fh_cfg1) & FH_FORCE_OST,
                (old ^ o->fh_cfg1) & FH_FORCE_CBC);
        break;
    case OP_FH_STATUS:
        break;
    }
    op_immediate(c, k);
    op_apply_force(o);
}

static void timer_write(Esp32McpwmState *s, int i, hwaddr reg, uint32_t v)
{
    Esp32McpwmCore *c = &s->core;
    Esp32McpwmTimer *t = &c->timer[i];
    uint32_t old;
    unsigned start;

    switch (reg) {
    case TIMER_CFG0:
        t->cfg0 = v & CFG0_MASK;
        t->period_pending = true;
        if (CFG0_UPMETHOD(t->cfg0) == 0) {
            timer_transfer(c, i, false);
        }
        break;
    case TIMER_CFG1:
        t->cfg1 = v & CFG1_MASK;
        if (timer_mode(t) == MOD_UP) {
            t->down = false;
        } else if (timer_mode(t) == MOD_DOWN) {
            t->down = true;
        }
        start = CFG1_START(t->cfg1);
        if (start > START_ONCE_TEP) {
            qemu_log_mask(LOG_GUEST_ERROR, "esp32-mcpwm: timer %d: reserved "
                          "TIMER_START %u\n", i, start);
        } else if (start >= START_RUN && !t->running) {
            /* The prescale takes effect as the timer starts. */
            t->running = true;
            t->prescale = CFG0_PRESCALE(t->cfg0);
            t->pre_cnt = 0;
        }
        break;
    case TIMER_SYNC:
        old = t->sync;
        t->sync = v & SYNC_MASK;
        /*
         * Toggling SYNC_SW is a sync of this timer, which always also
         * goes out on its sync_out.
         */
        if ((old ^ t->sync) & SYNC_SW) {
            if (t->sync & SYNC_SYNCI_EN) {
                timer_take_sync(c, i);
            }
            timer_sync_out(c, i, 1u << i);
        }
        break;
    case TIMER_STATUS:
        break;
    }
}

static void update_cfg_write(Esp32McpwmCore *c, uint32_t v)
{
    uint32_t old = c->update_cfg;

    c->update_cfg = v & UPDATE_CFG_MASK;
    if ((old ^ c->update_cfg) & UP_GLOBAL_FORCE) {
        for (int i = 0; i < ESP32_MCPWM_TIMERS; i++) {
            timer_transfer(c, i, true);
        }
        for (int k = 0; k < ESP32_MCPWM_OPS; k++) {
            op_transfer(&c->op[k], OP_PEND_ALL);
        }
    }
    for (int k = 0; k < ESP32_MCPWM_OPS; k++) {
        if ((old ^ c->update_cfg) & UP_OP_FORCE(k)) {
            op_transfer(&c->op[k], OP_PEND_ALL);
        }
    }
    for (int i = 0; i < ESP32_MCPWM_TIMERS; i++) {
        if (CFG0_UPMETHOD(c->timer[i].cfg0) == 0) {
            timer_transfer(c, i, false);
        }
    }
    for (int k = 0; k < ESP32_MCPWM_OPS; k++) {
        op_immediate(c, k);
    }
}

/* [spec:nuos:req:emu.esp32.mcpwm] */
static uint64_t esp32_mcpwm_read(void *opaque, hwaddr addr, unsigned size)
{
    Esp32McpwmState *s = opaque;
    Esp32McpwmCore *c = &s->core;
    uint64_t v = 0;

    mcpwm_sync(s);
    if (addr >= A_TIMER_BASE && addr < A_SYNCI_CFG) {
        Esp32McpwmTimer *t = &c->timer[(addr - A_TIMER_BASE) / TIMER_STRIDE];

        switch ((addr - A_TIMER_BASE) % TIMER_STRIDE) {
        case TIMER_CFG0:
            v = t->cfg0;
            break;
        case TIMER_CFG1:
            v = t->cfg1;
            break;
        case TIMER_SYNC:
            v = t->sync;
            break;
        case TIMER_STATUS:
            v = t->value | (t->down ? 1u << 16 : 0);
            break;
        }
    } else if (addr >= A_OP_BASE && addr < A_FAULT_DETECT) {
        v = op_read(c, (addr - A_OP_BASE) / OP_STRIDE,
                    (addr - A_OP_BASE) % OP_STRIDE);
    } else if (addr >= A_CAP_CH_CFG && addr < A_CAP_CH) {
        v = s->cap_cfg[(addr - A_CAP_CH_CFG) / 4];
    } else if (addr >= A_CAP_CH && addr < A_CAP_STATUS) {
        v = s->cap_val[(addr - A_CAP_CH) / 4];
    } else {
        switch (addr) {
        case A_CLK_CFG:
            v = s->clk_cfg;
            break;
        case A_SYNCI_CFG:
            v = c->synci_cfg;
            break;
        case A_TIMERSEL:
            v = c->timersel;
            break;
        case A_FAULT_DETECT:
            v = c->fault_detect;
            for (int n = 0; n < ESP32_MCPWM_FAULTS; n++) {
                v |= (uint32_t)c->fault_ev[n] << (6 + n);
            }
            break;
        case A_CAP_TIMER_CFG:
            v = c->cap_timer_cfg;
            break;
        case A_CAP_TIMER_PHASE:
            v = s->cap_phase;
            break;
        case A_CAP_STATUS:
            v = s->cap_status;
            break;
        case A_UPDATE_CFG:
            v = c->update_cfg;
            break;
        case A_INT_ENA:
            v = c->int_ena;
            break;
        case A_INT_RAW:
            v = c->int_raw;
            break;
        case A_INT_ST:
            v = c->int_raw & c->int_ena;
            break;
        case A_INT_CLR:
            break;
        case A_CLK:
            v = s->clk_reg;
            break;
        case A_VERSION:
            v = s->version;
            break;
        default:
            qemu_log_mask(LOG_GUEST_ERROR, "esp32-mcpwm: read of bad offset "
                          "0x%" HWADDR_PRIx "\n", addr);
            break;
        }
    }
    mcpwm_update(s);
    return v;
}

/* [spec:nuos:req:emu.esp32.mcpwm] */
static void esp32_mcpwm_write(void *opaque, hwaddr addr, uint64_t value,
                              unsigned size)
{
    Esp32McpwmState *s = opaque;
    Esp32McpwmCore *c = &s->core;
    uint32_t v = value;

    mcpwm_sync(s);
    if (addr >= A_TIMER_BASE && addr < A_SYNCI_CFG) {
        timer_write(s, (addr - A_TIMER_BASE) / TIMER_STRIDE,
                    (addr - A_TIMER_BASE) % TIMER_STRIDE, v);
    } else if (addr >= A_OP_BASE && addr < A_FAULT_DETECT) {
        op_write(c, (addr - A_OP_BASE) / OP_STRIDE,
                 (addr - A_OP_BASE) % OP_STRIDE, v);
    } else if (addr >= A_CAP_CH_CFG && addr < A_CAP_CH) {
        int n = (addr - A_CAP_CH_CFG) / 4;
        uint32_t old = s->cap_cfg[n];

        s->cap_cfg[n] = v & CAP_CH_CFG_MASK;
        if ((s->cap_cfg[n] & CAP_EN) && !(old & CAP_EN)) {
            /* The prescaler starts afresh. */
            s->cap_pre_cnt[n] = 0;
            s->cap_pre_out[n] = CAP_PRESCALE(s->cap_cfg[n]) ? false :
                                s->cap_x[n];
        }
        cap_input(s, n);
        if (v & CAP_SW) {
            cap_capture(s, n, s->cap_status & (1u << n));
        }
    } else if (addr >= A_CAP_CH && addr < A_CAP_STATUS) {
        /* Read-only */
    } else {
        switch (addr) {
        case A_CLK_CFG:
            /* PWM_clk restarts its count at the new rate. */
            s->base_ns = mcpwm_now(s);
            s->base_cycle = c->now;
            s->clk_cfg = v & 0xff;
            break;
        case A_SYNCI_CFG: {
            bool before[ESP32_MCPWM_SYNCS];

            for (int n = 0; n < ESP32_MCPWM_SYNCS; n++) {
                before[n] = sync_level(s, n);
            }
            c->synci_cfg = v & SYNCI_CFG_MASK;
            for (int n = 0; n < ESP32_MCPWM_SYNCS; n++) {
                if (!before[n] && sync_level(s, n)) {
                    gpio_sync(s, n);
                }
            }
            break;
        }
        case A_TIMERSEL:
            c->timersel = v & TIMERSEL_MASK;
            for (int k = 0; k < ESP32_MCPWM_OPS; k++) {
                if (op_timer(c, k) < 0) {
                    qemu_log_mask(LOG_GUEST_ERROR, "esp32-mcpwm: operator %d "
                                  "selects reserved timer 3\n", k);
                }
            }
            break;
        case A_FAULT_DETECT:
            c->fault_detect = v & FAULT_DETECT_MASK;
            core_faults(c);
            break;
        case A_CAP_TIMER_CFG:
            cap_reanchor(s);
            c->cap_timer_cfg = v & CAP_TIMER_CFG_MASK;
            if ((v & CAP_SYNC_SW) && (c->cap_timer_cfg & CAP_SYNCI_EN)) {
                cap_timer_sync(s);
            }
            break;
        case A_CAP_TIMER_PHASE:
            s->cap_phase = v;
            break;
        case A_UPDATE_CFG:
            update_cfg_write(c, v);
            break;
        case A_INT_ENA:
            c->int_ena = v & INT_MASK;
            break;
        case A_INT_RAW:
        case A_INT_CLR:
            c->int_raw &= ~v;
            break;
        case A_CLK:
            s->clk_reg = v & 1;
            break;
        case A_VERSION:
            s->version = v & VERSION_MASK;
            break;
        case A_CAP_STATUS:
        case A_INT_ST:
            break;
        default:
            qemu_log_mask(LOG_GUEST_ERROR, "esp32-mcpwm: write to bad offset "
                          "0x%" HWADDR_PRIx "\n", addr);
            break;
        }
    }
    core_settle(c);
    mcpwm_update(s);
}

static const MemoryRegionOps esp32_mcpwm_ops = {
    .read = esp32_mcpwm_read,
    .write = esp32_mcpwm_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/* Inputs from the GPIO matrix */

static void esp32_mcpwm_sync_in(void *opaque, int n, int level)
{
    Esp32McpwmState *s = opaque;
    bool before;

    mcpwm_sync(s);
    before = sync_level(s, n);
    s->sync_in[n] = level != 0;
    if (!before && sync_level(s, n)) {
        gpio_sync(s, n);
    }
    mcpwm_update(s);
}

static void esp32_mcpwm_fault_in(void *opaque, int n, int level)
{
    Esp32McpwmState *s = opaque;

    mcpwm_sync(s);
    s->core.fault_in[n] = level != 0;
    core_faults(&s->core);
    mcpwm_update(s);
}

static void esp32_mcpwm_cap_in(void *opaque, int n, int level)
{
    Esp32McpwmState *s = opaque;

    mcpwm_sync(s);
    s->cap_in[n] = level != 0;
    cap_input(s, n);
    mcpwm_update(s);
}

/*
 * A change of PLL_F160M_CLK: run to the change at the old rate, then count
 * from there at the new one.
 */
static void esp32_mcpwm_f160m_update(void *opaque, ClockEvent event)
{
    Esp32McpwmState *s = opaque;

    if (event == ClockPreUpdate) {
        mcpwm_sync(s);
        s->base_ns = mcpwm_now(s);
        s->base_cycle = s->core.now;
    } else {
        mcpwm_update(s);
    }
}

static void esp32_mcpwm_apb_update(void *opaque, ClockEvent event)
{
    Esp32McpwmState *s = opaque;

    if (event == ClockPreUpdate) {
        cap_reanchor(s);
    }
}

static void esp32_mcpwm_reset_hold(Object *obj, ResetType type)
{
    Esp32McpwmState *s = ESP32_MCPWM(obj);
    Esp32McpwmCore *c = &s->core;
    bool fault_in[ESP32_MCPWM_FAULTS];

    timer_del(s->timer);
    /* Input levels come from outside the block and survive its reset. */
    memcpy(fault_in, c->fault_in, sizeof(fault_in));
    memset(c, 0, sizeof(*c));
    memcpy(c->fault_in, fault_in, sizeof(fault_in));
    for (int i = 0; i < ESP32_MCPWM_TIMERS; i++) {
        Esp32McpwmTimer *t = &c->timer[i];

        t->cfg0 = CFG0_RESET;
        t->period = CFG0_PERIOD(CFG0_RESET);
    }
    for (int k = 0; k < ESP32_MCPWM_OPS; k++) {
        Esp32McpwmOp *o = &c->op[k];

        o->force = FORCE_RESET;
        o->dt_cfg = DT_CFG_RESET;
        o->red.due = NO_EVENT;
        o->fed.due = NO_EVENT;
    }
    c->update_cfg = UPDATE_CFG_RESET;

    s->clk_cfg = 0;
    s->clk_reg = 0;
    s->version = VERSION_RESET;
    s->seen_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->base_ns = mcpwm_now(s);
    s->base_cycle = 0;
    s->cap_phase = 0;
    s->cap_anchor = 0;
    s->cap_anchor_ns = s->base_ns;
    memset(s->cap_cfg, 0, sizeof(s->cap_cfg));
    memset(s->cap_val, 0, sizeof(s->cap_val));
    s->cap_status = 0;
    memset(s->cap_pre_cnt, 0, sizeof(s->cap_pre_cnt));
    memset(s->cap_pre_out, 0, sizeof(s->cap_pre_out));
    for (int n = 0; n < ESP32_MCPWM_CAPS; n++) {
        s->cap_x[n] = s->cap_in[n];
    }
    s->last_out_ns = INT64_MIN / 2;
    /* Drive every line afresh. */
    memset(s->out_level, 0xff, sizeof(s->out_level));
    s->irq_level = 0xff;
}

static void esp32_mcpwm_reset_exit(Object *obj, ResetType type)
{
    Esp32McpwmState *s = ESP32_MCPWM(obj);

    mcpwm_update(s);
}

static void esp32_mcpwm_init(Object *obj)
{
    Esp32McpwmState *s = ESP32_MCPWM(obj);
    DeviceState *dev = DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32_mcpwm_ops, s,
                          TYPE_ESP32_MCPWM, ESP32_MCPWM_REGION_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    qdev_init_gpio_out_named(dev, s->out, ESP32_MCPWM_OUT, ESP32_MCPWM_OUTS);
    qdev_init_gpio_in_named(dev, esp32_mcpwm_sync_in, ESP32_MCPWM_SYNC_IN,
                            ESP32_MCPWM_SYNCS);
    qdev_init_gpio_in_named(dev, esp32_mcpwm_fault_in, ESP32_MCPWM_FAULT_IN,
                            ESP32_MCPWM_FAULTS);
    qdev_init_gpio_in_named(dev, esp32_mcpwm_cap_in, ESP32_MCPWM_CAP_IN,
                            ESP32_MCPWM_CAPS);
    s->f160m = qdev_init_clock_in(dev, "f160m", esp32_mcpwm_f160m_update, s,
                                  ClockPreUpdate | ClockUpdate);
    s->apb = qdev_init_clock_in(dev, "apb", esp32_mcpwm_apb_update, s,
                                ClockPreUpdate);
}

static void esp32_mcpwm_realize(DeviceState *dev, Error **errp)
{
    Esp32McpwmState *s = ESP32_MCPWM(dev);

    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, mcpwm_timer_cb, s);
}

static const Property esp32_mcpwm_properties[] = {
    DEFINE_PROP_UINT32("min-edge-ns", Esp32McpwmState, min_edge_ns, 1000),
};

static const VMStateDescription vmstate_esp32_mcpwm_timer = {
    .name = TYPE_ESP32_MCPWM "-timer",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(cfg0, Esp32McpwmTimer),
        VMSTATE_UINT32(cfg1, Esp32McpwmTimer),
        VMSTATE_UINT32(sync, Esp32McpwmTimer),
        VMSTATE_UINT32(period, Esp32McpwmTimer),
        VMSTATE_UINT32(prescale, Esp32McpwmTimer),
        VMSTATE_UINT32(value, Esp32McpwmTimer),
        VMSTATE_UINT32(pre_cnt, Esp32McpwmTimer),
        VMSTATE_BOOL(down, Esp32McpwmTimer),
        VMSTATE_BOOL(running, Esp32McpwmTimer),
        VMSTATE_BOOL(period_pending, Esp32McpwmTimer),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_esp32_mcpwm_delay = {
    .name = TYPE_ESP32_MCPWM "-delay",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT64(due, Esp32McpwmDelay),
        VMSTATE_BOOL(in, Esp32McpwmDelay),
        VMSTATE_BOOL(out, Esp32McpwmDelay),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_esp32_mcpwm_op = {
    .name = TYPE_ESP32_MCPWM "-op",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(stmp_cfg, Esp32McpwmOp),
        VMSTATE_UINT32_ARRAY(tstmp, Esp32McpwmOp, 2),
        VMSTATE_UINT32_ARRAY(cmp, Esp32McpwmOp, 2),
        VMSTATE_UINT32(gen_cfg0, Esp32McpwmOp),
        VMSTATE_UINT32(force, Esp32McpwmOp),
        VMSTATE_UINT32_ARRAY(gen, Esp32McpwmOp, 2),
        VMSTATE_UINT32_ARRAY(gen_act, Esp32McpwmOp, 2),
        VMSTATE_UINT32(dt_cfg, Esp32McpwmOp),
        VMSTATE_UINT32(dt_fed, Esp32McpwmOp),
        VMSTATE_UINT32(dt_red, Esp32McpwmOp),
        VMSTATE_UINT32(fed_act, Esp32McpwmOp),
        VMSTATE_UINT32(red_act, Esp32McpwmOp),
        VMSTATE_UINT32(carrier, Esp32McpwmOp),
        VMSTATE_UINT32(fh_cfg0, Esp32McpwmOp),
        VMSTATE_UINT32(fh_cfg1, Esp32McpwmOp),
        VMSTATE_UINT32(cntu_act, Esp32McpwmOp),
        VMSTATE_UINT32(pending, Esp32McpwmOp),
        VMSTATE_BOOL_ARRAY(gen_out, Esp32McpwmOp, 2),
        VMSTATE_STRUCT(red, Esp32McpwmOp, 1, vmstate_esp32_mcpwm_delay,
                       Esp32McpwmDelay),
        VMSTATE_STRUCT(fed, Esp32McpwmOp, 1, vmstate_esp32_mcpwm_delay,
                       Esp32McpwmDelay),
        VMSTATE_BOOL_ARRAY(car_in, Esp32McpwmOp, 2),
        VMSTATE_UINT64_ARRAY(car_rise, Esp32McpwmOp, 2),
        VMSTATE_BOOL(cbc_on, Esp32McpwmOp),
        VMSTATE_BOOL(ost_on, Esp32McpwmOp),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_esp32_mcpwm_core = {
    .name = TYPE_ESP32_MCPWM "-core",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT64(now, Esp32McpwmCore),
        VMSTATE_STRUCT_ARRAY(timer, Esp32McpwmCore, ESP32_MCPWM_TIMERS, 1,
                             vmstate_esp32_mcpwm_timer, Esp32McpwmTimer),
        VMSTATE_STRUCT_ARRAY(op, Esp32McpwmCore, ESP32_MCPWM_OPS, 1,
                             vmstate_esp32_mcpwm_op, Esp32McpwmOp),
        VMSTATE_UINT32(synci_cfg, Esp32McpwmCore),
        VMSTATE_UINT32(timersel, Esp32McpwmCore),
        VMSTATE_UINT32(fault_detect, Esp32McpwmCore),
        VMSTATE_UINT32(update_cfg, Esp32McpwmCore),
        VMSTATE_UINT32(cap_timer_cfg, Esp32McpwmCore),
        VMSTATE_UINT32(int_ena, Esp32McpwmCore),
        VMSTATE_UINT32(int_raw, Esp32McpwmCore),
        VMSTATE_BOOL_ARRAY(fault_in, Esp32McpwmCore, ESP32_MCPWM_FAULTS),
        VMSTATE_BOOL_ARRAY(fault_ev, Esp32McpwmCore, ESP32_MCPWM_FAULTS),
        VMSTATE_BOOL(cap_sync_hit, Esp32McpwmCore),
        VMSTATE_UINT64(cap_sync, Esp32McpwmCore),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_esp32_mcpwm = {
    .name = TYPE_ESP32_MCPWM,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_CLOCK(f160m, Esp32McpwmState),
        VMSTATE_CLOCK(apb, Esp32McpwmState),
        VMSTATE_TIMER_PTR(timer, Esp32McpwmState),
        VMSTATE_STRUCT(core, Esp32McpwmState, 1, vmstate_esp32_mcpwm_core,
                       Esp32McpwmCore),
        VMSTATE_UINT32(clk_cfg, Esp32McpwmState),
        VMSTATE_UINT32(clk_reg, Esp32McpwmState),
        VMSTATE_UINT32(version, Esp32McpwmState),
        VMSTATE_INT64(base_ns, Esp32McpwmState),
        VMSTATE_UINT64(base_cycle, Esp32McpwmState),
        VMSTATE_BOOL_ARRAY(sync_in, Esp32McpwmState, ESP32_MCPWM_SYNCS),
        VMSTATE_UINT32(cap_phase, Esp32McpwmState),
        VMSTATE_UINT32(cap_anchor, Esp32McpwmState),
        VMSTATE_INT64(cap_anchor_ns, Esp32McpwmState),
        VMSTATE_UINT32_ARRAY(cap_cfg, Esp32McpwmState, ESP32_MCPWM_CAPS),
        VMSTATE_UINT32_ARRAY(cap_val, Esp32McpwmState, ESP32_MCPWM_CAPS),
        VMSTATE_UINT32(cap_status, Esp32McpwmState),
        VMSTATE_BOOL_ARRAY(cap_in, Esp32McpwmState, ESP32_MCPWM_CAPS),
        VMSTATE_BOOL_ARRAY(cap_x, Esp32McpwmState, ESP32_MCPWM_CAPS),
        VMSTATE_BOOL_ARRAY(cap_pre_out, Esp32McpwmState, ESP32_MCPWM_CAPS),
        VMSTATE_UINT32_ARRAY(cap_pre_cnt, Esp32McpwmState, ESP32_MCPWM_CAPS),
        VMSTATE_UINT8_ARRAY(out_level, Esp32McpwmState, ESP32_MCPWM_OUTS),
        VMSTATE_UINT8(irq_level, Esp32McpwmState),
        VMSTATE_INT64(last_out_ns, Esp32McpwmState),
        VMSTATE_INT64(due_ns, Esp32McpwmState),
        VMSTATE_INT64(seen_ns, Esp32McpwmState),
        VMSTATE_END_OF_LIST()
    },
};

static void esp32_mcpwm_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = esp32_mcpwm_realize;
    dc->vmsd = &vmstate_esp32_mcpwm;
    rc->phases.hold = esp32_mcpwm_reset_hold;
    rc->phases.exit = esp32_mcpwm_reset_exit;
    device_class_set_props(dc, esp32_mcpwm_properties);
}

/* [spec:nuos:req:emu.esp32.mcpwm] */
static const TypeInfo esp32_mcpwm_info = {
    .name          = TYPE_ESP32_MCPWM,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32McpwmState),
    .instance_init = esp32_mcpwm_init,
    .class_init    = esp32_mcpwm_class_init,
};

static void esp32_mcpwm_register_types(void)
{
    type_register_static(&esp32_mcpwm_info);
}

type_init(esp32_mcpwm_register_types)
