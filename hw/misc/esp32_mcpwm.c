/*
 * ESP32 and ESP32-P4 motor control PWM (MCPWM)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32 TRM, "Motor Control PWM (MCPWM)", and ESP-IDF's
 * soc/mcpwm_reg.h, soc/mcpwm_struct.h and hal/mcpwm_ll.h; for the
 * ESP32-P4, its TRM (v3.x silicon) chapter 59 and ESP-IDF's v3
 * mcpwm_reg.h and mcpwm_struct.h, and hal/mcpwm_ll.h.
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
 * of the state ahead. Every edge of an output that reaches a pad something
 * watches is driven at its own cycle, however close to others: a capture,
 * a pulse counter or an ETM event that an edge reaches sees each, so the
 * host's work grows with the edges' rate, and on the host's clock a host
 * that cannot keep up skips ahead (below). An output that reaches no such
 * pad has no edge anyone sees: it is driven at its level of the moment
 * whenever the block is brought up to date (mcpwm_routes_run()), and
 * costs nothing.
 *
 * The capture timer counts APB_CLK and is computed in closed form; capture
 * inputs, fault inputs and GPIO sync inputs act at the moment their level
 * changes. The input synchronisers in the GPIO matrix and the fault and
 * capture sampling are not modelled: an input takes effect in the PWM_clk
 * or APB_CLK cycle in which it changes.
 *
 * Times are those of the signal lines (hw/gpio/esp32_line.h). Each thing
 * the block does (a register access, an input change, an ETM task, a
 * clock change, a sync of the pads, the timer's callback) acts at one
 * moment, read once as it begins: without icount the virtual clock moves
 * on while the block works, and a block reading it twice would run its
 * counters to one moment and drive the edges it found at another. An
 * output edge is stamped with the moment it is due, however late the
 * timer's callback runs: whatever brings the block to a moment first does
 * all that fell due before it, edge by edge, each at its own time. Only
 * when that work is more than 10 ms behind (a host slower than the edges
 * it is to drive, which would never catch up) does the block skip ahead,
 * the edges in between lost; each output it then drives is stamped with
 * the moment it last changed, so that no edge carries a time other than
 * its own. An input acts at the time its change is stamped with,
 * so that a capture of an edge another block drives through the GPIO
 * matrix, or this block's own output coming back, reads the capture timer
 * at that edge. Whenever software samples the pads the outputs are
 * brought to their exact levels; a register access first brings every
 * other source of line changes up to the moment it began, and acts there.
 *
 * The ESP32-P4's block (TRM chapter 59) is this one with:
 *   - Its clock, MCPWMn_CLK, from HP_SYS_CLKRST's source mux and divider:
 *     CLK_PRESCALE divides it into PWM_clk (TRM 59.5, "PWM_clk = 6.25 ns
 *     * (PWM_CLK_PRESCALE + 1)" for the 160 MHz PLL_F160M_CLK), and the
 *     capture timer counts it ("the same as MCPWM's core clock", as
 *     ESP-IDF's SOC_MCPWM_CAPTURE_CLK_FROM_GROUP has it), not APB_CLK.
 *   - Event comparators: OPn_TSTMP_E1 and E2, without shadows, give the
 *     ETM events OPn_TEE1 and OPn_TEE2 as the count of operator n's timer
 *     equals them, as TSTMP_A gives TEA; they act on nothing else.
 *   - ETM events (EVT_EN, EVT_EN2): each of the 30 interrupt conditions,
 *     as an event of its own each time it happens, and TEE1/TEE2, pulsed
 *     at the time they happen. A timer fires for each enabled event, as
 *     for an interrupt.
 *   - ETM tasks (TASK_EN), as TRM table 59.3-8 gives them: CMPRn_A_UP and
 *     _B_UP transfer operator n's A or B shadow, if one is pending, as a
 *     forced update does; GEN_STOP stops every running timer where it
 *     stands, each raising its stop interrupt and event, the outputs
 *     holding their levels, until software starts it again; TIMERn_SYNC
 *     is timer n's software sync; TIMERn_PERIOD_UP transfers its period
 *     shadow; TZn_OST trips operator n's one-shot fault handling and
 *     CLRn_OST clears it, with no TZ_SW_OST enable needed; CAPn is
 *     capture channel n's software capture.
 *   - CLK and VERSION at 0x144 and 0x148, VERSION resetting to 0x2212290.
 */

#include "qemu/osdep.h"
#include "qemu/host-utils.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/gpio/esp32_line.h"
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
/* The ESP32-P4's: the ETM's enables and event comparators, CLK, VERSION */
#define A_P4_EVT_EN             0x120
#define A_P4_TASK_EN            0x124
#define A_P4_EVT_EN2            0x128
#define A_P4_OP_TSTMP_E         0x12c
#define A_P4_OP_TSTMP_E_END     0x144
#define A_P4_CLK                0x144
#define A_P4_VERSION            0x148

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
#define P4_VERSION_RESET        0x02212290u
#define EVT_EN_MASK             0x3fffffffu
#define EVT_EN2_MASK            0x3fu
#define TASK_EN_MASK            0x3fffffu
#define TSTMP_E_MASK            0xffffu

/*
 * The ESP32-P4's ETM events, as the lines of "etm-event": EVT_EN's bits,
 * then EVT_EN2's.
 */
#define ETM_TIMER_STOP(i)       (i)
#define ETM_TIMER_TEZ(i)        (3 + (i))
#define ETM_TIMER_TEP(i)        (6 + (i))
#define ETM_OP_TEA(k)           (9 + (k))
#define ETM_OP_TEB(k)           (12 + (k))
#define ETM_FAULT(n)            (15 + (n))
#define ETM_FAULT_CLR(n)        (18 + (n))
#define ETM_FH_CBC(k)           (21 + (k))
#define ETM_FH_OST(k)           (24 + (k))
#define ETM_CAP(n)              (27 + (n))
#define ETM_OP_TEE(k, e)        (30 + 3 * (e) + (k))

/* Its ETM tasks, as the lines of "etm-task" and TASK_EN's bits. */
#define TASK_CMPR_A_UP(k)       (k)
#define TASK_CMPR_B_UP(k)       (3 + (k))
#define TASK_GEN_STOP           6
#define TASK_TIMER_SYNC(i)      (7 + (i))
#define TASK_TIMER_PERIOD_UP(i) (10 + (i))
#define TASK_TZ_OST(k)          (13 + (k))
#define TASK_CLR_OST(k)         (16 + (k))
#define TASK_CAP(n)             (19 + (n))

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
/*
 * How far behind virtual time the timer's work may fall before the block
 * skips ahead (mcpwm_skip()) rather than drive every edge on the way.
 */
#define CATCHUP_NS              (10 * 1000 * 1000)
/* The PWM_clk cycles a skip first looks back over for the last changes */
#define SKIP_WINDOW             1024
/* The most events a look-ahead run takes before giving up for now. */
#define WATCH_MAX_EVENTS        65536

/* What a look-ahead run is looking for. */
typedef struct McpwmWatch {
    const uint8_t *driven;      /* output levels as driven */
    uint8_t outs;               /* the outputs whose edges count */
    bool irq;                   /* whether a rising interrupt counts */
    uint64_t etm;               /* the ETM events that count */
} McpwmWatch;

typedef enum McpwmSeen {
    SEEN_NOTHING,               /* nothing visible will ever happen */
    SEEN_HORIZON,               /* nothing visible yet; look again later */
    SEEN_OUTPUT,
    SEEN_IRQ,                   /* a rising interrupt or an ETM event */
} McpwmSeen;

/*
 * The ESP32-P4's ETM event etm_line has happened at the current cycle.
 * The groups of a run each run from its start, so the latest event of
 * any group is the one whose cycle is kept.
 */
static void core_etm_fire(Esp32McpwmCore *c, int etm_line)
{
    c->etm_fired |= 1ull << etm_line;
    c->etm_cycle = MAX(c->etm_cycle, c->now);
}

/*
 * Interrupt condition int_bit, which is also the ESP32-P4's ETM event
 * etm_line, has happened at the current cycle.
 */
static void core_raise(Esp32McpwmCore *c, uint32_t int_bit, int etm_line)
{
    c->int_raw |= int_bit;
    core_etm_fire(c, etm_line);
}

static uint64_t core_etm_enabled(const Esp32McpwmCore *c)
{
    return c->evt_en | ((uint64_t)c->evt_en2 << ETM_OP_TEE(0, 0));
}

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

/*
 * Whether operator k's event comparator e (E1, E2), on timer i, produces
 * its ETM event, TEE1 or TEE2. It acts on nothing else, so while the
 * event is disabled it is not looked for.
 */
static bool op_tee_live(const Esp32McpwmCore *c, int k, int i, int e)
{
    return (c->evt_en2 & (1u << (ETM_OP_TEE(k, e) - ETM_OP_TEE(0, 0)))) &&
           op_cmp_live(c, i, c->op[k].tstmp_e[e]);
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
        core_raise(c, INT_FH_OST(k), ETM_FH_OST(k));
    }
    if (cbc && !o->cbc_on) {
        o->cbc_on = true;
        core_raise(c, INT_FH_CBC(k), ETM_FH_CBC(k));
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
        core_raise(c, INT_OP_TEA(k), ETM_OP_TEA(k));
    }
    if (t->value == o->cmp[1] && op_cmp_live(c, i, o->cmp[1])) {
        ev |= EV_TEB;
        core_raise(c, INT_OP_TEB(k), ETM_OP_TEB(k));
    }
    for (int e = 0; e < 2; e++) {
        if (op_tee_live(c, k, i, e) && t->value == o->tstmp_e[e]) {
            core_etm_fire(c, ETM_OP_TEE(k, e));
        }
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
        core_raise(c, INT_TIMER_TEZ(i), ETM_TIMER_TEZ(i));
    }
    if (t->value == t->period) {
        ev |= EV_TEP;
        core_raise(c, INT_TIMER_TEP(i), ETM_TIMER_TEP(i));
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
        core_raise(c, INT_TIMER_STOP(i), ETM_TIMER_STOP(i));
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

/*
 * The next cycle after now at which a carrier can change one of the
 * outputs `outs` (bit n for output n).
 */
static uint64_t core_next_carrier(const Esp32McpwmCore *c, unsigned group,
                                  uint8_t outs)
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

            if (!(outs & BIT(2 * k + ch)) || !o->car_in[ch] ||
                act == ACT_LOW || act == ACT_HIGH) {
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
            if (op_tee_live(c, n, i, cmp)) {
                k = MIN(k, timer_ticks_to(t, o->tstmp_e[cmp]));
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
    n->etm_fired = 0;
    n->etm_cycle = 0;
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
                                const uint8_t *driven, uint8_t outs,
                                unsigned group)
{
    for (int n = 0; n < ESP32_MCPWM_OUTS; n++) {
        if ((outs & BIT(n)) && op_in(c, n / 2, group) &&
            op_out(c, n / 2, n % 2) != driven[n]) {
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
    unsigned events = 0, since = 0, window = SNAP_WINDOW, checks = 0;
    bool have_snap = false;
    uint32_t raw0 = c->int_raw & c->int_ena;

    while (c->now < target) {
        uint64_t next = core_next(c, tdue, group);

        if (w) {
            uint64_t check = core_next_carrier(c, group, w->outs);

            if (check < next && check <= target) {
                core_advance(c, check, NULL, group);
                if (core_outputs_differ(c, w->driven, w->outs, group)) {
                    return SEEN_OUTPUT;
                }
                if (++checks >= WATCH_MAX_EVENTS) {
                    return SEEN_HORIZON;
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
            if ((w->irq && (c->int_raw & c->int_ena & ~raw0)) ||
                (c->etm_fired & w->etm)) {
                return SEEN_IRQ;
            }
            if (core_outputs_differ(c, w->driven, w->outs, group)) {
                return SEEN_OUTPUT;
            }
            if (events >= WATCH_MAX_EVENTS) {
                return SEEN_HORIZON;
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
 * Whether nothing visible can change until the core is changed from
 * outside: no timer counts, no dead time or carrier is under way, and the
 * outputs are as driven.
 */
static bool core_idle(const Esp32McpwmCore *c, const McpwmWatch *w)
{
    for (int i = 0; i < ESP32_MCPWM_TIMERS; i++) {
        if (timer_counting(&c->timer[i])) {
            return false;
        }
    }
    for (int k = 0; k < ESP32_MCPWM_OPS; k++) {
        if (c->op[k].red.due != NO_EVENT || c->op[k].fed.due != NO_EVENT) {
            return false;
        }
    }
    return core_next_carrier(c, ALL_TIMERS, w->outs) == NO_EVENT &&
           !core_outputs_differ(c, w->driven, w->outs, ALL_TIMERS);
}

/*
 * Look ahead from the core's state for the first visible change: the
 * earliest of each group's. Returns what it is, with *when its cycle.
 */
static McpwmSeen core_look_ahead(const Esp32McpwmCore *c, const McpwmWatch *w,
                                 uint64_t *when)
{
    Esp32McpwmCore ahead;
    unsigned groups[ESP32_MCPWM_TIMERS];
    int n;
    McpwmSeen best = SEEN_NOTHING;

    *when = NO_EVENT;
    if (core_idle(c, w)) {
        return SEEN_NOTHING;
    }
    n = core_groups(c, groups);
    for (int g = 0; g < n; g++) {
        McpwmSeen seen;

        memcpy(&ahead, c, sizeof(ahead));
        ahead.etm_fired = 0;
        seen = core_run_group(&ahead, NO_EVENT - 1, w, groups[g]);
        if (seen == SEEN_NOTHING) {
            continue;
        }
        if (ahead.now < *when || (ahead.now == *when && seen == SEEN_IRQ)) {
            *when = ahead.now;
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
            core_raise(c, INT_FAULT_CLR(n), ETM_FAULT_CLR(n));
            continue;
        }
        core_raise(c, INT_FAULT(n), ETM_FAULT(n));
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

/*
 * @clk's whole cycles from @from to @t, and the first nanosecond by which
 * @ticks of them have elapsed from @from: the ESP32-P4's MCPWMn_CLK's
 * edges, at its exact period and phase (esp32p4_clk_period.h), the
 * ESP32's at its Clock's period from @from.
 */
static uint64_t mcpwm_clk_cycles(Esp32McpwmState *s, Clock *clk,
                                 int64_t from, int64_t t)
{
    if (t <= from) {
        return 0;
    }
    if (s->p4) {
        return esp32p4_clk_edges_between(&s->p4_edges, from, t);
    }
    return clock_ns_to_ticks(clk, t - from);
}

static int64_t mcpwm_clk_after(Esp32McpwmState *s, Clock *clk, int64_t from,
                               uint64_t ticks)
{
    uint64_t ns;

    if (s->p4) {
        return esp32p4_clk_edges_after(&s->p4_edges, from, ticks);
    }
    if (!clock_is_enabled(clk)) {
        return INT64_MAX;
    }
    ns = clock_ticks_to_ns(clk, ticks);
    while (clock_ns_to_ticks(clk, ns) < ticks) {
        ns++;
    }
    return from + MIN(ns, (uint64_t)INT64_MAX / 2);
}

/*
 * PWM_clk is its source clock divided by CLK_PRESCALE + 1, by a counter
 * of the source's cycles that keeps its count through a change of the
 * source's rate and of the division: a change keeps the part of the
 * PWM_clk cycle in progress that the counter has counted. The source's
 * own cycles are counted from its last change, src_ns, at its edges after
 * it as every block counting a clock takes them (esp32p4_clk_period.h;
 * the ESP32's Clocks from the change). At the source's cycle div_tick, at
 * div_ns, PWM_clk cycle div_cycle had div_k of them counted.
 */
static uint64_t mcpwm_src_cycles(Esp32McpwmState *s, int64_t ns)
{
    return mcpwm_clk_cycles(s, s->core_clk, s->src_ns, ns);
}

static uint64_t mcpwm_cycle_at(Esp32McpwmState *s, int64_t ns)
{
    uint64_t t = mcpwm_src_cycles(s, ns);

    if (t <= s->div_tick) {
        return s->div_cycle;
    }
    return s->div_cycle + (s->div_k + (t - s->div_tick)) / clk_div(s);
}

/* The first virtual time at which PWM_clk has reached `cycle`. */
static int64_t mcpwm_ns_at(Esp32McpwmState *s, uint64_t cycle)
{
    uint64_t ticks;

    if (cycle <= s->div_cycle) {
        return s->div_ns;
    }
    if (cycle - s->div_cycle > UINT64_MAX / 4 / clk_div(s)) {
        return INT64_MAX / 2;
    }
    ticks = s->div_tick + (cycle - s->div_cycle) * clk_div(s) - s->div_k;
    return MIN(mcpwm_clk_after(s, s->core_clk, s->src_ns, ticks),
               INT64_MAX / 2);
}

/*
 * The counter at time t, the core run to it, taken as the start of what
 * follows: its count kept, for a division of @div from here. A count
 * already at or past a smaller division ends the cycle at the source's
 * next cycle, the counter's comparison being "at least" (the TRM gives
 * none).
 */
static void mcpwm_div_rebase(Esp32McpwmState *s, int64_t t, uint32_t div)
{
    uint64_t ticks = mcpwm_src_cycles(s, t);
    uint64_t n = s->div_k + (ticks > s->div_tick ? ticks - s->div_tick : 0);

    s->div_cycle += n / clk_div(s);
    s->div_k = MIN(n % clk_div(s), div - 1);
    s->div_tick = ticks;
    s->div_ns = t;
}

/*
 * The present: the time of the line change being propagated, if this
 * block acts on one, so that an input acts at the moment its edge is
 * stamped with; otherwise the moment the block's own work began, as every
 * entry into the block runs its work at that one time (mcpwm_at()).
 */
static int64_t mcpwm_present(Esp32McpwmState *s)
{
    return esp32_line_time();
}

/*
 * The moment the block acts at: the present, but while the timer fires
 * the moment it was due: the timer can fire late, and the edges it
 * drives, and anything they cause in this block (a capture of its own
 * output through the GPIO matrix), belong to the due time. Time never
 * goes back from a moment the block has already acted at.
 */
static int64_t mcpwm_now(Esp32McpwmState *s)
{
    int64_t now = mcpwm_present(s);

    if (s->in_timer_cb) {
        now = MIN(s->due_ns, now);
    }
    s->seen_ns = MAX(s->seen_ns, now);
    return s->seen_ns;
}

/*
 * Run fn(opaque) at one moment, the present as fn is called: everything
 * fn does in the block, and every line change it makes and everything
 * that reaches, sees that time (esp32_line.h). Without icount the virtual
 * clock moves on while the block works; reading it again partway, the
 * block would run its counters past an edge due between the two readings
 * and then drive that edge late.
 */
static void mcpwm_at(void (*fn)(void *opaque), void *opaque)
{
    esp32_line_run_at(esp32_line_time(), fn, opaque);
}

/* The capture timer at time t, no earlier than its anchor. */
static uint32_t cap_timer_at(Esp32McpwmState *s, int64_t t)
{
    if (!(s->core.cap_timer_cfg & CAP_TIMER_EN) || t <= s->cap_anchor_ns) {
        return s->cap_anchor;
    }
    return s->cap_anchor + (uint32_t)mcpwm_clk_cycles(s, s->cap_clk,
                                                      s->cap_anchor_ns, t);
}

static uint32_t cap_timer_now(Esp32McpwmState *s)
{
    return cap_timer_at(s, mcpwm_now(s));
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

/*
 * [spec:nuos:req:emu.esp32p4.pwm]
 * [spec:nuos:req:emu.esp32p4.etm]
 * Pulse the enabled ETM events that have happened, stamped with time
 * when, in the order of their IDs. A task this answers in this block
 * comes back before this returns, and finds them cleared.
 */
static void mcpwm_etm_emit(Esp32McpwmState *s, int64_t when)
{
    Esp32McpwmCore *c = &s->core;
    uint64_t fired = c->etm_fired & core_etm_enabled(c);

    c->etm_fired = 0;
    c->etm_cycle = 0;
    if (!s->p4) {
        return;
    }
    while (fired) {
        int n = ctz64(fired);

        fired &= fired - 1;
        esp32_line_set_at(s->etm_event[n], 1, when);
        esp32_line_set_at(s->etm_event[n], 0, when);
    }
}

static void mcpwm_timer_fire(Esp32McpwmState *s);
static void mcpwm_skip(Esp32McpwmState *s, int64_t t);

/*
 * The timer's work, due at due_ns, done by `limit`: a firing at its due
 * time, or, more than CATCHUP_NS behind, a skip to CATCHUP_NS before
 * `limit`.
 */
static void mcpwm_due(Esp32McpwmState *s, int64_t limit)
{
    if (s->due_ns < limit - CATCHUP_NS) {
        mcpwm_skip(s, limit - CATCHUP_NS);
    } else {
        mcpwm_timer_fire(s);
    }
}

/*
 * Do everything the timer was due to do by `limit`, each firing at its own
 * due time: each firing drives the edges and raises the interrupts of its
 * moment, and sets the timer for the next, strictly later one, so that
 * this ends. A block that ran its counters to `limit` at once would drive
 * the edges it passed at `limit`.
 */
static void mcpwm_catch_up(Esp32McpwmState *s, int64_t limit)
{
    while (!s->in_timer_cb && timer_pending(s->timer) &&
           s->due_ns <= limit) {
        timer_del(s->timer);
        mcpwm_due(s, limit);
    }
}

/*
 * Run the core to the present. The timer's work that is already due is
 * done first, at its due time, so that an access racing a late callback
 * sees the edges driven when they happened. The run's ETM events go out
 * at the time they happened: the timer fires for each, so that a run
 * meets at most one cycle with any.
 */
static void mcpwm_sync(Esp32McpwmState *s)
{
    Esp32McpwmCore *c = &s->core;
    uint64_t target;

    mcpwm_catch_up(s, mcpwm_present(s));
    target = mcpwm_cycle_at(s, mcpwm_now(s));

    if (target > c->now) {
        core_run(c, target);
    }
    mcpwm_cap_sync_hit(s);
    if (c->etm_fired) {
        mcpwm_etm_emit(s, mcpwm_ns_at(s, c->etm_cycle));
    }
}

/*
 * The block has changed other than by running on: what the timer is set
 * for is to be found again. Running on alone leaves it as it is: the core
 * evolves alone, so that the first visible change from any state on its
 * way to that change is that same change.
 */
static void mcpwm_changed(Esp32McpwmState *s)
{
    s->schedule_stale = true;
}

static void mcpwm_schedule(Esp32McpwmState *s)
{
    McpwmWatch w;
    McpwmSeen seen;
    uint64_t when;

    s->schedule_stale = false;
    if (!clock_is_enabled(s->core_clk)) {
        timer_del(s->timer);
        return;
    }
    w.driven = s->out_level;
    w.outs = s->out_seen;
    w.irq = !s->irq_level;
    w.etm = s->p4 ? core_etm_enabled(&s->core) : 0;
    seen = core_look_ahead(&s->core, &w, &when);
    if (seen == SEEN_NOTHING) {
        timer_del(s->timer);
        return;
    }
    s->due_ns = mcpwm_ns_at(s, when);
    timer_mod(s->timer, s->due_ns);
}

/*
 * Drive the outputs and the interrupt from the core's state, and pulse
 * the ETM events of what has just been done.
 */
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
                mcpwm_changed(s);
                esp32_line_set_at(s->out[n], level, mcpwm_now(s));
            }
        }
        if (c->etm_fired) {
            mcpwm_etm_emit(s, mcpwm_now(s));
        }
        irq_level = !!(c->int_raw & c->int_ena);
        if (irq_level != s->irq_level) {
            s->irq_level = irq_level;
            mcpwm_changed(s);
            qemu_set_irq(s->irq, irq_level);
        }
    } while (s->update_pending);
    s->updating = false;
    if (s->schedule_stale) {
        mcpwm_schedule(s);
    }
}

/* The timer firing, at the moment it was due; it is no longer set. */
static void mcpwm_timer_fire(Esp32McpwmState *s)
{
    s->in_timer_cb = true;
    mcpwm_changed(s);
    mcpwm_sync(s);
    mcpwm_update(s);
    s->in_timer_cb = false;
}

/*
 * Run a copy of core c to cycle `to` (no earlier than its own), into
 * *end, and find each output's level there, level[n], and the cycle at
 * which it last changed, last[n] (NO_EVENT if not since c's cycle). Only
 * the last stretch is walked change by change, long enough to find the
 * last change of every output whose level at `to` differs from
 * driven[n]; the rest runs at once.
 */
static void core_last_changes(const Esp32McpwmCore *c, uint64_t to,
                              const uint8_t *driven, Esp32McpwmCore *end,
                              uint8_t *level, uint64_t *last)
{
    for (uint64_t w = SKIP_WINDOW;; w *= 2) {
        uint64_t start = to - c->now > w ? to - w : c->now;
        bool found = true;

        memcpy(end, c, sizeof(*end));
        core_run(end, start);
        for (int n = 0; n < ESP32_MCPWM_OUTS; n++) {
            level[n] = op_out(end, n / 2, n % 2);
            last[n] = NO_EVENT;
        }
        for (;;) {
            McpwmWatch watch = {
                .driven = level, .outs = MAKE_64BIT_MASK(0, ESP32_MCPWM_OUTS)
            };
            uint64_t when;

            if (core_look_ahead(end, &watch, &when) == SEEN_NOTHING ||
                when > to) {
                break;
            }
            core_run(end, when);
            for (int n = 0; n < ESP32_MCPWM_OUTS; n++) {
                uint8_t v = op_out(end, n / 2, n % 2);

                if (v != level[n]) {
                    level[n] = v;
                    last[n] = when;
                }
            }
        }
        core_run(end, to);
        for (int n = 0; n < ESP32_MCPWM_OUTS; n++) {
            if (level[n] != driven[n] && last[n] == NO_EVENT &&
                start > c->now) {
                found = false;
            }
        }
        if (found) {
            return;
        }
    }
}

/*
 * The timer's work more than CATCHUP_NS behind: the host may be slower
 * than the edges it is to drive, and catching up edge by edge would never
 * finish. The block skips ahead to the present (esp32_line_time()): its
 * counters run on to it at once, and each output whose level there
 * differs from its line is driven stamped with the moment it last
 * changed, in the order of those moments. The edges in between are lost,
 * as coalesced ones are; every edge driven keeps its own time, so that a
 * capture of it is exact still. The ETM events of the stretch pulse once,
 * at the time of the latest.
 */
static void mcpwm_skip_run(void *opaque)
{
    Esp32McpwmState *s = opaque;
    Esp32McpwmCore *c = &s->core;
    Esp32McpwmCore end;
    uint8_t level[ESP32_MCPWM_OUTS];
    uint64_t last[ESP32_MCPWM_OUTS];
    uint64_t to = MAX(mcpwm_cycle_at(s, mcpwm_now(s)), c->now);

    core_last_changes(c, to, s->out_level, &end, level, last);
    memcpy(c, &end, sizeof(*c));
    mcpwm_cap_sync_hit(s);
    if (c->etm_fired) {
        mcpwm_etm_emit(s, mcpwm_ns_at(s, c->etm_cycle));
    }
    mcpwm_changed(s);
    /* What the edges reach answers once they are all driven. */
    s->updating = true;
    for (;;) {
        int first = -1;
        int64_t when;

        for (int n = 0; n < ESP32_MCPWM_OUTS; n++) {
            if (level[n] != s->out_level[n] &&
                (first < 0 || last[n] < last[first])) {
                first = n;
            }
        }
        if (first < 0) {
            break;
        }
        when = last[first] == NO_EVENT ? mcpwm_now(s)
                                       : mcpwm_ns_at(s, last[first]);
        s->out_level[first] = level[first];
        esp32_line_set_at(s->out[first], level[first], when);
    }
    s->updating = false;
    mcpwm_update(s);
}

/* Skip to time t, the timer's work due before it undone (mcpwm_skip_run()) */
static void mcpwm_skip(Esp32McpwmState *s, int64_t t)
{
    timer_del(s->timer);
    esp32_line_run_at(t, mcpwm_skip_run, s);
}

/*
 * The timer's callback, at the moment it runs, does all the work due up
 * to that moment, each piece at its own due time, so that it leaves the
 * timer set after that moment: callbacks each due before the moment they
 * are run at would keep the main loop running timers.
 */
static void mcpwm_timer_run(void *opaque)
{
    Esp32McpwmState *s = opaque;
    int64_t limit = mcpwm_present(s);

    mcpwm_due(s, limit);
    mcpwm_catch_up(s, limit);
}

static void mcpwm_timer_cb(void *opaque)
{
    mcpwm_at(mcpwm_timer_run, opaque);
}

static void mcpwm_line_sync_run(void *opaque)
{
    Esp32McpwmState *s = opaque;

    mcpwm_sync(s);
    mcpwm_update(s);
}

/*
 * [spec:nuos:req:emu.esp32.mcpwm]
 * [spec:nuos:req:emu.esp32p4.pwm]
 * Which outputs reach a pad, through the GPIO matrix or an IO MUX
 * function, with the pad's output enabled, whose level something but
 * software sampling the pads sees (an input a modelled block takes from
 * it, an interrupt type, an ETM channel, a line the board listens to:
 * TYPE_ESP32_SIG_ROUTES): only their edges are looked ahead for and
 * timed. The others, which nothing sees, are driven at
 * their level of the moment whenever the block is brought up to date (a
 * register access, a firing for what is seen, software sampling the
 * pads), so that an output nobody watches costs the host nothing however
 * fast it switches. An output a pad takes up is first driven at its level
 * of that moment.
 */
static void mcpwm_routes_run(void *opaque)
{
    Esp32McpwmState *s = opaque;
    uint8_t seen = 0;

    mcpwm_sync(s);
    mcpwm_update(s);
    for (int n = 0; n < ESP32_MCPWM_OUTS; n++) {
        if (!s->gpio ||
            esp32_sig_out_watched(OBJECT(s->gpio), s->sig_base + n)) {
            seen |= BIT(n);
        }
    }
    if (seen != s->out_seen) {
        s->out_seen = seen;
        mcpwm_changed(s);
        mcpwm_schedule(s);
    }
}

static void mcpwm_route_notify(Notifier *n, void *data)
{
    mcpwm_at(mcpwm_routes_run,
             container_of(n, Esp32McpwmState, route_notifier));
}

/* Software sampling the pads: the outputs at their levels of the moment. */
static void mcpwm_line_sync(Notifier *n, void *data)
{
    Esp32McpwmState *s = container_of(n, Esp32McpwmState, line_source);

    if (s->syncing_others) {
        return;
    }
    mcpwm_at(mcpwm_line_sync_run, s);
}

/*
 * Bring every other source of line changes up to the present, the moment
 * this block acts at, so that an edge whose timer has not yet fired (the
 * host running behind virtual time) reaches a capture, fault or sync input
 * stamped with its own time before this block acts; edges after the
 * present are yet to come.
 */
static void mcpwm_sync_others(Esp32McpwmState *s)
{
    s->syncing_others = true;
    esp32_line_sync();
    s->syncing_others = false;
}

/*
 * A register access samples the block's inputs: the other sources first,
 * then the block's own timer. The access runs at the moment it began
 * (mcpwm_at()).
 */
static void mcpwm_access_begin(Esp32McpwmState *s)
{
    mcpwm_sync_others(s);
    mcpwm_sync(s);
}

/* Capture */

/*
 * A capture reads the capture timer at the time of the change that makes
 * it, though the block may have acted at a later moment already: an edge
 * another block drives late (the host running behind virtual time), which
 * a line sync brings after this block's own, is still timed exactly.
 */
static void cap_capture(Esp32McpwmState *s, int n, bool neg)
{
    s->cap_val[n] = cap_timer_at(s, mcpwm_present(s));
    s->cap_status = (s->cap_status & ~(1u << n)) | ((uint32_t)neg << n);
    core_raise(&s->core, INT_CAP(n), ETM_CAP(n));
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

/*
 * [spec:nuos:req:emu.esp32p4.pwm]
 * The ESP32-P4's registers from 0x120: the ETM's enables, the event
 * comparators, CLK and VERSION. Returns false for an offset not among
 * them.
 */
static bool p4_read(Esp32McpwmState *s, hwaddr addr, uint64_t *v)
{
    Esp32McpwmCore *c = &s->core;

    if (addr >= A_P4_OP_TSTMP_E && addr < A_P4_OP_TSTMP_E_END) {
        unsigned n = (addr - A_P4_OP_TSTMP_E) / 4;

        *v = c->op[n / 2].tstmp_e[n % 2];
        return true;
    }
    switch (addr) {
    case A_P4_EVT_EN:
        *v = c->evt_en;
        return true;
    case A_P4_TASK_EN:
        *v = c->task_en;
        return true;
    case A_P4_EVT_EN2:
        *v = c->evt_en2;
        return true;
    case A_P4_CLK:
        *v = s->clk_reg;
        return true;
    case A_P4_VERSION:
        *v = s->version;
        return true;
    }
    return false;
}

/* [spec:nuos:req:emu.esp32p4.pwm] */
static bool p4_write(Esp32McpwmState *s, hwaddr addr, uint32_t v)
{
    Esp32McpwmCore *c = &s->core;

    if (addr >= A_P4_OP_TSTMP_E && addr < A_P4_OP_TSTMP_E_END) {
        unsigned n = (addr - A_P4_OP_TSTMP_E) / 4;

        c->op[n / 2].tstmp_e[n % 2] = v & TSTMP_E_MASK;
        return true;
    }
    switch (addr) {
    case A_P4_EVT_EN:
        c->evt_en = v & EVT_EN_MASK;
        return true;
    case A_P4_TASK_EN:
        c->task_en = v & TASK_EN_MASK;
        return true;
    case A_P4_EVT_EN2:
        c->evt_en2 = v & EVT_EN2_MASK;
        return true;
    case A_P4_CLK:
        s->clk_reg = v & 1;
        return true;
    case A_P4_VERSION:
        s->version = v & VERSION_MASK;
        return true;
    }
    return false;
}

/* A register access, run at the moment it began (mcpwm_at()). */
typedef struct McpwmAccess {
    Esp32McpwmState *s;
    hwaddr addr;
    uint64_t value;
} McpwmAccess;

/* [spec:nuos:req:emu.esp32.mcpwm] */
/* [spec:nuos:req:emu.esp32p4.pwm] */
static void mcpwm_do_read(void *opaque)
{
    McpwmAccess *a = opaque;
    Esp32McpwmState *s = a->s;
    Esp32McpwmCore *c = &s->core;
    hwaddr addr = a->addr;
    uint64_t v = 0;

    mcpwm_access_begin(s);
    if (s->p4 && addr >= A_P4_EVT_EN) {
        if (!p4_read(s, addr, &v)) {
            qemu_log_mask(LOG_GUEST_ERROR, "esp32-mcpwm: read of bad offset "
                          "0x%" HWADDR_PRIx "\n", addr);
        }
    } else if (addr >= A_TIMER_BASE && addr < A_SYNCI_CFG) {
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
    a->value = v;
}

/* [spec:nuos:req:emu.esp32.mcpwm] */
/* [spec:nuos:req:emu.esp32p4.pwm] */
static uint64_t esp32_mcpwm_read(void *opaque, hwaddr addr, unsigned size)
{
    McpwmAccess a = { .s = opaque, .addr = addr };

    mcpwm_at(mcpwm_do_read, &a);
    return a.value;
}

/* [spec:nuos:req:emu.esp32.mcpwm] */
/* [spec:nuos:req:emu.esp32p4.pwm] */
static void mcpwm_do_write(void *opaque)
{
    McpwmAccess *a = opaque;
    Esp32McpwmState *s = a->s;
    Esp32McpwmCore *c = &s->core;
    hwaddr addr = a->addr;
    uint32_t v = a->value;

    mcpwm_access_begin(s);
    mcpwm_changed(s);
    if (s->p4 && addr >= A_P4_EVT_EN) {
        if (!p4_write(s, addr, v)) {
            qemu_log_mask(LOG_GUEST_ERROR, "esp32-mcpwm: write to bad offset "
                          "0x%" HWADDR_PRIx "\n", addr);
        }
    } else if (addr >= A_TIMER_BASE && addr < A_SYNCI_CFG) {
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
            /* The source runs on; the counter keeps its count. */
            mcpwm_div_rebase(s, mcpwm_now(s), (v & 0xff) + 1);
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

/* [spec:nuos:req:emu.esp32.mcpwm] */
/* [spec:nuos:req:emu.esp32p4.pwm] */
static void esp32_mcpwm_write(void *opaque, hwaddr addr, uint64_t value,
                              unsigned size)
{
    McpwmAccess a = { .s = opaque, .addr = addr, .value = value };

    mcpwm_at(mcpwm_do_write, &a);
}

static const MemoryRegionOps esp32_mcpwm_ops = {
    .read = esp32_mcpwm_read,
    .write = esp32_mcpwm_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/*
 * [spec:nuos:req:emu.esp32p4.pwm]
 * The ESP32-P4's GEN_STOP task: every running timer stops where it
 * stands, raising its stop interrupt and event; the operators, with no
 * more timing events, hold their outputs.
 */
static void core_gen_stop(Esp32McpwmCore *c)
{
    for (int i = 0; i < ESP32_MCPWM_TIMERS; i++) {
        Esp32McpwmTimer *t = &c->timer[i];

        if (t->running) {
            t->running = false;
            core_raise(c, INT_TIMER_STOP(i), ETM_TIMER_STOP(i));
        }
    }
}

/* A change of an input line, run at its time (mcpwm_at()). */
typedef struct McpwmInput {
    Esp32McpwmState *s;
    int n;
    int level;
} McpwmInput;

/*
 * [spec:nuos:req:emu.esp32p4.pwm]
 * [spec:nuos:req:emu.esp32p4.etm]
 * An ETM task, acted on as its line rises while TASK_EN enables it, at
 * the time of the event that generated it.
 */
static void mcpwm_do_etm_task(void *opaque)
{
    McpwmInput *in = opaque;
    Esp32McpwmState *s = in->s;
    Esp32McpwmCore *c = &s->core;
    int n = in->n;

    mcpwm_sync(s);
    mcpwm_changed(s);
    if (n <= TASK_CMPR_B_UP(ESP32_MCPWM_OPS - 1)) {
        op_transfer(&c->op[n % ESP32_MCPWM_OPS],
                    n < TASK_CMPR_B_UP(0) ? OP_PEND_A : OP_PEND_B);
    } else if (n == TASK_GEN_STOP) {
        core_gen_stop(c);
    } else if (n <= TASK_TIMER_SYNC(ESP32_MCPWM_TIMERS - 1)) {
        int i = n - TASK_TIMER_SYNC(0);

        if (c->timer[i].sync & SYNC_SYNCI_EN) {
            timer_take_sync(c, i);
        }
        timer_sync_out(c, i, 1u << i);
    } else if (n <= TASK_TIMER_PERIOD_UP(ESP32_MCPWM_TIMERS - 1)) {
        timer_transfer(c, n - TASK_TIMER_PERIOD_UP(0), true);
    } else if (n <= TASK_TZ_OST(ESP32_MCPWM_OPS - 1)) {
        int k = n - TASK_TZ_OST(0);

        if (!c->op[k].ost_on) {
            c->op[k].ost_on = true;
            core_raise(c, INT_FH_OST(k), ETM_FH_OST(k));
        }
    } else if (n <= TASK_CLR_OST(ESP32_MCPWM_OPS - 1)) {
        c->op[n - TASK_CLR_OST(0)].ost_on = false;
    } else {
        int cap = n - TASK_CAP(0);

        cap_capture(s, cap, s->cap_status & (1u << cap));
    }
    mcpwm_cap_sync_hit(s);
    core_settle(c);
    mcpwm_update(s);
}

/* [spec:nuos:req:emu.esp32p4.pwm] */
static void esp32_mcpwm_etm_task(void *opaque, int n, int level)
{
    McpwmInput in = { .s = opaque, .n = n, .level = level };

    if (!level || !(in.s->core.task_en & (1u << n))) {
        return;
    }
    mcpwm_at(mcpwm_do_etm_task, &in);
}

/* Inputs from the GPIO matrix */

static void mcpwm_do_sync_in(void *opaque)
{
    McpwmInput *in = opaque;
    Esp32McpwmState *s = in->s;
    bool before;

    mcpwm_sync(s);
    mcpwm_changed(s);
    before = sync_level(s, in->n);
    s->sync_in[in->n] = in->level != 0;
    if (!before && sync_level(s, in->n)) {
        gpio_sync(s, in->n);
    }
    mcpwm_update(s);
}

static void esp32_mcpwm_sync_in(void *opaque, int n, int level)
{
    McpwmInput in = { .s = opaque, .n = n, .level = level };

    mcpwm_at(mcpwm_do_sync_in, &in);
}

static void mcpwm_do_fault_in(void *opaque)
{
    McpwmInput *in = opaque;
    Esp32McpwmState *s = in->s;

    mcpwm_sync(s);
    mcpwm_changed(s);
    s->core.fault_in[in->n] = in->level != 0;
    core_faults(&s->core);
    mcpwm_update(s);
}

static void esp32_mcpwm_fault_in(void *opaque, int n, int level)
{
    McpwmInput in = { .s = opaque, .n = n, .level = level };

    mcpwm_at(mcpwm_do_fault_in, &in);
}

static void mcpwm_do_cap_in(void *opaque)
{
    McpwmInput *in = opaque;
    Esp32McpwmState *s = in->s;

    mcpwm_sync(s);
    mcpwm_changed(s);
    s->cap_in[in->n] = in->level != 0;
    cap_input(s, in->n);
    mcpwm_update(s);
}

static void esp32_mcpwm_cap_in(void *opaque, int n, int level)
{
    McpwmInput in = { .s = opaque, .n = n, .level = level };

    mcpwm_at(mcpwm_do_cap_in, &in);
}

/*
 * A clock about to change, at the moment of the change: the other line
 * sources and this block's own timer are first brought up to it, so that
 * every edge before it reaches this block and is counted at the old rate,
 * then the counting starts there afresh.
 */
static void mcpwm_clk_pre_update(Esp32McpwmState *s)
{
    mcpwm_sync_others(s);
    mcpwm_sync(s);
    mcpwm_changed(s);
}

/*
 * A change of PWM_clk's source, PLL_F160M_CLK or MCPWMn_CLK: run to the
 * change at the old rate; the counter keeps its count, and the source's
 * cycles start afresh from the change.
 */
static void mcpwm_core_clk_pre(void *opaque)
{
    Esp32McpwmState *s = opaque;

    mcpwm_clk_pre_update(s);
    mcpwm_div_rebase(s, mcpwm_now(s), clk_div(s));
    s->src_ns = s->div_ns;
    s->div_tick = 0;
}

/* The new rate takes effect at the moment of the change. */
static void mcpwm_core_clk_post(void *opaque)
{
    Esp32McpwmState *s = opaque;

    mcpwm_changed(s);
    mcpwm_update(s);
}

static void esp32_mcpwm_core_clk_update(void *opaque, ClockEvent event)
{
    Esp32McpwmState *s = opaque;

    if (event == ClockPreUpdate) {
        mcpwm_at(mcpwm_core_clk_pre, s);
    } else {
        esp32_line_run_at(s->src_ns, mcpwm_core_clk_post, s);
    }
}

/* A change of the capture timer's clock: the count so far at the old rate */
static void mcpwm_cap_clk_pre(void *opaque)
{
    Esp32McpwmState *s = opaque;

    mcpwm_clk_pre_update(s);
    cap_reanchor(s);
}

static void esp32_mcpwm_cap_clk_update(void *opaque, ClockEvent event)
{
    if (event == ClockPreUpdate) {
        mcpwm_at(mcpwm_cap_clk_pre, opaque);
    }
}

/*
 * The ESP32-P4's MCPWMn_CLK, PWM_clk's source and the capture timer's:
 * both change at one moment, and are counted at the clock's exact period
 * from the change on.
 */
static void esp32p4_mcpwm_clk_pre(void *opaque)
{
    mcpwm_cap_clk_pre(opaque);
    mcpwm_core_clk_pre(opaque);
}

static void esp32p4_mcpwm_clk_update(void *opaque, ClockEvent event)
{
    Esp32McpwmState *s = opaque;

    if (event == ClockPreUpdate) {
        mcpwm_at(esp32p4_mcpwm_clk_pre, s);
    } else {
        s->p4_edges = esp32p4_clk_edges_or_at(s->core_clk, s->src_ns);
        esp32_line_run_at(s->src_ns, mcpwm_core_clk_post, s);
    }
}

static void esp32_mcpwm_reset_hold(Object *obj, ResetType type)
{
    Esp32McpwmState *s = ESP32_MCPWM(obj);
    Esp32McpwmCore *c = &s->core;
    bool fault_in[ESP32_MCPWM_FAULTS];

    timer_del(s->timer);
    mcpwm_changed(s);
    if (s->p4) {
        s->p4_edges = esp32p4_clk_edges_or_at(
            s->core_clk, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    }
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
    s->version = s->p4 ? P4_VERSION_RESET : VERSION_RESET;
    s->seen_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    /* The counter starts afresh; its source runs on. */
    s->div_ns = mcpwm_now(s);
    s->div_tick = mcpwm_src_cycles(s, s->div_ns);
    s->div_cycle = 0;
    s->div_k = 0;
    s->cap_phase = 0;
    s->cap_anchor = 0;
    s->cap_anchor_ns = s->div_ns;
    memset(s->cap_cfg, 0, sizeof(s->cap_cfg));
    memset(s->cap_val, 0, sizeof(s->cap_val));
    s->cap_status = 0;
    memset(s->cap_pre_cnt, 0, sizeof(s->cap_pre_cnt));
    memset(s->cap_pre_out, 0, sizeof(s->cap_pre_out));
    for (int n = 0; n < ESP32_MCPWM_CAPS; n++) {
        s->cap_x[n] = s->cap_in[n];
    }
    /* Drive every line afresh. */
    memset(s->out_level, 0xff, sizeof(s->out_level));
    s->irq_level = 0xff;
}

static void esp32_mcpwm_reset_exit(Object *obj, ResetType type)
{
    Esp32McpwmState *s = ESP32_MCPWM(obj);

    mcpwm_update(s);
    mcpwm_at(mcpwm_routes_run, s);
}

static void esp32_mcpwm_init(Object *obj)
{
    Esp32McpwmState *s = ESP32_MCPWM(obj);
    DeviceState *dev = DEVICE(obj);

    s->p4 = ESP32_MCPWM_GET_CLASS(obj)->p4;
    memory_region_init_io(&s->iomem, obj, &esp32_mcpwm_ops, s,
                          object_get_typename(obj), ESP32_MCPWM_REGION_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    qdev_init_gpio_out_named(dev, s->out, ESP32_MCPWM_OUT, ESP32_MCPWM_OUTS);
    qdev_init_gpio_in_named(dev, esp32_mcpwm_sync_in, ESP32_MCPWM_SYNC_IN,
                            ESP32_MCPWM_SYNCS);
    qdev_init_gpio_in_named(dev, esp32_mcpwm_fault_in, ESP32_MCPWM_FAULT_IN,
                            ESP32_MCPWM_FAULTS);
    qdev_init_gpio_in_named(dev, esp32_mcpwm_cap_in, ESP32_MCPWM_CAP_IN,
                            ESP32_MCPWM_CAPS);
    if (s->p4) {
        s->core_clk = qdev_init_clock_in(dev, "clk", esp32p4_mcpwm_clk_update,
                                         s, ClockPreUpdate | ClockUpdate);
        s->cap_clk = s->core_clk;
        qdev_init_gpio_out_named(dev, s->etm_event, ESP32_MCPWM_ETM_EVENT,
                                 ESP32P4_MCPWM_ETM_EVENTS);
        qdev_init_gpio_in_named(dev, esp32_mcpwm_etm_task,
                                ESP32_MCPWM_ETM_TASK, ESP32P4_MCPWM_ETM_TASKS);
    } else {
        s->core_clk = qdev_init_clock_in(dev, "f160m",
                                         esp32_mcpwm_core_clk_update, s,
                                         ClockPreUpdate | ClockUpdate);
        s->cap_clk = qdev_init_clock_in(dev, "apb", esp32_mcpwm_cap_clk_update,
                                        s, ClockPreUpdate);
    }
    s->line_source.notify = mcpwm_line_sync;
    s->route_notifier.notify = mcpwm_route_notify;
    s->out_seen = MAKE_64BIT_MASK(0, ESP32_MCPWM_OUTS);
}

static void esp32_mcpwm_realize(DeviceState *dev, Error **errp)
{
    Esp32McpwmState *s = ESP32_MCPWM(dev);

    if (s->gpio && !object_dynamic_cast(OBJECT(s->gpio),
                                        TYPE_ESP32_SIG_ROUTES)) {
        error_setg(errp, "%s: \"gpio\" is not a GPIO matrix",
                   object_get_typename(OBJECT(dev)));
        return;
    }
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, mcpwm_timer_cb, s);
    esp32_line_add_source(&s->line_source);
    if (s->gpio) {
        esp32_sig_add_route_notifier(OBJECT(s->gpio), &s->route_notifier);
    }
}

static const Property esp32_mcpwm_properties[] = {
    DEFINE_PROP_LINK("gpio", Esp32McpwmState, gpio, TYPE_DEVICE,
                     DeviceState *),
    DEFINE_PROP_UINT32("sig-base", Esp32McpwmState, sig_base, 0),
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
        VMSTATE_UINT32_ARRAY(tstmp_e, Esp32McpwmOp, 2),
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
        VMSTATE_UINT32(evt_en, Esp32McpwmCore),
        VMSTATE_UINT32(evt_en2, Esp32McpwmCore),
        VMSTATE_UINT32(task_en, Esp32McpwmCore),
        VMSTATE_UINT64(etm_fired, Esp32McpwmCore),
        VMSTATE_UINT64(etm_cycle, Esp32McpwmCore),
        VMSTATE_END_OF_LIST()
    },
};

static bool esp32p4_mcpwm_period_needed(void *opaque)
{
    return ((Esp32McpwmState *)opaque)->p4;
}

static const VMStateDescription vmstate_esp32p4_mcpwm_period = {
    .name = TYPE_ESP32_MCPWM "/p4-period",
    .version_id = 2,
    .minimum_version_id = 2,
    .needed = esp32p4_mcpwm_period_needed,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT64(p4_edges.period.num_hi, Esp32McpwmState),
        VMSTATE_UINT64(p4_edges.period.num_lo, Esp32McpwmState),
        VMSTATE_UINT64(p4_edges.period.den, Esp32McpwmState),
        VMSTATE_UINT64(p4_edges.phase_hi, Esp32McpwmState),
        VMSTATE_UINT64(p4_edges.phase_lo, Esp32McpwmState),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_esp32_mcpwm = {
    .name = TYPE_ESP32_MCPWM,
    .version_id = 5,
    .minimum_version_id = 5,
    .fields = (const VMStateField[]) {
        VMSTATE_CLOCK(core_clk, Esp32McpwmState),
        VMSTATE_CLOCK(cap_clk, Esp32McpwmState),
        VMSTATE_TIMER_PTR(timer, Esp32McpwmState),
        VMSTATE_STRUCT(core, Esp32McpwmState, 1, vmstate_esp32_mcpwm_core,
                       Esp32McpwmCore),
        VMSTATE_UINT32(clk_cfg, Esp32McpwmState),
        VMSTATE_UINT32(clk_reg, Esp32McpwmState),
        VMSTATE_UINT32(version, Esp32McpwmState),
        VMSTATE_INT64(src_ns, Esp32McpwmState),
        VMSTATE_INT64(div_ns, Esp32McpwmState),
        VMSTATE_UINT64(div_tick, Esp32McpwmState),
        VMSTATE_UINT64(div_cycle, Esp32McpwmState),
        VMSTATE_UINT32(div_k, Esp32McpwmState),
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
        VMSTATE_INT64(due_ns, Esp32McpwmState),
        VMSTATE_INT64(seen_ns, Esp32McpwmState),
        VMSTATE_UINT8(out_seen, Esp32McpwmState),
        VMSTATE_END_OF_LIST()
    },
    .subsections = (const VMStateDescription * const []) {
        &vmstate_esp32p4_mcpwm_period,
        NULL
    },
};

static void esp32_mcpwm_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = esp32_mcpwm_realize;
    dc->vmsd = &vmstate_esp32_mcpwm;
    device_class_set_props(dc, esp32_mcpwm_properties);
    rc->phases.hold = esp32_mcpwm_reset_hold;
    rc->phases.exit = esp32_mcpwm_reset_exit;
}

static void esp32p4_mcpwm_class_init(ObjectClass *klass, const void *data)
{
    ESP32_MCPWM_CLASS(klass)->p4 = true;
}

/* [spec:nuos:req:emu.esp32.mcpwm] */
static const TypeInfo esp32_mcpwm_info = {
    .name          = TYPE_ESP32_MCPWM,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32McpwmState),
    .instance_init = esp32_mcpwm_init,
    .class_size    = sizeof(Esp32McpwmClass),
    .class_init    = esp32_mcpwm_class_init,
};

/* [spec:nuos:req:emu.esp32p4.pwm] */
static const TypeInfo esp32p4_mcpwm_info = {
    .name          = TYPE_ESP32P4_MCPWM,
    .parent        = TYPE_ESP32_MCPWM,
    .class_init    = esp32p4_mcpwm_class_init,
};

static void esp32_mcpwm_register_types(void)
{
    type_register_static(&esp32_mcpwm_info);
    type_register_static(&esp32p4_mcpwm_info);
}

type_init(esp32_mcpwm_register_types)
