/*
 * RP2350 DMA controller
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: RP2350 Datasheet, "DMA" and "Security: DMA"; register layout
 * and reset values from the pico-sdk dma.h.
 *
 * Timing: the DMA issues at most one transfer (a read and its paired
 * write) per clk_sys cycle (the "clk" clock, at its current frequency;
 * nothing happens while it is stopped), round-robin over the requesting
 * channels with high-priority channels first. The model keeps a cycle
 * count, `cycle`, up to which the engine has run. Each run of the engine
 * issues the transfers of every cycle between the last run and now;
 * then, while a channel paced by a DREQ requests, it goes on issuing
 * transfers in the cycles after now, one a cycle, so that a DREQ is
 * answered when it is raised rather than when a timer next fires. The
 * engine runs:
 *
 *  - from inside a DREQ change, unless the change comes from a vCPU or a
 *    transfer would reach a device in the middle of a register access,
 *    so that a peripheral running itself forward over a span of time sees
 *    its FIFO serviced at each DREQ in that span, as on hardware;
 *  - from a bottom half when a DREQ change cannot be answered in place,
 *    and after a register write leaves a DREQ-paced channel requesting;
 *  - from a timer in virtual time, armed for when there is more work, for
 *    the permanent request, the pacing timers and DREQs held high.
 *
 * A register access never issues a transfer itself: it only brings the
 * idle engine (and the pacing timers) up to date.
 *
 * Each transfer completes before the next is issued, so the bus pipeline
 * is never observable: FIFO_LEVELS reads zero, BUSY falls as the last
 * write completes, and CHAN_ABORT completes at once.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/main-loop.h"
#include "qemu/rcu.h"
#include "qemu/bswap.h"
#include "qemu/host-utils.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/core/cpu.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/dma/rp2350_dma.h"
#include "hw/misc/rp2350_atomic.h"
#include "migration/vmstate.h"

/* Channel registers: 0x40 per channel, four aliases of four registers. */
#define CH_STRIDE           0x40
#define CH_END              (RP2350_DMA_CHANNELS * CH_STRIDE)

#define A_INTR              0x400
#define A_IRQ_BASE          0x400   /* INTEn at 0x404 + 0x10 * n, etc. */
#define A_TIMER0            0x440
#define A_TIMER3            0x44c
#define A_MULTI_CHAN_TRIGGER 0x450
#define A_SNIFF_CTRL        0x454
#define A_SNIFF_DATA        0x458
#define A_FIFO_LEVELS       0x460
#define A_CHAN_ABORT        0x464
#define A_N_CHANNELS        0x468
#define A_SECCFG_CH0        0x480
#define A_SECCFG_CH15       0x4bc
#define A_SECCFG_IRQ0       0x4c0
#define A_SECCFG_IRQ3       0x4cc
#define A_SECCFG_MISC       0x4d0
#define A_MPU_CTRL          0x500
#define A_MPU_BAR0          0x504
#define A_MPU_LAR7          0x540
#define A_DBG_BASE          0x800
#define A_DBG_END           (A_DBG_BASE + RP2350_DMA_CHANNELS * CH_STRIDE)
#define A_DBG_CTDREQ        0x0
#define A_DBG_TCR           0x4

/* The four channel registers, as each alias orders them. */
enum { REG_READ_ADDR, REG_WRITE_ADDR, REG_TRANS_COUNT, REG_CTRL };

static const uint8_t alias_layout[4][4] = {
    { REG_READ_ADDR, REG_WRITE_ADDR, REG_TRANS_COUNT, REG_CTRL },
    { REG_CTRL, REG_READ_ADDR, REG_WRITE_ADDR, REG_TRANS_COUNT },
    { REG_CTRL, REG_TRANS_COUNT, REG_READ_ADDR, REG_WRITE_ADDR },
    { REG_CTRL, REG_WRITE_ADDR, REG_TRANS_COUNT, REG_READ_ADDR },
};

#define CTRL_EN             (1u << 0)
#define CTRL_HIGH_PRIORITY  (1u << 1)
#define CTRL_DATA_SIZE_SHIFT 2
#define CTRL_INCR_READ      (1u << 4)
#define CTRL_INCR_READ_REV  (1u << 5)
#define CTRL_INCR_WRITE     (1u << 6)
#define CTRL_INCR_WRITE_REV (1u << 7)
#define CTRL_RING_SIZE_SHIFT 8
#define CTRL_RING_SEL       (1u << 12)
#define CTRL_CHAIN_TO_SHIFT 13
#define CTRL_TREQ_SEL_SHIFT 17
#define CTRL_IRQ_QUIET      (1u << 23)
#define CTRL_BSWAP          (1u << 24)
#define CTRL_SNIFF_EN       (1u << 25)
#define CTRL_BUSY           (1u << 26)
#define CTRL_WRITE_ERROR    (1u << 29)
#define CTRL_READ_ERROR     (1u << 30)
#define CTRL_AHB_ERROR      (1u << 31)
#define CTRL_RW_MASK        0x03ffffffu
#define CTRL_ERRORS         (CTRL_READ_ERROR | CTRL_WRITE_ERROR)

#define TREQ_TIMER0         0x3b
#define TREQ_PERMANENT      0x3f

#define COUNT_MASK          0x0fffffffu
#define MODE_SHIFT          28
#define MODE_NORMAL         0x0
#define MODE_TRIGGER_SELF   0x1
#define MODE_ENDLESS        0xf

#define SECCFG_P            (1u << 0)
#define SECCFG_S            (1u << 1)
#define SECCFG_LOCK         (1u << 2)

#define SNIFF_EN            (1u << 0)
#define SNIFF_DMACH_SHIFT   1
#define SNIFF_CALC_SHIFT    5
#define SNIFF_BSWAP         (1u << 9)
#define SNIFF_OUT_REV       (1u << 10)
#define SNIFF_OUT_INV       (1u << 11)
#define SNIFF_CTRL_MASK     0xfffu
#define CALC_CRC32          0x0
#define CALC_CRC32R         0x1
#define CALC_CRC16          0x2
#define CALC_CRC16R         0x3
#define CALC_EVEN           0xe
#define CALC_SUM            0xf

#define MPU_CTRL_P          (1u << 1)
#define MPU_CTRL_S          (1u << 2)
#define MPU_CTRL_NS_HIDE_ADDR (1u << 3)
#define MPU_CTRL_MASK       0xeu
#define MPU_LAR_EN          (1u << 0)
#define MPU_LAR_P           (1u << 1)
#define MPU_LAR_S           (1u << 2)
#define MPU_ADDR_MASK       0xffffffe0u
#define MPU_LAR_MASK        0xffffffe7u

#define SECCFG_MISC_RESET   0x3ffu
#define DREQ_CREDIT_MAX     0x3f
#define ALL_CHANNELS        0xffffu

/* Security levels, ordered SP > SU > NSP > NSU. */
#define LEVEL_SP            RP2350_DMA_SECLEVEL_SP

/*
 * The most cycles the engine looks ahead when arming its timer while
 * channels are transferring: an upper bound on how stale the registers
 * can be while the engine is busy. It also bounds how far one run of the
 * engine goes past now for the DREQs.
 */
#define ENGINE_QUANTUM      1500

typedef enum EngineMode {
    /*
     * Bring the pacing timers and an idle engine up to date, stopping at
     * the first cycle in which a transfer or completion is due.
     */
    ENGINE_SYNC,
    /* Issue transfers, from the timer or the bottom half. */
    ENGINE_ISSUE,
    /* Issue transfers from inside a DREQ change. */
    ENGINE_SERVICE,
} EngineMode;

static unsigned access_level(MemTxAttrs attrs)
{
    if (attrs.unspecified) {
        /* qtest and loaders: a Secure, privileged debugger. */
        return LEVEL_SP;
    }
    return (attrs.secure ? 2 : 0) | (attrs.user ? 0 : 1);
}

static bool level_privileged(unsigned level)
{
    return level & 1;
}

static unsigned ch_level(RP2350DMAState *s, int n)
{
    return s->ch[n].seccfg & (SECCFG_S | SECCFG_P);
}

static unsigned irq_level(RP2350DMAState *s, int n)
{
    return s->seccfg_irq[n] & 3;
}

static unsigned timer_level(RP2350DMAState *s, int t)
{
    return (s->seccfg_misc >> (2 + 2 * t)) & 3;
}

static unsigned sniff_level(RP2350DMAState *s)
{
    return s->seccfg_misc & 3;
}

/* The channels whose level is at most `level`. */
static uint32_t channels_at_or_below(RP2350DMAState *s, unsigned level)
{
    uint32_t mask = 0;
    int n;

    for (n = 0; n < RP2350_DMA_CHANNELS; n++) {
        if (ch_level(s, n) <= level) {
            mask |= 1u << n;
        }
    }
    return mask;
}

static unsigned ctrl_treq(uint32_t ctrl)
{
    return extract32(ctrl, CTRL_TREQ_SEL_SHIFT, 6);
}

static unsigned ctrl_chain_to(uint32_t ctrl)
{
    return extract32(ctrl, CTRL_CHAIN_TO_SHIFT, 4);
}

static unsigned ch_mode(RP2350DMAChannel *c)
{
    return c->trans_count >> MODE_SHIFT;
}

/*
 * Clock conversions: the engine counts clk_sys cycles, cycle base_cycle
 * having been reached at virtual time base_ns at the current rate.
 */
/* [spec:nuos:req:emu.clock-tree] */
static uint64_t now_cycle(RP2350DMAState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (now <= s->base_ns) {
        return s->base_cycle;
    }
    return s->base_cycle + clock_ns_to_ticks(s->clk, now - s->base_ns);
}

/*
 * The first virtual time at which `cycle` has been reached, which never
 * comes while clk_sys is stopped.
 */
static int64_t cycle_ns(RP2350DMAState *s, uint64_t cycle)
{
    uint64_t d, ns;

    if (cycle <= s->base_cycle) {
        return s->base_ns;
    }
    if (!clock_is_enabled(s->clk)) {
        return INT64_MAX;
    }
    d = cycle - s->base_cycle;
    ns = clock_ticks_to_ns(s->clk, d);
    while (clock_ns_to_ticks(s->clk, ns) < d) {
        ns++;
    }
    return s->base_ns + MIN(ns, (uint64_t)INT64_MAX / 2);
}

/* Interrupts. */

/* [spec:nuos:req:emu.dma] */
static uint32_t ints(RP2350DMAState *s, int n)
{
    return (s->intr | s->intf[n]) & s->inte[n] &
           channels_at_or_below(s, irq_level(s, n));
}

/*
 * A channel's interrupt reaches an IRQ only if the IRQ's security level is
 * at least the channel's.
 */
/* [spec:nuos:req:emu.dma] */
static void update_irq(RP2350DMAState *s)
{
    int n;

    for (n = 0; n < RP2350_DMA_IRQS; n++) {
        qemu_set_irq(s->irq[n], ints(s, n) != 0);
    }
}

/* DREQs and pacing timers. */

/* The block whose ACCESSCTRL register sets the security level of a DREQ. */
static hwaddr dreq_block(unsigned dreq)
{
    static const hwaddr pio[] = { 0x50200000, 0x50300000, 0x50400000 };

    if (dreq < 24) {
        return pio[dreq / 8];
    }
    switch (dreq) {
    case 24 ... 25:
        return 0x40080000;      /* SPI0 */
    case 26 ... 27:
        return 0x40088000;      /* SPI1 */
    case 28 ... 29:
        return 0x40070000;      /* UART0 */
    case 30 ... 31:
        return 0x40078000;      /* UART1 */
    case 32 ... 43:
        return 0x400a8000;      /* PWM */
    case 44 ... 45:
        return 0x40090000;      /* I2C0 */
    case 46 ... 47:
        return 0x40098000;      /* I2C1 */
    case 48:
        return 0x400a0000;      /* ADC */
    case 49:
        return 0x400c8000;      /* XIP_CTRL (stream FIFO) */
    case 50 ... 51:
        return 0x400d0000;      /* XIP_QMI */
    case 52:
        return 0x400c0000;      /* HSTX */
    case 53:
        return 0x50700000;      /* CORESIGHT_TRACE */
    default:
        return 0x400f8000;      /* SHA256 */
    }
}

/*
 * Channels below a peripheral's security level are disconnected from its
 * DREQs.
 */
/* [spec:nuos:req:emu.dma] */
static bool dreq_visible(RP2350DMAState *s, int n, unsigned dreq)
{
    if (!s->accessctrl) {
        return true;
    }
    return ch_level(s, n) >=
           rp2350_accessctrl_dreq_level(s->accessctrl, dreq_block(dreq));
}

static void add_credit(RP2350DMAChannel *c, uint64_t pulses)
{
    c->dreq_credit = MIN(c->dreq_credit + pulses, DREQ_CREDIT_MAX);
}

/*
 * Pacing timer t emits a TREQ pulse in each cycle in which its fractional
 * accumulator passes Y: X/Y pulses per cycle, at most one. A timer with
 * X = 0 never pulses.
 */
static uint64_t pacing_pulses(RP2350DMAState *s, int t, uint64_t cycles)
{
    uint32_t x = s->pacing[t] >> 16;
    uint32_t y = s->pacing[t] & 0xffff;
    uint64_t total;

    if (!x) {
        return 0;
    }
    if (x >= y) {
        return cycles;
    }
    total = s->pacing_acc[t] + cycles * x;
    s->pacing_acc[t] = total % y;
    return total / y;
}

/* Cycles until timer t next pulses, the pulse's cycle included. */
static uint64_t pacing_next(RP2350DMAState *s, int t)
{
    uint32_t x = s->pacing[t] >> 16;
    uint32_t y = s->pacing[t] & 0xffff;

    if (!x) {
        return UINT64_MAX;
    }
    if (x >= y) {
        return 1;
    }
    return DIV_ROUND_UP(y - s->pacing_acc[t], x);
}

/* [spec:nuos:req:emu.dma] */
static void advance_pacing(RP2350DMAState *s, uint64_t cycles)
{
    int t, n;

    for (t = 0; t < RP2350_DMA_TIMERS; t++) {
        uint64_t pulses = pacing_pulses(s, t, cycles);

        if (!pulses) {
            continue;
        }
        for (n = 0; n < RP2350_DMA_CHANNELS; n++) {
            if (ctrl_treq(s->ch[n].ctrl) == TREQ_TIMER0 + t &&
                ch_level(s, n) >= timer_level(s, t)) {
                add_credit(&s->ch[n], pulses);
            }
        }
    }
}

static bool ch_active(RP2350DMAChannel *c)
{
    return (c->ctrl & (CTRL_BUSY | CTRL_EN | CTRL_ERRORS)) ==
           (CTRL_BUSY | CTRL_EN);
}

/*
 * Whether an active channel's TREQ lets it issue a transfer: a credit, a
 * DREQ held high, or the permanent request.
 */
/* [spec:nuos:req:emu.dma] */
static bool ch_requests(RP2350DMAState *s, int n)
{
    RP2350DMAChannel *c = &s->ch[n];
    unsigned treq = ctrl_treq(c->ctrl);

    if (!ch_active(c) || (s->zero_pending & (1u << n))) {
        return false;
    }
    if (treq == TREQ_PERMANENT || c->dreq_credit) {
        return true;
    }
    return treq < RP2350_DMA_NUM_DREQ && !s->edges_only &&
           (s->dreq_level >> treq & 1) && dreq_visible(s, n, treq);
}

/* The channels paced by a peripheral's DREQ. */
static uint32_t dreq_paced(RP2350DMAState *s)
{
    uint32_t mask = 0;
    int n;

    for (n = 0; n < RP2350_DMA_CHANNELS; n++) {
        if (ctrl_treq(s->ch[n].ctrl) < RP2350_DMA_NUM_DREQ) {
            mask |= 1u << n;
        }
    }
    return mask;
}

static uint32_t requesting(RP2350DMAState *s)
{
    uint32_t req = 0;
    int n;

    for (n = 0; n < RP2350_DMA_CHANNELS; n++) {
        if (ch_requests(s, n)) {
            req |= 1u << n;
        }
    }
    return req;
}

/* Cycles until a pacing timer gives a waiting channel a credit. */
static uint64_t cycles_to_pacing(RP2350DMAState *s)
{
    uint64_t next = UINT64_MAX;
    int n;

    for (n = 0; n < RP2350_DMA_CHANNELS; n++) {
        unsigned treq = ctrl_treq(s->ch[n].ctrl);
        int t = treq - TREQ_TIMER0;

        if (ch_active(&s->ch[n]) && t >= 0 && t < RP2350_DMA_TIMERS &&
            ch_level(s, n) >= timer_level(s, t)) {
            next = MIN(next, pacing_next(s, t));
        }
    }
    return next;
}

/* Channel sequencing. */

static void ch_trigger(RP2350DMAState *s, int n);

/*
 * Chaining never raises a channel's security level: a channel only
 * triggers channels at or below its own level.
 */
/* [spec:nuos:req:emu.dma] */
static void ch_chain(RP2350DMAState *s, int n)
{
    unsigned to = ctrl_chain_to(s->ch[n].ctrl);

    if (to != n && ch_level(s, to) <= ch_level(s, n)) {
        ch_trigger(s, to);
    }
}

/*
 * End of a transfer sequence: the completion interrupt (unless IRQ_QUIET),
 * CHAIN_TO, and a TRIGGER_SELF channel's restart.
 */
/* [spec:nuos:req:emu.dma] */
static void ch_complete(RP2350DMAState *s, int n, bool chain)
{
    RP2350DMAChannel *c = &s->ch[n];

    c->ctrl &= ~CTRL_BUSY;
    if (!(c->ctrl & CTRL_IRQ_QUIET)) {
        s->intr |= 1u << n;
    }
    if (chain) {
        ch_chain(s, n);
    }
    if (ch_mode(c) == MODE_TRIGGER_SELF) {
        ch_trigger(s, n);
    }
}

/*
 * A trigger starts an enabled, idle channel without an error outstanding:
 * the transfer counter reloads and BUSY rises. A channel triggered with a
 * zero count completes on the next cycle without any bus access, except
 * in ENDLESS mode, where it halts at once.
 */
/* [spec:nuos:req:emu.dma] */
static void ch_trigger(RP2350DMAState *s, int n)
{
    RP2350DMAChannel *c = &s->ch[n];
    unsigned mode;

    if ((c->ctrl & (CTRL_EN | CTRL_BUSY | CTRL_ERRORS)) != CTRL_EN) {
        return;
    }
    c->trans_count = c->reload;
    c->ctrl |= CTRL_BUSY;
    mode = ch_mode(c);
    if (mode != MODE_NORMAL && mode != MODE_TRIGGER_SELF &&
        mode != MODE_ENDLESS) {
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-dma: channel %d triggered "
                      "with reserved TRANS_COUNT mode 0x%x\n", n, mode);
    }
    if (!(c->trans_count & COUNT_MASK)) {
        if (mode == MODE_ENDLESS) {
            c->ctrl &= ~CTRL_BUSY;
        } else {
            s->zero_pending |= 1u << n;
        }
    }
}

/*
 * A bus error halts the channel without chaining and always raises its
 * interrupt; the channel refuses triggers until software clears the error.
 */
/* [spec:nuos:req:emu.dma] */
static void ch_bus_error(RP2350DMAState *s, int n, uint32_t flag)
{
    RP2350DMAChannel *c = &s->ch[n];

    c->ctrl = (c->ctrl & ~CTRL_BUSY) | flag;
    s->zero_pending &= ~(1u << n);
    s->intr |= 1u << n;
}

/*
 * CHAN_ABORT clears the transfer counter and BUSY. As erratum RP2350-E5
 * describes, the abort fires the channel's CHAIN_TO if and only if the
 * channel was the last to complete a write.
 */
/* [spec:nuos:req:emu.dma] */
static void ch_abort(RP2350DMAState *s, int n)
{
    RP2350DMAChannel *c = &s->ch[n];

    c->ctrl &= ~CTRL_BUSY;
    c->trans_count &= ~COUNT_MASK;
    s->zero_pending &= ~(1u << n);
    if (s->last_write == n) {
        ch_chain(s, n);
    }
}

/* The transfer engine. */

/*
 * The DMA MPU: the minimum channel security level for `addr`, from the
 * lowest-numbered enabled region that matches, else from MPU_CTRL.
 */
/* [spec:nuos:req:emu.dma] */
static unsigned mpu_level(RP2350DMAState *s, uint32_t addr)
{
    int i;

    for (i = 0; i < RP2350_DMA_MPU_REGIONS; i++) {
        uint32_t lar = s->mpu_lar[i];

        if ((lar & MPU_LAR_EN) &&
            (addr >> 5) >= (s->mpu_bar[i] >> 5) && (addr >> 5) <= (lar >> 5)) {
            return (lar >> 1) & 3;
        }
    }
    return (s->mpu_ctrl >> 1) & 3;
}

/*
 * One bus access by channel n, at the channel's security level. The MPU
 * checks it first and shoots it down before it reaches the bus; the bus
 * (ACCESSCTRL, decode, the target) may then fail it too.
 */
/* [spec:nuos:req:emu.dma] */
static bool bus_access(RP2350DMAState *s, int n, uint32_t addr,
                       uint32_t *data, unsigned size, bool is_write)
{
    unsigned level = ch_level(s, n);
    uint8_t buf[4];
    MemTxResult r;

    if (mpu_level(s, addr) > level) {
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-dma: channel %d %s at "
                      "0x%08" PRIx32 " denied by the DMA MPU\n", n,
                      is_write ? "write" : "read", addr);
        return false;
    }
    if (is_write) {
        stn_le_p(buf, size, *data);
    }
    r = address_space_rw(&s->as, addr, rp2350_accessctrl_dma_attrs(level),
                         buf, size, is_write);
    if (r != MEMTX_OK) {
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-dma: channel %d %s at "
                      "0x%08" PRIx32 " failed with a bus error\n", n,
                      is_write ? "write" : "read", addr);
        return false;
    }
    if (!is_write) {
        *data = ldn_le_p(buf, size);
    } else if (s->exclmon) {
        rp2350_exclmon_dma_write(s->exclmon, addr, size);
    }
    if (s->busctrl) {
        rp2350_busctrl_dma_access(s->busctrl, addr, size);
    }
    return true;
}

/*
 * The next address after a transfer: the increment is the transfer size
 * times 1, -1 (REV), 2 (REV without INCR) or 0; a ring keeps all but the
 * low RING_SIZE bits of the address fixed.
 */
/* [spec:nuos:req:emu.dma] */
static uint32_t next_addr(uint32_t ctrl, uint32_t addr, unsigned size,
                          bool is_write)
{
    bool incr = ctrl & (is_write ? CTRL_INCR_WRITE : CTRL_INCR_READ);
    bool rev = ctrl & (is_write ? CTRL_INCR_WRITE_REV : CTRL_INCR_READ_REV);
    unsigned ring = extract32(ctrl, CTRL_RING_SIZE_SHIFT, 4);
    uint32_t next = addr;

    if (incr) {
        next += rev ? -size : size;
    } else if (rev) {
        next += 2 * size;
    }
    if (ring && !!(ctrl & CTRL_RING_SEL) == is_write) {
        uint32_t mask = (1u << ring) - 1;

        next = (addr & ~mask) | (next & mask);
    }
    return next;
}

static uint32_t swap_bytes(uint32_t data, unsigned size)
{
    switch (size) {
    case 2:
        return bswap16(data);
    case 4:
        return bswap32(data);
    default:
        return data;
    }
}

/* Feed the low `bits` bits of `data`, most significant first. */
static uint32_t crc_feed(uint32_t crc, uint32_t data, unsigned bits,
                         unsigned width, uint32_t poly)
{
    uint32_t top = 1u << (width - 1);
    int i;

    for (i = bits - 1; i >= 0; i--) {
        bool fb = !!(crc & top) ^ ((data >> i) & 1);

        crc <<= 1;
        if (fb) {
            crc ^= poly;
        }
    }
    return crc;
}

/*
 * The sniffer sees each word the observed channel reads, after the
 * channel's byte swap. The CRCs shift data in most significant bit first;
 * the bit-reversed variants reverse the whole transfer first, so that
 * CRC32R with OUT_REV and OUT_INV computes the IEEE 802.3 CRC of the bytes
 * in memory order. CRC-16 runs in the low half of SNIFF_DATA.
 */
/* [spec:nuos:req:emu.dma] */
static void sniff(RP2350DMAState *s, int n, uint32_t data, unsigned size)
{
    uint32_t ctrl = s->sniff_ctrl;
    unsigned bits = size * 8;
    uint32_t crc16;

    if (!(ctrl & SNIFF_EN) ||
        extract32(ctrl, SNIFF_DMACH_SHIFT, 4) != n ||
        !(s->ch[n].ctrl & CTRL_SNIFF_EN) ||
        ch_level(s, n) > sniff_level(s)) {
        return;
    }
    if (ctrl & SNIFF_BSWAP) {
        data = swap_bytes(data, size);
    }
    switch (extract32(ctrl, SNIFF_CALC_SHIFT, 4)) {
    case CALC_CRC32R:
        data = revbit32(data) >> (32 - bits);
        /* fall through */
    case CALC_CRC32:
        s->sniff_data = crc_feed(s->sniff_data, data, bits, 32, 0x04c11db7);
        break;
    case CALC_CRC16R:
        data = revbit32(data) >> (32 - bits);
        /* fall through */
    case CALC_CRC16:
        crc16 = crc_feed(s->sniff_data, data, bits, 16, 0x1021) & 0xffff;
        s->sniff_data = (s->sniff_data & 0xffff0000u) | crc16;
        break;
    case CALC_EVEN:
        s->sniff_data ^= ctpop32(data) & 1;
        break;
    case CALC_SUM:
        s->sniff_data += data;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-dma: SNIFF_CTRL.CALC 0x%x "
                      "is reserved\n", extract32(ctrl, SNIFF_CALC_SHIFT, 4));
        break;
    }
}

/*
 * One transfer: a read, then the paired write. A failed read suppresses
 * the write; either failure halts the channel with its address registers
 * pointing past the faulting transfer.
 */
/* [spec:nuos:req:emu.dma] */
static void ch_transfer(RP2350DMAState *s, int n)
{
    RP2350DMAChannel *c = &s->ch[n];
    unsigned size_field = extract32(c->ctrl, CTRL_DATA_SIZE_SHIFT, 2);
    unsigned size = size_field == 3 ? 4 : 1u << size_field;
    uint32_t raddr = c->read_addr;
    uint32_t waddr = c->write_addr;
    uint32_t data = 0;
    bool ok;

    if (size_field == 3) {
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-dma: channel %d DATA_SIZE 3 "
                      "is reserved\n", n);
    }

    ok = bus_access(s, n, raddr, &data, size, false);
    c->read_addr = next_addr(c->ctrl, raddr, size, false);
    if (!ok) {
        c->write_addr = next_addr(c->ctrl, waddr, size, true);
        ch_bus_error(s, n, CTRL_READ_ERROR);
        return;
    }
    if (c->ctrl & CTRL_BSWAP) {
        data = swap_bytes(data, size);
    }
    sniff(s, n, data, size);

    c->write_addr = next_addr(c->ctrl, waddr, size, true);
    if (!bus_access(s, n, waddr, &data, size, true)) {
        ch_bus_error(s, n, CTRL_WRITE_ERROR);
        return;
    }
    s->last_write = n;

    /* The write may have reprogrammed or aborted this very channel. */
    if (!(c->ctrl & CTRL_BUSY) || ch_mode(c) == MODE_ENDLESS) {
        return;
    }
    if (c->trans_count & COUNT_MASK) {
        c->trans_count--;
    }
    if (!(c->trans_count & COUNT_MASK)) {
        ch_complete(s, n, true);
    }
}

static int next_channel(uint32_t mask, uint32_t last)
{
    int i;

    for (i = 1; i <= RP2350_DMA_CHANNELS; i++) {
        int n = (last + i) % RP2350_DMA_CHANNELS;

        if (mask & (1u << n)) {
            return n;
        }
    }
    g_assert_not_reached();
}

/*
 * Each scheduling round serves every requesting high-priority channel,
 * then a single low-priority channel; within each class, round-robin.
 */
/* [spec:nuos:req:emu.dma] */
static int arbitrate(RP2350DMAState *s, uint32_t req)
{
    uint32_t high = 0, low;
    int n;

    for (n = 0; n < RP2350_DMA_CHANNELS; n++) {
        if (s->ch[n].ctrl & CTRL_HIGH_PRIORITY) {
            high |= 1u << n;
        }
    }
    high &= req;
    low = req & ~high;

    if (high & ~s->hp_served) {
        n = next_channel(high & ~s->hp_served, s->rr_high);
        s->rr_high = n;
        s->hp_served |= 1u << n;
        return n;
    }
    s->hp_served = 0;
    if (low) {
        n = next_channel(low, s->rr_low);
        s->rr_low = n;
        return n;
    }
    n = next_channel(high, s->rr_high);
    s->rr_high = n;
    s->hp_served = 1u << n;
    return n;
}

/* Whether the memory core would refuse an access to `mr` as re-entrant. */
static bool region_in_access(MemoryRegion *mr)
{
    return mr->dev && !mr->disable_reentrancy_guard && !mr->ram &&
           !mr->ram_device && !mr->rom_device && !mr->readonly &&
           mr->dev->mem_reentrancy_guard.engaged_in_io;
}

/*
 * Whether an access to `addr` would reach a device that is in the middle
 * of a register access: one of a DREQ change's callers, which the memory
 * core refuses to re-enter. An ACCESSCTRL gate in front of the device is
 * looked through.
 */
static bool target_in_access(RP2350DMAState *s, uint32_t addr)
{
    MemoryRegion *mr;
    hwaddr xlat, len = 1;

    RCU_READ_LOCK_GUARD();
    mr = address_space_translate(&s->as, addr, &xlat, &len, false,
                                 MEMTXATTRS_UNSPECIFIED);
    if (region_in_access(mr)) {
        return true;
    }
    if (s->accessctrl && mr->owner == OBJECT(s->accessctrl)) {
        len = 1;
        mr = address_space_translate(&s->accessctrl->bus_as, addr, &xlat,
                                     &len, false, MEMTXATTRS_UNSPECIFIED);
        return region_in_access(mr);
    }
    return false;
}

static bool ch_targets_free(RP2350DMAState *s, int n)
{
    return !target_in_access(s, s->ch[n].read_addr) &&
           !target_in_access(s, s->ch[n].write_addr);
}

/*
 * Run the engine up to now and, outside ENGINE_SYNC, past now while a
 * DREQ-paced channel requests, by at most ENGINE_QUANTUM cycles. Returns
 * false if an ENGINE_SERVICE run stopped short of a transfer that would
 * re-enter a device in the middle of a register access.
 */
/* [spec:nuos:req:emu.dma] */
static bool engine_run(RP2350DMAState *s, EngineMode mode)
{
    uint64_t target = now_cycle(s);
    uint64_t limit = MAX(s->cycle, target) + ENGINE_QUANTUM;
    bool done = true;

    /* Nothing runs, not even ahead of now, while clk_sys is stopped. */
    if (s->running || !clock_is_enabled(s->clk)) {
        return true;
    }
    s->running = true;
    for (;;) {
        bool ahead = s->cycle >= target;
        uint32_t req, hp_served, rr_high, rr_low;
        int n;

        if (ahead && (mode == ENGINE_SYNC || s->cycle >= limit)) {
            break;
        }
        if (s->zero_pending) {
            uint32_t pending = s->zero_pending;

            if (mode == ENGINE_SYNC ||
                (ahead && !(requesting(s) & dreq_paced(s)))) {
                break;
            }
            /*
             * Erratum RP2350-E8: a zero-length sequence chains only if
             * its channel was the last to complete a write.
             */
            advance_pacing(s, 1);
            s->zero_pending = 0;
            for (n = 0; n < RP2350_DMA_CHANNELS; n++) {
                if (pending & (1u << n)) {
                    ch_complete(s, n, s->last_write == n);
                }
            }
            s->cycle++;
            continue;
        }

        req = requesting(s);
        if (ahead && !(req & dreq_paced(s))) {
            break;
        }
        if (!req) {
            uint64_t step = MIN(target - s->cycle, cycles_to_pacing(s));

            advance_pacing(s, step);
            s->cycle += step;
            continue;
        }
        if (mode == ENGINE_SYNC) {
            break;
        }
        hp_served = s->hp_served;
        rr_high = s->rr_high;
        rr_low = s->rr_low;
        n = arbitrate(s, req);
        if (mode == ENGINE_SERVICE && !ch_targets_free(s, n)) {
            /* The bottom half takes over from this very arbitration. */
            s->hp_served = hp_served;
            s->rr_high = rr_high;
            s->rr_low = rr_low;
            done = false;
            break;
        }
        advance_pacing(s, 1);
        if (s->ch[n].dreq_credit) {
            s->ch[n].dreq_credit--;
        }
        ch_transfer(s, n);
        s->cycle++;
    }
    s->running = false;
    update_irq(s);
    return done;
}

/* Arm the engine's timer for the next cycle with work to do. */
static void engine_kick(RP2350DMAState *s)
{
    uint32_t req;
    uint64_t wait;
    int n;

    if (s->running) {
        return;
    }
    req = requesting(s);
    if (s->zero_pending || req) {
        /*
         * Enough cycles for the transfers that are certainly due: a
         * channel paced by credits alone has only those to do.
         */
        wait = 0;
        for (n = 0; n < RP2350_DMA_CHANNELS; n++) {
            RP2350DMAChannel *c = &s->ch[n];
            unsigned treq = ctrl_treq(c->ctrl);
            bool level = treq == TREQ_PERMANENT ||
                         (treq < RP2350_DMA_NUM_DREQ &&
                          (s->dreq_level >> treq & 1));

            if (!(req & (1u << n))) {
                continue;
            }
            if (!level) {
                wait += c->dreq_credit;
            } else if (ch_mode(c) == MODE_ENDLESS) {
                wait += ENGINE_QUANTUM;
            } else {
                wait += c->trans_count & COUNT_MASK;
            }
        }
        wait = MAX(1, MIN(wait, ENGINE_QUANTUM));
    } else {
        wait = cycles_to_pacing(s);
        if (wait == UINT64_MAX) {
            timer_del(s->timer);
            return;
        }
    }
    if (!clock_is_enabled(s->clk)) {
        timer_del(s->timer);
        return;
    }
    timer_mod(s->timer, cycle_ns(s, s->cycle + wait));
}

/* The engine's timer and bottom half. */
static void engine_wake(void *opaque)
{
    RP2350DMAState *s = opaque;

    engine_run(s, ENGINE_ISSUE);
    engine_kick(s);
}

/*
 * A DREQ input. A rising edge is a credit for each busy channel that
 * paces on it and can see it (paused with EN clear or not), answered at
 * once: in place, unless a vCPU is making a register access or a transfer
 * would re-enter a device in the middle of one, else from the bottom
 * half. A level held high requests further transfers, which the engine's
 * timer issues. Answering in place, the engine heeds credits only: the
 * source may yet lower this line, as a pulse does, and it and other
 * sources may be part way through updating their lines, when their levels
 * are not yet to be trusted. A channel that is not busy takes no credits:
 * on hardware it starts its DREQ handshake afresh when triggered, so edges
 * from before (from whatever its TREQ_SEL then selected, DREQ_PIO0_TX0 at
 * reset) do not count.
 */
/* [spec:nuos:req:emu.dma] */
static void dreq_set(void *opaque, int dreq, int level)
{
    RP2350DMAState *s = opaque;
    bool rising = level && !(s->dreq_level >> dreq & 1);
    int n;

    engine_run(s, ENGINE_SYNC);
    s->dreq_level = deposit64(s->dreq_level, dreq, 1, level != 0);
    if (rising) {
        for (n = 0; n < RP2350_DMA_CHANNELS; n++) {
            if (ctrl_treq(s->ch[n].ctrl) == dreq &&
                (s->ch[n].ctrl & CTRL_BUSY) && dreq_visible(s, n, dreq)) {
                add_credit(&s->ch[n], 1);
            }
        }
    }
    if (s->running) {
        /* The engine's own transfer changed a DREQ: it sees the change. */
        return;
    }
    if (rising) {
        s->edges_only = true;
        if ((requesting(s) & dreq_paced(s)) &&
            (current_cpu || !engine_run(s, ENGINE_SERVICE))) {
            qemu_bh_schedule(s->bh);
        }
        s->edges_only = false;
    }
    engine_kick(s);
}

/* Registers. */

static void bad_offset(const char *what, hwaddr addr)
{
    qemu_log_mask(LOG_GUEST_ERROR, "rp2350-dma: %s of reserved offset "
                  "0x%" HWADDR_PRIx "\n", what, addr);
}

static MemTxResult denied(const char *what, hwaddr reg, unsigned level)
{
    qemu_log_mask(LOG_GUEST_ERROR, "rp2350-dma: %s of 0x%03" HWADDR_PRIx
                  " at security level %u denied\n", what, reg, level);
    return MEMTX_ERROR;
}

static uint32_t ctrl_read(RP2350DMAChannel *c)
{
    return c->ctrl | (c->ctrl & CTRL_ERRORS ? CTRL_AHB_ERROR : 0);
}

/*
 * Whether an access at `level` may use IRQ n's INTE/INTF/INTS.
 * Registers of an IRQ above the access's level fault.
 */
static bool irq_regs_allowed(RP2350DMAState *s, int n, unsigned level)
{
    return level >= irq_level(s, n);
}

/*
 * NSP may see an MPU region's addresses only while the region is
 * Non-secure and MPU_CTRL.NS_HIDE_ADDR is clear.
 */
static bool mpu_addr_visible(RP2350DMAState *s, int i, unsigned level)
{
    return level == LEVEL_SP ||
           (!(s->mpu_lar[i] & MPU_LAR_S) &&
            !(s->mpu_ctrl & MPU_CTRL_NS_HIDE_ADDR));
}

/* [spec:nuos:req:emu.dma] */
static MemTxResult reg_read(RP2350DMAState *s, hwaddr reg, unsigned level,
                            uint32_t *value, const char *what)
{
    int n;

    *value = 0;
    if (reg < CH_END) {
        RP2350DMAChannel *c;

        n = reg / CH_STRIDE;
        c = &s->ch[n];
        if (level < ch_level(s, n)) {
            return denied(what, reg, level);
        }
        switch (alias_layout[(reg >> 4) & 3][(reg >> 2) & 3]) {
        case REG_READ_ADDR:
            *value = c->read_addr;
            break;
        case REG_WRITE_ADDR:
            *value = c->write_addr;
            break;
        case REG_TRANS_COUNT:
            *value = c->trans_count;
            break;
        default:
            *value = ctrl_read(c);
            break;
        }
        return MEMTX_OK;
    }
    if (reg >= A_DBG_BASE && reg < A_DBG_END) {
        n = (reg - A_DBG_BASE) / CH_STRIDE;
        switch (reg & (CH_STRIDE - 1)) {
        case A_DBG_CTDREQ:
        case A_DBG_TCR:
            if (level < ch_level(s, n)) {
                return denied(what, reg, level);
            }
            *value = reg & 4 ? s->ch[n].reload : s->ch[n].dreq_credit;
            return MEMTX_OK;
        default:
            bad_offset(what, reg);
            return MEMTX_OK;
        }
    }

    switch (reg) {
    case A_INTR:
        *value = s->intr & channels_at_or_below(s, level);
        break;
    case A_IRQ_BASE + 0x04 ... A_IRQ_BASE + 0x3c:
        n = (reg - A_IRQ_BASE) / 0x10;
        if ((reg & 0xc) == 0) {
            bad_offset(what, reg);
            break;
        }
        if (!irq_regs_allowed(s, n, level)) {
            return denied(what, reg, level);
        }
        switch (reg & 0xc) {
        case 0x4:
            *value = s->inte[n];
            break;
        case 0x8:
            *value = s->intf[n];
            break;
        default:
            *value = ints(s, n);
            break;
        }
        break;
    case A_TIMER0 ... A_TIMER3:
        n = (reg - A_TIMER0) / 4;
        if (level < timer_level(s, n)) {
            return denied(what, reg, level);
        }
        *value = s->pacing[n];
        break;
    case A_MULTI_CHAN_TRIGGER:
    case A_FIFO_LEVELS:
    case A_CHAN_ABORT:
        break;
    case A_SNIFF_CTRL:
    case A_SNIFF_DATA:
        if (level < sniff_level(s)) {
            return denied(what, reg, level);
        }
        if (reg == A_SNIFF_CTRL) {
            *value = s->sniff_ctrl;
            break;
        }
        /* The output transforms act between the accumulator and the bus. */
        *value = s->sniff_data;
        if (s->sniff_ctrl & SNIFF_OUT_REV) {
            *value = revbit32(*value);
        }
        if (s->sniff_ctrl & SNIFF_OUT_INV) {
            *value = ~*value;
        }
        break;
    case A_N_CHANNELS:
        *value = RP2350_DMA_CHANNELS;
        break;
    case A_SECCFG_CH0 ... A_SECCFG_CH15:
        *value = s->ch[(reg - A_SECCFG_CH0) / 4].seccfg;
        break;
    case A_SECCFG_IRQ0 ... A_SECCFG_IRQ3:
        *value = s->seccfg_irq[(reg - A_SECCFG_IRQ0) / 4];
        break;
    case A_SECCFG_MISC:
        *value = s->seccfg_misc;
        break;
    case A_MPU_CTRL:
        if (!level_privileged(level)) {
            return denied(what, reg, level);
        }
        *value = s->mpu_ctrl;
        break;
    case A_MPU_BAR0 ... A_MPU_LAR7:
        if (!level_privileged(level)) {
            return denied(what, reg, level);
        }
        n = (reg - A_MPU_BAR0) / 8;
        if (reg & 4) {
            *value = s->mpu_bar[n];
            if (!mpu_addr_visible(s, n, level)) {
                *value = 0;
            }
        } else {
            *value = s->mpu_lar[n];
            if (!mpu_addr_visible(s, n, level)) {
                *value &= ~MPU_ADDR_MASK;
            }
        }
        break;
    default:
        bad_offset(what, reg);
        break;
    }
    return MEMTX_OK;
}

/*
 * A trigger register write starts the channel; writing zero instead is a
 * null trigger, which raises the channel's interrupt if it is IRQ_QUIET.
 */
/* [spec:nuos:req:emu.dma] */
static void ch_reg_write(RP2350DMAState *s, int n, int which, bool trigger,
                         uint32_t written, uint32_t strobe)
{
    RP2350DMAChannel *c = &s->ch[n];

    switch (which) {
    case REG_READ_ADDR:
        c->read_addr = written;
        break;
    case REG_WRITE_ADDR:
        c->write_addr = written;
        break;
    case REG_TRANS_COUNT:
        c->reload = written;
        break;
    default:
        c->ctrl = (c->ctrl & ~CTRL_RW_MASK) | (written & CTRL_RW_MASK);
        /* READ_ERROR and WRITE_ERROR are write-one-to-clear. */
        c->ctrl &= ~(written & strobe & CTRL_ERRORS);
        break;
    }
    /* A successful write to a control register locks SECCFG_CHn. */
    c->seccfg |= SECCFG_LOCK;

    if (trigger) {
        if (written) {
            ch_trigger(s, n);
        } else if (c->ctrl & CTRL_IRQ_QUIET) {
            s->intr |= 1u << n;
        }
    }
}

/*
 * The SECCFG registers take privileged writes only. Secure, privileged
 * code may write every field; Non-secure, privileged code may write a P
 * bit whose S bit is clear.
 */
/* [spec:nuos:req:emu.dma] */
static uint32_t seccfg_write(uint32_t old, uint32_t written, unsigned level,
                             int pairs)
{
    int i;

    if (level == LEVEL_SP) {
        return written;
    }
    for (i = 0; i < pairs; i++) {
        uint32_t p = SECCFG_P << (2 * i);
        uint32_t sbit = SECCFG_S << (2 * i);

        if (!(old & sbit)) {
            old = (old & ~p) | (written & p);
        }
    }
    return old;
}

/* [spec:nuos:req:emu.dma] */
static MemTxResult reg_write(RP2350DMAState *s, hwaddr addr, uint32_t value,
                             unsigned level)
{
    hwaddr reg = rp2350_atomic_reg(addr);
    uint32_t strobe = addr >= RP2350_ATOMIC_ALIAS_SIZE ? value : UINT32_MAX;
    uint32_t old, written, mask;
    MemTxResult r;
    int n;

    /*
     * An atomic alias acts on the register's value; a write-one-to-clear
     * bit is cleared only where the alias writes a one to it.
     */
    r = reg_read(s, reg, level, &old, "write");
    if (r != MEMTX_OK) {
        return r;
    }
    written = rp2350_atomic_apply(addr, old, value);

    if (reg < CH_END) {
        int idx = (reg >> 2) & 3;

        n = reg / CH_STRIDE;
        ch_reg_write(s, n, alias_layout[(reg >> 4) & 3][idx], idx == 3,
                     written, strobe);
        return MEMTX_OK;
    }
    if (reg >= A_DBG_BASE && reg < A_DBG_END) {
        n = (reg - A_DBG_BASE) / CH_STRIDE;
        if ((reg & (CH_STRIDE - 1)) == A_DBG_CTDREQ) {
            /* Any write clears the counter and restarts the handshake. */
            s->ch[n].dreq_credit = 0;
        }
        return MEMTX_OK;
    }

    switch (reg) {
    case A_INTR:
        s->intr &= ~(written & strobe & channels_at_or_below(s, level));
        break;
    case A_IRQ_BASE + 0x04 ... A_IRQ_BASE + 0x3c:
        n = (reg - A_IRQ_BASE) / 0x10;
        mask = channels_at_or_below(s, irq_level(s, n));
        switch (reg & 0xc) {
        case 0x4:
            s->inte[n] = written & ALL_CHANNELS;
            break;
        case 0x8:
            /* Channels above the IRQ's level cannot be forced through it. */
            s->intf[n] = (s->intf[n] & ~mask) | (written & mask);
            break;
        case 0xc:
            s->intr &= ~(written & strobe & mask);
            break;
        default:
            break;
        }
        break;
    case A_TIMER0 ... A_TIMER3:
        s->pacing[(reg - A_TIMER0) / 4] = written;
        break;
    case A_MULTI_CHAN_TRIGGER:
        mask = written & channels_at_or_below(s, level);
        for (n = 0; n < RP2350_DMA_CHANNELS; n++) {
            if (mask & (1u << n)) {
                ch_trigger(s, n);
            }
        }
        break;
    case A_CHAN_ABORT:
        mask = written & channels_at_or_below(s, level);
        for (n = 0; n < RP2350_DMA_CHANNELS; n++) {
            if (mask & (1u << n)) {
                ch_abort(s, n);
            }
        }
        break;
    case A_SNIFF_CTRL:
        s->sniff_ctrl = written & SNIFF_CTRL_MASK;
        break;
    case A_SNIFF_DATA:
        s->sniff_data = written;
        break;
    case A_FIFO_LEVELS:
    case A_N_CHANNELS:
        break;
    case A_SECCFG_CH0 ... A_SECCFG_CH15:
        if (!level_privileged(level)) {
            return denied("write", reg, level);
        }
        n = (reg - A_SECCFG_CH0) / 4;
        /* A locked SECCFG_CHn is read-only until the DMA resets. */
        if (!(s->ch[n].seccfg & SECCFG_LOCK)) {
            s->ch[n].seccfg = seccfg_write(s->ch[n].seccfg, written, level,
                                           1) & 7;
        }
        break;
    case A_SECCFG_IRQ0 ... A_SECCFG_IRQ3:
        if (!level_privileged(level)) {
            return denied("write", reg, level);
        }
        n = (reg - A_SECCFG_IRQ0) / 4;
        s->seccfg_irq[n] = seccfg_write(s->seccfg_irq[n], written, level,
                                        1) & 3;
        break;
    case A_SECCFG_MISC:
        if (!level_privileged(level)) {
            return denied("write", reg, level);
        }
        s->seccfg_misc = seccfg_write(s->seccfg_misc, written, level, 5) &
                         SECCFG_MISC_RESET;
        break;
    case A_MPU_CTRL:
        /* Privileged-only; read-only to Non-secure. */
        if (level == LEVEL_SP) {
            s->mpu_ctrl = written & MPU_CTRL_MASK;
        }
        break;
    case A_MPU_BAR0 ... A_MPU_LAR7:
        n = (reg - A_MPU_BAR0) / 8;
        if (reg & 4) {
            if (level == LEVEL_SP) {
                s->mpu_bar[n] = written & MPU_ADDR_MASK;
            }
        } else if (level == LEVEL_SP) {
            s->mpu_lar[n] = written & MPU_LAR_MASK;
        } else if (!(s->mpu_lar[n] & MPU_LAR_S)) {
            /* NSP decides whether a Non-secure region is NSU-accessible. */
            s->mpu_lar[n] = (s->mpu_lar[n] & ~MPU_LAR_P) |
                            (written & MPU_LAR_P);
        }
        break;
    default:
        break;
    }
    return MEMTX_OK;
}

/*
 * The registers are 32 bits wide: a narrow write is replicated across the
 * bus and writes the whole register; a narrow read returns its lanes.
 */
/* [spec:nuos:req:emu.dma] */
static MemTxResult rp2350_dma_read(void *opaque, hwaddr addr, uint64_t *data,
                                   unsigned size, MemTxAttrs attrs)
{
    RP2350DMAState *s = opaque;
    uint32_t v;
    MemTxResult r;

    engine_run(s, ENGINE_SYNC);
    r = reg_read(s, rp2350_atomic_reg(addr & ~3), access_level(attrs), &v,
                 "read");
    *data = extract32(v, (addr & 3) * 8, size * 8);
    return r;
}

/* [spec:nuos:req:emu.dma] */
static MemTxResult rp2350_dma_write(void *opaque, hwaddr addr, uint64_t value,
                                    unsigned size, MemTxAttrs attrs)
{
    RP2350DMAState *s = opaque;
    uint32_t v = value;
    MemTxResult r;

    if (size == 1) {
        v = (v & 0xff) * 0x01010101u;
    } else if (size == 2) {
        v = (v & 0xffff) * 0x00010001u;
    }
    engine_run(s, ENGINE_SYNC);
    r = reg_write(s, addr & ~3, v, access_level(attrs));
    update_irq(s);
    /* A trigger, say, may leave a DREQ-paced channel requesting. */
    if (requesting(s) & dreq_paced(s)) {
        qemu_bh_schedule(s->bh);
    }
    engine_kick(s);
    return r;
}

static const MemoryRegionOps rp2350_dma_ops = {
    .read_with_attrs = rp2350_dma_read,
    .write_with_attrs = rp2350_dma_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

/* SIO and the PPB are core-local: the DMA's accesses there fail decode. */
static MemTxResult port_hole_read(void *opaque, hwaddr addr, uint64_t *data,
                                  unsigned size, MemTxAttrs attrs)
{
    return MEMTX_DECODE_ERROR;
}

static MemTxResult port_hole_write(void *opaque, hwaddr addr, uint64_t value,
                                   unsigned size, MemTxAttrs attrs)
{
    return MEMTX_DECODE_ERROR;
}

static const MemoryRegionOps port_hole_ops = {
    .read_with_attrs = port_hole_read,
    .write_with_attrs = port_hole_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

#define CORE_LOCAL_BASE 0xd0000000ull

static void rp2350_dma_reset_hold(Object *obj, ResetType type)
{
    RP2350DMAState *s = RP2350_DMA(obj);
    int n;

    timer_del(s->timer);
    qemu_bh_cancel(s->bh);
    for (n = 0; n < RP2350_DMA_CHANNELS; n++) {
        s->ch[n] = (RP2350DMAChannel) {
            .seccfg = SECCFG_S | SECCFG_P,
        };
    }
    s->intr = 0;
    for (n = 0; n < RP2350_DMA_IRQS; n++) {
        s->inte[n] = 0;
        s->intf[n] = 0;
        s->seccfg_irq[n] = SECCFG_S | SECCFG_P;
    }
    for (n = 0; n < RP2350_DMA_TIMERS; n++) {
        s->pacing[n] = 0;
        s->pacing_acc[n] = 0;
    }
    s->sniff_ctrl = 0;
    s->sniff_data = 0;
    s->seccfg_misc = SECCFG_MISC_RESET;
    s->mpu_ctrl = 0;
    memset(s->mpu_bar, 0, sizeof(s->mpu_bar));
    memset(s->mpu_lar, 0, sizeof(s->mpu_lar));
    s->zero_pending = 0;
    s->last_write = -1;
    s->hp_served = 0;
    s->rr_high = RP2350_DMA_CHANNELS - 1;
    s->rr_low = RP2350_DMA_CHANNELS - 1;
    s->base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->base_cycle = 0;
    s->cycle = 0;
}

static void rp2350_dma_reset_exit(Object *obj, ResetType type)
{
    update_irq(RP2350_DMA(obj));
}

/*
 * clk_sys changes rate, starts or stops: cycles up to now count at the old
 * rate, and the engine's timer is re-armed for the new one.
 */
static void rp2350_dma_clk_changed(void *opaque, ClockEvent event)
{
    RP2350DMAState *s = opaque;

    if (event == ClockPreUpdate) {
        s->base_cycle = now_cycle(s);
        s->base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        return;
    }
    if (s->timer) {
        engine_kick(s);
    }
}

static void rp2350_dma_init(Object *obj)
{
    RP2350DMAState *s = RP2350_DMA(obj);
    int n;

    memory_region_init_io(&s->iomem, obj, &rp2350_dma_ops, s,
                          TYPE_RP2350_DMA, RP2350_ATOMIC_REGION_SIZE);
    /*
     * A channel may program the DMA itself; such writes arrive while the
     * engine runs a transfer, and are not re-entrant in effect.
     */
    s->iomem.disable_reentrancy_guard = true;
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    for (n = 0; n < RP2350_DMA_IRQS; n++) {
        sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq[n]);
    }
    qdev_init_gpio_in_named(DEVICE(obj), dreq_set, RP2350_DMA_DREQ,
                            RP2350_DMA_NUM_DREQ);
    s->clk = qdev_init_clock_in(DEVICE(obj), "clk", rp2350_dma_clk_changed, s,
                                ClockPreUpdate | ClockUpdate);
}

static void rp2350_dma_realize(DeviceState *dev, Error **errp)
{
    RP2350DMAState *s = RP2350_DMA(dev);
    Object *obj = OBJECT(dev);

    if (!s->bus) {
        error_setg(errp, "bus property was not set");
        return;
    }
    if (!clock_has_source(s->clk)) {
        error_setg(errp, "the clk clock must be connected");
        return;
    }
    memory_region_init(&s->port, obj, "rp2350-dma.port", UINT64_MAX);
    memory_region_init_alias(&s->port_bus, obj, "rp2350-dma.port-bus",
                             s->bus, 0, UINT64_MAX);
    memory_region_add_subregion_overlap(&s->port, 0, &s->port_bus, 0);
    memory_region_init_io(&s->port_hole, obj, &port_hole_ops, s,
                          "rp2350-dma.core-local", 0x100000000ull -
                          CORE_LOCAL_BASE);
    memory_region_add_subregion_overlap(&s->port, CORE_LOCAL_BASE,
                                        &s->port_hole, 1);
    address_space_init(&s->as, &s->port, "rp2350-dma");
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, engine_wake, s);
    s->bh = qemu_bh_new_guarded(engine_wake, s,
                                &DEVICE(s)->mem_reentrancy_guard);
}

static int rp2350_dma_post_load(void *opaque, int version_id)
{
    RP2350DMAState *s = opaque;

    if (requesting(s) & dreq_paced(s)) {
        qemu_bh_schedule(s->bh);
    }
    engine_kick(s);
    return 0;
}

static const VMStateDescription vmstate_rp2350_dma_channel = {
    .name = TYPE_RP2350_DMA "/channel",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(read_addr, RP2350DMAChannel),
        VMSTATE_UINT32(write_addr, RP2350DMAChannel),
        VMSTATE_UINT32(trans_count, RP2350DMAChannel),
        VMSTATE_UINT32(reload, RP2350DMAChannel),
        VMSTATE_UINT32(ctrl, RP2350DMAChannel),
        VMSTATE_UINT32(seccfg, RP2350DMAChannel),
        VMSTATE_UINT32(dreq_credit, RP2350DMAChannel),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_rp2350_dma = {
    .name = TYPE_RP2350_DMA,
    .version_id = 2,
    .minimum_version_id = 2,
    .post_load = rp2350_dma_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(ch, RP2350DMAState, RP2350_DMA_CHANNELS, 1,
                             vmstate_rp2350_dma_channel, RP2350DMAChannel),
        VMSTATE_UINT32(intr, RP2350DMAState),
        VMSTATE_UINT32_ARRAY(inte, RP2350DMAState, RP2350_DMA_IRQS),
        VMSTATE_UINT32_ARRAY(intf, RP2350DMAState, RP2350_DMA_IRQS),
        VMSTATE_UINT32_ARRAY(pacing, RP2350DMAState, RP2350_DMA_TIMERS),
        VMSTATE_UINT32_ARRAY(pacing_acc, RP2350DMAState, RP2350_DMA_TIMERS),
        VMSTATE_UINT32(sniff_ctrl, RP2350DMAState),
        VMSTATE_UINT32(sniff_data, RP2350DMAState),
        VMSTATE_UINT32_ARRAY(seccfg_irq, RP2350DMAState, RP2350_DMA_IRQS),
        VMSTATE_UINT32(seccfg_misc, RP2350DMAState),
        VMSTATE_UINT32(mpu_ctrl, RP2350DMAState),
        VMSTATE_UINT32_ARRAY(mpu_bar, RP2350DMAState,
                             RP2350_DMA_MPU_REGIONS),
        VMSTATE_UINT32_ARRAY(mpu_lar, RP2350DMAState,
                             RP2350_DMA_MPU_REGIONS),
        VMSTATE_UINT64(dreq_level, RP2350DMAState),
        VMSTATE_UINT32(zero_pending, RP2350DMAState),
        VMSTATE_INT32(last_write, RP2350DMAState),
        VMSTATE_UINT32(hp_served, RP2350DMAState),
        VMSTATE_UINT32(rr_high, RP2350DMAState),
        VMSTATE_UINT32(rr_low, RP2350DMAState),
        VMSTATE_UINT64(cycle, RP2350DMAState),
        VMSTATE_CLOCK(clk, RP2350DMAState),
        VMSTATE_UINT64(base_cycle, RP2350DMAState),
        VMSTATE_INT64(base_ns, RP2350DMAState),
        VMSTATE_END_OF_LIST()
    },
};

static const Property rp2350_dma_properties[] = {
    DEFINE_PROP_LINK("bus", RP2350DMAState, bus, TYPE_MEMORY_REGION,
                     MemoryRegion *),
    DEFINE_PROP_LINK("accessctrl", RP2350DMAState, accessctrl,
                     TYPE_RP2350_ACCESSCTRL, RP2350AccessCtrlState *),
    DEFINE_PROP_LINK("busctrl", RP2350DMAState, busctrl,
                     TYPE_RP2350_BUSCTRL, RP2350BusCtrlState *),
    DEFINE_PROP_LINK("exclmon", RP2350DMAState, exclmon,
                     TYPE_RP2350_EXCLMON, RP2350ExclMonState *),
};

static void rp2350_dma_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = rp2350_dma_realize;
    dc->vmsd = &vmstate_rp2350_dma;
    device_class_set_props(dc, rp2350_dma_properties);
    rc->phases.hold = rp2350_dma_reset_hold;
    rc->phases.exit = rp2350_dma_reset_exit;
}

/* [spec:nuos:req:emu.dma] */
static const TypeInfo rp2350_dma_info = {
    .name          = TYPE_RP2350_DMA,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RP2350DMAState),
    .instance_init = rp2350_dma_init,
    .class_init    = rp2350_dma_class_init,
};

static void rp2350_dma_register_types(void)
{
    type_register_static(&rp2350_dma_info);
}
type_init(rp2350_dma_register_types)
