/*
 * ESP32-P4 RISC-V trace encoder (TRACE)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapter
 * 3 (RISC-V Trace Encoder) and 2.11.4 (the core's trace interface);
 * ESP-IDF's trace_reg.h for v3, the authority for the registers and their
 * reset values (the TRM's TRACE_DATE differs), and its riscv_trace_ll.h
 * and riscv_trace_hal.c, which drive it; and the Efficient Trace for
 * RISC-V specification, version 2.0, whose reference branch trace
 * algorithm (its chapter "Reference Compressed Branch Trace Algorithm",
 * and the reference flow's encoder model) the encoder follows, with the
 * parameters of TRM table 3.3-1.
 *
 * One encoder per HP core. Started by TRIGGER_ON, or a debug trigger with
 * the trace-on action while TRACE_DM_TRIGGER_ENA is set, it traces every
 * instruction the core retires and every trap it takes (the core reports
 * each instruction's start, and with it its predecessor's retirement, and
 * each trap: target/riscv/tcg/esp32p4_helper.c), as the hardware's
 * three-stage pipeline would, one instruction at a time: branches go into
 * the branch map, and packets go out for the first instruction traced,
 * the target of every uninferable jump (JALR and its compressed forms,
 * Zcmp's returns, Zcmt's table jumps, the trap returns and a hardware
 * loop's jump back to its start), the last instruction before a trap and
 * the first of its handler, a privilege change, a full branch map (31
 * branches), resynchronisation and the end of qualification or of
 * tracing. The packets are those of TRM 3.6: a header (the packet's
 * length), a 16-bit index counting packets from the start, and the
 * payload of format 1 (branch map, with or without an address), format 2
 * (address only) or format 3 (synchronisation, exception, support), each
 * field least significant bit first; addresses are halved, and in delta
 * address mode (TRACE_FULL_ADDRESS clear) formats 1 and 2 give the
 * difference from the last address reported in the fewest bytes, 1 to 4,
 * that sign-extend to it. The notify and updiscon bits are sent as the
 * specification compresses them, equal to the address's most significant
 * bit unless they report something. After every 128 packets come 14 zero
 * bytes, the anchor that lets a decoder find a packet's start.
 *
 * Filtering (TRACE_FILTER_EN): an instruction is qualified, and traced,
 * when each match the filter enables holds: its privilege mode
 * (MATCH_PRIVILEGE), the handling of an exception of the chosen cause
 * (MATCH_ECAUSE) or of a trap of the chosen type (MATCH_INTERRUPT: an
 * interrupt or an exception), from the trap to the return from it, and
 * the comparators (MATCH_COMP) on the instruction's address or a trap's
 * value, primary alone, both, not both, or from the primary's match until
 * after the secondary's; a comparator with its NOTIFY bit set notifies the
 * instruction it matches rather than filtering. The end of qualification
 * reports the last qualified instruction and a support packet; a debug
 * trigger with the notify action, the instruction it fires on.
 *
 * Resynchronisation (TRACE_RESYNC_MODE): a synchronisation packet once
 * RESYNC_PROLONGED packets have gone out since the last (mode 2), or
 * RESYNC_PROLONGED cycles of TRACE_CPU_CLK, the core's clock, have passed
 * (mode 3), the branch map's branches first reported as the
 * specification's FSM has it. The cycles are counted as the core's
 * instructions, one each, as the machine runs a core under icount: the
 * cycles a core spends waiting in WFI are not counted.
 *
 * The packets go through the 128-byte FIFO to the trace memory,
 * [MEM_START_ADDR, MEM_END_ADDR), written at MEM_CURRENT_ADDR byte by
 * byte by the encoder's AHB master (DMA master TRACE0 or TRACE1, its
 * accesses checked by the DMA APM), which raises MEM_FULL_INTR as the
 * address reaches MEM_END_ADDR and goes back to MEM_START_ADDR in loop
 * mode (TRACE_MEM_LOOP), or drops what follows. The master writes at
 * once while TRACE_SYS_CLK runs and the bus takes its writes. Its write
 * into a memory controller's window (a trace memory in PSRAM, through
 * 0x48000000) waits while the controller's AXI port holds accesses (its
 * clocks stopped, held in reset, AXI_REQ_EN clear), as every DMA master's
 * transaction does, MEM_CURRENT_ADDR staying at the waiting byte, and is
 * made once the port takes accesses again; a port a reset release leaves
 * closed answers it with an error, which loses the bytes (logged): the
 * TRM says nothing of what the master does then, and it goes on with the
 * next. So the FIFO fills while the clock is stopped or the write waits:
 * a packet that does not fit is lost, raising FIFO_OVERFLOW_INTR and
 * stopping the encoder (WORK_STATUS 3) until the FIFO empties, when, with
 * TRACE_RESTART_ENA, it starts again with a support packet reporting the
 * loss (TRM 3.4: "When the memory bandwidth is insufficient, the FIFO may
 * overflow, and packet loss occurs"). The core is not held back by it:
 * only with TRACE_STALL_ENA (TRM 3.5.3, 3.8.1) is the stall request
 * asserted to the core while the FIFO is almost full
 * (ESP32P4_TRACE_STALL_ROOM), the core stopping before the instruction it
 * is starting until the FIFO drains, so that no packet is lost (lossless
 * trace); a stall request no event ends stops the core for good, as on
 * the chip. As TRM 3.5.7 has it, with TRACE_RESTART_ENA set the encoder
 * restarts once the FIFO is empty however it stopped, TRIGGER_OFF
 * included, which is why ESP-IDF clears it before stopping.
 * TRACE_RESET_ENA stops the trace while the core is held in reset, as for
 * TRIGGER_OFF but waiting (WORK_STATUS 2), and starts it again with the
 * core; the core is never halted, there being no debugger. COREn_TRACE_INTR
 * (sources 97, 98) is the OR of the enabled interrupts.
 *
 * Not modelled: implicit exception mode (TRACE_IMPLICIT_EXCEPT, logged:
 * exception packets keep their address, as the TRM's support packet says
 * the mode is not implemented); the AHB burst settings, which change no
 * data; and the register clock gate, CLOCK_GATE. Starting the encoder
 * takes effect from the core's next block of instructions, and stopping it
 * from a debug trigger, at the instruction the trigger fires on.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "qemu/main-loop.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/esp32p4_icm.h"
#include "hw/misc/esp32p4_pms.h"
#include "hw/misc/esp32p4_trace.h"
#include "migration/vmstate.h"
#include "target/riscv/esp32p4_cpu.h"

#define A_MEM_START_ADDR        0x00
#define A_MEM_END_ADDR          0x04
#define A_MEM_CURRENT_ADDR      0x08
#define A_MEM_ADDR_UPDATE       0x0c
#define A_FIFO_STATUS           0x10
#define A_INTR_ENA              0x14
#define A_INTR_RAW              0x18
#define A_INTR_CLR              0x1c
#define A_TRIGGER               0x20
#define A_CONFIG                0x24
#define A_FILTER_CONTROL        0x28
#define A_FILTER_MATCH_CONTROL  0x2c
#define A_FILTER_COMPARATOR_CONTROL 0x30
#define A_FILTER_P_COMPARATOR_MATCH 0x34
#define A_FILTER_S_COMPARATOR_MATCH 0x38
#define A_RESYNC_PROLONGED      0x3c
#define A_AHB_CONFIG            0x40
#define A_CLOCK_GATE            0x44
#define A_DATE                  0x3fc

#define INTR_FIFO_OVERFLOW      BIT(0)
#define INTR_MEM_FULL           BIT(1)
#define INTR_MASK               0x3

#define TRIGGER_ON              BIT(0)
#define TRIGGER_OFF             BIT(1)
#define TRIGGER_MEM_LOOP        BIT(2)
#define TRIGGER_RESTART_ENA     BIT(3)
#define TRIGGER_RW              (TRIGGER_MEM_LOOP | TRIGGER_RESTART_ENA)

#define CONFIG_DM_TRIGGER_ENA   BIT(0)
#define CONFIG_RESET_ENA        BIT(1)
#define CONFIG_STALL_ENA        BIT(3)
#define CONFIG_FULL_ADDRESS     BIT(4)
#define CONFIG_IMPLICIT_EXCEPT  BIT(5)
#define CONFIG_MASK             0x3f

#define FILTER_EN               BIT(0)
#define FILTER_MATCH_COMP       BIT(1)
#define FILTER_MATCH_PRIVILEGE  BIT(2)
#define FILTER_MATCH_ECAUSE     BIT(3)
#define FILTER_MATCH_INTERRUPT  BIT(4)
#define FILTER_CONTROL_MASK     0x1f

#define MATCH_CHOICE_PRIVILEGE  BIT(0)
#define MATCH_VALUE_INTERRUPT   BIT(1)
#define MATCH_CONTROL_MASK      0xff

#define COMP_CONTROL_MASK       0x33d3d
#define COMP_P_NOTIFY           BIT(5)
#define COMP_S_NOTIFY           BIT(13)

#define RESYNC_MASK             0x3ffffff
#define RESYNC_COUNT_MASK       0xffffff
#define RESYNC_MODE_SHIFT       24
#define RESYNC_PACKETS          2
#define RESYNC_CYCLES           3

#define AHB_CONFIG_MASK         0x3f
#define DATE_MASK               0x0fffffff
#define DATE_RESET              0x2211300

enum {
    WORK_IDLE,
    WORK_WORKING,
    WORK_WAIT,
    WORK_LOST,
};

/* The instruction types, itype_width_p 3 (Efficient Trace table 4.3). */
enum {
    ITYPE_NONE = 0,
    ITYPE_EXCEPTION = 1,
    ITYPE_INTERRUPT = 2,
    ITYPE_XRET = 3,
    ITYPE_NT_BRANCH = 4,
    ITYPE_T_BRANCH = 5,
    ITYPE_UNINFERABLE = 6,
};

/* A support packet's qual_status. */
#define QUAL_NO_CHANGE          0
#define QUAL_ENDED_REP          1
#define QUAL_TRACE_LOST         2

#define MAX_BRANCHES            31
#define ANCHOR_PACKETS          128
#define ANCHOR_BYTES            14

/*
 * What the FIFO's master writes once the lock is let go: up to a FIFO's
 * bytes, a restart's support packet and the FIFO's again
 * (trace_progress()). With `need_bql`, a write into I/O, which is asked
 * about and made with the BQL held, waits in the FIFO for the BQL.
 */
typedef struct {
    uint8_t data[2 * ESP32P4_TRACE_FIFO];
    uint32_t addr[2 * ESP32P4_TRACE_FIFO];
    unsigned n;
    bool need_bql;
} ESP32P4TraceDrain;

/* A packet's payload as it is built, least significant bit first. */
typedef struct {
    uint8_t buf[16];
    unsigned bits;
} ESP32P4TraceBits;

static void tb_put(ESP32P4TraceBits *b, uint64_t v, unsigned n)
{
    for (unsigned i = 0; i < n; i++, b->bits++) {
        if ((v >> i) & 1) {
            b->buf[b->bits / 8] |= 1 << (b->bits % 8);
        }
    }
}

/* Fill to @n bits more with copies of the last bit, sign-extending it. */
static void tb_sign_extend(ESP32P4TraceBits *b, unsigned n)
{
    unsigned last = b->bits - 1;
    bool one = (b->buf[last / 8] >> (last % 8)) & 1;

    tb_put(b, one ? UINT64_MAX : 0, n);
}

static bool trace_is_exc(const ESP32P4TraceInsn *r)
{
    return r->valid &&
           (r->itype == ITYPE_EXCEPTION || r->itype == ITYPE_INTERRUPT);
}

/* The trap returns are uninferable discontinuities too, for the decoder. */
static bool trace_is_updiscon(const ESP32P4TraceInsn *r)
{
    return r->valid &&
           (r->itype == ITYPE_UNINFERABLE || r->itype == ITYPE_XRET);
}

static uint32_t trace_resync_mode(ESP32P4TraceState *t)
{
    return t->resync_prolonged >> RESYNC_MODE_SHIFT;
}

/* Interrupts */

static bool trace_irq_level(ESP32P4TraceState *t)
{
    return t->intr_raw & t->intr_ena & INTR_MASK;
}

/* FIFO and memory */

static uint32_t trace_fifo_space(ESP32P4TraceState *t)
{
    return ESP32P4_TRACE_FIFO - t->fifo_count;
}

static void trace_fifo_push(ESP32P4TraceState *t, uint8_t byte)
{
    t->fifo[(t->fifo_head + t->fifo_count) % ESP32P4_TRACE_FIFO] = byte;
    t->fifo_count++;
}

static MemTxAttrs trace_attrs(ESP32P4TraceState *t)
{
    return esp32p4_dma_attrs(
        t->core ? ESP32P4_DMA_MASTER_TRACE1 : ESP32P4_DMA_MASTER_TRACE0,
        t->core ? ESP32P4_ICM_ID_TRACE1 : ESP32P4_ICM_ID_TRACE0);
}

/*
 * Whether the master's write at `addr` reaches I/O, not RAM: a memory
 * controller's window among it, whose AXI port may hold the write.
 */
static bool trace_dest_io(ESP32P4TraceState *t, uint32_t addr)
{
    hwaddr xlat, l = 1;
    MemoryRegion *mr;

    RCU_READ_LOCK_GUARD();
    mr = address_space_translate(esp32p4_pms_bus_as(t->pms), addr, &xlat,
                                 &l, true, trace_attrs(t));
    return !memory_region_is_ram(mr);
}

/* Drop the FIFO's bytes, past the end of the trace memory. */
static void trace_fifo_drop(ESP32P4TraceState *t)
{
    t->fifo_head = (t->fifo_head + t->fifo_count) % ESP32P4_TRACE_FIFO;
    t->fifo_count = 0;
}

/*
 * The master takes the FIFO's bytes while TRACE_SYS_CLK runs, each to
 * MEM_CURRENT_ADDR; the write itself is made once the lock is let go. A
 * write into I/O is asked about first, with the BQL held through to the
 * write, so that a memory controller's port cannot come to hold it in
 * between: one the port holds is not made, its bytes staying in the FIFO,
 * until the port takes accesses again (trace_dma_ready()).
 */
/* [spec:nuos:req:emu.esp32p4.debug] */
static void trace_drain(ESP32P4TraceState *t, ESP32P4TraceDrain *d)
{
    bool loop = t->trigger & TRIGGER_MEM_LOOP;

    if (!clock_is_enabled(t->sys_clk)) {
        return;
    }
    while (t->fifo_count && !t->held) {
        uint32_t n;

        if (t->mem_current >= t->mem_end) {
            if (!loop) {
                trace_fifo_drop(t);
                break;
            }
            t->mem_current = t->mem_start;
            if (t->mem_current >= t->mem_end) {
                trace_fifo_drop(t);
                break;
            }
        }
        n = MIN(t->fifo_count, t->mem_end - t->mem_current);
        if (trace_dest_io(t, t->mem_current)) {
            if (!bql_locked()) {
                d->need_bql = true;
                break;
            }
            if (esp32p4_mspi_dma_wait_on(&t->dma_wait,
                                         esp32p4_pms_bus_as(t->pms),
                                         t->mem_current, n, trace_attrs(t),
                                         true)) {
                t->held = true;
                break;
            }
        }
        while (n--) {
            d->data[d->n] = t->fifo[t->fifo_head];
            d->addr[d->n] = t->mem_current;
            d->n++;
            t->fifo_head = (t->fifo_head + 1) % ESP32P4_TRACE_FIFO;
            t->fifo_count--;
            t->mem_current++;
        }
        if (t->mem_current == t->mem_end) {
            t->intr_raw |= INTR_MEM_FULL;
            if (loop) {
                t->mem_current = t->mem_start;
            }
        }
    }
}

static void trace_write(ESP32P4TraceState *t, ESP32P4TraceDrain *d)
{
    MemTxAttrs attrs = trace_attrs(t);
    unsigned i = 0;

    while (i < d->n) {
        unsigned j = i + 1;

        while (j < d->n && d->addr[j] == d->addr[j - 1] + 1) {
            j++;
        }
        if (address_space_write(esp32p4_pms_bus_as(t->pms), d->addr[i],
                                attrs, &d->data[i], j - i) != MEMTX_OK) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s%d: the trace memory write at "
                          "0x%08" PRIx32 " failed\n", TYPE_ESP32P4_TRACE,
                          t->core, d->addr[i]);
        }
        i = j;
    }
}

/* Drive the interrupt at its level, with the BQL. */
static void trace_update_irq(ESP32P4TraceState *t)
{
    bool level;

    qemu_mutex_lock(&t->lock);
    level = trace_irq_level(t);
    qemu_mutex_unlock(&t->lock);
    if (level != t->irq_level) {
        t->irq_level = level;
        qemu_set_irq(t->irq, level);
    }
}

static void trace_irq_timer(void *opaque)
{
    trace_update_irq(opaque);
}

static void trace_unlock(ESP32P4TraceState *t, ESP32P4TraceDrain *d);
static void trace_progress(ESP32P4TraceState *t, ESP32P4TraceDrain *d);

/*
 * Let go of the lock after a report of the core's, made without the BQL,
 * as trace_unlock(); a write into I/O the drain left for the BQL is then
 * made with it, the BQL taken before the lock.
 */
static void trace_finish(ESP32P4TraceState *t, ESP32P4TraceDrain *d)
{
    bool retry = d->need_bql;

    trace_unlock(t, d);
    if (retry) {
        ESP32P4TraceDrain d2 = { .n = 0 };

        bql_lock();
        qemu_mutex_lock(&t->lock);
        trace_progress(t, &d2);
        trace_unlock(t, &d2);
        bql_unlock();
    }
}

/*
 * Let go of the lock: write what the FIFO gave, and drive the interrupt
 * with the BQL, taking it unless held (the core's reports come without
 * it), or, from the middle of the core's block of translated code, once
 * the block ends.
 */
static void trace_unlock(ESP32P4TraceState *t, ESP32P4TraceDrain *d)
{
    bool level = trace_irq_level(t);

    qemu_mutex_unlock(&t->lock);
    if (d && d->n) {
        trace_write(t, d);
    }
    if (level != qatomic_read(&t->irq_level)) {
        if (esp32p4_cpu_mid_block()) {
            timer_mod(t->irq_timer, 0);
        } else {
            BQL_LOCK_GUARD();
            trace_update_irq(t);
        }
    }
}

/* Encoder state */

static void trace_encoder_reset(ESP32P4TraceState *t)
{
    memset(t->pipe, 0, sizeof(t->pipe));
    t->pend_valid = false;
    t->pend_notify = false;
    t->notify_next = false;
    t->branches = 0;
    t->branch_map = 0;
    t->resync_count = 0;
    t->resync_due = false;
    t->trap_reported = false;
    t->trap_match = false;
    t->trap_depth = 0;
    t->comp_latch = false;
}

static void trace_set_working(ESP32P4TraceState *t, uint32_t status)
{
    t->work_status = status;
    if (t->cpu) {
        esp32p4_trace_set_enabled(t->cpu, status == WORK_WORKING);
    }
}

static void trace_lost(ESP32P4TraceState *t);

/*
 * A packet into the FIFO, header and index first; one that does not fit
 * is lost, stopping the encoder. Every 128 packets, the anchor.
 */
/* [spec:nuos:req:emu.esp32p4.debug] */
static void trace_emit(ESP32P4TraceState *t, ESP32P4TraceBits *b)
{
    unsigned payload = b->bits / 8;
    unsigned len = 3 + payload;
    unsigned need = len + (t->anchor_count + 1 == ANCHOR_PACKETS ?
                           ANCHOR_BYTES : 0);
    unsigned i;

    if (t->work_status != WORK_WORKING) {
        return;
    }
    if (trace_fifo_space(t) < need) {
        trace_lost(t);
        return;
    }
    trace_fifo_push(t, len);
    trace_fifo_push(t, t->index);
    trace_fifo_push(t, t->index >> 8);
    for (i = 0; i < payload; i++) {
        trace_fifo_push(t, b->buf[i]);
    }
    t->index++;
    if (++t->anchor_count == ANCHOR_PACKETS) {
        for (i = 0; i < ANCHOR_BYTES; i++) {
            trace_fifo_push(t, 0);
        }
        t->anchor_count = 0;
    }
    t->resync_count++;
    t->branches = 0;
    t->branch_map = 0;
}

/* The privilege field: 1 for machine mode, 0 for user mode. */
static unsigned trace_priv(int priv)
{
    return priv == PRV_M;
}

/* Format 3, subformat 3: support. */
static void trace_support(ESP32P4TraceState *t, bool enable, unsigned qual)
{
    ESP32P4TraceBits b = { 0 };
    unsigned ioptions = t->config & CONFIG_FULL_ADDRESS ? BIT(2) : 0;

    tb_put(&b, 3, 2);
    tb_put(&b, 3, 2);
    tb_put(&b, enable, 1);
    tb_put(&b, 0, 1);
    tb_put(&b, qual, 2);
    tb_put(&b, ioptions, 6);
    tb_sign_extend(&b, 2);
    trace_emit(t, &b);
}

static void trace_synced(ESP32P4TraceState *t, uint32_t addr)
{
    t->last_addr = addr;
    t->resync_count = 0;
    t->resync_due = false;
    t->sync_cycles = t->cycles;
}

/* Format 3, subformat 0: synchronisation at @c. */
static void trace_sync_start(ESP32P4TraceState *t, const ESP32P4TraceInsn *c)
{
    ESP32P4TraceBits b = { 0 };

    tb_put(&b, 3, 2);
    tb_put(&b, 0, 2);
    tb_put(&b, c->itype != ITYPE_T_BRANCH, 1);
    tb_put(&b, c->priv, 1);
    tb_put(&b, c->iaddr >> 1, 31);
    tb_sign_extend(&b, 3);
    trace_emit(t, &b);
    trace_synced(t, c->iaddr);
}

/*
 * Format 3, subformat 1: the trap @exc at @c, the handler's first
 * instruction (@thaddr) or the excepting one.
 */
static void trace_sync_trap(ESP32P4TraceState *t, const ESP32P4TraceInsn *c,
                            const ESP32P4TraceInsn *exc, bool thaddr)
{
    ESP32P4TraceBits b = { 0 };
    bool interrupt = exc->itype == ITYPE_INTERRUPT;

    tb_put(&b, 3, 2);
    tb_put(&b, 1, 2);
    tb_put(&b, c->itype != ITYPE_T_BRANCH, 1);
    tb_put(&b, c->priv, 1);
    tb_put(&b, exc->cause, 6);
    tb_put(&b, interrupt, 1);
    tb_put(&b, thaddr, 1);
    tb_put(&b, c->iaddr >> 1, 31);
    if (!interrupt) {
        tb_put(&b, exc->tval, 32);
    }
    tb_sign_extend(&b, 3);
    trace_emit(t, &b);
    trace_synced(t, c->iaddr);
}

/*
 * Format 1 or 2: @c's address, with the branch map if there are branches,
 * and the notify and updiscon flags.
 */
static void trace_address_packet(ESP32P4TraceState *t,
                                 const ESP32P4TraceInsn *c, bool updiscon)
{
    ESP32P4TraceBits b = { 0 };
    uint32_t field;
    unsigned bytes = 4, map_bits = 0, zero_ext = 0;
    bool msb, notify_bit;

    if (t->config & CONFIG_FULL_ADDRESS) {
        field = c->iaddr >> 1;
    } else {
        int32_t d = (int32_t)(c->iaddr - t->last_addr) >> 1;

        field = d;
        for (bytes = 1; bytes < 4; bytes++) {
            if (sextract32(d, 0, 8 * bytes) == d) {
                field = extract32(d, 0, 8 * bytes);
                break;
            }
        }
    }
    msb = (field >> (8 * bytes - 1)) & 1;
    notify_bit = msb ^ c->notify;

    if (t->branches) {
        if (t->branches == 1) {
            map_bits = 1;
            zero_ext = 6;
        } else if (t->branches <= 3) {
            map_bits = 3;
            zero_ext = 4;
        } else if (t->branches <= 7) {
            map_bits = 7;
        } else if (t->branches <= 15) {
            map_bits = 15;
        } else {
            map_bits = 31;
        }
        tb_put(&b, 1, 2);
        tb_put(&b, t->branches, 5);
        tb_put(&b, t->branch_map, map_bits);
    } else {
        tb_put(&b, 2, 2);
        zero_ext = 4;
    }
    tb_put(&b, notify_bit, 1);
    tb_put(&b, notify_bit ^ updiscon, 1);
    tb_put(&b, 0, zero_ext);
    tb_put(&b, field, 8 * bytes);
    trace_emit(t, &b);
    t->last_addr = c->iaddr;
}

/* Format 1 without an address: a full branch map. */
static void trace_branch_packet(ESP32P4TraceState *t)
{
    ESP32P4TraceBits b = { 0 };

    tb_put(&b, 1, 2);
    tb_put(&b, 0, 5);
    tb_put(&b, t->branch_map, 31);
    tb_sign_extend(&b, 2);
    trace_emit(t, &b);
}

/*
 * The reference algorithm on the current instruction, knowing the
 * previous and the next. A next that is not valid ends the trace: the
 * last instruction is reported, the support packet following.
 */
/* [spec:nuos:req:emu.esp32p4.debug] */
static void trace_encode(ESP32P4TraceState *t)
{
    ESP32P4TraceInsn *p = &t->pipe[0];
    ESP32P4TraceInsn *c = &t->pipe[1];
    ESP32P4TraceInsn *n = &t->pipe[2];
    uint32_t mode = trace_resync_mode(t);
    uint32_t max = t->resync_prolonged & RESYNC_COUNT_MASK;
    bool at_max = false, exceeded = false, resync_br = false;
    bool prev_updiscon, updiscon, prev_exception, curr_exc_only;
    bool next_exc_only, prev_reported, ppccd_br, next_ended;

    if (!c->valid || !c->qualified) {
        return;
    }
    if (c->itype == ITYPE_NT_BRANCH || c->itype == ITYPE_T_BRANCH) {
        if (c->itype == ITYPE_NT_BRANCH) {
            t->branch_map |= BIT(t->branches);
        }
        t->branches++;
    }

    if (mode == RESYNC_PACKETS) {
        at_max = t->resync_count == max;
        exceeded = t->resync_count > max;
        resync_br = at_max && t->branches;
    } else if (mode == RESYNC_CYCLES) {
        bool reached = t->cycles - t->sync_cycles >= max;

        at_max = reached;
        exceeded = t->resync_due || (reached && !t->branches);
        resync_br = reached && t->branches && !t->resync_due;
    }

    prev_updiscon = trace_is_updiscon(p);
    updiscon = prev_updiscon && (trace_is_exc(n) ||
                                 (n->valid && n->priv != c->priv) || at_max);
    prev_exception = trace_is_exc(p);
    curr_exc_only = !c->retired && trace_is_exc(c);
    next_exc_only = !n->retired && trace_is_exc(n);
    prev_reported = prev_exception && t->trap_reported;
    t->trap_reported = false;
    ppccd_br = t->branches && n->valid && n->priv != c->priv;
    next_ended = !n->valid || !n->qualified;

    if (prev_exception) {
        if (curr_exc_only) {
            trace_sync_trap(t, c, p, false);
            t->trap_reported = prev_updiscon || trace_is_exc(n);
        } else if (prev_reported) {
            trace_sync_start(t, c);
        } else {
            trace_sync_trap(t, c, p, true);
        }
        return;
    }
    if (!p->qualified || !p->valid || c->priv != p->priv || exceeded) {
        trace_sync_start(t, c);
        return;
    }
    if (prev_updiscon) {
        if (curr_exc_only) {
            trace_sync_trap(t, c, c, false);
            t->trap_reported = true;
        } else {
            trace_address_packet(t, c, updiscon);
        }
        return;
    }
    if (resync_br || c->notify) {
        trace_address_packet(t, c, false);
        if (resync_br) {
            t->resync_due = true;
        }
        return;
    }
    if (next_exc_only || ppccd_br || next_ended) {
        trace_address_packet(t, c, false);
        if (n->valid && !n->qualified) {
            trace_support(t, true, QUAL_ENDED_REP);
        }
        return;
    }
    if (t->branches == MAX_BRANCHES) {
        trace_branch_packet(t);
    }
}

static void trace_push(ESP32P4TraceState *t, const ESP32P4TraceInsn *r)
{
    t->pipe[0] = t->pipe[1];
    t->pipe[1] = t->pipe[2];
    t->pipe[2] = *r;
    trace_encode(t);
}

/* Filtering */

static bool trace_compare(unsigned fn, uint32_t a, uint32_t b)
{
    switch (fn) {
    case 0:
        return a == b;
    case 1:
        return a != b;
    case 2:
        return a < b;
    case 3:
        return a <= b;
    case 4:
        return a > b;
    case 5:
        return a >= b;
    default:
        return true;
    }
}

/*
 * The comparators on @r: whether they let it through, and whether one
 * with its NOTIFY bit notifies it.
 */
static bool trace_comparators(ESP32P4TraceState *t, ESP32P4TraceInsn *r)
{
    uint32_t ctl = t->filter_comparator_control;
    bool p_notify = ctl & COMP_P_NOTIFY, s_notify = ctl & COMP_S_NOTIFY;
    bool p = trace_compare(extract32(ctl, 2, 3),
                           extract32(ctl, 0, 1) ? r->tval : r->iaddr,
                           t->p_match);
    bool s = trace_compare(extract32(ctl, 10, 3),
                           extract32(ctl, 8, 1) ? r->tval : r->iaddr,
                           t->s_match);
    bool fp = p_notify || p, fs = s_notify || s;
    bool pass;

    if ((p_notify && p) || (s_notify && s)) {
        r->notify = true;
    }
    switch (extract32(ctl, 16, 2)) {
    case 0:
        return fp;
    case 1:
        return fp && fs;
    case 2:
        return !(fp && fs);
    default:
        if (fp) {
            t->comp_latch = true;
        }
        pass = t->comp_latch;
        if (pass && s && !s_notify) {
            t->comp_latch = false;
        }
        return pass;
    }
}

/*
 * Qualify @r. A trap of the chosen cause or type starts the qualification
 * of its handling, and the return from it ends it, after the return.
 */
/* [spec:nuos:req:emu.esp32p4.debug] */
static void trace_qualify(ESP32P4TraceState *t, ESP32P4TraceInsn *r)
{
    uint32_t ctl = t->filter_control;
    uint32_t match = t->filter_match_control;
    bool trap_filter = ctl & (FILTER_MATCH_ECAUSE | FILTER_MATCH_INTERRUPT);
    bool q = true;

    if (trace_is_exc(r) && trap_filter) {
        if (t->trap_match) {
            t->trap_depth++;
        } else {
            bool hit = true;

            if (ctl & FILTER_MATCH_ECAUSE) {
                hit &= r->itype == ITYPE_EXCEPTION &&
                       r->cause == extract32(match, 2, 6);
            }
            if (ctl & FILTER_MATCH_INTERRUPT) {
                hit &= r->itype == ((match & MATCH_VALUE_INTERRUPT) ?
                                    ITYPE_INTERRUPT : ITYPE_EXCEPTION);
            }
            if (hit) {
                t->trap_match = true;
                t->trap_depth = 1;
            }
        }
    }
    if (!(ctl & FILTER_EN)) {
        r->qualified = true;
        return;
    }
    if (ctl & FILTER_MATCH_PRIVILEGE) {
        q &= r->priv == (match & MATCH_CHOICE_PRIVILEGE);
    }
    if (trap_filter) {
        q &= t->trap_match;
    }
    if (ctl & FILTER_MATCH_COMP) {
        q &= trace_comparators(t, r);
    }
    r->qualified = q;
    if (r->itype == ITYPE_XRET && t->trap_match && !--t->trap_depth) {
        t->trap_match = false;
    }
}

/* The started instruction retired; the next starts at @next. */
/* [spec:nuos:req:emu.esp32p4.debug] */
static void trace_retire(ESP32P4TraceState *t, uint32_t next)
{
    ESP32P4TraceInsn r = {
        .iaddr = t->pend_pc,
        .priv = t->pend_priv,
        .retired = true,
        .notify = t->pend_notify,
        .valid = true,
    };
    unsigned len = (t->pend_kind & ESP32P4_TRACE_LEN_4) ? 4 : 2;
    bool seq = next == t->pend_pc + len;

    switch (t->pend_kind & ESP32P4_TRACE_KIND_MASK) {
    case ESP32P4_TRACE_KIND_BRANCH:
        r.itype = seq ? ITYPE_NT_BRANCH : ITYPE_T_BRANCH;
        break;
    case ESP32P4_TRACE_KIND_JAL:
        r.itype = ITYPE_NONE;
        break;
    case ESP32P4_TRACE_KIND_JALR:
        r.itype = ITYPE_UNINFERABLE;
        break;
    case ESP32P4_TRACE_KIND_XRET:
        r.itype = ITYPE_XRET;
        break;
    default:
        /* A hardware loop's jump back to its start. */
        r.itype = seq ? ITYPE_NONE : ITYPE_UNINFERABLE;
        break;
    }
    t->pend_valid = false;
    trace_qualify(t, &r);
    trace_push(t, &r);
}

/* Starting and stopping */

/* [spec:nuos:req:emu.esp32p4.debug] */
static void trace_start(ESP32P4TraceState *t, bool from_idle)
{
    if (from_idle) {
        t->index = 0;
        t->anchor_count = 0;
    }
    trace_encoder_reset(t);
    t->ran = true;
    t->sync_cycles = t->cycles;
    trace_set_working(t, WORK_WORKING);
    if (t->lost_owed) {
        t->lost_owed = false;
        trace_support(t, true, QUAL_TRACE_LOST);
    }
}

/*
 * The end of tracing: the instruction started counts as retired, the last
 * instruction is reported, then a support packet; the encoder goes to
 * @status.
 */
/* [spec:nuos:req:emu.esp32p4.debug] */
static void trace_stop(ESP32P4TraceState *t, uint32_t status)
{
    static const ESP32P4TraceInsn end = { 0 };

    if (t->work_status != WORK_WORKING) {
        return;
    }
    if (t->pend_valid) {
        trace_retire(t, t->pend_pc +
                     ((t->pend_kind & ESP32P4_TRACE_LEN_4) ? 4 : 2));
    }
    trace_push(t, &end);
    trace_support(t, false, QUAL_NO_CHANGE);
    trace_encoder_reset(t);
    trace_set_working(t, status);
}

/*
 * The FIFO is empty: an encoder stopped by a lost packet restarts with
 * TRACE_RESTART_ENA, or goes idle; with TRACE_RESTART_ENA an idle one
 * restarts too (TRM 3.5.7).
 */
static void trace_check_restart(ESP32P4TraceState *t)
{
    if (t->fifo_count) {
        return;
    }
    if (t->work_status == WORK_LOST) {
        if (t->trigger & TRIGGER_RESTART_ENA) {
            trace_start(t, false);
        } else {
            trace_set_working(t, WORK_IDLE);
        }
    } else if (t->work_status == WORK_IDLE && t->ran &&
               (t->trigger & TRIGGER_RESTART_ENA)) {
        trace_start(t, false);
    }
}

/* A packet did not fit the FIFO: it is lost, and the encoder stops. */
/* [spec:nuos:req:emu.esp32p4.debug] */
static void trace_lost(ESP32P4TraceState *t)
{
    t->intr_raw |= INTR_FIFO_OVERFLOW;
    trace_encoder_reset(t);
    t->lost_owed = true;
    trace_set_working(t, WORK_LOST);
}

/*
 * The stall request (TRACE_STALL_ENA) asserted while the FIFO has less
 * room than ESP32P4_TRACE_STALL_ROOM: from the report of an instruction's
 * start, which the core then does not execute until the request is
 * deasserted. Returns whether it stalls the core.
 */
/* [spec:nuos:req:emu.esp32p4.debug] */
static bool trace_stall_check(ESP32P4TraceState *t)
{
    if (!(t->config & CONFIG_STALL_ENA) || t->work_status != WORK_WORKING ||
        trace_fifo_space(t) >= ESP32P4_TRACE_STALL_ROOM) {
        return false;
    }
    t->stall = true;
    esp32p4_cpu_trace_stall(t->cpu);
    return true;
}

/*
 * The stall request deasserted, the core going on, once the FIFO has the
 * room or the request no longer applies (TRACE_STALL_ENA cleared, the
 * encoder stopped).
 */
static void trace_stall_update(ESP32P4TraceState *t)
{
    if (t->stall && (!(t->config & CONFIG_STALL_ENA) ||
                     t->work_status != WORK_WORKING ||
                     trace_fifo_space(t) >= ESP32P4_TRACE_STALL_ROOM)) {
        t->stall = false;
        esp32p4_cpu_trace_unstall(t->cpu);
    }
}

/* Drain, and restart if that empties the FIFO. */
static void trace_progress(ESP32P4TraceState *t, ESP32P4TraceDrain *d)
{
    trace_drain(t, d);
    trace_check_restart(t);
    trace_drain(t, d);
    trace_stall_update(t);
}

/*
 * [spec:nuos:req:emu.esp32p4.debug]
 * The memory controller's port takes accesses again: the master's write
 * goes on.
 */
static void trace_dma_ready(void *opaque)
{
    ESP32P4TraceState *t = opaque;
    ESP32P4TraceDrain d = { .n = 0 };

    qemu_mutex_lock(&t->lock);
    t->held = false;
    trace_progress(t, &d);
    trace_unlock(t, &d);
}

/* The core's reports */

/* [spec:nuos:req:emu.esp32p4.debug] */
static bool trace_insn(void *opaque, uint32_t pc, uint32_t kind, int priv)
{
    ESP32P4TraceState *t = opaque;
    ESP32P4TraceDrain d = { .n = 0 };
    bool stall = false;

    qemu_mutex_lock(&t->lock);
    if (t->work_status == WORK_WORKING) {
        t->cycles++;
        if (t->pend_valid) {
            trace_retire(t, pc);
        }
        t->pend_valid = true;
        t->pend_pc = pc;
        t->pend_kind = kind;
        t->pend_priv = trace_priv(priv);
        t->pend_notify = t->notify_next;
        t->notify_next = false;
        trace_progress(t, &d);
        if (!d.need_bql) {
            stall = trace_stall_check(t);
        }
    }
    if (d.need_bql) {
        trace_finish(t, &d);
        qemu_mutex_lock(&t->lock);
        stall = trace_stall_check(t);
        qemu_mutex_unlock(&t->lock);
        return stall;
    }
    trace_unlock(t, &d);
    return stall;
}

/* [spec:nuos:req:emu.esp32p4.debug] */
static void trace_trap(void *opaque, uint32_t epc, uint32_t cause,
                       bool interrupt, uint32_t tval, uint32_t handler,
                       int priv, bool retired)
{
    ESP32P4TraceState *t = opaque;
    ESP32P4TraceDrain d = { .n = 0 };
    ESP32P4TraceInsn r = {
        .iaddr = epc,
        .tval = interrupt ? 0 : tval,
        .itype = interrupt ? ITYPE_INTERRUPT : ITYPE_EXCEPTION,
        .priv = trace_priv(priv),
        .cause = cause & 0x3f,
        .valid = true,
    };

    qemu_mutex_lock(&t->lock);
    if (t->work_status == WORK_WORKING) {
        if (t->pend_valid) {
            if (retired) {
                trace_retire(t, epc);
            }
            t->pend_valid = false;
        }
        trace_qualify(t, &r);
        trace_push(t, &r);
        trace_progress(t, &d);
    }
    trace_finish(t, &d);
}

/* [spec:nuos:req:emu.esp32p4.debug] */
static void trace_trigger(void *opaque, int action, bool before)
{
    ESP32P4TraceState *t = opaque;
    ESP32P4TraceDrain d = { .n = 0 };

    qemu_mutex_lock(&t->lock);
    if (t->config & CONFIG_DM_TRIGGER_ENA) {
        switch (action) {
        case 2:
            if (t->work_status == WORK_IDLE) {
                trace_start(t, true);
            }
            break;
        case 3:
            trace_stop(t, WORK_IDLE);
            break;
        case 4:
            if (before || !t->pend_valid) {
                t->notify_next = true;
            } else {
                t->pend_notify = true;
            }
            break;
        default:
            break;
        }
        trace_progress(t, &d);
    }
    trace_finish(t, &d);
}

static const ESP32P4TraceOps esp32p4_trace_ops = {
    .insn = trace_insn,
    .trap = trace_trap,
    .trigger = trace_trigger,
};

/* Registers */

static uint64_t esp32p4_trace_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4TraceState *t = opaque;
    uint32_t v;

    qemu_mutex_lock(&t->lock);
    switch (addr & ~3) {
    case A_MEM_START_ADDR:
        v = t->mem_start;
        break;
    case A_MEM_END_ADDR:
        v = t->mem_end;
        break;
    case A_MEM_CURRENT_ADDR:
        v = t->mem_current;
        break;
    case A_FIFO_STATUS:
        v = (t->fifo_count == 0) | (t->work_status << 1);
        break;
    case A_INTR_ENA:
        v = t->intr_ena;
        break;
    case A_INTR_RAW:
        v = t->intr_raw;
        break;
    case A_TRIGGER:
        v = t->trigger;
        break;
    case A_CONFIG:
        v = t->config;
        break;
    case A_FILTER_CONTROL:
        v = t->filter_control;
        break;
    case A_FILTER_MATCH_CONTROL:
        v = t->filter_match_control;
        break;
    case A_FILTER_COMPARATOR_CONTROL:
        v = t->filter_comparator_control;
        break;
    case A_FILTER_P_COMPARATOR_MATCH:
        v = t->p_match;
        break;
    case A_FILTER_S_COMPARATOR_MATCH:
        v = t->s_match;
        break;
    case A_RESYNC_PROLONGED:
        v = t->resync_prolonged;
        break;
    case A_AHB_CONFIG:
        v = t->ahb_config;
        break;
    case A_CLOCK_GATE:
        v = t->clock_gate;
        break;
    case A_DATE:
        v = t->date;
        break;
    case A_MEM_ADDR_UPDATE:
    case A_INTR_CLR:
        v = 0;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s%d: read of reserved offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_TRACE, t->core, addr);
        v = 0;
        break;
    }
    qemu_mutex_unlock(&t->lock);
    return extract32(v, (addr & 3) * 8, size * 8);
}

static void esp32p4_trace_set(uint32_t *reg, uint32_t v, uint32_t bytes,
                              uint32_t mask)
{
    *reg = ((*reg & ~bytes) | (v & bytes)) & mask;
}

/* [spec:nuos:req:emu.esp32p4.debug] */
static void esp32p4_trace_write(void *opaque, hwaddr addr, uint64_t value,
                                unsigned size)
{
    ESP32P4TraceState *t = opaque;
    ESP32P4TraceDrain d = { .n = 0 };
    unsigned shift = (addr & 3) * 8;
    uint32_t bytes = MAKE_64BIT_MASK(shift, size * 8);
    uint32_t v = ((uint32_t)value << shift) & bytes;

    qemu_mutex_lock(&t->lock);
    switch (addr & ~3) {
    case A_MEM_START_ADDR:
        esp32p4_trace_set(&t->mem_start, v, bytes, UINT32_MAX);
        break;
    case A_MEM_END_ADDR:
        esp32p4_trace_set(&t->mem_end, v, bytes, UINT32_MAX);
        break;
    case A_MEM_ADDR_UPDATE:
        if (v & 1) {
            t->mem_current = t->mem_start;
        }
        break;
    case A_INTR_ENA:
        esp32p4_trace_set(&t->intr_ena, v, bytes, INTR_MASK);
        break;
    case A_INTR_CLR:
        t->intr_raw &= ~(v & INTR_MASK);
        break;
    case A_TRIGGER:
        esp32p4_trace_set(&t->trigger, v, bytes, TRIGGER_RW);
        if (v & TRIGGER_OFF) {
            trace_stop(t, WORK_IDLE);
        }
        if ((v & TRIGGER_ON) && t->work_status == WORK_IDLE) {
            trace_start(t, true);
        }
        break;
    case A_CONFIG:
        esp32p4_trace_set(&t->config, v, bytes, CONFIG_MASK);
        if (t->config & CONFIG_IMPLICIT_EXCEPT) {
            qemu_log_mask(LOG_UNIMP, "%s%d: implicit exception mode is not "
                          "modelled; exception packets keep their address\n",
                          TYPE_ESP32P4_TRACE, t->core);
        }
        break;
    case A_FILTER_CONTROL:
        esp32p4_trace_set(&t->filter_control, v, bytes, FILTER_CONTROL_MASK);
        break;
    case A_FILTER_MATCH_CONTROL:
        esp32p4_trace_set(&t->filter_match_control, v, bytes,
                          MATCH_CONTROL_MASK);
        break;
    case A_FILTER_COMPARATOR_CONTROL:
        esp32p4_trace_set(&t->filter_comparator_control, v, bytes,
                          COMP_CONTROL_MASK);
        break;
    case A_FILTER_P_COMPARATOR_MATCH:
        esp32p4_trace_set(&t->p_match, v, bytes, UINT32_MAX);
        break;
    case A_FILTER_S_COMPARATOR_MATCH:
        esp32p4_trace_set(&t->s_match, v, bytes, UINT32_MAX);
        break;
    case A_RESYNC_PROLONGED:
        esp32p4_trace_set(&t->resync_prolonged, v, bytes, RESYNC_MASK);
        break;
    case A_AHB_CONFIG:
        esp32p4_trace_set(&t->ahb_config, v, bytes, AHB_CONFIG_MASK);
        break;
    case A_CLOCK_GATE:
        esp32p4_trace_set(&t->clock_gate, v, bytes, 1);
        break;
    case A_DATE:
        esp32p4_trace_set(&t->date, v, bytes, DATE_MASK);
        break;
    case A_MEM_CURRENT_ADDR:
    case A_FIFO_STATUS:
    case A_INTR_RAW:
        qemu_log_mask(LOG_GUEST_ERROR, "%s%d: write to read-only offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_TRACE, t->core, addr);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s%d: write to reserved offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_TRACE, t->core, addr);
        break;
    }
    trace_progress(t, &d);
    trace_unlock(t, &d);
}

static const MemoryRegionOps esp32p4_trace_mmio_ops = {
    .read = esp32p4_trace_read,
    .write = esp32p4_trace_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

/* TRACE_SYS_CLK restarting: the FIFO drains. */
static void esp32p4_trace_sys_clock_update(void *opaque, ClockEvent event)
{
    ESP32P4TraceState *t = opaque;
    ESP32P4TraceDrain d = { .n = 0 };

    qemu_mutex_lock(&t->lock);
    trace_progress(t, &d);
    trace_unlock(t, &d);
}

/*
 * The core's reset sideband: with TRACE_RESET_ENA, a core held in reset
 * ends the trace, which starts again as the core leaves reset.
 */
/* [spec:nuos:req:emu.esp32p4.debug] */
static void esp32p4_trace_core_reset(void *opaque, int n, int level)
{
    ESP32P4TraceState *t = opaque;
    ESP32P4TraceDrain d = { .n = 0 };

    qemu_mutex_lock(&t->lock);
    t->core_reset = level;
    if (level && (t->config & CONFIG_RESET_ENA)) {
        trace_stop(t, WORK_WAIT);
    } else if (!level && t->work_status == WORK_WAIT) {
        trace_start(t, false);
    }
    trace_progress(t, &d);
    trace_unlock(t, &d);
}

static void esp32p4_trace_reset_hold(Object *obj, ResetType type)
{
    ESP32P4TraceState *t = ESP32P4_TRACE(obj);

    esp32p4_mspi_dma_wait_cancel(&t->dma_wait);
    qemu_mutex_lock(&t->lock);
    t->held = false;
    if (t->stall) {
        t->stall = false;
        esp32p4_cpu_trace_unstall(t->cpu);
    }
    t->mem_start = 0;
    t->mem_end = UINT32_MAX;
    t->mem_current = 0;
    t->intr_ena = 0;
    t->intr_raw = 0;
    t->trigger = TRIGGER_RW;
    t->config = 0;
    t->filter_control = 0;
    t->filter_match_control = 0;
    t->filter_comparator_control = 0;
    t->p_match = 0;
    t->s_match = 0;
    t->resync_prolonged = 0x80;
    t->ahb_config = 0;
    t->clock_gate = 1;
    t->date = DATE_RESET;
    t->fifo_head = 0;
    t->fifo_count = 0;
    t->index = 0;
    t->anchor_count = 0;
    t->lost_owed = false;
    t->ran = false;
    t->last_addr = 0;
    t->cycles = 0;
    t->sync_cycles = 0;
    trace_encoder_reset(t);
    trace_set_working(t, WORK_IDLE);
    qemu_mutex_unlock(&t->lock);
}

static void esp32p4_trace_reset_exit(Object *obj, ResetType type)
{
    ESP32P4TraceState *t = ESP32P4_TRACE(obj);

    t->irq_level = trace_irq_level(t);
    qemu_set_irq(t->irq, t->irq_level);
}

static void esp32p4_trace_realize(DeviceState *dev, Error **errp)
{
    ESP32P4TraceState *t = ESP32P4_TRACE(dev);

    if (!t->cpu || !t->pms) {
        error_setg(errp, "%s: the \"cpu\" and \"pms\" links must be set",
                   TYPE_ESP32P4_TRACE);
        return;
    }
    t->irq_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, trace_irq_timer, t);
    esp32p4_mspi_dma_wait_init(&t->dma_wait, trace_dma_ready, t);
    esp32p4_trace_connect(t->cpu, &esp32p4_trace_ops, t);
}

static void esp32p4_trace_init(Object *obj)
{
    ESP32P4TraceState *t = ESP32P4_TRACE(obj);

    qemu_mutex_init(&t->lock);
    memory_region_init_io(&t->iomem, obj, &esp32p4_trace_mmio_ops, t,
                          "esp32p4.trace", ESP32P4_TRACE_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &t->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &t->irq);
    t->sys_clk = qdev_init_clock_in(DEVICE(obj), "sys-clk",
                                    esp32p4_trace_sys_clock_update, t,
                                    ClockUpdate);
    qdev_init_gpio_in_named(DEVICE(obj), esp32p4_trace_core_reset,
                            ESP32P4_TRACE_CORE_RESET, 1);
}

static void esp32p4_trace_finalize(Object *obj)
{
    qemu_mutex_destroy(&ESP32P4_TRACE(obj)->lock);
}

/*
 * The encoder's reporting resumes on its own state after migration, and a
 * write a held port held asks again (a wait's place on a controller's list
 * is not migrated).
 */
static int esp32p4_trace_post_load(void *opaque, int version_id)
{
    ESP32P4TraceState *t = opaque;

    if (t->cpu) {
        esp32p4_trace_set_enabled(t->cpu, t->work_status == WORK_WORKING);
    }
    if (t->held) {
        esp32p4_mspi_dma_wait_retry(&t->dma_wait);
    }
    return 0;
}

static const VMStateDescription vmstate_esp32p4_trace_insn = {
    .name = "esp32p4-trace/insn",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(iaddr, ESP32P4TraceInsn),
        VMSTATE_UINT32(tval, ESP32P4TraceInsn),
        VMSTATE_UINT8(itype, ESP32P4TraceInsn),
        VMSTATE_UINT8(priv, ESP32P4TraceInsn),
        VMSTATE_UINT8(cause, ESP32P4TraceInsn),
        VMSTATE_BOOL(retired, ESP32P4TraceInsn),
        VMSTATE_BOOL(qualified, ESP32P4TraceInsn),
        VMSTATE_BOOL(notify, ESP32P4TraceInsn),
        VMSTATE_BOOL(valid, ESP32P4TraceInsn),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_esp32p4_trace = {
    .name = TYPE_ESP32P4_TRACE,
    .version_id = 2,
    .minimum_version_id = 2,
    .post_load = esp32p4_trace_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_CLOCK(sys_clk, ESP32P4TraceState),
        VMSTATE_UINT32(mem_start, ESP32P4TraceState),
        VMSTATE_UINT32(mem_end, ESP32P4TraceState),
        VMSTATE_UINT32(mem_current, ESP32P4TraceState),
        VMSTATE_UINT32(intr_ena, ESP32P4TraceState),
        VMSTATE_UINT32(intr_raw, ESP32P4TraceState),
        VMSTATE_UINT32(trigger, ESP32P4TraceState),
        VMSTATE_UINT32(config, ESP32P4TraceState),
        VMSTATE_UINT32(filter_control, ESP32P4TraceState),
        VMSTATE_UINT32(filter_match_control, ESP32P4TraceState),
        VMSTATE_UINT32(filter_comparator_control, ESP32P4TraceState),
        VMSTATE_UINT32(p_match, ESP32P4TraceState),
        VMSTATE_UINT32(s_match, ESP32P4TraceState),
        VMSTATE_UINT32(resync_prolonged, ESP32P4TraceState),
        VMSTATE_UINT32(ahb_config, ESP32P4TraceState),
        VMSTATE_UINT32(clock_gate, ESP32P4TraceState),
        VMSTATE_UINT32(date, ESP32P4TraceState),
        VMSTATE_UINT32(work_status, ESP32P4TraceState),
        VMSTATE_BOOL(core_reset, ESP32P4TraceState),
        VMSTATE_BOOL(irq_level, ESP32P4TraceState),
        VMSTATE_BOOL(held, ESP32P4TraceState),
        VMSTATE_BOOL(stall, ESP32P4TraceState),
        VMSTATE_TIMER_PTR(irq_timer, ESP32P4TraceState),
        VMSTATE_UINT8_ARRAY(fifo, ESP32P4TraceState, ESP32P4_TRACE_FIFO),
        VMSTATE_UINT32(fifo_head, ESP32P4TraceState),
        VMSTATE_UINT32(fifo_count, ESP32P4TraceState),
        VMSTATE_BOOL(pend_valid, ESP32P4TraceState),
        VMSTATE_UINT32(pend_pc, ESP32P4TraceState),
        VMSTATE_UINT32(pend_kind, ESP32P4TraceState),
        VMSTATE_UINT8(pend_priv, ESP32P4TraceState),
        VMSTATE_BOOL(pend_notify, ESP32P4TraceState),
        VMSTATE_BOOL(notify_next, ESP32P4TraceState),
        VMSTATE_STRUCT_ARRAY(pipe, ESP32P4TraceState, 3, 1,
                             vmstate_esp32p4_trace_insn, ESP32P4TraceInsn),
        VMSTATE_UINT32(branches, ESP32P4TraceState),
        VMSTATE_UINT32(branch_map, ESP32P4TraceState),
        VMSTATE_UINT32(resync_count, ESP32P4TraceState),
        VMSTATE_UINT64(cycles, ESP32P4TraceState),
        VMSTATE_UINT64(sync_cycles, ESP32P4TraceState),
        VMSTATE_BOOL(resync_due, ESP32P4TraceState),
        VMSTATE_BOOL(trap_reported, ESP32P4TraceState),
        VMSTATE_UINT32(last_addr, ESP32P4TraceState),
        VMSTATE_UINT16(index, ESP32P4TraceState),
        VMSTATE_UINT32(anchor_count, ESP32P4TraceState),
        VMSTATE_BOOL(lost_owed, ESP32P4TraceState),
        VMSTATE_BOOL(ran, ESP32P4TraceState),
        VMSTATE_BOOL(trap_match, ESP32P4TraceState),
        VMSTATE_UINT32(trap_depth, ESP32P4TraceState),
        VMSTATE_BOOL(comp_latch, ESP32P4TraceState),
        VMSTATE_END_OF_LIST()
    },
};

static const Property esp32p4_trace_properties[] = {
    DEFINE_PROP_LINK("cpu", ESP32P4TraceState, cpu, TYPE_ESP32P4_CPU,
                     ESP32P4CPU *),
    DEFINE_PROP_LINK("pms", ESP32P4TraceState, pms, TYPE_ESP32P4_PMS,
                     ESP32P4PmsState *),
    DEFINE_PROP_UINT8("core", ESP32P4TraceState, core, 0),
};

static void esp32p4_trace_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = esp32p4_trace_realize;
    rc->phases.hold = esp32p4_trace_reset_hold;
    rc->phases.exit = esp32p4_trace_reset_exit;
    dc->vmsd = &vmstate_esp32p4_trace;
    device_class_set_props(dc, esp32p4_trace_properties);
}

/* [spec:nuos:req:emu.esp32p4.debug] */
static const TypeInfo esp32p4_trace_info = {
    .name              = TYPE_ESP32P4_TRACE,
    .parent            = TYPE_SYS_BUS_DEVICE,
    .instance_size     = sizeof(ESP32P4TraceState),
    .instance_init     = esp32p4_trace_init,
    .instance_finalize = esp32p4_trace_finalize,
    .class_init        = esp32p4_trace_class_init,
};

static void esp32p4_trace_register_types(void)
{
    type_register_static(&esp32p4_trace_info);
}
type_init(esp32p4_trace_register_types)
