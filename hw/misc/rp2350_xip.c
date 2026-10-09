/*
 * RP2350 XIP subsystem: XIP_CTRL, the QSPI memory interface and XIP_AUX
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: RP2350 Datasheet, "External flash and PSRAM (XIP)" and "QSPI
 * memory interface (QMI)"; register layouts from the pico-sdk xip.h, qmi.h
 * and xip_aux.h.
 *
 * The 26-bit XIP space is mirrored four times from 0x10000000, decoded on
 * address bits 27:26: cached, uncached, cache maintenance (write-only), and
 * uncached-untranslated. Its lower half holds the two QMI chip select
 * windows, 16 MiB each; the upper half reaches no device, but cache lines
 * can be pinned there for use as SRAM.
 *
 * The devices on the two chip selects are QEMU SSI devices on the "qspi"
 * bus. Direct mode drives them byte by byte through the DIRECT_TX/RX FIFOs,
 * so commands such as JEDEC ID, status, erase and program act on the
 * device itself. Memory-mapped reads are served from the device's array,
 * a ROM device region which the SSI device keeps coherent: this is what a
 * correctly configured read command returns. The read format registers
 * are kept but not interpreted, so a format the device would not
 * understand still returns array data. Memory-mapped writes, when the
 * window is writable, are issued as serial transfers in the M0/M1 write
 * format, so a flash ignores them (its write enable latch is clear) while
 * a PSRAM stores them.
 *
 * Access model. Every access to the XIP space reaches `space_io`, which
 * implements the full behaviour: security checks from CTRL, address
 * translation and its bus errors, direct-mode bus errors, pinned cache
 * lines, the access counters, and the cache maintenance window. Where
 * that behaviour is the same for every access, views map the arrays and
 * the cache data RAM directly over it, so that code runs from flash at
 * RAM speed; the views are rebuilt whenever the mapping changes.
 *
 * The cache itself is not modelled beyond its tag memory for pinned lines:
 * an unpinned access behaves as a miss, so the XIP space is always
 * coherent with the devices, where hardware may return stale lines until
 * they are invalidated. Clean operations have no dirty lines to write
 * back, so RP2350-E11 cannot occur. The counters count only the accesses
 * that reach `space_io`; accesses through the direct views are not seen.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/rp2350_atomic.h"
#include "hw/misc/rp2350_xip.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"

#define A_CTRL          0x00
#define A_STAT          0x08
#define A_CTR_HIT       0x0c
#define A_CTR_ACC       0x10
#define A_STREAM_ADDR   0x14
#define A_STREAM_CTR    0x18
#define A_STREAM_FIFO   0x1c

#define CTRL_EN_SECURE              (1u << 0)
#define CTRL_EN_NONSECURE           (1u << 1)
#define CTRL_POWER_DOWN             (1u << 3)
#define CTRL_NO_UNCACHED_SEC        (1u << 4)
#define CTRL_NO_UNCACHED_NONSEC     (1u << 5)
#define CTRL_NO_UNTRANSLATED_SEC    (1u << 6)
#define CTRL_NO_UNTRANSLATED_NONSEC (1u << 7)
#define CTRL_MAINT_NONSEC           (1u << 8)
#define CTRL_SPLIT_WAYS             (1u << 9)
#define CTRL_WRITABLE_M0            (1u << 10)
#define CTRL_MASK                   0x00000ffb
#define CTRL_RESET                  0x00000083

#define STAT_FIFO_EMPTY             (1u << 1)
#define STAT_FIFO_FULL              (1u << 2)

#define STREAM_ADDR_MASK            0xfffffffc
#define STREAM_CTR_MASK             0x003fffff
#define STREAM_FIFO_DEPTH           2

#define A_DIRECT_CSR    0x00
#define A_DIRECT_TX     0x04
#define A_DIRECT_RX     0x08
#define A_M0_TIMING     0x0c
#define A_M1_WCMD       0x30
#define A_ATRANS0       0x34
#define A_ATRANS7       0x50
#define M_STRIDE        0x14

#define CSR_EN                      (1u << 0)
#define CSR_BUSY                    (1u << 1)
#define CSR_ASSERT_CS0N             (1u << 2)
#define CSR_AUTO_CS0N               (1u << 6)
#define CSR_TXFULL                  (1u << 10)
#define CSR_TXEMPTY                 (1u << 11)
#define CSR_TXLEVEL_SHIFT           12
#define CSR_RXEMPTY                 (1u << 16)
#define CSR_RXFULL                  (1u << 17)
#define CSR_RXLEVEL_SHIFT           18
#define CSR_CLKDIV_SHIFT            22
#define CSR_WRITABLE                0xffc000cd
#define CSR_RESET                   0x01800000

#define TX_DATA                     0x0000ffff
#define TX_IWIDTH_SHIFT             16
#define TX_DWIDTH                   (1u << 18)
#define TX_OE                       (1u << 19)
#define TX_NOPUSH                   (1u << 20)
#define TX_MASK                     0x001fffff
#define DIRECT_FIFO_DEPTH           4

#define TIMING_MASK                 0xf3fff7ff
#define TIMING_RESET                0x40000004
#define TIMING_CLKDIV_MASK          0xff
#define FMT_MASK                    0x1007d3ff
#define FMT_RESET                   0x00001000
#define FMT_DATA_WIDTH_SHIFT        8
#define FMT_PREFIX_LEN              (1u << 12)
#define FMT_SUFFIX_LEN_SHIFT        14
#define FMT_DUMMY_LEN_SHIFT         16
#define CMD_MASK                    0x0000ffff
#define RCMD_RESET                  0x0000a003
#define WCMD_RESET                  0x0000a002
#define ATRANS_MASK                 0x07ff0fff
#define ATRANS_SIZE_SHIFT           16
#define ATRANS_SIZE_MASK            0x7ff
#define ATRANS_BASE_MASK            0xfff
#define ATRANS_PANE_PAGES           0x400

#define A_AUX_STREAM        0x0
#define A_AUX_QMI_DIRECT_TX 0x4
#define A_AUX_QMI_DIRECT_RX 0x8
#define RP2350_XIP_AUX_SIZE 0x100000

/* The XIP space mirrors, on address bits 27:26. */
enum {
    XIP_CACHED = 0,
    XIP_UNCACHED = 1,
    XIP_MAINTENANCE = 2,
    XIP_UNTRANSLATED = 3,
};
#define XIP_MIRROR_SIZE (64 * MiB)
#define XIP_ADDR_MASK (XIP_MIRROR_SIZE - 1)
#define QMI_SPACE_SIZE (RP2350_QMI_CS * RP2350_QMI_WINDOW_SIZE)
#define PANE_SIZE (4 * MiB)
#define SECTOR_SIZE (4 * KiB)

/* Cache tag entries: the line's address bits 25:3, and its state. */
#define TAG_PINNED      (1u << 31)
#define TAG_ADDR_MASK   0x03fffff8
#define CACHE_SETS      1024
#define CACHE_LINE      8

/* Cache maintenance operations, on address bits 2:0. */
enum {
    MAINT_INVALIDATE_SET_WAY = 0,
    MAINT_CLEAN_SET_WAY = 1,
    MAINT_INVALIDATE_ADDR = 2,
    MAINT_CLEAN_ADDR = 3,
    MAINT_PIN = 7,
};

/*
 * Each chip select's array is seen through a 32 MiB container holding
 * repeated copies of it: the device ignores address bits above its size,
 * and a translated pane can extend past 16 MiB, where translation wraps.
 */
#define MIRROR_SPAN (2 * RP2350_QMI_WINDOW_SIZE)

static bool secure_access(MemTxAttrs attrs)
{
    /* Debugger and qtest accesses carry no attributes; treat as Secure. */
    return attrs.secure || attrs.unspecified;
}

static void counter_inc(uint32_t *ctr)
{
    if (*ctr != UINT32_MAX) {
        (*ctr)++;
    }
}

static int64_t sysclk_ns(RP2350XIPState *s, uint64_t cycles)
{
    return muldiv64(cycles, NANOSECONDS_PER_SECOND, s->sysclk_hz);
}

static unsigned clkdiv(uint32_t field)
{
    /* Divisors 1..255 are encoded directly; 0 encodes 256. */
    return field ? field : 256;
}

static unsigned width_bits(unsigned width)
{
    /* Single, dual and quad width; the reserved encoding counts as quad. */
    return width == 0 ? 1 : width == 1 ? 2 : 4;
}

static uint8_t *array_ptr(RP2350XIPState *s, int cs)
{
    return memory_region_get_ram_ptr(&s->array[cs].mr);
}

static bool writable(RP2350XIPState *s, int cs)
{
    return s->ctrl & (CTRL_WRITABLE_M0 << cs);
}

/* Chip selects, DREQs and the direct-mode serial interface. */

static bool direct_busy(RP2350XIPState *s)
{
    if ((s->direct_csr & CSR_EN) && !fifo32_is_empty(&s->direct_tx)) {
        return true;
    }
    return qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) < s->direct_busy_until;
}

static void rp2350_xip_update_cs(RP2350XIPState *s)
{
    bool busy = direct_busy(s);
    int i;

    for (i = 0; i < RP2350_QMI_CS; i++) {
        bool asserted = (s->direct_csr & (CSR_ASSERT_CS0N << i)) ||
                        ((s->direct_csr & (CSR_AUTO_CS0N << i)) && busy) ||
                        s->mm_cs == i;
        int level = !asserted;

        if (level != s->cs_level[i]) {
            s->cs_level[i] = level;
            qemu_set_irq(s->cs[i], level);
        }
    }
}

static void rp2350_xip_update_dreq(RP2350XIPState *s)
{
    qemu_set_irq(s->dreq[RP2350_XIP_DREQ_STREAM],
                 !fifo32_is_empty(&s->stream_fifo));
    qemu_set_irq(s->dreq[RP2350_XIP_DREQ_QMI_TX],
                 !fifo32_is_full(&s->direct_tx));
    qemu_set_irq(s->dreq[RP2350_XIP_DREQ_QMI_RX],
                 !fifo32_is_empty(&s->direct_rx));
}

/* One byte each way on the QSPI bus. */
static uint8_t qspi_byte(RP2350XIPState *s, uint8_t tx)
{
    return ssi_transfer(s->qspi, tx);
}

/*
 * Run queued direct-mode frames. Data moves at once; BUSY, and with it
 * any AUTO_CSxN chip select, stays up for as long as the frames take on
 * the serial interface, plus the half SCK period of chip select hold.
 * A full RX FIFO stalls the interface until software pops it.
 */
/* [spec:nuos:req:emu.xip+1] */
static void rp2350_xip_direct_run(RP2350XIPState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    unsigned div = clkdiv(extract32(s->direct_csr, CSR_CLKDIV_SHIFT, 8));

    while ((s->direct_csr & CSR_EN) && !fifo32_is_empty(&s->direct_tx) &&
           !fifo32_is_full(&s->direct_rx)) {
        uint32_t tx = fifo32_pop(&s->direct_tx);
        unsigned iwidth = extract32(tx, TX_IWIDTH_SHIFT, 2);
        unsigned bytes = (tx & TX_DWIDTH) ? 2 : 1;
        uint64_t cycles = (uint64_t)bytes * 8 / width_bits(iwidth) * div;
        uint32_t rx = 0;
        unsigned i;

        if (iwidth == 3) {
            qemu_log_mask(LOG_GUEST_ERROR, "rp2350-xip: DIRECT_TX IWIDTH 3 "
                          "is reserved\n");
        }
        s->direct_busy_until = MAX(now, s->direct_busy_until) +
                               sysclk_ns(s, cycles + div / 2);
        rp2350_xip_update_cs(s);
        /* 16-bit records go least significant byte first, both ways. */
        for (i = 0; i < bytes; i++) {
            uint8_t out = tx >> (8 * i);
            uint8_t in = qspi_byte(s, out);

            /* At dual/quad width with OE set, the pads read our output. */
            if (iwidth && (tx & TX_OE)) {
                in = out;
            }
            rx |= in << (8 * i);
        }
        if (!(tx & TX_NOPUSH)) {
            fifo32_push(&s->direct_rx, rx);
        }
    }
    if (s->direct_busy_until > now) {
        timer_mod(s->busy_timer, s->direct_busy_until);
    }
    rp2350_xip_update_cs(s);
    rp2350_xip_update_dreq(s);
}

static void rp2350_xip_busy_done(void *opaque)
{
    RP2350XIPState *s = opaque;

    rp2350_xip_update_cs(s);
}

static void rp2350_xip_direct_push(RP2350XIPState *s, uint32_t value)
{
    if (fifo32_is_full(&s->direct_tx)) {
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-xip: DIRECT_TX push while "
                      "full; data ignored\n");
        return;
    }
    fifo32_push(&s->direct_tx, value & TX_MASK);
    rp2350_xip_direct_run(s);
}

static uint32_t rp2350_xip_direct_pop(RP2350XIPState *s)
{
    uint32_t value;

    if (fifo32_is_empty(&s->direct_rx)) {
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-xip: DIRECT_RX pop while "
                      "empty\n");
        return 0;
    }
    value = fifo32_pop(&s->direct_rx);
    rp2350_xip_direct_run(s);
    return value;
}

static uint32_t rp2350_xip_direct_csr(RP2350XIPState *s)
{
    uint32_t tx = fifo32_num_used(&s->direct_tx);
    uint32_t rx = fifo32_num_used(&s->direct_rx);
    uint32_t v = s->direct_csr;

    v |= tx << CSR_TXLEVEL_SHIFT;
    v |= rx << CSR_RXLEVEL_SHIFT;
    v |= tx == 0 ? CSR_TXEMPTY : 0;
    v |= tx == DIRECT_FIFO_DEPTH ? CSR_TXFULL : 0;
    v |= rx == 0 ? CSR_RXEMPTY : 0;
    v |= rx == DIRECT_FIFO_DEPTH ? CSR_RXFULL : 0;
    v |= direct_busy(s) ? CSR_BUSY : 0;
    return v;
}

/* QMI memory-mapped transfers. */

/*
 * Turn a 26-bit XIP address into a chip select and the 24-bit address
 * sent to that device, applying the pane's ATRANS mapping unless
 * `translate` is false.
 */
/* [spec:nuos:req:emu.xip+1] */
static MemTxResult qmi_translate(RP2350XIPState *s, uint32_t xa,
                                 bool translate, int *cs, uint32_t *phys)
{
    uint32_t off = xa & (RP2350_QMI_WINDOW_SIZE - 1);

    if (xa >= QMI_SPACE_SIZE) {
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-xip: access to 0x%07x, "
                      "outside both QMI windows\n", xa);
        return MEMTX_DECODE_ERROR;
    }
    *cs = xa / RP2350_QMI_WINDOW_SIZE;
    if (translate) {
        uint32_t at = s->atrans[*cs * RP2350_QMI_PANES + off / PANE_SIZE];
        uint32_t size = extract32(at, ATRANS_SIZE_SHIFT, 11);
        uint32_t base = at & ATRANS_BASE_MASK;

        off &= PANE_SIZE - 1;
        if (off / SECTOR_SIZE >= size) {
            qemu_log_mask(LOG_GUEST_ERROR, "rp2350-xip: access to 0x%07x is "
                          "beyond its ATRANS%d size\n", xa,
                          (int)(xa / PANE_SIZE));
            return MEMTX_ERROR;
        }
        off = (off + base * SECTOR_SIZE) & (RP2350_QMI_WINDOW_SIZE - 1);
    }
    *phys = off;
    return MEMTX_OK;
}

static MemTxResult qmi_read_byte(RP2350XIPState *s, uint32_t xa,
                                 bool translate, uint8_t *data)
{
    MemTxResult r;
    uint32_t phys;
    int cs;

    if (s->direct_csr & CSR_EN) {
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-xip: memory-mapped access "
                      "while QMI direct mode is enabled\n");
        return MEMTX_ERROR;
    }
    r = qmi_translate(s, xa, translate, &cs, &phys);
    if (r != MEMTX_OK) {
        return r;
    }
    /* With no device on the chip select, the pulled-down bus reads 0. */
    *data = s->cs_size[cs] ? array_ptr(s, cs)[phys & (s->cs_size[cs] - 1)]
                           : 0;
    return MEMTX_OK;
}

/*
 * A memory-mapped write as the QMI issues it: prefix, 24-bit address,
 * suffix, dummy and data phases from the window's WFMT and WCMD.
 */
/* [spec:nuos:req:emu.xip+1] */
static void qmi_write_byte(RP2350XIPState *s, int cs, uint32_t phys,
                           uint8_t data)
{
    uint32_t fmt = s->wfmt[cs];
    uint32_t cmd = s->wcmd[cs];
    unsigned suffix = extract32(fmt, FMT_SUFFIX_LEN_SHIFT, 2);
    unsigned dummy_bits = extract32(fmt, FMT_DUMMY_LEN_SHIFT, 3) * 4;
    unsigned i;

    s->mm_cs = cs;
    rp2350_xip_update_cs(s);
    if (fmt & FMT_PREFIX_LEN) {
        qspi_byte(s, cmd);
    }
    qspi_byte(s, phys >> 16);
    qspi_byte(s, phys >> 8);
    qspi_byte(s, phys);
    if (suffix == 2) {
        qspi_byte(s, cmd >> 8);
    } else if (suffix) {
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-xip: M%d_WFMT SUFFIX_LEN %u "
                      "is not supported\n", cs, suffix);
    }
    for (i = 0; i < DIV_ROUND_UP(dummy_bits, 8); i++) {
        qspi_byte(s, 0);
    }
    qspi_byte(s, data);
    s->mm_cs = -1;
    rp2350_xip_update_cs(s);
}

/* The cache's tag memory: pinned lines only. */

/* The line holding `xa` for this access, or -1 on a miss. */
static int cache_lookup(RP2350XIPState *s, uint32_t xa, bool secure)
{
    uint32_t en = secure ? CTRL_EN_SECURE : CTRL_EN_NONSECURE;
    unsigned set = (xa / CACHE_LINE) % CACHE_SETS;
    int way;

    if (!(s->ctrl & en)) {
        return -1;
    }
    for (way = 0; way < 2; way++) {
        int line = way * CACHE_SETS + set;

        /* Split ways: Secure uses way 0 only, Non-secure way 1 only. */
        if ((s->ctrl & CTRL_SPLIT_WAYS) && way != !secure) {
            continue;
        }
        if ((s->tag[line] & TAG_PINNED) &&
            (s->tag[line] & TAG_ADDR_MASK) == (xa & TAG_ADDR_MASK)) {
            return line;
        }
    }
    return -1;
}

static uint8_t *cache_ptr(RP2350XIPState *s, int line, uint32_t xa)
{
    return (uint8_t *)memory_region_get_ram_ptr(&s->cache_data) +
           line * CACHE_LINE + xa % CACHE_LINE;
}

static void rp2350_xip_remap(RP2350XIPState *s);

static bool cache_invalidate_addr(RP2350XIPState *s, uint32_t xa)
{
    unsigned set = (xa / CACHE_LINE) % CACHE_SETS;
    bool changed = false;
    int way;

    for (way = 0; way < 2; way++) {
        uint32_t *tag = &s->tag[way * CACHE_SETS + set];

        if ((*tag & TAG_PINNED) &&
            (*tag & TAG_ADDR_MASK) == (xa & TAG_ADDR_MASK)) {
            *tag = 0;
            changed = true;
        }
    }
    return changed;
}

/*
 * A write to the maintenance window: address bits 2:0 select the
 * operation, bits 13:3 the line for set/way operations. Only pinned lines
 * have state here, so cleans, which write back dirty lines, change
 * nothing.
 */
/* [spec:nuos:req:emu.xip+1] */
static void cache_maintain(RP2350XIPState *s, uint32_t xa)
{
    int line = (xa / CACHE_LINE) % RP2350_XIP_CACHE_LINES;
    bool changed = false;

    if (s->ctrl & CTRL_POWER_DOWN) {
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-xip: cache maintenance while "
                      "the cache is powered down\n");
        return;
    }
    switch (xa & (CACHE_LINE - 1)) {
    case MAINT_INVALIDATE_SET_WAY:
        changed = s->tag[line] & TAG_PINNED;
        s->tag[line] = 0;
        break;
    case MAINT_CLEAN_SET_WAY:
    case MAINT_CLEAN_ADDR:
        break;
    case MAINT_INVALIDATE_ADDR:
        changed = cache_invalidate_addr(s, xa);
        break;
    case MAINT_PIN:
        /* An implicit invalidate by address keeps the address unique. */
        cache_invalidate_addr(s, xa);
        s->tag[line] = TAG_PINNED | (xa & TAG_ADDR_MASK);
        changed = true;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-xip: reserved cache "
                      "maintenance operation %u\n", xa & (CACHE_LINE - 1));
        break;
    }
    if (changed) {
        rp2350_xip_remap(s);
    }
}

/* The XIP space access model. */

static bool window_denied(RP2350XIPState *s, unsigned win, bool secure)
{
    uint32_t bit;

    switch (win) {
    case XIP_UNCACHED:
        bit = secure ? CTRL_NO_UNCACHED_SEC : CTRL_NO_UNCACHED_NONSEC;
        break;
    case XIP_UNTRANSLATED:
        bit = secure ? CTRL_NO_UNTRANSLATED_SEC : CTRL_NO_UNTRANSLATED_NONSEC;
        break;
    default:
        return false;
    }
    if (s->ctrl & bit) {
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-xip: %s access to the "
                      "%s window is disabled by CTRL\n",
                      secure ? "Secure" : "Non-secure",
                      win == XIP_UNCACHED ? "uncached" : "untranslated");
        return true;
    }
    return false;
}

/* [spec:nuos:req:emu.xip+1] */
static MemTxResult rp2350_xip_space_read(void *opaque, hwaddr addr,
                                         uint64_t *data, unsigned size,
                                         MemTxAttrs attrs)
{
    RP2350XIPState *s = opaque;
    unsigned win = addr / XIP_MIRROR_SIZE;
    bool secure = secure_access(attrs);
    bool hit = false;
    uint64_t v = 0;
    unsigned i;

    counter_inc(&s->ctr_acc);
    if (win == XIP_MAINTENANCE) {
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-xip: read from the write-only "
                      "cache maintenance window at 0x%" HWADDR_PRIx "\n",
                      addr);
        return MEMTX_ERROR;
    }
    if (window_denied(s, win, secure)) {
        return MEMTX_ERROR;
    }
    for (i = 0; i < size; i++) {
        uint32_t xa = (addr + i) & XIP_ADDR_MASK;
        int line = win == XIP_CACHED ? cache_lookup(s, xa, secure) : -1;
        uint8_t byte;

        if (line >= 0) {
            byte = *cache_ptr(s, line, xa);
            hit = true;
        } else {
            MemTxResult r = qmi_read_byte(s, xa, win != XIP_UNTRANSLATED,
                                          &byte);

            if (r != MEMTX_OK) {
                return r;
            }
        }
        v |= (uint64_t)byte << (8 * i);
    }
    if (hit) {
        counter_inc(&s->ctr_hit);
    }
    *data = v;
    return MEMTX_OK;
}

/* [spec:nuos:req:emu.xip+1] */
static MemTxResult rp2350_xip_space_write(void *opaque, hwaddr addr,
                                          uint64_t value, unsigned size,
                                          MemTxAttrs attrs)
{
    RP2350XIPState *s = opaque;
    unsigned win = addr / XIP_MIRROR_SIZE;
    bool secure = secure_access(attrs);
    bool hit = false;
    unsigned i;

    counter_inc(&s->ctr_acc);
    if (win == XIP_MAINTENANCE) {
        if (!secure && !(s->ctrl & CTRL_MAINT_NONSEC)) {
            qemu_log_mask(LOG_GUEST_ERROR, "rp2350-xip: Non-secure cache "
                          "maintenance without CTRL.MAINT_NONSEC\n");
            return MEMTX_ERROR;
        }
        cache_maintain(s, addr & XIP_ADDR_MASK);
        return MEMTX_OK;
    }
    if (window_denied(s, win, secure)) {
        return MEMTX_ERROR;
    }
    for (i = 0; i < size; i++) {
        uint32_t xa = (addr + i) & XIP_ADDR_MASK;
        uint8_t byte = value >> (8 * i);
        int line = win == XIP_CACHED ? cache_lookup(s, xa, secure) : -1;
        uint32_t phys;
        MemTxResult r;
        int cs;

        /*
         * A read-only window downgrades writes to reads: a pinned line
         * keeps its data, and anything else is read, which still faults
         * where a read would.
         */
        if (line >= 0) {
            hit = true;
            if (xa >= QMI_SPACE_SIZE ||
                writable(s, xa / RP2350_QMI_WINDOW_SIZE)) {
                hwaddr off = line * CACHE_LINE + xa % CACHE_LINE;

                address_space_write(&s->cache_as, off, MEMTXATTRS_UNSPECIFIED,
                                    &byte, 1);
            }
            continue;
        }
        if (s->direct_csr & CSR_EN) {
            qemu_log_mask(LOG_GUEST_ERROR, "rp2350-xip: memory-mapped access "
                          "while QMI direct mode is enabled\n");
            return MEMTX_ERROR;
        }
        r = qmi_translate(s, xa, win != XIP_UNTRANSLATED, &cs, &phys);
        if (r != MEMTX_OK) {
            return r;
        }
        if (writable(s, cs)) {
            qmi_write_byte(s, cs, phys, byte);
        }
    }
    if (hit) {
        counter_inc(&s->ctr_hit);
    }
    return MEMTX_OK;
}

static const MemoryRegionOps rp2350_xip_space_ops = {
    .read_with_attrs = rp2350_xip_space_read,
    .write_with_attrs = rp2350_xip_space_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .valid.unaligned = true,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
    .impl.unaligned = true,
};

/*
 * The arrays, mapped directly where the access model allows. Reads come
 * from RAM; writes land here, already translated, and have passed every
 * check but WRITABLE_Mx.
 */
static uint64_t rp2350_xip_array_read(void *opaque, hwaddr addr,
                                      unsigned size)
{
    RP2350XIPArray *a = opaque;
    uint64_t v = 0;

    memcpy(&v, array_ptr(a->xip, a->cs) + addr, size);
    return le64_to_cpu(v);
}

static void rp2350_xip_array_write(void *opaque, hwaddr addr, uint64_t value,
                                   unsigned size)
{
    RP2350XIPArray *a = opaque;
    unsigned i;

    if (!writable(a->xip, a->cs)) {
        return;
    }
    for (i = 0; i < size; i++) {
        qmi_write_byte(a->xip, a->cs, addr + i, value >> (8 * i));
    }
}

static const MemoryRegionOps rp2350_xip_array_ops = {
    .read = rp2350_xip_array_read,
    .write = rp2350_xip_array_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .valid.unaligned = true,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
    .impl.unaligned = true,
};

/* Fast views. */

typedef struct PinRun {
    uint32_t start;
    uint32_t len;
} PinRun;

static gint pin_run_cmp(gconstpointer a, gconstpointer b)
{
    uint32_t x = ((const PinRun *)a)->start;
    uint32_t y = ((const PinRun *)b)->start;

    return x < y ? -1 : x > y;
}

/*
 * Group the pinned lines into runs of consecutive addresses whose data
 * is contiguous in the cache: a line pinned at address A sits in way
 * A[13], set A[12:3], so runs break only at 16 KiB boundaries. Returns
 * the number of runs, which may exceed the size of `runs`.
 */
static int collect_pin_runs(RP2350XIPState *s, PinRun *runs, int max)
{
    g_autofree PinRun *lines = g_new(PinRun, RP2350_XIP_CACHE_LINES);
    int n = 0, nruns = 0, i;

    for (i = 0; i < RP2350_XIP_CACHE_LINES; i++) {
        if (s->tag[i] & TAG_PINNED) {
            lines[n].start = s->tag[i] & TAG_ADDR_MASK;
            lines[n].len = CACHE_LINE;
            n++;
        }
    }
    qsort(lines, n, sizeof(*lines), pin_run_cmp);
    for (i = 0; i < n; i++) {
        PinRun *last = nruns && nruns <= max ? &runs[nruns - 1] : NULL;

        if (last && last->start + last->len == lines[i].start &&
            lines[i].start % RP2350_XIP_CACHE_SIZE) {
            last->len += CACHE_LINE;
            continue;
        }
        if (nruns < max) {
            runs[nruns] = lines[i];
        }
        nruns++;
    }
    return nruns;
}

static void set_view(MemoryRegion *mr, bool enabled, hwaddr addr,
                     hwaddr offset, uint64_t size)
{
    if (enabled && size) {
        memory_region_set_alias_offset(mr, offset);
        memory_region_set_size(mr, size);
        memory_region_set_address(mr, addr);
    }
    memory_region_set_enabled(mr, enabled && size);
}

/*
 * Map the arrays and the pinned cache lines directly wherever every access
 * would see exactly what space_io gives it.
 */
/* [spec:nuos:req:emu.xip+1] */
static void rp2350_xip_remap(RP2350XIPState *s)
{
    PinRun runs[RP2350_XIP_PIN_VIEWS] = { };
    int nruns = collect_pin_runs(s, runs, RP2350_XIP_PIN_VIEWS);
    bool pins_mapped = nruns <= RP2350_XIP_PIN_VIEWS;
    bool direct = s->direct_csr & CSR_EN;
    bool uncached = !(s->ctrl & (CTRL_NO_UNCACHED_SEC |
                                 CTRL_NO_UNCACHED_NONSEC));
    bool untranslated = !(s->ctrl & (CTRL_NO_UNTRANSLATED_SEC |
                                     CTRL_NO_UNTRANSLATED_NONSEC));
    uint32_t en = s->ctrl & (CTRL_EN_SECURE | CTRL_EN_NONSECURE);
    /* Pinned lines look the same to every access only in this state. */
    bool pins_ram = en == (CTRL_EN_SECURE | CTRL_EN_NONSECURE) &&
                    !(s->ctrl & CTRL_SPLIT_WAYS);
    int cs, p, i;

    memory_region_transaction_begin();
    for (cs = 0; cs < RP2350_QMI_CS; cs++) {
        hwaddr window = cs * RP2350_QMI_WINDOW_SIZE;
        bool present = !direct;

        if (!s->cs_size[cs]) {
            continue;
        }
        for (p = 0; p < RP2350_QMI_PANES; p++) {
            uint32_t at = s->atrans[cs * RP2350_QMI_PANES + p];
            uint64_t size = MIN(extract32(at, ATRANS_SIZE_SHIFT, 11),
                                ATRANS_PANE_PAGES) * SECTOR_SIZE;
            hwaddr base = (at & ATRANS_BASE_MASK) * SECTOR_SIZE;
            hwaddr virt = window + p * PANE_SIZE;

            /*
             * Too many pinned runs to overlay: leave the cached window to
             * space_io, which still serves the pinned lines.
             */
            set_view(&s->view[cs][p], present && pins_mapped,
                     XIP_CACHED * XIP_MIRROR_SIZE + virt, base, size);
            set_view(&s->view[cs][RP2350_QMI_PANES + p], present && uncached,
                     XIP_UNCACHED * XIP_MIRROR_SIZE + virt, base, size);
        }
        set_view(&s->view[cs][2 * RP2350_QMI_PANES], present && untranslated,
                 XIP_UNTRANSLATED * XIP_MIRROR_SIZE + window, 0,
                 RP2350_QMI_WINDOW_SIZE);
    }
    for (i = 0; i < RP2350_XIP_PIN_VIEWS; i++) {
        bool used = pins_mapped && i < nruns && en;
        bool ram = pins_ram;

        /*
         * Lines in a read-only QMI window drop writes, which a RAM view
         * cannot do for every bus master, so the access model keeps them.
         */
        if (used && runs[i].start < QMI_SPACE_SIZE &&
            !writable(s, runs[i].start / RP2350_QMI_WINDOW_SIZE)) {
            ram = false;
        }
        set_view(&s->pin_ram[i], used && ram, runs[i].start,
                 runs[i].start % RP2350_XIP_CACHE_SIZE, runs[i].len);
        set_view(&s->pin_io[i], used && !ram, runs[i].start,
                 runs[i].start, runs[i].len);
    }
    memory_region_transaction_commit();
}

/* The XIP streaming FIFO. */

static void rp2350_xip_stream_schedule(RP2350XIPState *s)
{
    uint32_t xa = s->stream_addr & XIP_ADDR_MASK;
    int cs = xa >= RP2350_QMI_WINDOW_SIZE;
    unsigned width = width_bits(extract32(s->rfmt[cs],
                                          FMT_DATA_WIDTH_SHIFT, 2));
    uint64_t cycles = 32 / width * clkdiv(s->timing[cs] & TIMING_CLKDIV_MASK);

    if (s->stream_ctr && !fifo32_is_full(&s->stream_fifo) &&
        !(s->direct_csr & CSR_EN) && !timer_pending(s->stream_timer)) {
        timer_mod(s->stream_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                                   sysclk_ns(s, cycles));
    }
}

/*
 * One streamed word: a linear read through the QMI at STREAM_ADDR, in the
 * background, each taking the serial time of its data phase.
 */
/* [spec:nuos:req:emu.xip+1] */
static void rp2350_xip_stream_tick(void *opaque)
{
    RP2350XIPState *s = opaque;
    unsigned win = (s->stream_addr / XIP_MIRROR_SIZE) % 4;
    uint32_t word = 0;
    int i;

    if (!s->stream_ctr || fifo32_is_full(&s->stream_fifo) ||
        (s->direct_csr & CSR_EN)) {
        return;
    }
    for (i = 0; i < 4; i++) {
        uint32_t xa = (s->stream_addr + i) & XIP_ADDR_MASK;
        uint8_t byte;

        if (qmi_read_byte(s, xa, win != XIP_UNTRANSLATED, &byte) !=
            MEMTX_OK) {
            qemu_log_mask(LOG_GUEST_ERROR, "rp2350-xip: stream read of "
                          "0x%08x failed; stream halted\n", s->stream_addr);
            s->stream_ctr = 0;
            return;
        }
        word |= byte << (8 * i);
    }
    fifo32_push(&s->stream_fifo, word);
    s->stream_addr += 4;
    s->stream_ctr--;
    rp2350_xip_update_dreq(s);
    rp2350_xip_stream_schedule(s);
}

static uint32_t rp2350_xip_stream_pop(RP2350XIPState *s)
{
    uint32_t v = 0;

    if (fifo32_is_empty(&s->stream_fifo)) {
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-xip: STREAM_FIFO read while "
                      "empty\n");
    } else {
        v = fifo32_pop(&s->stream_fifo);
    }
    rp2350_xip_update_dreq(s);
    rp2350_xip_stream_schedule(s);
    return v;
}

/* XIP_CTRL registers. */

static uint32_t rp2350_xip_ctrl_peek(RP2350XIPState *s, hwaddr reg)
{
    switch (reg) {
    case A_CTRL:
        return s->ctrl;
    case A_STAT:
        return (fifo32_is_empty(&s->stream_fifo) ? STAT_FIFO_EMPTY : 0) |
               (fifo32_is_full(&s->stream_fifo) ? STAT_FIFO_FULL : 0);
    case A_CTR_HIT:
        return s->ctr_hit;
    case A_CTR_ACC:
        return s->ctr_acc;
    case A_STREAM_ADDR:
        return s->stream_addr;
    case A_STREAM_CTR:
        return s->stream_ctr;
    default:
        return 0;
    }
}

/* [spec:nuos:req:emu.xip+1] */
static MemTxResult rp2350_xip_ctrl_read(void *opaque, hwaddr addr,
                                        uint64_t *data, unsigned size,
                                        MemTxAttrs attrs)
{
    RP2350XIPState *s = opaque;
    hwaddr reg = rp2350_atomic_reg(addr);

    switch (reg) {
    case A_CTRL:
    case A_STAT:
    case A_CTR_HIT:
    case A_CTR_ACC:
    case A_STREAM_ADDR:
    case A_STREAM_CTR:
        *data = rp2350_xip_ctrl_peek(s, reg);
        break;
    case A_STREAM_FIFO:
        *data = rp2350_xip_stream_pop(s);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-xip: read of bad XIP_CTRL "
                      "offset 0x%" HWADDR_PRIx "\n", addr);
        *data = 0;
        break;
    }
    return MEMTX_OK;
}

/* [spec:nuos:req:emu.xip+1] */
static MemTxResult rp2350_xip_ctrl_write(void *opaque, hwaddr addr,
                                         uint64_t value, unsigned size,
                                         MemTxAttrs attrs)
{
    RP2350XIPState *s = opaque;
    hwaddr reg = rp2350_atomic_reg(addr);
    uint32_t v = rp2350_atomic_apply(addr, rp2350_xip_ctrl_peek(s, reg),
                                     value);

    switch (reg) {
    case A_CTRL:
        if (!secure_access(attrs)) {
            qemu_log_mask(LOG_GUEST_ERROR, "rp2350-xip: CTRL is read-only "
                          "from Non-secure\n");
            break;
        }
        v &= CTRL_MASK;
        /* The cache cannot be enabled while it is powered down. */
        if (v & CTRL_POWER_DOWN) {
            v &= ~(CTRL_EN_SECURE | CTRL_EN_NONSECURE);
        }
        if (v != s->ctrl) {
            s->ctrl = v;
            rp2350_xip_remap(s);
        }
        break;
    case A_CTR_HIT:
        s->ctr_hit = 0;
        break;
    case A_CTR_ACC:
        s->ctr_acc = 0;
        break;
    case A_STREAM_ADDR:
        s->stream_addr = v & STREAM_ADDR_MASK;
        break;
    case A_STREAM_CTR:
        s->stream_ctr = v & STREAM_CTR_MASK;
        if (!s->stream_ctr) {
            /* Halting discards the read in flight. */
            timer_del(s->stream_timer);
        }
        rp2350_xip_stream_schedule(s);
        break;
    case A_STAT:
    case A_STREAM_FIFO:
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-xip: write to bad XIP_CTRL "
                      "offset 0x%" HWADDR_PRIx "\n", addr);
        break;
    }
    return MEMTX_OK;
}

static const MemoryRegionOps rp2350_xip_ctrl_ops = {
    .read_with_attrs = rp2350_xip_ctrl_read,
    .write_with_attrs = rp2350_xip_ctrl_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/* QMI registers. */

static uint32_t *qmi_mreg(RP2350XIPState *s, hwaddr reg, uint32_t *mask)
{
    int cs = (reg - A_M0_TIMING) / M_STRIDE;

    switch ((reg - A_M0_TIMING) % M_STRIDE) {
    case 0x0:
        *mask = TIMING_MASK;
        return &s->timing[cs];
    case 0x4:
        *mask = FMT_MASK;
        return &s->rfmt[cs];
    case 0x8:
        *mask = CMD_MASK;
        return &s->rcmd[cs];
    case 0xc:
        *mask = FMT_MASK;
        return &s->wfmt[cs];
    default:
        *mask = CMD_MASK;
        return &s->wcmd[cs];
    }
}

/* [spec:nuos:req:emu.xip+1] */
static uint64_t rp2350_xip_qmi_read(void *opaque, hwaddr addr, unsigned size)
{
    RP2350XIPState *s = opaque;
    hwaddr reg = rp2350_atomic_reg(addr);
    uint32_t mask;

    switch (reg) {
    case A_DIRECT_CSR:
        return rp2350_xip_direct_csr(s);
    case A_DIRECT_TX:
        return 0;
    case A_DIRECT_RX:
        return rp2350_xip_direct_pop(s);
    case A_M0_TIMING ... A_M1_WCMD:
        return *qmi_mreg(s, reg, &mask);
    case A_ATRANS0 ... A_ATRANS7:
        return s->atrans[(reg - A_ATRANS0) / 4];
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-xip: read of bad QMI offset "
                      "0x%" HWADDR_PRIx "\n", addr);
        return 0;
    }
}

/* [spec:nuos:req:emu.xip+1] */
static void rp2350_xip_qmi_write(void *opaque, hwaddr addr, uint64_t value,
                                 unsigned size)
{
    RP2350XIPState *s = opaque;
    hwaddr reg = rp2350_atomic_reg(addr);
    uint32_t *r, mask, v, old;

    switch (reg) {
    case A_DIRECT_CSR:
        old = s->direct_csr;
        s->direct_csr = rp2350_atomic_apply(addr, old, value) & CSR_WRITABLE;
        if ((old ^ s->direct_csr) & CSR_EN) {
            /* Direct mode takes the QSPI bus from the XIP windows. */
            rp2350_xip_remap(s);
            rp2350_xip_stream_schedule(s);
        }
        rp2350_xip_direct_run(s);
        break;
    case A_DIRECT_TX:
        rp2350_xip_direct_push(s, rp2350_atomic_apply(addr, 0, value));
        break;
    case A_DIRECT_RX:
        break;
    case A_M0_TIMING ... A_M1_WCMD:
        r = qmi_mreg(s, reg, &mask);
        *r = rp2350_atomic_apply(addr, *r, value) & mask;
        break;
    case A_ATRANS0 ... A_ATRANS7:
        r = &s->atrans[(reg - A_ATRANS0) / 4];
        v = rp2350_atomic_apply(addr, *r, value) & ATRANS_MASK;
        if (v != *r) {
            *r = v;
            rp2350_xip_remap(s);
        }
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-xip: write to bad QMI offset "
                      "0x%" HWADDR_PRIx "\n", addr);
        break;
    }
}

static const MemoryRegionOps rp2350_xip_qmi_ops = {
    .read = rp2350_xip_qmi_read,
    .write = rp2350_xip_qmi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/* XIP_AUX: the FIFOs on a fast AHB port for DMA, with no atomic aliases. */

/* [spec:nuos:req:emu.xip+1] */
static uint64_t rp2350_xip_aux_read(void *opaque, hwaddr addr, unsigned size)
{
    RP2350XIPState *s = opaque;

    switch (addr) {
    case A_AUX_STREAM:
        return rp2350_xip_stream_pop(s);
    case A_AUX_QMI_DIRECT_RX:
        return rp2350_xip_direct_pop(s);
    case A_AUX_QMI_DIRECT_TX:
        return 0;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-xip: read of bad XIP_AUX "
                      "offset 0x%" HWADDR_PRIx "\n", addr);
        return 0;
    }
}

static void rp2350_xip_aux_write(void *opaque, hwaddr addr, uint64_t value,
                                 unsigned size)
{
    RP2350XIPState *s = opaque;

    switch (addr) {
    case A_AUX_QMI_DIRECT_TX:
        rp2350_xip_direct_push(s, value);
        break;
    case A_AUX_STREAM:
    case A_AUX_QMI_DIRECT_RX:
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-xip: write to bad XIP_AUX "
                      "offset 0x%" HWADDR_PRIx "\n", addr);
        break;
    }
}

static const MemoryRegionOps rp2350_xip_aux_ops = {
    .read = rp2350_xip_aux_read,
    .write = rp2350_xip_aux_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

MemoryRegion *rp2350_xip_array(RP2350XIPState *s, int cs)
{
    assert(cs < RP2350_QMI_CS && s->cs_size[cs]);
    return &s->array[cs].mr;
}

static void rp2350_xip_reset_regs(RP2350XIPState *s)
{
    int i;

    s->ctrl = CTRL_RESET;
    s->ctr_hit = 0;
    s->ctr_acc = 0;
    s->stream_addr = 0;
    s->stream_ctr = 0;
    fifo32_reset(&s->stream_fifo);
    s->direct_csr = CSR_RESET;
    fifo32_reset(&s->direct_tx);
    fifo32_reset(&s->direct_rx);
    s->direct_busy_until = 0;
    for (i = 0; i < RP2350_QMI_CS; i++) {
        s->timing[i] = TIMING_RESET;
        s->rfmt[i] = FMT_RESET;
        s->rcmd[i] = RCMD_RESET;
        s->wfmt[i] = FMT_RESET;
        s->wcmd[i] = WCMD_RESET;
    }
    /* An identity mapping: pane p of each window at p * 4 MiB. */
    for (i = 0; i < RP2350_QMI_ATRANS; i++) {
        s->atrans[i] = (ATRANS_PANE_PAGES << ATRANS_SIZE_SHIFT) |
                       (i % RP2350_QMI_PANES) * ATRANS_PANE_PAGES;
    }
    s->mm_cs = -1;
}

/*
 * The cache's tag and data memories are in the XIP memory power domain
 * and keep their contents across resets, so pinned lines stay pinned.
 * The window mapping is rebuilt in the enter phase so that the reset
 * mapping is in place before images are loaded into flash.
 */
static void rp2350_xip_reset_enter(Object *obj, ResetType type)
{
    RP2350XIPState *s = RP2350_XIP(obj);

    timer_del(s->busy_timer);
    timer_del(s->stream_timer);
    rp2350_xip_reset_regs(s);
    rp2350_xip_remap(s);
}

static void rp2350_xip_reset_exit(Object *obj, ResetType type)
{
    RP2350XIPState *s = RP2350_XIP(obj);
    int i;

    for (i = 0; i < RP2350_QMI_CS; i++) {
        s->cs_level[i] = -1;
    }
    rp2350_xip_update_cs(s);
    rp2350_xip_update_dreq(s);
}

void rp2350_xip_reset_block(RP2350XIPState *s)
{
    rp2350_xip_reset_enter(OBJECT(s), RESET_TYPE_COLD);
    rp2350_xip_reset_exit(OBJECT(s), RESET_TYPE_COLD);
}

/*
 * The XIP memory power domain lost power: the cache's tag and data
 * memories lose their contents, so no line stays pinned.
 */
/* [spec:nuos:req:emu.powman] */
void rp2350_xip_power_down(RP2350XIPState *s)
{
    memset(s->tag, 0, sizeof(s->tag));
    address_space_set(&s->cache_as, 0, 0, RP2350_XIP_CACHE_SIZE,
                      MEMTXATTRS_UNSPECIFIED);
    rp2350_xip_remap(s);
}

static void rp2350_xip_init(Object *obj)
{
    RP2350XIPState *s = RP2350_XIP(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->ctrl_iomem, obj, &rp2350_xip_ctrl_ops, s,
                          "rp2350-xip-ctrl", RP2350_ATOMIC_REGION_SIZE);
    memory_region_init_io(&s->qmi_iomem, obj, &rp2350_xip_qmi_ops, s,
                          "rp2350-xip-qmi", RP2350_ATOMIC_REGION_SIZE);
    memory_region_init_io(&s->aux_iomem, obj, &rp2350_xip_aux_ops, s,
                          "rp2350-xip-aux", RP2350_XIP_AUX_SIZE);
    memory_region_init(&s->space, obj, "rp2350-xip", RP2350_XIP_SPACE_SIZE);
    sysbus_init_mmio(sbd, &s->ctrl_iomem);
    sysbus_init_mmio(sbd, &s->qmi_iomem);
    sysbus_init_mmio(sbd, &s->aux_iomem);
    sysbus_init_mmio(sbd, &s->space);

    s->qspi = ssi_create_bus(DEVICE(obj), "qspi");
    qdev_init_gpio_out_named(DEVICE(obj), s->cs, "cs", RP2350_QMI_CS);
    qdev_init_gpio_out_named(DEVICE(obj), s->dreq, "dreq",
                             RP2350_XIP_NUM_DREQ);
}

static void rp2350_xip_realize(DeviceState *dev, Error **errp)
{
    RP2350XIPState *s = RP2350_XIP(dev);
    Object *obj = OBJECT(dev);
    int cs, i;

    if (!s->sysclk_hz) {
        error_setg(errp, "sysclk-hz must be set");
        return;
    }
    for (cs = 0; cs < RP2350_QMI_CS; cs++) {
        uint32_t size = s->cs_size[cs];

        if (size && (!is_power_of_2(size) || size < MiB ||
                     size > RP2350_QMI_WINDOW_SIZE)) {
            error_setg(errp, "cs%d-size must be a power of two from 1 MiB "
                       "to 16 MiB", cs);
            return;
        }
    }

    fifo32_create(&s->stream_fifo, STREAM_FIFO_DEPTH);
    fifo32_create(&s->direct_tx, DIRECT_FIFO_DEPTH);
    fifo32_create(&s->direct_rx, DIRECT_FIFO_DEPTH);
    s->busy_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, rp2350_xip_busy_done, s);
    s->stream_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, rp2350_xip_stream_tick,
                                   s);

    memory_region_init_io(&s->space_io, obj, &rp2350_xip_space_ops, s,
                          "rp2350-xip-space", RP2350_XIP_SPACE_SIZE);
    memory_region_add_subregion(&s->space, 0, &s->space_io);

    if (!memory_region_init_ram(&s->cache_data, obj, "rp2350.xip-cache",
                                RP2350_XIP_CACHE_SIZE, errp)) {
        return;
    }
    address_space_init(&s->cache_as, &s->cache_data, "rp2350-xip-cache");

    for (cs = 0; cs < RP2350_QMI_CS; cs++) {
        uint32_t size = s->cs_size[cs];
        g_autofree char *name = g_strdup_printf("rp2350.qspi-cs%d", cs);
        g_autofree char *mname = g_strdup_printf("rp2350-xip.cs%d", cs);

        if (!size) {
            continue;
        }
        s->array[cs].xip = s;
        s->array[cs].cs = cs;
        if (!memory_region_init_rom_device(&s->array[cs].mr, obj,
                                           &rp2350_xip_array_ops,
                                           &s->array[cs], name, size, errp)) {
            return;
        }
        memory_region_init(&s->mirror[cs], obj, mname, MIRROR_SPAN);
        for (i = 0; i < MIRROR_SPAN / size; i++) {
            memory_region_init_alias(&s->mirror_alias[cs][i], obj, mname,
                                     &s->array[cs].mr, 0, size);
            memory_region_add_subregion(&s->mirror[cs], i * size,
                                        &s->mirror_alias[cs][i]);
        }
        for (i = 0; i < RP2350_XIP_VIEWS; i++) {
            memory_region_init_alias(&s->view[cs][i], obj, mname,
                                     &s->mirror[cs], 0, PANE_SIZE);
            memory_region_set_enabled(&s->view[cs][i], false);
            memory_region_add_subregion_overlap(&s->space, 0,
                                                &s->view[cs][i], 1);
        }
    }
    for (i = 0; i < RP2350_XIP_PIN_VIEWS; i++) {
        memory_region_init_alias(&s->pin_ram[i], obj, "rp2350-xip.pinned",
                                 &s->cache_data, 0, CACHE_LINE);
        memory_region_set_enabled(&s->pin_ram[i], false);
        memory_region_add_subregion_overlap(&s->space, 0, &s->pin_ram[i], 2);
        memory_region_init_alias(&s->pin_io[i], obj, "rp2350-xip.pinned",
                                 &s->space_io, 0, CACHE_LINE);
        memory_region_set_enabled(&s->pin_io[i], false);
        memory_region_add_subregion_overlap(&s->space, 0, &s->pin_io[i], 2);
    }

    rp2350_xip_reset_regs(s);
    rp2350_xip_remap(s);
}

static int rp2350_xip_post_load(void *opaque, int version_id)
{
    rp2350_xip_remap(opaque);
    return 0;
}

static const VMStateDescription vmstate_rp2350_xip = {
    .name = TYPE_RP2350_XIP,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = rp2350_xip_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(ctrl, RP2350XIPState),
        VMSTATE_UINT32(ctr_hit, RP2350XIPState),
        VMSTATE_UINT32(ctr_acc, RP2350XIPState),
        VMSTATE_UINT32(stream_addr, RP2350XIPState),
        VMSTATE_UINT32(stream_ctr, RP2350XIPState),
        VMSTATE_FIFO32(stream_fifo, RP2350XIPState),
        VMSTATE_UINT32(direct_csr, RP2350XIPState),
        VMSTATE_FIFO32(direct_tx, RP2350XIPState),
        VMSTATE_FIFO32(direct_rx, RP2350XIPState),
        VMSTATE_INT64(direct_busy_until, RP2350XIPState),
        VMSTATE_UINT32_ARRAY(timing, RP2350XIPState, RP2350_QMI_CS),
        VMSTATE_UINT32_ARRAY(rfmt, RP2350XIPState, RP2350_QMI_CS),
        VMSTATE_UINT32_ARRAY(rcmd, RP2350XIPState, RP2350_QMI_CS),
        VMSTATE_UINT32_ARRAY(wfmt, RP2350XIPState, RP2350_QMI_CS),
        VMSTATE_UINT32_ARRAY(wcmd, RP2350XIPState, RP2350_QMI_CS),
        VMSTATE_UINT32_ARRAY(atrans, RP2350XIPState, RP2350_QMI_ATRANS),
        VMSTATE_INT32(mm_cs, RP2350XIPState),
        VMSTATE_INT32_ARRAY(cs_level, RP2350XIPState, RP2350_QMI_CS),
        VMSTATE_UINT32_ARRAY(tag, RP2350XIPState, RP2350_XIP_CACHE_LINES),
        VMSTATE_TIMER_PTR(busy_timer, RP2350XIPState),
        VMSTATE_TIMER_PTR(stream_timer, RP2350XIPState),
        VMSTATE_END_OF_LIST()
    },
};

static const Property rp2350_xip_properties[] = {
    DEFINE_PROP_UINT32("cs0-size", RP2350XIPState, cs_size[0], 0),
    DEFINE_PROP_UINT32("cs1-size", RP2350XIPState, cs_size[1], 0),
    DEFINE_PROP_UINT32("sysclk-hz", RP2350XIPState, sysclk_hz, 0),
};

/* [spec:nuos:req:emu.xip+1] */
static void rp2350_xip_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = rp2350_xip_realize;
    dc->vmsd = &vmstate_rp2350_xip;
    rc->phases.enter = rp2350_xip_reset_enter;
    rc->phases.exit = rp2350_xip_reset_exit;
    device_class_set_props(dc, rp2350_xip_properties);
}

static const TypeInfo rp2350_xip_info = {
    .name          = TYPE_RP2350_XIP,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RP2350XIPState),
    .instance_init = rp2350_xip_init,
    .class_init    = rp2350_xip_class_init,
};

static void rp2350_xip_register_types(void)
{
    type_register_static(&rp2350_xip_info);
}
type_init(rp2350_xip_register_types)
