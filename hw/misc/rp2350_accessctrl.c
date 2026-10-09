/*
 * RP2350 bus access control (ACCESSCTRL)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: RP2350 Datasheet, "Access control" and "Bus security
 * filtering"; register layout and reset values from the pico-sdk
 * accessctrl.h.
 *
 * Each master's view of the bus is a container holding all of board
 * memory, with a gate over every block that has a permission register.
 * A gate checks the access against the register and either fails it with
 * a bus error or forwards it unchanged to board memory. Gates are only
 * enabled while their register restricts the view's master, so that
 * fully open memories (the default for ROM, XIP and SRAM) stay plain RAM
 * and ROM to TCG.
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qemu/host-utils.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/arm/arm-security.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/rp2350_accessctrl.h"
#include "hw/misc/rp2350_atomic.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"

#define A_LOCK          0x000
#define A_FORCE_CORE_NS 0x004
#define A_CFGRESET      0x008
#define A_GPIO_NSMASK0  0x00c
#define A_GPIO_NSMASK1  0x010
#define A_ROM           0x014
#define A_XIP_MAIN      0x018
#define A_SRAM0         0x01c
#define A_SRAM4         0x02c
#define A_SRAM8         0x03c
#define A_SRAM9         0x040
#define A_DMA           0x044
#define A_USBCTRL       0x048
#define A_PIO0          0x04c
#define A_PIO1          0x050
#define A_PIO2          0x054
#define A_CORESIGHT_TRACE  0x058
#define A_CORESIGHT_PERIPH 0x05c
#define A_SYSINFO       0x060
#define A_RESETS        0x064
#define A_IO_BANK0      0x068
#define A_IO_BANK1      0x06c
#define A_PADS_BANK0    0x070
#define A_PADS_QSPI     0x074
#define A_BUSCTRL       0x078
#define A_ADC           0x07c
#define A_HSTX          0x080
#define A_I2C0          0x084
#define A_I2C1          0x088
#define A_PWM           0x08c
#define A_SPI0          0x090
#define A_SPI1          0x094
#define A_TIMER0        0x098
#define A_TIMER1        0x09c
#define A_UART0         0x0a0
#define A_UART1         0x0a4
#define A_OTP           0x0a8
#define A_TBMAN         0x0ac
#define A_POWMAN        0x0b0
#define A_TRNG          0x0b4
#define A_SHA256        0x0b8
#define A_SYSCFG        0x0bc
#define A_CLOCKS        0x0c0
#define A_XOSC          0x0c4
#define A_ROSC          0x0c8
#define A_PLL_SYS       0x0cc
#define A_PLL_USB       0x0d0
#define A_TICKS         0x0d4
#define A_WATCHDOG      0x0d8
#define A_PSM           0x0dc
#define A_XIP_CTRL      0x0e0
#define A_XIP_QMI       0x0e4
#define A_XIP_AUX       0x0e8

#define FIRST_BUS_REG   A_ROM
#define LAST_REG        A_XIP_AUX

/* Bus access permission register fields. */
#define PERM_NSU        (1u << 0)
#define PERM_NSP        (1u << 1)
#define PERM_SU         (1u << 2)
#define PERM_SP         (1u << 3)
#define PERM_SEC_ALL    0xfu
#define PERM_MASTER(m)  (1u << (4 + (m)))
#define PERM_MASK       0xffu

#define LOCK_MASTER(m)  (1u << (m))
#define LOCK_DMA        LOCK_MASTER(RP2350_MASTER_DMA)
#define LOCK_WMASK      (LOCK_MASTER(RP2350_MASTER_CORE0) | \
                         LOCK_MASTER(RP2350_MASTER_CORE1) | \
                         LOCK_MASTER(RP2350_MASTER_DEBUG))
#define FORCE_NS_CORE1  (1u << 1)
#define CFGRESET_BIT    (1u << 0)
#define NSMASK1_WMASK   0xff00ffffu

#define PASSWORD        0xacce0000u
#define PASSWORD_MASK   0xffff0000u

#define NO_REG          (-1)

/*
 * The blocks behind the bus filters, and the register that governs each.
 * A block absent here is not filtered: SIO and the PPB are core-local, the
 * ACCESSCTRL registers filter their own writes, and BOOTRAM is hardwired
 * Secure-only. BOOTRAM is listed with NO_REG so that core 1's accesses to
 * it are still made Non-secure under FORCE_CORE_NS.
 *
 * A striped range spans four word-interleaved SRAM banks: address bits
 * 3:2 select the bank, governed by consecutive registers from `reg`.
 */
struct RP2350AccessCtrlRange {
    const char *name;
    hwaddr base;
    hwaddr size;
    int reg;
    bool striped;
};

/* [spec:nuos:req:emu.accessctrl] */
static const RP2350AccessCtrlRange rp2350_accessctrl_ranges[] = {
    { "rom",              0x00000000, 0x8000,     A_ROM },
    { "xip",              0x10000000, 0x10000000, A_XIP_MAIN },
    { "sram0-3",          0x20000000, 0x40000,    A_SRAM0, true },
    { "sram4-7",          0x20040000, 0x40000,    A_SRAM4, true },
    { "sram8",            0x20080000, 0x1000,     A_SRAM8 },
    { "sram9",            0x20081000, 0x1000,     A_SRAM9 },
    { "sysinfo",          0x40000000, 0x8000,     A_SYSINFO },
    { "syscfg",           0x40008000, 0x8000,     A_SYSCFG },
    { "clocks",           0x40010000, 0x8000,     A_CLOCKS },
    { "psm",              0x40018000, 0x8000,     A_PSM },
    { "resets",           0x40020000, 0x8000,     A_RESETS },
    { "io_bank0",         0x40028000, 0x8000,     A_IO_BANK0 },
    { "io_qspi",          0x40030000, 0x8000,     A_IO_BANK1 },
    { "pads_bank0",       0x40038000, 0x8000,     A_PADS_BANK0 },
    { "pads_qspi",        0x40040000, 0x8000,     A_PADS_QSPI },
    { "xosc",             0x40048000, 0x8000,     A_XOSC },
    { "pll_sys",          0x40050000, 0x8000,     A_PLL_SYS },
    { "pll_usb",          0x40058000, 0x8000,     A_PLL_USB },
    { "busctrl",          0x40068000, 0x8000,     A_BUSCTRL },
    { "uart0",            0x40070000, 0x8000,     A_UART0 },
    { "uart1",            0x40078000, 0x8000,     A_UART1 },
    { "spi0",             0x40080000, 0x8000,     A_SPI0 },
    { "spi1",             0x40088000, 0x8000,     A_SPI1 },
    { "i2c0",             0x40090000, 0x8000,     A_I2C0 },
    { "i2c1",             0x40098000, 0x8000,     A_I2C1 },
    { "adc",              0x400a0000, 0x8000,     A_ADC },
    { "pwm",              0x400a8000, 0x8000,     A_PWM },
    { "timer0",           0x400b0000, 0x8000,     A_TIMER0 },
    { "timer1",           0x400b8000, 0x8000,     A_TIMER1 },
    { "hstx_ctrl",        0x400c0000, 0x8000,     A_HSTX },
    { "xip_ctrl",         0x400c8000, 0x8000,     A_XIP_CTRL },
    { "xip_qmi",          0x400d0000, 0x8000,     A_XIP_QMI },
    { "watchdog",         0x400d8000, 0x8000,     A_WATCHDOG },
    { "bootram",          0x400e0000, 0x8000,     NO_REG },
    { "rosc",             0x400e8000, 0x8000,     A_ROSC },
    { "trng",             0x400f0000, 0x8000,     A_TRNG },
    { "sha256",           0x400f8000, 0x8000,     A_SHA256 },
    { "powman",           0x40100000, 0x8000,     A_POWMAN },
    { "ticks",            0x40108000, 0x8000,     A_TICKS },
    { "otp",              0x40120000, 0x20000,    A_OTP },
    { "coresight_periph", 0x40140000, 0x10000,    A_CORESIGHT_PERIPH },
    { "tbman",            0x40160000, 0x8000,     A_TBMAN },
    { "dma",              0x50000000, 0x100000,   A_DMA },
    { "usbctrl",          0x50100000, 0x100000,   A_USBCTRL },
    { "pio0",             0x50200000, 0x100000,   A_PIO0 },
    { "pio1",             0x50300000, 0x100000,   A_PIO1 },
    { "pio2",             0x50400000, 0x100000,   A_PIO2 },
    { "xip_aux",          0x50500000, 0x100000,   A_XIP_AUX },
    { "hstx_fifo",        0x50600000, 0x100000,   A_HSTX },
    { "coresight_trace",  0x50700000, 0x100000,   A_CORESIGHT_TRACE },
};

#define NUM_RANGES ARRAY_SIZE(rp2350_accessctrl_ranges)

/*
 * Reset values (pico-sdk accessctrl.h). Memories and SYSINFO are fully
 * open; most peripherals are Secure-only from any master (0xfc); the clock,
 * power, reset and XIP control blocks are Secure, privileged only without
 * DMA (0xb8); SHA-256 and XIP_AUX are Secure, privileged only (0xf8).
 */
static uint32_t reset_value(hwaddr reg)
{
    switch (reg) {
    case A_LOCK:
        return LOCK_DMA;
    case A_ROM ... A_SRAM9:
    case A_SYSINFO:
        return 0xff;
    case A_CORESIGHT_TRACE:
    case A_CORESIGHT_PERIPH:
    case A_POWMAN:
    case A_TRNG:
    case A_SYSCFG ... A_XIP_QMI:
        return 0xb8;
    case A_SHA256:
    case A_XIP_AUX:
        return 0xf8;
    case A_DMA ... A_PIO2:
    case A_RESETS ... A_TBMAN:
        return 0xfc;
    default:
        return 0;
    }
}

static uint32_t reg_get(RP2350AccessCtrlState *s, hwaddr reg)
{
    return s->regs[reg / 4];
}

static bool core1_forced_ns(RP2350AccessCtrlState *s)
{
    return reg_get(s, A_FORCE_CORE_NS) & FORCE_NS_CORE1;
}

/*
 * Whether `master` may make an access with these attributes to the block
 * governed by `reg`. SU and NSU only grant access together with SP and
 * NSP respectively.
 */
static bool permitted(RP2350AccessCtrlState *s, hwaddr reg,
                      RP2350BusMaster master, bool secure, bool user)
{
    uint32_t v = reg_get(s, reg);
    uint32_t need;

    if (!(v & PERM_MASTER(master))) {
        return false;
    }
    if (secure) {
        need = user ? PERM_SP | PERM_SU : PERM_SP;
    } else {
        need = user ? PERM_NSP | PERM_NSU : PERM_NSP;
    }
    return (v & need) == need;
}

/*
 * The master an access through `master`'s view comes from, and its
 * security and privilege as the bus sees them.
 */
static RP2350BusMaster access_master(RP2350AccessCtrlState *s,
                                     RP2350BusMaster master,
                                     MemTxAttrs *attrs)
{
    if (attrs->unspecified) {
        attrs->secure = 1;
        attrs->space = ARMSS_Secure;
        attrs->user = 0;
        return RP2350_MASTER_DEBUG;
    }
    if (attrs->debug) {
        return RP2350_MASTER_DEBUG;
    }
    /* FORCE_CORE_NS makes core 1's accesses Non-secure at system level. */
    if (master == RP2350_MASTER_CORE1 && core1_forced_ns(s)) {
        attrs->secure = 0;
        attrs->space = ARMSS_NonSecure;
    }
    return master;
}

static const char *master_name(RP2350BusMaster m)
{
    static const char *const names[] = {
        [RP2350_MASTER_CORE0] = "core 0",
        [RP2350_MASTER_CORE1] = "core 1",
        [RP2350_MASTER_DMA] = "DMA",
        [RP2350_MASTER_DEBUG] = "debugger",
    };

    return names[m];
}

/* Whether a gate is needed between `master`'s view and range `r`. */
static bool gate_needed(RP2350AccessCtrlState *s, RP2350BusMaster master,
                        const RP2350AccessCtrlRange *r)
{
    uint32_t mask = PERM_SEC_ALL | PERM_MASTER(master);
    int banks = r->striped ? 4 : 1;
    int i;

    if (master == RP2350_MASTER_CORE1 && core1_forced_ns(s)) {
        return true;
    }
    if (r->reg == NO_REG) {
        return false;
    }
    /* Debug accesses arrive through the cores' views. */
    if (master != RP2350_MASTER_DMA) {
        mask |= PERM_MASTER(RP2350_MASTER_DEBUG);
    }
    for (i = 0; i < banks; i++) {
        if ((reg_get(s, r->reg + 4 * i) & mask) != mask) {
            return true;
        }
    }
    return false;
}

static void update_gates(RP2350AccessCtrlState *s)
{
    int m, i;

    if (!s->gate[0]) {
        return;
    }
    memory_region_transaction_begin();
    for (m = 0; m < RP2350_ACCESSCTRL_NUM_VIEWS; m++) {
        for (i = 0; i < NUM_RANGES; i++) {
            RP2350AccessCtrlGate *g = &s->gate[m][i];

            memory_region_set_enabled(&g->iomem, gate_needed(s, m, g->range));
        }
    }
    memory_region_transaction_commit();
}

static void config_changed(RP2350AccessCtrlState *s)
{
    update_gates(s);
    notifier_list_notify(&s->config_notifiers, s);
}

/*
 * The filter: a denied access returns a bus error and never reaches the
 * block. A striped SRAM access is checked against the register of every
 * bank it touches.
 */
/* [spec:nuos:req:emu.accessctrl] */
static MemTxResult gate_access(RP2350AccessCtrlGate *g, hwaddr addr,
                               uint64_t *data, unsigned size,
                               MemTxAttrs attrs, bool is_write)
{
    RP2350AccessCtrlState *s = g->s;
    const RP2350AccessCtrlRange *r = g->range;
    RP2350BusMaster master = access_master(s, g->master, &attrs);
    uint8_t buf[8];
    MemTxResult res;
    hwaddr a;

    if (r->reg != NO_REG) {
        for (a = addr & ~3ull; a < addr + size; a += 4) {
            hwaddr reg = r->striped ? r->reg + 4 * ((a >> 2) & 3) : r->reg;

            if (!permitted(s, reg, master, attrs.secure, attrs.user)) {
                qemu_log_mask(LOG_GUEST_ERROR,
                              "rp2350-accessctrl: %s %s%s %s of %s at "
                              "0x%" HWADDR_PRIx " denied\n",
                              master_name(master),
                              attrs.secure ? "Secure" : "Non-secure",
                              attrs.user ? " unprivileged" : "",
                              is_write ? "write" : "read", r->name,
                              r->base + addr);
                return MEMTX_ERROR;
            }
        }
    }

    if (is_write) {
        stn_le_p(buf, size, *data);
    }
    res = address_space_rw(&s->bus_as, r->base + addr, attrs, buf, size,
                           is_write);
    if (!is_write) {
        *data = ldn_le_p(buf, size);
    }
    return res;
}

static MemTxResult gate_read(void *opaque, hwaddr addr, uint64_t *data,
                             unsigned size, MemTxAttrs attrs)
{
    return gate_access(opaque, addr, data, size, attrs, false);
}

static MemTxResult gate_write(void *opaque, hwaddr addr, uint64_t value,
                              unsigned size, MemTxAttrs attrs)
{
    return gate_access(opaque, addr, &value, size, attrs, true);
}

/* Gates front memories as well as registers, so take any access. */
static const MemoryRegionOps gate_ops = {
    .read_with_attrs = gate_read,
    .write_with_attrs = gate_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 8,
    .valid.unaligned = true,
    .impl.min_access_size = 1,
    .impl.max_access_size = 8,
    .impl.unaligned = true,
};

static void cfgreset(RP2350AccessCtrlState *s)
{
    hwaddr reg;

    for (reg = A_GPIO_NSMASK0; reg <= LAST_REG; reg += 4) {
        s->regs[reg / 4] = reset_value(reg);
    }
}

/* [spec:nuos:req:emu.accessctrl] */
static MemTxResult rp2350_accessctrl_read(void *opaque, hwaddr addr,
                                          uint64_t *data, unsigned size,
                                          MemTxAttrs attrs)
{
    RP2350AccessCtrlPort *p = opaque;
    hwaddr reg = rp2350_atomic_reg(addr & ~3ull);
    uint32_t v = 0;

    /*
     * Every register is readable by any master in any state. A narrow
     * read returns its byte lanes of the register.
     */
    if (reg <= LAST_REG && reg != A_CFGRESET) {
        v = reg_get(p->s, reg);
    }
    *data = extract32(v, (addr & 3) * 8, size * 8);
    return MEMTX_OK;
}

/*
 * Unprivileged and DMA writes fail with a bus error, as do writes without
 * the password (all but GPIO_NSMASK0/1). Otherwise a write from a master
 * whose LOCK bit is set is ignored, and a Non-secure privileged write
 * changes only the NSU bit of a bus access register whose NSP bit is set.
 */
/* [spec:nuos:req:emu.accessctrl] */
static MemTxResult rp2350_accessctrl_write(void *opaque, hwaddr addr,
                                           uint64_t value, unsigned size,
                                           MemTxAttrs attrs)
{
    RP2350AccessCtrlPort *p = opaque;
    RP2350AccessCtrlState *s = p->s;
    RP2350BusMaster master = access_master(s, p->master, &attrs);
    hwaddr reg = rp2350_atomic_reg(addr & ~3ull);
    bool password = reg != A_GPIO_NSMASK0 && reg != A_GPIO_NSMASK1;
    uint32_t old, new;

    /*
     * A narrow write is replicated across the 32-bit bus and writes the
     * whole register (datasheet "Narrow IO register writes"): a byte
     * write never carries the password, and a halfword write only when
     * its value is the password.
     */
    if (size == 1) {
        value = (value & 0xff) * 0x01010101u;
    } else if (size == 2) {
        value = (value & 0xffff) * 0x00010001u;
    }
    addr &= ~3ull;

    if (attrs.user || master == RP2350_MASTER_DMA) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "rp2350-accessctrl: %s %s write to 0x%" HWADDR_PRIx
                      "\n", master_name(master),
                      attrs.user ? "unprivileged" : "", addr);
        return MEMTX_ERROR;
    }
    if (password && (value & PASSWORD_MASK) != PASSWORD) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "rp2350-accessctrl: write to 0x%" HWADDR_PRIx
                      " without the 0xacce password\n", addr);
        return MEMTX_ERROR;
    }
    if (reg > LAST_REG) {
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-accessctrl: write to "
                      "reserved offset 0x%" HWADDR_PRIx "\n", addr);
        return MEMTX_OK;
    }
    if (reg_get(s, A_LOCK) & LOCK_MASTER(master)) {
        return MEMTX_OK;
    }
    if (password) {
        value &= ~PASSWORD_MASK;
    }

    old = reg == A_CFGRESET ? 0 : reg_get(s, reg);
    new = rp2350_atomic_apply(addr, old, value);

    if (!attrs.secure && master != RP2350_MASTER_DEBUG) {
        /* Non-secure may only grant or revoke NSU, and only under NSP. */
        if (reg < FIRST_BUS_REG || !(old & PERM_NSP)) {
            return MEMTX_OK;
        }
        new = (old & ~PERM_NSU) | (new & PERM_NSU);
    }

    switch (reg) {
    case A_LOCK:
        /* LOCK bits are set-only, until ACCESSCTRL itself is reset. */
        new = old | (new & LOCK_WMASK);
        break;
    case A_FORCE_CORE_NS:
        new &= FORCE_NS_CORE1;
        break;
    case A_CFGRESET:
        if (new & CFGRESET_BIT) {
            cfgreset(s);
            config_changed(s);
        }
        return MEMTX_OK;
    case A_GPIO_NSMASK0:
        break;
    case A_GPIO_NSMASK1:
        new &= NSMASK1_WMASK;
        break;
    default:
        new &= PERM_MASK;
        break;
    }
    if (new != old) {
        s->regs[reg / 4] = new;
        config_changed(s);
    }
    return MEMTX_OK;
}

static const MemoryRegionOps rp2350_accessctrl_ops = {
    .read_with_attrs = rp2350_accessctrl_read,
    .write_with_attrs = rp2350_accessctrl_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

MemoryRegion *rp2350_accessctrl_view(RP2350AccessCtrlState *s,
                                     RP2350BusMaster master)
{
    assert(master < RP2350_ACCESSCTRL_NUM_VIEWS);
    return &s->view[master];
}

MemTxAttrs rp2350_accessctrl_dma_attrs(unsigned seclevel)
{
    MemTxAttrs attrs = {
        .secure = seclevel >= RP2350_DMA_SECLEVEL_SU,
        .space = seclevel >= RP2350_DMA_SECLEVEL_SU ? ARMSS_Secure
                                                    : ARMSS_NonSecure,
        .user = !(seclevel & 1),
    };

    return attrs;
}

uint32_t rp2350_accessctrl_gpio_nsmask(RP2350AccessCtrlState *s, int bank)
{
    return reg_get(s, bank ? A_GPIO_NSMASK1 : A_GPIO_NSMASK0);
}

bool rp2350_accessctrl_ns_accessible(RP2350AccessCtrlState *s, hwaddr addr)
{
    int i;

    for (i = 0; i < NUM_RANGES; i++) {
        const RP2350AccessCtrlRange *r = &rp2350_accessctrl_ranges[i];

        if (addr >= r->base && addr - r->base < r->size) {
            return r->reg != NO_REG && (reg_get(s, r->reg) & PERM_NSP);
        }
    }
    return false;
}

/*
 * A DREQ's security level is the lowest effective permission bit of its
 * block's register, where SU counts only with SP and NSU only with NSP.
 */
/* [spec:nuos:req:emu.dma] */
unsigned rp2350_accessctrl_dreq_level(RP2350AccessCtrlState *s, hwaddr addr)
{
    int i;

    for (i = 0; i < NUM_RANGES; i++) {
        const RP2350AccessCtrlRange *r = &rp2350_accessctrl_ranges[i];
        uint32_t v;

        if (addr < r->base || addr - r->base >= r->size) {
            continue;
        }
        if (r->reg == NO_REG) {
            break;
        }
        v = reg_get(s, r->reg) & PERM_SEC_ALL;
        if (!(v & PERM_SP)) {
            v &= ~PERM_SU;
        }
        if (!(v & PERM_NSP)) {
            v &= ~PERM_NSU;
        }
        return v ? ctz32(v) : 4;
    }
    return RP2350_DMA_SECLEVEL_SP;
}

void rp2350_accessctrl_add_notifier(RP2350AccessCtrlState *s, Notifier *n)
{
    notifier_list_add(&s->config_notifiers, n);
}

static void reset_regs(RP2350AccessCtrlState *s)
{
    hwaddr reg;

    for (reg = 0; reg <= LAST_REG; reg += 4) {
        s->regs[reg / 4] = reset_value(reg);
    }
}

/*
 * The registers reset in the enter phase, so that the cores, which fetch
 * their reset vectors in their hold phase, are already filtered by the
 * reset permissions.
 */
static void rp2350_accessctrl_enter_reset(Object *obj, ResetType type)
{
    reset_regs(RP2350_ACCESSCTRL(obj));
}

static void rp2350_accessctrl_hold_reset(Object *obj, ResetType type)
{
    config_changed(RP2350_ACCESSCTRL(obj));
}

static void rp2350_accessctrl_init(Object *obj)
{
    RP2350AccessCtrlState *s = RP2350_ACCESSCTRL(obj);
    int m;

    notifier_list_init(&s->config_notifiers);
    for (m = 0; m < RP2350_NUM_MASTERS; m++) {
        s->port[m].s = s;
        s->port[m].master = m;
        memory_region_init_io(&s->port[m].iomem, obj, &rp2350_accessctrl_ops,
                              &s->port[m], TYPE_RP2350_ACCESSCTRL,
                              RP2350_ATOMIC_REGION_SIZE);
    }
    /* System memory itself is the debugger's port. */
    sysbus_init_mmio(SYS_BUS_DEVICE(obj),
                     &s->port[RP2350_MASTER_DEBUG].iomem);
}

static void rp2350_accessctrl_realize(DeviceState *dev, Error **errp)
{
    RP2350AccessCtrlState *s = RP2350_ACCESSCTRL(dev);
    Object *obj = OBJECT(dev);
    int m, i;

    if (!s->bus) {
        error_setg(errp, "bus property was not set");
        return;
    }
    address_space_init(&s->bus_as, s->bus, "rp2350-accessctrl.bus");

    for (m = 0; m < RP2350_ACCESSCTRL_NUM_VIEWS; m++) {
        g_autofree char *name = g_strdup_printf("rp2350.%s-bus",
                                                m == RP2350_MASTER_DMA ?
                                                "dma" : m ? "core1" : "core0");

        memory_region_init(&s->view[m], obj, name, UINT64_MAX);
        memory_region_init_alias(&s->view_bus[m], obj, "rp2350.bus", s->bus,
                                 0, UINT64_MAX);
        memory_region_add_subregion_overlap(&s->view[m], 0, &s->view_bus[m],
                                            0);
        memory_region_add_subregion_overlap(&s->view[m],
                                            RP2350_ACCESSCTRL_BASE,
                                            &s->port[m].iomem, 2);

        s->gate[m] = g_new0(RP2350AccessCtrlGate, NUM_RANGES);
        for (i = 0; i < NUM_RANGES; i++) {
            RP2350AccessCtrlGate *g = &s->gate[m][i];
            const RP2350AccessCtrlRange *r = &rp2350_accessctrl_ranges[i];
            g_autofree char *gname = g_strdup_printf("rp2350-accessctrl.%s",
                                                     r->name);

            g->s = s;
            g->master = m;
            g->range = r;
            memory_region_init_io(&g->iomem, obj, &gate_ops, g, gname,
                                  r->size);
            memory_region_set_enabled(&g->iomem, false);
            memory_region_add_subregion_overlap(&s->view[m], r->base,
                                                &g->iomem, 1);
        }
    }
    reset_regs(s);
    update_gates(s);
}

static int rp2350_accessctrl_post_load(void *opaque, int version_id)
{
    RP2350AccessCtrlState *s = opaque;

    update_gates(s);
    return 0;
}

static const VMStateDescription vmstate_rp2350_accessctrl = {
    .name = TYPE_RP2350_ACCESSCTRL,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = rp2350_accessctrl_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, RP2350AccessCtrlState,
                             RP2350_ACCESSCTRL_NUM_REGS),
        VMSTATE_END_OF_LIST()
    },
};

static const Property rp2350_accessctrl_properties[] = {
    DEFINE_PROP_LINK("bus", RP2350AccessCtrlState, bus, TYPE_MEMORY_REGION,
                     MemoryRegion *),
};

static void rp2350_accessctrl_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = rp2350_accessctrl_realize;
    dc->vmsd = &vmstate_rp2350_accessctrl;
    device_class_set_props(dc, rp2350_accessctrl_properties);
    rc->phases.enter = rp2350_accessctrl_enter_reset;
    rc->phases.hold = rp2350_accessctrl_hold_reset;
}

/* [spec:nuos:req:emu.accessctrl] */
static const TypeInfo rp2350_accessctrl_info = {
    .name          = TYPE_RP2350_ACCESSCTRL,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RP2350AccessCtrlState),
    .instance_init = rp2350_accessctrl_init,
    .class_init    = rp2350_accessctrl_class_init,
};

static void rp2350_accessctrl_register_types(void)
{
    type_register_static(&rp2350_accessctrl_info);
}
type_init(rp2350_accessctrl_register_types)
