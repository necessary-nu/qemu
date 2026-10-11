/*
 * ESP32-P4 LP system registers (LP_SYS)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapter
 * 21 (System Registers), 5 (Low-Power CPU) and 11 (Reset and Clock), and
 * ESP-IDF's lp_system_reg.h for v3, the authority for the registers, their
 * reset values and access types.
 *
 * Every register of the header is here, with its reset value; a write
 * changes only its read-write bits, and its read-only bits read as the
 * hardware drives them. The block is in the LP always-on domain: an HP
 * core reset leaves it as it is. What the registers do:
 *
 * - BOOT_ADDR_HP_CORE1, the word through which software hands HP core 1
 *   its entry point once released: the boot ROM's
 *   ets_set_appcpu_boot_addr() writes it, and the ROM on core 1 jumps to
 *   it, at once if bit 31 is set (to the address with bit 31 clear), or
 *   once it is non-zero after the ROM's own start-up. LP_STORE0 to 15,
 *   scratch words that keep their values through CPU resets and HP core
 *   resets, which ESP-IDF and the ROM use for the reset reason, the wake
 *   stub and the sleep mode (RTC_*_REG in ESP-IDF's rtc_cntl names).
 *
 * - The LP AHB matrix's record of the access it refused (21.2.2.11), one
 *   the LP APM did not permit (SECURE 0) or one to an address nothing
 *   decodes (SECURE 1): LP_ADDRHOLE_ADDR and LP_ADDRHOLE_INFO, the master's
 *   ID, whether it was a write, and SECURE. The record is taken as
 *   LP_ADDRHOLE_INT's raw bit rises and holds until software clears the
 *   bit, so it is the first refusal since. The LP APM records its refusals
 *   here; undecoded addresses answer with a bus error but are not
 *   recorded, so the record of the LP core's IBUS and DBUS, which only
 *   such accesses raise, IDBUS_ADDRHOLE_ADDR and _INFO, stays clear.
 *
 * - The interrupt registers INT_RAW, INT_ST, INT_ENA and INT_CLR, whose
 *   enabled raw bits drive LP_SYSREG_INTR, source 19 of the interrupt
 *   matrix. LP_ADDRHOLE_INT is raised as above, ETM_TASK_ULP_INT and
 *   SLOW_CLK_TICK_INT by their inputs from the ETM and the LP clock
 *   control; the LP core's bus timeouts are never raised, as no slave
 *   keeps an access waiting.
 *
 * - LP_CORE_ERR_RESP_DIS (0x1BC): bit 0 for the LP core's IBUS, bit 1 its
 *   DBUS and bit 2 its AHB bus: an access on a bus whose error response
 *   is disabled that the bus would answer with an error completes without
 *   one, a read returning 0, and the LP core takes no access fault.
 *
 * - SYS_CTRL's SYS_SW_RST (write-only) makes the software system reset,
 *   which the machine carries out ("sys-sw-reset"); LP_CORE_DISABLE and
 *   SYSTIMER_STALL_SEL go to the LP core and the RTC timer; the FIB
 *   bits give the super watchdog's and the brownout detector's resets
 *   ("swd-reset-en", "bod-reset-en"; FIB bits 2 and 1). The TRM does not
 *   describe the FIB: as the field names have it, LP_FIB_SEL's bit set
 *   selects the software's DIG_FIB bit and clear the analog state's
 *   ANA_FIB (which reads its reset value, the analog FIB not being
 *   modelled), and, as ESP32-C6's LP_ANA_FIB_ENABLE has it, a FIB bit set
 *   bypasses the reset: from reset neither reset is enabled, and
 *   software enables one by clearing its DIG_FIB bit;
 *   LP_CORE_ETM_WAKEUP_FLAG sets on its ETM input and clears on a write of
 *   1 to it or to LP_CORE_ETM_WAKEUP_FLAG_CLR, and is the PMU's ETM wakeup
 *   source (bit 17), which wakes the LP core.
 *
 * - LP_CORE_BOOT_ADDR (0x50100000 from reset), the LP core's boot address,
 *   and BOOT_ADDR_HP_LP, the application's, at which the LP ROM starts it,
 *   for the LP core's start (esp32p4_lp_sys_lp_core_boot_addr(),
 *   esp32p4_lp_sys_lp_core_app_addr()).
 *
 * - LP_CPU_DBG_PC, the LP core's current PC, and LP_CPU_EXC_PC, its PC at
 *   its last trap.
 *
 * - PAD_RTC_HOLD_CTRL0 and _CTRL1: bit n of CTRL0 holds GPIO n and bit n
 *   of CTRL1 GPIO(32 + n), as ESP-IDF's gpio_ll_hold_en() has them on
 *   v3.x silicon ("pad-hold" outputs, ORed with HP_SYSTEM's hold for the
 *   GPIO block). DED_PAD_RTC_HOLD_CTRL holds the flash and PSRAM pads,
 *   which have no pad model, and is kept as written.
 *
 * - HP_ROOT_CLK_CTRL's gates of the HP system's root clocks go to the
 *   reset and clock control ("hp-root-clk").
 *
 * - RNG_DATA: the legacy random number generator, a fresh random word on
 *   each read while RNG_CFG's timer or SAR sampling runs (from QEMU's
 *   guest random source, which -seed makes reproducible), the last word
 *   otherwise. Its sample count, RNG_SAR_DATA, reads 0.
 *
 * - EXT_WAKEUP1_STATUS records the RTC pads of the PMU's ext1 wakeup
 *   (esp32p4_lp_sys_ext_wakeup1()) until EXT_WAKEUP1_STATUS_CLR.
 *
 * - USB_CTRL (0x100): SW_HW_USB_PHY_SEL and SW_USB_PHY_SEL, which choose
 *   the full-speed PHYs of USB Serial/JTAG and the full-speed OTG, to
 *   USB_WRAP ("usb-phy-sel" outputs); the high-speed OTG's suspend and
 *   mode flags for the PMU hold what software writes, and
 *   USBOTG20_WAKEUP_CLR reads as zero. HP_USB_OTGHS_PHY_CTRL (0x204), the
 *   high-speed OTG PHY's ID pull-up, D+/D- pull-downs and VBUS charge and
 *   discharge, holds what software writes: the model does not take the
 *   bus's electrical state from it.
 *
 * - PAD_COMP0 and PAD_COMP1 configure the analog comparators
 *   (esp32p4_ana_cmpr.c), which read them with esp32p4_lp_sys_reg() and
 *   are told of each write ("pad-comp").
 *
 * Kept as written for the blocks that would act on them, none of which is
 * modelled, and given to them by esp32p4_lp_sys_reg(): the reset bypasses
 * (LP_RST_CTRL, HP_POR_RST_BYPASS_CTRL), the boot mode override
 * FORCE_DOWNLOAD_BOOT, the IO MUX reset disable, the PAU's
 * BACKUP_DMA_CFG0 to 2, the memories' and ROMs' aux and power controls,
 * the pad groups' power, the discharge controls, the bus timeouts'
 * thresholds and the ECO registers. The debug probe's signals are not
 * modelled: LP_PROBE_OUT reads 0.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/guest-random.h"
#include "qemu/log.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/esp32p4_lp_sys.h"
#include "migration/vmstate.h"
#include "target/riscv/esp32p4_cpu.h"

#define A_VER_DATE              0x000
#define A_CLK_SEL_CTRL          0x004
#define A_SYS_CTRL              0x008
#define A_LP_CLK_CTRL           0x00c
#define A_LP_RST_CTRL           0x010
#define A_LP_CORE_BOOT_ADDR     0x018
#define A_EXT_WAKEUP1           0x01c
#define A_EXT_WAKEUP1_STATUS    0x020
#define A_LP_TCM_PWR_CTRL       0x024
#define A_BOOT_ADDR_HP_LP       0x028
#define A_LP_STORE(n)           (0x02c + 4 * (n))
#define A_LP_PROBEA_CTRL        0x06c
#define A_LP_PROBEB_CTRL        0x070
#define A_LP_PROBE_OUT          0x074
#define A_F2S_APB_BRG_CNTL      0x09c
#define A_USB_CTRL              0x100
#define A_ANA_XPD_PAD_GROUP     0x10c
#define A_LP_TCM_RAM_RDN_ECO_CS 0x110
#define A_LP_TCM_RAM_RDN_ECO_LOW 0x114
#define A_LP_TCM_RAM_RDN_ECO_HIGH 0x118
#define A_LP_TCM_ROM_RDN_ECO_CS 0x11c
#define A_LP_TCM_ROM_RDN_ECO_LOW 0x120
#define A_LP_TCM_ROM_RDN_ECO_HIGH 0x124
#define A_HP_ROOT_CLK_CTRL      0x130
#define A_LP_PMU_RDN_ECO_LOW    0x138
#define A_LP_PMU_RDN_ECO_HIGH   0x13c
#define A_PAD_COMP0             0x148
#define A_PAD_COMP1             0x14c
#define A_BACKUP_DMA_CFG0       0x154
#define A_BACKUP_DMA_CFG1       0x158
#define A_BACKUP_DMA_CFG2       0x15c
#define A_BOOT_ADDR_HP_CORE1    0x164
#define A_LP_ADDRHOLE_ADDR      0x168
#define A_LP_ADDRHOLE_INFO      0x16c
#define A_INT_RAW               0x170
#define A_INT_ST                0x174
#define A_INT_ENA               0x178
#define A_INT_CLR               0x17c
#define A_HP_MEM_AUX_CTRL       0x180
#define A_LP_MEM_AUX_CTRL       0x184
#define A_HP_ROM_AUX_CTRL       0x188
#define A_LP_ROM_AUX_CTRL       0x18c
#define A_LP_CPU_DBG_PC         0x190
#define A_LP_CPU_EXC_PC         0x194
#define A_IDBUS_ADDRHOLE_ADDR   0x198
#define A_IDBUS_ADDRHOLE_INFO   0x19c
#define A_HP_POR_RST_BYPASS_CTRL 0x1a0
#define A_RNG_DATA              0x1a4
#define A_LP_CORE_AHB_TIMEOUT   0x1b0
#define A_LP_CORE_IBUS_TIMEOUT  0x1b4
#define A_LP_CORE_DBUS_TIMEOUT  0x1b8
#define A_LP_CORE_ERR_RESP_DIS  0x1bc
#define A_RNG_CFG               0x1c0
#define A_PAD_RTC_HOLD_CTRL0    0x1c4
#define A_PAD_RTC_HOLD_CTRL1    0x1c8
#define A_DED_PAD_RTC_HOLD_CTRL 0x1cc
#define A_DISCHARGE             0x200
#define A_HP_USB_OTGHS_PHY_CTRL 0x204

#define R(a)                    ((a) / 4)

/* SYS_CTRL */
#define SYS_CTRL_LP_CORE_DISABLE BIT(0)
#define SYS_CTRL_SYS_SW_RST     BIT(1)
#define SYS_CTRL_DIG_FIB_SHIFT  3
#define SYS_CTRL_ANA_FIB_SHIFT  14
#define SYS_CTRL_LP_FIB_SEL_SHIFT 21
#define SYS_CTRL_ETM_WAKEUP_FLAG_CLR BIT(29)
#define SYS_CTRL_ETM_WAKEUP_FLAG BIT(30)
#define SYS_CTRL_SYSTIMER_STALL_SEL BIT(31)
/* The FIB bits of the brownout detector and the super watchdog. */
#define FIB_BOD                 1
#define FIB_SWD                 2

#define EXT_WAKEUP1_STATUS_CLR  BIT(16)
#define EXT_WAKEUP1_PADS        0xffff

/* INT_*: LP_ADDRHOLE_INT, IDBUS_ADDRHOLE_INT, three timeouts, ETM, tick. */
#define INT_LP_ADDRHOLE         BIT(0)
#define INT_ETM_TASK_ULP        BIT(5)
#define INT_SLOW_CLK_TICK       BIT(6)
#define INT_ALL                 0x7f

/* LP_ADDRHOLE_INFO: the master's ID in bits 4:0, then WR and SECURE. */
#define ADDRHOLE_ID_MASK        0x1f
#define ADDRHOLE_WR             BIT(5)
#define ADDRHOLE_SECURE         BIT(6)

#define RNG_TIMER_EN            BIT(0)
#define RNG_SAR_ENABLE          BIT(9)

typedef struct {
    uint32_t reset;
    uint32_t wmask;
    bool valid;
} ESP32P4LPSysReg;

#define REG(a, r, w)    [R(a)] = { .reset = (r), .wmask = (w), .valid = true }
#define RO(a)           [R(a)] = { .valid = true }

static const ESP32P4LPSysReg esp32p4_lp_sys_regs[ESP32P4_LP_SYS_NREGS] = {
    REG(A_VER_DATE, 0x20250214, 0xffffffff),
    REG(A_CLK_SEL_CTRL, 0, 0x00030000),
    REG(A_SYS_CTRL, 0x1fffc7f8, 0x9fe00ffd),
    REG(A_LP_CLK_CTRL, 0x4001, 0x4001),
    REG(A_LP_RST_CTRL, 0x3, 0x7),
    REG(A_LP_CORE_BOOT_ADDR, 0x50100000, 0xffffffff),
    REG(A_EXT_WAKEUP1, 0, EXT_WAKEUP1_PADS),
    RO(A_EXT_WAKEUP1_STATUS),
    REG(A_LP_TCM_PWR_CTRL, 0, 0xa0),
    REG(A_BOOT_ADDR_HP_LP, 0, 0xffffffff),
    REG(A_LP_STORE(0), 0, 0xffffffff),
    REG(A_LP_STORE(1), 0, 0xffffffff),
    REG(A_LP_STORE(2), 0, 0xffffffff),
    REG(A_LP_STORE(3), 0, 0xffffffff),
    REG(A_LP_STORE(4), 0, 0xffffffff),
    REG(A_LP_STORE(5), 0, 0xffffffff),
    REG(A_LP_STORE(6), 0, 0xffffffff),
    REG(A_LP_STORE(7), 0, 0xffffffff),
    REG(A_LP_STORE(8), 0, 0xffffffff),
    REG(A_LP_STORE(9), 0, 0xffffffff),
    REG(A_LP_STORE(10), 0, 0xffffffff),
    REG(A_LP_STORE(11), 0, 0xffffffff),
    REG(A_LP_STORE(12), 0, 0xffffffff),
    REG(A_LP_STORE(13), 0, 0xffffffff),
    REG(A_LP_STORE(14), 0, 0xffffffff),
    REG(A_LP_STORE(15), 0, 0xffffffff),
    REG(A_LP_PROBEA_CTRL, 0, 0x1fffffff),
    REG(A_LP_PROBEB_CTRL, 0, 0x01ffffff),
    RO(A_LP_PROBE_OUT),
    REG(A_F2S_APB_BRG_CNTL, 0, 0x1),
    REG(A_USB_CTRL, 0, 0x1b),
    REG(A_ANA_XPD_PAD_GROUP, 0xff, 0xff),
    REG(A_LP_TCM_RAM_RDN_ECO_CS, 0, 0x1),
    REG(A_LP_TCM_RAM_RDN_ECO_LOW, 0, 0xffffffff),
    REG(A_LP_TCM_RAM_RDN_ECO_HIGH, 0xffffffff, 0xffffffff),
    REG(A_LP_TCM_ROM_RDN_ECO_CS, 0, 0x1),
    REG(A_LP_TCM_ROM_RDN_ECO_LOW, 0, 0xffffffff),
    REG(A_LP_TCM_ROM_RDN_ECO_HIGH, 0xffffffff, 0xffffffff),
    REG(A_HP_ROOT_CLK_CTRL, 0x3, 0x3),
    REG(A_LP_PMU_RDN_ECO_LOW, 0, 0xffffffff),
    REG(A_LP_PMU_RDN_ECO_HIGH, 0xffffffff, 0xffffffff),
    REG(A_PAD_COMP0, 0, 0x1f),
    REG(A_PAD_COMP1, 0, 0x1f),
    REG(A_BACKUP_DMA_CFG0, 0x1906414a, 0xffffffff),
    REG(A_BACKUP_DMA_CFG1, 0, 0x80000000),
    REG(A_BACKUP_DMA_CFG2, 0, 0xffffffff),
    REG(A_BOOT_ADDR_HP_CORE1, 0, 0xffffffff),
    RO(A_LP_ADDRHOLE_ADDR),
    RO(A_LP_ADDRHOLE_INFO),
    RO(A_INT_RAW),
    RO(A_INT_ST),
    REG(A_INT_ENA, 0, INT_ALL),
    RO(A_INT_CLR),
    REG(A_HP_MEM_AUX_CTRL, 0x2070, 0xffffffff),
    REG(A_LP_MEM_AUX_CTRL, 0x2070, 0xffffffff),
    REG(A_HP_ROM_AUX_CTRL, 0x70, 0xffffffff),
    REG(A_LP_ROM_AUX_CTRL, 0x70, 0xffffffff),
    RO(A_LP_CPU_DBG_PC),
    RO(A_LP_CPU_EXC_PC),
    RO(A_IDBUS_ADDRHOLE_ADDR),
    RO(A_IDBUS_ADDRHOLE_INFO),
    REG(A_HP_POR_RST_BYPASS_CTRL, 0xff00ff00, 0xff00ff00),
    RO(A_RNG_DATA),
    REG(A_LP_CORE_AHB_TIMEOUT, 0x7fffff, 0x7fffff),
    REG(A_LP_CORE_IBUS_TIMEOUT, 0x1ffff, 0x1ffff),
    REG(A_LP_CORE_DBUS_TIMEOUT, 0x1ffff, 0x1ffff),
    REG(A_LP_CORE_ERR_RESP_DIS, 0, 0x7),
    REG(A_RNG_CFG, 0x3, 0x3ff),
    REG(A_PAD_RTC_HOLD_CTRL0, 0, 0xffffffff),
    REG(A_PAD_RTC_HOLD_CTRL1, 0, 0x01ffffff),
    REG(A_DED_PAD_RTC_HOLD_CTRL, 0, 0x03ffffff),
    REG(A_DISCHARGE, 0x4, 0xf),
    REG(A_HP_USB_OTGHS_PHY_CTRL, 0x7, 0x1f),
};

static uint32_t *esp32p4_lp_sys_r(ESP32P4LPSysState *s, hwaddr a)
{
    return &s->regs[R(a)];
}

static void esp32p4_lp_sys_update(ESP32P4LPSysState *s)
{
    qemu_set_irq(s->irq, !!(s->regs[R(A_INT_RAW)] & s->regs[R(A_INT_ENA)]));
}

static void esp32p4_lp_sys_raise(ESP32P4LPSysState *s, uint32_t bits)
{
    s->regs[R(A_INT_RAW)] |= bits;
    esp32p4_lp_sys_update(s);
}

/* [spec:nuos:req:emu.esp32p4.pms] */
void esp32p4_lp_sys_addrhole(ESP32P4LPSysState *s, uint32_t addr,
                             uint8_t id, bool write, bool illegal)
{
    if (!(s->regs[R(A_INT_RAW)] & INT_LP_ADDRHOLE)) {
        *esp32p4_lp_sys_r(s, A_LP_ADDRHOLE_ADDR) = addr;
        *esp32p4_lp_sys_r(s, A_LP_ADDRHOLE_INFO) =
            (id & ADDRHOLE_ID_MASK) | (write ? ADDRHOLE_WR : 0) |
            (illegal ? ADDRHOLE_SECURE : 0);
    }
    esp32p4_lp_sys_raise(s, INT_LP_ADDRHOLE);
}

/* [spec:nuos:req:emu.esp32p4.debug] */
void esp32p4_lp_sys_ext_wakeup1(ESP32P4LPSysState *s, uint32_t pads)
{
    *esp32p4_lp_sys_r(s, A_EXT_WAKEUP1_STATUS) |= pads & EXT_WAKEUP1_PADS;
}

/* [spec:nuos:req:emu.esp32p4.lp-core] */
uint32_t esp32p4_lp_sys_lp_core_boot_addr(ESP32P4LPSysState *s)
{
    return s->regs[R(A_LP_CORE_BOOT_ADDR)];
}

/* [spec:nuos:req:emu.esp32p4.lp-core] */
uint32_t esp32p4_lp_sys_lp_core_app_addr(ESP32P4LPSysState *s)
{
    return s->regs[R(A_BOOT_ADDR_HP_LP)];
}

/*
 * A FIB-controlled reset's enable: the FIB bit, the software's when
 * LP_FIB_SEL's bit is set, else the analog state's, clear.
 */
static bool esp32p4_lp_sys_fib_enable(ESP32P4LPSysState *s, int bit)
{
    uint32_t ctrl = s->regs[R(A_SYS_CTRL)];

    if (extract32(ctrl, SYS_CTRL_LP_FIB_SEL_SHIFT + bit, 1)) {
        return !extract32(ctrl, SYS_CTRL_DIG_FIB_SHIFT + bit, 1);
    }
    return !extract32(ctrl, SYS_CTRL_ANA_FIB_SHIFT + bit, 1);
}

/* [spec:nuos:req:emu.esp32p4.debug] */
static void esp32p4_lp_sys_update_outputs(ESP32P4LPSysState *s)
{
    uint32_t ctrl = s->regs[R(A_SYS_CTRL)];
    uint64_t held = s->regs[R(A_PAD_RTC_HOLD_CTRL0)] |
                    ((uint64_t)s->regs[R(A_PAD_RTC_HOLD_CTRL1)] << 32);
    uint32_t dis = s->regs[R(A_LP_CORE_ERR_RESP_DIS)];
    int i;

    for (i = 0; i < 3; i++) {
        qemu_set_irq(s->err_resp_dis[i], extract32(dis, i, 1));
    }
    for (i = 0; i < ESP32P4_LP_SYS_HOLDS; i++) {
        qemu_set_irq(s->pad_hold[i], extract64(held, i, 1));
    }
    qemu_set_irq(s->lp_core_disable, ctrl & SYS_CTRL_LP_CORE_DISABLE);
    qemu_set_irq(s->etm_wakeup_flag, !!(ctrl & SYS_CTRL_ETM_WAKEUP_FLAG));
    qemu_set_irq(s->stall_sel, !!(ctrl & SYS_CTRL_SYSTIMER_STALL_SEL));
    qemu_set_irq(s->swd_reset_en, esp32p4_lp_sys_fib_enable(s, FIB_SWD));
    qemu_set_irq(s->bod_reset_en, esp32p4_lp_sys_fib_enable(s, FIB_BOD));
    for (i = 0; i < 2; i++) {
        qemu_set_irq(s->hp_root_clk[i],
                     extract32(s->regs[R(A_HP_ROOT_CLK_CTRL)], i, 1));
        qemu_set_irq(s->usb_phy_sel[i],
                     extract32(s->regs[R(A_USB_CTRL)], i, 1));
    }
}

/* The legacy RNG: a new word while its timer or SAR sampling runs. */
static uint32_t esp32p4_lp_sys_rng(ESP32P4LPSysState *s)
{
    uint32_t *data = esp32p4_lp_sys_r(s, A_RNG_DATA);

    if (s->regs[R(A_RNG_CFG)] & (RNG_TIMER_EN | RNG_SAR_ENABLE)) {
        qemu_guest_getrandom_nofail(data, sizeof(*data));
    }
    return *data;
}

uint32_t esp32p4_lp_sys_reg(ESP32P4LPSysState *s, hwaddr offset)
{
    CPURISCVState *env;

    switch (offset) {
    case A_INT_ST:
        return s->regs[R(A_INT_RAW)] & s->regs[R(A_INT_ENA)];
    case A_LP_CPU_DBG_PC:
        env = s->lp_cpu ? &s->lp_cpu->parent_obj.env : NULL;
        return env ? env->pc : 0;
    case A_LP_CPU_EXC_PC:
        return s->lp_cpu ? s->lp_cpu->exc_pc : 0;
    default:
        return offset < ESP32P4_LP_SYS_NREGS * 4 ? s->regs[R(offset)] : 0;
    }
}

static uint64_t esp32p4_lp_sys_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4LPSysState *s = opaque;
    hwaddr reg = addr & ~3;
    uint32_t v;

    if (reg >= ESP32P4_LP_SYS_NREGS * 4 ||
        !esp32p4_lp_sys_regs[R(reg)].valid) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of %u bytes at reserved "
                      "offset 0x%" HWADDR_PRIx "\n", TYPE_ESP32P4_LP_SYS,
                      size, addr);
        return 0;
    }
    if (reg == A_RNG_DATA) {
        v = esp32p4_lp_sys_rng(s);
    } else {
        v = esp32p4_lp_sys_reg(s, reg);
    }
    return extract32(v, (addr & 3) * 8, size * 8);
}

/* [spec:nuos:req:emu.esp32p4.machine] */
/* [spec:nuos:req:emu.esp32p4.pms] */
/* [spec:nuos:req:emu.esp32p4.debug] */
static void esp32p4_lp_sys_write(void *opaque, hwaddr addr, uint64_t value,
                                 unsigned size)
{
    ESP32P4LPSysState *s = opaque;
    hwaddr reg = addr & ~3;
    unsigned shift = (addr & 3) * 8;
    uint32_t bytes = MAKE_64BIT_MASK(shift, size * 8);
    uint32_t v = ((uint32_t)value << shift) & bytes;
    const ESP32P4LPSysReg *info;
    uint32_t *r;

    if (reg >= ESP32P4_LP_SYS_NREGS * 4 ||
        !esp32p4_lp_sys_regs[R(reg)].valid) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write of %u bytes at reserved "
                      "offset 0x%" HWADDR_PRIx " (value 0x%" PRIx64 ")\n",
                      TYPE_ESP32P4_LP_SYS, size, addr, value);
        return;
    }
    info = &esp32p4_lp_sys_regs[R(reg)];
    r = &s->regs[R(reg)];

    switch (reg) {
    case A_INT_RAW:
    case A_INT_CLR:
        /* INT_RAW is write-1-to-clear, as INT_CLR is. */
        s->regs[R(A_INT_RAW)] &= ~(v & INT_ALL);
        esp32p4_lp_sys_update(s);
        return;
    case A_SYS_CTRL:
        /* LP_CORE_ETM_WAKEUP_FLAG is write-1-to-clear, as its CLR is. */
        if (v & (SYS_CTRL_ETM_WAKEUP_FLAG | SYS_CTRL_ETM_WAKEUP_FLAG_CLR)) {
            *r &= ~SYS_CTRL_ETM_WAKEUP_FLAG;
        }
        *r = (*r & ~(info->wmask & bytes)) | (v & info->wmask);
        esp32p4_lp_sys_update_outputs(s);
        if (v & SYS_CTRL_SYS_SW_RST) {
            qemu_irq_pulse(s->sw_reset);
        }
        return;
    case A_EXT_WAKEUP1:
        if (v & EXT_WAKEUP1_STATUS_CLR) {
            *esp32p4_lp_sys_r(s, A_EXT_WAKEUP1_STATUS) = 0;
        }
        break;
    case A_USB_CTRL:
        /* USBOTG20_WAKEUP_CLR clears the PMU's wakeup, not modelled. */
        break;
    default:
        if (!info->wmask) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only offset "
                          "0x%" HWADDR_PRIx "\n", TYPE_ESP32P4_LP_SYS, addr);
            return;
        }
        break;
    }
    *r = (*r & ~(info->wmask & bytes)) | (v & info->wmask);

    switch (reg) {
    case A_INT_ENA:
        esp32p4_lp_sys_update(s);
        break;
    case A_LP_CORE_ERR_RESP_DIS:
    case A_PAD_RTC_HOLD_CTRL0:
    case A_PAD_RTC_HOLD_CTRL1:
    case A_HP_ROOT_CLK_CTRL:
    case A_USB_CTRL:
        esp32p4_lp_sys_update_outputs(s);
        break;
    case A_PAD_COMP0:
    case A_PAD_COMP1:
        qemu_irq_pulse(s->pad_comp[(reg - A_PAD_COMP0) / 4]);
        break;
    default:
        break;
    }
}

static const MemoryRegionOps esp32p4_lp_sys_ops = {
    .read = esp32p4_lp_sys_read,
    .write = esp32p4_lp_sys_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

/* [spec:nuos:req:emu.esp32p4.lp-core] */
static void esp32p4_lp_sys_set_etm_wakeup(void *opaque, int n, int level)
{
    ESP32P4LPSysState *s = opaque;

    if (level) {
        s->regs[R(A_SYS_CTRL)] |= SYS_CTRL_ETM_WAKEUP_FLAG;
        qemu_set_irq(s->etm_wakeup_flag, 1);
    }
}

static void esp32p4_lp_sys_set_etm_task_ulp(void *opaque, int n, int level)
{
    if (level) {
        esp32p4_lp_sys_raise(opaque, INT_ETM_TASK_ULP);
    }
}

static void esp32p4_lp_sys_set_slow_clk_tick(void *opaque, int n, int level)
{
    if (level) {
        esp32p4_lp_sys_raise(opaque, INT_SLOW_CLK_TICK);
    }
}

static void esp32p4_lp_sys_reset_hold(Object *obj, ResetType type)
{
    ESP32P4LPSysState *s = ESP32P4_LP_SYS(obj);

    for (int i = 0; i < ESP32P4_LP_SYS_NREGS; i++) {
        s->regs[i] = esp32p4_lp_sys_regs[i].reset;
    }
}

static void esp32p4_lp_sys_reset_exit(Object *obj, ResetType type)
{
    ESP32P4LPSysState *s = ESP32P4_LP_SYS(obj);

    esp32p4_lp_sys_update(s);
    esp32p4_lp_sys_update_outputs(s);
    for (int i = 0; i < ESP32P4_LP_SYS_PAD_COMPS; i++) {
        qemu_irq_pulse(s->pad_comp[i]);
    }
}

static void esp32p4_lp_sys_init(Object *obj)
{
    ESP32P4LPSysState *s = ESP32P4_LP_SYS(obj);
    DeviceState *dev = DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32p4_lp_sys_ops, s,
                          TYPE_ESP32P4_LP_SYS, ESP32P4_LP_SYS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    qdev_init_gpio_out_named(dev, s->err_resp_dis, ESP32P4_LP_SYS_ERR_RESP_DIS,
                             3);
    qdev_init_gpio_out_named(dev, s->pad_hold, ESP32P4_LP_SYS_PAD_HOLD,
                             ESP32P4_LP_SYS_HOLDS);
    qdev_init_gpio_out_named(dev, &s->sw_reset, ESP32P4_LP_SYS_SW_RESET, 1);
    qdev_init_gpio_out_named(dev, &s->lp_core_disable,
                             ESP32P4_LP_SYS_LP_CORE_DISABLE, 1);
    qdev_init_gpio_out_named(dev, &s->stall_sel, ESP32P4_LP_SYS_STALL_SEL, 1);
    qdev_init_gpio_out_named(dev, &s->swd_reset_en,
                             ESP32P4_LP_SYS_SWD_RESET_EN, 1);
    qdev_init_gpio_out_named(dev, &s->bod_reset_en,
                             ESP32P4_LP_SYS_BOD_RESET_EN, 1);
    qdev_init_gpio_out_named(dev, s->hp_root_clk, ESP32P4_LP_SYS_HP_ROOT_CLK,
                             2);
    qdev_init_gpio_out_named(dev, s->usb_phy_sel, ESP32P4_LP_SYS_USB_PHY_SEL,
                             2);
    qdev_init_gpio_out_named(dev, s->pad_comp, ESP32P4_LP_SYS_PAD_COMP,
                             ESP32P4_LP_SYS_PAD_COMPS);
    qdev_init_gpio_out_named(dev, &s->etm_wakeup_flag,
                             ESP32P4_LP_SYS_ETM_WAKEUP_FLAG, 1);
    qdev_init_gpio_in_named(dev, esp32p4_lp_sys_set_etm_wakeup,
                            ESP32P4_LP_SYS_LP_CORE_ETM_WAKEUP, 1);
    qdev_init_gpio_in_named(dev, esp32p4_lp_sys_set_etm_task_ulp,
                            ESP32P4_LP_SYS_ETM_TASK_ULP_INT, 1);
    qdev_init_gpio_in_named(dev, esp32p4_lp_sys_set_slow_clk_tick,
                            ESP32P4_LP_SYS_SLOW_CLK_TICK, 1);
}

static const VMStateDescription vmstate_esp32p4_lp_sys = {
    .name = TYPE_ESP32P4_LP_SYS,
    .version_id = 3,
    .minimum_version_id = 3,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, ESP32P4LPSysState, ESP32P4_LP_SYS_NREGS),
        VMSTATE_END_OF_LIST()
    },
};

static const Property esp32p4_lp_sys_properties[] = {
    DEFINE_PROP_LINK("lp-cpu", ESP32P4LPSysState, lp_cpu, TYPE_ESP32P4_CPU,
                     ESP32P4CPU *),
};

static void esp32p4_lp_sys_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32p4_lp_sys_reset_hold;
    rc->phases.exit = esp32p4_lp_sys_reset_exit;
    dc->vmsd = &vmstate_esp32p4_lp_sys;
    device_class_set_props(dc, esp32p4_lp_sys_properties);
}

/* [spec:nuos:req:emu.esp32p4.machine] */
/* [spec:nuos:req:emu.esp32p4.debug] */
static const TypeInfo esp32p4_lp_sys_info = {
    .name          = TYPE_ESP32P4_LP_SYS,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32P4LPSysState),
    .instance_init = esp32p4_lp_sys_init,
    .class_init    = esp32p4_lp_sys_class_init,
};

static void esp32p4_lp_sys_register_types(void)
{
    type_register_static(&esp32p4_lp_sys_info);
}
type_init(esp32p4_lp_sys_register_types)
