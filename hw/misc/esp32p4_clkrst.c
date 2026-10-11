/*
 * ESP32-P4 reset and clock control: HP_SYS_CLKRST, LP_AON_CLKRST, LPPERI
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapter
 * 11 (Reset and Clock); ESP-IDF's hp_sys_clkrst_reg.h, lp_clkrst_reg.h
 * and lpperi_reg.h for v3, which give the registers' layout, access and
 * reset values; and its hal/esp32p4 clk_tree_ll.h, clk_gate_ll.h and
 * peripheral LL drivers, esp_hw_support/port/esp32p4/rtc_clk.c and
 * soc/esp32p4 clk_tree_defs.h, which show how software drives the tree.
 *
 * The three register blocks of the chip's reset and clock control:
 * HP_SYS_CLKRST, the HP system's clocks and resets; LP_AON_CLKRST, the LP
 * system's, the HP root clock's source, the HP cores' reset control and
 * the reset causes; and LPPERI, the LP peripherals' clocks and resets.
 *
 * The clock tree, from its sources to the clock each block takes:
 *
 *   - The sources: the 40 MHz crystal (XTAL_CLK); RC_FAST_CLK, 20 MHz at
 *     LP_CLKRST_FOSC_DFREQ's reset value of 400, which trims it; RC_SLOW
 *     _CLK, 150 kHz; the RC32K oscillator and the board's 32 kHz crystal,
 *     32768 Hz; OSC_SLOW_CLK, a clock on XTAL_32K_N; the 8 MHz LP PLL from
 *     RC32K or XTAL32K (LP_CLKRST_ANA_SEL_REF_PLL8M; the TRM calls 0
 *     invalid, ESP-IDF uses it for RC32K); and the CPU, system, MSPI,
 *     audio and SDIO PLLs, which the analog blocks model (esp32p4_ana.c).
 *     The PMU powers the crystal, RC_FAST, RC32K, XTAL32K and the LP PLL
 *     up and down; a powered-down source is stopped. FOSC_DFREQ is the
 *     one trim the model applies: ESP-IDF gives RC_FAST 17.5 MHz at 100,
 *     the TRM 20 MHz at the reset value, and the model takes it to be
 *     linear through those. The other trims (SOSC_DFREQ, RC32K_DFREQ, the
 *     XTAL32K settings) are not characterised and change nothing, and the
 *     oscillators run from the moment they are powered up.
 *   - LP_CLKRST_HP_CLK_CTRL and CLK_TO_HP gate each source into the HP
 *     system, and gate the clocks the pads bring in.
 *   - ROOT_CLK, from XTAL_CLK, CPLL_CLK or RC_FAST_CLK (HP_ROOT_CLK_SRC_SEL)
 *     and gated by HP_ROOT_CLK_EN, divides into CPU_CLK, which divides
 *     into MEM_CLK, then SYS_CLK, then APB_CLK, each by an integer and a
 *     fraction, (DIV_NUM + 1) + NUMERATOR / DENOMINATOR, none with a
 *     denominator of 0. Writing SOC_CLK_DIV_UPDATE takes all four dividers
 *     from ROOT_CLK_CTRL0..3 at once; until then the ones in effect stay,
 *     and CPU_CLK_STATUS0 shows CPU_CLK's. The source mux changes at once.
 *     The TRM limits CPU_CLK to 400 MHz on v3.x, MEM_CLK and SYS_CLK to
 *     200 MHz and APB_CLK to 100 MHz; a configuration above them is logged
 *     as a guest error and runs as configured, as what the hardware then
 *     does is not documented.
 *   - The PLLs' fixed dividers: PLL_F240M, F160M, F120M, F80M and F20M
 *     from SPLL_CLK, F50M and F25M from MPLL_CLK (REF_CLK_CTRL*), and the
 *     USB 48 MHz, 12 MHz and 25 MHz from SPLL_CLK and MPLL_CLK
 *     (LP_CLKRST_HP_USB_CLKRST_CTRL0).
 *   - The LP clocks: LP_SLOW_CLK (SLOW_CLK_SEL: RC_SLOW, XTAL32K, RC32K
 *     as ESP-IDF has it, or OSC_SLOW), LP_FAST_CLK (FAST_CLK_SEL: RC_FAST,
 *     XTAL or the LP PLL), LP_DYN_SLOW_CLK, which follows LP_SLOW_CLK, and
 *     LP_DYN_FAST_CLK, which follows LP_FAST_CLK but in LP_SLEEP, when the
 *     PMU has it follow LP_SLOW_CLK (TRM 15.4.2.5), LP_PERI_CLK
 *     (LP_DYN_FAST_CLK / (LP_PERI_DIV_NUM + 1)) and XTAL_D2_CLK.
 *   - Every block's bus clocks (SOC_CLK_CTRL0..3: CPU_CLK, MEM_CLK,
 *     SYS_CLK or APB_CLK, gated), and its function clocks (PERI_CLK_CTRL*,
 *     LP_CLKRST's USB clocks, LPPERI, and the eFuse controller's 20 MHz
 *     XTAL_D2_CLK, which TRM table 11.2-7 gives it and LPPERI_CK_EN_LP_EFUSE
 *     gates): a source mux, a gate and a divider,
 *     integer, fractional as above, or I2S's N + b/a from its X, Y, Z and
 *     YN1. Each divider register holds its divisor less one; I2S's N is
 *     the divisor's integer part. A mux setting the TRM calls invalid
 *     selects a stopped clock; where the TRM calls invalid a setting
 *     ESP-IDF uses (I2S's PLL_F160M, the calibration clock's RC32K and
 *     the DSI PHY's clock), the model follows ESP-IDF.
 *   - The PVT monitor's clocks from PLL_F160M_CLK, which ESP-IDF enables
 *     for them (pmu_pvt.c), and the LP ADC's function clock from its bus
 *     clock, LP_PERI_CLK, as TRM table 11.2-7 has it, and its SAR clocks
 *     from that, as the HP ADC's are from its function clock. (The TRM
 *     gives LPADC_SAR1_DIV_NUM and LPADC_SAR2_DIV_NUM each the other's
 *     description; the model follows their names.)
 *   - The SDIO host's clocks: in low-speed mode its source divided by
 *     (SDIO_LS_CLK_DIV_NUM + 1) and by EDGE_N + 1 of the edge counts
 *     SDIO_LS_CLK_EDGE_CFG_UPDATE takes, in high-speed mode the SDIO PLL's
 *     three outputs. The phases the edge settings give are not modelled.
 *   - The three debug clock outputs (DBG_CLK_CTRL*), from the signals
 *     ESP-IDF's soc_clkout_sig_id_t names.
 *
 * Every clock is a clock output of this device, at its current
 * frequency, and the header gives the convention by which a block takes
 * its clocks and reset. Each clock's period is computed exactly from its
 * sources' frequencies (RC_FAST_CLK's as the fraction of Hz its trim
 * gives, the PLLs' as the analog blocks give them) through every divider
 * on its way, and rounded down to its Clock's unit once
 * (esp32p4_clk_period.h): the rounding of one clock never reaches those
 * divided from it. The device is the TYPE_ESP32P4_CLK_SOURCE of its
 * outputs, giving their exact periods to the blocks that count them. The
 * PMU, in the states that give it control, gates the clocks
 * PMU_n1_DIG_ICG_FUNC_EN lists (TRM table 15.4-2) and selects and gates
 * ROOT_CLK's source in place of HP_ROOT_CLK_SRC_SEL and HP_ROOT_CLK_EN.
 * FORCE_ON bits bypass automatic clock gating, which on the chip stops a
 * clock while its block is idle and restarts it as the block is used, so
 * that software sees no difference; ESP-IDF names one exception, and keeps
 * L2MEM_MEM_CLK_FORCE_ON set as a DMA access to L2MEM does not restart its
 * clock: clearing it is logged as unimplemented. The WFI autoclock gating
 * of the CPU subsystem (ESP-IDF's name) is of that kind. Once every HP
 * core whose CPU_WAITI_CTRL0 WAITI_ICG_EN is set waits in WFI, it stops
 * CPUICM_CLK, the clock of the subsystem's own interconnect, the
 * CPUICM_DELAY_NUM cycles of ROOT_CLK_CTRL0 on, unless
 * CLK_FORCE_ON_CTRL0's CPUICM_GATED_CLK_FORCE_ON bypasses that, as it
 * does from reset (ESP-IDF clears it as it boots), and restarts it as a
 * core leaves WFI; ESP-IDF clears core 1's WAITI_ICG_EN while it keeps
 * core 1 stopped, so that core 0's WFI alone gates the clock. The TRM
 * names the clock only in those registers' descriptions. Only the cores
 * reach memory and the peripherals through it, and none does while every
 * one waits: the cores' own clocks (which their cycle counters count),
 * their CLICs, the CLINT and every other master's path run on, and the
 * model shows nothing of it. The DPA strategy's random divider of the
 * crypto clock is not modelled.
 *
 * Resets. HP_RST_EN*, LP_CLKRST's resets and LPPERI_RESET_EN hold their
 * blocks in reset while set, unless the block's FORCE_NORST bit is set;
 * RST_EN_CRYPTO also resets every crypto block. The HP cores are held by
 * RST_EN_CORE<n>_GLOBAL, unless FORCE_NORST_CORE<n>, and reset by a CPU
 * reset: software's (LP_CLKRST_HPCORE<n>_SW_RESET), the RTC watchdog's
 * and the MWDTs': when STALL_EN is set the core stalls for STALL_WAIT
 * cycles first, then it is held for RESET_LENGTH cycles, counted in
 * LP_DYN_FAST_CLK for LP_CLKRST's resets and XTAL_CLK for HP_SYS_CLKRST's
 * (the TRM gives only "clock cycles"); while that clock is stopped the
 * count waits. LPPERI_RST_EN_LP_CORE resets the LP core at once. Each
 * reset records its cause in LP_CLKRST_RESET_CAUSE, and a chip reset
 * records 0x01 for all three cores, which the ROM reads as a power-on.
 * An HP core reset, which the watchdogs request, holds both HP cores and
 * every HP block in reset for its length and returns HP_SYS_CLKRST's
 * registers to their reset values, leaving the LP system as it is; a
 * system reset is QEMU's, with the cause a block gives it.
 * LP_CLKRST_HPCORE<n>_STAT_VECTOR_SEL picks the core's reset vector.
 * The PMU's power domains hold their blocks in reset while powered down
 * (the header's "pmu-power"), and record their power-up resets.
 *
 * HP_SYS_CLKRST_ANA_PLL_CTRL0's calibration controls drive the analog
 * blocks', and show their calibrations' end.
 *
 * Held as written, as nothing documents what they clock: the test-mode
 * clocks (REF_CLK_CTRL*'s TM_*_CLK_EN), the pad BIST clocks
 * (PERI_CLK_CTRL26/27's PADBIST_*), EMAC_UNUSED0/1_CLK_EN, the USB 2.0
 * OTG's ULPI clock enable, LP_CLKRST_LP_CLK_PO_EN and
 * ETM_EVENT_TICK_EN.
 *
 * Not modelled: halting a core on reset for the debugger
 * (OCD_HALT_ON_RESET), the lockup reset (the cores do not report a
 * lockup), LP_CLKRST's software stall codes, and the HP system's
 * interconnect, TCM, L2MEM and register clocks and resets, which the
 * model keeps running; software that stops them is logged as
 * unimplemented.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/bswap.h"
#include "qemu/host-utils.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/esp32p4_clkrst.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"

/*
 * The register file: HP_SYS_CLKRST's registers at (offset / 4),
 * LP_AON_CLKRST's at 64 + (offset / 4), LPPERI's at 88 + (offset / 4),
 * and each block's DATE register after its others.
 */
enum {
    R_CLK_EN0,
    R_ROOT_CLK_CTRL0,
    R_ROOT_CLK_CTRL1,
    R_ROOT_CLK_CTRL2,
    R_ROOT_CLK_CTRL3,
    R_SOC_CLK_CTRL0,
    R_SOC_CLK_CTRL1,
    R_SOC_CLK_CTRL2,
    R_SOC_CLK_CTRL3,
    R_REF_CLK_CTRL0,
    R_REF_CLK_CTRL1,
    R_REF_CLK_CTRL2,
    R_PERI_CLK_CTRL00,
    R_PERI_CLK_CTRL01,
    R_PERI_CLK_CTRL02,
    R_PERI_CLK_CTRL03,
    R_PERI_CLK_CTRL10,
    R_PERI_CLK_CTRL11,
    R_PERI_CLK_CTRL12,
    R_PERI_CLK_CTRL13,
    R_PERI_CLK_CTRL14,
    R_PERI_CLK_CTRL15,
    R_PERI_CLK_CTRL16,
    R_PERI_CLK_CTRL17,
    R_PERI_CLK_CTRL18,
    R_PERI_CLK_CTRL19,
    R_PERI_CLK_CTRL110,
    R_PERI_CLK_CTRL111,
    R_PERI_CLK_CTRL112,
    R_PERI_CLK_CTRL113,
    R_PERI_CLK_CTRL114,
    R_PERI_CLK_CTRL115,
    R_PERI_CLK_CTRL116,
    R_PERI_CLK_CTRL117,
    R_PERI_CLK_CTRL118,
    R_PERI_CLK_CTRL119,
    R_PERI_CLK_CTRL120,
    R_PERI_CLK_CTRL20,
    R_PERI_CLK_CTRL21,
    R_PERI_CLK_CTRL22,
    R_PERI_CLK_CTRL23,
    R_PERI_CLK_CTRL24,
    R_PERI_CLK_CTRL25,
    R_PERI_CLK_CTRL26,
    R_PERI_CLK_CTRL27,
    R_CLK_FORCE_ON_CTRL0,
    R_DPA_CTRL0,
    R_ANA_PLL_CTRL0,
    R_HP_RST_EN0,
    R_HP_RST_EN1,
    R_HP_RST_EN2,
    R_HP_FORCE_NORST0,
    R_HP_FORCE_NORST1,
    R_HPWDT_CORE0_RST_CTRL0,
    R_HPWDT_CORE1_RST_CTRL0,
    R_CPU_SRC_FREQ0,
    R_CPU_CLK_STATUS0,
    R_DBG_CLK_CTRL0,
    R_DBG_CLK_CTRL1,
    R_HPCORE_WDT_RESET_SOURCE0,
    R_AXI_PERF_MON_CLKRST_CTRL0,
    R_CPU_WAITI_CTRL0,
    R_HP_END,

    R_LP_CLK_CONF = 64,
    R_LP_CLK_PO_EN,
    R_LP_CLK_EN,
    R_LP_RST_EN,
    R_LP_RESET_CAUSE,
    R_LP_HPCPU_RESET_CTRL0,
    R_LP_HPCPU_RESET_CTRL1,
    R_LP_FOSC_CNTL,
    R_LP_RC32K_CNTL,
    R_LP_SOSC_CNTL,
    R_LP_CLK_TO_HP,
    R_LP_LPMEM_FORCE,
    R_LP_XTAL32K,
    R_LP_MUX_HPSYS_RESET_BYPASS,
    R_LP_HPSYS_0_RESET_BYPASS,
    R_LP_HPSYS_APM_RESET_BYPASS,
    R_LP_HP_CLK_CTRL,
    R_LP_HP_USB_CLKRST_CTRL0,
    R_LP_HP_USB_CLKRST_CTRL1,
    R_LP_HP_SDMMC_EMAC_RST_CTRL,
    R_LP_DATE,

    R_LPPERI_CLK_EN = 88,
    R_LPPERI_CORE_CLK_SEL,
    R_LPPERI_RESET_EN,
    R_LPPERI_CPU,
    R_LPPERI_MEM_CTRL = 88 + 0x28 / 4,
    R_LPPERI_ADC_CTRL,
    R_LPPERI_I2S_RXCLK_DIV_NUM,
    R_LPPERI_I2S_RXCLK_DIV_XYZ,
    R_LPPERI_I2S_TXCLK_DIV_NUM,
    R_LPPERI_I2S_TXCLK_DIV_XYZ,
    R_LPPERI_DATE,
};

QEMU_BUILD_BUG_ON(R_HP_END != 0xf8 / 4);
QEMU_BUILD_BUG_ON(R_LP_HP_SDMMC_EMAC_RST_CTRL != 64 + 0x4c / 4);
QEMU_BUILD_BUG_ON(R_LPPERI_DATE + 1 != ESP32P4_CLKRST_NREGS);

/*
 * Each register's name, reset value and fields by access: stored as
 * written (R/W), acting when written with 1 and reading 0 (WT), and
 * driven by the hardware (RO), from the v3 headers.
 */
typedef struct {
    const char *name;
    uint32_t reset;
    uint32_t rw;
    uint32_t wt;
    uint32_t ro;
} ESP32P4ClkRstReg;

#define REG(r, n, rst, w, t, o) \
    [R_##r] = { .name = (n), .reset = (rst), .rw = (w), .wt = (t), .ro = (o) }

static const ESP32P4ClkRstReg esp32p4_clkrst_regs[ESP32P4_CLKRST_NREGS] = {
    REG(CLK_EN0, "HP_SYS_CLKRST_CLK_EN0", 0x00000001, 0x00000001, 0, 0),
    REG(ROOT_CLK_CTRL0, "HP_SYS_CLKRST_ROOT_CLK_CTRL0",
        0x00000000, 0x1fffffef, 0x00000010, 0),
    REG(ROOT_CLK_CTRL1, "HP_SYS_CLKRST_ROOT_CLK_CTRL1",
        0x00000001, 0xffffffff, 0, 0),
    REG(ROOT_CLK_CTRL2, "HP_SYS_CLKRST_ROOT_CLK_CTRL2",
        0x00010000, 0xffffffff, 0, 0),
    REG(ROOT_CLK_CTRL3, "HP_SYS_CLKRST_ROOT_CLK_CTRL3",
        0x00000000, 0x000000ff, 0, 0),
    REG(SOC_CLK_CTRL0, "HP_SYS_CLKRST_SOC_CLK_CTRL0",
        0xe6df97af, 0xffffffff, 0, 0),
    REG(SOC_CLK_CTRL1, "HP_SYS_CLKRST_SOC_CLK_CTRL1",
        0x7c7f001f, 0xffffffff, 0, 0),
    REG(SOC_CLK_CTRL2, "HP_SYS_CLKRST_SOC_CLK_CTRL2",
        0x20f80fde, 0xffffffff, 0, 0),
    REG(SOC_CLK_CTRL3, "HP_SYS_CLKRST_SOC_CLK_CTRL3",
        0x00000018, 0x0000001f, 0, 0),
    REG(REF_CLK_CTRL0, "HP_SYS_CLKRST_REF_CLK_CTRL0",
        0x02011309, 0xffffffff, 0, 0),
    REG(REF_CLK_CTRL1, "HP_SYS_CLKRST_REF_CLK_CTRL1",
        0x58170503, 0xffffffff, 0, 0),
    REG(REF_CLK_CTRL2, "HP_SYS_CLKRST_REF_CLK_CTRL2",
        0x00000115, 0x000003ff, 0, 0),
    REG(PERI_CLK_CTRL00, "HP_SYS_CLKRST_PERI_CLK_CTRL00",
        0x0000c03c, 0x3fffffff, 0, 0),
    REG(PERI_CLK_CTRL01, "HP_SYS_CLKRST_PERI_CLK_CTRL01",
        0x00000401, 0x01ffffff, 0, 0),
    REG(PERI_CLK_CTRL02, "HP_SYS_CLKRST_PERI_CLK_CTRL02",
        0x00000000, 0xfffffeff, 0x00000100, 0),
    REG(PERI_CLK_CTRL03, "HP_SYS_CLKRST_PERI_CLK_CTRL03",
        0x00000000, 0x0000ffff, 0, 0),
    REG(PERI_CLK_CTRL10, "HP_SYS_CLKRST_PERI_CLK_CTRL10",
        0x00000000, 0x0fffffff, 0, 0),
    REG(PERI_CLK_CTRL11, "HP_SYS_CLKRST_PERI_CLK_CTRL11",
        0x00000000, 0x07ffffff, 0, 0),
    REG(PERI_CLK_CTRL12, "HP_SYS_CLKRST_PERI_CLK_CTRL12",
        0x00000000, 0x03ffffff, 0, 0),
    REG(PERI_CLK_CTRL13, "HP_SYS_CLKRST_PERI_CLK_CTRL13",
        0x00000000, 0x3fffffff, 0, 0),
    REG(PERI_CLK_CTRL14, "HP_SYS_CLKRST_PERI_CLK_CTRL14",
        0x00000000, 0x7fffffff, 0, 0),
    REG(PERI_CLK_CTRL15, "HP_SYS_CLKRST_PERI_CLK_CTRL15",
        0x00000000, 0x7fffffff, 0, 0),
    REG(PERI_CLK_CTRL16, "HP_SYS_CLKRST_PERI_CLK_CTRL16",
        0x00000000, 0x03ffffff, 0, 0),
    REG(PERI_CLK_CTRL17, "HP_SYS_CLKRST_PERI_CLK_CTRL17",
        0x00000000, 0x7fffffff, 0, 0),
    REG(PERI_CLK_CTRL18, "HP_SYS_CLKRST_PERI_CLK_CTRL18",
        0x00000000, 0x3fffffff, 0, 0),
    REG(PERI_CLK_CTRL19, "HP_SYS_CLKRST_PERI_CLK_CTRL19",
        0x00000000, 0xffffffff, 0, 0),
    REG(PERI_CLK_CTRL110, "HP_SYS_CLKRST_PERI_CLK_CTRL110",
        0x04000000, 0x07ffffff, 0, 0),
    REG(PERI_CLK_CTRL111, "HP_SYS_CLKRST_PERI_CLK_CTRL111",
        0x04000000, 0x07ffffff, 0, 0),
    REG(PERI_CLK_CTRL112, "HP_SYS_CLKRST_PERI_CLK_CTRL112",
        0x04000000, 0x07ffffff, 0, 0),
    REG(PERI_CLK_CTRL113, "HP_SYS_CLKRST_PERI_CLK_CTRL113",
        0x04000000, 0x07ffffff, 0, 0),
    REG(PERI_CLK_CTRL114, "HP_SYS_CLKRST_PERI_CLK_CTRL114",
        0x04000000, 0x07ffffff, 0, 0),
    REG(PERI_CLK_CTRL115, "HP_SYS_CLKRST_PERI_CLK_CTRL115",
        0x00000000, 0x3fffffff, 0, 0),
    REG(PERI_CLK_CTRL116, "HP_SYS_CLKRST_PERI_CLK_CTRL116",
        0x01100008, 0x01ffffff, 0, 0),
    REG(PERI_CLK_CTRL117, "HP_SYS_CLKRST_PERI_CLK_CTRL117",
        0x00010000, 0x0fffffff, 0, 0),
    REG(PERI_CLK_CTRL118, "HP_SYS_CLKRST_PERI_CLK_CTRL118",
        0x00000000, 0x07ffffff, 0, 0),
    REG(PERI_CLK_CTRL119, "HP_SYS_CLKRST_PERI_CLK_CTRL119",
        0x00000000, 0x3fffffff, 0, 0),
    REG(PERI_CLK_CTRL120, "HP_SYS_CLKRST_PERI_CLK_CTRL120",
        0x00000000, 0x00ffffff, 0, 0),
    REG(PERI_CLK_CTRL20, "HP_SYS_CLKRST_PERI_CLK_CTRL20",
        0xc9000000, 0xffffffff, 0, 0),
    REG(PERI_CLK_CTRL21, "HP_SYS_CLKRST_PERI_CLK_CTRL21",
        0x52400000, 0x7fffffff, 0, 0),
    REG(PERI_CLK_CTRL22, "HP_SYS_CLKRST_PERI_CLK_CTRL22",
        0x00000000, 0xffffffff, 0, 0),
    REG(PERI_CLK_CTRL23, "HP_SYS_CLKRST_PERI_CLK_CTRL23",
        0x00000008, 0x01ffffff, 0, 0),
    REG(PERI_CLK_CTRL24, "HP_SYS_CLKRST_PERI_CLK_CTRL24",
        0x00000404, 0x01ffffff, 0, 0),
    REG(PERI_CLK_CTRL25, "HP_SYS_CLKRST_PERI_CLK_CTRL25",
        0x007fc000, 0x03ffffff, 0, 0),
    REG(PERI_CLK_CTRL26, "HP_SYS_CLKRST_PERI_CLK_CTRL26",
        0x00000200, 0x3fffffff, 0, 0),
    REG(PERI_CLK_CTRL27, "HP_SYS_CLKRST_PERI_CLK_CTRL27",
        0x00000000, 0x0003ffff, 0, 0),
    REG(CLK_FORCE_ON_CTRL0, "HP_SYS_CLKRST_CLK_FORCE_ON_CTRL0",
        0x0007ffff, 0x0007ffff, 0, 0),
    REG(DPA_CTRL0, "HP_SYS_CLKRST_DPA_CTRL0", 0x00000000, 0x00000007, 0, 0),
    REG(ANA_PLL_CTRL0, "HP_SYS_CLKRST_ANA_PLL_CTRL0",
        0x00000000, 0x000002aa, 0, 0x00000155),
    REG(HP_RST_EN0, "HP_SYS_CLKRST_HP_RST_EN0", 0x00000100, 0xffffffff, 0, 0),
    REG(HP_RST_EN1, "HP_SYS_CLKRST_HP_RST_EN1", 0x00000000, 0xffffffff, 0, 0),
    REG(HP_RST_EN2, "HP_SYS_CLKRST_HP_RST_EN2", 0x00000000, 0x01ffffff, 0, 0),
    REG(HP_FORCE_NORST0, "HP_SYS_CLKRST_HP_FORCE_NORST0",
        0x00000000, 0xffffffff, 0, 0),
    REG(HP_FORCE_NORST1, "HP_SYS_CLKRST_HP_FORCE_NORST1",
        0x00000000, 0x07ffffff, 0, 0),
    REG(HPWDT_CORE0_RST_CTRL0, "HP_SYS_CLKRST_HPWDT_CORE0_RST_CTRL0",
        0x00001011, 0x0001ffff, 0, 0),
    REG(HPWDT_CORE1_RST_CTRL0, "HP_SYS_CLKRST_HPWDT_CORE1_RST_CTRL0",
        0x00001011, 0x0001ffff, 0, 0),
    REG(CPU_SRC_FREQ0, "HP_SYS_CLKRST_CPU_SRC_FREQ0", 0, 0, 0, 0xffffffff),
    REG(CPU_CLK_STATUS0, "HP_SYS_CLKRST_CPU_CLK_STATUS0", 0, 0, 0, 0x07ffffff),
    REG(DBG_CLK_CTRL0, "HP_SYS_CLKRST_DBG_CLK_CTRL0",
        0x03ffffff, 0xffffffff, 0, 0),
    REG(DBG_CLK_CTRL1, "HP_SYS_CLKRST_DBG_CLK_CTRL1",
        0x00000303, 0x0007ffff, 0, 0),
    REG(HPCORE_WDT_RESET_SOURCE0, "HP_SYS_CLKRST_HPCORE_WDT_RESET_SOURCE0",
        0x00000002, 0x00000003, 0, 0),
    REG(AXI_PERF_MON_CLKRST_CTRL0, "HP_SYS_CLKRST_AXI_PERF_MON_CLKRST_CTRL0",
        0x00000000, 0x00000003, 0, 0),
    REG(CPU_WAITI_CTRL0, "HP_SYS_CLKRST_CPU_WAITI_CTRL0",
        0x00000003, 0x00000003, 0, 0),

    REG(LP_CLK_CONF, "LP_CLKRST_LP_CLK_CONF", 0x00000004, 0x000007ff, 0, 0),
    REG(LP_CLK_PO_EN, "LP_CLKRST_LP_CLK_PO_EN", 0x00000000, 0x000003ff, 0, 0),
    REG(LP_CLK_EN, "LP_CLKRST_LP_CLK_EN", 0x08000000, 0xfc000000, 0, 0),
    REG(LP_RST_EN, "LP_CLKRST_LP_RST_EN", 0x00000000, 0xff000000, 0, 0),
    REG(LP_RESET_CAUSE, "LP_CLKRST_RESET_CAUSE",
        0x02000000, 0x02000000, 0xfc000000, 0x001fffff),
    REG(LP_HPCPU_RESET_CTRL0, "LP_CLKRST_HPCPU_RESET_CTRL0",
        0x80028002, 0xdfffdfff, 0x20002000, 0),
    REG(LP_HPCPU_RESET_CTRL1, "LP_CLKRST_HPCPU_RESET_CTRL1",
        0x00000000, 0xffff0000, 0, 0),
    REG(LP_FOSC_CNTL, "LP_CLKRST_FOSC_CNTL", 0x64000000, 0xffc00000, 0, 0),
    REG(LP_RC32K_CNTL, "LP_CLKRST_RC32K_CNTL", 0x0000028a, 0xffffffff, 0, 0),
    REG(LP_SOSC_CNTL, "LP_CLKRST_SOSC_CNTL", 0x2b000000, 0xffc00000, 0, 0),
    REG(LP_CLK_TO_HP, "LP_CLKRST_CLK_TO_HP", 0xf0000000, 0xf0000000, 0, 0),
    REG(LP_LPMEM_FORCE, "LP_CLKRST_LPMEM_FORCE", 0x00000000, 0x80000000, 0, 0),
    REG(LP_XTAL32K, "LP_CLKRST_XTAL32K", 0x66c00000, 0xffc00000, 0, 0),
    REG(LP_MUX_HPSYS_RESET_BYPASS, "LP_CLKRST_MUX_HPSYS_RESET_BYPASS",
        0xffffffff, 0xffffffff, 0, 0),
    REG(LP_HPSYS_0_RESET_BYPASS, "LP_CLKRST_HPSYS_0_RESET_BYPASS",
        0xffffffff, 0xffffffff, 0, 0),
    REG(LP_HPSYS_APM_RESET_BYPASS, "LP_CLKRST_HPSYS_APM_RESET_BYPASS",
        0xffffffff, 0xffffffff, 0, 0),
    REG(LP_HP_CLK_CTRL, "LP_CLKRST_HP_CLK_CTRL", 0x1ffffffc, 0x1fffffff, 0, 0),
    REG(LP_HP_USB_CLKRST_CTRL0, "LP_CLKRST_HP_USB_CLKRST_CTRL0",
        0x09c4c27a, 0x3fffffff, 0, 0),
    REG(LP_HP_USB_CLKRST_CTRL1, "LP_CLKRST_HP_USB_CLKRST_CTRL1",
        0xc0000000, 0xf000001f, 0, 0),
    REG(LP_HP_SDMMC_EMAC_RST_CTRL, "LP_CLKRST_HP_SDMMC_EMAC_RST_CTRL",
        0x00000000, 0xf0000000, 0, 0),
    REG(LP_DATE, "LP_CLKRST_DATE", 0x00000000, 0x80000000, 0, 0),

    REG(LPPERI_CLK_EN, "LPPERI_CLK_EN", 0x7fff0000, 0xffff0000, 0, 0),
    REG(LPPERI_CORE_CLK_SEL, "LPPERI_CORE_CLK_SEL",
        0x00000000, 0xff000000, 0, 0),
    REG(LPPERI_RESET_EN, "LPPERI_RESET_EN",
        0x00000000, 0x7ffe0000, 0x80000000, 0),
    REG(LPPERI_CPU, "LPPERI_CPU", 0x80000000, 0x80000000, 0, 0),
    REG(LPPERI_MEM_CTRL, "LPPERI_MEM_CTRL",
        0x80000000, 0xe0000000, 0x00000001, 0x00000002),
    REG(LPPERI_ADC_CTRL, "LPPERI_ADC_CTRL", 0x04040400, 0xffffffc0, 0, 0),
    REG(LPPERI_I2S_RXCLK_DIV_NUM, "LPPERI_LP_I2S_RXCLK_DIV_NUM",
        0x02000000, 0xff000000, 0, 0),
    REG(LPPERI_I2S_RXCLK_DIV_XYZ, "LPPERI_LP_I2S_RXCLK_DIV_XYZ",
        0x00004000, 0xfffffff0, 0, 0),
    REG(LPPERI_I2S_TXCLK_DIV_NUM, "LPPERI_LP_I2S_TXCLK_DIV_NUM",
        0x02000000, 0xff000000, 0, 0),
    REG(LPPERI_I2S_TXCLK_DIV_XYZ, "LPPERI_LP_I2S_TXCLK_DIV_XYZ",
        0x00004000, 0xfffffff0, 0, 0),
    REG(LPPERI_DATE, "LPPERI_DATE", 0x00000000, 0x80000000, 0, 0),
};

/* Fields of the registers this file acts on. */
#define SOC_CLK_DIV_UPDATE          BIT(4)
#define SDIO_LS_CLK_EDGE_CFG_UPDATE BIT(8)
#define L2MEM_MEM_CLK_FORCE_ON      BIT(14)
#define CORE_CPU_CLK_EN(n)          BIT(3 + (n))
#define RST_EN_CORE_GLOBAL(n)       BIT(7 + (n))
#define FORCE_NORST_CORE(n)         BIT(n)
#define HPCORE_SHIFT(n)             (16 * (n))
#define HPCORE_LOCKUP_RESET_EN      BIT(0)
#define HPCORE_RESET_LENGTH(v)      extract32((v), 1, 3)
#define HPCORE_LP_WDT_RESET_EN      BIT(4)
#define HPCORE_STALL_WAIT(v)        extract32((v), 5, 7)
#define HPCORE_STALL_EN             BIT(12)
#define HPCORE_SW_RESET             BIT(13)
#define HPCORE_OCD_HALT_ON_RESET    BIT(14)
#define HPCORE_STAT_VECTOR_SEL      BIT(15)
#define HPWDT_STALL_EN              BIT(0)
#define HPWDT_STALL_WAIT_NUM(v)     extract32((v), 1, 8)
#define HPWDT_RST_LEN(v)            extract32((v), 9, 8)
#define LPCORE_CAUSE_MASK_PMU       BIT(25)
#define RST_EN_LP_CORE              BIT(31)
#define CK_EN_LP_CORE               BIT(31)
#define LP_UART_WAKEUP_FLAG_CLR     BIT(0)
#define LP_UART_WAKEUP_FLAG         BIT(1)
#define LP_UART_WAKEUP_EN           BIT(29)
#define ROOT_CLK_SRC_CPLL           1
#define CAUSE_PMU_LP_CPU            0x0a

/* The cause and flag fields of each core in LP_CLKRST_RESET_CAUSE. */
static const struct {
    uint8_t cause_shift;
    uint8_t flag_bit;
    uint8_t cause_clr;
    uint8_t flag_clr;
} esp32p4_clkrst_cause_fields[3] = {
    { 7, 13, 28, 29 },      /* HP core 0 */
    { 14, 20, 30, 31 },     /* HP core 1 */
    { 0, 6, 26, 27 },       /* the LP core */
};

/*
 * The clock tree. Each clock is its parent, which a mux may select, gated
 * by up to three enable bits and divided; or, a SPECIAL one, computed
 * apart. A parent is another clock, before it in the table, an input
 * (PIN) or none (PNONE), a stopped clock.
 */
typedef struct {
    uint8_t reg;
    uint8_t shift;
    uint8_t width;
} ClkField;

enum { DIV_NONE, DIV_INT, DIV_FRAC, DIV_I2S, DIV_FIXED, SPECIAL };

#define PARENT_IN   0x4000
#define PARENT_NONE 0x7fff
#define MAX_PARENTS 16

typedef struct {
    int16_t parent[MAX_PARENTS];
    uint8_t nparents;
    uint8_t kind;
    uint8_t fixed;
    ClkField sel;
    ClkField gate[3];
    /* Integer, numerator, denominator; or I2S's N, X, Y, Z and YN1. */
    ClkField div[5];
} ClkDesc;

#define F(r, s, w)  { R_##r, (s), (w) }
#define B(r, s)     F(r, s, 1)
#define C(x)        ESP32P4_CLK_##x
#define PIN(x)      (PARENT_IN | ESP32P4_CLKIN_##x)
#define PNONE       PARENT_NONE
#define PARENTS(...) \
    .parent = { __VA_ARGS__ }, \
    .nparents = sizeof((const int16_t[]){ __VA_ARGS__ }) / sizeof(int16_t)

#define GATE(id, par, r, b) \
    [C(id)] = { PARENTS(C(par)), .gate = { B(r, b) } }
#define LPPERI_GATE(id, b) \
    [C(id)] = { PARENTS(C(LP_PERI)), .gate = { B(LPPERI_CLK_EN, b) } }
#define FRAC(r0, s0, r1, s1, r2, s2) \
    .kind = DIV_FRAC, .div = { F(r0, s0, 8), F(r1, s1, 8), F(r2, s2, 8) }
#define I2S(rn, sn, rx, sx, ry, sy, rz, sz, ryn, syn) \
    .kind = DIV_I2S, \
    .div = { F(rn, sn, 8), F(rx, sx, 9), F(ry, sy, 9), F(rz, sz, 9), \
             B(ryn, syn) }
#define INTDIV(r, s, w) .kind = DIV_INT, .div = { F(r, s, w) }

#define PARENTS_XRF80 PARENTS(C(XTAL), C(RC_FAST), C(PLL_F80M), PNONE)

static const ClkDesc esp32p4_clk_desc[ESP32P4_NUM_CLOCKS] = {
    [C(OSC_XTAL)] = { .kind = SPECIAL },
    [C(OSC_RC_FAST)] = { .kind = SPECIAL },
    [C(OSC_RC_SLOW)] = { .kind = SPECIAL },
    [C(OSC_RC32K)] = { .kind = SPECIAL },
    [C(OSC_XTAL32K)] = { .kind = SPECIAL },
    [C(OSC_SLOW)] = { PARENTS(PIN(PAD_OSC_SLOW_IN)) },
    [C(OSC_PLL_LP)] = { .kind = SPECIAL },

    [C(XTAL)] = { PARENTS(C(OSC_XTAL)), .gate = { B(LP_HP_CLK_CTRL, 25) } },
    [C(RC_FAST)] = { PARENTS(C(OSC_RC_FAST)),
                     .gate = { B(LP_HP_CLK_CTRL, 24), B(LP_CLK_TO_HP, 31) } },
    [C(RC_SLOW)] = { PARENTS(C(OSC_RC_SLOW)),
                     .gate = { B(LP_HP_CLK_CTRL, 18), B(LP_CLK_TO_HP, 29) } },
    [C(RC32K)] = { PARENTS(C(OSC_RC32K)),
                   .gate = { B(LP_HP_CLK_CTRL, 17), B(LP_CLK_TO_HP, 30) } },
    [C(XTAL32K)] = { PARENTS(C(OSC_XTAL32K)),
                     .gate = { B(LP_HP_CLK_CTRL, 16), B(LP_CLK_TO_HP, 28) } },
    [C(PLL_LP)] = { PARENTS(C(OSC_PLL_LP)),
                    .gate = { B(LP_HP_CLK_CTRL, 19) } },
    [C(CPLL)] = { PARENTS(PIN(CPLL_IN)), .gate = { B(LP_HP_CLK_CTRL, 26) } },
    [C(SPLL)] = { PARENTS(PIN(SPLL_IN)), .gate = { B(LP_HP_CLK_CTRL, 27) } },
    [C(MPLL)] = { PARENTS(PIN(MPLL_IN)), .gate = { B(LP_HP_CLK_CTRL, 28) } },
    [C(APLL)] = { PARENTS(PIN(APLL_IN)), .gate = { B(LP_HP_CLK_CTRL, 20) } },
    [C(SDIO_PLL0)] = { PARENTS(PIN(SDIO_PLL_IN)),
                       .gate = { B(LP_HP_CLK_CTRL, 23) } },
    [C(SDIO_PLL1)] = { PARENTS(PIN(SDIO_PLL_IN)),
                       .gate = { B(LP_HP_CLK_CTRL, 22) } },
    [C(SDIO_PLL2)] = { PARENTS(PIN(SDIO_PLL_IN)),
                       .gate = { B(LP_HP_CLK_CTRL, 21) } },
    [C(PAD_EMAC_TXRX)] = { PARENTS(PIN(PAD_EMAC_TXRX_IN)),
                           .gate = { B(LP_HP_CLK_CTRL, 15) } },
    [C(PAD_EMAC_RX)] = { PARENTS(PIN(PAD_EMAC_RX_IN)),
                         .gate = { B(LP_HP_CLK_CTRL, 14) } },
    [C(PAD_EMAC_TX)] = { PARENTS(PIN(PAD_EMAC_TX_IN)),
                         .gate = { B(LP_HP_CLK_CTRL, 13) } },
    [C(PAD_I2S0_MCLK)] = { PARENTS(PIN(PAD_I2S0_MCLK_IN)),
                           .gate = { B(LP_HP_CLK_CTRL, 12) } },
    [C(PAD_I2S1_MCLK)] = { PARENTS(PIN(PAD_I2S1_MCLK_IN)),
                           .gate = { B(LP_HP_CLK_CTRL, 11) } },
    [C(PAD_I2S2_MCLK)] = { PARENTS(PIN(PAD_I2S2_MCLK_IN)),
                           .gate = { B(LP_HP_CLK_CTRL, 10) } },
    [C(PAD_UART0_SLP)] = { PARENTS(PIN(PAD_UART0_SLP_IN)),
                           .gate = { B(LP_HP_CLK_CTRL, 9) } },
    [C(PAD_UART1_SLP)] = { PARENTS(PIN(PAD_UART1_SLP_IN)),
                           .gate = { B(LP_HP_CLK_CTRL, 8) } },
    [C(PAD_UART2_SLP)] = { PARENTS(PIN(PAD_UART2_SLP_IN)),
                           .gate = { B(LP_HP_CLK_CTRL, 7) } },
    [C(PAD_UART3_SLP)] = { PARENTS(PIN(PAD_UART3_SLP_IN)),
                           .gate = { B(LP_HP_CLK_CTRL, 6) } },
    [C(PAD_UART4_SLP)] = { PARENTS(PIN(PAD_UART4_SLP_IN)),
                           .gate = { B(LP_HP_CLK_CTRL, 5) } },
    [C(PAD_PARLIO_RX)] = { PARENTS(PIN(PAD_PARLIO_RX_IN)),
                           .gate = { B(LP_HP_CLK_CTRL, 4) } },
    [C(PAD_PARLIO_TX)] = { PARENTS(PIN(PAD_PARLIO_TX_IN)),
                           .gate = { B(LP_HP_CLK_CTRL, 3) } },

    [C(ROOT)] = { PARENTS(C(XTAL), C(CPLL), C(RC_FAST), PNONE),
                  .sel = F(LP_HP_CLK_CTRL, 0, 2),
                  .gate = { B(LP_HP_CLK_CTRL, 2) } },
    [C(CPU)] = { .kind = SPECIAL },
    [C(MEM)] = { .kind = SPECIAL },
    [C(SYS)] = { .kind = SPECIAL },
    [C(APB)] = { .kind = SPECIAL },

    [C(PLL_F240M)] = { PARENTS(C(SPLL)), .gate = { B(REF_CLK_CTRL1, 30) },
                       INTDIV(REF_CLK_CTRL0, 16, 8) },
    [C(PLL_F160M)] = { PARENTS(C(SPLL)), .gate = { B(REF_CLK_CTRL2, 0) },
                       INTDIV(REF_CLK_CTRL0, 24, 8) },
    [C(PLL_F120M)] = { PARENTS(C(SPLL)), .gate = { B(REF_CLK_CTRL2, 2) },
                       INTDIV(REF_CLK_CTRL1, 0, 8) },
    [C(PLL_F80M)] = { PARENTS(C(SPLL)), .gate = { B(REF_CLK_CTRL2, 4) },
                      INTDIV(REF_CLK_CTRL1, 8, 8) },
    [C(PLL_F20M)] = { PARENTS(C(SPLL)), .gate = { B(REF_CLK_CTRL2, 8) },
                      INTDIV(REF_CLK_CTRL1, 16, 8) },
    [C(PLL_F50M)] = { PARENTS(C(MPLL)), .gate = { B(REF_CLK_CTRL1, 27) },
                      INTDIV(REF_CLK_CTRL0, 0, 8) },
    [C(PLL_F25M)] = { PARENTS(C(MPLL)), .gate = { B(REF_CLK_CTRL1, 28) },
                      INTDIV(REF_CLK_CTRL0, 8, 8) },
    [C(PLL_F48M)] = { PARENTS(C(SPLL)),
                      INTDIV(LP_HP_USB_CLKRST_CTRL0, 6, 8) },
    [C(PLL_F12M)] = { PARENTS(C(SPLL)),
                      INTDIV(LP_HP_USB_CLKRST_CTRL0, 22, 8) },
    [C(USB_PLL_F25M)] = { PARENTS(C(MPLL)),
                          INTDIV(LP_HP_USB_CLKRST_CTRL0, 14, 8) },

    [C(LP_SLOW)] = { PARENTS(C(OSC_RC_SLOW), C(OSC_XTAL32K), C(OSC_RC32K),
                             C(OSC_SLOW)),
                     .sel = F(LP_CLK_CONF, 0, 2) },
    [C(LP_FAST)] = { PARENTS(C(OSC_RC_FAST), C(OSC_XTAL), C(OSC_PLL_LP),
                             PNONE),
                     .sel = F(LP_CLK_CONF, 2, 2) },
    [C(LP_DYN_SLOW)] = { PARENTS(C(LP_SLOW)) },
    [C(LP_DYN_FAST)] = { .kind = SPECIAL },
    [C(LP_PERI)] = { PARENTS(C(LP_DYN_FAST)), INTDIV(LP_CLK_CONF, 4, 6) },
    [C(XTAL_D2)] = { PARENTS(C(OSC_XTAL)), .kind = DIV_FIXED, .fixed = 2 },

    GATE(CORE0_CLIC, CPU, SOC_CLK_CTRL0, 0),
    GATE(CORE1_CLIC, CPU, SOC_CLK_CTRL0, 1),
    GATE(MISC_CPU, CPU, SOC_CLK_CTRL0, 2),
    GATE(CORE0_CPU, CPU, SOC_CLK_CTRL0, 3),
    GATE(CORE1_CPU, CPU, SOC_CLK_CTRL0, 4),
    GATE(TCM_CPU, CPU, SOC_CLK_CTRL0, 5),
    GATE(BUSMON_CPU, CPU, SOC_CLK_CTRL0, 6),
    GATE(L1CACHE_CPU, CPU, SOC_CLK_CTRL0, 7),
    GATE(L1CACHE_D_CPU, CPU, SOC_CLK_CTRL0, 8),
    GATE(L1CACHE_I0_CPU, CPU, SOC_CLK_CTRL0, 9),
    GATE(L1CACHE_I1_CPU, CPU, SOC_CLK_CTRL0, 10),
    GATE(TRACE_CPU, CPU, SOC_CLK_CTRL0, 11),
    GATE(ICM_CPU, CPU, SOC_CLK_CTRL0, 12),
    GATE(GDMA_CPU, CPU, SOC_CLK_CTRL0, 13),
    GATE(VPU_CPU, CPU, SOC_CLK_CTRL0, 14),
    GATE(L1CACHE_MEM, MEM, SOC_CLK_CTRL0, 15),
    GATE(L1CACHE_D_MEM, MEM, SOC_CLK_CTRL0, 16),
    GATE(L1CACHE_I0_MEM, MEM, SOC_CLK_CTRL0, 17),
    GATE(L1CACHE_I1_MEM, MEM, SOC_CLK_CTRL0, 18),
    GATE(L2CACHE_MEM, MEM, SOC_CLK_CTRL0, 19),
    GATE(L2MEM_MEM, MEM, SOC_CLK_CTRL0, 20),
    GATE(L2MEMMON_MEM, MEM, SOC_CLK_CTRL0, 21),
    GATE(ICM_MEM, MEM, SOC_CLK_CTRL0, 22),
    GATE(MISC_SYS, SYS, SOC_CLK_CTRL0, 23),
    GATE(TRACE_SYS, SYS, SOC_CLK_CTRL0, 24),
    GATE(L2CACHE_SYS, SYS, SOC_CLK_CTRL0, 25),
    GATE(L2MEM_SYS, SYS, SOC_CLK_CTRL0, 26),
    GATE(L2MEMMON_SYS, SYS, SOC_CLK_CTRL0, 27),
    GATE(TCMMON_SYS, SYS, SOC_CLK_CTRL0, 28),
    GATE(ICM_SYS, SYS, SOC_CLK_CTRL0, 29),
    GATE(FLASH_SYS, SYS, SOC_CLK_CTRL0, 30),
    GATE(PSRAM_SYS, SYS, SOC_CLK_CTRL0, 31),

    GATE(GPSPI2_SYS, SYS, SOC_CLK_CTRL1, 0),
    GATE(GPSPI3_SYS, SYS, SOC_CLK_CTRL1, 1),
    GATE(REGDMA_SYS, SYS, SOC_CLK_CTRL1, 2),
    GATE(AHB_PDMA_SYS, SYS, SOC_CLK_CTRL1, 3),
    GATE(AXI_PDMA_SYS, SYS, SOC_CLK_CTRL1, 4),
    GATE(GDMA_SYS, SYS, SOC_CLK_CTRL1, 5),
    GATE(DMA2D_SYS, SYS, SOC_CLK_CTRL1, 6),
    GATE(VPU_SYS, SYS, SOC_CLK_CTRL1, 7),
    GATE(JPEG_SYS, SYS, SOC_CLK_CTRL1, 8),
    GATE(PPA_SYS, SYS, SOC_CLK_CTRL1, 9),
    GATE(CSI_BRG_SYS, SYS, SOC_CLK_CTRL1, 10),
    GATE(CSI_HOST_SYS, SYS, SOC_CLK_CTRL1, 11),
    GATE(DSI_SYS, SYS, SOC_CLK_CTRL1, 12),
    GATE(EMAC_SYS, SYS, SOC_CLK_CTRL1, 13),
    GATE(SDMMC_SYS, SYS, SOC_CLK_CTRL1, 14),
    GATE(USB_OTG11_SYS, SYS, SOC_CLK_CTRL1, 15),
    GATE(USB_OTG20_SYS, SYS, SOC_CLK_CTRL1, 16),
    GATE(UHCI_SYS, SYS, SOC_CLK_CTRL1, 17),
    GATE(UART0_SYS, SYS, SOC_CLK_CTRL1, 18),
    GATE(UART1_SYS, SYS, SOC_CLK_CTRL1, 19),
    GATE(UART2_SYS, SYS, SOC_CLK_CTRL1, 20),
    GATE(UART3_SYS, SYS, SOC_CLK_CTRL1, 21),
    GATE(UART4_SYS, SYS, SOC_CLK_CTRL1, 22),
    GATE(PARLIO_SYS, SYS, SOC_CLK_CTRL1, 23),
    GATE(ETM_SYS, SYS, SOC_CLK_CTRL1, 24),
    GATE(PVT_SYS, SYS, SOC_CLK_CTRL1, 25),
    GATE(CRYPTO_SYS, SYS, SOC_CLK_CTRL1, 26),
    GATE(KEY_MANAGER_SYS, SYS, SOC_CLK_CTRL1, 27),
    GATE(BITSCRAMBLER_SYS, SYS, SOC_CLK_CTRL1, 28),
    GATE(BITSCRAMBLER_RX_SYS, SYS, SOC_CLK_CTRL1, 29),
    GATE(BITSCRAMBLER_TX_SYS, SYS, SOC_CLK_CTRL1, 30),
    GATE(H264_SYS, SYS, SOC_CLK_CTRL1, 31),

    GATE(RMT_SYS, SYS, SOC_CLK_CTRL2, 0),
    GATE(HP_CLKRST_APB, APB, SOC_CLK_CTRL2, 1),
    GATE(SYSREG_APB, APB, SOC_CLK_CTRL2, 2),
    GATE(ICM_APB, APB, SOC_CLK_CTRL2, 3),
    GATE(INTRMTX_APB, APB, SOC_CLK_CTRL2, 4),
    GATE(ADC_APB, APB, SOC_CLK_CTRL2, 5),
    GATE(UHCI_APB, APB, SOC_CLK_CTRL2, 6),
    GATE(UART0_APB, APB, SOC_CLK_CTRL2, 7),
    GATE(UART1_APB, APB, SOC_CLK_CTRL2, 8),
    GATE(UART2_APB, APB, SOC_CLK_CTRL2, 9),
    GATE(UART3_APB, APB, SOC_CLK_CTRL2, 10),
    GATE(UART4_APB, APB, SOC_CLK_CTRL2, 11),
    GATE(I2C0_APB, APB, SOC_CLK_CTRL2, 12),
    GATE(I2C1_APB, APB, SOC_CLK_CTRL2, 13),
    GATE(I2S0_APB, APB, SOC_CLK_CTRL2, 14),
    GATE(I2S1_APB, APB, SOC_CLK_CTRL2, 15),
    GATE(I2S2_APB, APB, SOC_CLK_CTRL2, 16),
    GATE(I3C_MST_APB, APB, SOC_CLK_CTRL2, 17),
    GATE(I3C_SLV_APB, APB, SOC_CLK_CTRL2, 18),
    GATE(GPSPI2_APB, APB, SOC_CLK_CTRL2, 19),
    GATE(GPSPI3_APB, APB, SOC_CLK_CTRL2, 20),
    GATE(TIMERGRP0_APB, APB, SOC_CLK_CTRL2, 21),
    GATE(TIMERGRP1_APB, APB, SOC_CLK_CTRL2, 22),
    GATE(SYSTIMER_APB, APB, SOC_CLK_CTRL2, 23),
    GATE(TWAI0_APB, APB, SOC_CLK_CTRL2, 24),
    GATE(TWAI1_APB, APB, SOC_CLK_CTRL2, 25),
    GATE(TWAI2_APB, APB, SOC_CLK_CTRL2, 26),
    GATE(MCPWM0_APB, APB, SOC_CLK_CTRL2, 27),
    GATE(MCPWM1_APB, APB, SOC_CLK_CTRL2, 28),
    GATE(USB_DEVICE_APB, APB, SOC_CLK_CTRL2, 29),
    GATE(PCNT_APB, APB, SOC_CLK_CTRL2, 30),
    GATE(PARLIO_APB, APB, SOC_CLK_CTRL2, 31),

    GATE(LEDC_APB, APB, SOC_CLK_CTRL3, 0),
    GATE(LCDCAM_APB, APB, SOC_CLK_CTRL3, 1),
    GATE(ETM_APB, APB, SOC_CLK_CTRL3, 2),
    GATE(IOMUX_APB, APB, SOC_CLK_CTRL3, 3),
    GATE(L2CACHE_L2MEM, MEM, SOC_CLK_CTRL3, 4),
    GATE(AXI_PERF_MON_SYS, SYS, AXI_PERF_MON_CLKRST_CTRL0, 0),

    [C(FLASH)] = { PARENTS(C(XTAL), C(SPLL), C(CPLL), PNONE),
                   .sel = F(PERI_CLK_CTRL00, 0, 2) },
    [C(FLASH_PLL)] = { PARENTS(C(FLASH)),
                       .gate = { B(PERI_CLK_CTRL00, 2) } },
    [C(FLASH_CORE)] = { PARENTS(C(FLASH)), .gate = { B(PERI_CLK_CTRL00, 3) },
                        INTDIV(PERI_CLK_CTRL00, 4, 8) },
    [C(PSRAM)] = { PARENTS(C(XTAL), C(MPLL), C(SPLL), C(CPLL)),
                   .sel = F(PERI_CLK_CTRL00, 12, 2) },
    [C(PSRAM_PLL)] = { PARENTS(C(PSRAM)),
                       .gate = { B(PERI_CLK_CTRL00, 14) } },
    [C(PSRAM_CORE)] = { PARENTS(C(PSRAM)),
                        .gate = { B(PERI_CLK_CTRL00, 15) },
                        INTDIV(PERI_CLK_CTRL00, 16, 8) },
    [C(PAD_EMAC_REF)] = { PARENTS(C(PLL_F50M)),
                          .gate = { B(PERI_CLK_CTRL00, 24) } },
    [C(EMAC_RMII)] = { PARENTS(C(PAD_EMAC_TXRX), C(PAD_EMAC_RX),
                               C(PAD_EMAC_TX), PNONE),
                       .sel = F(PERI_CLK_CTRL00, 25, 2),
                       .gate = { B(PERI_CLK_CTRL00, 27) } },
    [C(EMAC_RX)] = { PARENTS(C(PAD_EMAC_TXRX), C(PAD_EMAC_RX)),
                     .sel = B(PERI_CLK_CTRL00, 28),
                     .gate = { B(PERI_CLK_CTRL00, 29) },
                     INTDIV(PERI_CLK_CTRL01, 0, 8) },
    [C(EMAC_TX)] = { PARENTS(C(PAD_EMAC_TXRX), C(PAD_EMAC_TX)),
                     .sel = B(PERI_CLK_CTRL01, 8),
                     .gate = { B(PERI_CLK_CTRL01, 9) },
                     INTDIV(PERI_CLK_CTRL01, 10, 8) },
    [C(EMAC_PTP_REF)] = { PARENTS(C(XTAL), C(PLL_F80M)),
                          .sel = B(PERI_CLK_CTRL01, 18),
                          .gate = { B(PERI_CLK_CTRL01, 19) } },
    [C(SDIO_LS)] = { .kind = SPECIAL },
    [C(SDIO_SLF)] = { .kind = SPECIAL },
    [C(SDIO_DRV)] = { .kind = SPECIAL },
    [C(SDIO_SAM)] = { .kind = SPECIAL },
    [C(MIPI_DSI_DPHY_CFG)] = { PARENTS(C(PLL_F20M), C(RC_FAST), C(PLL_F25M),
                                       PNONE),
                               .sel = F(PERI_CLK_CTRL02, 30, 2),
                               .gate = { B(PERI_CLK_CTRL03, 0) } },
    [C(MIPI_DSI_DPHY_PLLREF)] = { PARENTS(C(PLL_F20M), C(RC_FAST),
                                          C(PLL_F25M), PNONE),
                                  .sel = F(PERI_CLK_CTRL02, 30, 2),
                                  .gate = { B(PERI_CLK_CTRL03, 1) } },
    [C(MIPI_CSI_DPHY_CFG)] = { PARENTS(C(PLL_F20M), C(RC_FAST), C(PLL_F25M),
                                       PNONE),
                               .sel = F(PERI_CLK_CTRL03, 2, 2),
                               .gate = { B(PERI_CLK_CTRL03, 4) } },
    [C(MIPI_DSI_DPI)] = { PARENTS(C(XTAL), C(PLL_F240M), C(PLL_F160M),
                                  PNONE),
                          .sel = F(PERI_CLK_CTRL03, 5, 2),
                          .gate = { B(PERI_CLK_CTRL03, 7) },
                          INTDIV(PERI_CLK_CTRL03, 8, 8) },
    [C(I2C0)] = { PARENTS(C(XTAL), C(RC_FAST)),
                  .sel = B(PERI_CLK_CTRL10, 0),
                  .gate = { B(PERI_CLK_CTRL10, 1) },
                  FRAC(PERI_CLK_CTRL10, 2, PERI_CLK_CTRL10, 10,
                       PERI_CLK_CTRL10, 18) },
    [C(I2C1)] = { PARENTS(C(XTAL), C(RC_FAST)),
                  .sel = B(PERI_CLK_CTRL10, 26),
                  .gate = { B(PERI_CLK_CTRL10, 27) },
                  FRAC(PERI_CLK_CTRL11, 0, PERI_CLK_CTRL11, 8,
                       PERI_CLK_CTRL11, 16) },
    [C(I2S0_RX)] = { PARENTS(C(XTAL), C(APLL), C(PAD_I2S0_MCLK),
                             C(PLL_F160M)),
                     .sel = F(PERI_CLK_CTRL11, 25, 2),
                     .gate = { B(PERI_CLK_CTRL11, 24) },
                     I2S(PERI_CLK_CTRL12, 0, PERI_CLK_CTRL12, 8,
                         PERI_CLK_CTRL12, 17, PERI_CLK_CTRL13, 0,
                         PERI_CLK_CTRL13, 9) },
    [C(I2S0_TX)] = { PARENTS(C(XTAL), C(APLL), C(PAD_I2S0_MCLK),
                             C(PLL_F160M)),
                     .sel = F(PERI_CLK_CTRL13, 11, 2),
                     .gate = { B(PERI_CLK_CTRL13, 10) },
                     I2S(PERI_CLK_CTRL13, 13, PERI_CLK_CTRL13, 21,
                         PERI_CLK_CTRL14, 0, PERI_CLK_CTRL14, 9,
                         PERI_CLK_CTRL14, 18) },
    [C(I2S0_MST)] = { PARENTS(C(I2S0_RX), C(I2S0_TX)),
                      .sel = B(PERI_CLK_CTRL14, 19) },
    [C(I2S1_RX)] = { PARENTS(C(XTAL), C(APLL), C(PAD_I2S1_MCLK),
                             C(PLL_F160M)),
                     .sel = F(PERI_CLK_CTRL14, 21, 2),
                     .gate = { B(PERI_CLK_CTRL14, 20) },
                     I2S(PERI_CLK_CTRL14, 23, PERI_CLK_CTRL15, 0,
                         PERI_CLK_CTRL15, 9, PERI_CLK_CTRL15, 18,
                         PERI_CLK_CTRL15, 27) },
    [C(I2S1_TX)] = { PARENTS(C(XTAL), C(APLL), C(PAD_I2S1_MCLK),
                             C(PLL_F160M)),
                     .sel = F(PERI_CLK_CTRL15, 29, 2),
                     .gate = { B(PERI_CLK_CTRL15, 28) },
                     I2S(PERI_CLK_CTRL16, 0, PERI_CLK_CTRL16, 8,
                         PERI_CLK_CTRL16, 17, PERI_CLK_CTRL17, 0,
                         PERI_CLK_CTRL17, 9) },
    [C(I2S1_MST)] = { PARENTS(C(I2S1_RX), C(I2S1_TX)),
                      .sel = B(PERI_CLK_CTRL17, 10) },
    [C(I2S2_RX)] = { PARENTS(C(XTAL), C(APLL), C(PAD_I2S2_MCLK),
                             C(PLL_F160M)),
                     .sel = F(PERI_CLK_CTRL17, 12, 2),
                     .gate = { B(PERI_CLK_CTRL17, 11) },
                     I2S(PERI_CLK_CTRL17, 14, PERI_CLK_CTRL17, 22,
                         PERI_CLK_CTRL18, 0, PERI_CLK_CTRL18, 9,
                         PERI_CLK_CTRL18, 18) },
    [C(I2S2_TX)] = { PARENTS(C(XTAL), C(APLL), C(PAD_I2S2_MCLK),
                             C(PLL_F160M)),
                     .sel = F(PERI_CLK_CTRL18, 20, 2),
                     .gate = { B(PERI_CLK_CTRL18, 19) },
                     I2S(PERI_CLK_CTRL18, 22, PERI_CLK_CTRL19, 0,
                         PERI_CLK_CTRL19, 9, PERI_CLK_CTRL19, 18,
                         PERI_CLK_CTRL19, 27) },
    [C(I2S2_MST)] = { PARENTS(C(I2S2_RX), C(I2S2_TX)),
                      .sel = B(PERI_CLK_CTRL19, 28) },
    [C(LCD)] = { PARENTS(C(XTAL), C(PLL_F160M), C(APLL), PNONE),
                 .sel = F(PERI_CLK_CTRL19, 29, 2),
                 .gate = { B(PERI_CLK_CTRL19, 31) },
                 FRAC(PERI_CLK_CTRL110, 0, PERI_CLK_CTRL110, 8,
                      PERI_CLK_CTRL110, 16) },
    [C(UART0_SCLK)] = { PARENTS_XRF80, .sel = F(PERI_CLK_CTRL110, 24, 2),
                        .gate = { B(PERI_CLK_CTRL110, 26) },
                        FRAC(PERI_CLK_CTRL111, 0, PERI_CLK_CTRL111, 8,
                             PERI_CLK_CTRL111, 16) },
    [C(UART1_SCLK)] = { PARENTS_XRF80, .sel = F(PERI_CLK_CTRL111, 24, 2),
                        .gate = { B(PERI_CLK_CTRL111, 26) },
                        FRAC(PERI_CLK_CTRL112, 0, PERI_CLK_CTRL112, 8,
                             PERI_CLK_CTRL112, 16) },
    [C(UART2_SCLK)] = { PARENTS_XRF80, .sel = F(PERI_CLK_CTRL112, 24, 2),
                        .gate = { B(PERI_CLK_CTRL112, 26) },
                        FRAC(PERI_CLK_CTRL113, 0, PERI_CLK_CTRL113, 8,
                             PERI_CLK_CTRL113, 16) },
    [C(UART3_SCLK)] = { PARENTS_XRF80, .sel = F(PERI_CLK_CTRL113, 24, 2),
                        .gate = { B(PERI_CLK_CTRL113, 26) },
                        FRAC(PERI_CLK_CTRL114, 0, PERI_CLK_CTRL114, 8,
                             PERI_CLK_CTRL114, 16) },
    [C(UART4_SCLK)] = { PARENTS_XRF80, .sel = F(PERI_CLK_CTRL114, 24, 2),
                        .gate = { B(PERI_CLK_CTRL114, 26) },
                        FRAC(PERI_CLK_CTRL115, 0, PERI_CLK_CTRL115, 8,
                             PERI_CLK_CTRL115, 16) },
    [C(TWAI0)] = { PARENTS(C(XTAL), C(RC_FAST)),
                   .sel = B(PERI_CLK_CTRL115, 24),
                   .gate = { B(PERI_CLK_CTRL115, 25) } },
    [C(TWAI1)] = { PARENTS(C(XTAL), C(RC_FAST)),
                   .sel = B(PERI_CLK_CTRL115, 26),
                   .gate = { B(PERI_CLK_CTRL115, 27) } },
    [C(TWAI2)] = { PARENTS(C(XTAL), C(RC_FAST)),
                   .sel = B(PERI_CLK_CTRL115, 28),
                   .gate = { B(PERI_CLK_CTRL115, 29) } },
    [C(GPSPI2)] = { PARENTS(C(XTAL), C(RC_FAST), C(SDIO_PLL0), C(APLL),
                            C(SPLL), PNONE, PNONE, PNONE),
                    .sel = F(PERI_CLK_CTRL116, 0, 3) },
    [C(GPSPI2_HS)] = { PARENTS(C(GPSPI2)),
                       .gate = { B(PERI_CLK_CTRL116, 3) },
                       INTDIV(PERI_CLK_CTRL116, 4, 8) },
    [C(GPSPI2_MST)] = { PARENTS(C(GPSPI2)),
                        .gate = { B(PERI_CLK_CTRL116, 20) },
                        INTDIV(PERI_CLK_CTRL116, 12, 8) },
    [C(GPSPI3)] = { PARENTS(C(XTAL), C(RC_FAST), C(SDIO_PLL0), C(APLL),
                            C(SPLL), PNONE, PNONE, PNONE),
                    .sel = F(PERI_CLK_CTRL116, 21, 3) },
    [C(GPSPI3_HS)] = { PARENTS(C(GPSPI3)),
                       .gate = { B(PERI_CLK_CTRL116, 24) },
                       INTDIV(PERI_CLK_CTRL117, 0, 8) },
    [C(GPSPI3_MST)] = { PARENTS(C(GPSPI3)),
                        .gate = { B(PERI_CLK_CTRL117, 16) },
                        INTDIV(PERI_CLK_CTRL117, 8, 8) },
    [C(PARLIO_RX)] = { PARENTS(C(XTAL), C(RC_FAST), C(PLL_F160M),
                               C(PAD_PARLIO_RX)),
                       .sel = F(PERI_CLK_CTRL117, 17, 2),
                       .gate = { B(PERI_CLK_CTRL117, 19) },
                       FRAC(PERI_CLK_CTRL117, 20, PERI_CLK_CTRL118, 0,
                            PERI_CLK_CTRL118, 8) },
    [C(PARLIO_TX)] = { PARENTS(C(XTAL), C(RC_FAST), C(PLL_F160M),
                               C(PAD_PARLIO_TX)),
                       .sel = F(PERI_CLK_CTRL118, 16, 2),
                       .gate = { B(PERI_CLK_CTRL118, 18) },
                       FRAC(PERI_CLK_CTRL118, 19, PERI_CLK_CTRL119, 0,
                            PERI_CLK_CTRL119, 8) },
    [C(I3C_MST)] = { PARENTS(C(XTAL), C(PLL_F160M), C(PLL_F120M), PNONE),
                     .sel = F(PERI_CLK_CTRL119, 16, 2),
                     .gate = { B(PERI_CLK_CTRL119, 18) },
                     INTDIV(PERI_CLK_CTRL119, 19, 8) },
    [C(CAM)] = { PARENTS(C(XTAL), C(PLL_F160M), C(APLL), PNONE),
                 .sel = F(PERI_CLK_CTRL119, 27, 2),
                 .gate = { B(PERI_CLK_CTRL119, 29) },
                 FRAC(PERI_CLK_CTRL120, 0, PERI_CLK_CTRL120, 8,
                      PERI_CLK_CTRL120, 16) },
    [C(MCPWM0)] = { PARENTS(C(XTAL), C(RC_FAST), C(PLL_F160M), PNONE),
                    .sel = F(PERI_CLK_CTRL20, 0, 2),
                    .gate = { B(PERI_CLK_CTRL20, 2) },
                    INTDIV(PERI_CLK_CTRL20, 3, 8) },
    [C(MCPWM1)] = { PARENTS(C(XTAL), C(RC_FAST), C(PLL_F160M), PNONE),
                    .sel = F(PERI_CLK_CTRL20, 11, 2),
                    .gate = { B(PERI_CLK_CTRL20, 13) },
                    INTDIV(PERI_CLK_CTRL20, 14, 8) },
    [C(TIMERGRP0_T0)] = { PARENTS_XRF80, .sel = F(PERI_CLK_CTRL20, 22, 2),
                          .gate = { B(PERI_CLK_CTRL20, 24) } },
    [C(TIMERGRP0_T1)] = { PARENTS_XRF80, .sel = F(PERI_CLK_CTRL20, 25, 2),
                          .gate = { B(PERI_CLK_CTRL20, 27) } },
    [C(TIMERGRP0_WDT)] = { PARENTS_XRF80, .sel = F(PERI_CLK_CTRL20, 28, 2),
                           .gate = { B(PERI_CLK_CTRL20, 30) } },
    [C(TIMERGRP0_TGRT)] = { PARENTS(C(MPLL), C(SPLL), C(CPLL), C(APLL),
                                    C(SDIO_PLL0), C(SDIO_PLL1), C(SDIO_PLL2),
                                    C(RC_FAST), C(RC_SLOW), C(RC32K),
                                    C(XTAL32K), C(PLL_LP), PIN(DSI_DPHY_IN),
                                    PNONE, PNONE, PNONE),
                            .sel = F(PERI_CLK_CTRL21, 0, 4),
                            .gate = { B(PERI_CLK_CTRL20, 31) },
                            INTDIV(PERI_CLK_CTRL21, 4, 16) },
    [C(TIMERGRP1_T0)] = { PARENTS_XRF80, .sel = F(PERI_CLK_CTRL21, 20, 2),
                          .gate = { B(PERI_CLK_CTRL21, 22) } },
    [C(TIMERGRP1_T1)] = { PARENTS_XRF80, .sel = F(PERI_CLK_CTRL21, 23, 2),
                          .gate = { B(PERI_CLK_CTRL21, 25) } },
    [C(TIMERGRP1_WDT)] = { PARENTS_XRF80, .sel = F(PERI_CLK_CTRL21, 26, 2),
                           .gate = { B(PERI_CLK_CTRL21, 28) } },
    [C(SYSTIMER)] = { PARENTS(C(XTAL), C(RC_FAST)),
                      .sel = B(PERI_CLK_CTRL21, 29),
                      .gate = { B(PERI_CLK_CTRL21, 30) } },
    [C(LEDC)] = { PARENTS_XRF80, .sel = F(PERI_CLK_CTRL22, 0, 2),
                  .gate = { B(PERI_CLK_CTRL22, 2) } },
    [C(RMT)] = { PARENTS_XRF80, .sel = F(PERI_CLK_CTRL22, 3, 2),
                 .gate = { B(PERI_CLK_CTRL22, 5) },
                 FRAC(PERI_CLK_CTRL22, 6, PERI_CLK_CTRL22, 14,
                      PERI_CLK_CTRL22, 22) },
    [C(ADC)] = { PARENTS_XRF80, .sel = F(PERI_CLK_CTRL22, 30, 2),
                 .gate = { B(PERI_CLK_CTRL23, 0) },
                 FRAC(PERI_CLK_CTRL23, 1, PERI_CLK_CTRL23, 9,
                      PERI_CLK_CTRL23, 17) },
    [C(ADC_SAR1)] = { PARENTS(C(ADC)), INTDIV(PERI_CLK_CTRL24, 0, 8) },
    [C(ADC_SAR2)] = { PARENTS(C(ADC)), INTDIV(PERI_CLK_CTRL24, 8, 8) },
    [C(PVT)] = { PARENTS(C(PLL_F160M)), .gate = { B(PERI_CLK_CTRL24, 24) },
                 INTDIV(PERI_CLK_CTRL24, 16, 8) },
    [C(PVT_PERI_GROUP)] = { PARENTS(C(PLL_F160M)),
                            INTDIV(PERI_CLK_CTRL25, 0, 8) },
    GATE(PVT_PERI_GROUP1, PVT_PERI_GROUP, PERI_CLK_CTRL25, 8),
    GATE(PVT_PERI_GROUP2, PVT_PERI_GROUP, PERI_CLK_CTRL25, 9),
    GATE(PVT_PERI_GROUP3, PVT_PERI_GROUP, PERI_CLK_CTRL25, 10),
    GATE(PVT_PERI_GROUP4, PVT_PERI_GROUP, PERI_CLK_CTRL25, 11),
    [C(CRYPTO)] = { PARENTS(C(XTAL), C(RC_FAST), C(PLL_F240M), C(PLL_F160M)),
                    .sel = F(PERI_CLK_CTRL25, 12, 2) },
    GATE(CRYPTO_AES, CRYPTO, PERI_CLK_CTRL25, 14),
    GATE(CRYPTO_DS, CRYPTO, PERI_CLK_CTRL25, 15),
    GATE(CRYPTO_ECC, CRYPTO, PERI_CLK_CTRL25, 16),
    GATE(CRYPTO_HMAC, CRYPTO, PERI_CLK_CTRL25, 17),
    GATE(CRYPTO_RSA, CRYPTO, PERI_CLK_CTRL25, 18),
    GATE(CRYPTO_SEC, CRYPTO, PERI_CLK_CTRL25, 19),
    GATE(CRYPTO_SHA, CRYPTO, PERI_CLK_CTRL25, 20),
    GATE(CRYPTO_ECDSA, CRYPTO, PERI_CLK_CTRL25, 21),
    GATE(CRYPTO_KM, CRYPTO, PERI_CLK_CTRL25, 22),
    [C(ISP)] = { PARENTS(C(XTAL), C(PLL_F240M), C(PLL_F160M), PNONE),
                 .sel = F(PERI_CLK_CTRL25, 23, 2),
                 .gate = { B(PERI_CLK_CTRL25, 25) },
                 INTDIV(PERI_CLK_CTRL26, 0, 8) },
    [C(IOMUX)] = { PARENTS(C(XTAL), C(PLL_F80M)),
                   .sel = B(PERI_CLK_CTRL26, 8),
                   .gate = { B(PERI_CLK_CTRL26, 9) },
                   INTDIV(PERI_CLK_CTRL26, 10, 8) },
    [C(H264)] = { PARENTS(C(XTAL), C(PLL_F240M)),
                  .sel = B(PERI_CLK_CTRL26, 18),
                  .gate = { B(PERI_CLK_CTRL26, 19) },
                  INTDIV(PERI_CLK_CTRL26, 20, 8) },
    [C(CLKOUT0)] = { .kind = SPECIAL },
    [C(CLKOUT1)] = { .kind = SPECIAL },
    [C(CLKOUT2)] = { .kind = SPECIAL },

    GATE(USB_OTG11_48M, PLL_F48M, LP_HP_USB_CLKRST_CTRL0, 4),
    GATE(USB_DEVICE_48M, PLL_F48M, LP_HP_USB_CLKRST_CTRL0, 5),
    [C(USB_OTG20_PHYREF)] = { PARENTS(C(PLL_F12M), C(USB_PLL_F25M),
                                      PIN(PAD_USB_HSPHY_REFCLK_IN), PNONE),
                              .sel = F(LP_HP_USB_CLKRST_CTRL1, 28, 2),
                              .gate = { B(LP_HP_USB_CLKRST_CTRL1, 30) } },
    GATE(LP_RAM, LP_DYN_FAST, LP_CLK_EN, 27),

    LPPERI_GATE(LP_RNG, 16),
    LPPERI_GATE(LP_TSENS, 17),
    LPPERI_GATE(LP_PMS, 18),
    LPPERI_GATE(LP_EFUSE, 19),
    LPPERI_GATE(LP_IOMUX, 20),
    LPPERI_GATE(LP_TOUCH, 21),
    LPPERI_GATE(LP_SPI, 22),
    LPPERI_GATE(LP_ADC, 23),
    LPPERI_GATE(LP_I2S, 26),
    LPPERI_GATE(LP_I2CMST, 27),
    LPPERI_GATE(LP_I2C, 28),
    LPPERI_GATE(LP_UART, 29),
    LPPERI_GATE(LP_INTR, 30),
    [C(LP_I2S_RX)] = { PARENTS(C(LP_FAST), C(XTAL_D2), C(OSC_PLL_LP), PNONE),
                       .sel = F(LPPERI_CORE_CLK_SEL, 26, 2),
                       .gate = { B(LPPERI_CLK_EN, 25) },
                       I2S(LPPERI_I2S_RXCLK_DIV_NUM, 24,
                           LPPERI_I2S_RXCLK_DIV_XYZ, 23,
                           LPPERI_I2S_RXCLK_DIV_XYZ, 14,
                           LPPERI_I2S_RXCLK_DIV_XYZ, 5,
                           LPPERI_I2S_RXCLK_DIV_XYZ, 4) },
    [C(LP_I2S_TX)] = { PARENTS(C(LP_FAST), C(XTAL_D2), C(OSC_PLL_LP), PNONE),
                       .sel = F(LPPERI_CORE_CLK_SEL, 24, 2),
                       .gate = { B(LPPERI_CLK_EN, 24) },
                       I2S(LPPERI_I2S_TXCLK_DIV_NUM, 24,
                           LPPERI_I2S_TXCLK_DIV_XYZ, 23,
                           LPPERI_I2S_TXCLK_DIV_XYZ, 14,
                           LPPERI_I2S_TXCLK_DIV_XYZ, 5,
                           LPPERI_I2S_TXCLK_DIV_XYZ, 4) },
    [C(LP_I2C_SCLK)] = { PARENTS(C(LP_FAST), C(XTAL_D2), C(OSC_PLL_LP),
                                 PNONE),
                         .sel = F(LPPERI_CORE_CLK_SEL, 28, 2),
                         .gate = { B(LPPERI_CLK_EN, 28) } },
    [C(LP_UART_SCLK)] = { PARENTS(C(OSC_RC_FAST), C(XTAL_D2), C(OSC_PLL_LP),
                                  PNONE),
                          .sel = F(LPPERI_CORE_CLK_SEL, 30, 2),
                          .gate = { B(LPPERI_CLK_EN, 29) } },
    [C(LP_ADC_FUNC)] = { PARENTS(C(LP_ADC)),
                         INTDIV(LPPERI_ADC_CTRL, 8, 8) },
    [C(LP_ADC_SAR1)] = { PARENTS(C(LP_ADC_FUNC)),
                         INTDIV(LPPERI_ADC_CTRL, 24, 8) },
    [C(LP_ADC_SAR2)] = { PARENTS(C(LP_ADC_FUNC)),
                         INTDIV(LPPERI_ADC_CTRL, 16, 8) },
    GATE(EFUSE, XTAL_D2, LPPERI_CLK_EN, 19),
    [C(LP_CORE)] = { .kind = SPECIAL },
};

/*
 * The block resets: the RST_EN bit that holds the block, the FORCE_NORST
 * bit that overrides it, if any, and a group reset that also holds it.
 */
typedef struct {
    ClkField rst;
    ClkField norst;
    ClkField group;
} RstDesc;

#define RST(id, r, b) [ESP32P4_RST_##id] = { .rst = B(r, b) }
#define RSTN(id, r, b, nr, nb) \
    [ESP32P4_RST_##id] = { .rst = B(r, b), .norst = B(nr, nb) }
#define RSTC(id, b) \
    [ESP32P4_RST_##id] = { .rst = B(HP_RST_EN2, b), \
                           .group = B(HP_RST_EN2, 14) }

static const RstDesc esp32p4_rst_desc[ESP32P4_NUM_RESETS] = {
    RST(CORECTRL, HP_RST_EN0, 0),
    RST(PVT_TOP, HP_RST_EN0, 1),
    RST(PVT_PERI_GROUP1, HP_RST_EN0, 2),
    RST(PVT_PERI_GROUP2, HP_RST_EN0, 3),
    RST(PVT_PERI_GROUP3, HP_RST_EN0, 4),
    RST(PVT_PERI_GROUP4, HP_RST_EN0, 5),
    RST(REGDMA, HP_RST_EN0, 6),
    RSTN(CORETRACE0, HP_RST_EN0, 9, HP_FORCE_NORST0, 2),
    RSTN(CORETRACE1, HP_RST_EN0, 10, HP_FORCE_NORST0, 3),
    RST(HP_TCM, HP_RST_EN0, 11),
    RST(HP_CACHE, HP_RST_EN0, 12),
    RST(L1_I0_CACHE, HP_RST_EN0, 13),
    RST(L1_I1_CACHE, HP_RST_EN0, 14),
    RST(L1_D_CACHE, HP_RST_EN0, 15),
    RST(L2_CACHE, HP_RST_EN0, 16),
    RST(L2_MEM, HP_RST_EN0, 17),
    RSTN(L2MEMMON, HP_RST_EN0, 18, HP_FORCE_NORST0, 4),
    RSTN(TCMMON, HP_RST_EN0, 19, HP_FORCE_NORST0, 5),
    RST(PVT_APB, HP_RST_EN0, 20),
    RSTN(GDMA, HP_RST_EN0, 21, HP_FORCE_NORST0, 6),
    RSTN(MSPI_AXI, HP_RST_EN0, 22, HP_FORCE_NORST0, 7),
    RSTN(DUAL_MSPI_AXI, HP_RST_EN0, 23, HP_FORCE_NORST0, 8),
    RSTN(MSPI_APB, HP_RST_EN0, 24, HP_FORCE_NORST0, 9),
    RSTN(DUAL_MSPI_APB, HP_RST_EN0, 25, HP_FORCE_NORST0, 10),
    RSTN(DSI_BRG, HP_RST_EN0, 26, HP_FORCE_NORST0, 11),
    RSTN(CSI_HOST, HP_RST_EN0, 27, HP_FORCE_NORST0, 12),
    RSTN(CSI_BRG, HP_RST_EN0, 28, HP_FORCE_NORST0, 13),
    RSTN(ISP, HP_RST_EN0, 29, HP_FORCE_NORST0, 14),
    RSTN(JPEG, HP_RST_EN0, 30, HP_FORCE_NORST0, 15),
    RSTN(DMA2D, HP_RST_EN0, 31, HP_FORCE_NORST0, 16),
    RSTN(PPA, HP_RST_EN1, 0, HP_FORCE_NORST0, 17),
    RSTN(AHB_PDMA, HP_RST_EN1, 1, HP_FORCE_NORST0, 18),
    RSTN(AXI_PDMA, HP_RST_EN1, 2, HP_FORCE_NORST0, 19),
    RSTN(IOMUX, HP_RST_EN1, 3, HP_FORCE_NORST0, 20),
    RSTN(PADBIST, HP_RST_EN1, 4, HP_FORCE_NORST0, 21),
    RSTN(STIMER, HP_RST_EN1, 5, HP_FORCE_NORST0, 22),
    RSTN(TIMERGRP0, HP_RST_EN1, 6, HP_FORCE_NORST0, 23),
    RSTN(TIMERGRP1, HP_RST_EN1, 7, HP_FORCE_NORST0, 24),
    RSTN(UART0_CORE, HP_RST_EN1, 8, HP_FORCE_NORST0, 25),
    RSTN(UART1_CORE, HP_RST_EN1, 9, HP_FORCE_NORST0, 26),
    RSTN(UART2_CORE, HP_RST_EN1, 10, HP_FORCE_NORST0, 27),
    RSTN(UART3_CORE, HP_RST_EN1, 11, HP_FORCE_NORST0, 28),
    RSTN(UART4_CORE, HP_RST_EN1, 12, HP_FORCE_NORST0, 29),
    RSTN(UART0_APB, HP_RST_EN1, 13, HP_FORCE_NORST0, 25),
    RSTN(UART1_APB, HP_RST_EN1, 14, HP_FORCE_NORST0, 26),
    RSTN(UART2_APB, HP_RST_EN1, 15, HP_FORCE_NORST0, 27),
    RSTN(UART3_APB, HP_RST_EN1, 16, HP_FORCE_NORST0, 28),
    RSTN(UART4_APB, HP_RST_EN1, 17, HP_FORCE_NORST0, 29),
    RSTN(UHCI, HP_RST_EN1, 18, HP_FORCE_NORST0, 30),
    RSTN(I3CMST, HP_RST_EN1, 19, HP_FORCE_NORST0, 31),
    RSTN(I3CSLV, HP_RST_EN1, 20, HP_FORCE_NORST1, 0),
    RSTN(I2C1, HP_RST_EN1, 21, HP_FORCE_NORST1, 1),
    RSTN(I2C0, HP_RST_EN1, 22, HP_FORCE_NORST1, 2),
    RSTN(RMT, HP_RST_EN1, 23, HP_FORCE_NORST1, 3),
    RSTN(PWM0, HP_RST_EN1, 24, HP_FORCE_NORST1, 4),
    RSTN(PWM1, HP_RST_EN1, 25, HP_FORCE_NORST1, 5),
    RSTN(CAN0, HP_RST_EN1, 26, HP_FORCE_NORST1, 6),
    RSTN(CAN1, HP_RST_EN1, 27, HP_FORCE_NORST1, 7),
    RSTN(CAN2, HP_RST_EN1, 28, HP_FORCE_NORST1, 8),
    RSTN(LEDC, HP_RST_EN1, 29, HP_FORCE_NORST1, 9),
    RSTN(PCNT, HP_RST_EN1, 30, HP_FORCE_NORST1, 10),
    RSTN(ETM, HP_RST_EN1, 31, HP_FORCE_NORST1, 11),
    RSTN(INTRMTX, HP_RST_EN2, 0, HP_FORCE_NORST1, 12),
    RSTN(PARLIO, HP_RST_EN2, 1, HP_FORCE_NORST1, 13),
    RSTN(PARLIO_RX, HP_RST_EN2, 2, HP_FORCE_NORST1, 14),
    RSTN(PARLIO_TX, HP_RST_EN2, 3, HP_FORCE_NORST1, 15),
    RSTN(I2S0_APB, HP_RST_EN2, 4, HP_FORCE_NORST1, 16),
    RSTN(I2S1_APB, HP_RST_EN2, 5, HP_FORCE_NORST1, 17),
    RSTN(I2S2_APB, HP_RST_EN2, 6, HP_FORCE_NORST1, 18),
    RSTN(SPI2, HP_RST_EN2, 7, HP_FORCE_NORST1, 19),
    RSTN(SPI3, HP_RST_EN2, 8, HP_FORCE_NORST1, 20),
    RSTN(LCDCAM, HP_RST_EN2, 9, HP_FORCE_NORST1, 21),
    RSTN(ADC, HP_RST_EN2, 10, HP_FORCE_NORST1, 22),
    RSTN(BITSCRAMBLER, HP_RST_EN2, 11, HP_FORCE_NORST1, 23),
    RSTN(BITSCRAMBLER_RX, HP_RST_EN2, 12, HP_FORCE_NORST1, 24),
    RSTN(BITSCRAMBLER_TX, HP_RST_EN2, 13, HP_FORCE_NORST1, 25),
    RST(CRYPTO, HP_RST_EN2, 14),
    RSTC(SEC, 15),
    RSTC(AES, 16),
    RSTC(DS, 17),
    RSTC(SHA, 18),
    RSTC(HMAC, 19),
    RSTC(ECDSA, 20),
    RSTC(RSA, 21),
    RSTC(ECC, 22),
    RSTC(KM, 23),
    RSTN(H264, HP_RST_EN2, 24, HP_FORCE_NORST1, 26),
    RST(AXI_PERF_MON, AXI_PERF_MON_CLKRST_CTRL0, 1),
    RST(LP_HUK, LP_RST_EN, 24),
    RST(LP_ANAPERI, LP_RST_EN, 25),
    RST(LP_WDT, LP_RST_EN, 26),
    RST(LP_TIMER, LP_RST_EN, 27),
    RST(LP_RTC, LP_RST_EN, 28),
    RST(LP_MAILBOX, LP_RST_EN, 29),
    RST(LP_AONEFUSEREG, LP_RST_EN, 30),
    RST(LP_RAM, LP_RST_EN, 31),
    RST(USB_OTG20_ADP, LP_HP_USB_CLKRST_CTRL1, 0),
    RST(USB_OTG20_PHY, LP_HP_USB_CLKRST_CTRL1, 1),
    RST(USB_OTG20, LP_HP_USB_CLKRST_CTRL1, 2),
    RST(USB_OTG11, LP_HP_USB_CLKRST_CTRL1, 3),
    RST(USB_DEVICE, LP_HP_USB_CLKRST_CTRL1, 4),
    RSTN(SDMMC, LP_HP_SDMMC_EMAC_RST_CTRL, 28, LP_HP_SDMMC_EMAC_RST_CTRL, 29),
    RSTN(EMAC, LP_HP_SDMMC_EMAC_RST_CTRL, 30, LP_HP_SDMMC_EMAC_RST_CTRL, 31),
    RST(LP_RNG, LPPERI_RESET_EN, 17),
    RST(LP_TSENS, LPPERI_RESET_EN, 18),
    RST(LP_PMS, LPPERI_RESET_EN, 19),
    RST(LP_EFUSE, LPPERI_RESET_EN, 20),
    RST(LP_IOMUX, LPPERI_RESET_EN, 21),
    RST(LP_TOUCH, LPPERI_RESET_EN, 22),
    RST(LP_SPI, LPPERI_RESET_EN, 23),
    RST(LP_ADC, LPPERI_RESET_EN, 24),
    RST(LP_I2S, LPPERI_RESET_EN, 25),
    RST(LP_I2CMST, LPPERI_RESET_EN, 26),
    RST(LP_I2C, LPPERI_RESET_EN, 27),
    RST(LP_UART, LPPERI_RESET_EN, 28),
    RST(LP_INTR, LPPERI_RESET_EN, 29),
    RST(LP_ROM, LPPERI_RESET_EN, 30),
};

#define ESP32P4_CLKRST_NAME(id, name) [ESP32P4_CLKIN_##id] = (name),
const char *const esp32p4_clock_input_names[ESP32P4_NUM_CLKIN] = {
    ESP32P4_CLKRST_INPUTS(ESP32P4_CLKRST_NAME)
};
#undef ESP32P4_CLKRST_NAME

#define ESP32P4_CLKRST_NAME(id, name) [ESP32P4_CLK_##id] = (name),
const char *const esp32p4_clock_names[ESP32P4_NUM_CLOCKS] = {
    ESP32P4_CLKRST_CLOCKS(ESP32P4_CLKRST_NAME)
};
#undef ESP32P4_CLKRST_NAME

#define ESP32P4_CLKRST_NAME(id, name) [ESP32P4_RST_##id] = (name),
const char *const esp32p4_reset_names[ESP32P4_NUM_RESETS] = {
    ESP32P4_CLKRST_RESETS(ESP32P4_CLKRST_NAME)
};
#undef ESP32P4_CLKRST_NAME

/* The frequencies of the oscillators the model does not trim. */
#define XTAL_HZ         40000000
#define RC_SLOW_HZ      150000
#define RC32K_HZ        32768
#define PLL_LP_HZ       8000000

/* RC_FAST_CLK at two FOSC_DFREQ settings, ESP-IDF's and the reset value. */
#define RC_FAST_LO_DFREQ 100
#define RC_FAST_LO_HZ    17500000
#define RC_FAST_HI_DFREQ 400
#define RC_FAST_HI_HZ    20000000

/* The SoC clocks' limits (TRM section 11.2.4.1, v3.x). */
static const uint64_t esp32p4_soc_max_hz[4] = {
    400000000, 200000000, 200000000, 100000000,
};
static const char *const esp32p4_soc_names[4] = {
    "CPU_CLK", "MEM_CLK", "SYS_CLK", "APB_CLK",
};

/* The debug clock outputs' signals (ESP-IDF's soc_clkout_sig_id_t). */
static const struct {
    uint8_t sig;
    uint16_t clk;
} esp32p4_clkout_sigs[] = {
    { 0, C(MPLL) },
    { 1, C(SPLL) },
    { 2, C(CPLL) },
    { 3, C(XTAL) },
    { 4, C(RC_FAST) },
    { 5, C(RC_SLOW) },
    { 6, C(RC32K) },
    { 7, C(XTAL32K) },
    { 16, C(I2S0_MST) },
    { 17, C(I2S1_MST) },
    { 18, C(I2S2_MST) },
    { 26, C(CPU) },
    { 27, C(MEM) },
    { 28, C(SYS) },
    { 29, C(APB) },
    { 105, C(PLL_F80M) },
};
#define CLKOUT_NONE 0xff

/*
 * Gating these does not stop the parts of the HP system the machine
 * keeps running: the interconnect, the TCM, L2MEM and the system
 * registers. The bits by register, of the clock gates and of the resets.
 */
#define KEPT_SOC_CLK_CTRL0 (BIT(2) | BIT(5) | BIT(12) | BIT(20) | BIT(22) | \
                            BIT(23) | BIT(26) | BIT(29))
#define KEPT_SOC_CLK_CTRL2 (BIT(1) | BIT(2) | BIT(3))
#define KEPT_SOC_CLK_CTRL3 BIT(4)
#define KEPT_HP_RST_EN0    (BIT(0) | BIT(11) | BIT(17))

struct ESP32P4ClkRstGate {
    ESP32P4ClkRstState *s;
    DeviceState *dev;
    unsigned clock;
    unsigned reset;
    MemoryRegion iomem;
    AddressSpace dev_as;
    bool held;
    bool has_volatile;
    hwaddr volatile_offset;
    unsigned volatile_size;
    QTAILQ_ENTRY(ESP32P4ClkRstGate) next;
};

static uint32_t fld(ESP32P4ClkRstState *s, ClkField f)
{
    return f.width ? extract32(s->regs[f.reg], f.shift, f.width) : 0;
}

/*
 * [spec:nuos:req:emu.esp32p4.clocks]
 * Clock input @in's period, exactly as the device driving it gives it: a
 * PLL's from the analog block, a pad's from the block whose clock the
 * pad carries (esp32p4_clk_exact()).
 */
static ESP32P4ClkPeriod esp32p4_clkrst_input(ESP32P4ClkRstState *s, int in)
{
    return esp32p4_clk_exact(s->clk_in[in]);
}

static ESP32P4ClkPeriod esp32p4_clkrst_parent(ESP32P4ClkRstState *s,
                                              int16_t p)
{
    if (p == PARENT_NONE) {
        return esp32p4_clk_period_stopped();
    }
    if (p & PARENT_IN) {
        return esp32p4_clkrst_input(s, p & ~PARENT_IN);
    }
    return s->exact[p];
}

/* The period divided by (int + 1) + num / den; den 0 drops the fraction. */
static ESP32P4ClkPeriod esp32p4_clkrst_frac(ESP32P4ClkPeriod p,
                                            uint32_t intpart, uint32_t num,
                                            uint32_t den)
{
    if (!den) {
        return esp32p4_clk_period_scale(p, intpart + 1ull, 1);
    }
    return esp32p4_clk_period_scale(p, (uint64_t)(intpart + 1) * den + num,
                                    den);
}

/*
 * The I2S divider: N + b/a, where X, Y, Z and YN1 encode the fraction
 * (TRM section 49.6): with YN1 clear, b = Z and a = (X + 1) Z + Y; with
 * it set, a - b = Z and a = (X + 1) Z + Y.
 */
static ESP32P4ClkPeriod esp32p4_clkrst_i2s(ESP32P4ClkPeriod p, uint32_t n,
                                           uint32_t x, uint32_t y, uint32_t z,
                                           uint32_t yn1)
{
    uint64_t a = (uint64_t)(x + 1) * z + y;
    uint64_t b = yn1 ? a - MIN(z, a) : z;

    if (!a) {
        b = 0;
        a = 1;
    }
    return esp32p4_clk_period_scale(p, n * a + b, a);
}

/*
 * RC_FAST_CLK, linear in FOSC_DFREQ through the two settings whose
 * frequencies are known, as the exact fraction of Hz that gives.
 */
static ESP32P4ClkPeriod esp32p4_clkrst_rc_fast(ESP32P4ClkRstState *s)
{
    int64_t dfreq = extract32(s->regs[R_LP_FOSC_CNTL], 22, 10);
    int64_t den = RC_FAST_HI_DFREQ - RC_FAST_LO_DFREQ;
    int64_t num = RC_FAST_LO_HZ * den +
                  (dfreq - RC_FAST_LO_DFREQ) * (RC_FAST_HI_HZ - RC_FAST_LO_HZ);

    return esp32p4_clk_period_of_hz(num, den);
}

static ESP32P4ClkPeriod esp32p4_clkrst_special(ESP32P4ClkRstState *s, int id)
{
    ESP32P4ClkPeriod stopped = esp32p4_clk_period_stopped();
    uint32_t v, sel;
    int i, src;

    switch (id) {
    case C(OSC_XTAL):
        return s->pmu_xpd[ESP32P4_CLKRST_XPD_XTAL] ?
               esp32p4_clk_period_of_hz(XTAL_HZ, 1) : stopped;
    case C(OSC_RC_FAST):
        return s->pmu_xpd[ESP32P4_CLKRST_XPD_RC_FAST] ?
               esp32p4_clkrst_rc_fast(s) : stopped;
    case C(OSC_RC_SLOW):
        return esp32p4_clk_period_of_hz(RC_SLOW_HZ, 1);
    case C(OSC_RC32K):
        return s->pmu_xpd[ESP32P4_CLKRST_XPD_RC32K] ?
               esp32p4_clk_period_of_hz(RC32K_HZ, 1) : stopped;
    case C(OSC_XTAL32K):
        return s->pmu_xpd[ESP32P4_CLKRST_XPD_XTAL32K] ?
               esp32p4_clk_period_of_hz(s->xtal32k_hz, 1) : stopped;
    case C(OSC_PLL_LP):
        src = extract32(s->regs[R_LP_CLK_CONF], 10, 1) ? C(OSC_XTAL32K) :
                                                         C(OSC_RC32K);
        return s->pmu_xpd[ESP32P4_CLKRST_XPD_LP_PLL] && s->period[src] ?
               esp32p4_clk_period_of_hz(PLL_LP_HZ, 1) : stopped;
    case C(CPU):
    case C(MEM):
    case C(SYS):
    case C(APB):
        i = id - C(CPU);
        return esp32p4_clkrst_frac(s->exact[id - 1], s->soc_div[i][0],
                                   s->soc_div[i][1], s->soc_div[i][2]);
    case C(SDIO_LS):
        v = s->regs[R_PERI_CLK_CTRL01];
        if (!extract32(v, 24, 1)) {
            return stopped;
        }
        src = extract32(v, 23, 1) ? C(APLL) : C(PLL_F160M);
        return esp32p4_clk_period_scale(
            s->exact[src],
            (uint64_t)(extract32(s->regs[R_PERI_CLK_CTRL02], 0, 8) + 1) *
            (s->sdio_edge[2] + 1), 1);
    case C(SDIO_SLF):
    case C(SDIO_DRV):
    case C(SDIO_SAM):
        i = id - C(SDIO_SLF);
        if (!extract32(s->regs[R_PERI_CLK_CTRL02], 27 + i, 1)) {
            return stopped;
        }
        if (extract32(s->regs[R_PERI_CLK_CTRL01], 22, 1)) {
            return s->exact[C(SDIO_PLL0) + i];
        }
        return s->exact[C(SDIO_LS)];
    case C(CLKOUT0):
    case C(CLKOUT1):
    case C(CLKOUT2):
        i = id - C(CLKOUT0);
        if (!extract32(s->regs[R_DBG_CLK_CTRL1], 16 + i, 1)) {
            return stopped;
        }
        sel = extract32(s->regs[R_DBG_CLK_CTRL0], 8 * i, 8);
        v = i == 0 ? extract32(s->regs[R_DBG_CLK_CTRL0], 24, 8) :
                     extract32(s->regs[R_DBG_CLK_CTRL1], 8 * (i - 1), 8);
        for (src = 0; src < ARRAY_SIZE(esp32p4_clkout_sigs); src++) {
            if (esp32p4_clkout_sigs[src].sig == sel) {
                return esp32p4_clk_period_scale(
                    s->exact[esp32p4_clkout_sigs[src].clk], v + 1, 1);
            }
        }
        return stopped;
    case C(LP_DYN_FAST):
        return s->exact[s->pmu_lp_sleep ? C(LP_SLOW) : C(LP_FAST)];
    case C(LP_CORE):
        if (!(s->regs[R_LPPERI_CLK_EN] & CK_EN_LP_CORE) &&
            !s->pmu_lp_core_clock) {
            return stopped;
        }
        return s->exact[C(LP_DYN_FAST)];
    default:
        g_assert_not_reached();
    }
}

/* The clocks PMU_n1_DIG_ICG_FUNC_EN gates, by bit (TRM table 15.4-2). */
static const int16_t esp32p4_pmu_icg_clocks[ESP32P4_CLKRST_NUM_PMU_ICG] = {
    C(CORE0_CPU), C(CORE1_CPU), C(CORE0_CLIC), C(CORE1_CLIC),
    C(MISC_CPU), C(MISC_SYS), C(ICM_SYS), C(ICM_CPU), C(ICM_MEM),
    C(ICM_APB), C(TCM_CPU), C(L2MEM_MEM), C(L2MEM_SYS), C(L1CACHE_CPU),
    C(L1CACHE_D_CPU), C(L1CACHE_I0_CPU), C(L1CACHE_I1_CPU),
    C(L1CACHE_MEM), C(L1CACHE_D_MEM), C(L1CACHE_I0_MEM),
    C(L1CACHE_I1_MEM), C(L2CACHE_MEM), C(L2CACHE_SYS), C(REGDMA_SYS),
    C(HP_CLKRST_APB), C(SYSREG_APB), C(INTRMTX_APB),
};

/*
 * [spec:nuos:req:emu.esp32p4.low-power]
 * Whether the PMU stops clock @id: ROOT_CLK by its gate of the HP root
 * clock, the clocks of table 15.4-2 by their clock gates.
 */
static bool esp32p4_clkrst_pmu_gated(ESP32P4ClkRstState *s, int id)
{
    int i;

    if (id == C(ROOT)) {
        return s->pmu_sysclk[ESP32P4_CLKRST_SYSCLK_GATE];
    }
    for (i = 0; i < ESP32P4_CLKRST_NUM_PMU_ICG; i++) {
        if (esp32p4_pmu_icg_clocks[i] == id) {
            return !s->pmu_icg[i];
        }
    }
    return false;
}

/*
 * [spec:nuos:req:emu.esp32p4.clocks]
 * Clock @id's period, exactly, from its parent's: each divider scales the
 * exact period, so that only the Clock's rounding is ever lost.
 */
static ESP32P4ClkPeriod esp32p4_clkrst_compute(ESP32P4ClkRstState *s, int id)
{
    const ClkDesc *d = &esp32p4_clk_desc[id];
    ESP32P4ClkPeriod p;
    uint32_t sel;
    int i;

    if (d->kind == SPECIAL) {
        return esp32p4_clkrst_special(s, id);
    }
    sel = fld(s, d->sel);
    if (id == C(ROOT) && s->pmu_sysclk[ESP32P4_CLKRST_SYSCLK_PMU_SEL]) {
        sel = s->pmu_sysclk[ESP32P4_CLKRST_SYSCLK_SEL0] |
              s->pmu_sysclk[ESP32P4_CLKRST_SYSCLK_SEL1] << 1;
    }
    p = sel < d->nparents ? esp32p4_clkrst_parent(s, d->parent[sel]) :
                            esp32p4_clk_period_stopped();
    for (i = 0; i < ARRAY_SIZE(d->gate); i++) {
        if (d->gate[i].width && !fld(s, d->gate[i])) {
            return esp32p4_clk_period_stopped();
        }
    }
    if (esp32p4_clkrst_pmu_gated(s, id)) {
        return esp32p4_clk_period_stopped();
    }
    switch (d->kind) {
    case DIV_INT:
        return esp32p4_clk_period_scale(p, fld(s, d->div[0]) + 1ull, 1);
    case DIV_FRAC:
        return esp32p4_clkrst_frac(p, fld(s, d->div[0]), fld(s, d->div[1]),
                                   fld(s, d->div[2]));
    case DIV_I2S:
        return esp32p4_clkrst_i2s(p, fld(s, d->div[0]), fld(s, d->div[1]),
                                  fld(s, d->div[2]), fld(s, d->div[3]),
                                  fld(s, d->div[4]));
    case DIV_FIXED:
        return esp32p4_clk_period_scale(p, d->fixed, 1);
    default:
        return p;
    }
}

static void esp32p4_clkrst_check_soc(ESP32P4ClkRstState *s)
{
    int i;
    uint64_t period, hz;

    for (i = 0; i < 4; i++) {
        period = s->period[C(CPU) + i];
        if (period == s->checked_soc[i]) {
            continue;
        }
        s->checked_soc[i] = period;
        hz = period ? CLOCK_PERIOD_TO_HZ(period) : 0;
        if (hz > esp32p4_soc_max_hz[i]) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: %s at %" PRIu64 " Hz is above "
                          "its %" PRIu64 " Hz maximum\n", TYPE_ESP32P4_CLKRST,
                          esp32p4_soc_names[i], hz, esp32p4_soc_max_hz[i]);
        }
    }
}

/* Whether the PMU has the HP cores' power, PD_TOP's or PD_HP_CPU's, down. */
static bool esp32p4_clkrst_hp_cores_off(ESP32P4ClkRstState *s)
{
    return !s->pmu_power[ESP32P4_CLKRST_PD_TOP] ||
           !s->pmu_power[ESP32P4_CLKRST_PD_HP_CPU];
}

/*
 * [spec:nuos:req:emu.esp32p4.clocks]
 * The HP and LP cores' reset, clock enable and reset vector.
 */
static void esp32p4_clkrst_update_cores(ESP32P4ClkRstState *s, bool force)
{
    uint32_t ctrl0 = s->regs[R_LP_HPCPU_RESET_CTRL0];
    bool level;
    int n;

    for (n = 0; n < ESP32P4_CLKRST_HP_CORES; n++) {
        level = s->hp_core_reset || esp32p4_clkrst_hp_cores_off(s) ||
                (!(s->regs[R_HP_FORCE_NORST0] & FORCE_NORST_CORE(n)) &&
                 ((s->regs[R_HP_RST_EN0] & RST_EN_CORE_GLOBAL(n)) ||
                  s->core_pulse[n] == ESP32P4_CORE_PULSE_RESET));
        if (force || level != s->core_reset_level[n]) {
            s->core_reset_level[n] = level;
            qemu_set_irq(s->core_reset[n], level);
        }
        level = s->core_pulse[n] == ESP32P4_CORE_PULSE_STALL ||
                s->pmu_core_stall[n];
        if (force || level != s->core_stall_level[n]) {
            s->core_stall_level[n] = level;
            qemu_set_irq(s->core_stall[n], level);
        }
        level = s->period[C(CORE0_CPU) + n] &&
                s->core_pulse[n] != ESP32P4_CORE_PULSE_STALL &&
                !s->pmu_core_stall[n];
        if (force || level != s->core_clock_level[n]) {
            s->core_clock_level[n] = level;
            qemu_set_irq(s->core_clock[n], level);
        }
        level = ctrl0 & (HPCORE_STAT_VECTOR_SEL << HPCORE_SHIFT(n));
        if (force || level != s->core_vector_level[n]) {
            s->core_vector_level[n] = level;
            qemu_set_irq(s->core_vector[n], level);
        }
    }

    level = s->pmu_lp_core_reset ||
            !s->pmu_power[ESP32P4_CLKRST_PD_LP_PERI];
    if (force || level != s->lp_core_reset_level) {
        s->lp_core_reset_level = level;
        qemu_set_irq(s->lp_core_reset, level);
    }
    level = s->period[C(LP_CORE)] != 0 && !s->pmu_lp_core_stall;
    if (force || level != s->lp_core_clock_level) {
        s->lp_core_clock_level = level;
        qemu_set_irq(s->lp_core_clock, level);
    }
}

/*
 * Whether block reset @i is an HP block's, which an HP core reset holds:
 * all but those of LP_CLKRST_LP_RST_EN and LPPERI_RESET_EN, the LP
 * system's. (LP_CLKRST's USB, SD/MMC and EMAC resets are HP blocks'.)
 */
static bool esp32p4_clkrst_hp_block(int i)
{
    const RstDesc *d = &esp32p4_rst_desc[i];

    return d->rst.reg != R_LP_RST_EN && d->rst.reg != R_LPPERI_RESET_EN;
}

/* Whether block reset @i is one of PD_HP_CNNT's blocks. */
static bool esp32p4_clkrst_cnnt_block(int i)
{
    switch (i) {
    case ESP32P4_RST_USB_OTG20_ADP:
    case ESP32P4_RST_USB_OTG20_PHY:
    case ESP32P4_RST_USB_OTG20:
    case ESP32P4_RST_USB_OTG11:
    case ESP32P4_RST_SDMMC:
    case ESP32P4_RST_EMAC:
        return true;
    default:
        return false;
    }
}

/* [spec:nuos:req:emu.esp32p4.low-power] */
static bool esp32p4_clkrst_reset_held(ESP32P4ClkRstState *s, int i)
{
    const RstDesc *d = &esp32p4_rst_desc[i];

    if ((s->hp_core_reset || !s->pmu_power[ESP32P4_CLKRST_PD_TOP]) &&
        esp32p4_clkrst_hp_block(i)) {
        return true;
    }
    if (!s->pmu_power[ESP32P4_CLKRST_PD_HP_CNNT] &&
        esp32p4_clkrst_cnnt_block(i)) {
        return true;
    }
    if (!s->pmu_power[ESP32P4_CLKRST_PD_LP_PERI] &&
        d->rst.reg == R_LPPERI_RESET_EN) {
        return true;
    }
    if (d->norst.width && fld(s, d->norst)) {
        return false;
    }
    return fld(s, d->rst) || (d->group.width && fld(s, d->group));
}

/*
 * [spec:nuos:req:emu.esp32p4.clocks]
 * The block resets. A gated block is reset as its reset is asserted and
 * again as it is released, except in this device's own reset, which is
 * the chip's.
 */
static void esp32p4_clkrst_update_resets(ESP32P4ClkRstState *s, bool force)
{
    ESP32P4ClkRstGate *g;
    bool level;
    int i;

    for (i = 0; i < ESP32P4_NUM_RESETS; i++) {
        level = esp32p4_clkrst_reset_held(s, i);
        if (!force && level == s->reset_level[i]) {
            continue;
        }
        s->reset_level[i] = level;
        QTAILQ_FOREACH(g, &s->gates, next) {
            if (g->reset != i || g->held == level) {
                continue;
            }
            if (level && !s->resetting) {
                device_cold_reset(g->dev);
            }
            g->held = level;
            if (!level && !s->resetting) {
                device_cold_reset(g->dev);
            }
        }
        qemu_set_irq(s->reset_out[i], level);
    }
}

static void esp32p4_clkrst_update_cal_stop(ESP32P4ClkRstState *s)
{
    int i;

    for (i = 0; i < ESP32P4_CLKRST_NUM_PLL_CAL; i++) {
        qemu_set_irq(s->pll_cal_stop[i],
                     extract32(s->regs[R_ANA_PLL_CTRL0], 2 * i + 1, 1));
    }
}

static void esp32p4_clkrst_arm_pulse(ESP32P4ClkRstState *s, int n);

/*
 * The "hp-core-reset" output: high while an HP core reset is held or the
 * PMU has PD_TOP, and so the HP system, powered down.
 */
static void esp32p4_clkrst_update_hp_core_reset(ESP32P4ClkRstState *s,
                                                bool force)
{
    bool level = s->hp_core_reset || !s->pmu_power[ESP32P4_CLKRST_PD_TOP];

    if (force || level != s->hp_core_reset_level) {
        s->hp_core_reset_level = level;
        qemu_set_irq(s->hp_core_reset_out, level);
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.clocks]
 * Recompute the clock tree and everything that follows from the
 * registers, driving the outputs that change.
 */
static void esp32p4_clkrst_update(ESP32P4ClkRstState *s, bool force)
{
    ESP32P4ClkPeriod old[ESP32P4_NUM_CLOCKS];
    int i;

    if (s->recomputing) {
        s->recompute_again = true;
        return;
    }
    s->recomputing = true;
    do {
        s->recompute_again = false;
        for (i = 0; i < ESP32P4_NUM_CLOCKS; i++) {
            old[i] = s->exact[i];
            s->exact[i] = esp32p4_clkrst_compute(s, i);
            s->period[i] = esp32p4_clk_period_clock(s->exact[i]);
        }
        esp32p4_clkrst_check_soc(s);
        for (i = 0; i < ESP32P4_NUM_CLOCKS; i++) {
            esp32p4_clk_update(s->clk[i], old[i], s->exact[i]);
        }
        for (i = 0; i < ESP32P4_CLKRST_HP_CORES; i++) {
            if (s->core_pulse_waiting[i] &&
                s->period[s->core_pulse_clk[i]]) {
                esp32p4_clkrst_arm_pulse(s, i);
            }
        }
        esp32p4_clkrst_update_cores(s, force);
        esp32p4_clkrst_update_resets(s, force);
        esp32p4_clkrst_update_cal_stop(s);
        esp32p4_clkrst_update_hp_core_reset(s, force);
        force = false;
    } while (s->recompute_again);
    s->recomputing = false;
}

/* [spec:nuos:req:emu.esp32p4.clocks] */
void esp32p4_clkrst_record_reset(ESP32P4ClkRstState *s, unsigned cores,
                                 uint8_t cause)
{
    uint32_t *r = &s->regs[R_LP_RESET_CAUSE];
    int i;

    for (i = 0; i < ARRAY_SIZE(esp32p4_clkrst_cause_fields); i++) {
        if (!(cores & BIT(i))) {
            continue;
        }
        if (i == 2 && cause == CAUSE_PMU_LP_CPU &&
            (*r & LPCORE_CAUSE_MASK_PMU)) {
            continue;
        }
        *r = deposit32(*r, esp32p4_clkrst_cause_fields[i].cause_shift, 6,
                       cause);
        *r |= BIT(esp32p4_clkrst_cause_fields[i].flag_bit);
    }
}

/* [spec:nuos:req:emu.esp32p4.clocks] */
void esp32p4_clkrst_set_system_reset_cause(ESP32P4ClkRstState *s,
                                           uint8_t cause)
{
    s->system_reset_cause = cause;
}

/* [spec:nuos:req:emu.esp32p4.low-power] */
void esp32p4_clkrst_set_chip_reset_cause(ESP32P4ClkRstState *s,
                                         uint8_t cause)
{
    s->chip_reset_cause = cause;
}

/* [spec:nuos:req:emu.esp32p4.timers] */
uint8_t esp32p4_clkrst_system_reset_cause(ESP32P4ClkRstState *s)
{
    return s->system_reset_cause;
}

/* [spec:nuos:req:emu.esp32p4.parlio] */
bool esp32p4_clkrst_pad_source(ESP32P4ClkRstState *s, unsigned clk,
                               ESP32P4ClkDiv *div)
{
    const ClkDesc *d, *pd;
    uint32_t sel;
    int16_t p;
    int i;

    assert(clk < ESP32P4_NUM_CLOCKS);
    d = &esp32p4_clk_desc[clk];
    if (d->kind == SPECIAL) {
        return false;
    }
    sel = fld(s, d->sel);
    if (sel >= d->nparents) {
        return false;
    }
    p = d->parent[sel];
    if (p == PARENT_NONE || (p & PARENT_IN)) {
        return false;
    }
    /* A pad clock: the clock input of a pad, behind its own gate. */
    pd = &esp32p4_clk_desc[p];
    if (pd->kind == SPECIAL || pd->nparents != 1 ||
        pd->parent[0] == PARENT_NONE || !(pd->parent[0] & PARENT_IN)) {
        return false;
    }
    for (i = 0; i < ARRAY_SIZE(d->gate); i++) {
        if ((d->gate[i].width && !fld(s, d->gate[i])) ||
            (pd->gate[i].width && !fld(s, pd->gate[i]))) {
            return false;
        }
    }
    div->integer = 1;
    div->numerator = 0;
    div->denominator = 0;
    if (d->kind == DIV_INT || d->kind == DIV_FRAC) {
        div->integer = fld(s, d->div[0]) + 1;
    }
    if (d->kind == DIV_FRAC) {
        div->numerator = fld(s, d->div[1]);
        div->denominator = fld(s, d->div[2]);
    }
    return true;
}

/*
 * Time the current phase of core @n's CPU reset, in cycles of its clock;
 * while that clock is stopped the phase waits for it.
 */
static void esp32p4_clkrst_arm_pulse(ESP32P4ClkRstState *s, int n)
{
    unsigned clk = s->core_pulse_clk[n];

    if (!s->period[clk]) {
        s->core_pulse_waiting[n] = true;
        timer_del(s->core_timer[n]);
        return;
    }
    s->core_pulse_waiting[n] = false;
    timer_mod(s->core_timer[n], qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              esp32p4_clk_cycles_ns(s->exact[clk], s->core_pulse_cycles[n]));
}

/*
 * [spec:nuos:req:emu.esp32p4.clocks]
 * A CPU reset of HP core @n: a stall of @stall cycles if @stall_en, then
 * the reset for @length cycles, both of clock ESP32P4_CLK_<@clk>.
 */
static void esp32p4_clkrst_core_pulse(ESP32P4ClkRstState *s, int n,
                                      uint8_t cause, bool stall_en,
                                      uint32_t stall, uint32_t length,
                                      unsigned clk)
{
    esp32p4_clkrst_record_reset(s, BIT(n), cause);
    s->core_pulse_clk[n] = clk;
    s->core_pulse_reset_cycles[n] = MAX(length, 1);
    if (stall_en) {
        s->core_pulse[n] = ESP32P4_CORE_PULSE_STALL;
        s->core_pulse_cycles[n] = stall;
    } else {
        s->core_pulse[n] = ESP32P4_CORE_PULSE_RESET;
        s->core_pulse_cycles[n] = s->core_pulse_reset_cycles[n];
    }
    esp32p4_clkrst_arm_pulse(s, n);
}

static void esp32p4_clkrst_core_timer(void *opaque)
{
    ESP32P4ClkRstCoreTimer *t = opaque;
    ESP32P4ClkRstState *s = t->s;
    int n = t->n;

    if (s->core_pulse[n] == ESP32P4_CORE_PULSE_STALL) {
        s->core_pulse[n] = ESP32P4_CORE_PULSE_RESET;
        s->core_pulse_cycles[n] = s->core_pulse_reset_cycles[n];
        esp32p4_clkrst_arm_pulse(s, n);
    } else {
        s->core_pulse[n] = ESP32P4_CORE_PULSE_IDLE;
    }
    esp32p4_clkrst_update(s, false);
}

/* LP_CLKRST's CPU reset of core @n, by software or the RTC watchdog. */
static void esp32p4_clkrst_aon_core_reset(ESP32P4ClkRstState *s, int n,
                                         uint8_t cause)
{
    uint32_t ctrl = s->regs[R_LP_HPCPU_RESET_CTRL0] >> HPCORE_SHIFT(n);

    esp32p4_clkrst_core_pulse(s, n, cause, ctrl & HPCORE_STALL_EN,
                              HPCORE_STALL_WAIT(ctrl),
                              HPCORE_RESET_LENGTH(ctrl), C(LP_DYN_FAST));
}

/* The SoC clock dividers in effect out of reset. */
static void esp32p4_clkrst_reset_soc_div(ESP32P4ClkRstState *s)
{
    memset(s->soc_div, 0, sizeof(s->soc_div));
    s->soc_div[1][0] = 1;
    s->soc_div[3][0] = 1;
}

/* The HP system's registers' reset as PD_TOP or an HP core reset has it. */
static void esp32p4_clkrst_reset_hp_regs(ESP32P4ClkRstState *s)
{
    int i;

    for (i = 0; i < R_HP_END; i++) {
        s->regs[i] = esp32p4_clkrst_regs[i].reset;
    }
    esp32p4_clkrst_reset_soc_div(s);
    memset(s->sdio_edge, 0, sizeof(s->sdio_edge));
    for (i = 0; i < ESP32P4_CLKRST_HP_CORES; i++) {
        s->core_pulse[i] = ESP32P4_CORE_PULSE_IDLE;
        s->core_pulse_waiting[i] = false;
        timer_del(s->core_timer[i]);
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.timers]
 * An HP core reset: HP_SYS_CLKRST's registers and the dividers they set
 * take their reset values, which hold core 1 again; CPU resets in progress
 * end; and both HP cores and every HP block are held in reset until
 * hp_core_timer runs. The LP system's registers, LP_CLKRST's and LPPERI's,
 * keep their values, and the reset vector LP_CLKRST selects with them.
 */
void esp32p4_clkrst_hp_core_reset(ESP32P4ClkRstState *s, unsigned source,
                                  int64_t length_ns)
{
    uint8_t cause = ESP32P4_RESET_CAUSE_RWDT_CORE;

    if (source == ESP32P4_HP_CORE_RESET_SW) {
        cause = ESP32P4_RESET_CAUSE_SW_SYS;
    } else if (source != ESP32P4_HP_CORE_RESET_RWDT) {
        unsigned mwdt = source - ESP32P4_HP_CORE_RESET_MWDT0;
        uint32_t sel = s->regs[R_HPCORE_WDT_RESET_SOURCE0];

        if (extract32(sel, 0, 1) != mwdt && extract32(sel, 1, 1) != mwdt) {
            return;
        }
        cause = ESP32P4_RESET_CAUSE_MWDT_CORE;
    }
    esp32p4_clkrst_record_reset(s, ESP32P4_CLKRST_CORE_HP0 |
                                ESP32P4_CLKRST_CORE_HP1, cause);
    esp32p4_clkrst_reset_hp_regs(s);
    s->hp_core_reset = true;
    timer_mod(s->hp_core_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              MAX(length_ns, 1));
    esp32p4_clkrst_update(s, false);
}

static void esp32p4_clkrst_hp_core_timer(void *opaque)
{
    ESP32P4ClkRstState *s = opaque;

    s->hp_core_reset = false;
    esp32p4_clkrst_update(s, false);
}

/* The value a read of register @idx returns. */
static uint32_t esp32p4_clkrst_value(ESP32P4ClkRstState *s, unsigned idx)
{
    uint32_t v = s->regs[idx];
    uint64_t hz;
    int i, src;

    switch (idx) {
    case R_ANA_PLL_CTRL0:
        for (i = 0; i < ESP32P4_CLKRST_NUM_PLL_CAL; i++) {
            v = deposit32(v, 2 * i, 1, s->pll_cal_end[i]);
        }
        return v;
    case R_CPU_SRC_FREQ0:
        src = extract32(s->regs[R_LP_HP_CLK_CTRL], 0, 2);
        if (src >= esp32p4_clk_desc[C(ROOT)].nparents) {
            return 0;
        }
        /* The source's whole cycles a second, at its exact period. */
        hz = esp32p4_clk_ns_cycles(
            esp32p4_clkrst_parent(s, esp32p4_clk_desc[C(ROOT)].parent[src]),
            NANOSECONDS_PER_SECOND);
        return (hz * 4 + 500000) / 1000000;
    case R_CPU_CLK_STATUS0:
        v = (s->soc_div[0][0] || (s->soc_div[0][2] && s->soc_div[0][1])) << 1;
        v |= (extract32(s->regs[R_LP_HP_CLK_CTRL], 0, 2) ==
              ROOT_CLK_SRC_CPLL) << 2;
        v |= s->soc_div[0][0] << 3;
        v |= s->soc_div[0][1] << 11;
        v |= s->soc_div[0][2] << 19;
        return v;
    default:
        return v;
    }
}

/* The register a block's address decodes to, or -1. */
static int esp32p4_clkrst_index(int block, hwaddr addr)
{
    switch (block) {
    case ESP32P4_CLKRST_MMIO_HP:
        return addr < R_HP_END * 4 ? addr / 4 : -1;
    case ESP32P4_CLKRST_MMIO_LP:
        if (addr < 0x50) {
            return R_LP_CLK_CONF + addr / 4;
        }
        return addr == 0x3fc ? R_LP_DATE : -1;
    case ESP32P4_CLKRST_MMIO_LPPERI:
    default:
        if (addr < 0x10 || (addr >= 0x28 && addr < 0x40)) {
            return R_LPPERI_CLK_EN + addr / 4;
        }
        return addr == 0x3fc ? R_LPPERI_DATE : -1;
    }
}

static const char *const esp32p4_clkrst_block_names[] = {
    "HP_SYS_CLKRST", "LP_CLKRST", "LPPERI",
};

static uint64_t esp32p4_clkrst_read(ESP32P4ClkRstState *s, int block,
                                    hwaddr addr, unsigned size)
{
    int idx = esp32p4_clkrst_index(block, addr & ~3);

    if (idx < 0) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of reserved address "
                      "0x%" HWADDR_PRIx " in %s\n", TYPE_ESP32P4_CLKRST,
                      addr, esp32p4_clkrst_block_names[block]);
        return 0;
    }
    return extract32(esp32p4_clkrst_value(s, idx), (addr & 3) * 8, size * 8);
}

/* Log software stopping what the model keeps running. */
static void esp32p4_clkrst_check_kept(unsigned idx, uint32_t old,
                                      uint32_t new)
{
    uint32_t stopped = 0;

    switch (idx) {
    case R_SOC_CLK_CTRL0:
        stopped = old & ~new & KEPT_SOC_CLK_CTRL0;
        break;
    case R_SOC_CLK_CTRL2:
        stopped = old & ~new & KEPT_SOC_CLK_CTRL2;
        break;
    case R_SOC_CLK_CTRL3:
        stopped = old & ~new & KEPT_SOC_CLK_CTRL3;
        break;
    case R_HP_RST_EN0:
        stopped = new & ~old & KEPT_HP_RST_EN0;
        break;
    case R_LP_CLK_EN:
        stopped = old & ~new & BIT(27);
        break;
    case R_LP_RST_EN:
        stopped = new & ~old & BIT(31);
        break;
    case R_LPPERI_RESET_EN:
        stopped = new & ~old & BIT(30);
        break;
    case R_CLK_FORCE_ON_CTRL0:
        if (old & ~new & L2MEM_MEM_CLK_FORCE_ON) {
            qemu_log_mask(LOG_UNIMP, "%s: L2MEM's clock gating, which a DMA "
                          "access to L2MEM does not undo, is not modelled\n",
                          TYPE_ESP32P4_CLKRST);
        }
        break;
    default:
        break;
    }
    if (stopped) {
        qemu_log_mask(LOG_UNIMP, "%s: %s bits 0x%08" PRIx32 " stop or reset "
                      "a part of the system the model keeps running\n",
                      TYPE_ESP32P4_CLKRST, esp32p4_clkrst_regs[idx].name,
                      stopped);
    }
}

/* [spec:nuos:req:emu.esp32p4.clocks] */
static void esp32p4_clkrst_write(ESP32P4ClkRstState *s, int block,
                                 hwaddr addr, uint64_t value, unsigned size)
{
    int idx = esp32p4_clkrst_index(block, addr & ~3);
    uint32_t lanes, v, rw, wt, old, *r;
    int n;

    if (idx < 0) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to reserved address "
                      "0x%" HWADDR_PRIx " in %s\n", TYPE_ESP32P4_CLKRST,
                      addr, esp32p4_clkrst_block_names[block]);
        return;
    }
    lanes = deposit32(0, (addr & 3) * 8, size * 8, UINT32_MAX);
    v = deposit32(0, (addr & 3) * 8, size * 8, value);
    rw = esp32p4_clkrst_regs[idx].rw & lanes;
    wt = esp32p4_clkrst_regs[idx].wt & lanes & v;
    r = &s->regs[idx];
    old = *r;
    *r = (old & ~rw) | (v & rw);
    esp32p4_clkrst_check_kept(idx, old, *r);

    switch (idx) {
    case R_ROOT_CLK_CTRL0:
        if (wt & SOC_CLK_DIV_UPDATE) {
            uint32_t c0 = s->regs[R_ROOT_CLK_CTRL0];
            uint32_t c1 = s->regs[R_ROOT_CLK_CTRL1];
            uint32_t c2 = s->regs[R_ROOT_CLK_CTRL2];
            uint32_t c3 = s->regs[R_ROOT_CLK_CTRL3];

            s->soc_div[0][0] = extract32(c0, 5, 8);
            s->soc_div[0][1] = extract32(c0, 13, 8);
            s->soc_div[0][2] = extract32(c0, 21, 8);
            s->soc_div[1][0] = extract32(c1, 0, 8);
            s->soc_div[1][1] = extract32(c1, 8, 8);
            s->soc_div[1][2] = extract32(c1, 16, 8);
            s->soc_div[2][0] = extract32(c1, 24, 8);
            s->soc_div[2][1] = extract32(c2, 0, 8);
            s->soc_div[2][2] = extract32(c2, 8, 8);
            s->soc_div[3][0] = extract32(c2, 16, 8);
            s->soc_div[3][1] = extract32(c2, 24, 8);
            s->soc_div[3][2] = extract32(c3, 0, 8);
        }
        break;
    case R_PERI_CLK_CTRL02:
        if (wt & SDIO_LS_CLK_EDGE_CFG_UPDATE) {
            s->sdio_edge[0] = extract32(*r, 9, 4);
            s->sdio_edge[1] = extract32(*r, 13, 4);
            s->sdio_edge[2] = extract32(*r, 17, 4);
        }
        break;
    case R_DBG_CLK_CTRL0:
    case R_DBG_CLK_CTRL1:
        for (n = 0; n < 3; n++) {
            uint32_t sel = extract32(s->regs[R_DBG_CLK_CTRL0], 8 * n, 8);
            int k;

            if (!extract32(s->regs[R_DBG_CLK_CTRL1], 16 + n, 1) ||
                sel == CLKOUT_NONE) {
                continue;
            }
            for (k = 0; k < ARRAY_SIZE(esp32p4_clkout_sigs); k++) {
                if (esp32p4_clkout_sigs[k].sig == sel) {
                    break;
                }
            }
            if (k == ARRAY_SIZE(esp32p4_clkout_sigs)) {
                qemu_log_mask(LOG_UNIMP, "%s: debug clock output %d: signal "
                              "%" PRIu32 " is not modelled\n",
                              TYPE_ESP32P4_CLKRST, n, sel);
            }
        }
        break;
    case R_DPA_CTRL0:
        if (*r & ~old) {
            qemu_log_mask(LOG_UNIMP, "%s: the crypto clock's random divider "
                          "against DPA is not modelled\n",
                          TYPE_ESP32P4_CLKRST);
        }
        break;
    case R_LP_RESET_CAUSE:
        for (n = 0; n < ARRAY_SIZE(esp32p4_clkrst_cause_fields); n++) {
            if (wt & BIT(esp32p4_clkrst_cause_fields[n].cause_clr)) {
                *r = deposit32(*r, esp32p4_clkrst_cause_fields[n].cause_shift,
                               6, 0);
            }
            if (wt & BIT(esp32p4_clkrst_cause_fields[n].flag_clr)) {
                *r &= ~BIT(esp32p4_clkrst_cause_fields[n].flag_bit);
            }
        }
        break;
    case R_LP_HPCPU_RESET_CTRL0:
        for (n = 0; n < ESP32P4_CLKRST_HP_CORES; n++) {
            uint32_t set = *r & ~old;

            if (set & (HPCORE_OCD_HALT_ON_RESET << HPCORE_SHIFT(n))) {
                qemu_log_mask(LOG_UNIMP, "%s: HPCORE%d_OCD_HALT_ON_RESET: "
                              "halting for the debugger is not modelled\n",
                              TYPE_ESP32P4_CLKRST, n);
            }
            if (set & (HPCORE_LOCKUP_RESET_EN << HPCORE_SHIFT(n))) {
                qemu_log_mask(LOG_UNIMP, "%s: HPCORE%d_LOCKUP_RESET_EN: the "
                              "cores do not report a lockup\n",
                              TYPE_ESP32P4_CLKRST, n);
            }
            if (wt & (HPCORE_SW_RESET << HPCORE_SHIFT(n))) {
                esp32p4_clkrst_aon_core_reset(s, n,
                                             ESP32P4_RESET_CAUSE_SW_CPU);
            }
        }
        break;
    case R_LP_HPCPU_RESET_CTRL1:
        if (*r & ~old) {
            qemu_log_mask(LOG_UNIMP, "%s: the software stall codes are not "
                          "modelled\n", TYPE_ESP32P4_CLKRST);
        }
        break;
    case R_LPPERI_RESET_EN:
        if (wt & RST_EN_LP_CORE) {
            esp32p4_clkrst_record_reset(s, ESP32P4_CLKRST_CORE_LP,
                                        ESP32P4_RESET_CAUSE_SW_LP_CPU);
            qemu_set_irq(s->lp_core_reset, 1);
            qemu_set_irq(s->lp_core_reset, s->lp_core_reset_level);
        }
        break;
    case R_LPPERI_MEM_CTRL:
        if ((wt & LP_UART_WAKEUP_FLAG_CLR) ||
            (v & lanes & LP_UART_WAKEUP_FLAG)) {
            *r &= ~LP_UART_WAKEUP_FLAG;
        }
        break;
    default:
        break;
    }
    esp32p4_clkrst_update(s, false);
}

static uint64_t esp32p4_clkrst_hp_read(void *opaque, hwaddr addr,
                                       unsigned size)
{
    return esp32p4_clkrst_read(opaque, ESP32P4_CLKRST_MMIO_HP, addr, size);
}

static void esp32p4_clkrst_hp_write(void *opaque, hwaddr addr,
                                    uint64_t value, unsigned size)
{
    esp32p4_clkrst_write(opaque, ESP32P4_CLKRST_MMIO_HP, addr, value, size);
}

static uint64_t esp32p4_clkrst_lp_read(void *opaque, hwaddr addr,
                                       unsigned size)
{
    return esp32p4_clkrst_read(opaque, ESP32P4_CLKRST_MMIO_LP, addr, size);
}

static void esp32p4_clkrst_lp_write(void *opaque, hwaddr addr,
                                    uint64_t value, unsigned size)
{
    esp32p4_clkrst_write(opaque, ESP32P4_CLKRST_MMIO_LP, addr, value, size);
}

static uint64_t esp32p4_clkrst_lpperi_read(void *opaque, hwaddr addr,
                                           unsigned size)
{
    return esp32p4_clkrst_read(opaque, ESP32P4_CLKRST_MMIO_LPPERI, addr,
                               size);
}

static void esp32p4_clkrst_lpperi_write(void *opaque, hwaddr addr,
                                        uint64_t value, unsigned size)
{
    esp32p4_clkrst_write(opaque, ESP32P4_CLKRST_MMIO_LPPERI, addr, value,
                         size);
}

#define ESP32P4_CLKRST_OPS(r, w) { \
    .read = (r), \
    .write = (w), \
    .endianness = DEVICE_LITTLE_ENDIAN, \
    .valid.min_access_size = 1, \
    .valid.max_access_size = 4, \
    .impl.min_access_size = 1, \
    .impl.max_access_size = 4, \
}

static const MemoryRegionOps esp32p4_clkrst_ops[3] = {
    ESP32P4_CLKRST_OPS(esp32p4_clkrst_hp_read, esp32p4_clkrst_hp_write),
    ESP32P4_CLKRST_OPS(esp32p4_clkrst_lp_read, esp32p4_clkrst_lp_write),
    ESP32P4_CLKRST_OPS(esp32p4_clkrst_lpperi_read,
                       esp32p4_clkrst_lpperi_write),
};

static void esp32p4_clkrst_set_cal_end(void *opaque, int n, int level)
{
    ESP32P4_CLKRST(opaque)->pll_cal_end[n] = level;
}

/* [spec:nuos:req:emu.esp32p4.clocks] */
static void esp32p4_clkrst_set_pmu_xpd(void *opaque, int n, int level)
{
    ESP32P4ClkRstState *s = opaque;

    s->pmu_xpd[n] = level;
    esp32p4_clkrst_update(s, false);
}

static void esp32p4_clkrst_set_pmu_core_stall(void *opaque, int n, int level)
{
    ESP32P4ClkRstState *s = opaque;

    s->pmu_core_stall[n] = level;
    esp32p4_clkrst_update(s, false);
}

static void esp32p4_clkrst_set_pmu_lp_core_reset(void *opaque, int n,
                                                 int level)
{
    ESP32P4ClkRstState *s = opaque;

    s->pmu_lp_core_reset = level;
    esp32p4_clkrst_update(s, false);
}

static void esp32p4_clkrst_set_pmu_lp_core_clock(void *opaque, int n,
                                                 int level)
{
    ESP32P4ClkRstState *s = opaque;

    s->pmu_lp_core_clock = level;
    esp32p4_clkrst_update(s, false);
}

static void esp32p4_clkrst_set_pmu_lp_core_stall(void *opaque, int n,
                                                 int level)
{
    ESP32P4ClkRstState *s = opaque;

    s->pmu_lp_core_stall = level;
    esp32p4_clkrst_update(s, false);
}

/*
 * [spec:nuos:req:emu.esp32p4.low-power]
 * The PMU powers domain @n down or up. Powered down, PD_TOP's registers
 * here lose their values and PD_LP_PERI's, LPPERI's, theirs; the domain's
 * blocks are held in reset until it powers up, which records the PMU's
 * power-down reset for the cores it held, unless this device is in a
 * reset of its own, which records its cause.
 */
static void esp32p4_clkrst_set_pmu_power(void *opaque, int n, int level)
{
    ESP32P4ClkRstState *s = opaque;
    bool on = level;
    bool in_reset = resettable_is_in_reset(OBJECT(s));
    int i;

    if (s->pmu_power[n] == on) {
        return;
    }
    s->pmu_power[n] = on;
    switch (n) {
    case ESP32P4_CLKRST_PD_TOP:
        if (!on) {
            esp32p4_clkrst_reset_hp_regs(s);
        }
        /* fall through */
    case ESP32P4_CLKRST_PD_HP_CPU:
        if (!on) {
            for (i = 0; i < ESP32P4_CLKRST_HP_CORES; i++) {
                s->core_pulse[i] = ESP32P4_CORE_PULSE_IDLE;
                s->core_pulse_waiting[i] = false;
                timer_del(s->core_timer[i]);
            }
        } else if (!in_reset && !esp32p4_clkrst_hp_cores_off(s)) {
            esp32p4_clkrst_record_reset(s, ESP32P4_CLKRST_CORE_HP0 |
                                        ESP32P4_CLKRST_CORE_HP1,
                                        ESP32P4_RESET_CAUSE_PMU_HP_PD);
        }
        break;
    case ESP32P4_CLKRST_PD_LP_PERI:
        if (!on) {
            for (i = R_LPPERI_CLK_EN; i < ESP32P4_CLKRST_NREGS; i++) {
                s->regs[i] = esp32p4_clkrst_regs[i].reset;
            }
        } else if (!in_reset) {
            esp32p4_clkrst_record_reset(s, ESP32P4_CLKRST_CORE_LP,
                                        ESP32P4_RESET_CAUSE_PMU_LP_PD);
        }
        break;
    default:
        break;
    }
    esp32p4_clkrst_update(s, false);
}

/* [spec:nuos:req:emu.esp32p4.low-power] */
static void esp32p4_clkrst_set_pmu_icg(void *opaque, int n, int level)
{
    ESP32P4ClkRstState *s = opaque;

    s->pmu_icg[n] = level;
    esp32p4_clkrst_update(s, false);
}

/* [spec:nuos:req:emu.esp32p4.low-power] */
static void esp32p4_clkrst_set_pmu_sysclk(void *opaque, int n, int level)
{
    ESP32P4ClkRstState *s = opaque;

    s->pmu_sysclk[n] = level;
    esp32p4_clkrst_update(s, false);
}

/* [spec:nuos:req:emu.esp32p4.low-power] */
static void esp32p4_clkrst_set_pmu_lp_sleep(void *opaque, int n, int level)
{
    ESP32P4ClkRstState *s = opaque;

    s->pmu_lp_sleep = level;
    esp32p4_clkrst_update(s, false);
}

/*
 * [spec:nuos:req:emu.esp32p4.clocks]
 * [spec:nuos:req:emu.esp32p4.timers]
 * An MWDT asks for a CPU reset of core n (line ESP32P4_CLKRST_MWDT_LINE):
 * it takes place if HPCORE<n>_WDT_RESET_SOURCE_SEL picks that MWDT, as the
 * core's HPWDT_CORE<n>_RST_CTRL0 configures it.
 */
static void esp32p4_clkrst_mwdt_reset(void *opaque, int line, int level)
{
    ESP32P4ClkRstState *s = opaque;
    int mwdt = line / ESP32P4_CLKRST_HP_CORES;
    int n = line % ESP32P4_CLKRST_HP_CORES;
    uint32_t ctrl;

    if (!level ||
        extract32(s->regs[R_HPCORE_WDT_RESET_SOURCE0], n, 1) != mwdt) {
        return;
    }
    ctrl = s->regs[R_HPWDT_CORE0_RST_CTRL0 + n];
    esp32p4_clkrst_core_pulse(s, n, ESP32P4_RESET_CAUSE_MWDT_CPU,
                              ctrl & HPWDT_STALL_EN,
                              HPWDT_STALL_WAIT_NUM(ctrl),
                              HPWDT_RST_LEN(ctrl), C(XTAL));
    esp32p4_clkrst_update(s, false);
}

/*
 * [spec:nuos:req:emu.esp32p4.clocks]
 * [spec:nuos:req:emu.esp32p4.timers]
 * The RTC watchdog asks for a CPU reset of core @n: it takes place if
 * LP_WDT_HPCORE<n>_RESET_EN is set.
 */
static void esp32p4_clkrst_rwdt_reset(void *opaque, int n, int level)
{
    ESP32P4ClkRstState *s = opaque;

    if (!level || !(s->regs[R_LP_HPCPU_RESET_CTRL0] &
                    (HPCORE_LP_WDT_RESET_EN << HPCORE_SHIFT(n)))) {
        return;
    }
    esp32p4_clkrst_aon_core_reset(s, n, ESP32P4_RESET_CAUSE_RWDT_CPU);
    esp32p4_clkrst_update(s, false);
}

static void esp32p4_clkrst_lp_uart_wakeup(void *opaque, int n, int level)
{
    ESP32P4ClkRstState *s = opaque;

    if (level && (s->regs[R_LPPERI_MEM_CTRL] & LP_UART_WAKEUP_EN)) {
        s->regs[R_LPPERI_MEM_CTRL] |= LP_UART_WAKEUP_FLAG;
    }
}

static void esp32p4_clkrst_clock_in_update(void *opaque, ClockEvent event)
{
    esp32p4_clkrst_update(opaque, false);
}

/* [spec:nuos:req:emu.esp32p4.clocks] */
static MemTxResult esp32p4_clkrst_gate_read(void *opaque, hwaddr addr,
                                            uint64_t *data, unsigned size,
                                            MemTxAttrs attrs)
{
    ESP32P4ClkRstGate *g = opaque;
    bool running = !g->held && clock_is_enabled(g->s->clk[g->clock]);
    uint8_t buf[8];
    MemTxResult r;

    if (!running && g->has_volatile &&
        addr < g->volatile_offset + g->volatile_size &&
        addr + size > g->volatile_offset) {
        *data = 0;
        return MEMTX_OK;
    }
    r = address_space_read(&g->dev_as, addr, attrs, buf, size);
    *data = ldn_le_p(buf, size);
    return r;
}

/* [spec:nuos:req:emu.esp32p4.clocks] */
static MemTxResult esp32p4_clkrst_gate_write(void *opaque, hwaddr addr,
                                             uint64_t data, unsigned size,
                                             MemTxAttrs attrs)
{
    ESP32P4ClkRstGate *g = opaque;
    uint8_t buf[8];

    if (g->held || !clock_is_enabled(g->s->clk[g->clock])) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to %s at 0x%" HWADDR_PRIx
                      " while its clock is gated or it is held in reset\n",
                      TYPE_ESP32P4_CLKRST, memory_region_name(&g->iomem),
                      addr);
        return MEMTX_OK;
    }
    stn_le_p(buf, size, data);
    return address_space_write(&g->dev_as, addr, attrs, buf, size);
}

static const MemoryRegionOps esp32p4_clkrst_gate_ops = {
    .read_with_attrs = esp32p4_clkrst_gate_read,
    .write_with_attrs = esp32p4_clkrst_gate_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

/* [spec:nuos:req:emu.esp32p4.clocks] */
ESP32P4ClkRstGate *esp32p4_clkrst_map_gated(ESP32P4ClkRstState *s,
                                            SysBusDevice *dev, int n,
                                            hwaddr addr, unsigned bus_clock,
                                            unsigned reset, int priority)
{
    ESP32P4ClkRstGate *g = g_new0(ESP32P4ClkRstGate, 1);
    MemoryRegion *mr = sysbus_mmio_get_region(dev, n);
    g_autofree char *as_name = g_strdup_printf("%s-gate",
                                               memory_region_name(mr));

    assert(bus_clock < ESP32P4_NUM_CLOCKS &&
           reset <= ESP32P4_CLKRST_GATE_NO_RESET);
    g->s = s;
    g->dev = DEVICE(dev);
    g->clock = bus_clock;
    g->reset = reset;
    g->held = reset != ESP32P4_CLKRST_GATE_NO_RESET && s->reset_level[reset];
    address_space_init(&g->dev_as, mr, as_name);
    /*
     * No owner: an owner's re-entrancy guard would block the forwarded
     * access to the block's own region.
     */
    memory_region_init_io(&g->iomem, NULL, &esp32p4_clkrst_gate_ops, g,
                          memory_region_name(mr), memory_region_size(mr));
    memory_region_add_subregion_overlap(get_system_memory(), addr, &g->iomem,
                                        priority);
    QTAILQ_INSERT_TAIL(&s->gates, g, next);
    return g;
}

void esp32p4_clkrst_gate_volatile(ESP32P4ClkRstGate *g, hwaddr offset,
                                  unsigned size)
{
    g->has_volatile = true;
    g->volatile_offset = offset;
    g->volatile_size = size;
}

/*
 * [spec:nuos:req:emu.esp32p4.clocks]
 * A chip or system reset: every register takes its reset value, the
 * dividers in effect theirs, the CPU resets in progress end, and
 * LP_CLKRST_RESET_CAUSE records the reset for every core, as a chip reset
 * unless a block has given another cause.
 */
static void esp32p4_clkrst_reset_hold(Object *obj, ResetType type)
{
    ESP32P4ClkRstState *s = ESP32P4_CLKRST(obj);
    int i;

    for (i = 0; i < ESP32P4_CLKRST_NREGS; i++) {
        s->regs[i] = esp32p4_clkrst_regs[i].reset;
    }
    esp32p4_clkrst_reset_soc_div(s);
    memset(s->sdio_edge, 0, sizeof(s->sdio_edge));
    for (i = 0; i < ESP32P4_CLKRST_HP_CORES; i++) {
        s->core_pulse[i] = ESP32P4_CORE_PULSE_IDLE;
        s->core_pulse_waiting[i] = false;
        timer_del(s->core_timer[i]);
    }
    s->hp_core_reset = false;
    timer_del(s->hp_core_timer);
    esp32p4_clkrst_record_reset(s, ESP32P4_CLKRST_CORE_HP0 |
                                ESP32P4_CLKRST_CORE_HP1 |
                                ESP32P4_CLKRST_CORE_LP,
                                s->system_reset_cause ?:
                                s->chip_reset_cause ?:
                                ESP32P4_RESET_CAUSE_CHIP);
    s->system_reset_cause = 0;
    s->chip_reset_cause = 0;
    s->resetting = true;
    esp32p4_clkrst_update(s, true);
    s->resetting = false;
}

static void esp32p4_clkrst_init(Object *obj)
{
    ESP32P4ClkRstState *s = ESP32P4_CLKRST(obj);
    DeviceState *dev = DEVICE(obj);
    static const char *const region_names[3] = {
        "esp32p4.hp-sys-clkrst", "esp32p4.lp-clkrst", "esp32p4.lp-peri",
    };
    int i;

    for (i = 0; i < 3; i++) {
        memory_region_init_io(&s->iomem[i], obj, &esp32p4_clkrst_ops[i], s,
                              region_names[i], ESP32P4_CLKRST_BLOCK_SIZE);
        sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem[i]);
    }
    for (i = 0; i < ESP32P4_NUM_CLKIN; i++) {
        s->clk_in[i] = qdev_init_clock_in(dev, esp32p4_clock_input_names[i],
                                          esp32p4_clkrst_clock_in_update, s,
                                          ClockUpdate);
    }
    s->clk_ids = g_hash_table_new(NULL, NULL);
    for (i = 0; i < ESP32P4_NUM_CLOCKS; i++) {
        s->clk[i] = qdev_init_clock_out(dev, esp32p4_clock_names[i]);
        g_hash_table_insert(s->clk_ids, s->clk[i], GINT_TO_POINTER(i + 1));
    }

    qdev_init_gpio_out_named(dev, s->reset_out, ESP32P4_CLKRST_RESET,
                             ESP32P4_NUM_RESETS);
    qdev_init_gpio_out_named(dev, s->core_reset, ESP32P4_CLKRST_CORE_RESET,
                             ESP32P4_CLKRST_HP_CORES);
    qdev_init_gpio_out_named(dev, s->core_clock, ESP32P4_CLKRST_CORE_CLOCK,
                             ESP32P4_CLKRST_HP_CORES);
    qdev_init_gpio_out_named(dev, s->core_vector, ESP32P4_CLKRST_CORE_VECTOR,
                             ESP32P4_CLKRST_HP_CORES);
    qdev_init_gpio_out_named(dev, &s->lp_core_reset,
                             ESP32P4_CLKRST_LP_CORE_RESET, 1);
    qdev_init_gpio_out_named(dev, &s->lp_core_clock,
                             ESP32P4_CLKRST_LP_CORE_CLOCK, 1);
    qdev_init_gpio_out_named(dev, s->pll_cal_stop,
                             ESP32P4_CLKRST_PLL_CAL_STOP,
                             ESP32P4_CLKRST_NUM_PLL_CAL);
    qdev_init_gpio_out_named(dev, s->core_stall, ESP32P4_CLKRST_CORE_STALL,
                             ESP32P4_CLKRST_HP_CORES);
    qdev_init_gpio_out_named(dev, &s->hp_core_reset_out,
                             ESP32P4_CLKRST_HP_CORE_RESET, 1);

    qdev_init_gpio_in_named(dev, esp32p4_clkrst_set_cal_end,
                            ESP32P4_CLKRST_PLL_CAL_END,
                            ESP32P4_CLKRST_NUM_PLL_CAL);
    qdev_init_gpio_in_named(dev, esp32p4_clkrst_set_pmu_xpd,
                            ESP32P4_CLKRST_PMU_XPD, ESP32P4_CLKRST_NUM_XPD);
    qdev_init_gpio_in_named(dev, esp32p4_clkrst_set_pmu_core_stall,
                            ESP32P4_CLKRST_PMU_CORE_STALL,
                            ESP32P4_CLKRST_HP_CORES);
    qdev_init_gpio_in_named(dev, esp32p4_clkrst_set_pmu_lp_core_reset,
                            ESP32P4_CLKRST_PMU_LP_CORE_RESET, 1);
    qdev_init_gpio_in_named(dev, esp32p4_clkrst_set_pmu_lp_core_clock,
                            ESP32P4_CLKRST_PMU_LP_CORE_CLOCK, 1);
    qdev_init_gpio_in_named(dev, esp32p4_clkrst_set_pmu_lp_core_stall,
                            ESP32P4_CLKRST_PMU_LP_CORE_STALL, 1);
    qdev_init_gpio_in_named(dev, esp32p4_clkrst_set_pmu_power,
                            ESP32P4_CLKRST_PMU_POWER, ESP32P4_CLKRST_NUM_PD);
    qdev_init_gpio_in_named(dev, esp32p4_clkrst_set_pmu_icg,
                            ESP32P4_CLKRST_PMU_ICG,
                            ESP32P4_CLKRST_NUM_PMU_ICG);
    qdev_init_gpio_in_named(dev, esp32p4_clkrst_set_pmu_sysclk,
                            ESP32P4_CLKRST_PMU_SYSCLK,
                            ESP32P4_CLKRST_NUM_SYSCLK);
    qdev_init_gpio_in_named(dev, esp32p4_clkrst_set_pmu_lp_sleep,
                            ESP32P4_CLKRST_PMU_LP_SLEEP, 1);
    qdev_init_gpio_in_named(dev, esp32p4_clkrst_mwdt_reset,
                            ESP32P4_CLKRST_MWDT_CORE_RESET,
                            ESP32P4_CLKRST_MWDTS * ESP32P4_CLKRST_HP_CORES);
    qdev_init_gpio_in_named(dev, esp32p4_clkrst_rwdt_reset,
                            ESP32P4_CLKRST_RWDT_CORE_RESET,
                            ESP32P4_CLKRST_HP_CORES);
    qdev_init_gpio_in_named(dev, esp32p4_clkrst_lp_uart_wakeup,
                            ESP32P4_CLKRST_LP_UART_WAKEUP, 1);

    for (i = 0; i < ESP32P4_CLKRST_HP_CORES; i++) {
        s->core_timer_ctx[i].s = s;
        s->core_timer_ctx[i].n = i;
        s->core_timer[i] = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                        esp32p4_clkrst_core_timer,
                                        &s->core_timer_ctx[i]);
    }
    s->hp_core_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                    esp32p4_clkrst_hp_core_timer, s);

    /* The PMU's levels until it drives them; see the header. */
    s->pmu_xpd[ESP32P4_CLKRST_XPD_RC_FAST] = true;
    s->pmu_xpd[ESP32P4_CLKRST_XPD_XTAL] = true;
    s->pmu_lp_core_reset = true;
    for (i = 0; i < ESP32P4_CLKRST_NUM_PD; i++) {
        s->pmu_power[i] = true;
    }
    for (i = 0; i < ESP32P4_CLKRST_NUM_PMU_ICG; i++) {
        s->pmu_icg[i] = true;
    }

    QTAILQ_INIT(&s->gates);
}

static void esp32p4_clkrst_finalize(Object *obj)
{
    g_hash_table_destroy(ESP32P4_CLKRST(obj)->clk_ids);
}

static void esp32p4_clkrst_realize(DeviceState *dev, Error **errp)
{
    int i, j;

    /* The tree is computed in table order: a clock follows its parents. */
    for (i = 0; i < ESP32P4_NUM_CLOCKS; i++) {
        const ClkDesc *d = &esp32p4_clk_desc[i];

        for (j = 0; j < d->nparents; j++) {
            assert(d->parent[j] == PARENT_NONE ||
                   (d->parent[j] & PARENT_IN) || d->parent[j] < i);
        }
        assert(d->kind == SPECIAL || d->nparents);
    }
}

static int esp32p4_clkrst_post_load(void *opaque, int version_id)
{
    ESP32P4ClkRstState *s = opaque;
    ESP32P4ClkRstGate *g;
    int i;

    for (i = 0; i < ESP32P4_NUM_CLOCKS; i++) {
        s->exact[i] = esp32p4_clkrst_compute(s, i);
        s->period[i] = esp32p4_clk_period_clock(s->exact[i]);
        clock_set(s->clk[i], s->period[i]);
    }
    for (i = 0; i < ESP32P4_NUM_RESETS; i++) {
        s->reset_level[i] = esp32p4_clkrst_reset_held(s, i);
    }
    QTAILQ_FOREACH(g, &s->gates, next) {
        g->held = g->reset != ESP32P4_CLKRST_GATE_NO_RESET &&
                  s->reset_level[g->reset];
    }
    return 0;
}

static const VMStateDescription vmstate_esp32p4_clkrst = {
    .name = TYPE_ESP32P4_CLKRST,
    .version_id = 2,
    .minimum_version_id = 2,
    .post_load = esp32p4_clkrst_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, ESP32P4ClkRstState, ESP32P4_CLKRST_NREGS),
        VMSTATE_UINT8_2DARRAY(soc_div, ESP32P4ClkRstState, 4, 3),
        VMSTATE_UINT8_ARRAY(sdio_edge, ESP32P4ClkRstState, 3),
        VMSTATE_ARRAY_CLOCK(clk_in, ESP32P4ClkRstState, ESP32P4_NUM_CLKIN),
        VMSTATE_BOOL_ARRAY(pll_cal_end, ESP32P4ClkRstState,
                           ESP32P4_CLKRST_NUM_PLL_CAL),
        VMSTATE_BOOL_ARRAY(pmu_xpd, ESP32P4ClkRstState,
                           ESP32P4_CLKRST_NUM_XPD),
        VMSTATE_BOOL_ARRAY(pmu_core_stall, ESP32P4ClkRstState,
                           ESP32P4_CLKRST_HP_CORES),
        VMSTATE_BOOL(pmu_lp_core_reset, ESP32P4ClkRstState),
        VMSTATE_BOOL(pmu_lp_core_clock, ESP32P4ClkRstState),
        VMSTATE_BOOL(pmu_lp_core_stall, ESP32P4ClkRstState),
        VMSTATE_BOOL_ARRAY(pmu_power, ESP32P4ClkRstState,
                           ESP32P4_CLKRST_NUM_PD),
        VMSTATE_BOOL_ARRAY(pmu_icg, ESP32P4ClkRstState,
                           ESP32P4_CLKRST_NUM_PMU_ICG),
        VMSTATE_BOOL_ARRAY(pmu_sysclk, ESP32P4ClkRstState,
                           ESP32P4_CLKRST_NUM_SYSCLK),
        VMSTATE_BOOL(pmu_lp_sleep, ESP32P4ClkRstState),
        VMSTATE_UINT32_ARRAY(core_pulse, ESP32P4ClkRstState,
                             ESP32P4_CLKRST_HP_CORES),
        VMSTATE_TIMER_PTR_ARRAY(core_timer, ESP32P4ClkRstState,
                                ESP32P4_CLKRST_HP_CORES),
        VMSTATE_UINT32_ARRAY(core_pulse_cycles, ESP32P4ClkRstState,
                             ESP32P4_CLKRST_HP_CORES),
        VMSTATE_UINT32_ARRAY(core_pulse_reset_cycles, ESP32P4ClkRstState,
                             ESP32P4_CLKRST_HP_CORES),
        VMSTATE_UINT32_ARRAY(core_pulse_clk, ESP32P4ClkRstState,
                             ESP32P4_CLKRST_HP_CORES),
        VMSTATE_BOOL_ARRAY(core_pulse_waiting, ESP32P4ClkRstState,
                           ESP32P4_CLKRST_HP_CORES),
        VMSTATE_BOOL_ARRAY(core_reset_level, ESP32P4ClkRstState,
                           ESP32P4_CLKRST_HP_CORES),
        VMSTATE_BOOL_ARRAY(core_clock_level, ESP32P4ClkRstState,
                           ESP32P4_CLKRST_HP_CORES),
        VMSTATE_BOOL_ARRAY(core_vector_level, ESP32P4ClkRstState,
                           ESP32P4_CLKRST_HP_CORES),
        VMSTATE_BOOL_ARRAY(core_stall_level, ESP32P4ClkRstState,
                           ESP32P4_CLKRST_HP_CORES),
        VMSTATE_BOOL(hp_core_reset, ESP32P4ClkRstState),
        VMSTATE_BOOL(hp_core_reset_level, ESP32P4ClkRstState),
        VMSTATE_TIMER_PTR(hp_core_timer, ESP32P4ClkRstState),
        VMSTATE_BOOL(lp_core_reset_level, ESP32P4ClkRstState),
        VMSTATE_BOOL(lp_core_clock_level, ESP32P4ClkRstState),
        VMSTATE_UINT64_ARRAY(checked_soc, ESP32P4ClkRstState, 4),
        VMSTATE_UINT8(system_reset_cause, ESP32P4ClkRstState),
        VMSTATE_UINT8(chip_reset_cause, ESP32P4ClkRstState),
        VMSTATE_END_OF_LIST()
    },
};

static const Property esp32p4_clkrst_properties[] = {
    DEFINE_PROP_UINT32("xtal32k-hz", ESP32P4ClkRstState, xtal32k_hz, 32768),
};

/* [spec:nuos:req:emu.esp32p4.clocks] */
static bool esp32p4_clkrst_exact_period(Object *obj, Clock *out,
                                        ESP32P4ClkPeriod *p)
{
    ESP32P4ClkRstState *s = ESP32P4_CLKRST(obj);
    gpointer id = g_hash_table_lookup(s->clk_ids, out);

    if (!id) {
        return false;
    }
    *p = s->exact[GPOINTER_TO_INT(id) - 1];
    return true;
}

static void esp32p4_clkrst_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    ESP32P4ClkSourceClass *csc = ESP32P4_CLK_SOURCE_CLASS(klass);

    rc->phases.hold = esp32p4_clkrst_reset_hold;
    dc->realize = esp32p4_clkrst_realize;
    dc->vmsd = &vmstate_esp32p4_clkrst;
    device_class_set_props(dc, esp32p4_clkrst_properties);
    csc->exact_period = esp32p4_clkrst_exact_period;
}

/* [spec:nuos:req:emu.esp32p4.clocks] */
static const TypeInfo esp32p4_clkrst_info = {
    .name          = TYPE_ESP32P4_CLKRST,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32P4ClkRstState),
    .instance_init = esp32p4_clkrst_init,
    .instance_finalize = esp32p4_clkrst_finalize,
    .class_init    = esp32p4_clkrst_class_init,
    .interfaces    = (const InterfaceInfo[]) {
        { TYPE_ESP32P4_CLK_SOURCE },
        { }
    },
};

static void esp32p4_clkrst_register_types(void)
{
    type_register_static(&esp32p4_clkrst_info);
}
type_init(esp32p4_clkrst_register_types)
