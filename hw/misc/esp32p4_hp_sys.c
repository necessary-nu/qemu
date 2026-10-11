/*
 * ESP32-P4 HP system registers (HP_SYSTEM)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapters
 * 21 (System Registers), 2.13.6 (the cores' WFI wakeup), 9.3.2 (HP L2MEM
 * and HP SPM) and 34 (External Memory Encryption and Decryption), and
 * ESP-IDF's hp_system_reg.h for v3, the authority for the registers, their
 * reset values and access types (the TRM's register list differs from it:
 * HP SPM where the header has TCM, and no SYS_RDN_ECO, L2_MEM_SUBSIZE,
 * TCM_RAM_WRR_CONFIG, PROBE or DESIGN_FOR_VERIFICATION registers).
 *
 * Every register of the header is here, with its reset value; a write
 * changes only its read-write bits, and its read-only and reserved bits
 * read as the hardware drives them. What the registers do:
 *
 *   - CPU_INT_FROM_CPU_0 to _3 (0x10 to 0x1C): the software interrupts
 *     through which the HP cores signal each other, CPU_INTR_FROM_CPU_0 to
 *     _3, sources 79 to 82 of the interrupt matrix, each high while its bit
 *     is set; software clears it by writing 0, as ESP-IDF's crosscore_int_ll
 *     does.
 *   - CACHE_CLK_CONFIG (0x20) and CACHE_RESET_CONFIG (0x24): the L1 caches'
 *     resets, each emptying its cache, to the cache controller
 *     ("cache-reset" outputs); the caches' clock enables are held, a cache
 *     with its clock stopped working on (logged as unimplemented).
 *   - CRYPTO_CTRL (0x70): the four enables of the external memory
 *     encryption blocks, to the MSPI controllers ("crypto-ctrl" outputs).
 *   - L2_MEM_ERR_RESP_CTRL (0x198): whether an access to the part of L2MEM
 *     the L2 cache occupies is answered with an error, to the cache
 *     controller ("l2mem-err-resp"); and L2_MEM_INT_* with
 *     L2_MEM_INT_RECORD0, whose EXCEED_ADDR interrupt records such an
 *     access as the cache controller reports it
 *     (esp32p4_hp_sys_l2mem_exceed()): its offset into L2MEM and whether it
 *     wrote, the master being recorded as 0, as the encoding of
 *     EXCEED_ADDR_INT_MASTER is not published.
 *   - CORE_ERR_RESP_DIS (0x1A4): bit 0 for the cores' IBUS, bit 1 their
 *     DBUS and bit 2 their AHB bus, each to both HP cores
 *     ("core-err-resp-dis"): an access on a bus whose error response is
 *     disabled that the bus would answer with an error (one the access
 *     permission managers refuse, or to an address nothing decodes)
 *     completes without one, a read returning 0 and a write being dropped,
 *     and the core takes no access fault. ESP-IDF disables the error
 *     response for the dummy reads of its MSPI workarounds.
 *   - GPIO_O_HOLD_CTRL0 (0x74) and _CTRL1 (0x78): bit n of CTRL0 holds
 *     GPIO(16 + n), bit n of CTRL1 GPIO(48 + n), as ESP-IDF's
 *     GPIO_HOLD_MASK has them ("pad-hold" outputs, ORed with LP_SYSTEM's
 *     PAD_RTC_HOLD_CTRL for the GPIO block). GPIO_DED_HOLD_CTRL (0xF0)
 *     holds the flash and PSRAM pads, which have no pad model: it is
 *     kept as written.
 *   - HP2LP_INTR_GROUP0..3_EN (0x1F0 to 0x1FC) and _ST (0x200 to 0x20C):
 *     the LP core's HP_INTR, its interrupt 30, is high while any HP
 *     interrupt source an enable bit selects is; the status shows the
 *     enabled sources' levels. The 110 bits of the four groups are
 *     sources 21 to 130 of the interrupt matrix in order: the HP sources,
 *     after the 21 LP ones the LP core takes directly (the TRM does not
 *     list the groups' bits; their number, 32 + 32 + 32 + 14, is the
 *     number of HP sources of v3.x silicon, which adds three to v1's 107,
 *     and v1 has neither the sources nor the groups).
 *     HP2LP_WAKEUP_GROUP0..3_EN (0x210 to 0x21C) selects the same way the
 *     sources that wake the chip through the PMU ("hp-wakeup").
 *   - CPU_WAKEUP_EVENT (0x1EC): each core's dedicated wakeup input (TRM
 *     2.13.6.2), which wakes the core from WFI; while the bit is set a WFI
 *     does not wait ("cpu-wakeup" outputs).
 *   - CPU_WAITI_CONF (0x118): whether the cores' clocks run on in WFI
 *     (WAIT_MODE_FORCE_ON) and, if not, for how many cycles before they
 *     stop (WAITI_DELAY_NUM), to both cores ("cpu-waiti-conf" outputs),
 *     whose mcycle stops with the clock.
 *   - L2_MEM_REFRESH (0x164) and TCM_INIT (0x168): the initialisation of
 *     L2MEM's ECC bits, unit by unit, and of the SPM's parity bits, before
 *     their checks are enabled (TRM 9.3.2). A unit's REFRESH_DONE, or
 *     INIT_DONE, sets once its enable is set with the count reset clear,
 *     and clears when the count is reset. The TRM gives no duration, and
 *     software is told not to touch the memory meanwhile: the model
 *     completes at once, leaving the memories' contents as they were.
 *   - SYS_GMAC_CTRL0 (0x14C) to _CTRL2 (0x154): the EMAC's PHY interface
 *     (PHY_INTF_SEL) and sideband flow control request (SBD_FLOWCTRL), to
 *     the EMAC ("gmac-phy-intf-sel", "gmac-sbd-flowctrl" outputs); its PPS
 *     output and its resets in its transmit and receive clock domains as
 *     the EMAC drives them ("gmac-status" inputs); and its system time,
 *     read from the EMAC (esp32p4_hp_sys_set_gmac_ptp()).
 *     GMAC_MEM_CLK_FORCE_ON is held: the EMAC's FIFO memories always run.
 *   - USB20OTG_MEM_CTRL (0xF8), SYS_USBOTG20_CTRL (0x15C) and
 *     USB_OTGHS_PHY_ST (0x1E8): the high-speed OTG's UTMI+ PHY.
 *     SYS_USBOTG20_CTRL's suspend, reset and PLL forces go to the PHY
 *     ("usbotg20-ctrl" outputs), which stops its clock as they say; its
 *     other fields (the FIFO RAM's power controls, the transmit bit
 *     stuffing and reference clock mode) hold what software writes, and
 *     the PHY's BIST (OTG_PHY_BISTEN) is logged as unimplemented, its
 *     TEST_DONE reading 0. USB_OTGHS_PHY_ST reads the PHY's VBUS
 *     comparators ("usb-otghs-phy-st" inputs).
 *   - CPU_CORESTALLED_ST and CORE_DMACTIVE_LPCORE show the inputs from
 *     the blocks that drive them (see the header), which read low until
 *     connected.
 *   - SYS_DMA_ADDR_CTRL (0x2C) and PSRAM_FLASH_ADDR_INTERCHANGE (0x180)
 *     go out for the DMA engines and the external memory model
 *     ("dma-addr-sel", "psram-flash-interchange").
 *   - BITSCRAMBLER_PERI_SEL (0x140): the peripherals whose DMA paths the
 *     BitScrambler's RX and TX cores are attached to, to the BitScrambler
 *     ("bitscrambler-peri-sel" outputs).
 *   - RSA_PD_CTRL (0x1D0) and ECC_PD_CTRL (0x1D4): the power of the RSA
 *     and ECC accelerators' memories, down while MEM_FORCE_PD is set or
 *     MEM_PD is set without MEM_FORCE_PU, to the accelerators ("mem-pd"
 *     outputs).
 *
 * HP_SYSREG_INTR (source 110, sysbus IRQ 0) is the OR of the enabled
 * interrupts of L2_MEM_INT, TCM_INT, AHB2AXI_BRESP_ERR_INT and
 * CORE_TIMEOUT_INT. Their raw bits are write-1-to-clear as well as cleared
 * through their CLR registers. As the TRM and the header have them, the
 * AHB2AXI and TCM interrupts' raw bit is bit 0 and bit 31, and their ST,
 * ENA and CLR bit is bit 31. Of their sources only L2MEM's EXCEED_ADDR is
 * raised: L2MEM and the SPM have no bit flips to correct or detect, no
 * access waits on a slave, so none times out, and the AHB-to-AXI bridge's
 * posted writes complete without a late error response.
 *
 * Kept as written, with no effect the model can show: the memories' clock
 * force-ons, the other power controls (UART_PD_CTRL, SYS_VPU_CTRL, the
 * caches' power controls) and ECC and parity enables;
 * the bus timeouts' thresholds; the post-write enables; the arbitration
 * weights of TCM_RAM_WRR_CONFIG; the ECO and design-for-verification
 * registers; the debug run-stall between the HP and LP
 * cores, which have no debugger; SYS_GDMA_CTRL, for blocks not modelled
 * (esp32p4_hp_sys_reg() gives it to a block that needs it). The debug probe
 * (PROBEA_CTRL, PROBEB_CTRL) selects internal signals that are not
 * modelled: PROBE_OUT reads 0, logged as unimplemented once the probe is
 * enabled.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "hw/core/irq.h"
#include "hw/misc/esp32p4_hp_sys.h"
#include "migration/vmstate.h"

#define A_VER_DATE              0x000
#define A_CLK_EN                0x004
#define A_CPU_INT_FROM_CPU_0    0x010
#define A_CPU_INT_FROM_CPU_3    0x01c
#define A_CACHE_CLK_CONFIG      0x020
#define A_CACHE_RESET_CONFIG    0x024
#define A_SYS_DMA_ADDR_CTRL     0x02c
#define A_TCM_RAM_WRR_CONFIG    0x034
#define A_TCM_SW_PARITY_BWE_MASK 0x038
#define A_TCM_RAM_PWR_CTRL0     0x03c
#define A_L2_ROM_PWR_CTRL0      0x040
#define A_PROBEA_CTRL           0x050
#define A_PROBEB_CTRL           0x054
#define A_PROBE_OUT             0x05c
#define A_L2_MEM_RAM_PWR_CTRL0  0x060
#define A_CPU_CORESTALLED_ST    0x064
#define A_CRYPTO_CTRL           0x070
#define A_GPIO_O_HOLD_CTRL0     0x074
#define A_GPIO_O_HOLD_CTRL1     0x078
#define A_SYS_RDN_ECO_CS        0x07c
#define A_CACHE_APB_POSTW_EN    0x080
#define A_L2_MEM_SUBSIZE        0x084
#define A_L2_MEM_INT_RAW        0x09c
#define A_L2_MEM_INT_ST         0x0a0
#define A_L2_MEM_INT_ENA        0x0a4
#define A_L2_MEM_INT_CLR        0x0a8
#define A_L2_MEM_L2_RAM_ECC     0x0ac
#define A_L2_MEM_INT_RECORD0    0x0b0
#define A_L2_MEM_INT_RECORD1    0x0b4
#define A_L2_MEM_L2_CACHE_ECC   0x0c4
#define A_L1CACHE_BUS0_ID       0x0c8
#define A_L1CACHE_BUS1_ID       0x0cc
#define A_L2_MEM_RDN_ECO_CS     0x0d8
#define A_L2_MEM_RDN_ECO_LOW    0x0dc
#define A_L2_MEM_RDN_ECO_HIGH   0x0e0
#define A_TCM_RDN_ECO_CS        0x0e4
#define A_TCM_RDN_ECO_LOW       0x0e8
#define A_TCM_RDN_ECO_HIGH      0x0ec
#define A_GPIO_DED_HOLD_CTRL    0x0f0
#define A_L2_MEM_SW_ECC_BWE_MASK 0x0f4
#define A_USB20OTG_MEM_CTRL     0x0f8
#define A_TCM_INT_RAW           0x0fc
#define A_TCM_INT_ST            0x100
#define A_TCM_INT_ENA           0x104
#define A_TCM_INT_CLR           0x108
#define A_TCM_PARITY_INT_RECORD 0x10c
#define A_L1_CACHE_PWR_CTRL     0x110
#define A_L2_CACHE_PWR_CTRL     0x114
#define A_CPU_WAITI_CONF        0x118
#define A_SYS_CORE_DEBUG_RUNSTALL_CONF 0x11c
#define A_CORE_AHB_TIMEOUT      0x120
#define A_CORE_IBUS_TIMEOUT     0x124
#define A_CORE_DBUS_TIMEOUT     0x128
#define A_ICM_CPU_H2X_CFG       0x138
#define A_PERI1_APB_POSTW_EN    0x13c
#define A_BITSCRAMBLER_PERI_SEL 0x140
#define A_SYS_APB_SYNC_POSTW_EN 0x144
#define A_SYS_GDMA_CTRL         0x148
#define A_SYS_GMAC_CTRL0        0x14c
#define A_SYS_GMAC_CTRL1        0x150
#define A_SYS_GMAC_CTRL2        0x154
#define A_SYS_VPU_CTRL          0x158
#define A_SYS_USBOTG20_CTRL     0x15c
#define A_TCM_ERR_RESP_CTRL     0x160
#define A_L2_MEM_REFRESH        0x164
#define A_TCM_INIT              0x168
#define A_TCM_PARITY_CHECK_CTRL 0x16c
#define A_DESIGN_FOR_VERIFICATION0 0x170
#define A_DESIGN_FOR_VERIFICATION1 0x174
#define A_PSRAM_FLASH_ADDR_INTERCHANGE 0x180
#define A_AHB2AXI_BRESP_ERR_INT_RAW 0x188
#define A_AHB2AXI_BRESP_ERR_INT_ST  0x18c
#define A_AHB2AXI_BRESP_ERR_INT_ENA 0x190
#define A_AHB2AXI_BRESP_ERR_INT_CLR 0x194
#define A_L2_MEM_ERR_RESP_CTRL  0x198
#define A_L2_MEM_AHB_BUFFER_CTRL 0x19c
#define A_CORE_DMACTIVE_LPCORE  0x1a0
#define A_CORE_ERR_RESP_DIS     0x1a4
#define A_CORE_TIMEOUT_INT_RAW  0x1a8
#define A_CORE_TIMEOUT_INT_ST   0x1ac
#define A_CORE_TIMEOUT_INT_ENA  0x1b0
#define A_CORE_TIMEOUT_INT_CLR  0x1b4
#define A_GPIO_O_HYS_CTRL0      0x1c0
#define A_GPIO_O_HYS_CTRL1      0x1c4
#define A_RSA_PD_CTRL           0x1d0
#define A_ECC_PD_CTRL           0x1d4
#define A_RNG_CFG               0x1d8
#define A_UART_PD_CTRL          0x1dc
#define A_PERI_MEM_CLK_FORCE_ON 0x1e0
#define A_USB_OTGHS_PHY_ST      0x1e8
#define A_CPU_WAKEUP_EVENT      0x1ec
#define A_HP2LP_INTR_GROUP_EN(n) (0x1f0 + 4 * (n))
#define A_HP2LP_INTR_GROUP_ST(n) (0x200 + 4 * (n))
#define A_HP2LP_WAKEUP_GROUP_EN(n) (0x210 + 4 * (n))

#define R(a)                    ((a) / 4)

#define CPU_INT_FROM_CPU        BIT(0)
#define CACHE_CLK_CONFIG_MASK   0x33
#define L2_MEM_INT_MASK         0x7
#define L2_MEM_EXCEED_ADDR_INT  BIT(1)
#define RECORD0_ADDR_MASK       0x1fffff
#define RECORD0_WE              BIT(21)
#define TCM_INT_BIT             BIT(31)
#define AHB2AXI_RAW_BIT         BIT(0)
#define AHB2AXI_BIT             BIT(31)
#define CORE_TIMEOUT_INT_MASK   0x3f
#define PROBE_A_GLOBAL_EN       BIT(28)
#define PROBE_B_EN              BIT(24)
#define REFRESH_UNITS           6
#define REFRESH_EN_MASK         0x3f
#define REFRESH_CNT_RESET       BIT(6)
#define REFRESH_DONE_SHIFT      7
#define TCM_INIT_EN             BIT(0)
#define TCM_INIT_CNT_RESET      BIT(1)
#define MEM_FORCE_PD            BIT(0)
#define MEM_FORCE_PU            BIT(1)
#define MEM_PD                  BIT(2)
#define TCM_INIT_DONE           BIT(2)
#define GMAC_CTRL0_SBD_FLOWCTRL BIT(1)
#define GMAC_CTRL0_PHY_INTF_SHIFT 2
#define GMAC_CTRL0_PHY_INTF     (7u << GMAC_CTRL0_PHY_INTF_SHIFT)
#define GMAC_CTRL0_STATUS       (BIT(0) | BIT(6) | BIT(7))
#define USBOTG20_CTRL_PHY_SHIFT 15
#define USBOTG20_CTRL_BISTEN    BIT(24)
#define GROUP3_MASK             0x3fff

/* A register: its reset value and the bits software writes. */
typedef struct {
    uint32_t reset;
    uint32_t wmask;
    bool valid;
} ESP32P4HPSysReg;

#define REG(a, r, w)    [R(a)] = { .reset = (r), .wmask = (w), .valid = true }
/* A register only hardware sets, or one that only acts on a write. */
#define RO(a)           [R(a)] = { .valid = true }

static const ESP32P4HPSysReg esp32p4_hp_sys_regs[ESP32P4_HP_SYS_NREGS] = {
    REG(A_VER_DATE, 0x20250307, 0xffffffff),
    REG(A_CLK_EN, 0, 0x1),
    REG(A_CPU_INT_FROM_CPU_0, 0, CPU_INT_FROM_CPU),
    REG(A_CPU_INT_FROM_CPU_0 + 4, 0, CPU_INT_FROM_CPU),
    REG(A_CPU_INT_FROM_CPU_0 + 8, 0, CPU_INT_FROM_CPU),
    REG(A_CPU_INT_FROM_CPU_3, 0, CPU_INT_FROM_CPU),
    REG(A_CACHE_CLK_CONFIG, 0x33, CACHE_CLK_CONFIG_MASK),
    REG(A_CACHE_RESET_CONFIG, 0, 0x32),
    REG(A_SYS_DMA_ADDR_CTRL, 0, 0x1),
    REG(A_TCM_RAM_WRR_CONFIG, 0x826ed93f, 0x87ffffff),
    REG(A_TCM_SW_PARITY_BWE_MASK, 0, 0x1),
    REG(A_TCM_RAM_PWR_CTRL0, 0, 0x1),
    REG(A_L2_ROM_PWR_CTRL0, 0, 0x1),
    REG(A_PROBEA_CTRL, 0, 0x1fffffff),
    REG(A_PROBEB_CTRL, 0, 0x01ffffff),
    RO(A_PROBE_OUT),
    REG(A_L2_MEM_RAM_PWR_CTRL0, 0, 0x1),
    RO(A_CPU_CORESTALLED_ST),
    REG(A_CRYPTO_CTRL, 0, 0xf),
    REG(A_GPIO_O_HOLD_CTRL0, 0, 0xffffffff),
    REG(A_GPIO_O_HOLD_CTRL1, 0, 0x1ff),
    REG(A_SYS_RDN_ECO_CS, 0, 0x1),
    REG(A_CACHE_APB_POSTW_EN, 0, 0x1),
    REG(A_L2_MEM_SUBSIZE, 0, 0x3),
    RO(A_L2_MEM_INT_RAW),
    RO(A_L2_MEM_INT_ST),
    REG(A_L2_MEM_INT_ENA, 0, L2_MEM_INT_MASK),
    RO(A_L2_MEM_INT_CLR),
    REG(A_L2_MEM_L2_RAM_ECC, 0, 0x3f),
    RO(A_L2_MEM_INT_RECORD0),
    RO(A_L2_MEM_INT_RECORD1),
    REG(A_L2_MEM_L2_CACHE_ECC, 0, 0x1),
    REG(A_L1CACHE_BUS0_ID, 0, 0xf),
    REG(A_L1CACHE_BUS1_ID, 0, 0xf),
    REG(A_L2_MEM_RDN_ECO_CS, 0, 0x1),
    REG(A_L2_MEM_RDN_ECO_LOW, 0, 0xffffffff),
    REG(A_L2_MEM_RDN_ECO_HIGH, 0xffffffff, 0xffffffff),
    REG(A_TCM_RDN_ECO_CS, 0, 0x1),
    REG(A_TCM_RDN_ECO_LOW, 0, 0xffffffff),
    REG(A_TCM_RDN_ECO_HIGH, 0xffffffff, 0xffffffff),
    REG(A_GPIO_DED_HOLD_CTRL, 0, 0x03ffffff),
    REG(A_L2_MEM_SW_ECC_BWE_MASK, 0, 0x1),
    REG(A_USB20OTG_MEM_CTRL, 0, 0x1),
    RO(A_TCM_INT_RAW),
    RO(A_TCM_INT_ST),
    REG(A_TCM_INT_ENA, 0, TCM_INT_BIT),
    RO(A_TCM_INT_CLR),
    RO(A_TCM_PARITY_INT_RECORD),
    REG(A_L1_CACHE_PWR_CTRL, 0, 0x3f),
    REG(A_L2_CACHE_PWR_CTRL, 0, 0x3),
    REG(A_CPU_WAITI_CONF, 0x1, 0x1f),
    REG(A_SYS_CORE_DEBUG_RUNSTALL_CONF, 0, 0x1),
    REG(A_CORE_AHB_TIMEOUT, 0x1ffff, 0x1ffff),
    REG(A_CORE_IBUS_TIMEOUT, 0x1ffff, 0x1ffff),
    REG(A_CORE_DBUS_TIMEOUT, 0x1ffff, 0x1ffff),
    REG(A_ICM_CPU_H2X_CFG, 0x3, 0x3),
    REG(A_PERI1_APB_POSTW_EN, 0, 0x1),
    REG(A_BITSCRAMBLER_PERI_SEL, 0xff, 0xff),
    REG(A_SYS_APB_SYNC_POSTW_EN, 0, 0xf),
    REG(A_SYS_GDMA_CTRL, 0, 0x3),
    REG(A_SYS_GMAC_CTRL0, 0, 0x3e),
    RO(A_SYS_GMAC_CTRL1),
    RO(A_SYS_GMAC_CTRL2),
    REG(A_SYS_VPU_CTRL, 0, 0x1f),
    REG(A_SYS_USBOTG20_CTRL, 0x822640, 0x01fffffe),
    REG(A_TCM_ERR_RESP_CTRL, 0, 0x1),
    REG(A_L2_MEM_REFRESH, 0x40, REFRESH_EN_MASK | REFRESH_CNT_RESET),
    REG(A_TCM_INIT, 0x2, TCM_INIT_EN | TCM_INIT_CNT_RESET),
    REG(A_TCM_PARITY_CHECK_CTRL, 0, 0x1),
    REG(A_DESIGN_FOR_VERIFICATION0, 0, 0xffffffff),
    REG(A_DESIGN_FOR_VERIFICATION1, 0, 0xffffffff),
    REG(A_PSRAM_FLASH_ADDR_INTERCHANGE, 0, 0x3),
    RO(A_AHB2AXI_BRESP_ERR_INT_RAW),
    RO(A_AHB2AXI_BRESP_ERR_INT_ST),
    REG(A_AHB2AXI_BRESP_ERR_INT_ENA, 0, AHB2AXI_BIT),
    RO(A_AHB2AXI_BRESP_ERR_INT_CLR),
    REG(A_L2_MEM_ERR_RESP_CTRL, 0, 0x1),
    REG(A_L2_MEM_AHB_BUFFER_CTRL, 0, 0x3),
    RO(A_CORE_DMACTIVE_LPCORE),
    REG(A_CORE_ERR_RESP_DIS, 0, 0x7),
    RO(A_CORE_TIMEOUT_INT_RAW),
    RO(A_CORE_TIMEOUT_INT_ST),
    REG(A_CORE_TIMEOUT_INT_ENA, 0, CORE_TIMEOUT_INT_MASK),
    RO(A_CORE_TIMEOUT_INT_CLR),
    REG(A_GPIO_O_HYS_CTRL0, 0, 0xffffffff),
    REG(A_GPIO_O_HYS_CTRL1, 0, 0x1ff),
    REG(A_RSA_PD_CTRL, 0x2, 0x7),
    REG(A_ECC_PD_CTRL, 0x2, 0x7),
    REG(A_RNG_CFG, 0, 0x00ff0001),
    REG(A_UART_PD_CTRL, 0x2, 0x3),
    REG(A_PERI_MEM_CLK_FORCE_ON, 0, 0xf),
    RO(A_USB_OTGHS_PHY_ST),
    REG(A_CPU_WAKEUP_EVENT, 0, 0x3),
    REG(A_HP2LP_INTR_GROUP_EN(0), 0xffffffff, 0xffffffff),
    REG(A_HP2LP_INTR_GROUP_EN(1), 0xffffffff, 0xffffffff),
    REG(A_HP2LP_INTR_GROUP_EN(2), 0xffffffff, 0xffffffff),
    REG(A_HP2LP_INTR_GROUP_EN(3), GROUP3_MASK, GROUP3_MASK),
    RO(A_HP2LP_INTR_GROUP_ST(0)),
    RO(A_HP2LP_INTR_GROUP_ST(1)),
    RO(A_HP2LP_INTR_GROUP_ST(2)),
    RO(A_HP2LP_INTR_GROUP_ST(3)),
    REG(A_HP2LP_WAKEUP_GROUP_EN(0), 0, 0xffffffff),
    REG(A_HP2LP_WAKEUP_GROUP_EN(1), 0, 0xffffffff),
    REG(A_HP2LP_WAKEUP_GROUP_EN(2), 0, 0xffffffff),
    REG(A_HP2LP_WAKEUP_GROUP_EN(3), 0, GROUP3_MASK),
};

static inline uint32_t *esp32p4_hp_sys_r(ESP32P4HPSysState *s, hwaddr a)
{
    return &s->regs[R(a)];
}

/* HP_SYSREG_INTR: the OR of the four groups' enabled raw bits. */
/* [spec:nuos:req:emu.esp32p4.debug] */
static void esp32p4_hp_sys_update_irq(ESP32P4HPSysState *s)
{
    uint32_t *r = s->regs;
    bool level = (r[R(A_L2_MEM_INT_RAW)] & r[R(A_L2_MEM_INT_ENA)]) ||
                 ((r[R(A_TCM_INT_RAW)] & r[R(A_TCM_INT_ENA)]) &
                  TCM_INT_BIT) ||
                 ((r[R(A_AHB2AXI_BRESP_ERR_INT_RAW)] & AHB2AXI_RAW_BIT) &&
                  (r[R(A_AHB2AXI_BRESP_ERR_INT_ENA)] & AHB2AXI_BIT)) ||
                 (r[R(A_CORE_TIMEOUT_INT_RAW)] &
                  r[R(A_CORE_TIMEOUT_INT_ENA)]);

    qemu_set_irq(s->sysreg_irq, level);
}

/* [spec:nuos:req:emu.esp32p4.memory] */
void esp32p4_hp_sys_l2mem_exceed(ESP32P4HPSysState *s, uint32_t offset,
                                 bool write)
{
    uint32_t *raw = esp32p4_hp_sys_r(s, A_L2_MEM_INT_RAW);

    if (!(*raw & L2_MEM_EXCEED_ADDR_INT)) {
        *esp32p4_hp_sys_r(s, A_L2_MEM_INT_RECORD0) =
            (offset & RECORD0_ADDR_MASK) | (write ? RECORD0_WE : 0);
    }
    *raw |= L2_MEM_EXCEED_ADDR_INT;
    esp32p4_hp_sys_update_irq(s);
}

void esp32p4_hp_sys_set_gmac_ptp(ESP32P4HPSysState *s,
                                 uint64_t (*time)(void *opaque),
                                 void *opaque)
{
    s->gmac_ptp_time = time;
    s->gmac_ptp_opaque = opaque;
}

/* [spec:nuos:req:emu.esp32p4.emac] */
static uint64_t esp32p4_hp_sys_gmac_time(ESP32P4HPSysState *s)
{
    return s->gmac_ptp_time ? s->gmac_ptp_time(s->gmac_ptp_opaque) : 0;
}

/*
 * The enabled HP sources' OR, for HP_INTR (the group enables) or the HP
 * wakeup (the wakeup enables).
 */
static bool esp32p4_hp_sys_group_any(ESP32P4HPSysState *s, hwaddr en)
{
    for (int n = 0; n < 4; n++) {
        if (s->hp_source[n] & s->regs[R(en) + n]) {
            return true;
        }
    }
    return false;
}

/* [spec:nuos:req:emu.esp32p4.debug] */
static void esp32p4_hp_sys_update_hp2lp(ESP32P4HPSysState *s)
{
    qemu_set_irq(s->hp_intr,
                 esp32p4_hp_sys_group_any(s, A_HP2LP_INTR_GROUP_EN(0)));
    qemu_set_irq(s->hp_wakeup,
                 esp32p4_hp_sys_group_any(s, A_HP2LP_WAKEUP_GROUP_EN(0)));
}

/* [spec:nuos:req:emu.esp32p4.debug] */
static void esp32p4_hp_sys_update_hold(ESP32P4HPSysState *s)
{
    uint64_t held = s->regs[R(A_GPIO_O_HOLD_CTRL0)] |
                    ((uint64_t)s->regs[R(A_GPIO_O_HOLD_CTRL1)] << 32);

    for (int n = 0; n < ESP32P4_HP_SYS_HOLDS; n++) {
        qemu_set_irq(s->pad_hold[n], extract64(held, n, 1));
    }
}

/* [spec:nuos:req:emu.esp32p4.debug] */
static void esp32p4_hp_sys_update_err_resp(ESP32P4HPSysState *s)
{
    uint32_t dis = s->regs[R(A_CORE_ERR_RESP_DIS)];

    for (int core = 0; core < ESP32P4_HP_SYS_CORES; core++) {
        for (int bus = 0; bus < ESP32P4_BUSES; bus++) {
            qemu_set_irq(s->err_resp_dis[core * ESP32P4_BUSES + bus],
                         extract32(dis, bus, 1));
        }
    }
}

/* L1_I0_CACHE_RESET (bit 5), L1_I1_CACHE_RESET (4), L1_D_CACHE_RESET (1). */
/* [spec:nuos:req:emu.esp32p4.memory] */
static void esp32p4_hp_sys_update_cache_reset(ESP32P4HPSysState *s)
{
    static const int bit[ESP32P4_HP_SYS_CACHE_RESET_LINES] = { 5, 4, 1 };

    for (int i = 0; i < ESP32P4_HP_SYS_CACHE_RESET_LINES; i++) {
        qemu_set_irq(s->cache_reset[i],
                     extract32(s->regs[R(A_CACHE_RESET_CONFIG)], bit[i], 1));
    }
}

/* [spec:nuos:req:emu.esp32p4.memory] */
static void esp32p4_hp_sys_update_crypto(ESP32P4HPSysState *s)
{
    for (int i = 0; i < ESP32P4_HP_SYS_CRYPTO_CTRL_LINES; i++) {
        qemu_set_irq(s->crypto_ctrl_out[i],
                     extract32(s->regs[R(A_CRYPTO_CTRL)], i, 1));
    }
}

/* [spec:nuos:req:emu.esp32p4.crypto] */
static void esp32p4_hp_sys_update_mem_pd(ESP32P4HPSysState *s)
{
    for (int i = 0; i < ESP32P4_HP_SYS_MEM_PD_LINES; i++) {
        uint32_t c = s->regs[R(A_RSA_PD_CTRL) + i];

        qemu_set_irq(s->mem_pd[i], (c & MEM_FORCE_PD) ||
                                   ((c & MEM_PD) && !(c & MEM_FORCE_PU)));
    }
}

static void esp32p4_hp_sys_update_wakeup(ESP32P4HPSysState *s)
{
    for (int i = 0; i < ESP32P4_HP_SYS_CORES; i++) {
        qemu_set_irq(s->cpu_wakeup[i],
                     extract32(s->regs[R(A_CPU_WAKEUP_EVENT)], i, 1));
    }
}

/* CPU_WAITI_CONF, which both HP cores' clocks in WFI follow. */
/* [spec:nuos:req:emu.esp32p4.cpu] */
static void esp32p4_hp_sys_update_waiti(ESP32P4HPSysState *s)
{
    for (int core = 0; core < ESP32P4_HP_SYS_CORES; core++) {
        for (int n = 0; n < ESP32P4_HP_SYS_WAITI_LINES; n++) {
            qemu_set_irq(s->cpu_waiti_conf[core * ESP32P4_HP_SYS_WAITI_LINES +
                                           n],
                         extract32(s->regs[R(A_CPU_WAITI_CONF)], n, 1));
        }
    }
}

/* [spec:nuos:req:emu.esp32p4.emac] */
static void esp32p4_hp_sys_update_gmac(ESP32P4HPSysState *s)
{
    uint32_t ctrl0 = s->regs[R(A_SYS_GMAC_CTRL0)];

    for (int i = 0; i < ESP32P4_HP_SYS_GMAC_PHY_INTF_LINES; i++) {
        qemu_set_irq(s->gmac_phy_intf[i],
                     extract32(ctrl0, GMAC_CTRL0_PHY_INTF_SHIFT + i, 1));
    }
    qemu_set_irq(s->gmac_sbd_flowctrl, !!(ctrl0 & GMAC_CTRL0_SBD_FLOWCTRL));
}

/* [spec:nuos:req:emu.esp32p4.usb] */
static void esp32p4_hp_sys_update_usb(ESP32P4HPSysState *s)
{
    for (int i = 0; i < ESP32P4_HP_SYS_USBOTG20_CTRL_LINES; i++) {
        qemu_set_irq(s->usbotg20_ctrl_out[i],
                     extract32(s->regs[R(A_SYS_USBOTG20_CTRL)],
                               USBOTG20_CTRL_PHY_SHIFT + i, 1));
    }
}

/* [spec:nuos:req:emu.esp32p4.parlio] */
static void esp32p4_hp_sys_update_bs(ESP32P4HPSysState *s)
{
    for (int i = 0; i < ESP32P4_HP_SYS_BS_PERI_SEL_LINES; i++) {
        qemu_set_irq(s->bs_peri_sel[i],
                     extract32(s->regs[R(A_BITSCRAMBLER_PERI_SEL)], i, 1));
    }
}

static void esp32p4_hp_sys_update_misc(ESP32P4HPSysState *s)
{
    qemu_set_irq(s->l2mem_err_resp, s->regs[R(A_L2_MEM_ERR_RESP_CTRL)]);
    qemu_set_irq(s->dma_addr_sel, s->regs[R(A_SYS_DMA_ADDR_CTRL)]);
    for (int i = 0; i < 2; i++) {
        qemu_set_irq(s->addr_interchange[i],
                     extract32(s->regs[R(A_PSRAM_FLASH_ADDR_INTERCHANGE)],
                               i, 1));
    }
}

/*
 * The memories' initialisation, which completes at once: a unit is done
 * once enabled with the count reset clear, until the count is reset.
 */
/* [spec:nuos:req:emu.esp32p4.debug] */
static void esp32p4_hp_sys_update_init(ESP32P4HPSysState *s)
{
    uint32_t *refresh = esp32p4_hp_sys_r(s, A_L2_MEM_REFRESH);
    uint32_t *init = esp32p4_hp_sys_r(s, A_TCM_INIT);

    if (*refresh & REFRESH_CNT_RESET) {
        *refresh &= REFRESH_EN_MASK | REFRESH_CNT_RESET;
    } else {
        *refresh |= (*refresh & REFRESH_EN_MASK) << REFRESH_DONE_SHIFT;
    }
    if (*init & TCM_INIT_CNT_RESET) {
        *init &= ~TCM_INIT_DONE;
    } else if (*init & TCM_INIT_EN) {
        *init |= TCM_INIT_DONE;
    }
}

uint32_t esp32p4_hp_sys_reg(ESP32P4HPSysState *s, hwaddr offset)
{
    uint32_t *r = s->regs;
    int n;

    switch (offset) {
    case A_L2_MEM_INT_ST:
        return r[R(A_L2_MEM_INT_RAW)] & r[R(A_L2_MEM_INT_ENA)];
    case A_TCM_INT_ST:
        return r[R(A_TCM_INT_RAW)] & r[R(A_TCM_INT_ENA)];
    case A_AHB2AXI_BRESP_ERR_INT_ST:
        return (r[R(A_AHB2AXI_BRESP_ERR_INT_RAW)] & AHB2AXI_RAW_BIT) ?
               r[R(A_AHB2AXI_BRESP_ERR_INT_ENA)] & AHB2AXI_BIT : 0;
    case A_CORE_TIMEOUT_INT_ST:
        return r[R(A_CORE_TIMEOUT_INT_RAW)] & r[R(A_CORE_TIMEOUT_INT_ENA)];
    case A_CPU_CORESTALLED_ST:
        return s->core_stalled;
    case A_CORE_DMACTIVE_LPCORE:
        return s->dmactive_lpcore;
    case A_SYS_GMAC_CTRL0:
        return r[R(offset)] | (s->gmac_status & GMAC_CTRL0_STATUS);
    case A_SYS_GMAC_CTRL1:
        return esp32p4_hp_sys_gmac_time(s);
    case A_SYS_GMAC_CTRL2:
        return esp32p4_hp_sys_gmac_time(s) >> 32;
    case A_USB_OTGHS_PHY_ST:
        return s->usb_otghs_phy_st << 1;
    case A_HP2LP_INTR_GROUP_ST(0) ... A_HP2LP_INTR_GROUP_ST(3):
        n = (offset - A_HP2LP_INTR_GROUP_ST(0)) / 4;
        return s->hp_source[n] & r[R(A_HP2LP_INTR_GROUP_EN(n))];
    default:
        if (offset < ESP32P4_HP_SYS_NREGS * 4) {
            return r[R(offset)];
        }
        return 0;
    }
}

static uint64_t esp32p4_hp_sys_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4HPSysState *s = opaque;
    hwaddr reg = addr & ~3;

    if (reg >= ESP32P4_HP_SYS_NREGS * 4 ||
        !esp32p4_hp_sys_regs[R(reg)].valid) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of %u bytes at reserved "
                      "offset 0x%" HWADDR_PRIx "\n", TYPE_ESP32P4_HP_SYS,
                      size, addr);
        return 0;
    }
    if (reg == A_PROBE_OUT &&
        (s->regs[R(A_PROBEA_CTRL)] & PROBE_A_GLOBAL_EN)) {
        qemu_log_mask(LOG_UNIMP, "%s: the debug probe's signals are not "
                      "modelled; PROBE_OUT reads 0\n", TYPE_ESP32P4_HP_SYS);
    }
    return extract32(esp32p4_hp_sys_reg(s, reg), (addr & 3) * 8, size * 8);
}

/* [spec:nuos:req:emu.esp32p4.interrupt-matrix] */
/* [spec:nuos:req:emu.esp32p4.memory] */
/* [spec:nuos:req:emu.esp32p4.debug] */
static void esp32p4_hp_sys_write(void *opaque, hwaddr addr, uint64_t value,
                                 unsigned size)
{
    ESP32P4HPSysState *s = opaque;
    hwaddr reg = addr & ~3;
    unsigned shift = (addr & 3) * 8;
    uint32_t bytes = MAKE_64BIT_MASK(shift, size * 8);
    uint32_t v = (uint32_t)value << shift;
    const ESP32P4HPSysReg *info;
    uint32_t *r;

    if (reg >= ESP32P4_HP_SYS_NREGS * 4 ||
        !esp32p4_hp_sys_regs[R(reg)].valid) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write of %u bytes at reserved "
                      "offset 0x%" HWADDR_PRIx " (value 0x%" PRIx64 ")\n",
                      TYPE_ESP32P4_HP_SYS, size, addr, value);
        return;
    }
    info = &esp32p4_hp_sys_regs[R(reg)];
    r = &s->regs[R(reg)];

    switch (reg) {
    /* Raw interrupt bits are write-1-to-clear, as their CLR bits are. */
    case A_L2_MEM_INT_RAW:
    case A_L2_MEM_INT_CLR:
        s->regs[R(A_L2_MEM_INT_RAW)] &= ~(v & bytes & L2_MEM_INT_MASK);
        break;
    case A_TCM_INT_RAW:
    case A_TCM_INT_CLR:
        s->regs[R(A_TCM_INT_RAW)] &= ~(v & bytes & TCM_INT_BIT);
        break;
    case A_AHB2AXI_BRESP_ERR_INT_RAW:
        *r &= ~(v & bytes & AHB2AXI_RAW_BIT);
        break;
    case A_AHB2AXI_BRESP_ERR_INT_CLR:
        if (v & bytes & AHB2AXI_BIT) {
            s->regs[R(A_AHB2AXI_BRESP_ERR_INT_RAW)] &= ~AHB2AXI_RAW_BIT;
        }
        break;
    case A_CORE_TIMEOUT_INT_RAW:
    case A_CORE_TIMEOUT_INT_CLR:
        s->regs[R(A_CORE_TIMEOUT_INT_RAW)] &=
            ~(v & bytes & CORE_TIMEOUT_INT_MASK);
        break;
    default:
        if (!info->wmask) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only offset "
                          "0x%" HWADDR_PRIx "\n", TYPE_ESP32P4_HP_SYS, addr);
            return;
        }
        *r = (*r & ~(info->wmask & bytes)) | (v & info->wmask & bytes);
        break;
    }

    switch (reg) {
    case A_CPU_INT_FROM_CPU_0 ... A_CPU_INT_FROM_CPU_3:
        qemu_set_irq(s->cpu_intr[R(reg - A_CPU_INT_FROM_CPU_0)], *r);
        break;
    case A_CACHE_CLK_CONFIG:
        if (*r != CACHE_CLK_CONFIG_MASK) {
            qemu_log_mask(LOG_UNIMP, "%s: stopping a cache's clock "
                          "(CACHE_CLK_CONFIG 0x%02" PRIx32 ") is not "
                          "modelled; the caches keep working\n",
                          TYPE_ESP32P4_HP_SYS, *r);
        }
        break;
    case A_CACHE_RESET_CONFIG:
        esp32p4_hp_sys_update_cache_reset(s);
        break;
    case A_CRYPTO_CTRL:
        esp32p4_hp_sys_update_crypto(s);
        break;
    case A_GPIO_O_HOLD_CTRL0:
    case A_GPIO_O_HOLD_CTRL1:
        esp32p4_hp_sys_update_hold(s);
        break;
    case A_CORE_ERR_RESP_DIS:
        esp32p4_hp_sys_update_err_resp(s);
        break;
    case A_L2_MEM_REFRESH:
    case A_TCM_INIT:
        esp32p4_hp_sys_update_init(s);
        break;
    case A_CPU_WAKEUP_EVENT:
        esp32p4_hp_sys_update_wakeup(s);
        break;
    case A_CPU_WAITI_CONF:
        esp32p4_hp_sys_update_waiti(s);
        break;
    case A_RSA_PD_CTRL:
    case A_ECC_PD_CTRL:
        esp32p4_hp_sys_update_mem_pd(s);
        break;
    case A_HP2LP_INTR_GROUP_EN(0) ... A_HP2LP_INTR_GROUP_EN(3):
    case A_HP2LP_WAKEUP_GROUP_EN(0) ... A_HP2LP_WAKEUP_GROUP_EN(3):
        esp32p4_hp_sys_update_hp2lp(s);
        break;
    case A_L2_MEM_ERR_RESP_CTRL:
    case A_SYS_DMA_ADDR_CTRL:
    case A_PSRAM_FLASH_ADDR_INTERCHANGE:
        esp32p4_hp_sys_update_misc(s);
        break;
    case A_BITSCRAMBLER_PERI_SEL:
        esp32p4_hp_sys_update_bs(s);
        break;
    case A_SYS_USBOTG20_CTRL:
        if (*r & USBOTG20_CTRL_BISTEN) {
            qemu_log_mask(LOG_UNIMP, "%s: the USB 2.0 OTG PHY's BIST "
                          "(SYS_OTG_PHY_BISTEN) is not modelled\n",
                          TYPE_ESP32P4_HP_SYS);
        }
        esp32p4_hp_sys_update_usb(s);
        break;
    case A_SYS_GMAC_CTRL0: {
        unsigned intf = (*r & GMAC_CTRL0_PHY_INTF) >>
                        GMAC_CTRL0_PHY_INTF_SHIFT;

        if (intf != 0 && intf != 4) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: PHY_INTF_SEL %u is neither "
                          "MII (0) nor RMII (4)\n", TYPE_ESP32P4_HP_SYS,
                          intf);
        }
        esp32p4_hp_sys_update_gmac(s);
        break;
    }
    default:
        break;
    }
    esp32p4_hp_sys_update_irq(s);
}

static const MemoryRegionOps esp32p4_hp_sys_ops = {
    .read = esp32p4_hp_sys_read,
    .write = esp32p4_hp_sys_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

/* The level of HP interrupt source 21 + n. */
/* [spec:nuos:req:emu.esp32p4.debug] */
static void esp32p4_hp_sys_set_source(void *opaque, int n, int level)
{
    ESP32P4HPSysState *s = opaque;

    s->hp_source[n / 32] = deposit32(s->hp_source[n / 32], n % 32, 1,
                                     level != 0);
    esp32p4_hp_sys_update_hp2lp(s);
}

static void esp32p4_hp_sys_set_core_stalled(void *opaque, int n, int level)
{
    ESP32P4HPSysState *s = opaque;

    s->core_stalled = deposit32(s->core_stalled, n, 1, level != 0);
}

static void esp32p4_hp_sys_set_dmactive(void *opaque, int n, int level)
{
    ESP32P4_HP_SYS(opaque)->dmactive_lpcore = level != 0;
}

/* PTP_PPS (bit 0), RST_CLK_TX_N (bit 6), RST_CLK_RX_N (bit 7). */
static void esp32p4_hp_sys_set_gmac_status(void *opaque, int n, int level)
{
    static const int bit[ESP32P4_HP_SYS_GMAC_STATUS_LINES] = {
        [ESP32P4_HP_SYS_GMAC_PTP_PPS] = 0,
        [ESP32P4_HP_SYS_GMAC_RST_CLK_TX_N] = 6,
        [ESP32P4_HP_SYS_GMAC_RST_CLK_RX_N] = 7,
    };
    ESP32P4HPSysState *s = opaque;

    s->gmac_status = deposit32(s->gmac_status, bit[n], 1, level != 0);
}

static void esp32p4_hp_sys_set_otg_phy(void *opaque, int n, int level)
{
    ESP32P4HPSysState *s = opaque;

    s->usb_otghs_phy_st = deposit32(s->usb_otghs_phy_st, n, 1, level != 0);
}

static void esp32p4_hp_sys_reset_hold(Object *obj, ResetType type)
{
    ESP32P4HPSysState *s = ESP32P4_HP_SYS(obj);

    for (int i = 0; i < ESP32P4_HP_SYS_NREGS; i++) {
        s->regs[i] = esp32p4_hp_sys_regs[i].reset;
    }
}

static void esp32p4_hp_sys_reset_exit(Object *obj, ResetType type)
{
    ESP32P4HPSysState *s = ESP32P4_HP_SYS(obj);

    for (int i = 0; i < ESP32P4_HP_SYS_CPU_INTRS; i++) {
        qemu_set_irq(s->cpu_intr[i], s->regs[R(A_CPU_INT_FROM_CPU_0) + i]);
    }
    esp32p4_hp_sys_update_cache_reset(s);
    esp32p4_hp_sys_update_crypto(s);
    esp32p4_hp_sys_update_hold(s);
    esp32p4_hp_sys_update_err_resp(s);
    esp32p4_hp_sys_update_wakeup(s);
    esp32p4_hp_sys_update_waiti(s);
    esp32p4_hp_sys_update_hp2lp(s);
    esp32p4_hp_sys_update_misc(s);
    esp32p4_hp_sys_update_gmac(s);
    esp32p4_hp_sys_update_usb(s);
    esp32p4_hp_sys_update_bs(s);
    esp32p4_hp_sys_update_mem_pd(s);
    esp32p4_hp_sys_update_irq(s);
}

static void esp32p4_hp_sys_init(Object *obj)
{
    ESP32P4HPSysState *s = ESP32P4_HP_SYS(obj);
    DeviceState *dev = DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32p4_hp_sys_ops, s,
                          "esp32p4.hp-sys", ESP32P4_HP_SYS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->sysreg_irq);
    qdev_init_gpio_out_named(dev, s->cpu_intr, ESP32P4_HP_SYS_CPU_INTR,
                             ESP32P4_HP_SYS_CPU_INTRS);
    qdev_init_gpio_out_named(dev, s->cache_reset, ESP32P4_HP_SYS_CACHE_RESET,
                             ESP32P4_HP_SYS_CACHE_RESET_LINES);
    qdev_init_gpio_out_named(dev, s->crypto_ctrl_out,
                             ESP32P4_HP_SYS_CRYPTO_CTRL,
                             ESP32P4_HP_SYS_CRYPTO_CTRL_LINES);
    qdev_init_gpio_out_named(dev, &s->l2mem_err_resp,
                             ESP32P4_HP_SYS_L2MEM_ERR_RESP, 1);
    qdev_init_gpio_out_named(dev, s->err_resp_dis,
                             ESP32P4_HP_SYS_ERR_RESP_DIS,
                             ESP32P4_HP_SYS_CORES * ESP32P4_BUSES);
    qdev_init_gpio_out_named(dev, s->pad_hold, ESP32P4_HP_SYS_PAD_HOLD,
                             ESP32P4_HP_SYS_HOLDS);
    qdev_init_gpio_out_named(dev, &s->hp_intr, ESP32P4_HP_SYS_HP_INTR, 1);
    qdev_init_gpio_out_named(dev, &s->hp_wakeup, ESP32P4_HP_SYS_HP_WAKEUP, 1);
    qdev_init_gpio_out_named(dev, s->cpu_wakeup, ESP32P4_HP_SYS_CPU_WAKEUP,
                             ESP32P4_HP_SYS_CORES);
    qdev_init_gpio_out_named(dev, s->cpu_waiti_conf,
                             ESP32P4_HP_SYS_CPU_WAITI_CONF,
                             ESP32P4_HP_SYS_CORES *
                             ESP32P4_HP_SYS_WAITI_LINES);
    qdev_init_gpio_out_named(dev, &s->dma_addr_sel,
                             ESP32P4_HP_SYS_DMA_ADDR_SEL, 1);
    qdev_init_gpio_out_named(dev, s->addr_interchange,
                             ESP32P4_HP_SYS_ADDR_INTERCHANGE, 2);
    qdev_init_gpio_out_named(dev, s->gmac_phy_intf,
                             ESP32P4_HP_SYS_GMAC_PHY_INTF,
                             ESP32P4_HP_SYS_GMAC_PHY_INTF_LINES);
    qdev_init_gpio_out_named(dev, &s->gmac_sbd_flowctrl,
                             ESP32P4_HP_SYS_GMAC_SBD_FLOWCTRL, 1);
    qdev_init_gpio_out_named(dev, s->usbotg20_ctrl_out,
                             ESP32P4_HP_SYS_USBOTG20_CTRL,
                             ESP32P4_HP_SYS_USBOTG20_CTRL_LINES);
    qdev_init_gpio_out_named(dev, s->bs_peri_sel, ESP32P4_HP_SYS_BS_PERI_SEL,
                             ESP32P4_HP_SYS_BS_PERI_SEL_LINES);
    qdev_init_gpio_out_named(dev, s->mem_pd, ESP32P4_HP_SYS_MEM_PD,
                             ESP32P4_HP_SYS_MEM_PD_LINES);
    qdev_init_gpio_in_named(dev, esp32p4_hp_sys_set_source,
                            ESP32P4_HP_SYS_HP_SOURCE,
                            ESP32P4_HP_SYS_HP2LP_SOURCES);
    qdev_init_gpio_in_named(dev, esp32p4_hp_sys_set_core_stalled,
                            ESP32P4_HP_SYS_CORE_STALLED, ESP32P4_HP_SYS_CORES);
    qdev_init_gpio_in_named(dev, esp32p4_hp_sys_set_dmactive,
                            ESP32P4_HP_SYS_DMACTIVE_LPCORE, 1);
    qdev_init_gpio_in_named(dev, esp32p4_hp_sys_set_gmac_status,
                            ESP32P4_HP_SYS_GMAC_STATUS,
                            ESP32P4_HP_SYS_GMAC_STATUS_LINES);
    qdev_init_gpio_in_named(dev, esp32p4_hp_sys_set_otg_phy,
                            ESP32P4_HP_SYS_USB_OTGHS_PHY_ST,
                            ESP32P4_HP_SYS_USB_OTGHS_PHY_ST_LINES);
}

static const VMStateDescription vmstate_esp32p4_hp_sys = {
    .name = TYPE_ESP32P4_HP_SYS,
    .version_id = 4,
    .minimum_version_id = 4,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, ESP32P4HPSysState, ESP32P4_HP_SYS_NREGS),
        VMSTATE_UINT32_ARRAY(hp_source, ESP32P4HPSysState, 4),
        VMSTATE_UINT32(core_stalled, ESP32P4HPSysState),
        VMSTATE_BOOL(dmactive_lpcore, ESP32P4HPSysState),
        VMSTATE_UINT32(gmac_status, ESP32P4HPSysState),
        VMSTATE_UINT32(usb_otghs_phy_st, ESP32P4HPSysState),
        VMSTATE_END_OF_LIST()
    },
};

static void esp32p4_hp_sys_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32p4_hp_sys_reset_hold;
    rc->phases.exit = esp32p4_hp_sys_reset_exit;
    dc->vmsd = &vmstate_esp32p4_hp_sys;
}

/* [spec:nuos:req:emu.esp32p4.interrupt-matrix] */
/* [spec:nuos:req:emu.esp32p4.debug] */
static const TypeInfo esp32p4_hp_sys_info = {
    .name          = TYPE_ESP32P4_HP_SYS,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32P4HPSysState),
    .instance_init = esp32p4_hp_sys_init,
    .class_init    = esp32p4_hp_sys_class_init,
};

static void esp32p4_hp_sys_register_types(void)
{
    type_register_static(&esp32p4_hp_sys_info);
}
type_init(esp32p4_hp_sys_register_types)
