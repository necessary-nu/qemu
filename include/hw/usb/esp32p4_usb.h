/*
 * ESP32-P4 USB 2.0 OTG controllers and their PHYs
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_USB_ESP32P4_USB_H
#define HW_USB_ESP32P4_USB_H

#include "hw/core/clock.h"
#include "hw/core/sysbus.h"
#include "hw/nvram/esp32p4_efuse.h"
#include "hw/misc/esp32p4_clk_period.h"
#include "hw/usb/dwc2-otg.h"
#include "hw/ssi/esp32p4_mspi.h"
#include "qom/object.h"

/*
 * The two DWC2 cores, configured as the chip has them: properties set,
 * so that -global esp32p4-usb-otg-hs.token-chardev=... picks one.
 */
#define TYPE_ESP32P4_USB_OTG_HS "esp32p4-usb-otg-hs"
#define TYPE_ESP32P4_USB_OTG_FS "esp32p4-usb-otg-fs"

#define ESP32P4_USB_OTG_HS_BASE     0x50000000
#define ESP32P4_USB_OTG_FS_BASE     0x50040000
#define ESP32P4_USB_WRAP_BASE       0x50080000
#define ESP32P4_USB_UTMI_BASE       0x5009c000

#define TYPE_ESP32P4_USB_WRAP "esp32p4-usb-wrap"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4UsbWrapState, ESP32P4_USB_WRAP)

/*
 * USB_WRAP, the full-speed OTG's PHY control (0x50080000, 4 KiB), and the
 * selection of which of the two full-speed PHYs, PHY 0 on GPIO24 (D-) and
 * GPIO25 (D+) and PHY 1 on GPIO26 and GPIO27, the OTG and USB
 * Serial/JTAG each have.
 *
 * Named GPIO inputs: "phy-sel" [2], LP_SYS_USB_CTRL's SW_HW_USB_PHY_SEL
 * and SW_USB_PHY_SEL; "sessend-in", the session-end signal from the GPIO
 * matrix; "core-dp-pullup", the core's D+ pull-up; "usj-pull-up" [2] and
 * "usj-pad-enable", USB Serial/JTAG's D- and D+ pull-ups and its
 * USB_PAD_ENABLE, for the PHY it has. Named GPIO outputs:
 * "phy-connected" and "pull-override" [2], to the full-speed core's
 * inputs of those names; "sessend", the session end the core sees
 * (SRP_SESSEND_OVERRIDE's, or the matrix's); "otg-phy", which of the two
 * PHYs the OTG has (USB Serial/JTAG has the other); "pad-pull-up" [4] and
 * "pad-phy" [4], for GPIO24 to GPIO27, the pull-up each PHY puts on its
 * pads and whether the PHY has them, for the GPIO block's inputs of
 * those names.
 */
#define ESP32P4_USB_WRAP_PHY_SEL    "phy-sel"
#define ESP32P4_USB_WRAP_SESSEND_IN "sessend-in"
#define ESP32P4_USB_WRAP_SESSEND    "sessend"
#define ESP32P4_USB_WRAP_OTG_PHY    "otg-phy"
#define ESP32P4_USB_WRAP_CORE_DP_PULLUP "core-dp-pullup"
#define ESP32P4_USB_WRAP_USJ_PULL_UP    "usj-pull-up"
#define ESP32P4_USB_WRAP_USJ_PAD_ENABLE "usj-pad-enable"
#define ESP32P4_USB_WRAP_PAD_PULL_UP    "pad-pull-up"
#define ESP32P4_USB_WRAP_PAD_PHY        "pad-phy"
/* The two PHYs' pads, D- then D+, from GPIO24. */
#define ESP32P4_USB_FS_PAD0         24
#define ESP32P4_USB_FS_PADS         4

struct ESP32P4UsbWrapState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq phy_connected;
    qemu_irq pull_override[2];
    qemu_irq otg_phy;
    qemu_irq sessend;
    qemu_irq pad_pull_up[ESP32P4_USB_FS_PADS];
    qemu_irq pad_phy[ESP32P4_USB_FS_PADS];
    ESP32P4EfuseState *efuse;

    uint32_t otg_conf;
    uint32_t test_conf;
    bool sw_phy_sel;
    bool sw_phy_val;
    bool sessend_in;
    bool core_dp_pullup;
    bool usj_pull_up[2];
    bool usj_pad_enable;
};

#define TYPE_ESP32P4_USB_UTMI "esp32p4-usb-utmi"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4UsbUtmiState, ESP32P4_USB_UTMI)

/*
 * The high-speed OTG's UTMI+ PHY and its registers (0x5009C000, 4 KiB).
 *
 * Clock input "refclk", the PHY's reference (LP_CLKRST's
 * USB_OTG20_PHYREF); clock output "utmi", the 60 MHz the PHY's PLL makes
 * from it, to the core.
 *
 * Named GPIO inputs: "phy-reset", the PHY's reset (LP_CLKRST's
 * RST_EN_USB_OTG20_PHY); "sys-ctrl" [ESP32P4_USB_UTMI_CTRL_NUM],
 * HP_SYSTEM_SYS_USBOTG20_CTRL's PHY controls; "drvvbus", the core's VBUS
 * drive; "host-attached", whether a host is at the other end of the
 * core's device port.
 *
 * Named GPIO outputs: "otg-sense" [DWC2_OTG_SENSE_NUM], to the core;
 * "phy-st" [4], HP_SYSTEM_USB_OTGHS_PHY_ST's SESSEND, VBUSVALID, BVALID
 * and SESSVALID.
 */
#define ESP32P4_USB_UTMI_PHY_RESET      "phy-reset"
#define ESP32P4_USB_UTMI_SYS_CTRL       "sys-ctrl"
#define ESP32P4_USB_UTMI_DRVVBUS        "drvvbus"
#define ESP32P4_USB_UTMI_HOST_ATTACHED  "host-attached"
#define ESP32P4_USB_UTMI_PHY_ST         "phy-st"

/* HP_SYSTEM_SYS_USBOTG20_CTRL's fields, bits 15 to 21, in order. */
enum {
    ESP32P4_USB_UTMI_CTRL_PHY_SUSPENDM,
    ESP32P4_USB_UTMI_CTRL_PHY_SUSPEND_FORCE_EN,
    ESP32P4_USB_UTMI_CTRL_PHY_RSTN,
    ESP32P4_USB_UTMI_CTRL_PHY_RESET_FORCE_EN,
    ESP32P4_USB_UTMI_CTRL_PHY_PLL_FORCE_EN,
    ESP32P4_USB_UTMI_CTRL_PHY_PLL_EN,
    ESP32P4_USB_UTMI_CTRL_OTG_SUSPENDM,
    ESP32P4_USB_UTMI_CTRL_NUM,
};

#define ESP32P4_USB_UTMI_REGS 8

struct ESP32P4UsbUtmiState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    Clock *refclk;
    Clock *utmi_clk;
    /* The edges of the PLL's clock, from its lock to the reference. */
    ESP32P4ClkEdges utmi_edges;
    qemu_irq sense[DWC2_OTG_SENSE_NUM];
    qemu_irq phy_st[4];

    uint8_t fc[ESP32P4_USB_UTMI_REGS];
    bool phy_reset;
    bool ctrl[ESP32P4_USB_UTMI_CTRL_NUM];
    bool drvvbus;
    bool host_attached;
};

/*
 * An OTG core's DMA waiting on a memory controller's held AXI port (the
 * flash's or the PSRAM's while it cannot take an access: its clocks
 * stopped, held in reset, AXI_REQ_EN clear; esp32p4_mspi_dma_wait()),
 * through dwc2_otg_set_dma_wait(): esp32p4_usb_dma_wait_init() sets it
 * up for @core, after dwc2_otg_set_dma() and before realize.
 */
typedef struct ESP32P4UsbDmaWait {
    ESP32P4MspiDmaWait wait;
    DWC2OTGState *core;
} ESP32P4UsbDmaWait;

void esp32p4_usb_dma_wait_init(ESP32P4UsbDmaWait *w, DWC2OTGState *core);

#endif
