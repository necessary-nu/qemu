/*
 * ESP32 Ethernet MAC (EMAC), a Synopsys DesignWare GMAC
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32 TRM, "Ethernet Media Access Controller (EMAC)", with the
 * field layout of ESP-IDF's soc/esp32 emac_dma_struct.h, emac_mac_struct.h
 * and emac_ext_struct.h.
 *
 * One 8 KiB block: the GMAC's DMA registers at 0x0000, the ESP32's clock
 * and PHY interface wrapper at 0x0800 and the GMAC's MAC registers at
 * 0x1000. The GMAC itself, with its 2 KiB FIFOs, eight MAC addresses and
 * no timestamp engine, is hw/net/esp32_dwmac.c; this is the ESP32's
 * arrangement of it.
 *
 * The MAC's transmit and receive paths run on the MII clocks the wrapper
 * enables: TX_CLK and RX_CLK in MII mode, the 50 MHz reference in RMII
 * mode. Its AHB master reaches internal SRAM 1 and 2 only. The PHY is
 * wired straight to the MAC.
 *
 * Not modelled: the IEEE 1588 timestamp engine (the TRM documents none of
 * its registers, so TDES0.TTSE has no effect).
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/qdev-properties.h"
#include "hw/net/esp32_emac.h"
#include "migration/vmstate.h"

/* Clock and PHY interface wrapper */
#define A_EX_BASE           0x0800
#define A_EX_CLKOUT_CONF    0x0800
#define A_EX_OSCCLK_CONF    0x0804
#define A_EX_CLK_CTRL       0x0808
#define A_EX_PHYINF_CONF    0x080c
#define A_EX_PD_SEL         0x0810
#define A_EX_DATE           0x08fc
/* The GMAC's MAC registers */
#define A_MAC_BASE          0x1000

/* EMAC_EX_CLK_CTRL */
#define EX_CLK_EXT_EN       (1u << 0)
#define EX_CLK_INT_EN       (1u << 1)
#define EX_CLK_MII_TX_EN    (1u << 3)
#define EX_CLK_MII_RX_EN    (1u << 4)
/* EMAC_EX_PHYINF_CONF */
#define EX_PHY_INTF_SHIFT   13
#define EX_PHY_INTF_MII     0
#define EX_PHY_INTF_RMII    4

#define EX_CLKOUT_RESET     0x00000024u
#define EX_CLKOUT_RW        0x000003ffu
#define EX_OSCCLK_RESET     0x00001253u
#define EX_OSCCLK_RW        0x01ffffffu
#define EX_CLK_CTRL_RW      0x0000003fu
#define EX_PHYINF_RW        0x001fffffu
#define EX_PD_SEL_RW        0x00000003u
#define EX_DATE_RESET       0x16042200u

/* The SRAM the EMAC's AHB master reaches: internal SRAM 1 and 2 */
#define DMA_RAM_START       0x3ffae000u
#define DMA_RAM_END         0x40000000u

/* The MII transmit and receive clocks the wrapper passes to the MAC */
static bool esp32_emac_tx_clk(Esp32DwmacState *d)
{
    Esp32EmacState *s = ESP32_EMAC(d);

    switch ((s->ex_phyinf_conf >> EX_PHY_INTF_SHIFT) & 7) {
    case EX_PHY_INTF_MII:
        return s->ex_clk_ctrl & EX_CLK_MII_TX_EN;
    case EX_PHY_INTF_RMII:
        return s->ex_clk_ctrl & (EX_CLK_EXT_EN | EX_CLK_INT_EN);
    default:
        return false;
    }
}

static bool esp32_emac_rx_clk(Esp32DwmacState *d)
{
    Esp32EmacState *s = ESP32_EMAC(d);

    switch ((s->ex_phyinf_conf >> EX_PHY_INTF_SHIFT) & 7) {
    case EX_PHY_INTF_MII:
        return s->ex_clk_ctrl & EX_CLK_MII_RX_EN;
    case EX_PHY_INTF_RMII:
        return s->ex_clk_ctrl & (EX_CLK_EXT_EN | EX_CLK_INT_EN);
    default:
        return false;
    }
}

/*
 * [spec:nuos:req:emu.esp32.emac]
 * A DMA access outside the SRAM the AHB master reaches is answered with
 * an error.
 */
static bool esp32_emac_dma_ok(Esp32DwmacState *d, uint32_t addr,
                              uint32_t len)
{
    return addr >= DMA_RAM_START && addr < DMA_RAM_END &&
           len <= DMA_RAM_END - addr;
}

static uint64_t esp32_emac_ex_read(Esp32EmacState *s, hwaddr addr)
{
    switch (addr) {
    case A_EX_CLKOUT_CONF:
        return s->ex_clkout_conf;
    case A_EX_OSCCLK_CONF:
        return s->ex_oscclk_conf;
    case A_EX_CLK_CTRL:
        return s->ex_clk_ctrl;
    case A_EX_PHYINF_CONF:
        return s->ex_phyinf_conf;
    case A_EX_PD_SEL:
        return s->ex_pd_sel;
    case A_EX_DATE:
        return s->ex_date;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32.emac: read of reserved offset 0x%04" HWADDR_PRIx
                      "\n", addr);
        return 0;
    }
}

static void esp32_emac_ex_write(Esp32EmacState *s, hwaddr addr, uint32_t v)
{
    unsigned intf;

    switch (addr) {
    case A_EX_CLKOUT_CONF:
        s->ex_clkout_conf = v & EX_CLKOUT_RW;
        return;
    case A_EX_OSCCLK_CONF:
        s->ex_oscclk_conf = v & EX_OSCCLK_RW;
        return;
    case A_EX_CLK_CTRL:
        s->ex_clk_ctrl = v & EX_CLK_CTRL_RW;
        esp32_dwmac_mii_changed(ESP32_DWMAC(s));
        return;
    case A_EX_PHYINF_CONF:
        intf = (v >> EX_PHY_INTF_SHIFT) & 7;
        if (intf != EX_PHY_INTF_MII && intf != EX_PHY_INTF_RMII) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "esp32.emac: reserved PHY interface %u\n", intf);
        }
        s->ex_phyinf_conf = v & EX_PHYINF_RW;
        esp32_dwmac_mii_changed(ESP32_DWMAC(s));
        return;
    case A_EX_PD_SEL:
        s->ex_pd_sel = v & EX_PD_SEL_RW;
        if (s->ex_pd_sel) {
            qemu_log_mask(LOG_UNIMP, "esp32.emac: FIFO RAM power-down\n");
        }
        return;
    case A_EX_DATE:
        s->ex_date = v;
        return;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32.emac: write to reserved offset 0x%04"
                      HWADDR_PRIx "\n", addr);
        return;
    }
}

/* [spec:nuos:req:emu.esp32.emac] */
static uint64_t esp32_emac_read(void *opaque, hwaddr addr, unsigned size)
{
    Esp32EmacState *s = opaque;
    Esp32DwmacState *d = ESP32_DWMAC(s);

    if (addr >= A_MAC_BASE) {
        return esp32_dwmac_read(d, ESP32_DWMAC_MAC_BASE + addr - A_MAC_BASE,
                                addr);
    }
    if (addr >= A_EX_BASE) {
        return esp32_emac_ex_read(s, addr);
    }
    return esp32_dwmac_read(d, ESP32_DWMAC_DMA_BASE + addr, addr);
}

/* [spec:nuos:req:emu.esp32.emac] */
static void esp32_emac_write(void *opaque, hwaddr addr, uint64_t value,
                             unsigned size)
{
    Esp32EmacState *s = opaque;
    Esp32DwmacState *d = ESP32_DWMAC(s);

    if (addr >= A_MAC_BASE) {
        esp32_dwmac_write(d, ESP32_DWMAC_MAC_BASE + addr - A_MAC_BASE, value,
                          addr);
    } else if (addr >= A_EX_BASE) {
        esp32_emac_ex_write(s, addr, value);
    } else {
        esp32_dwmac_write(d, ESP32_DWMAC_DMA_BASE + addr, value, addr);
    }
}

static const MemoryRegionOps esp32_emac_ops = {
    .read = esp32_emac_read,
    .write = esp32_emac_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void esp32_emac_reset_hold(Object *obj, ResetType type)
{
    Esp32EmacState *s = ESP32_EMAC(obj);
    Esp32EmacClass *k = ESP32_EMAC_GET_CLASS(obj);

    if (k->parent_phases.hold) {
        k->parent_phases.hold(obj, type);
    }
    s->ex_clkout_conf = EX_CLKOUT_RESET;
    s->ex_oscclk_conf = EX_OSCCLK_RESET;
    s->ex_clk_ctrl = 0;
    s->ex_phyinf_conf = 0;
    s->ex_pd_sel = 0;
    s->ex_date = EX_DATE_RESET;
}

static void esp32_emac_init(Object *obj)
{
    Esp32EmacState *s = ESP32_EMAC(obj);

    memory_region_init_io(&s->iomem, obj, &esp32_emac_ops, s, TYPE_ESP32_EMAC,
                          ESP32_EMAC_REG_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static void esp32_emac_realize(DeviceState *dev, Error **errp)
{
    Esp32EmacState *s = ESP32_EMAC(dev);
    Esp32EmacClass *k = ESP32_EMAC_GET_CLASS(dev);

    if (!s->dma_mr) {
        error_setg(errp, "esp32.emac: 'dma-mr' link not set");
        return;
    }
    address_space_init(&s->dma_as, s->dma_mr, "esp32-emac-dma");
    ESP32_DWMAC(s)->dma_as = &s->dma_as;
    k->parent_realize(dev, errp);
}

static void esp32_emac_unrealize(DeviceState *dev)
{
    Esp32EmacState *s = ESP32_EMAC(dev);
    Esp32EmacClass *k = ESP32_EMAC_GET_CLASS(dev);

    k->parent_unrealize(dev);
    address_space_destroy(&s->dma_as);
}

static const Property esp32_emac_properties[] = {
    DEFINE_PROP_LINK("dma-mr", Esp32EmacState, dma_mr, TYPE_MEMORY_REGION,
                     MemoryRegion *),
};

static const VMStateDescription vmstate_esp32_emac = {
    .name = TYPE_ESP32_EMAC,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT(parent_obj, Esp32EmacState, 1, vmstate_esp32_dwmac,
                       Esp32DwmacState),
        VMSTATE_UINT32(ex_clkout_conf, Esp32EmacState),
        VMSTATE_UINT32(ex_oscclk_conf, Esp32EmacState),
        VMSTATE_UINT32(ex_clk_ctrl, Esp32EmacState),
        VMSTATE_UINT32(ex_phyinf_conf, Esp32EmacState),
        VMSTATE_UINT32(ex_pd_sel, Esp32EmacState),
        VMSTATE_UINT32(ex_date, Esp32EmacState),
        VMSTATE_END_OF_LIST()
    }
};

static void esp32_emac_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    Esp32DwmacClass *dk = ESP32_DWMAC_CLASS(klass);
    Esp32EmacClass *k = ESP32_EMAC_CLASS(klass);

    dk->log_name = TYPE_ESP32_EMAC;
    dk->bus_clock = "apb";
    dk->addr_count = 8;
    dk->rx_fifo_size = 2048;
    dk->tx_fifo_size = 2048;
    dk->bus_mode_rw = 0x07ffffff;
    dk->bus_mode_reset = 0x00020100;
    dk->config_rw = 0x78dffeff;
    dk->config_reset = 0;
    dk->ff_rw = 0x800003f9;
    dk->ff_reset = 0;
    dk->int_mask_rw = 0x00000408;
    dk->lpi_rw = 0x000b0300;
    dk->tx_clk = esp32_emac_tx_clk;
    dk->rx_clk = esp32_emac_rx_clk;
    dk->dma_ok = esp32_emac_dma_ok;

    resettable_class_set_parent_phases(rc, NULL, esp32_emac_reset_hold, NULL,
                                       &k->parent_phases);
    device_class_set_parent_realize(dc, esp32_emac_realize,
                                    &k->parent_realize);
    device_class_set_parent_unrealize(dc, esp32_emac_unrealize,
                                      &k->parent_unrealize);
    dc->vmsd = &vmstate_esp32_emac;
    device_class_set_props(dc, esp32_emac_properties);
}

/* [spec:nuos:req:emu.esp32.emac] */
static const TypeInfo esp32_emac_info = {
    .name = TYPE_ESP32_EMAC,
    .parent = TYPE_ESP32_DWMAC,
    .instance_size = sizeof(Esp32EmacState),
    .instance_init = esp32_emac_init,
    .class_size = sizeof(Esp32EmacClass),
    .class_init = esp32_emac_class_init,
};

static void esp32_emac_register_types(void)
{
    type_register_static(&esp32_emac_info);
}

type_init(esp32_emac_register_types)
