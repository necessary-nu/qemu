/*
 * ESP32-P4 Ethernet MAC (EMAC), a Synopsys DesignWare GMAC
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 TRM (v3.x silicon), chapter 55 "Ethernet Media
 * Access Controller (EMAC)", 11 "Reset and Clock" (EMAC_*_CLK, PAD_EMAC_*,
 * RST_EN_EMAC), 21 "System Registers" (HP_SYSTEM_GMAC_CTRL0..2) and
 * 10 "GPIO Matrix and IO MUX"; ESP-IDF's v3 emac_reg.h, emac_*_struct.h,
 * and its esp32p4 emac_ll.h, emac_periph.c and esp_eth driver.
 *
 * One 16 KiB block at 0x50098000: the GMAC's MAC registers at 0x0000,
 * with the system time registers at 0x0700, and its DMA registers at
 * 0x1000. The GMAC (hw/net/esp32_dwmac.c) has 256-byte transmit and
 * receive FIFOs, nine MAC addresses, the IEEE 1588 advanced timestamp
 * engine with the PPS output, the receive VLAN tag filter and transmit
 * VLAN tag control, and DMABUSMODE.SW_RST set out of reset until its MII
 * clocks run. This is the ESP32-P4's arrangement of it:
 *
 *   - Its registers run on EMAC_SYS_CLK, the CSR clock MDC divides; it is
 *     held in reset by LP_CLKRST_RST_EN_EMAC.
 *   - The MII clocks come from HP_SYS_CLKRST: EMAC_RX_CLK and EMAC_TX_CLK
 *     in MII mode, and EMAC_RMII_CLK as well in RMII mode, where the
 *     transmit and receive clocks are the reference divided to 25 MHz
 *     (100 Mbit/s) or 2.5 MHz (10 Mbit/s); data clocked at a rate that is
 *     not EMACCONFIG.FES's, or not the PHY's, is lost. EMACCONFIG.PS clear
 *     selects the GMII port, which has no clock on this chip.
 *   - HP_SYSTEM_GMAC_CTRL0.PHY_INTF_SEL picks MII (0) or RMII (4); the
 *     GMAC takes it as a reset completes ("The PHY interface can be
 *     selected only once after the chip reset").
 *   - The DMA is a master on the AHB matrix, GMAC to the DMA APM (master
 *     21, ICM ID 5): its accesses go through the APMs, and one they or
 *     the bus refuse is a fatal bus error.
 *   - The MII reaches the PHY through pads: MDC and MDIO through the GPIO
 *     matrix (MII_MDC, MII_MDO, MII_MDI), the RMII data through the IO
 *     MUX's EMAC functions or the matrix, and the reference clock only
 *     through an IO MUX EMAC_RMII_CLK function. The board's wiring is in
 *     the "pad-*" properties; their defaults are the Waveshare
 *     ESP32-P4-WIFI6-POE-ETH's IP101GRI: MDC GPIO31, MDIO GPIO52, TX_EN
 *     GPIO49, TXD0 GPIO34, TXD1 GPIO35, CRS_DV GPIO28, RXD0 GPIO29, RXD1
 *     GPIO30 and its 50 MHz 50M_CLKO on GPIO50. A frame goes nowhere, and
 *     a management frame unanswered, where software has not routed the
 *     pins to those pads.
 *
 * The registers ESP-IDF's v3 emac_reg.h lists but the TRM does not
 * document (the hash tables, the layer 3/4 filters, the auxiliary
 * snapshots, the flexible PPS registers, the AV channels at 0x1100 and
 * 0x1200, AXI bus mode and the hardware feature register) are logged as
 * unimplemented and read as zero.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/gpio/esp32p4_gpio_sig.h"
#include "hw/misc/esp32p4_icm.h"
#include "hw/net/esp32p4_emac.h"
#include "migration/vmstate.h"

#define A_DMA_BASE          0x1000
#define A_DMA_END           0x2000

/* EMACCONFIG bits the clocking depends on */
#define CFG_FES             (1u << 14)
#define CFG_PS              (1u << 15)

/* HP_SYSTEM_GMAC_CTRL0.PHY_INTF_SEL */
#define PHY_INTF_MII        0
#define PHY_INTF_RMII       4

/* The MII transmit and receive clocks at 100 and 10 Mbit/s */
#define MII_CLK_100M        25000000u
#define MII_CLK_10M         2500000u

static ESP32P4EmacState *p4_emac(Esp32DwmacState *d)
{
    return ESP32P4_EMAC(d);
}

/*
 * The PHY interface the GMAC works with: the one it took at its last
 * reset, or, while a reset is still finishing, the one it will take.
 */
static unsigned p4_emac_intf(ESP32P4EmacState *s)
{
    Esp32DwmacState *d = ESP32_DWMAC(s);

    if (d->held || esp32_dwmac_sw_reset_pending(d)) {
        return s->phy_intf_sel;
    }
    return s->phy_intf;
}

static bool p4_emac_clk(ESP32P4EmacState *s, Clock *clk)
{
    Esp32DwmacState *d = ESP32_DWMAC(s);

    if (!(d->config & CFG_PS) || !clock_get_hz(clk)) {
        return false;
    }
    switch (p4_emac_intf(s)) {
    case PHY_INTF_RMII:
        return clock_get_hz(s->rmii_clk) != 0;
    case PHY_INTF_MII:
        return true;
    default:
        return false;
    }
}

/* [spec:nuos:req:emu.esp32p4.emac] */
static bool p4_emac_tx_clk(Esp32DwmacState *d)
{
    return p4_emac_clk(p4_emac(d), p4_emac(d)->tx_clk);
}

/* [spec:nuos:req:emu.esp32p4.emac] */
static bool p4_emac_rx_clk(Esp32DwmacState *d)
{
    return p4_emac_clk(p4_emac(d), p4_emac(d)->rx_clk);
}

/* [spec:nuos:req:emu.esp32p4.emac] */
static MemTxAttrs p4_emac_dma_attrs(Esp32DwmacState *d)
{
    return esp32p4_dma_attrs(ESP32P4_DMA_MASTER_GMAC, ESP32P4_ICM_ID_GMAC);
}

static bool p4_emac_pad(uint8_t pad)
{
    return pad < ESP32P4_GPIO_PADS;
}

/* Whether output signal @sig is routed, enabled, to board pad @pad */
static bool p4_emac_out_to(ESP32P4EmacState *s, unsigned sig, uint8_t pad)
{
    return p4_emac_pad(pad) &&
           (esp32p4_gpio_sig_out_pads(s->gpio, sig) & (1ull << pad));
}

/* Whether input signal @sig is routed from board pad @pad */
static bool p4_emac_in_from(ESP32P4EmacState *s, unsigned sig, uint8_t pad)
{
    return p4_emac_pad(pad) && esp32p4_gpio_sig_in_pad(s->gpio, sig) == pad;
}

/*
 * [spec:nuos:req:emu.esp32p4.emac]
 * A management frame reaches the PHY when MII_MDC and MII_MDO drive its
 * MDC and MDIO pads, and the MAC hears its answer when MII_MDI comes from
 * the MDIO pad. Otherwise MII_MDI reads as whatever the matrix gives it.
 */
static void p4_emac_mdio_path(Esp32DwmacState *d, Esp32DwmacMdioPath *path)
{
    ESP32P4EmacState *s = p4_emac(d);

    if (!s->gpio) {
        path->to_phy = path->from_phy = path->idle_level = true;
        return;
    }
    path->to_phy = p4_emac_out_to(s, ESP32P4_SIG_MII_MDC_PAD_OUT,
                                  s->pad_mdc) &&
                   p4_emac_out_to(s, ESP32P4_SIG_MII_MDO_PAD_OUT,
                                  s->pad_mdio);
    path->from_phy = p4_emac_in_from(s, ESP32P4_SIG_MII_MDI_PAD_IN,
                                     s->pad_mdio);
    path->idle_level = s->mdi;
}

static void p4_emac_mdio_drive(Esp32DwmacState *d, bool on)
{
    qemu_set_irq(p4_emac(d)->mdo_oe, on);
}

static void p4_emac_tx_drive(Esp32DwmacState *d, bool on)
{
    qemu_set_irq(p4_emac(d)->tx_en, on);
}

/*
 * [spec:nuos:req:emu.esp32p4.emac]
 * Whether data clocked by @clk reaches the PHY intact: in RMII mode at
 * the rate EMACCONFIG.FES selects, and at the PHY's speed when it has a
 * link.
 */
static bool p4_emac_rate_ok(ESP32P4EmacState *s, Clock *clk, bool tx)
{
    Esp32DwmacState *d = ESP32_DWMAC(s);
    bool fast = d->config & CFG_FES;
    uint64_t want = fast ? MII_CLK_100M : MII_CLK_10M;
    uint64_t hz = clock_get_hz(clk);

    if (p4_emac_intf(s) == PHY_INTF_RMII &&
        (hz < want - want / 100 || hz > want + want / 100)) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32p4.emac: RMII %s clock at %"
                      PRIu64 " Hz for %s Mbit/s; the frame is lost\n",
                      tx ? "transmit" : "receive", hz, fast ? "100" : "10");
        return false;
    }
    if (d->phy && d->phy->link_up && d->phy->speed_100 != fast) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32p4.emac: MAC at %s Mbit/s, "
                      "PHY at %s; the frame is lost\n", fast ? "100" : "10",
                      d->phy->speed_100 ? "100" : "10");
        return false;
    }
    return true;
}

/*
 * [spec:nuos:req:emu.esp32p4.emac]
 * The board's PHY talks RMII: transmit data reaches it when TX_EN, TXD0
 * and TXD1 drive its pads, receive data when CRS_DV, RXD0 and RXD1 come
 * from them.
 */
static bool p4_emac_tx_path(Esp32DwmacState *d)
{
    ESP32P4EmacState *s = p4_emac(d);

    if (s->gpio &&
        (p4_emac_intf(s) != PHY_INTF_RMII ||
         !p4_emac_out_to(s, ESP32P4_SIG_EMAC_PHY_TXEN_PAD_OUT,
                         s->pad_tx_en) ||
         !p4_emac_out_to(s, ESP32P4_SIG_EMAC_PHY_TXD0_PAD_OUT,
                         s->pad_txd0) ||
         !p4_emac_out_to(s, ESP32P4_SIG_EMAC_PHY_TXD1_PAD_OUT,
                         s->pad_txd1))) {
        return false;
    }
    return p4_emac_rate_ok(s, s->tx_clk, true);
}

/* [spec:nuos:req:emu.esp32p4.emac] */
static bool p4_emac_rx_path(Esp32DwmacState *d)
{
    ESP32P4EmacState *s = p4_emac(d);

    if (s->gpio &&
        (p4_emac_intf(s) != PHY_INTF_RMII ||
         !p4_emac_in_from(s, ESP32P4_SIG_EMAC_PHY_RXDV_PAD_IN,
                          s->pad_crs_dv) ||
         !p4_emac_in_from(s, ESP32P4_SIG_EMAC_PHY_RXD0_PAD_IN,
                          s->pad_rxd0) ||
         !p4_emac_in_from(s, ESP32P4_SIG_EMAC_PHY_RXD1_PAD_IN,
                          s->pad_rxd1))) {
        return false;
    }
    return p4_emac_rate_ok(s, s->rx_clk, false);
}

/*
 * [spec:nuos:req:emu.esp32p4.emac]
 * HP_SYSTEM_GMAC_CTRL0's GMAC_RST_CLK_TX_N and _RX_N: the GMAC's reset as
 * its transmit and receive clock domains see it, released once the core
 * is out of reset and the domain's clock runs.
 */
static void p4_emac_update_rst_n(ESP32P4EmacState *s)
{
    Esp32DwmacState *d = ESP32_DWMAC(s);
    bool out = !d->held && !esp32_dwmac_sw_reset_pending(d);

    qemu_set_irq(s->rst_clk_n[0], out && p4_emac_tx_clk(d));
    qemu_set_irq(s->rst_clk_n[1], out && p4_emac_rx_clk(d));
}

static void p4_emac_reset_changed(Esp32DwmacState *d)
{
    ESP32P4EmacState *s = p4_emac(d);

    if (!d->held && !esp32_dwmac_sw_reset_pending(d)) {
        s->phy_intf = s->phy_intf_sel;
    }
    p4_emac_update_rst_n(s);
}

/*
 * [spec:nuos:req:emu.esp32p4.emac]
 * The PHY's reference clock reaches PAD_EMAC_TXRX_CLK while the IO MUX
 * selects EMAC_RMII_CLK on its pad.
 */
static void p4_emac_update_pad_ref(ESP32P4EmacState *s)
{
    bool through = !s->gpio ||
                   (p4_emac_pad(s->pad_ref_clk) &&
                    esp32p4_gpio_iomux_in_selected(s->gpio, s->pad_ref_clk,
                        ESP32P4_EMAC_RMII_CLK_FUNC));

    s->pad_txrx_period = through ? esp32p4_clk_exact(s->phy_ref_clk)
                                 : esp32p4_clk_period_stopped();
    clock_update(s->pad_txrx_clk,
                 esp32p4_clk_period_clock(s->pad_txrx_period));
}

/*
 * [spec:nuos:req:emu.esp32p4.emac]
 * PAD_EMAC_TXRX_CLK carries the PHY's reference clock at its exact
 * period, as the TYPE_ESP32P4_CLK_SOURCE of "pad-txrx".
 */
static bool p4_emac_exact_period(Object *obj, Clock *out,
                                 ESP32P4ClkPeriod *p)
{
    ESP32P4EmacState *s = ESP32P4_EMAC(obj);

    if (out != s->pad_txrx_clk) {
        return false;
    }
    *p = s->pad_txrx_period;
    return true;
}

/* The edges of @clk from time 0, at its exact period. */
static uint64_t p4_emac_clock_edges(Esp32DwmacState *d, Clock *clk,
                                    int64_t ns)
{
    return esp32p4_clk_ns_cycles(esp32p4_clk_exact(clk), ns);
}

static int64_t p4_emac_clock_edge_ns(Esp32DwmacState *d, Clock *clk,
                                     uint64_t edge)
{
    return esp32p4_clk_cycles_ns(esp32p4_clk_exact(clk), edge);
}

static void p4_emac_route_changed(Notifier *n, void *data)
{
    ESP32P4EmacState *s = container_of(n, ESP32P4EmacState, route_notifier);

    p4_emac_update_pad_ref(s);
}

static void p4_emac_mii_clk_update(void *opaque, ClockEvent event)
{
    ESP32P4EmacState *s = opaque;

    p4_emac_update_rst_n(s);
    esp32_dwmac_mii_changed(ESP32_DWMAC(s));
}

static void p4_emac_phy_ref_update(void *opaque, ClockEvent event)
{
    p4_emac_update_pad_ref(opaque);
}

static void p4_emac_set_reset(void *opaque, int n, int level)
{
    esp32_dwmac_set_held(ESP32_DWMAC(opaque), level);
}

/* [spec:nuos:req:emu.esp32p4.emac] */
static void p4_emac_set_phy_intf(void *opaque, int n, int level)
{
    ESP32P4EmacState *s = opaque;

    s->phy_intf_sel = deposit32(s->phy_intf_sel, n, 1, level != 0);
    p4_emac_update_rst_n(s);
    esp32_dwmac_mii_changed(ESP32_DWMAC(s));
}

static void p4_emac_set_mdi(void *opaque, int n, int level)
{
    ESP32P4EmacState *s = opaque;

    s->mdi = level;
}

/*
 * The registers ESP-IDF's v3 emac_reg.h lists that the TRM does not
 * document: they may not be in the chip's configuration of the GMAC.
 */
static bool p4_emac_undocumented(hwaddr addr)
{
    static const struct {
        hwaddr start;
        hwaddr end;
    } ranges[] = {
        { 0x0008, 0x000c },   /* hash table */
        { 0x0020, 0x0020 },   /* version */
        { 0x00c0, 0x00d4 },   /* auto-negotiation, TBI */
        { 0x00e0, 0x00e0 },   /* general purpose IO */
        { 0x0400, 0x041c },   /* layer 3 and 4 filters */
        { 0x0500, 0x0500 },   /* hash table register 0 */
        { 0x0588, 0x0588 },   /* VLAN hash table */
        { 0x0730, 0x0738 },   /* auxiliary timestamp, AV MAC control */
        { 0x0760, 0x07ac },   /* flexible PPS intervals, widths, targets */
        { 0x0880, 0x0880 },   /* MAC address 32 */
        { 0x1028, 0x1028 },   /* AXI bus mode */
        { 0x1058, 0x1058 },   /* hardware features */
        { 0x1100, 0x1174 },   /* AV channel 1 */
        { 0x1200, 0x1274 },   /* AV channel 2 */
    };

    for (unsigned i = 0; i < ARRAY_SIZE(ranges); i++) {
        if (addr >= ranges[i].start && addr <= ranges[i].end) {
            return true;
        }
    }
    return false;
}

/* [spec:nuos:req:emu.esp32p4.emac] */
static uint64_t p4_emac_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4EmacState *s = opaque;

    if (p4_emac_undocumented(addr)) {
        qemu_log_mask(LOG_UNIMP, "esp32p4.emac: read of register 0x%04"
                      HWADDR_PRIx ", which the TRM does not document\n",
                      addr);
        return 0;
    }
    if (addr >= A_DMA_END) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32p4.emac: read of reserved "
                      "offset 0x%04" HWADDR_PRIx "\n", addr);
        return 0;
    }
    return esp32_dwmac_read(ESP32_DWMAC(s), addr, addr);
}

/* [spec:nuos:req:emu.esp32p4.emac] */
static void p4_emac_write(void *opaque, hwaddr addr, uint64_t value,
                          unsigned size)
{
    ESP32P4EmacState *s = opaque;

    if (p4_emac_undocumented(addr)) {
        qemu_log_mask(LOG_UNIMP, "esp32p4.emac: write of 0x%08" PRIx64
                      " to register 0x%04" HWADDR_PRIx ", which the TRM "
                      "does not document\n", value, addr);
        return;
    }
    if (addr >= A_DMA_END) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32p4.emac: write to reserved "
                      "offset 0x%04" HWADDR_PRIx "\n", addr);
        return;
    }
    esp32_dwmac_write(ESP32_DWMAC(s), addr, value, addr);
}

static const MemoryRegionOps p4_emac_ops = {
    .read = p4_emac_read,
    .write = p4_emac_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void p4_emac_reset_hold(Object *obj, ResetType type)
{
    ESP32P4EmacClass *k = ESP32P4_EMAC_GET_CLASS(obj);

    if (k->parent_phases.hold) {
        k->parent_phases.hold(obj, type);
    }
}

/*
 * The MII's idle levels out of reset: MDC low, MDIO released (the board
 * pulls it up), TX_EN low.
 */
static void p4_emac_reset_exit(Object *obj, ResetType type)
{
    ESP32P4EmacState *s = ESP32P4_EMAC(obj);

    qemu_set_irq(s->mdc, 0);
    qemu_set_irq(s->mdo, 1);
    qemu_set_irq(s->mdo_oe, 0);
    qemu_set_irq(s->tx_en, 0);
    p4_emac_update_rst_n(s);
}

static void p4_emac_init(Object *obj)
{
    ESP32P4EmacState *s = ESP32P4_EMAC(obj);
    DeviceState *dev = DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &p4_emac_ops, s, "esp32p4.emac",
                          ESP32P4_EMAC_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    s->rmii_clk = qdev_init_clock_in(dev, "rmii", p4_emac_mii_clk_update, s,
                                     ClockUpdate);
    s->rx_clk = qdev_init_clock_in(dev, "rx", p4_emac_mii_clk_update, s,
                                   ClockUpdate);
    s->tx_clk = qdev_init_clock_in(dev, "tx", p4_emac_mii_clk_update, s,
                                   ClockUpdate);
    s->phy_ref_clk = qdev_init_clock_in(dev, "phy-ref",
                                        p4_emac_phy_ref_update, s,
                                        ClockUpdate);
    s->pad_txrx_clk = qdev_init_clock_out(dev, "pad-txrx");
    s->pad_txrx_period = esp32p4_clk_period_stopped();
    qdev_init_gpio_in_named(dev, p4_emac_set_reset, ESP32P4_EMAC_RESET, 1);
    qdev_init_gpio_in_named(dev, p4_emac_set_phy_intf, ESP32P4_EMAC_PHY_INTF,
                            ESP32P4_EMAC_PHY_INTF_LINES);
    qdev_init_gpio_in_named(dev, p4_emac_set_mdi, ESP32P4_EMAC_MDI, 1);
    qdev_init_gpio_out_named(dev, &s->mdc, ESP32P4_EMAC_MDC, 1);
    qdev_init_gpio_out_named(dev, &s->mdo, ESP32P4_EMAC_MDO, 1);
    qdev_init_gpio_out_named(dev, &s->mdo_oe, ESP32P4_EMAC_MDO_OE, 1);
    qdev_init_gpio_out_named(dev, &s->tx_en, ESP32P4_EMAC_TX_EN, 1);
    qdev_init_gpio_out_named(dev, s->rst_clk_n, ESP32P4_EMAC_RST_N, 2);
    s->route_notifier.notify = p4_emac_route_changed;
    s->mdi = true;
}

static void p4_emac_realize(DeviceState *dev, Error **errp)
{
    ERRP_GUARD();
    ESP32P4EmacState *s = ESP32P4_EMAC(dev);
    ESP32P4EmacClass *k = ESP32P4_EMAC_GET_CLASS(dev);

    if (!s->pms) {
        error_setg(errp, "esp32p4.emac: 'pms' link not set");
        return;
    }
    ESP32_DWMAC(s)->dma_as = esp32p4_pms_bus_as(s->pms);
    k->parent_realize(dev, errp);
    if (*errp) {
        return;
    }
    if (s->gpio) {
        esp32p4_gpio_add_route_notifier(s->gpio, &s->route_notifier);
    }
    p4_emac_update_pad_ref(s);
}

static const Property p4_emac_properties[] = {
    DEFINE_PROP_LINK("pms", ESP32P4EmacState, pms, TYPE_ESP32P4_PMS,
                     ESP32P4PmsState *),
    DEFINE_PROP_LINK("gpio", ESP32P4EmacState, gpio, TYPE_ESP32P4_GPIO,
                     ESP32P4GpioState *),
    DEFINE_PROP_UINT8("pad-mdc", ESP32P4EmacState, pad_mdc, 31),
    DEFINE_PROP_UINT8("pad-mdio", ESP32P4EmacState, pad_mdio, 52),
    DEFINE_PROP_UINT8("pad-tx-en", ESP32P4EmacState, pad_tx_en, 49),
    DEFINE_PROP_UINT8("pad-txd0", ESP32P4EmacState, pad_txd0, 34),
    DEFINE_PROP_UINT8("pad-txd1", ESP32P4EmacState, pad_txd1, 35),
    DEFINE_PROP_UINT8("pad-crs-dv", ESP32P4EmacState, pad_crs_dv, 28),
    DEFINE_PROP_UINT8("pad-rxd0", ESP32P4EmacState, pad_rxd0, 29),
    DEFINE_PROP_UINT8("pad-rxd1", ESP32P4EmacState, pad_rxd1, 30),
    DEFINE_PROP_UINT8("pad-ref-clk", ESP32P4EmacState, pad_ref_clk, 50),
};

static const VMStateDescription vmstate_p4_emac = {
    .name = TYPE_ESP32P4_EMAC,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT(parent_obj, ESP32P4EmacState, 1, vmstate_esp32_dwmac,
                       Esp32DwmacState),
        VMSTATE_UINT8(phy_intf_sel, ESP32P4EmacState),
        VMSTATE_UINT8(phy_intf, ESP32P4EmacState),
        VMSTATE_BOOL(mdi, ESP32P4EmacState),
        VMSTATE_CLOCK(parent_obj.ptp_clk, ESP32P4EmacState),
        VMSTATE_CLOCK(rmii_clk, ESP32P4EmacState),
        VMSTATE_CLOCK(rx_clk, ESP32P4EmacState),
        VMSTATE_CLOCK(tx_clk, ESP32P4EmacState),
        VMSTATE_CLOCK(phy_ref_clk, ESP32P4EmacState),
        VMSTATE_ESP32P4_CLK_PERIOD(pad_txrx_period, ESP32P4EmacState),
        VMSTATE_END_OF_LIST()
    }
};

static void p4_emac_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    Esp32DwmacClass *dk = ESP32_DWMAC_CLASS(klass);
    ESP32P4EmacClass *k = ESP32P4_EMAC_CLASS(klass);
    ESP32P4ClkSourceClass *csc = ESP32P4_CLK_SOURCE_CLASS(klass);

    dk->log_name = "esp32p4.emac";
    dk->bus_clock = "sys";
    dk->addr_count = 9;
    dk->rx_fifo_size = 256;
    dk->tx_fifo_size = 256;
    /* TRM register 55.1, its reset value with SW_RST set */
    dk->bus_mode_rw = 0x8ffffffe;
    dk->bus_mode_reset = 0x00020101;
    /* Registers 55.16 and 55.17: PS and VTFE set out of reset */
    dk->config_rw = 0x7adffeff;
    dk->config_reset = CFG_PS;
    dk->ff_rw = 0x800103f9;
    dk->ff_reset = 0x00010000;
    dk->int_mask_rw = 0x00000608;
    dk->lpi_rw = 0x000b0000;
    dk->config_clock_bits = CFG_PS;
    dk->has_ahb_status = true;
    dk->has_ptp = true;
    dk->has_vlan = true;
    dk->tx_clk = p4_emac_tx_clk;
    dk->rx_clk = p4_emac_rx_clk;
    dk->dma_attrs = p4_emac_dma_attrs;
    dk->mdio_path = p4_emac_mdio_path;
    dk->mdio_drive = p4_emac_mdio_drive;
    dk->tx_path = p4_emac_tx_path;
    dk->rx_path = p4_emac_rx_path;
    dk->tx_drive = p4_emac_tx_drive;
    dk->reset_changed = p4_emac_reset_changed;
    dk->clock_edges = p4_emac_clock_edges;
    dk->clock_edge_ns = p4_emac_clock_edge_ns;
    csc->exact_period = p4_emac_exact_period;

    resettable_class_set_parent_phases(rc, NULL, p4_emac_reset_hold,
                                       p4_emac_reset_exit, &k->parent_phases);
    device_class_set_parent_realize(dc, p4_emac_realize, &k->parent_realize);
    dc->vmsd = &vmstate_p4_emac;
    device_class_set_props(dc, p4_emac_properties);
}

/* [spec:nuos:req:emu.esp32p4.emac] */
static const TypeInfo p4_emac_info = {
    .name = TYPE_ESP32P4_EMAC,
    .parent = TYPE_ESP32_DWMAC,
    .instance_size = sizeof(ESP32P4EmacState),
    .instance_init = p4_emac_init,
    .class_size = sizeof(ESP32P4EmacClass),
    .class_init = p4_emac_class_init,
    .interfaces = (const InterfaceInfo[]) {
        { TYPE_ESP32P4_CLK_SOURCE },
        { }
    },
};

static void p4_emac_register_types(void)
{
    type_register_static(&p4_emac_info);
}

type_init(p4_emac_register_types)
