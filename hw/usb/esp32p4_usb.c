/*
 * ESP32-P4 USB 2.0 OTG controllers and their PHYs
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapters
 * 52 (USB 2.0 High-Speed OTG) and 53 (USB 2.0 Full-Speed OTG), whose
 * register descriptions Synopsys keeps under NDA; ESP-IDF's usb_dwc_struct.h,
 * usb_wrap_struct.h and usb_utmi_struct.h, and its usb_dwc_ll.h,
 * usb_wrap_ll.h and usb_utmi_ll.h; and the cores' identification and
 * hardware configuration registers as TinyUSB's dwc2_info.md records them
 * from silicon.
 *
 * The chip has two DWC2 cores (hw/usb/dwc2-otg.c):
 *
 *   - OTG_HS at 0x50000000, a 4.30a core (GSNPSID 0x4F54430A) with 16 host
 *     channels, 16 device endpoints of which 8 IN at once, 4 KiB of FIFO
 *     RAM (896 words for the FIFOs, the rest the DMA's endpoint state),
 *     buffer and scatter/gather DMA, and multi-processor interrupts, on
 *     its UTMI+ PHY (0x5009C000). TRM 52.4.7: its ID and VBUS inputs are
 *     not wired to pins; this model takes them from the PHY's VBUS
 *     comparators, which see VBUS while the core drives it or a host at
 *     the other end of the cable supplies it, the ID pin floating high
 *     (a B-device), so that software forces host mode or overrides
 *     B-valid, as ESP-IDF and TinyUSB do.
 *   - OTG_FS at 0x50040000, a 4.00a core (GSNPSID 0x4F54400A) like the
 *     ESP32-S2's and S3's: 8 host channels, 7 device endpoints of which 5
 *     IN at once, 1 KiB of FIFO RAM (200 words for the FIFOs), with
 *     buffer and scatter/gather DMA, on a full-speed PHY that USB_WRAP
 *     (0x50080000) controls. Its UTMI+ OTG signals go through the GPIO
 *     matrix (signals 110 to 115), as ESP-IDF's usb_phy.c routes them.
 *
 * Both cores' DMA masters reach memory through the DMA access permission
 * manager as USB 2.0 High-Speed OTG (master 20, ICM ID 7) and Full-Speed
 * OTG (master 23, ICM ID 3); an access a memory controller's AXI port
 * holds waits (esp32p4_usb_dma_wait_init()). OTG_HS raises USB_OTG_INTR and, for its
 * endpoints, USB_OTG_ENDP_MULTI_PROC_INTR; OTG_FS USB_OTG11_INTR.
 *
 * Both cores count their PHY clocks at the clocks' exact periods
 * (esp32p4_clk_period.h): a (micro)frame of HFIR.FrInt PHY clocks ends at
 * the first ns that many have elapsed, counted from the first frame, at
 * whatever frequency the clock tree gives the PHY clock, whole Hz or not.
 *
 * The UTMI+ PHY's PLL makes its 60 MHz clock from the reference clock
 * while the PHY is out of reset and HP_SYSTEM_SYS_USBOTG20_CTRL does not
 * force it into reset, suspend or its PLL off; the reference's frequency
 * is not checked against HP_SYSTEM's REFCLK_MODE. Its registers FC_00 to
 * FC_07 are analog trims that hold what software writes.
 *
 * The full-speed PHY is the one of the two the OTG gets: by default the
 * eFuse USB_PHY_SEL gives USB Serial/JTAG the PHY on GPIO24/25 and OTG_FS
 * the one on GPIO26/27 (or the other way round), LP_SYS_USB_CTRL's
 * SW_HW_USB_PHY_SEL hands the choice to SW_USB_PHY_SEL. The PHY joins
 * the core to the cable while USB_WRAP_OTG_CONF.USB_PAD_ENABLE is set,
 * the external PHY is not selected (PHY_SEL, not modelled) and the pads
 * are not in test mode (USB_WRAP_TEST_CONF); PAD_PULL_OVERRIDE's D+
 * pull-up decides whether a host sees the device, as TinyUSB uses it to
 * disconnect, and the D+ pull-up shows on the PHY's pad (GPIO27, or
 * GPIO25 when the OTG has that PHY). SRP_SESSEND_OVERRIDE replaces the
 * session-end input. D+/D- exchange (EXCHG_PINS, eFuse
 * USB_OTG11_EXCHG_PINS) is not modelled and logged as such.
 *
 * [spec:nuos:req:emu.esp32p4.usb]
 * [spec:nuos:req:emu.esp32p4.gpio]
 * The PHYs' pads. While its controller's USB_PAD_ENABLE is set
 * (USB_WRAP_OTG_CONF's for the OTG's PHY, USB_SERIAL_JTAG_CONF0's for
 * USB Serial/JTAG's, which resets to 1) a PHY has its two pads: the PHY's
 * transceiver drives them, and the pads' own output drivers are off them,
 * so that a GPIO output software enables there drives nothing (the GPIO
 * block's "pad-phy"); the pads' pulls and input buffers stay as set (the
 * TRM's table 10.14-1 note: a USB pad's pull-up is its own or its PHY's).
 * ESP-IDF's gpio_ll_func_sel() clears the controller's USB_PAD_ENABLE
 * before it gives one of these pads an IO MUX function, and so takes the
 * pads from the PHY; the device then detaches, the PHY's pull-up gone
 * (USB Serial/JTAG's host sees no D+ pull-up; the OTG's PHY disconnects).
 * Software that takes the pads without clearing it gets no output on
 * them, the USB untouched.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/esp32p4_clk_period.h"
#include "hw/usb/esp32p4_usb.h"
#include "migration/vmstate.h"

/* The cores */

static void esp32p4_usb_otg_hs_init(Object *obj)
{
    DWC2OTGState *s = DWC2_OTG(obj);

    s->name = g_strdup("esp32p4.usb-otg-hs");
    s->snpsid = 0x4f54430a;
    s->hwcfg1 = 0;
    s->hwcfg2 = 0x215fffd0;
    s->hwcfg3 = 0x03805eb5;
    s->hwcfg4 = 0xdff1a030;
    s->ram_words = 1024;
}

static void esp32p4_usb_otg_fs_init(Object *obj)
{
    DWC2OTGState *s = DWC2_OTG(obj);

    s->name = g_strdup("esp32p4.usb-otg-fs");
    s->snpsid = 0x4f54400a;
    s->hwcfg1 = 0;
    s->hwcfg2 = 0x224dd930;
    s->hwcfg3 = 0x00c804b5;
    s->hwcfg4 = 0xd3f0a030;
    s->ram_words = 256;
}

/* The PHY clocks' cycles, at the clock tree's exact periods. */
static uint64_t esp32p4_usb_clock_cycles(DWC2OTGState *s, Clock *clk,
                                         int64_t ns)
{
    return esp32p4_clk_ns_cycles(esp32p4_clk_exact(clk), ns);
}

static int64_t esp32p4_usb_clock_cycles_ns(DWC2OTGState *s, Clock *clk,
                                           uint64_t n)
{
    return esp32p4_clk_cycles_ns(esp32p4_clk_exact(clk), n);
}

/* And their edges, with their phase. */
static uint64_t esp32p4_usb_clock_count(DWC2OTGState *s, Clock *clk,
                                        int64_t from, int64_t t)
{
    ESP32P4ClkEdges e = esp32p4_clk_edges_or_at(clk, from);

    return esp32p4_clk_edges_between(&e, from, t);
}

static int64_t esp32p4_usb_clock_after(DWC2OTGState *s, Clock *clk,
                                       int64_t from, uint64_t n)
{
    ESP32P4ClkEdges e = esp32p4_clk_edges_or_at(clk, from);

    return esp32p4_clk_edges_after(&e, from, n);
}

/* [spec:nuos:req:emu.esp32p4.usb] */
static bool esp32p4_usb_dma_wait(void *opaque, uint32_t addr, uint32_t len,
                                 bool write)
{
    ESP32P4UsbDmaWait *w = opaque;

    return esp32p4_mspi_dma_wait_on(&w->wait, w->core->dma_as, addr, len,
                                    w->core->dma_attrs, write);
}

static void esp32p4_usb_dma_ready(void *opaque)
{
    ESP32P4UsbDmaWait *w = opaque;

    dwc2_otg_dma_ready(w->core);
}

void esp32p4_usb_dma_wait_init(ESP32P4UsbDmaWait *w, DWC2OTGState *core)
{
    w->core = core;
    esp32p4_mspi_dma_wait_init(&w->wait, esp32p4_usb_dma_ready, w);
    dwc2_otg_set_dma_wait(core, esp32p4_usb_dma_wait, w);
}

static void esp32p4_usb_otg_class_init(ObjectClass *klass, const void *data)
{
    DWC2OTGClass *oc = DWC2_OTG_CLASS(klass);

    oc->clock_cycles = esp32p4_usb_clock_cycles;
    oc->clock_cycles_ns = esp32p4_usb_clock_cycles_ns;
    oc->clock_count = esp32p4_usb_clock_count;
    oc->clock_after = esp32p4_usb_clock_after;
}

/* USB_WRAP */

#define A_OTG_CONF              0x000
#define A_TEST_CONF             0x004
#define A_DATE                  0x3fc
#define WRAP_DATE               0x23030504

#define OTG_CONF_SRP_SESSEND_OVERRIDE   BIT(0)
#define OTG_CONF_SRP_SESSEND_VALUE      BIT(1)
#define OTG_CONF_PHY_SEL                BIT(2)
#define OTG_CONF_EXCHG_PINS_OVERRIDE    BIT(5)
#define OTG_CONF_EXCHG_PINS             BIT(6)
#define OTG_CONF_PAD_PULL_OVERRIDE      BIT(12)
#define OTG_CONF_DP_PULLUP              BIT(13)
#define OTG_CONF_DM_PULLUP              BIT(15)
#define OTG_CONF_USB_PAD_ENABLE         BIT(18)
#define OTG_CONF_PHY_CLK_FORCE_ON       BIT(20)
#define OTG_CONF_WRITABLE               0x807fffff
#define TEST_CONF_ENABLE                BIT(0)
#define TEST_CONF_OE                    BIT(1)
#define TEST_CONF_TX_DP                 BIT(2)
#define TEST_CONF_TX_DM                 BIT(3)
#define TEST_CONF_RX_RCV                BIT(4)
#define TEST_CONF_RX_DP                 BIT(5)
#define TEST_CONF_RX_DM                 BIT(6)
#define TEST_CONF_WRITABLE              0xf

/* eFuse fields: BLOCK0 RD_REPEAT_DATA0 and BLOCK10 (SYS_PART2) word 7. */
#define EFUSE_USB_PHY_SEL_WORD          1
#define EFUSE_USB_PHY_SEL_BIT           25
#define EFUSE_OTG11_EXCHG_BLOCK         10
#define EFUSE_OTG11_EXCHG_WORD          7
#define EFUSE_OTG11_EXCHG_BIT           5

static bool wrap_efuse_bit(ESP32P4UsbWrapState *s, unsigned block,
                           unsigned word, unsigned bit)
{
    return s->efuse && extract32(esp32p4_efuse_word(s->efuse, block, word),
                                 bit, 1);
}

/*
 * Which full-speed PHY the OTG has: SW_USB_PHY_SEL clear gives it the one
 * on GPIO26/27 (USB Serial/JTAG the one on GPIO24/25), and so does the
 * eFuse USB_PHY_SEL clear when SW_HW_USB_PHY_SEL leaves it to the eFuse.
 */
static int wrap_otg_phy(ESP32P4UsbWrapState *s)
{
    bool swap = s->sw_phy_sel ? s->sw_phy_val
                              : wrap_efuse_bit(s, 0, EFUSE_USB_PHY_SEL_WORD,
                                               EFUSE_USB_PHY_SEL_BIT);

    return swap ? 0 : 1;
}

/* [spec:nuos:req:emu.esp32p4.usb] */
static void wrap_update(ESP32P4UsbWrapState *s)
{
    bool exchg = (s->otg_conf & OTG_CONF_EXCHG_PINS_OVERRIDE)
                 ? s->otg_conf & OTG_CONF_EXCHG_PINS
                 : wrap_efuse_bit(s, EFUSE_OTG11_EXCHG_BLOCK,
                                  EFUSE_OTG11_EXCHG_WORD,
                                  EFUSE_OTG11_EXCHG_BIT);
    bool connected = (s->otg_conf & OTG_CONF_USB_PAD_ENABLE) &&
                     !(s->otg_conf & OTG_CONF_PHY_SEL) &&
                     !(s->test_conf & TEST_CONF_ENABLE);
    bool pullup;

    if (s->otg_conf & OTG_CONF_PHY_SEL) {
        qemu_log_mask(LOG_UNIMP, "%s: the external full-speed PHY "
                      "(USB_WRAP_OTG_CONF.PHY_SEL) is not modelled\n",
                      TYPE_ESP32P4_USB_WRAP);
    }
    if (exchg && connected) {
        qemu_log_mask(LOG_UNIMP, "%s: the D+/D- exchange is not modelled\n",
                      TYPE_ESP32P4_USB_WRAP);
    }
    qemu_set_irq(s->phy_connected, connected);
    qemu_set_irq(s->pull_override[0],
                 !!(s->otg_conf & OTG_CONF_PAD_PULL_OVERRIDE));
    qemu_set_irq(s->pull_override[1], !!(s->otg_conf & OTG_CONF_DP_PULLUP));
    qemu_set_irq(s->otg_phy, wrap_otg_phy(s));
    qemu_set_irq(s->sessend, (s->otg_conf & OTG_CONF_SRP_SESSEND_OVERRIDE)
                             ? !!(s->otg_conf & OTG_CONF_SRP_SESSEND_VALUE)
                             : s->sessend_in);
    /* The PHY's D+ pull-up: the override's, or the core's. */
    pullup = connected && ((s->otg_conf & OTG_CONF_PAD_PULL_OVERRIDE)
                           ? !!(s->otg_conf & OTG_CONF_DP_PULLUP)
                           : s->core_dp_pullup);
    for (int phy = 0; phy < 2; phy++) {
        bool otg = wrap_otg_phy(s) == phy;
        bool own = otg ? !!(s->otg_conf & OTG_CONF_USB_PAD_ENABLE)
                       : s->usj_pad_enable;
        bool dm = otg ? connected &&
                        (s->otg_conf & OTG_CONF_PAD_PULL_OVERRIDE) &&
                        (s->otg_conf & OTG_CONF_DM_PULLUP)
                      : s->usj_pull_up[0];
        bool dp = otg ? pullup : s->usj_pull_up[1];

        qemu_set_irq(s->pad_phy[2 * phy], own);
        qemu_set_irq(s->pad_phy[2 * phy + 1], own);
        qemu_set_irq(s->pad_pull_up[2 * phy], dm);
        qemu_set_irq(s->pad_pull_up[2 * phy + 1], dp);
    }
}

/* The pads as the test mode reads them: driven by it, or by the pulls. */
static uint32_t wrap_test_conf(ESP32P4UsbWrapState *s)
{
    uint32_t v = s->test_conf & TEST_CONF_WRITABLE;
    bool dp, dm;

    if (!(v & TEST_CONF_ENABLE)) {
        return v;
    }
    if (!(v & TEST_CONF_OE)) {
        dp = v & TEST_CONF_TX_DP;
        dm = v & TEST_CONF_TX_DM;
    } else {
        bool pulls = s->otg_conf & OTG_CONF_PAD_PULL_OVERRIDE;

        dp = pulls && (s->otg_conf & OTG_CONF_DP_PULLUP);
        dm = pulls && (s->otg_conf & OTG_CONF_DM_PULLUP);
    }
    if (dp) {
        v |= TEST_CONF_RX_DP;
    }
    if (dm) {
        v |= TEST_CONF_RX_DM;
    }
    if (dp && !dm) {
        v |= TEST_CONF_RX_RCV;
    }
    return v;
}

static uint64_t wrap_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4UsbWrapState *s = opaque;

    switch (addr) {
    case A_OTG_CONF:
        return s->otg_conf;
    case A_TEST_CONF:
        return wrap_test_conf(s);
    case A_DATE:
        return WRAP_DATE;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of reserved offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_USB_WRAP, addr);
        return 0;
    }
}

/* [spec:nuos:req:emu.esp32p4.usb] */
static void wrap_write(void *opaque, hwaddr addr, uint64_t data,
                       unsigned size)
{
    ESP32P4UsbWrapState *s = opaque;

    switch (addr) {
    case A_OTG_CONF:
        s->otg_conf = data & OTG_CONF_WRITABLE;
        break;
    case A_TEST_CONF:
        s->test_conf = data & TEST_CONF_WRITABLE;
        break;
    case A_DATE:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only DATE\n",
                      TYPE_ESP32P4_USB_WRAP);
        return;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to reserved offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_USB_WRAP, addr);
        return;
    }
    wrap_update(s);
}

static const MemoryRegionOps wrap_ops = {
    .read = wrap_read,
    .write = wrap_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void wrap_phy_sel_in(void *opaque, int n, int level)
{
    ESP32P4UsbWrapState *s = opaque;

    if (n == 0) {
        s->sw_phy_sel = level;
    } else {
        s->sw_phy_val = level;
    }
    wrap_update(s);
}

static void wrap_sessend_in(void *opaque, int n, int level)
{
    ESP32P4UsbWrapState *s = opaque;

    s->sessend_in = level;
    wrap_update(s);
}

static void wrap_core_dp_pullup_in(void *opaque, int n, int level)
{
    ESP32P4UsbWrapState *s = opaque;

    if (s->core_dp_pullup == !!level) {
        return;
    }
    s->core_dp_pullup = level;
    wrap_update(s);
}

/* [spec:nuos:req:emu.esp32p4.usb] */
static void wrap_usj_pull_up_in(void *opaque, int n, int level)
{
    ESP32P4UsbWrapState *s = opaque;

    s->usj_pull_up[n] = level;
    wrap_update(s);
}

/* [spec:nuos:req:emu.esp32p4.usb] */
static void wrap_usj_pad_enable_in(void *opaque, int n, int level)
{
    ESP32P4UsbWrapState *s = opaque;

    s->usj_pad_enable = level;
    wrap_update(s);
}

static void wrap_reset_hold(Object *obj, ResetType type)
{
    ESP32P4UsbWrapState *s = ESP32P4_USB_WRAP(obj);

    s->otg_conf = OTG_CONF_PHY_CLK_FORCE_ON;
    s->test_conf = 0;
}

static void wrap_reset_exit(Object *obj, ResetType type)
{
    wrap_update(ESP32P4_USB_WRAP(obj));
}

static void wrap_init(Object *obj)
{
    ESP32P4UsbWrapState *s = ESP32P4_USB_WRAP(obj);
    DeviceState *dev = DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &wrap_ops, s,
                          TYPE_ESP32P4_USB_WRAP, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    qdev_init_gpio_in_named(dev, wrap_phy_sel_in, ESP32P4_USB_WRAP_PHY_SEL,
                            2);
    qdev_init_gpio_out_named(dev, &s->phy_connected, DWC2_OTG_PHY_CONNECTED,
                             1);
    qdev_init_gpio_out_named(dev, s->pull_override, DWC2_OTG_PULL_OVERRIDE,
                             2);
    qdev_init_gpio_in_named(dev, wrap_sessend_in,
                            ESP32P4_USB_WRAP_SESSEND_IN, 1);
    qdev_init_gpio_out_named(dev, &s->otg_phy, ESP32P4_USB_WRAP_OTG_PHY, 1);
    qdev_init_gpio_out_named(dev, &s->sessend, ESP32P4_USB_WRAP_SESSEND, 1);
    qdev_init_gpio_in_named(dev, wrap_core_dp_pullup_in,
                            ESP32P4_USB_WRAP_CORE_DP_PULLUP, 1);
    qdev_init_gpio_in_named(dev, wrap_usj_pull_up_in,
                            ESP32P4_USB_WRAP_USJ_PULL_UP, 2);
    qdev_init_gpio_in_named(dev, wrap_usj_pad_enable_in,
                            ESP32P4_USB_WRAP_USJ_PAD_ENABLE, 1);
    qdev_init_gpio_out_named(dev, s->pad_pull_up,
                             ESP32P4_USB_WRAP_PAD_PULL_UP,
                             ESP32P4_USB_FS_PADS);
    qdev_init_gpio_out_named(dev, s->pad_phy, ESP32P4_USB_WRAP_PAD_PHY,
                             ESP32P4_USB_FS_PADS);
    /* USB Serial/JTAG's PHY has its pads, and pulls D+ up, from reset. */
    s->usj_pad_enable = true;
    s->usj_pull_up[1] = true;
}

static const VMStateDescription vmstate_wrap = {
    .name = TYPE_ESP32P4_USB_WRAP,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(otg_conf, ESP32P4UsbWrapState),
        VMSTATE_UINT32(test_conf, ESP32P4UsbWrapState),
        VMSTATE_BOOL(sw_phy_sel, ESP32P4UsbWrapState),
        VMSTATE_BOOL(sw_phy_val, ESP32P4UsbWrapState),
        VMSTATE_BOOL(sessend_in, ESP32P4UsbWrapState),
        VMSTATE_BOOL(core_dp_pullup, ESP32P4UsbWrapState),
        VMSTATE_BOOL_ARRAY(usj_pull_up, ESP32P4UsbWrapState, 2),
        VMSTATE_BOOL(usj_pad_enable, ESP32P4UsbWrapState),
        VMSTATE_END_OF_LIST()
    },
};

static const Property wrap_properties[] = {
    DEFINE_PROP_LINK("efuse", ESP32P4UsbWrapState, efuse,
                     TYPE_ESP32P4_EFUSE, ESP32P4EfuseState *),
};

static void wrap_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->desc = "ESP32-P4 USB_WRAP (full-speed OTG PHY control)";
    dc->vmsd = &vmstate_wrap;
    rc->phases.hold = wrap_reset_hold;
    rc->phases.exit = wrap_reset_exit;
    device_class_set_props(dc, wrap_properties);
}

/* The UTMI+ PHY */

#define UTMI_HZ                 60000000

/* The PLL's 60 MHz, exactly: 50/3 ns. */
static ESP32P4ClkPeriod utmi_period(void)
{
    return esp32p4_clk_period_of_hz(UTMI_HZ, 1);
}

static const uint8_t utmi_reset[ESP32P4_USB_UTMI_REGS] = {
    0x80, 0xf2, 0x47, 0x05, 0x00, 0xc1, 0x40, 0x00,
};

static const uint8_t utmi_writable[ESP32P4_USB_UTMI_REGS] = {
    0xff, 0xff, 0x7f, 0x7f, 0xff, 0xff, 0x7f, 0x07,
};

/*
 * The PHY's PLL runs from the reference clock unless the PHY is in reset,
 * or HP_SYSTEM forces it into reset or suspend or its PLL off.
 */
/* [spec:nuos:req:emu.esp32p4.usb] */
static void utmi_update_clock(ESP32P4UsbUtmiState *s)
{
    bool *c = s->ctrl;
    bool on = clock_is_enabled(s->refclk) && !s->phy_reset &&
              !(c[ESP32P4_USB_UTMI_CTRL_PHY_RESET_FORCE_EN] &&
                !c[ESP32P4_USB_UTMI_CTRL_PHY_RSTN]) &&
              !(c[ESP32P4_USB_UTMI_CTRL_PHY_SUSPEND_FORCE_EN] &&
                !c[ESP32P4_USB_UTMI_CTRL_PHY_SUSPENDM]) &&
              (!c[ESP32P4_USB_UTMI_CTRL_PHY_PLL_FORCE_EN] ||
               c[ESP32P4_USB_UTMI_CTRL_PHY_PLL_EN]);

    ESP32P4ClkEdges old = s->utmi_edges, ref;

    if (!on) {
        s->utmi_edges = esp32p4_clk_edges_stopped();
    } else if (!esp32p4_clk_edges_running(&old)) {
        /* The PLL locks to the reference clock's first edge from now. */
        ref = esp32p4_clk_edges_or_at(s->refclk,
                                      qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
        s->utmi_edges = esp32p4_clk_edges_from(
            utmi_period(), &ref, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    }
    esp32p4_clk_update(s->utmi_clk, &old, &s->utmi_edges);
}

/* [spec:nuos:req:emu.esp32p4.usb] */
static bool utmi_exact_edges(Object *obj, Clock *out, ESP32P4ClkEdges *e)
{
    ESP32P4UsbUtmiState *s = ESP32P4_USB_UTMI(obj);

    if (out != s->utmi_clk) {
        return false;
    }
    *e = s->utmi_edges;
    return true;
}

/*
 * The VBUS comparators: VBUS is on the cable while the core drives it as
 * a host or a host at the other end supplies it.
 */
static void utmi_update_vbus(ESP32P4UsbUtmiState *s)
{
    bool vbus = s->drvvbus || s->host_attached;

    qemu_set_irq(s->sense[DWC2_OTG_SENSE_IDDIG], 1);
    qemu_set_irq(s->sense[DWC2_OTG_SENSE_AVALID], vbus);
    qemu_set_irq(s->sense[DWC2_OTG_SENSE_BVALID], vbus);
    qemu_set_irq(s->sense[DWC2_OTG_SENSE_VBUSVALID], vbus);
    qemu_set_irq(s->sense[DWC2_OTG_SENSE_SESSEND], !vbus);
    qemu_set_irq(s->phy_st[0], !vbus);
    qemu_set_irq(s->phy_st[1], vbus);
    qemu_set_irq(s->phy_st[2], vbus);
    qemu_set_irq(s->phy_st[3], vbus);
}

static void utmi_refclk_changed(void *opaque, ClockEvent event)
{
    utmi_update_clock(opaque);
}

static void utmi_phy_reset_in(void *opaque, int n, int level)
{
    ESP32P4UsbUtmiState *s = opaque;

    s->phy_reset = level;
    utmi_update_clock(s);
}

static void utmi_ctrl_in(void *opaque, int n, int level)
{
    ESP32P4UsbUtmiState *s = opaque;

    s->ctrl[n] = level;
    utmi_update_clock(s);
}

static void utmi_drvvbus_in(void *opaque, int n, int level)
{
    ESP32P4UsbUtmiState *s = opaque;

    s->drvvbus = level;
    utmi_update_vbus(s);
}

static void utmi_host_attached_in(void *opaque, int n, int level)
{
    ESP32P4UsbUtmiState *s = opaque;

    s->host_attached = level;
    utmi_update_vbus(s);
}

static uint64_t utmi_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4UsbUtmiState *s = opaque;

    if (addr >= ESP32P4_USB_UTMI_REGS * 4) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of reserved offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_USB_UTMI, addr);
        return 0;
    }
    return s->fc[addr / 4];
}

/* [spec:nuos:req:emu.esp32p4.usb] */
static void utmi_write(void *opaque, hwaddr addr, uint64_t data,
                       unsigned size)
{
    ESP32P4UsbUtmiState *s = opaque;

    if (addr >= ESP32P4_USB_UTMI_REGS * 4) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to reserved offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_USB_UTMI, addr);
        return;
    }
    s->fc[addr / 4] = data & utmi_writable[addr / 4];
}

static const MemoryRegionOps utmi_ops = {
    .read = utmi_read,
    .write = utmi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void utmi_reset_hold(Object *obj, ResetType type)
{
    ESP32P4UsbUtmiState *s = ESP32P4_USB_UTMI(obj);

    memcpy(s->fc, utmi_reset, sizeof(s->fc));
}

static void utmi_reset_exit(Object *obj, ResetType type)
{
    ESP32P4UsbUtmiState *s = ESP32P4_USB_UTMI(obj);

    utmi_update_clock(s);
    utmi_update_vbus(s);
}

static void utmi_init(Object *obj)
{
    ESP32P4UsbUtmiState *s = ESP32P4_USB_UTMI(obj);
    DeviceState *dev = DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &utmi_ops, s,
                          TYPE_ESP32P4_USB_UTMI, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    s->refclk = qdev_init_clock_in(dev, "refclk", utmi_refclk_changed, s,
                                   ClockUpdate);
    s->utmi_clk = qdev_init_clock_out(dev, "utmi");
    qdev_init_gpio_in_named(dev, utmi_phy_reset_in,
                            ESP32P4_USB_UTMI_PHY_RESET, 1);
    qdev_init_gpio_in_named(dev, utmi_ctrl_in, ESP32P4_USB_UTMI_SYS_CTRL,
                            ESP32P4_USB_UTMI_CTRL_NUM);
    qdev_init_gpio_in_named(dev, utmi_drvvbus_in, ESP32P4_USB_UTMI_DRVVBUS,
                            1);
    qdev_init_gpio_in_named(dev, utmi_host_attached_in,
                            ESP32P4_USB_UTMI_HOST_ATTACHED, 1);
    qdev_init_gpio_out_named(dev, s->sense, DWC2_OTG_SENSE,
                             DWC2_OTG_SENSE_NUM);
    qdev_init_gpio_out_named(dev, s->phy_st, ESP32P4_USB_UTMI_PHY_ST, 4);
    /* HP_SYSTEM_SYS_USBOTG20_CTRL's reset: PHY_RSTN set. */
    s->ctrl[ESP32P4_USB_UTMI_CTRL_PHY_RSTN] = true;
}

static const VMStateDescription vmstate_utmi = {
    .name = TYPE_ESP32P4_USB_UTMI,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_CLOCK(refclk, ESP32P4UsbUtmiState),
        VMSTATE_ESP32P4_CLK_EDGES(utmi_edges, ESP32P4UsbUtmiState),
        VMSTATE_UINT8_ARRAY(fc, ESP32P4UsbUtmiState, ESP32P4_USB_UTMI_REGS),
        VMSTATE_BOOL(phy_reset, ESP32P4UsbUtmiState),
        VMSTATE_BOOL_ARRAY(ctrl, ESP32P4UsbUtmiState,
                           ESP32P4_USB_UTMI_CTRL_NUM),
        VMSTATE_BOOL(drvvbus, ESP32P4UsbUtmiState),
        VMSTATE_BOOL(host_attached, ESP32P4UsbUtmiState),
        VMSTATE_END_OF_LIST()
    },
};

static void utmi_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    ESP32P4ClkSourceClass *csc = ESP32P4_CLK_SOURCE_CLASS(klass);

    dc->desc = "ESP32-P4 USB 2.0 high-speed OTG UTMI+ PHY";
    dc->vmsd = &vmstate_utmi;
    rc->phases.hold = utmi_reset_hold;
    rc->phases.exit = utmi_reset_exit;
    csc->exact_edges = utmi_exact_edges;
}

/* [spec:nuos:req:emu.esp32p4.usb] */
static const TypeInfo esp32p4_usb_types[] = {
    {
        .name          = TYPE_ESP32P4_USB_OTG_HS,
        .parent        = TYPE_DWC2_OTG,
        .instance_init = esp32p4_usb_otg_hs_init,
        .class_init    = esp32p4_usb_otg_class_init,
    },
    {
        .name          = TYPE_ESP32P4_USB_OTG_FS,
        .parent        = TYPE_DWC2_OTG,
        .instance_init = esp32p4_usb_otg_fs_init,
        .class_init    = esp32p4_usb_otg_class_init,
    },
    {
        .name          = TYPE_ESP32P4_USB_WRAP,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(ESP32P4UsbWrapState),
        .instance_init = wrap_init,
        .class_init    = wrap_class_init,
    },
    {
        .name          = TYPE_ESP32P4_USB_UTMI,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(ESP32P4UsbUtmiState),
        .instance_init = utmi_init,
        .class_init    = utmi_class_init,
        .interfaces    = (const InterfaceInfo[]) {
            { TYPE_ESP32P4_CLK_SOURCE },
            { }
        },
    },
};

DEFINE_TYPES(esp32p4_usb_types)
