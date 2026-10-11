/*
 * ESP32-P4 HP system registers (HP_SYSTEM)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapters
 * 21 (System Registers), 9.3.2 (HP L2MEM) and 34 (External Memory
 * Encryption and Decryption), and ESP-IDF's hp_system_reg.h for v3.
 *
 * Of the block:
 *
 *   - HP_SYSTEM_CPU_INT_FROM_CPU_0_REG to _3_REG (0x10 to 0x1C): the
 *     software interrupts through which the HP cores signal each other,
 *     CPU_INTR_FROM_CPU_0 to _3, sources 79 to 82 of the interrupt matrix.
 *     Each register's bit 0 is read-write and its interrupt is high while
 *     it is set; software clears it by writing 0, as ESP-IDF's
 *     crosscore_int_ll does.
 *   - HP_SYSTEM_CACHE_CLK_CONFIG_REG (0x20) and _CACHE_RESET_CONFIG_REG
 *     (0x24): the L1 caches' resets, each emptying its cache, to the cache
 *     controller ("cache-reset" outputs); the caches' clock enables are
 *     held, a cache with its clock stopped working on (logged as
 *     unimplemented).
 *   - HP_SYSTEM_CRYPTO_CTRL_REG (0x70): the four enables of the external
 *     memory encryption blocks, which go to the MSPI controllers
 *     ("crypto-ctrl" outputs).
 *   - HP_SYSTEM_L2_MEM_ERR_RESP_CTRL_REG (0x198): whether an access to the
 *     part of L2MEM the L2 cache occupies is answered with an error, to the
 *     cache controller ("l2mem-err-resp" output); and the L2MEM interrupt
 *     registers (0x9C to 0xA8) with L2_MEM_INT_RECORD0 (0xB0), whose
 *     EXCEED_ADDR interrupt records such an access, as the cache
 *     controller reports it (esp32p4_hp_sys_l2mem_exceed()): its offset
 *     into L2MEM and whether it wrote. The master that made it is recorded
 *     as 0, as its encoding is not published. The ECC and error-response
 *     interrupts never raise: L2MEM has no ECC errors here, and nothing
 *     else answers L2MEM accesses with an error. Their interrupt is the
 *     HP_SYSREG source (sysbus IRQ 0).
 *
 *   - HP_SYSTEM_GMAC_CTRL0_REG (0x14C) to _CTRL2_REG (0x154): the EMAC's
 *     PHY interface (PHY_INTF_SEL) and sideband flow control request
 *     (SBD_FLOWCTRL), to the EMAC ("gmac-phy-intf-sel",
 *     "gmac-sbd-flowctrl" outputs); its PPS output and its resets in its
 *     transmit and receive clock domains as the EMAC drives them
 *     ("gmac-status" inputs); and its system time, read from the EMAC.
 *     GMAC_MEM_CLK_FORCE_ON is held: the EMAC's FIFO memories always run.
 *
 *   - HP_SYSTEM_USB20OTG_MEM_CTRL_REG (0xF8), HP_SYSTEM_SYS_USBOTG20_CTRL_REG
 *     (0x15C) and HP_SYSTEM_USB_OTGHS_PHY_ST_REG (0x1E8): the high-speed
 *     OTG's UTMI+ PHY. SYS_USBOTG20_CTRL's suspend, reset and PLL forces
 *     go to the PHY ("usbotg20-ctrl" outputs), which stops its clock as
 *     they say; its other fields (the FIFO RAM's power controls, the
 *     transmit bit stuffing and reference clock mode) hold what software
 *     writes, and the PHY's BIST (OTG_PHY_BISTEN) is logged as
 *     unimplemented, its TEST_DONE reading 0. USB_OTGHS_PHY_ST reads the
 *     PHY's VBUS comparators ("usb-otghs-phy-st" inputs).
 *
 * The rest of the block's registers are logged as unimplemented and read
 * as zero.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "hw/core/irq.h"
#include "hw/misc/esp32p4_hp_sys.h"
#include "migration/vmstate.h"

#define A_CPU_INT_FROM_CPU_0    0x10
#define A_CPU_INT_FROM_CPU_3    0x1c
#define CPU_INT_FROM_CPU        BIT(0)
#define A_CACHE_CLK_CONFIG      0x20
#define CACHE_CLK_CONFIG_MASK   0x33
#define A_CACHE_RESET_CONFIG    0x24
#define CACHE_RESET_CONFIG_MASK 0x32
#define A_CRYPTO_CTRL           0x70
#define CRYPTO_CTRL_MASK        0xf
#define A_L2_MEM_INT_RAW        0x9c
#define A_L2_MEM_INT_ST         0xa0
#define A_L2_MEM_INT_ENA        0xa4
#define A_L2_MEM_INT_CLR        0xa8
#define L2_MEM_INT_MASK         0x7
#define L2_MEM_EXCEED_ADDR_INT  BIT(1)
#define A_L2_MEM_INT_RECORD0    0xb0
#define RECORD0_ADDR_MASK       0x1fffff
#define RECORD0_WE              BIT(21)
#define A_GMAC_CTRL0            0x14c
#define GMAC_CTRL0_PTP_PPS      BIT(0)
#define GMAC_CTRL0_SBD_FLOWCTRL BIT(1)
#define GMAC_CTRL0_PHY_INTF_SHIFT 2
#define GMAC_CTRL0_PHY_INTF     (7u << GMAC_CTRL0_PHY_INTF_SHIFT)
#define GMAC_CTRL0_RST_CLK_TX_N BIT(6)
#define GMAC_CTRL0_RST_CLK_RX_N BIT(7)
#define GMAC_CTRL0_RW           0x3e
#define A_GMAC_CTRL1            0x150
#define A_GMAC_CTRL2            0x154
#define A_L2_MEM_ERR_RESP_CTRL  0x198
#define L2_MEM_ERR_RESP_EN      BIT(0)
#define A_USB20OTG_MEM_CTRL     0xf8
#define USB20OTG_MEM_CTRL_MASK  BIT(0)
#define A_SYS_USBOTG20_CTRL     0x15c
#define USBOTG20_CTRL_MASK      0x01fffffe
#define USBOTG20_CTRL_RESET     0x00822640
#define USBOTG20_CTRL_PHY_SHIFT 15
#define USBOTG20_CTRL_BISTEN    BIT(24)
#define A_USB_OTGHS_PHY_ST      0x1e8

static bool esp32p4_hp_sys_is_cpu_intr(hwaddr addr)
{
    return addr >= A_CPU_INT_FROM_CPU_0 && addr < A_CPU_INT_FROM_CPU_3 + 4;
}

/* [spec:nuos:req:emu.esp32p4.memory] */
static void esp32p4_hp_sys_update_irq(ESP32P4HPSysState *s)
{
    qemu_set_irq(s->sysreg_irq, s->l2_mem_int_raw & s->l2_mem_int_ena);
}

/* [spec:nuos:req:emu.esp32p4.memory] */
void esp32p4_hp_sys_l2mem_exceed(ESP32P4HPSysState *s, uint32_t offset,
                                 bool write)
{
    if (!(s->l2_mem_int_raw & L2_MEM_EXCEED_ADDR_INT)) {
        s->l2_mem_int_record0 = (offset & RECORD0_ADDR_MASK) |
                                (write ? RECORD0_WE : 0);
    }
    s->l2_mem_int_raw |= L2_MEM_EXCEED_ADDR_INT;
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

static uint32_t esp32p4_hp_sys_reg(ESP32P4HPSysState *s, hwaddr addr,
                                   bool *known)
{
    *known = true;
    if (esp32p4_hp_sys_is_cpu_intr(addr)) {
        return s->cpu_int_from_cpu[(addr - A_CPU_INT_FROM_CPU_0) / 4];
    }
    switch (addr & ~3) {
    case A_CACHE_CLK_CONFIG:
        return s->cache_clk_config;
    case A_CACHE_RESET_CONFIG:
        return s->cache_reset_config;
    case A_CRYPTO_CTRL:
        return s->crypto_ctrl;
    case A_L2_MEM_INT_RAW:
        return s->l2_mem_int_raw;
    case A_L2_MEM_INT_ST:
        return s->l2_mem_int_raw & s->l2_mem_int_ena;
    case A_L2_MEM_INT_ENA:
        return s->l2_mem_int_ena;
    case A_L2_MEM_INT_CLR:
        return 0;
    case A_L2_MEM_INT_RECORD0:
        return s->l2_mem_int_record0;
    case A_L2_MEM_ERR_RESP_CTRL:
        return s->l2_mem_err_resp_ctrl;
    case A_GMAC_CTRL0:
        return s->gmac_ctrl0 | s->gmac_status;
    case A_GMAC_CTRL1:
        return esp32p4_hp_sys_gmac_time(s);
    case A_GMAC_CTRL2:
        return esp32p4_hp_sys_gmac_time(s) >> 32;
    case A_USB20OTG_MEM_CTRL:
        return s->usb20otg_mem_ctrl;
    case A_SYS_USBOTG20_CTRL:
        return s->sys_usbotg20_ctrl;
    case A_USB_OTGHS_PHY_ST:
        return s->usb_otghs_phy_st << 1;
    default:
        *known = false;
        return 0;
    }
}

static uint64_t esp32p4_hp_sys_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4HPSysState *s = opaque;
    bool known;
    uint32_t v = esp32p4_hp_sys_reg(s, addr, &known);

    if (!known) {
        qemu_log_mask(LOG_UNIMP, "%s: unimplemented read of %u bytes at "
                      "0x%" HWADDR_PRIx "\n", TYPE_ESP32P4_HP_SYS, size, addr);
        return 0;
    }
    return extract32(v, (addr & 3) * 8, size * 8);
}

/* [spec:nuos:req:emu.esp32p4.emac] */
static void esp32p4_hp_sys_update_gmac(ESP32P4HPSysState *s)
{
    for (int i = 0; i < ESP32P4_HP_SYS_GMAC_PHY_INTF_LINES; i++) {
        qemu_set_irq(s->gmac_phy_intf[i],
                     extract32(s->gmac_ctrl0,
                               GMAC_CTRL0_PHY_INTF_SHIFT + i, 1));
    }
    qemu_set_irq(s->gmac_sbd_flowctrl,
                 !!(s->gmac_ctrl0 & GMAC_CTRL0_SBD_FLOWCTRL));
}

static void esp32p4_hp_sys_set_gmac_status(void *opaque, int n, int level)
{
    static const uint32_t bit[ESP32P4_HP_SYS_GMAC_STATUS_LINES] = {
        [ESP32P4_HP_SYS_GMAC_PTP_PPS] = GMAC_CTRL0_PTP_PPS,
        [ESP32P4_HP_SYS_GMAC_RST_CLK_TX_N] = GMAC_CTRL0_RST_CLK_TX_N,
        [ESP32P4_HP_SYS_GMAC_RST_CLK_RX_N] = GMAC_CTRL0_RST_CLK_RX_N,
    };
    ESP32P4HPSysState *s = opaque;

    if (level) {
        s->gmac_status |= bit[n];
    } else {
        s->gmac_status &= ~bit[n];
    }
}

static void esp32p4_hp_sys_update_crypto(ESP32P4HPSysState *s)
{
    for (int i = 0; i < ESP32P4_HP_SYS_CRYPTO_CTRL_LINES; i++) {
        qemu_set_irq(s->crypto_ctrl_out[i], extract32(s->crypto_ctrl, i, 1));
    }
}

/* [spec:nuos:req:emu.esp32p4.usb] */
static void esp32p4_hp_sys_update_usb(ESP32P4HPSysState *s)
{
    for (int i = 0; i < ESP32P4_HP_SYS_USBOTG20_CTRL_LINES; i++) {
        qemu_set_irq(s->usbotg20_ctrl_out[i],
                     extract32(s->sys_usbotg20_ctrl,
                               USBOTG20_CTRL_PHY_SHIFT + i, 1));
    }
}

static void esp32p4_hp_sys_phy_st_in(void *opaque, int n, int level)
{
    ESP32P4HPSysState *s = opaque;

    s->usb_otghs_phy_st = deposit32(s->usb_otghs_phy_st, n, 1, level);
}

/* L1_I0_CACHE_RESET (bit 5), L1_I1_CACHE_RESET (4), L1_D_CACHE_RESET (1). */
static void esp32p4_hp_sys_update_cache_reset(ESP32P4HPSysState *s)
{
    static const int bit[ESP32P4_HP_SYS_CACHE_RESET_LINES] = { 5, 4, 1 };

    for (int i = 0; i < ESP32P4_HP_SYS_CACHE_RESET_LINES; i++) {
        qemu_set_irq(s->cache_reset[i],
                     extract32(s->cache_reset_config, bit[i], 1));
    }
}

/* [spec:nuos:req:emu.esp32p4.interrupt-matrix] */
/* [spec:nuos:req:emu.esp32p4.memory] */
static void esp32p4_hp_sys_write(void *opaque, hwaddr addr, uint64_t value,
                                 unsigned size)
{
    ESP32P4HPSysState *s = opaque;
    unsigned shift = (addr & 3) * 8;
    uint32_t mask = MAKE_64BIT_MASK(shift, size * 8);
    uint32_t v = value << shift;
    int n = (addr - A_CPU_INT_FROM_CPU_0) / 4;
    uint32_t *reg;

    if (esp32p4_hp_sys_is_cpu_intr(addr)) {
        reg = &s->cpu_int_from_cpu[n];
        *reg = deposit32(*reg, shift, size * 8, value) & CPU_INT_FROM_CPU;
        qemu_set_irq(s->cpu_intr[n], *reg);
        return;
    }
    switch (addr & ~3) {
    case A_CACHE_CLK_CONFIG:
        s->cache_clk_config = ((s->cache_clk_config & ~mask) | (v & mask)) &
                              CACHE_CLK_CONFIG_MASK;
        if (s->cache_clk_config != CACHE_CLK_CONFIG_MASK) {
            qemu_log_mask(LOG_UNIMP, "%s: stopping a cache's clock "
                          "(CACHE_CLK_CONFIG 0x%02" PRIx32 ") is not "
                          "modelled; the caches keep working\n",
                          TYPE_ESP32P4_HP_SYS, s->cache_clk_config);
        }
        break;
    case A_CACHE_RESET_CONFIG:
        s->cache_reset_config = ((s->cache_reset_config & ~mask) |
                                 (v & mask)) & CACHE_RESET_CONFIG_MASK;
        esp32p4_hp_sys_update_cache_reset(s);
        break;
    case A_CRYPTO_CTRL:
        s->crypto_ctrl = ((s->crypto_ctrl & ~mask) | (v & mask)) &
                         CRYPTO_CTRL_MASK;
        esp32p4_hp_sys_update_crypto(s);
        break;
    case A_L2_MEM_INT_RAW:
        s->l2_mem_int_raw &= ~(v & mask & L2_MEM_INT_MASK);
        break;
    case A_L2_MEM_INT_ENA:
        s->l2_mem_int_ena = ((s->l2_mem_int_ena & ~mask) | (v & mask)) &
                            L2_MEM_INT_MASK;
        break;
    case A_L2_MEM_INT_CLR:
        s->l2_mem_int_raw &= ~(v & mask & L2_MEM_INT_MASK);
        break;
    case A_L2_MEM_INT_ST:
    case A_L2_MEM_INT_RECORD0:
    case A_USB_OTGHS_PHY_ST:
        break;
    case A_USB20OTG_MEM_CTRL:
        s->usb20otg_mem_ctrl = ((s->usb20otg_mem_ctrl & ~mask) |
                                (v & mask)) & USB20OTG_MEM_CTRL_MASK;
        break;
    case A_SYS_USBOTG20_CTRL:
        s->sys_usbotg20_ctrl = ((s->sys_usbotg20_ctrl & ~mask) |
                                (v & mask)) & USBOTG20_CTRL_MASK;
        if (s->sys_usbotg20_ctrl & USBOTG20_CTRL_BISTEN) {
            qemu_log_mask(LOG_UNIMP, "%s: the USB 2.0 OTG PHY's BIST "
                          "(SYS_OTG_PHY_BISTEN) is not modelled\n",
                          TYPE_ESP32P4_HP_SYS);
        }
        esp32p4_hp_sys_update_usb(s);
        break;
    case A_L2_MEM_ERR_RESP_CTRL:
        s->l2_mem_err_resp_ctrl = ((s->l2_mem_err_resp_ctrl & ~mask) |
                                   (v & mask)) & L2_MEM_ERR_RESP_EN;
        qemu_set_irq(s->l2mem_err_resp, s->l2_mem_err_resp_ctrl);
        break;
    case A_GMAC_CTRL0: {
        unsigned intf;

        s->gmac_ctrl0 = ((s->gmac_ctrl0 & ~mask) | (v & mask)) &
                        GMAC_CTRL0_RW;
        intf = (s->gmac_ctrl0 & GMAC_CTRL0_PHY_INTF) >>
               GMAC_CTRL0_PHY_INTF_SHIFT;
        if (intf != 0 && intf != 4) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: PHY_INTF_SEL %u is neither "
                          "MII (0) nor RMII (4)\n", TYPE_ESP32P4_HP_SYS,
                          intf);
        }
        esp32p4_hp_sys_update_gmac(s);
        break;
    }
    case A_GMAC_CTRL1:
    case A_GMAC_CTRL2:
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "%s: unimplemented write of %u bytes at "
                      "0x%" HWADDR_PRIx " (value 0x%" PRIx64 ")\n",
                      TYPE_ESP32P4_HP_SYS, size, addr, value);
        return;
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

static void esp32p4_hp_sys_reset_hold(Object *obj, ResetType type)
{
    ESP32P4HPSysState *s = ESP32P4_HP_SYS(obj);

    memset(s->cpu_int_from_cpu, 0, sizeof(s->cpu_int_from_cpu));
    s->cache_clk_config = CACHE_CLK_CONFIG_MASK;
    s->cache_reset_config = 0;
    s->crypto_ctrl = 0;
    s->l2_mem_int_raw = 0;
    s->l2_mem_int_ena = 0;
    s->l2_mem_int_record0 = 0;
    s->l2_mem_err_resp_ctrl = 0;
    s->gmac_ctrl0 = 0;
    s->usb20otg_mem_ctrl = 0;
    s->sys_usbotg20_ctrl = USBOTG20_CTRL_RESET;
}

static void esp32p4_hp_sys_reset_exit(Object *obj, ResetType type)
{
    ESP32P4HPSysState *s = ESP32P4_HP_SYS(obj);

    for (int i = 0; i < ESP32P4_HP_SYS_CPU_INTRS; i++) {
        qemu_set_irq(s->cpu_intr[i], s->cpu_int_from_cpu[i]);
    }
    esp32p4_hp_sys_update_cache_reset(s);
    esp32p4_hp_sys_update_crypto(s);
    qemu_set_irq(s->l2mem_err_resp, s->l2_mem_err_resp_ctrl);
    esp32p4_hp_sys_update_gmac(s);
    esp32p4_hp_sys_update_usb(s);
    esp32p4_hp_sys_update_irq(s);
}

static void esp32p4_hp_sys_init(Object *obj)
{
    ESP32P4HPSysState *s = ESP32P4_HP_SYS(obj);

    memory_region_init_io(&s->iomem, obj, &esp32p4_hp_sys_ops, s,
                          "esp32p4.hp-sys", ESP32P4_HP_SYS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->sysreg_irq);
    qdev_init_gpio_out_named(DEVICE(obj), s->cpu_intr,
                             ESP32P4_HP_SYS_CPU_INTR,
                             ESP32P4_HP_SYS_CPU_INTRS);
    qdev_init_gpio_out_named(DEVICE(obj), s->cache_reset,
                             ESP32P4_HP_SYS_CACHE_RESET,
                             ESP32P4_HP_SYS_CACHE_RESET_LINES);
    qdev_init_gpio_out_named(DEVICE(obj), s->crypto_ctrl_out,
                             ESP32P4_HP_SYS_CRYPTO_CTRL,
                             ESP32P4_HP_SYS_CRYPTO_CTRL_LINES);
    qdev_init_gpio_out_named(DEVICE(obj), &s->l2mem_err_resp,
                             ESP32P4_HP_SYS_L2MEM_ERR_RESP, 1);
    qdev_init_gpio_out_named(DEVICE(obj), s->gmac_phy_intf,
                             ESP32P4_HP_SYS_GMAC_PHY_INTF,
                             ESP32P4_HP_SYS_GMAC_PHY_INTF_LINES);
    qdev_init_gpio_out_named(DEVICE(obj), &s->gmac_sbd_flowctrl,
                             ESP32P4_HP_SYS_GMAC_SBD_FLOWCTRL, 1);
    qdev_init_gpio_in_named(DEVICE(obj), esp32p4_hp_sys_set_gmac_status,
                            ESP32P4_HP_SYS_GMAC_STATUS,
                            ESP32P4_HP_SYS_GMAC_STATUS_LINES);
    qdev_init_gpio_out_named(DEVICE(obj), s->usbotg20_ctrl_out,
                             ESP32P4_HP_SYS_USBOTG20_CTRL,
                             ESP32P4_HP_SYS_USBOTG20_CTRL_LINES);
    qdev_init_gpio_in_named(DEVICE(obj), esp32p4_hp_sys_phy_st_in,
                            ESP32P4_HP_SYS_USB_OTGHS_PHY_ST, 4);
}

static const VMStateDescription vmstate_esp32p4_hp_sys_usb = {
    .name = TYPE_ESP32P4_HP_SYS "/usb",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(usb20otg_mem_ctrl, ESP32P4HPSysState),
        VMSTATE_UINT32(sys_usbotg20_ctrl, ESP32P4HPSysState),
        VMSTATE_UINT32(usb_otghs_phy_st, ESP32P4HPSysState),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_esp32p4_hp_sys = {
    .name = TYPE_ESP32P4_HP_SYS,
    .version_id = 3,
    .minimum_version_id = 3,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(cpu_int_from_cpu, ESP32P4HPSysState,
                             ESP32P4_HP_SYS_CPU_INTRS),
        VMSTATE_UINT32(cache_clk_config, ESP32P4HPSysState),
        VMSTATE_UINT32(cache_reset_config, ESP32P4HPSysState),
        VMSTATE_UINT32(crypto_ctrl, ESP32P4HPSysState),
        VMSTATE_UINT32(l2_mem_int_raw, ESP32P4HPSysState),
        VMSTATE_UINT32(l2_mem_int_ena, ESP32P4HPSysState),
        VMSTATE_UINT32(l2_mem_int_record0, ESP32P4HPSysState),
        VMSTATE_UINT32(l2_mem_err_resp_ctrl, ESP32P4HPSysState),
        VMSTATE_UINT32(gmac_ctrl0, ESP32P4HPSysState),
        VMSTATE_UINT32(gmac_status, ESP32P4HPSysState),
        VMSTATE_END_OF_LIST()
    },
    .subsections = (const VMStateDescription * const []) {
        &vmstate_esp32p4_hp_sys_usb,
        NULL
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
