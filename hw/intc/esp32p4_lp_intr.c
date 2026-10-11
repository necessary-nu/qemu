/*
 * ESP32-P4 LP interrupt controller (LP_INTR)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), 5.4.1
 * (the LP CPU's interrupts, table 5.4-1), and ESP-IDF's lp_intr_reg.h for
 * v3, the authority for the registers, their reset values and access
 * types. The TRM calls the block the LP interrupt matrix
 * (LPPERI_CK_EN_LP_INTR, LPPERI_RST_EN_LP_INTR).
 *
 * The LP core's interrupt controller: it takes the LP peripherals'
 * interrupt sources and HP_INTR to the LP core's interrupt IDs, by TRM
 * table 5.4-1:
 *
 *   ID   source                             ID   source
 *    3   LP_SW_INTR                         24   LP_ANAPERI_INTR
 *    7   LP_UART_INTR                       25   PMU_REG_0_INTR, PMU_REG_1_INTR
 *   11   LP_SPI_INTR                        26   MB_HP_INTR, MB_LP_INTR
 *   17   LP_I2C_INTR                        27   LP_TIMER_REG_0_INTR, _1_INTR
 *   18   LP_GPIO_INTR                       28   LP_WDT_INTR
 *   19   LP_ADC_INTR                        29   LP_RTC_INTR
 *   20   LP_TOUCH_INTR                      30   HP_INTR
 *   21   LP_TSENS_INTR
 *   22   LP_EFUSE_INTR
 *   23   LP_SYSREG_INTR
 *
 * An ID with two sources is their OR. ID 16, which the TRM counts among
 * the 18 but gives no source, is LP TRNG's in ESP-IDF's vector table; the
 * TRNG is not modelled and the line stays low. LP_I2S_INTR and
 * LP_HUK_INTR reach no ID. The routing is combinational and the levels
 * pass while the block's register clock is gated; while the PMU's sleep
 * process masks the core's interrupts, every ID is low.
 *
 * The registers:
 *
 *   - LP_SW_INT_RAW, _ST, _ENA and _CLR (bit 31): the software interrupt,
 *     whose raw bit software writes, and clears also by writing 1 to the
 *     clear bit; the raw bit enabled is LP_SW_INTR, which is also interrupt
 *     matrix source 18.
 *   - LPINTR_STATUS (read-only): the sources' levels, bit 30 - n for source
 *     n (LP_RTC_INTR, source 0, at bit 30, down to LP_HUK_INTR at bit 10),
 *     LP_SW_INTR at bit 12 and HP_INTR at bit 31.
 *   - DATE's CLK_EN, which forces the register clock on; the registers
 *     behave the same either way.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "hw/core/irq.h"
#include "hw/intc/esp32p4_intmatrix.h"
#include "hw/intc/esp32p4_lp_intr.h"
#include "migration/vmstate.h"

#define A_SW_INT_RAW        0x000
#define A_SW_INT_ST         0x004
#define A_SW_INT_ENA        0x008
#define A_SW_INT_CLR        0x00c
#define A_STATUS            0x010
#define A_DATE              0x3fc

#define SW_INT              BIT(31)
#define CLK_EN              BIT(31)
#define STATUS_HP_INTR      BIT(31)

#define LP_INTR_HP          (-1)

/* Each interrupt ID's sources, or LP_INTR_HP; -2 ends a list. */
static const struct {
    int id;
    int src[2];
} esp32p4_lp_intr_map[] = {
    { 3,  { ESP32P4_INTR_LP_SW, -2 } },
    { 7,  { ESP32P4_INTR_LP_UART, -2 } },
    { 11, { ESP32P4_INTR_LP_SPI, -2 } },
    { 17, { ESP32P4_INTR_LP_I2C, -2 } },
    { 18, { ESP32P4_INTR_LP_GPIO, -2 } },
    { 19, { ESP32P4_INTR_LP_ADC, -2 } },
    { 20, { ESP32P4_INTR_LP_TOUCH, -2 } },
    { 21, { ESP32P4_INTR_LP_TSENS, -2 } },
    { 22, { ESP32P4_INTR_LP_EFUSE, -2 } },
    { 23, { ESP32P4_INTR_LP_SYSREG, -2 } },
    { 24, { ESP32P4_INTR_LP_ANAPERI, -2 } },
    { 25, { ESP32P4_INTR_PMU_0, ESP32P4_INTR_PMU_1 } },
    { 26, { ESP32P4_INTR_MB_HP, ESP32P4_INTR_MB_LP } },
    { 27, { ESP32P4_INTR_LP_TIMER_REG_0, ESP32P4_INTR_LP_TIMER_REG_1 } },
    { 28, { ESP32P4_INTR_LP_WDT, -2 } },
    { 29, { ESP32P4_INTR_LP_RTC, -2 } },
    { 30, { LP_INTR_HP, -2 } },
};

/* The sources' levels with the block's own LP_SW_INTR among them. */
static uint32_t esp32p4_lp_intr_levels(ESP32P4LPIntrState *s)
{
    uint32_t v = s->sources & ~BIT(ESP32P4_INTR_LP_SW);

    if (s->sw_raw & s->sw_ena) {
        v |= BIT(ESP32P4_INTR_LP_SW);
    }
    return v;
}

/* [spec:nuos:req:emu.esp32p4.lp-core] */
static uint32_t esp32p4_lp_intr_status(ESP32P4LPIntrState *s)
{
    uint32_t levels = esp32p4_lp_intr_levels(s);
    uint32_t v = s->hp_intr ? STATUS_HP_INTR : 0;

    for (int n = 0; n < ESP32P4_LP_INTR_SOURCES; n++) {
        if (levels & BIT(n)) {
            v |= BIT(30 - n);
        }
    }
    return v;
}

/* [spec:nuos:req:emu.esp32p4.lp-core] */
static void esp32p4_lp_intr_update(ESP32P4LPIntrState *s)
{
    uint32_t levels = esp32p4_lp_intr_levels(s);
    uint32_t ids = 0;
    uint32_t changed;

    qemu_set_irq(s->sw_irq, (s->sw_raw & s->sw_ena) != 0);
    for (int i = 0; i < ARRAY_SIZE(esp32p4_lp_intr_map); i++) {
        for (int k = 0; k < 2 && esp32p4_lp_intr_map[i].src[k] != -2; k++) {
            int src = esp32p4_lp_intr_map[i].src[k];

            if (src == LP_INTR_HP ? s->hp_intr : (levels & BIT(src))) {
                ids |= BIT(esp32p4_lp_intr_map[i].id);
            }
        }
    }
    if (s->masked) {
        ids = 0;
    }
    changed = ids ^ s->cpu_level;
    s->cpu_level = ids;
    while (changed) {
        int id = ctz32(changed);

        changed &= changed - 1;
        qemu_set_irq(s->cpu_irq[id], extract32(ids, id, 1));
    }
}

static uint64_t esp32p4_lp_intr_read(void *opaque, hwaddr addr,
                                     unsigned size)
{
    ESP32P4LPIntrState *s = opaque;
    uint32_t v;

    switch (addr & ~3) {
    case A_SW_INT_RAW:
        v = s->sw_raw;
        break;
    case A_SW_INT_ST:
        v = s->sw_raw & s->sw_ena;
        break;
    case A_SW_INT_ENA:
        v = s->sw_ena;
        break;
    case A_SW_INT_CLR:
        /* Write-only. */
        v = 0;
        break;
    case A_STATUS:
        v = esp32p4_lp_intr_status(s);
        break;
    case A_DATE:
        v = s->clk_en;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of %u bytes at reserved "
                      "offset 0x%" HWADDR_PRIx "\n", TYPE_ESP32P4_LP_INTR,
                      size, addr);
        return 0;
    }
    return extract32(v, (addr & 3) * 8, size * 8);
}

/* [spec:nuos:req:emu.esp32p4.lp-core] */
static void esp32p4_lp_intr_write(void *opaque, hwaddr addr, uint64_t value,
                                  unsigned size)
{
    ESP32P4LPIntrState *s = opaque;
    unsigned shift = (addr & 3) * 8;
    uint32_t lanes = MAKE_64BIT_MASK(shift, size * 8);
    uint32_t v = ((uint32_t)value << shift) & lanes;

    switch (addr & ~3) {
    case A_SW_INT_RAW:
        s->sw_raw = (s->sw_raw & ~(lanes & SW_INT)) | (v & SW_INT);
        break;
    case A_SW_INT_ENA:
        s->sw_ena = (s->sw_ena & ~(lanes & SW_INT)) | (v & SW_INT);
        break;
    case A_SW_INT_CLR:
        s->sw_raw &= ~(v & SW_INT);
        break;
    case A_DATE:
        s->clk_en = (s->clk_en & ~(lanes & CLK_EN)) | (v & CLK_EN);
        return;
    case A_SW_INT_ST:
    case A_STATUS:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_LP_INTR, addr);
        return;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write of %u bytes at reserved "
                      "offset 0x%" HWADDR_PRIx " (value 0x%" PRIx64 ")\n",
                      TYPE_ESP32P4_LP_INTR, size, addr, value);
        return;
    }
    esp32p4_lp_intr_update(s);
}

static const MemoryRegionOps esp32p4_lp_intr_ops = {
    .read = esp32p4_lp_intr_read,
    .write = esp32p4_lp_intr_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

static void esp32p4_lp_intr_set_source(void *opaque, int n, int level)
{
    ESP32P4LPIntrState *s = opaque;

    s->sources = deposit32(s->sources, n, 1, level != 0);
    esp32p4_lp_intr_update(s);
}

static void esp32p4_lp_intr_set_hp(void *opaque, int n, int level)
{
    ESP32P4LPIntrState *s = opaque;

    s->hp_intr = level;
    esp32p4_lp_intr_update(s);
}

static void esp32p4_lp_intr_set_mask(void *opaque, int n, int level)
{
    ESP32P4LPIntrState *s = opaque;

    s->masked = level;
    esp32p4_lp_intr_update(s);
}

/*
 * A reset clears the registers; the inputs keep their levels, which their
 * sources drive.
 */
static void esp32p4_lp_intr_reset_hold(Object *obj, ResetType type)
{
    ESP32P4LPIntrState *s = ESP32P4_LP_INTR(obj);

    s->sw_raw = 0;
    s->sw_ena = 0;
    s->clk_en = 0;
}

static void esp32p4_lp_intr_reset_exit(Object *obj, ResetType type)
{
    esp32p4_lp_intr_update(ESP32P4_LP_INTR(obj));
}

static void esp32p4_lp_intr_init(Object *obj)
{
    ESP32P4LPIntrState *s = ESP32P4_LP_INTR(obj);
    DeviceState *dev = DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32p4_lp_intr_ops, s,
                          "esp32p4.lp-intr", ESP32P4_LP_INTR_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->sw_irq);
    qdev_init_gpio_in_named(dev, esp32p4_lp_intr_set_source,
                            ESP32P4_LP_INTR_SOURCE, ESP32P4_LP_INTR_SOURCES);
    qdev_init_gpio_in_named(dev, esp32p4_lp_intr_set_hp, ESP32P4_LP_INTR_HP,
                            1);
    qdev_init_gpio_in_named(dev, esp32p4_lp_intr_set_mask,
                            ESP32P4_LP_INTR_MASK, 1);
    qdev_init_gpio_out_named(dev, s->cpu_irq, ESP32P4_LP_INTR_CPU_IRQ,
                             ESP32P4_LP_INTR_CPU_IRQS);
}

static const VMStateDescription vmstate_esp32p4_lp_intr = {
    .name = TYPE_ESP32P4_LP_INTR,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(sources, ESP32P4LPIntrState),
        VMSTATE_BOOL(hp_intr, ESP32P4LPIntrState),
        VMSTATE_BOOL(masked, ESP32P4LPIntrState),
        VMSTATE_UINT32(sw_raw, ESP32P4LPIntrState),
        VMSTATE_UINT32(sw_ena, ESP32P4LPIntrState),
        VMSTATE_UINT32(clk_en, ESP32P4LPIntrState),
        VMSTATE_UINT32(cpu_level, ESP32P4LPIntrState),
        VMSTATE_END_OF_LIST()
    },
};

static void esp32p4_lp_intr_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32p4_lp_intr_reset_hold;
    rc->phases.exit = esp32p4_lp_intr_reset_exit;
    dc->vmsd = &vmstate_esp32p4_lp_intr;
}

/* [spec:nuos:req:emu.esp32p4.lp-core] */
static const TypeInfo esp32p4_lp_intr_types[] = {
    {
        .name          = TYPE_ESP32P4_LP_INTR,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(ESP32P4LPIntrState),
        .instance_init = esp32p4_lp_intr_init,
        .class_init    = esp32p4_lp_intr_class_init,
    },
};

DEFINE_TYPES(esp32p4_lp_intr_types)
