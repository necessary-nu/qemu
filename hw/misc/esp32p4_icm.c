/*
 * ESP32-P4 AXI interconnect (ICM) registers
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), 21.2.1.14
 * (Illegal Access and Unauthorized Access) and 21.4.2 (ICM Register
 * Summary), and 20.5 (Illegal Access and Interrupts); ESP-IDF's
 * icm_sys_reg.h for v3.
 *
 * The HP buses record the access they refuse, one the HP or DMA APM does
 * not permit (SECURE 0) or one to an address nothing decodes (SECURE 1):
 * its address, its master's ID and whether it was a write. A refusal of
 * an access by the CPUs or an AHB master (the HP APM's, and the DMA APM's
 * for the AHB DMA masters) goes to ICM_CPU_ADDRHOLE_ADDR/INFO and raises
 * ICM_CPU_ADDRHOLE_INT; one by an AXI master (VDMA, GDMA-AXI, 2D-DMA, the
 * H264 DMA) to ICM_SYS_ADDRHOLE_ADDR/INFO and ICM_SYS_ADDRHOLE_INT. The
 * record is taken as the interrupt's raw bit rises and holds until
 * software clears the bit, so it is the first refusal since. The three
 * raw bits, as INT_ENA enables them, drive the ICM's interrupt,
 * SYS_ICM_INTR, source 21 of the interrupt matrix. The APMs record their
 * refusals here, and the bus matrix a DMA master's access to an address
 * its port has no path to (hw/misc/esp32p4_pms.c); a core's access to an
 * address nothing decodes answers with a bus error but is not recorded.
 *
 * The other registers, the arbitration priorities and QoS levels and the
 * deadlock timeout, are kept as written: the model has no arbitration and
 * no transaction ever deadlocks, so DLOCK_STATUS and the deadlock
 * interrupt stay clear.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "hw/core/irq.h"
#include "hw/misc/esp32p4_icm.h"
#include "migration/vmstate.h"

#define A_VER_DATE              0x00
#define A_CLK_EN                0x04
#define A_DLOCK_STATUS          0x08
#define A_INT_RAW               0x0c
#define A_INT_ST                0x10
#define A_INT_ENA               0x14
#define A_INT_CLR               0x18
#define A_MST_ARB_PRIORITY_REG0 0x1c
#define A_SLV_ARB_PRIORITY      0x24
#define A_MST_ARQOS_REG0        0x28
#define A_MST_AWQOS_REG0        0x30
#define A_SYS_ADDRHOLE_ADDR     0x38
#define A_SYS_ADDRHOLE_INFO     0x3c
#define A_CPU_ADDRHOLE_ADDR     0x40
#define A_CPU_ADDRHOLE_INFO     0x44
#define A_DLOCK_TIMEOUT         0x48
#define A_RDN_ECO_CS            0x50
#define A_RDN_ECO_LOW           0x54
#define A_RDN_ECO_HIGH          0x58

#define INT_DLOCK               BIT(0)
#define INT_SYS_ADDRHOLE        BIT(1)
#define INT_CPU_ADDRHOLE        BIT(2)
#define INT_ALL                 (INT_DLOCK | INT_SYS_ADDRHOLE | \
                                 INT_CPU_ADDRHOLE)

/* ICM_*_ADDRHOLE_INFO: the ID in the low bits, then WR and SECURE. */
#define ADDRHOLE_SYS_ID_MASK    0xff
#define ADDRHOLE_CPU_ID_MASK    0x1f
#define ADDRHOLE_WR             BIT(8)
#define ADDRHOLE_SECURE         BIT(9)

#define VER_DATE_RESET          0x20230214
#define SLV_ARB_PRIORITY_MASK   0x00fff038
#define DLOCK_TIMEOUT_MASK      0x1fff
#define DLOCK_TIMEOUT_RESET     2048
#define RDN_ECO_EN              BIT(0)

static void esp32p4_icm_update(ESP32P4IcmState *s)
{
    qemu_set_irq(s->irq, !!(s->int_raw & s->int_ena));
}

/* [spec:nuos:req:emu.esp32p4.pms] */
void esp32p4_icm_addrhole(ESP32P4IcmState *s, bool sys, uint32_t addr,
                          uint8_t id, bool write, bool illegal)
{
    uint32_t bit = sys ? INT_SYS_ADDRHOLE : INT_CPU_ADDRHOLE;
    uint32_t info = (id & (sys ? ADDRHOLE_SYS_ID_MASK
                               : ADDRHOLE_CPU_ID_MASK)) |
                    (write ? ADDRHOLE_WR : 0) |
                    (illegal ? ADDRHOLE_SECURE : 0);

    if (!(s->int_raw & bit)) {
        if (sys) {
            s->sys_addrhole_addr = addr;
            s->sys_addrhole_info = info;
        } else {
            s->cpu_addrhole_addr = addr;
            s->cpu_addrhole_info = info;
        }
        s->int_raw |= bit;
    }
    esp32p4_icm_update(s);
}

static uint32_t esp32p4_icm_reg(ESP32P4IcmState *s, hwaddr addr, bool *ok)
{
    *ok = true;
    switch (addr) {
    case A_VER_DATE:
        return s->ver_date;
    case A_CLK_EN:
        return s->clk_en;
    case A_DLOCK_STATUS:
        return 0;
    case A_INT_RAW:
        return s->int_raw;
    case A_INT_ST:
        return s->int_raw & s->int_ena;
    case A_INT_ENA:
        return s->int_ena;
    case A_INT_CLR:
        return 0;
    case A_MST_ARB_PRIORITY_REG0:
        return s->mst_arb_priority;
    case A_SLV_ARB_PRIORITY:
        return s->slv_arb_priority;
    case A_MST_ARQOS_REG0:
        return s->mst_arqos;
    case A_MST_AWQOS_REG0:
        return s->mst_awqos;
    case A_SYS_ADDRHOLE_ADDR:
        return s->sys_addrhole_addr;
    case A_SYS_ADDRHOLE_INFO:
        return s->sys_addrhole_info;
    case A_CPU_ADDRHOLE_ADDR:
        return s->cpu_addrhole_addr;
    case A_CPU_ADDRHOLE_INFO:
        return s->cpu_addrhole_info;
    case A_DLOCK_TIMEOUT:
        return s->dlock_timeout;
    case A_RDN_ECO_CS:
        return s->rdn_eco_cs;
    case A_RDN_ECO_LOW:
        return s->rdn_eco_low;
    case A_RDN_ECO_HIGH:
        return s->rdn_eco_high;
    default:
        *ok = false;
        return 0;
    }
}

static uint64_t esp32p4_icm_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4IcmState *s = opaque;
    bool ok;
    uint32_t v = esp32p4_icm_reg(s, addr & ~3, &ok);

    if (!ok) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of reserved offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_ICM, addr);
    }
    return extract32(v, (addr & 3) * 8, size * 8);
}

/* [spec:nuos:req:emu.esp32p4.pms] */
static void esp32p4_icm_write(void *opaque, hwaddr addr, uint64_t value,
                              unsigned size)
{
    ESP32P4IcmState *s = opaque;
    hwaddr reg = addr & ~3;
    bool ok;
    uint32_t old = esp32p4_icm_reg(s, reg, &ok);
    uint32_t v = deposit32(reg == A_INT_RAW || reg == A_INT_CLR ? 0 : old,
                           (addr & 3) * 8, size * 8, value);

    switch (reg) {
    case A_VER_DATE:
        s->ver_date = v;
        break;
    case A_CLK_EN:
        s->clk_en = v & 1;
        break;
    case A_INT_RAW:
    case A_INT_CLR:
        /* INT_RAW is write-1-to-clear, as INT_CLR is. */
        s->int_raw &= ~(v & INT_ALL);
        esp32p4_icm_update(s);
        break;
    case A_INT_ENA:
        s->int_ena = v & INT_ALL;
        esp32p4_icm_update(s);
        break;
    case A_MST_ARB_PRIORITY_REG0:
        s->mst_arb_priority = v;
        break;
    case A_SLV_ARB_PRIORITY:
        s->slv_arb_priority = v & SLV_ARB_PRIORITY_MASK;
        break;
    case A_MST_ARQOS_REG0:
        s->mst_arqos = v;
        break;
    case A_MST_AWQOS_REG0:
        s->mst_awqos = v;
        break;
    case A_DLOCK_TIMEOUT:
        s->dlock_timeout = v & DLOCK_TIMEOUT_MASK;
        break;
    case A_RDN_ECO_CS:
        s->rdn_eco_cs = v & RDN_ECO_EN;
        break;
    case A_RDN_ECO_LOW:
        s->rdn_eco_low = v;
        break;
    case A_RDN_ECO_HIGH:
        s->rdn_eco_high = v;
        break;
    case A_DLOCK_STATUS:
    case A_INT_ST:
    case A_SYS_ADDRHOLE_ADDR:
    case A_SYS_ADDRHOLE_INFO:
    case A_CPU_ADDRHOLE_ADDR:
    case A_CPU_ADDRHOLE_INFO:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_ICM, addr);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to reserved offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_ICM, addr);
        break;
    }
}

static const MemoryRegionOps esp32p4_icm_ops = {
    .read = esp32p4_icm_read,
    .write = esp32p4_icm_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

static void esp32p4_icm_reset_hold(Object *obj, ResetType type)
{
    ESP32P4IcmState *s = ESP32P4_ICM(obj);

    s->ver_date = VER_DATE_RESET;
    s->clk_en = 0;
    s->int_raw = 0;
    s->int_ena = INT_ALL;
    s->mst_arb_priority = 0;
    s->slv_arb_priority = 0;
    s->mst_arqos = 0;
    s->mst_awqos = 0;
    s->sys_addrhole_addr = 0;
    s->sys_addrhole_info = 0;
    s->cpu_addrhole_addr = 0;
    s->cpu_addrhole_info = 0;
    s->dlock_timeout = DLOCK_TIMEOUT_RESET;
    s->rdn_eco_cs = 0;
    s->rdn_eco_low = 0;
    s->rdn_eco_high = 0xffffffff;
}

static void esp32p4_icm_reset_exit(Object *obj, ResetType type)
{
    esp32p4_icm_update(ESP32P4_ICM(obj));
}

static void esp32p4_icm_init(Object *obj)
{
    ESP32P4IcmState *s = ESP32P4_ICM(obj);

    memory_region_init_io(&s->iomem, obj, &esp32p4_icm_ops, s,
                          "esp32p4.axi-icm", ESP32P4_ICM_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static const VMStateDescription vmstate_esp32p4_icm = {
    .name = TYPE_ESP32P4_ICM,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(ver_date, ESP32P4IcmState),
        VMSTATE_UINT32(clk_en, ESP32P4IcmState),
        VMSTATE_UINT32(int_raw, ESP32P4IcmState),
        VMSTATE_UINT32(int_ena, ESP32P4IcmState),
        VMSTATE_UINT32(mst_arb_priority, ESP32P4IcmState),
        VMSTATE_UINT32(slv_arb_priority, ESP32P4IcmState),
        VMSTATE_UINT32(mst_arqos, ESP32P4IcmState),
        VMSTATE_UINT32(mst_awqos, ESP32P4IcmState),
        VMSTATE_UINT32(sys_addrhole_addr, ESP32P4IcmState),
        VMSTATE_UINT32(sys_addrhole_info, ESP32P4IcmState),
        VMSTATE_UINT32(cpu_addrhole_addr, ESP32P4IcmState),
        VMSTATE_UINT32(cpu_addrhole_info, ESP32P4IcmState),
        VMSTATE_UINT32(dlock_timeout, ESP32P4IcmState),
        VMSTATE_UINT32(rdn_eco_cs, ESP32P4IcmState),
        VMSTATE_UINT32(rdn_eco_low, ESP32P4IcmState),
        VMSTATE_UINT32(rdn_eco_high, ESP32P4IcmState),
        VMSTATE_END_OF_LIST()
    },
};

static void esp32p4_icm_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32p4_icm_reset_hold;
    rc->phases.exit = esp32p4_icm_reset_exit;
    dc->vmsd = &vmstate_esp32p4_icm;
}

/* [spec:nuos:req:emu.esp32p4.pms] */
static const TypeInfo esp32p4_icm_info = {
    .name          = TYPE_ESP32P4_ICM,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32P4IcmState),
    .instance_init = esp32p4_icm_init,
    .class_init    = esp32p4_icm_class_init,
};

static void esp32p4_icm_register_types(void)
{
    type_register_static(&esp32p4_icm_info);
}
type_init(esp32p4_icm_register_types)
