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
#define A_L2_MEM_ERR_RESP_CTRL  0x198
#define L2_MEM_ERR_RESP_EN      BIT(0)

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

static uint32_t esp32p4_hp_sys_reg(ESP32P4HPSysState *s, hwaddr addr,
                                   bool *known)
{
    *known = true;
    if (esp32p4_hp_sys_is_cpu_intr(addr)) {
        return s->cpu_int_from_cpu[(addr - A_CPU_INT_FROM_CPU_0) / 4];
    }
    switch (addr & ~3) {
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

static void esp32p4_hp_sys_update_crypto(ESP32P4HPSysState *s)
{
    for (int i = 0; i < ESP32P4_HP_SYS_CRYPTO_CTRL_LINES; i++) {
        qemu_set_irq(s->crypto_ctrl_out[i], extract32(s->crypto_ctrl, i, 1));
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
        break;
    case A_L2_MEM_ERR_RESP_CTRL:
        s->l2_mem_err_resp_ctrl = ((s->l2_mem_err_resp_ctrl & ~mask) |
                                   (v & mask)) & L2_MEM_ERR_RESP_EN;
        qemu_set_irq(s->l2mem_err_resp, s->l2_mem_err_resp_ctrl);
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
    s->crypto_ctrl = 0;
    s->l2_mem_int_raw = 0;
    s->l2_mem_int_ena = 0;
    s->l2_mem_int_record0 = 0;
    s->l2_mem_err_resp_ctrl = 0;
}

static void esp32p4_hp_sys_reset_exit(Object *obj, ResetType type)
{
    ESP32P4HPSysState *s = ESP32P4_HP_SYS(obj);

    for (int i = 0; i < ESP32P4_HP_SYS_CPU_INTRS; i++) {
        qemu_set_irq(s->cpu_intr[i], s->cpu_int_from_cpu[i]);
    }
    esp32p4_hp_sys_update_crypto(s);
    qemu_set_irq(s->l2mem_err_resp, s->l2_mem_err_resp_ctrl);
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
    qdev_init_gpio_out_named(DEVICE(obj), s->crypto_ctrl_out,
                             ESP32P4_HP_SYS_CRYPTO_CTRL,
                             ESP32P4_HP_SYS_CRYPTO_CTRL_LINES);
    qdev_init_gpio_out_named(DEVICE(obj), &s->l2mem_err_resp,
                             ESP32P4_HP_SYS_L2MEM_ERR_RESP, 1);
}

static const VMStateDescription vmstate_esp32p4_hp_sys = {
    .name = TYPE_ESP32P4_HP_SYS,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(cpu_int_from_cpu, ESP32P4HPSysState,
                             ESP32P4_HP_SYS_CPU_INTRS),
        VMSTATE_UINT32(crypto_ctrl, ESP32P4HPSysState),
        VMSTATE_UINT32(l2_mem_int_raw, ESP32P4HPSysState),
        VMSTATE_UINT32(l2_mem_int_ena, ESP32P4HPSysState),
        VMSTATE_UINT32(l2_mem_int_record0, ESP32P4HPSysState),
        VMSTATE_UINT32(l2_mem_err_resp_ctrl, ESP32P4HPSysState),
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
