/*
 * ESP32-P4 ECC accelerator
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapter
 * 28 (ECC Accelerator), and ESP-IDF's ecc_mult_reg.h for v3 (memories of
 * 48 bytes at 0x100, 0x130, ... for P-384, where the TRM still has 32
 * bytes at 0x100, 0x120, ...), ecc_ll.h and ecc_hal.c.
 *
 * On the curve ECC_MULT_KEY_LENGTH selects (P-192, P-256, P-384; 3 is
 * invalid), ECC_MULT_START runs the working mode of ECC_MULT_WORK_MODE on
 * the memories, numbers of the curve's width stored little-endian:
 *
 *   0  Affine point multiplication: (Px, Py) = k (Px, Py).
 *   2  Affine point verification: whether (Px, Py) is on the curve.
 *   3  Verification, then if it passes the multiplication, the result
 *      also in Jacobian coordinates in (Qx, Qy, Qz).
 *   4  Jacobian point multiplication: (Qx, Qy, Qz) = k (Px, Py, 1).
 *   5  Point addition: (Px, Py, 1) + (Qx, Qy, Qz), affine into (Px, Py)
 *      and Jacobian into (Qx, Qy, Qz).
 *   6  Jacobian point verification of (Qx, Qy, Qz).
 *   7  Affine verification, then if it passes the Jacobian multiplication.
 *   8-11  Px + Py, Px - Py (into Px), Px * Py, Px / Py (into Py), modulo
 *      the order n or, with ECC_MULT_MOD_BASE, the prime p.
 *
 * Verifications leave their result in ECC_MULT_VERIFICATION_RESULT. A
 * point at infinity comes out as zeros in affine coordinates and with
 * Z = 0 in Jacobian ones, whose X and Y the model leaves as zeros; the
 * Jacobian coordinates of a result are one of the point's equivalent
 * representations, as software takes them. Mode 1 and modes 12 to 15
 * are reserved: the calculation ends at once and does nothing, which is
 * logged, as is curve mode 3.
 *
 * Timing: a point multiplication takes about 8 field multiplications per
 * bit of k and 12 more per set bit, a field multiplication 2w^2 + 10
 * cycles of CRYPTO_ECC_CLK for a w-word curve (the model's figures);
 * with ECC_MULT_SECURITY_MODE, or EFUSE_ECC_FORCE_CONST_TIME, every point
 * multiplication takes the time of a scalar with every bit set. When done
 * ECC_MULT_START clears and ECC_MULT_CALC_DONE_INT raises (ECC_INTR while
 * enabled). ECC_MULT_RESET abandons a calculation in progress. The
 * memories and ECC_MULT_CONF_REG cannot be written during a calculation
 * (dropped and logged); powered down by HP_SYSTEM_ECC_PD_CTRL_REG the
 * memories lose their contents and read as zero. While the ECDSA_DS
 * peripheral occupies the accelerator its registers read as zero and take
 * no writes, and it resets when released. ECC_MULT_CLK_EN and
 * ECC_MULT_MEM_CLOCK_GATE_FORCE_ON only keep clocks running, and are held.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/esp32p4_ecc.h"
#include "migration/vmstate.h"

#define A_INT_RAW       0x0c
#define A_INT_ST        0x10
#define A_INT_ENA       0x14
#define A_INT_CLR       0x18
#define A_CONF          0x1c
#define A_DATE          0xfc
#define A_MEM           0x100
#define A_MEM_END       (A_MEM + ESP32P4_ECC_MEMS * ESP32P4_ECC_MEM_BYTES)

#define CONF_START          BIT(0)
#define CONF_RESET          BIT(1)
#define CONF_KEY_LENGTH_S   2
#define CONF_MOD_BASE       BIT(4)
#define CONF_WORK_MODE_S    5
#define CONF_SECURITY_MODE  BIT(9)
#define CONF_VERIFY_RESULT  BIT(29)
#define CONF_RW_MASK        (0x3ffu << 2 | BIT(30) | BIT(31))

#define INT_CALC_DONE       BIT(0)
#define DATE_RESET          37781792

enum {
    MODE_POINT_MUL = 0,
    MODE_VERIFY = 2,
    MODE_VERIFY_MUL = 3,
    MODE_JACOBIAN_MUL = 4,
    MODE_POINT_ADD = 5,
    MODE_JACOBIAN_VERIFY = 6,
    MODE_VERIFY_JACOBIAN_MUL = 7,
    MODE_MOD_ADD = 8,
    MODE_MOD_SUB = 9,
    MODE_MOD_MUL = 10,
    MODE_MOD_DIV = 11,
};

static bool ecc_in_reset(ESP32P4EccState *s)
{
    return s->reset_in[0] || s->reset_in[1];
}

static void ecc_update_irq(ESP32P4EccState *s)
{
    qemu_set_irq(s->irq, s->int_raw & s->int_ena & INT_CALC_DONE);
}

static unsigned ecc_curve(ESP32P4EccState *s)
{
    return extract32(s->conf, CONF_KEY_LENGTH_S, 2);
}

static unsigned ecc_mode(ESP32P4EccState *s)
{
    return extract32(s->conf, CONF_WORK_MODE_S, 4);
}

static bool ecc_busy(ESP32P4EccState *s)
{
    return s->conf & CONF_START;
}

/* EFUSE_ECC_FORCE_CONST_TIME, EFUSE_RD_REPEAT_DATA1_REG bit 15. */
static bool ecc_force_const_time(ESP32P4EccState *s)
{
    return s->efuse && (esp32p4_efuse_word(s->efuse, 0, 2) & BIT(15));
}

static void ecc_get(ESP32P4EccState *s, int mem, uint32_t *v, unsigned w)
{
    esp32p4_bn_from_le(v, w, s->mem[mem], 4 * w);
}

static void ecc_put(ESP32P4EccState *s, int mem, const uint32_t *v,
                    unsigned w)
{
    esp32p4_bn_to_le(s->mem[mem], 4 * w, v, w);
}

static uint64_t ecc_cycles(ESP32P4EccState *s)
{
    const ESP32P4Curve *c;
    uint32_t k[ESP32P4_EC_MAX_WORDS];
    bool constant;
    unsigned w, bits, ones = 0, i;
    uint64_t f;

    if (ecc_curve(s) >= ESP32P4_CURVES) {
        return 1;
    }
    c = esp32p4_curve(ecc_curve(s));
    w = esp32p4_curve_words(c);
    f = esp32p4_ec_op_cycles(c);
    constant = (s->conf & CONF_SECURITY_MODE) || ecc_force_const_time(s);
    ecc_get(s, ESP32P4_ECC_MEM_K, k, w);
    bits = esp32p4_bn_bits(k, w);
    for (i = 0; i < w; i++) {
        ones += ctpop32(k[i]);
    }
    switch (ecc_mode(s)) {
    case MODE_POINT_MUL:
    case MODE_JACOBIAN_MUL:
        return esp32p4_ec_mul_cycles(c, bits, ones, constant) + 2 * f;
    case MODE_VERIFY_MUL:
    case MODE_VERIFY_JACOBIAN_MUL:
        return esp32p4_ec_mul_cycles(c, bits, ones, constant) + 8 * f;
    case MODE_VERIFY:
    case MODE_JACOBIAN_VERIFY:
        return 8 * f;
    case MODE_POINT_ADD:
        return 16 * f;
    case MODE_MOD_ADD:
    case MODE_MOD_SUB:
        return w + 4;
    case MODE_MOD_MUL:
        return f;
    case MODE_MOD_DIV:
        return 32ULL * w * f;
    default:
        return 1;
    }
}

/* [spec:nuos:req:emu.esp32p4.crypto] */
static void ecc_calc(ESP32P4EccState *s)
{
    uint32_t k[ESP32P4_EC_MAX_WORDS], px[ESP32P4_EC_MAX_WORDS];
    uint32_t py[ESP32P4_EC_MAX_WORDS], r[ESP32P4_EC_MAX_WORDS];
    uint32_t t[ESP32P4_EC_MAX_WORDS];
    bool order = !(s->conf & CONF_MOD_BASE);
    const ESP32P4Curve *c;
    ESP32P4EcPoint q, res;
    unsigned w, mode = ecc_mode(s);
    bool ok;

    if (ecc_curve(s) >= ESP32P4_CURVES) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: ECC_MULT_KEY_LENGTH 3 is invalid; "
                      "the calculation does nothing\n", TYPE_ESP32P4_ECC);
        return;
    }
    c = esp32p4_curve(ecc_curve(s));
    w = esp32p4_curve_words(c);
    ecc_get(s, ESP32P4_ECC_MEM_K, k, w);
    ecc_get(s, ESP32P4_ECC_MEM_PX, px, w);
    ecc_get(s, ESP32P4_ECC_MEM_PY, py, w);
    memset(&q, 0, sizeof(q));
    ecc_get(s, ESP32P4_ECC_MEM_QX, q.x, w);
    ecc_get(s, ESP32P4_ECC_MEM_QY, q.y, w);
    ecc_get(s, ESP32P4_ECC_MEM_QZ, q.z, w);

    switch (mode) {
    case MODE_VERIFY:
    case MODE_VERIFY_MUL:
    case MODE_VERIFY_JACOBIAN_MUL:
        ok = esp32p4_ec_on_curve(c, px, py);
        s->conf = deposit32(s->conf, 29, 1, ok);
        if (!ok || mode == MODE_VERIFY) {
            break;
        }
        QEMU_FALLTHROUGH;
    case MODE_POINT_MUL:
    case MODE_JACOBIAN_MUL:
        esp32p4_ec_mul(c, &res, k, px, py);
        if (mode == MODE_POINT_MUL || mode == MODE_VERIFY_MUL) {
            esp32p4_ec_affine(c, &res, px, py);
            ecc_put(s, ESP32P4_ECC_MEM_PX, px, w);
            ecc_put(s, ESP32P4_ECC_MEM_PY, py, w);
        }
        if (mode != MODE_POINT_MUL) {
            ecc_put(s, ESP32P4_ECC_MEM_QX, res.x, w);
            ecc_put(s, ESP32P4_ECC_MEM_QY, res.y, w);
            ecc_put(s, ESP32P4_ECC_MEM_QZ, res.z, w);
        }
        break;
    case MODE_POINT_ADD: {
        ESP32P4EcPoint p;

        esp32p4_ec_point_from_affine(c, &p, px, py);
        esp32p4_ec_add(c, &res, &p, &q);
        esp32p4_ec_affine(c, &res, px, py);
        ecc_put(s, ESP32P4_ECC_MEM_PX, px, w);
        ecc_put(s, ESP32P4_ECC_MEM_PY, py, w);
        ecc_put(s, ESP32P4_ECC_MEM_QX, res.x, w);
        ecc_put(s, ESP32P4_ECC_MEM_QY, res.y, w);
        ecc_put(s, ESP32P4_ECC_MEM_QZ, res.z, w);
        break;
    }
    case MODE_JACOBIAN_VERIFY:
        ok = esp32p4_ec_on_curve_jacobian(c, q.x, q.y, q.z);
        s->conf = deposit32(s->conf, 29, 1, ok);
        break;
    case MODE_MOD_ADD:
        esp32p4_ec_mod_add(c, order, r, px, py);
        ecc_put(s, ESP32P4_ECC_MEM_PX, r, w);
        break;
    case MODE_MOD_SUB:
        esp32p4_ec_mod_sub(c, order, r, px, py);
        ecc_put(s, ESP32P4_ECC_MEM_PX, r, w);
        break;
    case MODE_MOD_MUL:
        esp32p4_ec_mod_mul(c, order, r, px, py);
        ecc_put(s, ESP32P4_ECC_MEM_PY, r, w);
        break;
    case MODE_MOD_DIV:
        esp32p4_ec_mod_inv(c, order, t, py);
        esp32p4_ec_mod_mul(c, order, r, px, t);
        ecc_put(s, ESP32P4_ECC_MEM_PY, r, w);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: ECC_MULT_WORK_MODE %u is "
                      "reserved; the calculation does nothing\n",
                      TYPE_ESP32P4_ECC, mode);
        break;
    }
}

/* [spec:nuos:req:emu.esp32p4.crypto] */
static void ecc_op_done(void *opaque)
{
    ESP32P4EccState *s = opaque;

    ecc_calc(s);
    s->conf &= ~CONF_START;
    s->int_raw |= INT_CALC_DONE;
    ecc_update_irq(s);
}

static uint64_t ecc_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4EccState *s = opaque;
    hwaddr reg = addr & ~3;
    uint32_t v = 0;

    if (s->occupied) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read at 0x%" HWADDR_PRIx
                      " while the ECDSA_DS peripheral occupies the "
                      "accelerator\n", TYPE_ESP32P4_ECC, addr);
        return 0;
    }
    if (reg >= A_MEM && reg < A_MEM_END) {
        unsigned off = reg - A_MEM;

        if (s->mem_pd || ecc_busy(s)) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: read of memory at 0x%"
                          HWADDR_PRIx " while it is %s\n", TYPE_ESP32P4_ECC,
                          addr, s->mem_pd ? "powered down" : "in use");
            return 0;
        }
        v = ldl_le_p(&s->mem[off / ESP32P4_ECC_MEM_BYTES]
                            [off % ESP32P4_ECC_MEM_BYTES]);
        return extract32(v, (addr & 3) * 8, size * 8);
    }
    switch (reg) {
    case A_INT_RAW:
        v = s->int_raw;
        break;
    case A_INT_ST:
        v = s->int_raw & s->int_ena;
        break;
    case A_INT_ENA:
        v = s->int_ena;
        break;
    case A_CONF:
        v = s->conf;
        break;
    case A_DATE:
        v = s->date;
        break;
    case A_INT_CLR:
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of unknown offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_ECC, addr);
        return 0;
    }
    return extract32(v, (addr & 3) * 8, size * 8);
}

/* [spec:nuos:req:emu.esp32p4.crypto] */
static void ecc_write(void *opaque, hwaddr addr, uint64_t value,
                      unsigned size)
{
    ESP32P4EccState *s = opaque;
    hwaddr reg = addr & ~3;
    int shift = (addr & 3) * 8;
    uint32_t v = deposit32(0, shift, size * 8, value);

    if (s->occupied || ecc_in_reset(s) || !clock_is_enabled(s->clk)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write at 0x%" HWADDR_PRIx
                      " dropped: the accelerator is %s\n", TYPE_ESP32P4_ECC,
                      addr, s->occupied ? "occupied by the ECDSA_DS "
                      "peripheral" : ecc_in_reset(s) ? "held in reset"
                      : "not clocked (CRYPTO_ECC_CLK)");
        return;
    }
    if (reg >= A_MEM && reg < A_MEM_END) {
        unsigned off = reg - A_MEM;
        uint8_t *p = &s->mem[off / ESP32P4_ECC_MEM_BYTES]
                            [off % ESP32P4_ECC_MEM_BYTES];

        if (s->mem_pd || ecc_busy(s)) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: write of memory at 0x%"
                          HWADDR_PRIx " while it is %s is dropped\n",
                          TYPE_ESP32P4_ECC, addr,
                          s->mem_pd ? "powered down" : "in use");
            return;
        }
        stl_le_p(p, deposit32(ldl_le_p(p), shift, size * 8, value));
        return;
    }
    switch (reg) {
    case A_INT_ENA:
        s->int_ena = deposit32(s->int_ena, shift, size * 8, value) &
                     INT_CALC_DONE;
        ecc_update_irq(s);
        break;
    case A_INT_CLR:
        s->int_raw &= ~(v & INT_CALC_DONE);
        ecc_update_irq(s);
        break;
    case A_CONF:
        if (v & CONF_RESET) {
            esp32p4_crypto_timer_cancel(&s->op);
            s->conf &= ~(CONF_START | CONF_VERIFY_RESULT);
            break;
        }
        if (ecc_busy(s)) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: ECC_MULT_CONF_REG written "
                          "during a calculation; dropped\n",
                          TYPE_ESP32P4_ECC);
            break;
        }
        s->conf = (s->conf & ~CONF_RW_MASK) |
                  (deposit32(s->conf, shift, size * 8, value) &
                   CONF_RW_MASK);
        if (v & CONF_START) {
            s->conf |= CONF_START;
            s->conf &= ~CONF_VERIFY_RESULT;
            esp32p4_crypto_timer_start(&s->op, ecc_cycles(s));
        }
        break;
    case A_DATE:
        s->date = deposit32(s->date, shift, size * 8, value) & 0x0fffffff;
        break;
    case A_INT_RAW:
    case A_INT_ST:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_ECC, addr);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to unknown offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_ECC, addr);
        break;
    }
}

static const MemoryRegionOps ecc_ops = {
    .read = ecc_read,
    .write = ecc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

void esp32p4_ecc_occupy(ESP32P4EccState *s, bool occupied)
{
    if (s->occupied && !occupied) {
        s->occupied = false;
        device_cold_reset(DEVICE(s));
        return;
    }
    s->occupied = occupied;
}

/* [spec:nuos:req:emu.esp32p4.crypto] */
static void ecc_reset_line(void *opaque, int n, int level)
{
    ESP32P4EccState *s = opaque;
    bool was = ecc_in_reset(s);

    s->reset_in[n] = level;
    if (was != ecc_in_reset(s)) {
        device_cold_reset(DEVICE(s));
    }
}

/* [spec:nuos:req:emu.esp32p4.crypto] */
static void ecc_mem_pd(void *opaque, int n, int level)
{
    ESP32P4EccState *s = opaque;

    if (level && !s->mem_pd) {
        memset(s->mem, 0, sizeof(s->mem));
    }
    s->mem_pd = level;
}

static void ecc_clk_update(void *opaque, ClockEvent event)
{
    esp32p4_crypto_timer_clock_update(&ESP32P4_ECC(opaque)->op);
}

static void ecc_reset_hold(Object *obj, ResetType type)
{
    ESP32P4EccState *s = ESP32P4_ECC(obj);

    esp32p4_crypto_timer_cancel(&s->op);
    memset(s->mem, 0, sizeof(s->mem));
    s->int_raw = 0;
    s->int_ena = 0;
    s->conf = 0;
    s->date = DATE_RESET;
}

static void ecc_reset_exit(Object *obj, ResetType type)
{
    ecc_update_irq(ESP32P4_ECC(obj));
}

static void ecc_init(Object *obj)
{
    ESP32P4EccState *s = ESP32P4_ECC(obj);

    memory_region_init_io(&s->iomem, obj, &ecc_ops, s, "esp32p4.ecc",
                          ESP32P4_ECC_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->clk = qdev_init_clock_in(DEVICE(obj), "clk", ecc_clk_update, s,
                                ClockUpdate);
    qdev_init_gpio_in_named(DEVICE(obj), ecc_reset_line, ESP32P4_ECC_RESET,
                            ESP32P4_ECC_RESETS);
    qdev_init_gpio_in_named(DEVICE(obj), ecc_mem_pd, ESP32P4_ECC_MEM_PD, 1);
}

static void ecc_realize(DeviceState *dev, Error **errp)
{
    ESP32P4EccState *s = ESP32P4_ECC(dev);

    esp32p4_crypto_timer_init(&s->op, s->clk, ecc_op_done, s);
}

static const Property ecc_properties[] = {
    DEFINE_PROP_LINK("efuse", ESP32P4EccState, efuse, TYPE_ESP32P4_EFUSE,
                     ESP32P4EfuseState *),
};

static const VMStateDescription vmstate_esp32p4_ecc = {
    .name = TYPE_ESP32P4_ECC,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_ESP32P4_CRYPTO_TIMER(op, ESP32P4EccState),
        VMSTATE_BOOL_ARRAY(reset_in, ESP32P4EccState, ESP32P4_ECC_RESETS),
        VMSTATE_BOOL(mem_pd, ESP32P4EccState),
        VMSTATE_BOOL(occupied, ESP32P4EccState),
        VMSTATE_UINT8_2DARRAY(mem, ESP32P4EccState, ESP32P4_ECC_MEMS,
                              ESP32P4_ECC_MEM_BYTES),
        VMSTATE_UINT32(int_raw, ESP32P4EccState),
        VMSTATE_UINT32(int_ena, ESP32P4EccState),
        VMSTATE_UINT32(conf, ESP32P4EccState),
        VMSTATE_UINT32(date, ESP32P4EccState),
        VMSTATE_END_OF_LIST()
    },
};

static void ecc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = ecc_realize;
    dc->vmsd = &vmstate_esp32p4_ecc;
    device_class_set_props(dc, ecc_properties);
    rc->phases.hold = ecc_reset_hold;
    rc->phases.exit = ecc_reset_exit;
}

/* [spec:nuos:req:emu.esp32p4.crypto] */
static const TypeInfo ecc_info = {
    .name          = TYPE_ESP32P4_ECC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32P4EccState),
    .instance_init = ecc_init,
    .class_init    = ecc_class_init,
};

static void ecc_register_types(void)
{
    type_register_static(&ecc_info);
}
type_init(ecc_register_types)
