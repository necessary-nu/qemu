/*
 * ESP32 RSA accelerator
 *
 * Copyright (c) 2020 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/core/sysbus.h"
#include "hw/core/boards.h"
#include "hw/misc/esp32_rsa.h"


#define ESP32_RSA_REGS_SIZE (A_RSA_QUERY_CLEAN_REG + 4)
#define ESP32_RSA_MAX_WORDS (ESP32_RSA_MEM_BLK_SIZE / 4)

/*
 * Montgomery product r = a * b * R^-1 mod m, R = 2^(32 * n), with the
 * hardware's own M' (-m^-1 mod 2^32) from RSA_M_DASH_REG. Like the
 * accelerator, a wrong M' or an operand out of range gives a wrong result.
 * r may alias a or b.
 */
static void esp32_rsa_mont_mul(uint32_t *r, const uint32_t *a,
                               const uint32_t *b, const uint32_t *m,
                               uint32_t mprime, size_t n)
{
    uint32_t t[ESP32_RSA_MAX_WORDS + 2] = {};
    uint32_t d[ESP32_RSA_MAX_WORDS];
    uint64_t c;
    bool ge;

    for (size_t i = 0; i < n; i++) {
        c = 0;
        for (size_t j = 0; j < n; j++) {
            c += (uint64_t)t[j] + (uint64_t)a[j] * b[i];
            t[j] = c;
            c >>= 32;
        }
        c += t[n];
        t[n] = c;
        t[n + 1] = c >> 32;

        uint32_t q = t[0] * mprime;
        c = ((uint64_t)t[0] + (uint64_t)q * m[0]) >> 32;
        for (size_t j = 1; j < n; j++) {
            c += (uint64_t)t[j] + (uint64_t)q * m[j];
            t[j - 1] = c;
            c >>= 32;
        }
        c += t[n];
        t[n - 1] = c;
        t[n] = t[n + 1] + (c >> 32);
    }

    ge = t[n] != 0;
    if (!ge) {
        ge = true;
        for (size_t j = n; j-- > 0;) {
            if (t[j] != m[j]) {
                ge = t[j] > m[j];
                break;
            }
        }
    }
    if (ge) {
        int64_t borrow = 0;
        for (size_t j = 0; j < n; j++) {
            borrow += (int64_t)t[j] - m[j];
            d[j] = borrow;
            borrow >>= 32;
        }
        memcpy(r, d, n * sizeof(uint32_t));
    } else {
        memcpy(r, t, n * sizeof(uint32_t));
    }
}

/*
 * Calculates Z_MEM = X_MEM ^ Y_MEM mod M_MEM. Software loads
 * Rb = R^2 mod M into Z_MEM beforehand, as the hardware requires.
 */
static void esp32_rsa_exp_mod(Esp32RsaState *s)
{
    size_t n = (s->rsa_modexp_mode_reg + 1) * 16;
    uint32_t one[ESP32_RSA_MAX_WORDS] = { 1 };
    uint32_t xm[ESP32_RSA_MAX_WORDS];
    uint32_t acc[ESP32_RSA_MAX_WORDS];

    if (n > ESP32_RSA_MAX_WORDS) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: invalid mode %u\n", __func__,
                      s->rsa_modexp_mode_reg);
        return;
    }

    esp32_rsa_mont_mul(xm, s->rsa_x_mem, s->rsa_z_mem, s->rsa_m_mem,
                       s->rsa_mprime_reg, n);
    esp32_rsa_mont_mul(acc, one, s->rsa_z_mem, s->rsa_m_mem,
                       s->rsa_mprime_reg, n);
    for (size_t bit = n * 32; bit-- > 0;) {
        esp32_rsa_mont_mul(acc, acc, acc, s->rsa_m_mem, s->rsa_mprime_reg, n);
        if (s->rsa_y_mem[bit / 32] & (1u << (bit % 32))) {
            esp32_rsa_mont_mul(acc, acc, xm, s->rsa_m_mem,
                               s->rsa_mprime_reg, n);
        }
    }
    esp32_rsa_mont_mul(acc, acc, one, s->rsa_m_mem, s->rsa_mprime_reg, n);

    memset(s->rsa_z_mem, 0, sizeof(s->rsa_z_mem));
    memcpy(s->rsa_z_mem, acc, n * sizeof(uint32_t));
    s->rsa_q_int_reg = 1;
}

/*
 * Calculates Z_MEM = X_MEM * (Z_MEM >> N), N being the input length. The
 * output length is set by RSA_MULT_MODE_REG and the inputs are half that.
 */
static void esp32_rsa_mul_op(Esp32RsaState *s)
{
    size_t n_out = (s->rsa_mult_mode_reg - 8 + 1) * 16;
    size_t n_in = n_out / 2;
    uint32_t r[ESP32_RSA_MAX_WORDS] = {};
    const uint32_t *z = s->rsa_z_mem + n_in;

    for (size_t i = 0; i < n_in; i++) {
        uint64_t c = 0;
        for (size_t j = 0; j < n_in; j++) {
            c += (uint64_t)r[i + j] + (uint64_t)s->rsa_x_mem[j] * z[i];
            r[i + j] = c;
            c >>= 32;
        }
        r[i + n_in] = c;
    }

    memcpy(s->rsa_z_mem, r, sizeof(s->rsa_z_mem));
    s->rsa_q_int_reg = 1;
}

/*
 * Calculates Z_MEM = Z_MEM * X_MEM * R^-1 mod M_MEM, one Montgomery
 * multiplication. Software runs it twice, first with Z_MEM = Rb, to get
 * X * Y mod M.
 */
static void esp32_rsa_mod_mul_op(Esp32RsaState *s)
{
    size_t n = (s->rsa_mult_mode_reg + 1) * 16;
    uint32_t r[ESP32_RSA_MAX_WORDS];

    esp32_rsa_mont_mul(r, s->rsa_z_mem, s->rsa_x_mem, s->rsa_m_mem,
                       s->rsa_mprime_reg, n);
    memset(s->rsa_z_mem, 0, sizeof(s->rsa_z_mem));
    memcpy(s->rsa_z_mem, r, n * sizeof(uint32_t));
    s->rsa_q_int_reg = 1;
}

static void esp32_rsa_mul_start(Esp32RsaState *s)
{
    if (s->rsa_mult_mode_reg < 8) {
        esp32_rsa_mod_mul_op(s);
    } else if (s->rsa_mult_mode_reg < 16) {
        esp32_rsa_mul_op(s);
    } else {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: invalid mode %u\n", __func__,
                      s->rsa_mult_mode_reg);
    }
}

static void esp32_rsa_clean_mem(Esp32RsaState *s)
{
    memset(s->rsa_m_mem, 0, sizeof(s->rsa_m_mem));
    memset(s->rsa_x_mem, 0, sizeof(s->rsa_x_mem));
    memset(s->rsa_y_mem, 0, sizeof(s->rsa_y_mem));
    memset(s->rsa_z_mem, 0, sizeof(s->rsa_z_mem));
}

static uint64_t esp32_rsa_read(void *opaque, hwaddr addr, unsigned int size)
{
    Esp32RsaState *s = ESP32_RSA(opaque);
    uint64_t val = 0;

    switch (addr) {
        case A_RSA_MEM_Z_BLOCK_BASE ... (A_RSA_MEM_Z_BLOCK_BASE + ESP32_RSA_MEM_BLK_SIZE - 1):
            val = s->rsa_z_mem[(addr - A_RSA_MEM_Z_BLOCK_BASE) / sizeof(uint32_t)];
            break;

        case A_RSA_QUERY_CLEAN_REG:
            /* After coming out from reset, RSA Accelerator first initialize
             * internal memory block to zeros before turning this register to 1.
             * Software poll this register to read 1, before using the internal
             * memory blocks. Internal memory block initialisation performed
             * here before returning this read operation to 1.
             */
            esp32_rsa_clean_mem(s);
            val = s->rsa_clean_reg;
            break;

        case A_RSA_QUERY_INTERRUPT_REG:
            val = s->rsa_q_int_reg;
            break;
    }

    return val;
}


static void esp32_rsa_write(void *opaque, hwaddr addr,
                       uint64_t value, unsigned int size)
{
    Esp32RsaState *s = ESP32_RSA(opaque);

    switch (addr) {

        case A_RSA_MEM_M_BLOCK_BASE ... (A_RSA_MEM_M_BLOCK_BASE + ESP32_RSA_MEM_BLK_SIZE - 1):
            s->rsa_m_mem[(addr - A_RSA_MEM_M_BLOCK_BASE) / sizeof(uint32_t)] = (uint32_t)value;
                    break;

        case A_RSA_MEM_RB_BLOCK_BASE ... (A_RSA_MEM_RB_BLOCK_BASE + ESP32_RSA_MEM_BLK_SIZE - 1):
            s->rsa_z_mem[(addr - A_RSA_MEM_RB_BLOCK_BASE) / sizeof(uint32_t)] = (uint32_t)value;
            break;

        case A_RSA_MEM_Y_BLOCK_BASE ... (A_RSA_MEM_Y_BLOCK_BASE + ESP32_RSA_MEM_BLK_SIZE - 1):
            s->rsa_y_mem[(addr - A_RSA_MEM_Y_BLOCK_BASE) / sizeof(uint32_t)] = (uint32_t)value;
            break;

        case A_RSA_MEM_X_BLOCK_BASE ... (A_RSA_MEM_X_BLOCK_BASE + ESP32_RSA_MEM_BLK_SIZE - 1):
            s->rsa_x_mem[(addr - A_RSA_MEM_X_BLOCK_BASE) / sizeof(uint32_t)] = (uint32_t)value;
            break;

        case A_RSA_M_DASH_REG:
            s->rsa_mprime_reg = value;
            break;

        case A_RSA_MODEXP_MODE_REG:
            s->rsa_modexp_mode_reg = value;
            break;

        case A_RSA_MULT_MODE_REG:
            s->rsa_mult_mode_reg = value;
            break;

        case A_RSA_MODEXP_START_REG:
            esp32_rsa_exp_mod(s);
            break;

        case A_RSA_MULT_START_REG:
            esp32_rsa_mul_start(s);
            break;

        case A_RSA_QUERY_INTERRUPT_REG:
            /* Clear on write register */
            s->rsa_q_int_reg &= ~value;
            break;
    }

}

static const MemoryRegionOps esp32_rsa_ops = {
    .read =  esp32_rsa_read,
    .write = esp32_rsa_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32_rsa_reset_hold(Object *obj, ResetType type)
{
    Esp32RsaState *s = ESP32_RSA(obj);

    esp32_rsa_clean_mem(s);

    /* Clear any spurious interrupt */
    s->rsa_q_int_reg = 0;

    /* RSA memory block initialization complete */
    s->rsa_clean_reg = 1;
}

static void esp32_rsa_init(Object *obj)
{
    Esp32RsaState *s = ESP32_RSA(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32_rsa_ops, s,
                          TYPE_ESP32_RSA, ESP32_RSA_REGS_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
}

static void esp32_rsa_class_init(ObjectClass *klass, const void *data)
{
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    rc->phases.hold = esp32_rsa_reset_hold;
}

static const TypeInfo esp32_rsa_info = {
    .name = TYPE_ESP32_RSA,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32RsaState),
    .instance_init = esp32_rsa_init,
    .class_init = esp32_rsa_class_init
};

static void esp32_rsa_register_types(void)
{
    type_register_static(&esp32_rsa_info);
}

type_init(esp32_rsa_register_types)
