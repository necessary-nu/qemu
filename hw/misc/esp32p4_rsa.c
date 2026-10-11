/*
 * ESP32-P4 RSA accelerator
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapter
 * 30 (RSA Accelerator), ESP-IDF's rsa_reg.h for v3, its mpi_ll.h and
 * mpi_hal.c.
 *
 * The three operations of TRM 30.3 on operands of N = 32 * n bits, with
 * RSA_MODE_REG = n - 1 (n from 1 to 128):
 *
 *   - Modular exponentiation (RSA_SET_START_MODEXP_REG): Z = X^Y mod M by
 *     Montgomery multiplication with software's r = R^2 mod M in RSA_Z_MEM
 *     and M' = -M^-1 mod 2^32 in RSA_M_PRIME_REG, which the accelerator
 *     uses as given, so wrong values give wrong results. RSA_X_MEM is left
 *     holding X in Montgomery form, X * R mod M, and RSA_Z_MEM the result;
 *     RSA_Y_MEM, RSA_M_MEM and M' are kept.
 *   - Modular multiplication (RSA_SET_START_MODMULT_REG): Z = X * Y mod M,
 *     with r in RSA_Z_MEM, which the result replaces.
 *   - Multiplication (RSA_SET_START_MULT_REG): with RSA_MODE_REG = 2n - 1,
 *     Z = X * Y for n-word X in RSA_X_MEM and Y in the upper half of
 *     RSA_Z_MEM (words n to 2n - 1); the 2n-word product replaces Z. An
 *     odd 2n is taken as its lower even length and logged.
 *
 * The SEARCH option (RSA_SEARCH_ENABLE_REG) ignores the bits of Y above
 * RSA_SEARCH_POS_REG, which gives a wrong result if a set bit is above it,
 * as the TRM warns. The CONSTANT_TIME option (RSA_CONSTANT_TIME_REG, set
 * from reset) does a multiplication for every bit of Y; cleared, the
 * accelerator skips Y's leading zeros and multiplies only for its set
 * bits. Neither changes a correct result, only the time.
 *
 * Timing: a Montgomery multiplication of n-word numbers takes 3n^2 + 8n
 * cycles of CRYPTO_RSA_CLK, which with the two conversions reproduces the
 * figures of TRM table 30.3-1 (174.6 million cycles for a 3072-bit
 * exponentiation without the options, 1.02 million with SEARCH at 16). A
 * multiplication takes 3n^2 + 8n cycles too. RSA_QUERY_IDLE_REG reads 0
 * until the operation is done; then RSA_INTR raises while RSA_INT_ENA_REG
 * is set, until RSA_INT_CLR_REG.
 *
 * After its reset the accelerator clears its memories, which takes 128
 * cycles during which RSA_QUERY_CLEAN_REG and RSA_QUERY_IDLE_REG read 0.
 * Its memories cannot be reached while it computes or initializes (reads
 * give 0, writes are dropped and logged); powered down by
 * HP_SYSTEM_RSA_PD_CTRL_REG they lose their contents and read as zero.
 * Writes while its clock is stopped or it is held in reset are dropped and
 * logged, and while the RSA_DS peripheral occupies it its registers read
 * as zero and take no writes; it resets when released.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/misc/esp32p4_rsa.h"
#include "migration/vmstate.h"

#define A_MEM_END           0x800
#define A_M_PRIME           0x800
#define A_MODE              0x804
#define A_QUERY_CLEAN       0x808
#define A_SET_START_MODEXP  0x80c
#define A_SET_START_MODMULT 0x810
#define A_SET_START_MULT    0x814
#define A_QUERY_IDLE        0x818
#define A_INT_CLR           0x81c
#define A_CONSTANT_TIME     0x820
#define A_SEARCH_ENABLE     0x824
#define A_SEARCH_POS        0x828
#define A_INT_ENA           0x82c
#define A_DATE              0x830

#define DATE_RESET          538969624
#define CLEAN_CYCLES        128

/* The memories in address order: M, Z, Y, X. */
#define MEM_BYTES           (ESP32P4_RSA_WORDS * 4)

enum {
    RSA_OP_NONE,
    RSA_OP_CLEAN,
    RSA_OP_MODEXP,
    RSA_OP_MODMULT,
    RSA_OP_MULT,
};

static bool rsa_in_reset(ESP32P4RsaState *s)
{
    for (int i = 0; i < ESP32P4_RSA_RESETS; i++) {
        if (s->reset_in[i]) {
            return true;
        }
    }
    return false;
}

static void rsa_update_irq(ESP32P4RsaState *s)
{
    qemu_set_irq(s->irq, s->done_int && (s->int_ena & 1));
}

static uint64_t mont_cycles(unsigned n)
{
    return 3ULL * n * n + 8ULL * n;
}

uint64_t esp32p4_rsa_modexp(uint32_t *z, const uint32_t *x,
                            const uint32_t *y, const uint32_t *m,
                            const uint32_t *r, uint32_t mprime, unsigned n,
                            unsigned top, bool constant_time,
                            uint32_t *x_mont)
{
    uint32_t one[ESP32P4_BN_MAX_WORDS] = { 1 };
    uint32_t xm[ESP32P4_BN_MAX_WORDS];
    uint32_t acc[ESP32P4_BN_MAX_WORDS];
    uint64_t mults = 2;
    bool started = false;
    int bit;

    esp32p4_bn_mont_mul(xm, x, r, m, mprime, n);
    esp32p4_bn_mont_mul(acc, one, r, m, mprime, n);
    for (bit = top; bit >= 0; bit--) {
        bool set = esp32p4_bn_bit(y, bit);

        started |= set;
        if (constant_time) {
            mults += 2;
        } else if (started) {
            mults += set ? 2 : 1;
        }
        esp32p4_bn_mont_mul(acc, acc, acc, m, mprime, n);
        if (set) {
            esp32p4_bn_mont_mul(acc, acc, xm, m, mprime, n);
        }
    }
    esp32p4_bn_mont_mul(z, acc, one, m, mprime, n);
    if (x_mont) {
        memcpy(x_mont, xm, n * sizeof(uint32_t));
    }
    return mults * mont_cycles(n);
}

/* The operand length in words for an operation of kind @op. */
static unsigned rsa_words(ESP32P4RsaState *s, uint32_t op)
{
    unsigned n = s->mode + 1;

    return op == RSA_OP_MULT ? n / 2 : n;
}

static unsigned rsa_top(ESP32P4RsaState *s, unsigned n)
{
    unsigned top = 32 * n - 1;

    if (s->search_enable & 1) {
        top = MIN(top, s->search_pos);
    }
    return top;
}

/* [spec:nuos:req:emu.esp32p4.crypto] */
static void rsa_op_done(void *opaque)
{
    ESP32P4RsaState *s = opaque;
    uint32_t *m = s->mem[ESP32P4_RSA_MEM_M];
    uint32_t *z = s->mem[ESP32P4_RSA_MEM_Z];
    uint32_t *y = s->mem[ESP32P4_RSA_MEM_Y];
    uint32_t *x = s->mem[ESP32P4_RSA_MEM_X];
    uint32_t t[2 * ESP32P4_BN_MAX_WORDS];
    unsigned n = rsa_words(s, s->op_kind);

    switch (s->op_kind) {
    case RSA_OP_CLEAN:
        memset(s->mem, 0, sizeof(s->mem));
        s->clean = true;
        s->idle = true;
        s->op_kind = RSA_OP_NONE;
        return;
    case RSA_OP_MODEXP:
        esp32p4_rsa_modexp(t, x, y, m, z, s->m_prime, n, rsa_top(s, n),
                           s->constant_time & 1, x);
        memcpy(z, t, n * sizeof(uint32_t));
        break;
    case RSA_OP_MODMULT:
        esp32p4_bn_mont_mul(t, x, z, m, s->m_prime, n);
        esp32p4_bn_mont_mul(z, t, y, m, s->m_prime, n);
        break;
    case RSA_OP_MULT:
        esp32p4_bn_mul(t, x, z + n, n);
        memcpy(z, t, 2 * n * sizeof(uint32_t));
        break;
    default:
        return;
    }
    s->op_kind = RSA_OP_NONE;
    s->idle = true;
    s->done_int = true;
    rsa_update_irq(s);
}

/* [spec:nuos:req:emu.esp32p4.crypto] */
static void rsa_start(ESP32P4RsaState *s, uint32_t op)
{
    unsigned n = rsa_words(s, op);
    uint64_t cycles;

    if (!s->idle) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: an operation was started while "
                      "the accelerator is busy; it is ignored\n",
                      TYPE_ESP32P4_RSA);
        return;
    }
    if (op == RSA_OP_MULT && ((s->mode + 1) & 1)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: multiplication with RSA_MODE %u "
                      "(an odd result length); the operands are taken as %u "
                      "words\n", TYPE_ESP32P4_RSA, s->mode, n);
    }
    if (!n) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: multiplication with RSA_MODE 0 "
                      "has no operands; it is ignored\n", TYPE_ESP32P4_RSA);
        return;
    }
    switch (op) {
    case RSA_OP_MODEXP: {
        const uint32_t *y = s->mem[ESP32P4_RSA_MEM_Y];
        unsigned top = rsa_top(s, n);
        uint64_t mults = 2;

        if (s->constant_time & 1) {
            mults += 2ULL * (top + 1);
        } else {
            int bit;
            bool started = false;

            for (bit = top; bit >= 0; bit--) {
                bool set = esp32p4_bn_bit(y, bit);

                started |= set;
                if (started) {
                    mults += set ? 2 : 1;
                }
            }
        }
        cycles = mults * mont_cycles(n);
        break;
    }
    case RSA_OP_MODMULT:
        cycles = 2 * mont_cycles(n);
        break;
    default:
        cycles = mont_cycles(n);
        break;
    }
    s->op_kind = op;
    s->idle = false;
    esp32p4_crypto_timer_start(&s->op, cycles);
}

static bool rsa_mem_ok(ESP32P4RsaState *s, hwaddr addr, bool write)
{
    if (s->mem_pd) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: %s of memory at 0x%" HWADDR_PRIx
                      " while HP_SYSTEM_RSA_PD_CTRL_REG powers it down\n",
                      TYPE_ESP32P4_RSA, write ? "write" : "read", addr);
        return false;
    }
    if (!s->idle) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: %s of memory at 0x%" HWADDR_PRIx
                      " while the accelerator is busy\n", TYPE_ESP32P4_RSA,
                      write ? "write" : "read", addr);
        return false;
    }
    return true;
}

static uint64_t rsa_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4RsaState *s = opaque;
    hwaddr reg = addr & ~3;
    uint32_t v = 0;

    if (s->occupied) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read at 0x%" HWADDR_PRIx
                      " while the RSA_DS peripheral occupies the "
                      "accelerator\n", TYPE_ESP32P4_RSA, addr);
        return 0;
    }
    if (reg < A_MEM_END) {
        if (!rsa_mem_ok(s, addr, false)) {
            return 0;
        }
        v = s->mem[reg / MEM_BYTES][(reg % MEM_BYTES) / 4];
        return extract32(v, (addr & 3) * 8, size * 8);
    }
    switch (reg) {
    case A_M_PRIME:
        v = s->m_prime;
        break;
    case A_MODE:
        v = s->mode;
        break;
    case A_QUERY_CLEAN:
        v = s->clean;
        break;
    case A_QUERY_IDLE:
        v = s->idle;
        break;
    case A_CONSTANT_TIME:
        v = s->constant_time;
        break;
    case A_SEARCH_ENABLE:
        v = s->search_enable;
        break;
    case A_SEARCH_POS:
        v = s->search_pos;
        break;
    case A_INT_ENA:
        v = s->int_ena;
        break;
    case A_DATE:
        v = s->date;
        break;
    case A_SET_START_MODEXP:
    case A_SET_START_MODMULT:
    case A_SET_START_MULT:
    case A_INT_CLR:
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of unknown offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_RSA, addr);
        return 0;
    }
    return extract32(v, (addr & 3) * 8, size * 8);
}

/* [spec:nuos:req:emu.esp32p4.crypto] */
static void rsa_write(void *opaque, hwaddr addr, uint64_t value,
                      unsigned size)
{
    ESP32P4RsaState *s = opaque;
    hwaddr reg = addr & ~3;
    int shift = (addr & 3) * 8;
    uint32_t v = deposit32(0, shift, size * 8, value);

    if (s->occupied || rsa_in_reset(s) || !clock_is_enabled(s->clk)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write at 0x%" HWADDR_PRIx
                      " dropped: the accelerator is %s\n", TYPE_ESP32P4_RSA,
                      addr, s->occupied ? "occupied by the RSA_DS peripheral"
                      : rsa_in_reset(s) ? "held in reset"
                      : "not clocked (CRYPTO_RSA_CLK)");
        return;
    }
    if (reg < A_MEM_END) {
        uint32_t *w;

        if (!rsa_mem_ok(s, addr, true)) {
            return;
        }
        w = &s->mem[reg / MEM_BYTES][(reg % MEM_BYTES) / 4];
        *w = deposit32(*w, shift, size * 8, value);
        return;
    }
    if (!s->idle && reg != A_INT_CLR && reg != A_INT_ENA) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write at 0x%" HWADDR_PRIx
                      " while the accelerator is busy is dropped\n",
                      TYPE_ESP32P4_RSA, addr);
        return;
    }

#define MERGE(old) deposit32((old), shift, size * 8, value)
    switch (reg) {
    case A_M_PRIME:
        s->m_prime = MERGE(s->m_prime);
        break;
    case A_MODE:
        s->mode = MERGE(s->mode) & 0x7f;
        break;
    case A_SET_START_MODEXP:
        if (v & 1) {
            rsa_start(s, RSA_OP_MODEXP);
        }
        break;
    case A_SET_START_MODMULT:
        if (v & 1) {
            rsa_start(s, RSA_OP_MODMULT);
        }
        break;
    case A_SET_START_MULT:
        if (v & 1) {
            rsa_start(s, RSA_OP_MULT);
        }
        break;
    case A_INT_CLR:
        if (v & 1) {
            s->done_int = false;
            rsa_update_irq(s);
        }
        break;
    case A_CONSTANT_TIME:
        s->constant_time = MERGE(s->constant_time) & 1;
        break;
    case A_SEARCH_ENABLE:
        s->search_enable = MERGE(s->search_enable) & 1;
        break;
    case A_SEARCH_POS:
        s->search_pos = MERGE(s->search_pos) & 0xfff;
        break;
    case A_INT_ENA:
        s->int_ena = MERGE(s->int_ena) & 1;
        rsa_update_irq(s);
        break;
    case A_DATE:
        s->date = MERGE(s->date) & 0x3fffffff;
        break;
    case A_QUERY_CLEAN:
    case A_QUERY_IDLE:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_RSA, addr);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to unknown offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_RSA, addr);
        break;
    }
#undef MERGE
}

static const MemoryRegionOps rsa_ops = {
    .read = rsa_read,
    .write = rsa_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

void esp32p4_rsa_occupy(ESP32P4RsaState *s, bool occupied)
{
    if (s->occupied && !occupied) {
        s->occupied = false;
        device_cold_reset(DEVICE(s));
        return;
    }
    s->occupied = occupied;
}

/* After its reset the accelerator initializes its memories. */
static void rsa_begin_clean(ESP32P4RsaState *s)
{
    if (rsa_in_reset(s)) {
        return;
    }
    s->op_kind = RSA_OP_CLEAN;
    esp32p4_crypto_timer_start(&s->op, CLEAN_CYCLES);
}

/* [spec:nuos:req:emu.esp32p4.crypto] */
static void rsa_reset_line(void *opaque, int n, int level)
{
    ESP32P4RsaState *s = opaque;
    bool was = rsa_in_reset(s);

    s->reset_in[n] = level;
    if (was != rsa_in_reset(s)) {
        device_cold_reset(DEVICE(s));
    }
}

/* [spec:nuos:req:emu.esp32p4.crypto] */
static void rsa_mem_pd(void *opaque, int n, int level)
{
    ESP32P4RsaState *s = opaque;

    if (level && !s->mem_pd) {
        memset(s->mem, 0, sizeof(s->mem));
    }
    s->mem_pd = level;
}

static void rsa_clk_update(void *opaque, ClockEvent event)
{
    esp32p4_crypto_timer_clock_update(&ESP32P4_RSA(opaque)->op);
}

static void rsa_reset_hold(Object *obj, ResetType type)
{
    ESP32P4RsaState *s = ESP32P4_RSA(obj);

    esp32p4_crypto_timer_cancel(&s->op);
    memset(s->mem, 0, sizeof(s->mem));
    s->m_prime = 0;
    s->mode = 0;
    s->constant_time = 1;
    s->search_enable = 0;
    s->search_pos = 0;
    s->int_ena = 0;
    s->date = DATE_RESET;
    s->clean = false;
    s->idle = false;
    s->done_int = false;
    s->op_kind = RSA_OP_NONE;
}

static void rsa_reset_exit(Object *obj, ResetType type)
{
    ESP32P4RsaState *s = ESP32P4_RSA(obj);

    rsa_update_irq(s);
    rsa_begin_clean(s);
}

static void rsa_init(Object *obj)
{
    ESP32P4RsaState *s = ESP32P4_RSA(obj);

    memory_region_init_io(&s->iomem, obj, &rsa_ops, s, "esp32p4.rsa",
                          ESP32P4_RSA_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->clk = qdev_init_clock_in(DEVICE(obj), "clk", rsa_clk_update, s,
                                ClockUpdate);
    qdev_init_gpio_in_named(DEVICE(obj), rsa_reset_line, ESP32P4_RSA_RESET,
                            ESP32P4_RSA_RESETS);
    qdev_init_gpio_in_named(DEVICE(obj), rsa_mem_pd, ESP32P4_RSA_MEM_PD, 1);
}

static void rsa_realize(DeviceState *dev, Error **errp)
{
    ESP32P4RsaState *s = ESP32P4_RSA(dev);

    esp32p4_crypto_timer_init(&s->op, s->clk, rsa_op_done, s);
}

static const VMStateDescription vmstate_esp32p4_rsa = {
    .name = TYPE_ESP32P4_RSA,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_ESP32P4_CRYPTO_TIMER(op, ESP32P4RsaState),
        VMSTATE_BOOL_ARRAY(reset_in, ESP32P4RsaState, ESP32P4_RSA_RESETS),
        VMSTATE_BOOL(mem_pd, ESP32P4RsaState),
        VMSTATE_BOOL(occupied, ESP32P4RsaState),
        VMSTATE_UINT32_2DARRAY(mem, ESP32P4RsaState, ESP32P4_RSA_MEMS,
                               ESP32P4_RSA_WORDS),
        VMSTATE_UINT32(m_prime, ESP32P4RsaState),
        VMSTATE_UINT32(mode, ESP32P4RsaState),
        VMSTATE_UINT32(constant_time, ESP32P4RsaState),
        VMSTATE_UINT32(search_enable, ESP32P4RsaState),
        VMSTATE_UINT32(search_pos, ESP32P4RsaState),
        VMSTATE_UINT32(int_ena, ESP32P4RsaState),
        VMSTATE_UINT32(date, ESP32P4RsaState),
        VMSTATE_BOOL(clean, ESP32P4RsaState),
        VMSTATE_BOOL(idle, ESP32P4RsaState),
        VMSTATE_BOOL(done_int, ESP32P4RsaState),
        VMSTATE_UINT32(op_kind, ESP32P4RsaState),
        VMSTATE_END_OF_LIST()
    },
};

static void rsa_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = rsa_realize;
    dc->vmsd = &vmstate_esp32p4_rsa;
    rc->phases.hold = rsa_reset_hold;
    rc->phases.exit = rsa_reset_exit;
}

/* [spec:nuos:req:emu.esp32p4.crypto] */
static const TypeInfo rsa_info = {
    .name          = TYPE_ESP32P4_RSA,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32P4RsaState),
    .instance_init = rsa_init,
    .class_init    = rsa_class_init,
};

static void rsa_register_types(void)
{
    type_register_static(&rsa_info);
}
type_init(rsa_register_types)
