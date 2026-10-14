/*
 * ESP32-P4 SHA accelerator
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapter
 * 31 (SHA Accelerator), ESP-IDF's sha_reg.h for v3 and its sha_ll.h.
 *
 * SHA-1, SHA-224, SHA-256, SHA-384, SHA-512, SHA-512/224, SHA-512/256 and
 * SHA-512/t (SHA_MODE_REG 0 to 7) on blocks software has padded.
 *
 * Typical SHA: SHA_START_REG hashes the block in SHA_M_n_REG from the
 * algorithm's initial hash value, SHA_CONTINUE_REG from the hash value in
 * SHA_H_n_REG, which software can save and restore to interleave
 * messages. With SHA-512/t, SHA_START_REG instead computes the algorithm's
 * initial hash value from SHA_T_STRING_REG and SHA_T_LENGTH_REG (TRM
 * 31.4.1.3) and the message's blocks all follow with SHA_CONTINUE_REG.
 *
 * DMA-SHA: SHA_DMA_START_REG and SHA_DMA_CONTINUE_REG hash
 * SHA_DMA_BLOCK_NUM_REG blocks the TX channel of GDMA-AXI that selects
 * the SHA peripheral delivers, from the initial or the current hash value
 * (SHA-512/t: from the value SHA_START_REG computed, as the TRM's process
 * has it), and at the end raise SHA_INTR while SHA_INT_ENA_REG is set,
 * until SHA_CLEAR_IRQ_REG; as the TRM notes, the interrupt has no raw or
 * status register. SHA_DMA_RX_RESET_REG drops a partly received block.
 *
 * SHA_H_n_REG and SHA_M_n_REG hold the digest and the block as byte
 * strings, byte 0 in bits 7:0 of the first register, which is what
 * ESP-IDF writes and reads: the TRM's "most significant 32 bits ... in
 * SHA_M_0_REG" is the byte string's first four bytes.
 *
 * Timing: a block takes as many cycles of CRYPTO_SHA_CLK as the
 * algorithm has rounds, plus 8 (the model's figures), during which
 * SHA_BUSY_REG reads 1; computing SHA-512/t's initial value is one
 * SHA-512 block. The accelerator waits while its clock is stopped.
 *
 * SHA_START_REG and SHA_CONTINUE_REG take the block in SHA_M_n_REG and
 * the algorithm in SHA_MODE_REG as the operation starts, so that, as the
 * TRM's Typical SHA process notes (31.4.2.1, note 1), software may write
 * the next block into SHA_M_n_REG while the accelerator hashes this one;
 * the v3.x boot ROM's ets_sha_process() writes SHA_MODE_REG and the next
 * block before it waits for SHA_BUSY_REG, and only then starts the block.
 *
 * A write of 1 to SHA_START_REG, SHA_CONTINUE_REG, SHA_DMA_START_REG or
 * SHA_DMA_CONTINUE_REG while the accelerator is busy waits on the bus
 * until the operation in progress ends, holding the core that makes it
 * (esp32p4_cpu_bus_hold()), then starts its operation. ESP-IDF's block
 * mode, since v5.5 (commit fe78370ec9), writes each block and its
 * SHA_CONTINUE_REG without waiting for SHA_BUSY_REG between blocks, on
 * every chip of this accelerator, and its HAL test hashes 1030 bytes so
 * with known digests on silicon; the TRM says nothing of such writes.
 * Dropped, they would leave blocks unhashed; taken later without holding
 * the core, they would let it overwrite SHA_M_n_REG with the next block
 * before the operation took this one. A core held this way makes the
 * write before it takes an interrupt pending as the wait ends, such as
 * SHA_INTR at the end of a DMA-SHA operation. A start written by another
 * master (qtest), which no wait state holds, while the accelerator is
 * busy is dropped and logged.
 *
 * Other registers written while it is busy are dropped and logged, as are
 * writes while its clock is stopped or it is held in reset. While HMAC,
 * RSA_DS or ECDSA_DS occupies it its registers read as zero and take no
 * writes, and it resets when released.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/bswap.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/gpio/esp32_line.h"
#include "hw/misc/esp32p4_sha.h"
#include "migration/vmstate.h"
#include "target/riscv/esp32p4_cpu.h"

#define A_MODE          0x00
#define A_T_STRING      0x04
#define A_T_LENGTH      0x08
#define A_DMA_BLOCK_NUM 0x0c
#define A_START         0x10
#define A_CONTINUE      0x14
#define A_BUSY          0x18
#define A_DMA_START     0x1c
#define A_DMA_CONTINUE  0x20
#define A_CLEAR_IRQ     0x24
#define A_IRQ_ENA       0x28
#define A_DATE          0x2c
#define A_DMA_RX_RESET  0x30
#define A_H_MEM         0x40
#define A_M_MEM         0x80
#define SHA_REGS_END    0x100

#define DATE_RESET      539232291

enum {
    OP_NONE,
    OP_START,           /* a typical block from the initial value */
    OP_CONTINUE,        /* a typical block from SHA_H */
    OP_T_INIT,          /* SHA-512/t's initial value */
    OP_DMA,             /* DMA blocks */
};

static bool sha_in_reset(ESP32P4ShaDevState *s)
{
    for (int i = 0; i < ESP32P4_SHA_RESETS; i++) {
        if (s->reset_in[i]) {
            return true;
        }
    }
    return false;
}

static void sha_update_irq(ESP32P4ShaDevState *s)
{
    qemu_set_irq(s->irq, s->done_int && (s->irq_ena & 1));
}

/* The cores waiting to start an operation may make their writes again. */
/* [spec:nuos:req:emu.esp32p4.crypto] */
static void sha_release_waiters(ESP32P4ShaDevState *s)
{
    CPUState *cs;

    CPU_FOREACH(cs) {
        if (s->waiters & BIT(cs->cpu_index)) {
            esp32p4_cpu_bus_release(ESP32P4_CPU(cs));
        }
    }
    s->waiters = 0;
}

static uint64_t sha_block_cycles(uint32_t mode)
{
    if (mode == ESP32P4_SHA1) {
        return 80 + 8;
    }
    return (mode >= ESP32P4_SHA384 ? 80 : 64) + 8;
}

static void sha_store(ESP32P4ShaDevState *s)
{
    esp32p4_sha_state_to_bytes(s->op_mode, &s->state, s->h);
}

static void sha_pump(ESP32P4ShaDevState *s);

/* [spec:nuos:req:emu.esp32p4.crypto] */
static void sha_op_done(void *opaque)
{
    ESP32P4ShaDevState *s = opaque;

    switch (s->op_kind) {
    case OP_START:
    case OP_CONTINUE:
        esp32p4_sha_compress(s->op_mode, &s->state, s->in_buf);
        sha_store(s);
        break;
    case OP_T_INIT:
        sha_store(s);
        break;
    case OP_DMA:
        esp32p4_sha_compress(s->op_mode, &s->state, s->in_buf);
        s->in_len = 0;
        s->left--;
        sha_pump(s);
        return;
    default:
        return;
    }
    s->op_kind = OP_NONE;
    s->busy = false;
    sha_release_waiters(s);
}

/* [spec:nuos:req:emu.esp32p4.crypto] */
static void sha_pump(ESP32P4ShaDevState *s)
{
    unsigned bs = esp32p4_sha_block_size(s->op_mode);
    bool eof;

    if (s->op_kind != OP_DMA || s->op.active || sha_in_reset(s) ||
        !clock_is_enabled(s->clk)) {
        return;
    }
    if (!s->left) {
        sha_store(s);
        s->op_kind = OP_NONE;
        s->busy = false;
        s->done_int = true;
        sha_update_irq(s);
        sha_release_waiters(s);
        return;
    }
    if (s->in_len < bs) {
        s->in_len += esp32p4_gdma_pull(s->gdma, ESP32P4_GDMA_AXI_SHA,
                                       s->in_buf + s->in_len,
                                       bs - s->in_len, &eof);
        if (s->in_len < bs) {
            return;
        }
    }
    esp32p4_crypto_timer_start(&s->op, sha_block_cycles(s->op_mode));
}

static void sha_gdma_ready(void *opaque)
{
    sha_pump(opaque);
}

/*
 * [spec:nuos:req:emu.esp32p4.crypto]
 * GDMA's register access: the blocks whose cycles have run by then are
 * done, as their DMA descriptors and EOF show then.
 */
static void sha_gdma_sync(void *opaque, int64_t when)
{
    ESP32P4ShaDevState *s = opaque;

    esp32p4_crypto_timer_sync(&s->op, when);
}

static const ESP32P4GdmaPeriphOps sha_gdma_ops = {
    .tx_ready = sha_gdma_ready,
    .sync = sha_gdma_sync,
};

/* [spec:nuos:req:emu.esp32p4.crypto] */
static void sha_begin(ESP32P4ShaDevState *s, uint32_t kind, bool initial)
{
    s->op_mode = s->mode;
    if (s->mode == ESP32P4_SHA512_T && initial) {
        if (kind == OP_START) {
            kind = OP_T_INIT;
        }
        esp32p4_sha512_t_init(&s->state, s->t_string, s->t_length);
    } else if (initial) {
        esp32p4_sha_init(s->mode, &s->state);
    } else {
        esp32p4_sha_state_from_bytes(s->mode, &s->state, s->h);
    }
    s->op_kind = kind;
    s->busy = true;
    if (kind == OP_START || kind == OP_CONTINUE) {
        memcpy(s->in_buf, s->m, esp32p4_sha_block_size(s->op_mode));
    }
    if (kind == OP_DMA) {
        s->left = s->dma_block_num;
        s->in_len = 0;
        sha_pump(s);
    } else {
        esp32p4_crypto_timer_start(&s->op, sha_block_cycles(
                                   kind == OP_T_INIT ? ESP32P4_SHA512
                                                     : s->mode));
    }
}

static uint64_t sha_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4ShaDevState *s = opaque;
    hwaddr reg = addr & ~3;
    uint32_t v = 0;

    esp32p4_crypto_timer_sync(&s->op, esp32_line_time());
    if (s->occupied) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read at 0x%" HWADDR_PRIx
                      " while another block occupies the accelerator\n",
                      TYPE_ESP32P4_SHA, addr);
        return 0;
    }
    switch (reg) {
    case A_MODE:
        v = s->mode;
        break;
    case A_T_STRING:
        v = s->t_string;
        break;
    case A_T_LENGTH:
        v = s->t_length;
        break;
    case A_DMA_BLOCK_NUM:
        v = s->dma_block_num;
        break;
    case A_BUSY:
        v = s->busy;
        break;
    case A_IRQ_ENA:
        v = s->irq_ena;
        break;
    case A_DATE:
        v = s->date;
        break;
    case A_H_MEM ... A_M_MEM - 1:
        v = ldl_le_p(s->h + reg - A_H_MEM);
        break;
    case A_M_MEM ... SHA_REGS_END - 1:
        v = ldl_le_p(s->m + reg - A_M_MEM);
        break;
    case A_START:
    case A_CONTINUE:
    case A_DMA_START:
    case A_DMA_CONTINUE:
    case A_CLEAR_IRQ:
    case A_DMA_RX_RESET:
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of unknown offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_SHA, addr);
        return 0;
    }
    return extract32(v, (addr & 3) * 8, size * 8);
}

/* [spec:nuos:req:emu.esp32p4.crypto] */
static MemTxResult sha_write(void *opaque, hwaddr addr, uint64_t value,
                             unsigned size, MemTxAttrs attrs)
{
    ESP32P4ShaDevState *s = opaque;
    hwaddr reg = addr & ~3;
    int shift = (addr & 3) * 8;
    uint32_t v = deposit32(0, shift, size * 8, value);

    esp32p4_crypto_timer_sync(&s->op, esp32_line_time());
    if (s->occupied || sha_in_reset(s) || !clock_is_enabled(s->clk)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write at 0x%" HWADDR_PRIx
                      " dropped: the accelerator is %s\n", TYPE_ESP32P4_SHA,
                      addr, s->occupied ? "occupied by another block"
                      : sha_in_reset(s) ? "held in reset"
                      : "not clocked (CRYPTO_SHA_CLK)");
        return MEMTX_OK;
    }
    if (s->busy && (v & 1) && (reg == A_START || reg == A_CONTINUE ||
                               reg == A_DMA_START || reg == A_DMA_CONTINUE)) {
        ESP32P4CPU *cpu = esp32p4_cpu_bus_waiter(attrs);

        if (cpu) {
            s->waiters |= BIT(CPU(cpu)->cpu_index);
            return esp32p4_cpu_bus_hold(cpu);
        }
    }
    if (s->busy && reg != A_CLEAR_IRQ && reg != A_IRQ_ENA &&
        reg != A_MODE && !(reg >= A_M_MEM && reg < SHA_REGS_END)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write at 0x%" HWADDR_PRIx
                      " while the accelerator is busy is dropped\n",
                      TYPE_ESP32P4_SHA, addr);
        return MEMTX_OK;
    }

#define MERGE(old) deposit32((old), shift, size * 8, value)
    switch (reg) {
    case A_MODE:
        s->mode = MERGE(s->mode) & 7;
        break;
    case A_T_STRING:
        s->t_string = MERGE(s->t_string);
        break;
    case A_T_LENGTH:
        s->t_length = MERGE(s->t_length) & 0x7f;
        break;
    case A_DMA_BLOCK_NUM:
        s->dma_block_num = MERGE(s->dma_block_num) & 0xffff;
        break;
    case A_START:
        if (v & 1) {
            sha_begin(s, OP_START, true);
        }
        break;
    case A_CONTINUE:
        if (v & 1) {
            sha_begin(s, OP_CONTINUE, false);
        }
        break;
    case A_DMA_START:
        if (v & 1) {
            sha_begin(s, OP_DMA, true);
        }
        break;
    case A_DMA_CONTINUE:
        if (v & 1) {
            sha_begin(s, OP_DMA, false);
        }
        break;
    case A_CLEAR_IRQ:
        if (v & 1) {
            s->done_int = false;
            sha_update_irq(s);
        }
        break;
    case A_IRQ_ENA:
        s->irq_ena = MERGE(s->irq_ena) & 1;
        sha_update_irq(s);
        break;
    case A_DATE:
        s->date = MERGE(s->date) & 0x3fffffff;
        break;
    case A_DMA_RX_RESET:
        if (v & 1) {
            s->in_len = 0;
        }
        break;
    case A_H_MEM ... A_M_MEM - 1:
        stl_le_p(s->h + reg - A_H_MEM,
                 MERGE(ldl_le_p(s->h + reg - A_H_MEM)));
        break;
    case A_M_MEM ... SHA_REGS_END - 1:
        stl_le_p(s->m + reg - A_M_MEM,
                 MERGE(ldl_le_p(s->m + reg - A_M_MEM)));
        break;
    case A_BUSY:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_SHA, addr);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to unknown offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_SHA, addr);
        break;
    }
#undef MERGE
    return MEMTX_OK;
}

static const MemoryRegionOps sha_ops = {
    .read = sha_read,
    .write_with_attrs = sha_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

void esp32p4_sha_occupy(ESP32P4ShaDevState *s, bool occupied)
{
    if (s->occupied && !occupied) {
        s->occupied = false;
        device_cold_reset(DEVICE(s));
        return;
    }
    s->occupied = occupied;
}

/* [spec:nuos:req:emu.esp32p4.crypto] */
static void sha_reset_line(void *opaque, int n, int level)
{
    ESP32P4ShaDevState *s = opaque;
    bool was = sha_in_reset(s);

    s->reset_in[n] = level;
    if (was != sha_in_reset(s)) {
        device_cold_reset(DEVICE(s));
    }
}

static void sha_clk_update(void *opaque, ClockEvent event)
{
    ESP32P4ShaDevState *s = opaque;

    esp32p4_crypto_timer_clock_update(&s->op);
    sha_pump(s);
}

static void sha_reset_hold(Object *obj, ResetType type)
{
    ESP32P4ShaDevState *s = ESP32P4_SHA(obj);

    esp32p4_crypto_timer_cancel(&s->op);
    sha_release_waiters(s);
    s->mode = 0;
    s->t_string = 0;
    s->t_length = 0;
    s->dma_block_num = 0;
    s->irq_ena = 0;
    s->date = DATE_RESET;
    memset(s->h, 0, sizeof(s->h));
    memset(s->m, 0, sizeof(s->m));
    s->busy = false;
    s->done_int = false;
    s->op_kind = OP_NONE;
    s->op_mode = 0;
    s->left = 0;
    s->in_len = 0;
    memset(&s->state, 0, sizeof(s->state));
}

static void sha_reset_exit(Object *obj, ResetType type)
{
    sha_update_irq(ESP32P4_SHA(obj));
}

static void sha_init(Object *obj)
{
    ESP32P4ShaDevState *s = ESP32P4_SHA(obj);

    memory_region_init_io(&s->iomem, obj, &sha_ops, s, "esp32p4.sha",
                          ESP32P4_SHA_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->clk = qdev_init_clock_in(DEVICE(obj), "clk", sha_clk_update, s,
                                ClockUpdate);
    qdev_init_gpio_in_named(DEVICE(obj), sha_reset_line, ESP32P4_SHA_RESET,
                            ESP32P4_SHA_RESETS);
}

static void sha_realize(DeviceState *dev, Error **errp)
{
    ESP32P4ShaDevState *s = ESP32P4_SHA(dev);

    if (!s->gdma) {
        error_setg(errp, "%s: the 'gdma' link is not set", TYPE_ESP32P4_SHA);
        return;
    }
    esp32p4_crypto_timer_init(&s->op, s->clk, sha_op_done, s);
    esp32p4_gdma_attach(s->gdma, ESP32P4_GDMA_AXI_SHA, &sha_gdma_ops, s);
}

static const Property sha_properties[] = {
    DEFINE_PROP_LINK("gdma", ESP32P4ShaDevState, gdma, TYPE_ESP32P4_GDMA,
                     ESP32P4GdmaState *),
};

static const VMStateDescription vmstate_esp32p4_sha_state = {
    .name = "esp32p4-sha-state",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT64_ARRAY(h, ESP32P4ShaState, 8),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_esp32p4_sha = {
    .name = TYPE_ESP32P4_SHA,
    .version_id = 3,
    .minimum_version_id = 3,
    .fields = (const VMStateField[]) {
        VMSTATE_ESP32P4_CRYPTO_TIMER(op, ESP32P4ShaDevState),
        VMSTATE_BOOL_ARRAY(reset_in, ESP32P4ShaDevState, ESP32P4_SHA_RESETS),
        VMSTATE_BOOL(occupied, ESP32P4ShaDevState),
        VMSTATE_UINT32(mode, ESP32P4ShaDevState),
        VMSTATE_UINT32(t_string, ESP32P4ShaDevState),
        VMSTATE_UINT32(t_length, ESP32P4ShaDevState),
        VMSTATE_UINT32(dma_block_num, ESP32P4ShaDevState),
        VMSTATE_UINT32(irq_ena, ESP32P4ShaDevState),
        VMSTATE_UINT32(date, ESP32P4ShaDevState),
        VMSTATE_UINT8_ARRAY(h, ESP32P4ShaDevState, 64),
        VMSTATE_UINT8_ARRAY(m, ESP32P4ShaDevState, 128),
        VMSTATE_BOOL(busy, ESP32P4ShaDevState),
        VMSTATE_BOOL(done_int, ESP32P4ShaDevState),
        VMSTATE_UINT32(op_kind, ESP32P4ShaDevState),
        VMSTATE_UINT32(op_mode, ESP32P4ShaDevState),
        VMSTATE_UINT32(left, ESP32P4ShaDevState),
        VMSTATE_UINT8_ARRAY(in_buf, ESP32P4ShaDevState, 128),
        VMSTATE_UINT32(in_len, ESP32P4ShaDevState),
        VMSTATE_UINT32(waiters, ESP32P4ShaDevState),
        VMSTATE_STRUCT(state, ESP32P4ShaDevState, 1,
                       vmstate_esp32p4_sha_state, ESP32P4ShaState),
        VMSTATE_END_OF_LIST()
    },
};

static void sha_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = sha_realize;
    dc->vmsd = &vmstate_esp32p4_sha;
    device_class_set_props(dc, sha_properties);
    rc->phases.hold = sha_reset_hold;
    rc->phases.exit = sha_reset_exit;
}

/* [spec:nuos:req:emu.esp32p4.crypto] */
static const TypeInfo sha_info = {
    .name          = TYPE_ESP32P4_SHA,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32P4ShaDevState),
    .instance_init = sha_init,
    .class_init    = sha_class_init,
};

static void sha_register_types(void)
{
    type_register_static(&sha_info);
}
type_init(sha_register_types)
