/*
 * AP Memory APS6404L QSPI PSRAM
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: APS6404L-3SQR datasheet, AP Memory. An 8 MiB pseudo-SRAM
 * with a SPI/QPI command interface, the PSRAM fitted on CS1 of many
 * RP2350 boards. The model works at byte level, like QEMU's other SSI
 * devices: interface width is the controller's business, and wait cycles
 * are counted in the bytes they occupy at the width the command uses.
 *
 * Commands modelled, in SPI mode: 03h read, 0Bh fast read, EBh quad read,
 * 02h write, 38h quad write, 9Fh read ID, 35h enter QPI mode, 66h/99h
 * reset, C0h wrap boundary toggle. In QPI mode 03h, 9Fh and 35h are not
 * accepted, and F5h leaves QPI mode. Refresh and timing limits (tCEM,
 * page-crossing frequency limits) are not modelled: the array never loses
 * data and accepts any burst length.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/block/aps6404l.h"
#include "hw/core/qdev-properties.h"
#include "hw/ssi/ssi.h"
#include "migration/vmstate.h"
#include "system/memory.h"
#include "qom/object.h"

OBJECT_DECLARE_SIMPLE_TYPE(APS6404LState, APS6404L)

#define CMD_READ        0x03
#define CMD_FAST_READ   0x0b
#define CMD_QUAD_READ   0xeb
#define CMD_WRITE       0x02
#define CMD_QUAD_WRITE  0x38
#define CMD_READ_ID     0x9f
#define CMD_ENTER_QPI   0x35
#define CMD_EXIT_QPI    0xf5
#define CMD_RESET_EN    0x66
#define CMD_RESET       0x99
#define CMD_WRAP_TOGGLE 0xc0

/* Read ID: manufacturer, known-good-die mark, then the 48-bit EID. */
static const uint8_t aps6404l_id[] = {
    0x0d, 0x5d, 0x26, 0x00, 0x00, 0x00, 0x00, 0x00,
};

/* Wrapped bursts stay inside one 32-byte block. */
#define WRAP_SIZE 32

typedef enum {
    STATE_IDLE,
    STATE_ADDR,
    STATE_DUMMY,
    STATE_READ,
    STATE_WRITE,
    STATE_ID,
    STATE_IGNORE,
} APS6404LPhase;

struct APS6404LState {
    SSIPeripheral parent_obj;

    /* The array: a linked ROM device region, or our own RAM. */
    MemoryRegion *mem;
    MemoryRegion ram;
    uint8_t *storage;

    uint8_t state;
    uint8_t cmd;
    uint8_t count;
    uint32_t addr;
    bool qpi;
    bool wrap32;
    bool reset_enable;
};

static void aps6404l_mode_reset(APS6404LState *s)
{
    s->qpi = false;
    s->wrap32 = false;
    s->reset_enable = false;
}

/* Bytes of wait cycles before read data, at the command's width. */
static uint8_t aps6404l_wait_bytes(APS6404LState *s)
{
    switch (s->cmd) {
    case CMD_FAST_READ:
        /* 8 wait cycles serially, 4 at quad width. */
        return s->qpi ? 2 : 1;
    case CMD_QUAD_READ:
        /* 6 wait cycles at quad width. */
        return 3;
    default:
        return 0;
    }
}

static void aps6404l_next_addr(APS6404LState *s)
{
    if (s->wrap32) {
        s->addr = (s->addr & ~(WRAP_SIZE - 1)) |
                  ((s->addr + 1) & (WRAP_SIZE - 1));
    } else {
        s->addr = (s->addr + 1) & (APS6404L_SIZE - 1);
    }
}

static void aps6404l_decode(APS6404LState *s, uint8_t cmd)
{
    bool was_reset_enable = s->reset_enable;

    s->cmd = cmd;
    s->reset_enable = false;
    s->state = STATE_IGNORE;

    switch (cmd) {
    case CMD_READ:
    case CMD_READ_ID:
        if (!s->qpi) {
            s->state = STATE_ADDR;
            s->count = 0;
        }
        break;
    case CMD_FAST_READ:
    case CMD_QUAD_READ:
    case CMD_WRITE:
    case CMD_QUAD_WRITE:
        s->state = STATE_ADDR;
        s->count = 0;
        break;
    case CMD_ENTER_QPI:
        if (!s->qpi) {
            s->qpi = true;
        }
        break;
    case CMD_EXIT_QPI:
        if (s->qpi) {
            s->qpi = false;
        }
        break;
    case CMD_RESET_EN:
        s->reset_enable = true;
        break;
    case CMD_RESET:
        if (was_reset_enable) {
            aps6404l_mode_reset(s);
        }
        break;
    case CMD_WRAP_TOGGLE:
        s->wrap32 = !s->wrap32;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "aps6404l: unknown command 0x%02x in "
                      "%s mode\n", cmd, s->qpi ? "QPI" : "SPI");
        break;
    }
    if (s->state == STATE_IGNORE && s->qpi &&
        (cmd == CMD_READ || cmd == CMD_READ_ID)) {
        qemu_log_mask(LOG_GUEST_ERROR, "aps6404l: command 0x%02x is SPI "
                      "mode only\n", cmd);
    }
}

static uint32_t aps6404l_transfer(SSIPeripheral *ss, uint32_t tx)
{
    APS6404LState *s = APS6404L(ss);
    uint32_t r = 0;

    switch (s->state) {
    case STATE_IDLE:
        aps6404l_decode(s, tx);
        break;
    case STATE_ADDR:
        s->addr = (s->addr << 8) | (tx & 0xff);
        if (++s->count < 3) {
            break;
        }
        s->addr &= APS6404L_SIZE - 1;
        s->count = 0;
        switch (s->cmd) {
        case CMD_WRITE:
        case CMD_QUAD_WRITE:
            s->state = STATE_WRITE;
            break;
        case CMD_READ_ID:
            s->state = STATE_ID;
            break;
        default:
            s->count = aps6404l_wait_bytes(s);
            s->state = s->count ? STATE_DUMMY : STATE_READ;
            break;
        }
        break;
    case STATE_DUMMY:
        if (--s->count == 0) {
            s->state = STATE_READ;
        }
        break;
    case STATE_READ:
        r = s->storage[s->addr];
        aps6404l_next_addr(s);
        break;
    case STATE_WRITE:
        s->storage[s->addr] = tx;
        if (s->mem) {
            memory_region_flush_rom_device(s->mem, s->addr, 1);
        } else {
            memory_region_set_dirty(&s->ram, s->addr, 1);
        }
        aps6404l_next_addr(s);
        break;
    case STATE_ID:
        r = aps6404l_id[s->count];
        s->count = (s->count + 1) % sizeof(aps6404l_id);
        break;
    case STATE_IGNORE:
        break;
    }
    return r;
}

static int aps6404l_set_cs(SSIPeripheral *ss, bool deselect)
{
    APS6404LState *s = APS6404L(ss);

    /* Every command ends when the chip select rises. */
    if (deselect) {
        s->state = STATE_IDLE;
        s->count = 0;
    }
    return 0;
}

static void aps6404l_realize(SSIPeripheral *ss, Error **errp)
{
    APS6404LState *s = APS6404L(ss);

    if (s->mem) {
        if (!memory_region_is_romd(s->mem) ||
            memory_region_size(s->mem) != APS6404L_SIZE) {
            error_setg(errp, "memory must be a ROM device region of %u bytes",
                       (unsigned)APS6404L_SIZE);
            return;
        }
        s->storage = memory_region_get_ram_ptr(s->mem);
    } else {
        if (!memory_region_init_ram(&s->ram, OBJECT(s), "aps6404l.array",
                                    APS6404L_SIZE, errp)) {
            return;
        }
        s->storage = memory_region_get_ram_ptr(&s->ram);
    }
}

static void aps6404l_reset_hold(Object *obj, ResetType type)
{
    APS6404LState *s = APS6404L(obj);

    /* Power-on reset: SPI mode, linear bursts. The array keeps its data. */
    aps6404l_mode_reset(s);
    s->state = STATE_IDLE;
    s->count = 0;
    s->addr = 0;
}

static const VMStateDescription vmstate_aps6404l = {
    .name = TYPE_APS6404L,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8(state, APS6404LState),
        VMSTATE_UINT8(cmd, APS6404LState),
        VMSTATE_UINT8(count, APS6404LState),
        VMSTATE_UINT32(addr, APS6404LState),
        VMSTATE_BOOL(qpi, APS6404LState),
        VMSTATE_BOOL(wrap32, APS6404LState),
        VMSTATE_BOOL(reset_enable, APS6404LState),
        VMSTATE_END_OF_LIST()
    },
};

static const Property aps6404l_properties[] = {
    DEFINE_PROP_LINK("memory", APS6404LState, mem, TYPE_MEMORY_REGION,
                     MemoryRegion *),
};

static void aps6404l_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SSIPeripheralClass *k = SSI_PERIPHERAL_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    k->realize = aps6404l_realize;
    k->transfer = aps6404l_transfer;
    k->set_cs = aps6404l_set_cs;
    k->cs_polarity = SSI_CS_LOW;
    rc->phases.hold = aps6404l_reset_hold;
    dc->vmsd = &vmstate_aps6404l;
    device_class_set_props(dc, aps6404l_properties);
}

static const TypeInfo aps6404l_info = {
    .name          = TYPE_APS6404L,
    .parent        = TYPE_SSI_PERIPHERAL,
    .instance_size = sizeof(APS6404LState),
    .class_init    = aps6404l_class_init,
};

static void aps6404l_register_types(void)
{
    type_register_static(&aps6404l_info);
}
type_init(aps6404l_register_types)
