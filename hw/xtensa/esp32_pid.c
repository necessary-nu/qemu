/*
 * ESP32 process ID controller (PIDCTRL)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

/*
 * Each CPU has a PID controller (TRM chapter 13) that tells the MMUs and
 * MPUs which of eight processes the CPU is running. Fetching from an
 * enabled interrupt vector entry address switches the CPU to PID 0 and
 * records the interrupt level and the previous level and PID. Software
 * running as PID 0 or 1 switches to another process by writing
 * PIDCTRL_PID_NEW and PIDCTRL_PID_CONFIRM; the new PID takes effect
 * PIDCTRL_PID_DELAY cycles later, so that it can apply from the new
 * process's first instruction. The controller can also mask the CPU's NMI,
 * and unmask it PIDCTRL_NMI_DELAY cycles after a request.
 *
 * The model counts the delays in instructions: the core runs a countdown
 * from the instruction after the write, and the change applies from the
 * instruction after the delay has run out. The LX6 retires most
 * instructions in one cycle; software that times a switch around slower
 * instructions sees the switch up to their extra cycles later than on
 * the hardware.
 *
 * Fetches are recognised when the core vectors to an interrupt or
 * exception; a branch to a vector entry address is not.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "exec/cputlb.h"
#include "hw/xtensa/esp32_pid.h"

/* Reset values from the TRM, registers 13.2 to 13.10 */
static const uint32_t esp32_pid_vector_reset[ESP32_PID_LEVELS] = {
    0x40000340, 0x40000180, 0x400001c0, 0x40000200,
    0x40000240, 0x40000280, 0x400002c0,
};
#define PID_DELAY_RESET     20
#define NMI_DELAY_RESET     16

#define INT_ENABLE_MASK     0xfe
#define DELAY_MASK          0xfff
#define LEVEL_MASK          0xf
#define FROM_MASK           0x7f
#define PID_MASK            0x7

unsigned esp32_pid_current(Esp32PidState *s)
{
    return s->pid;
}

static void esp32_pid_set(Esp32PidState *s, uint32_t pid)
{
    if (pid != s->pid) {
        s->pid = pid;
        /* The TLB holds the previous process's mappings. */
        tlb_flush(CPU(s->cpu));
    }
}

/*
 * [spec:nuos:req:emu.esp32.memory-protection]
 * The CPU fetches from pc as it vectors: an enabled interrupt vector entry
 * address switches it to PID 0 (TRM 13.3.1 and 13.3.2).
 */
void esp32_pid_vector(Esp32PidState *s, uint32_t pc)
{
    for (unsigned n = 1; n <= ESP32_PID_LEVELS; ++n) {
        if ((s->int_enable & BIT(n)) && s->int_addr[n - 1] == pc) {
            s->from[n - 1] = (s->level << 3) | s->pid;
            s->level = n;
            esp32_pid_set(s, 0);
            return;
        }
    }
}

/* [spec:nuos:req:emu.esp32.memory-protection] */
void esp32_pid_countdown_expired(Esp32PidState *s, unsigned n)
{
    switch (n) {
    case ESP32_PID_COUNTDOWN_PID:
        esp32_pid_set(s, s->pid_new);
        break;
    case ESP32_PID_COUNTDOWN_NMI:
        s->nmi_masked = false;
        xtensa_set_nmi_masked(&s->cpu->env, false);
        break;
    }
}

static uint64_t esp32_pid_read(void *opaque, hwaddr addr, unsigned size)
{
    Esp32PidState *s = ESP32_PID(opaque);

    switch (addr) {
    case A_PIDCTRL_INTERRUPT_ENABLE:
        return s->int_enable;
    case A_PIDCTRL_INTERRUPT_ADDR_1 ... A_PIDCTRL_INTERRUPT_ADDR_7:
        return s->int_addr[(addr - A_PIDCTRL_INTERRUPT_ADDR_1) / 4];
    case A_PIDCTRL_PID_DELAY:
        return s->pid_delay;
    case A_PIDCTRL_NMI_DELAY:
        return s->nmi_delay;
    case A_PIDCTRL_LEVEL:
        return s->level;
    case A_PIDCTRL_FROM_1 ... A_PIDCTRL_FROM_7:
        return s->from[(addr - A_PIDCTRL_FROM_1) / 4];
    case A_PIDCTRL_PID_NEW:
        return s->pid_new;
    case A_PIDCTRL_PID_CONFIRM:
    case A_PIDCTRL_NMI_MASK_ENABLE:
    case A_PIDCTRL_NMI_MASK_DISABLE:
        /* Write-only */
        return 0;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_pid: read of reserved offset 0x%" HWADDR_PRIx
                      "\n", addr);
        return 0;
    }
}

/* [spec:nuos:req:emu.esp32.memory-protection] */
static void esp32_pid_write(void *opaque, hwaddr addr, uint64_t value,
                            unsigned size)
{
    Esp32PidState *s = ESP32_PID(opaque);
    CPUXtensaState *env = &s->cpu->env;

    switch (addr) {
    case A_PIDCTRL_INTERRUPT_ENABLE:
        s->int_enable = value & INT_ENABLE_MASK;
        break;
    case A_PIDCTRL_INTERRUPT_ADDR_1 ... A_PIDCTRL_INTERRUPT_ADDR_7:
        s->int_addr[(addr - A_PIDCTRL_INTERRUPT_ADDR_1) / 4] = value;
        break;
    case A_PIDCTRL_PID_DELAY:
        s->pid_delay = value & DELAY_MASK;
        break;
    case A_PIDCTRL_NMI_DELAY:
        s->nmi_delay = value & DELAY_MASK;
        break;
    case A_PIDCTRL_LEVEL:
        s->level = value & LEVEL_MASK;
        break;
    case A_PIDCTRL_FROM_1 ... A_PIDCTRL_FROM_7:
        s->from[(addr - A_PIDCTRL_FROM_1) / 4] = value & FROM_MASK;
        break;
    case A_PIDCTRL_PID_NEW:
        s->pid_new = value & PID_MASK;
        break;
    case A_PIDCTRL_PID_CONFIRM:
        if (value & 1) {
            xtensa_ext_countdown_start(env, ESP32_PID_COUNTDOWN_PID,
                                       s->pid_delay);
        }
        break;
    case A_PIDCTRL_NMI_MASK_ENABLE:
        if (value & 1) {
            xtensa_ext_countdown_cancel(env, ESP32_PID_COUNTDOWN_NMI);
            s->nmi_masked = true;
            xtensa_set_nmi_masked(env, true);
        }
        break;
    case A_PIDCTRL_NMI_MASK_DISABLE:
        if (value & 1) {
            xtensa_ext_countdown_start(env, ESP32_PID_COUNTDOWN_NMI,
                                       s->nmi_delay);
        }
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_pid: write of reserved offset 0x%" HWADDR_PRIx
                      "\n", addr);
        break;
    }
}

static const MemoryRegionOps esp32_pid_ops = {
    .read = esp32_pid_read,
    .write = esp32_pid_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void esp32_pid_reset_hold(Object *obj, ResetType type)
{
    Esp32PidState *s = ESP32_PID(obj);
    CPUXtensaState *env = &s->cpu->env;

    s->int_enable = 0;
    memcpy(s->int_addr, esp32_pid_vector_reset, sizeof(s->int_addr));
    s->pid_delay = PID_DELAY_RESET;
    s->nmi_delay = NMI_DELAY_RESET;
    s->level = 0;
    memset(s->from, 0, sizeof(s->from));
    s->pid_new = 0;
    esp32_pid_set(s, 0);
    s->nmi_masked = false;
    xtensa_ext_countdown_cancel(env, ESP32_PID_COUNTDOWN_PID);
    xtensa_ext_countdown_cancel(env, ESP32_PID_COUNTDOWN_NMI);
    env->nmi_masked = false;
}

static void esp32_pid_realize(DeviceState *dev, Error **errp)
{
    Esp32PidState *s = ESP32_PID(dev);

    if (!s->cpu) {
        error_setg(errp, "esp32_pid: 'cpu' link not set");
    }
}

static void esp32_pid_init(Object *obj)
{
    Esp32PidState *s = ESP32_PID(obj);

    memory_region_init_io(&s->iomem, obj, &esp32_pid_ops, s, TYPE_ESP32_PID,
                          ESP32_PID_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    /* No register reports the current PID; expose it for inspection. */
    object_property_add_uint32_ptr(obj, "pid", &s->pid, OBJ_PROP_FLAG_READ);
}

static const VMStateDescription vmstate_esp32_pid = {
    .name = TYPE_ESP32_PID,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(int_enable, Esp32PidState),
        VMSTATE_UINT32_ARRAY(int_addr, Esp32PidState, ESP32_PID_LEVELS),
        VMSTATE_UINT32(pid_delay, Esp32PidState),
        VMSTATE_UINT32(nmi_delay, Esp32PidState),
        VMSTATE_UINT32(level, Esp32PidState),
        VMSTATE_UINT32_ARRAY(from, Esp32PidState, ESP32_PID_LEVELS),
        VMSTATE_UINT32(pid_new, Esp32PidState),
        VMSTATE_UINT32(pid, Esp32PidState),
        VMSTATE_BOOL(nmi_masked, Esp32PidState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property esp32_pid_properties[] = {
    DEFINE_PROP_LINK("cpu", Esp32PidState, cpu, TYPE_XTENSA_CPU, XtensaCPU *),
};

static void esp32_pid_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = esp32_pid_realize;
    dc->vmsd = &vmstate_esp32_pid;
    rc->phases.hold = esp32_pid_reset_hold;
    device_class_set_props(dc, esp32_pid_properties);
}

/* [spec:nuos:req:emu.esp32.memory-protection] */
static const TypeInfo esp32_pid_info = {
    .name = TYPE_ESP32_PID,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32PidState),
    .instance_init = esp32_pid_init,
    .class_init = esp32_pid_class_init,
};

static void esp32_pid_register_types(void)
{
    type_register_static(&esp32_pid_info);
}

type_init(esp32_pid_register_types)
