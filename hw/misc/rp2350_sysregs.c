/*
 * RP2350 system register blocks (SYSINFO, SYSCFG, TBMAN, glitch detector,
 * DFT)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: RP2350 Datasheet, "SYSINFO", "SYSCFG", "TBMAN" and "Glitch
 * detector"; register layouts from the pico-sdk hardware_regs headers.
 * All five are APB blocks with the atomic XOR/SET/CLR aliases. Access
 * permissions for SYSINFO, SYSCFG and TBMAN are applied by the ACCESSCTRL
 * bus filter in front of them; the glitch detector has no ACCESSCTRL
 * register and is hardwired to Secure access only.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/rp2350_atomic.h"
#include "hw/misc/rp2350_sysregs.h"
#include "migration/vmstate.h"

/* SYSINFO */
#define A_SYSINFO_CHIP_ID       0x00
#define A_SYSINFO_PACKAGE_SEL   0x04
#define A_SYSINFO_PLATFORM      0x08
#define A_SYSINFO_GITREF        0x14

/*
 * JEP-106 identifier: REVISION 3, PART 0x0004, MANUFACTURER 0x493 and the
 * stop bit. The A4 stepping changed only the boot ROM and kept the A3
 * silicon revision, so A4 parts report revision 3; the stepping is told
 * apart by the boot ROM version byte at 0x13.
 */
#define SYSINFO_CHIP_ID         0x30004927
/* Production silicon: ASIC, not FPGA or any simulation. */
#define SYSINFO_PLATFORM_ASIC   (1u << 1)

/* SYSCFG */
#define A_PROC_CONFIG               0x00
#define A_PROC_IN_SYNC_BYPASS       0x04
#define A_PROC_IN_SYNC_BYPASS_HI    0x08
#define A_DBGFORCE                  0x0c
#define A_MEMPOWERDOWN              0x10
#define A_AUXCTRL                   0x14

#define PROC_IN_SYNC_BYPASS_HI_MASK 0xff00ffff
#define DBGFORCE_ATTACH             (1u << 3)
#define DBGFORCE_RW_MASK            0x0000000e
#define DBGFORCE_RESET              0x00000006
#define MEMPOWERDOWN_MASK           0x00001fff
#define AUXCTRL_MASK                0x000000ff

/* TBMAN */
#define A_TBMAN_PLATFORM        0x00
#define TBMAN_PLATFORM_ASIC     (1u << 0)

/* Glitch detector */
#define A_GD_ARM                0x00
#define A_GD_DISARM             0x04
#define A_GD_SENSITIVITY        0x08
#define A_GD_LOCK               0x0c
#define A_GD_TRIG_STATUS        0x10
#define A_GD_TRIG_FORCE         0x14

#define GD_ARM_NO               0x5bad
#define GD_DISARM_YES           0xdcaf
#define GD_HALF_MASK            0x0000ffff
#define GD_SENSITIVITY_MASK     0xff00ffff
#define GD_LOCK_MASK            0x000000ff
#define GD_DET_MASK             0x0000000f

/* [spec:nuos:req:emu.system-regs] */
static uint64_t rp2350_sysinfo_read(void *opaque, hwaddr addr, unsigned size)
{
    RP2350SysInfoState *s = opaque;

    switch (rp2350_atomic_reg(addr)) {
    case A_SYSINFO_CHIP_ID:
        return SYSINFO_CHIP_ID;
    case A_SYSINFO_PACKAGE_SEL:
        return s->qfn60;
    case A_SYSINFO_PLATFORM:
        return SYSINFO_PLATFORM_ASIC;
    case A_SYSINFO_GITREF:
        return s->gitref;
    }
    qemu_log_mask(LOG_GUEST_ERROR, "rp2350-sysinfo: read of bad offset 0x%"
                  HWADDR_PRIx "\n", addr);
    return 0;
}

/* Every SYSINFO register is read-only; writes are ignored. */
static void rp2350_sysinfo_write(void *opaque, hwaddr addr, uint64_t value,
                                 unsigned size)
{
    switch (rp2350_atomic_reg(addr)) {
    case A_SYSINFO_CHIP_ID:
    case A_SYSINFO_PACKAGE_SEL:
    case A_SYSINFO_PLATFORM:
    case A_SYSINFO_GITREF:
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-sysinfo: write to bad offset "
                      "0x%" HWADDR_PRIx "\n", addr);
        break;
    }
}

static const MemoryRegionOps rp2350_sysinfo_ops = {
    .read = rp2350_sysinfo_read,
    .write = rp2350_sysinfo_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/* [spec:nuos:req:emu.system-regs] */
static uint64_t rp2350_syscfg_read(void *opaque, hwaddr addr, unsigned size)
{
    RP2350SysCfgState *s = opaque;

    switch (rp2350_atomic_reg(addr)) {
    case A_PROC_CONFIG:
        /*
         * PROCn_HALTED reflect a core held in debug halt. The machine has
         * no halting debug a core could observe: a debugger attached to
         * QEMU stops the whole machine.
         */
        return 0;
    case A_PROC_IN_SYNC_BYPASS:
        return s->proc_in_sync_bypass;
    case A_PROC_IN_SYNC_BYPASS_HI:
        return s->proc_in_sync_bypass_hi;
    case A_DBGFORCE:
        /*
         * SWDO observes the debug port's SWDIO output, which the datasheet
         * leaves undefined. No SWD transfer is ever in progress, so the
         * port drives nothing and the bit reads 0.
         */
        return s->dbgforce;
    case A_MEMPOWERDOWN:
        return s->mempowerdown;
    case A_AUXCTRL:
        return s->auxctrl;
    }
    qemu_log_mask(LOG_GUEST_ERROR, "rp2350-syscfg: read of bad offset 0x%"
                  HWADDR_PRIx "\n", addr);
    return 0;
}

static void rp2350_syscfg_write(void *opaque, hwaddr addr, uint64_t value,
                                unsigned size)
{
    RP2350SysCfgState *s = opaque;
    uint32_t v;

    switch (rp2350_atomic_reg(addr)) {
    case A_PROC_CONFIG:
        break;
    case A_PROC_IN_SYNC_BYPASS:
        /*
         * Bypassing the GPIO input synchronisers only saves two cycles of
         * input latency, which the GPIO model does not have.
         */
        s->proc_in_sync_bypass =
            rp2350_atomic_apply(addr, s->proc_in_sync_bypass, value);
        break;
    case A_PROC_IN_SYNC_BYPASS_HI:
        s->proc_in_sync_bypass_hi =
            rp2350_atomic_apply(addr, s->proc_in_sync_bypass_hi, value) &
            PROC_IN_SYNC_BYPASS_HI_MASK;
        break;
    case A_DBGFORCE:
        v = rp2350_atomic_apply(addr, s->dbgforce, value) & DBGFORCE_RW_MASK;
        if ((v & DBGFORCE_ATTACH) && !(s->dbgforce & DBGFORCE_ATTACH)) {
            qemu_log_mask(LOG_UNIMP, "rp2350-syscfg: DBGFORCE.ATTACH: "
                          "driving the SWD debug port from software is not "
                          "modelled\n");
        }
        s->dbgforce = v;
        break;
    case A_MEMPOWERDOWN:
        v = rp2350_atomic_apply(addr, s->mempowerdown, value) &
            MEMPOWERDOWN_MASK;
        /*
         * A powered-down memory keeps its contents but cannot be
         * accessed; the datasheet does not say what an access returns.
         */
        if (v & ~s->mempowerdown) {
            qemu_log_mask(LOG_UNIMP, "rp2350-syscfg: MEMPOWERDOWN 0x%04x: "
                          "memories stay accessible while powered down\n", v);
        }
        s->mempowerdown = v;
        break;
    case A_AUXCTRL:
        /*
         * The defined bits steer the POWMAN clock across a watchdog reset,
         * the LPOSC contribution to TRNG entropy and the OTP supply
         * detector, none of which software can observe in the machine.
         */
        s->auxctrl = rp2350_atomic_apply(addr, s->auxctrl, value) &
                     AUXCTRL_MASK;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-syscfg: write to bad offset "
                      "0x%" HWADDR_PRIx "\n", addr);
        break;
    }
}

static const MemoryRegionOps rp2350_syscfg_ops = {
    .read = rp2350_syscfg_read,
    .write = rp2350_syscfg_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/* [spec:nuos:req:emu.system-regs] */
static uint64_t rp2350_tbman_read(void *opaque, hwaddr addr, unsigned size)
{
    if (rp2350_atomic_reg(addr) == A_TBMAN_PLATFORM) {
        return TBMAN_PLATFORM_ASIC;
    }
    qemu_log_mask(LOG_GUEST_ERROR, "rp2350-tbman: read of bad offset 0x%"
                  HWADDR_PRIx "\n", addr);
    return 0;
}

static void rp2350_tbman_write(void *opaque, hwaddr addr, uint64_t value,
                               unsigned size)
{
    if (rp2350_atomic_reg(addr) != A_TBMAN_PLATFORM) {
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-tbman: write to bad offset "
                      "0x%" HWADDR_PRIx "\n", addr);
    }
}

static const MemoryRegionOps rp2350_tbman_ops = {
    .read = rp2350_tbman_read,
    .write = rp2350_tbman_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/*
 * DISARM is ignored while ARM holds an arming pattern; otherwise the OTP
 * enable arms the detectors unless DISARM holds the disarming pattern.
 */
static bool glitch_detector_armed(RP2350GlitchDetectorState *s)
{
    if (s->arm != GD_ARM_NO) {
        return true;
    }
    return s->otp_enable && s->disarm != GD_DISARM_YES;
}

/*
 * The detectors themselves are never triggered: the machine has no clock
 * or supply glitches. TRIG_FORCE simulates a trigger as hardware does.
 */
static void glitch_detector_trigger(RP2350GlitchDetectorState *s,
                                    uint32_t dets)
{
    s->trig_status |= dets;
    if (glitch_detector_armed(s)) {
        /*
         * An armed trigger is a chip-level reset through the power
         * manager, which records it and resets the PSM and the watchdog.
         * It is not a reset of the whole switched core: the detector's own
         * registers are outside the PSM's reset domain, so TRIG_STATUS
         * still shows which detector fired, and ARM, DISARM, SENSITIVITY
         * and LOCK keep their values.
         */
        qemu_irq_pulse(s->chip_reset);
    }
}

/* [spec:nuos:req:emu.system-regs] */
static MemTxResult rp2350_glitch_detector_read(void *opaque, hwaddr addr,
                                               uint64_t *data, unsigned size,
                                               MemTxAttrs attrs)
{
    RP2350GlitchDetectorState *s = opaque;

    if (!attrs.secure && !attrs.unspecified) {
        return MEMTX_ERROR;
    }
    switch (rp2350_atomic_reg(addr)) {
    case A_GD_ARM:
        *data = s->arm;
        break;
    case A_GD_DISARM:
        *data = s->disarm;
        break;
    case A_GD_SENSITIVITY:
        *data = s->sensitivity;
        break;
    case A_GD_LOCK:
        *data = s->lock;
        break;
    case A_GD_TRIG_STATUS:
        *data = s->trig_status;
        break;
    case A_GD_TRIG_FORCE:
        /* Self-clearing. */
        *data = 0;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-glitch-detector: read of bad "
                      "offset 0x%" HWADDR_PRIx "\n", addr);
        *data = 0;
        break;
    }
    return MEMTX_OK;
}

static MemTxResult rp2350_glitch_detector_write(void *opaque, hwaddr addr,
                                                uint64_t value, unsigned size,
                                                MemTxAttrs attrs)
{
    RP2350GlitchDetectorState *s = opaque;
    hwaddr reg = rp2350_atomic_reg(addr);

    if (!attrs.secure && !attrs.unspecified) {
        return MEMTX_ERROR;
    }
    switch (reg) {
    case A_GD_ARM:
    case A_GD_DISARM:
    case A_GD_SENSITIVITY:
    case A_GD_LOCK:
        /* A nonzero LOCK freezes ARM, DISARM, SENSITIVITY and itself. */
        if (s->lock) {
            break;
        }
        if (reg == A_GD_ARM) {
            s->arm = rp2350_atomic_apply(addr, s->arm, value) & GD_HALF_MASK;
        } else if (reg == A_GD_DISARM) {
            s->disarm = rp2350_atomic_apply(addr, s->disarm, value) &
                        GD_HALF_MASK;
        } else if (reg == A_GD_SENSITIVITY) {
            /*
             * Only the effective delay-line setting depends on the DETn
             * and DETn_INV pairs agreeing, and no detector ever fires, so
             * the register is plain storage.
             */
            s->sensitivity = rp2350_atomic_apply(addr, s->sensitivity, value)
                             & GD_SENSITIVITY_MASK;
        } else {
            s->lock = rp2350_atomic_apply(addr, s->lock, value) &
                      GD_LOCK_MASK;
        }
        break;
    case A_GD_TRIG_STATUS:
        /* Write 1 to clear. */
        s->trig_status &= ~(value & GD_DET_MASK);
        break;
    case A_GD_TRIG_FORCE:
        if (value & GD_DET_MASK) {
            glitch_detector_trigger(s, value & GD_DET_MASK);
        }
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-glitch-detector: write to bad "
                      "offset 0x%" HWADDR_PRIx "\n", addr);
        break;
    }
    return MEMTX_OK;
}

static const MemoryRegionOps rp2350_glitch_detector_ops = {
    .read_with_attrs = rp2350_glitch_detector_read,
    .write_with_attrs = rp2350_glitch_detector_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/*
 * The DFT (design for test) block is used only in production test. The
 * datasheet documents its base address and nothing else, so its
 * registers cannot be modelled.
 */
/* [spec:nuos:req:emu.system-regs] */
static uint64_t rp2350_dft_read(void *opaque, hwaddr addr, unsigned size)
{
    qemu_log_mask(LOG_UNIMP, "rp2350-dft: read of undocumented register 0x%"
                  HWADDR_PRIx "\n", addr);
    return 0;
}

static void rp2350_dft_write(void *opaque, hwaddr addr, uint64_t value,
                             unsigned size)
{
    qemu_log_mask(LOG_UNIMP, "rp2350-dft: write of 0x%08" PRIx64 " to "
                  "undocumented register 0x%" HWADDR_PRIx "\n", value, addr);
}

static const MemoryRegionOps rp2350_dft_ops = {
    .read = rp2350_dft_read,
    .write = rp2350_dft_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void rp2350_sysinfo_init(Object *obj)
{
    RP2350SysInfoState *s = RP2350_SYSINFO(obj);

    memory_region_init_io(&s->iomem, obj, &rp2350_sysinfo_ops, s,
                          TYPE_RP2350_SYSINFO, RP2350_ATOMIC_REGION_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

/*
 * GITREF_RP2350 is the git hash of the chip source, which the datasheet
 * does not give for any stepping; boards that know theirs set it.
 */
static const Property rp2350_sysinfo_properties[] = {
    DEFINE_PROP_BOOL("qfn60", RP2350SysInfoState, qfn60, true),
    DEFINE_PROP_UINT32("gitref", RP2350SysInfoState, gitref, 0),
};

static void rp2350_sysinfo_class_init(ObjectClass *klass, const void *data)
{
    device_class_set_props(DEVICE_CLASS(klass), rp2350_sysinfo_properties);
}

static void rp2350_syscfg_hold_reset(Object *obj, ResetType type)
{
    RP2350SysCfgState *s = RP2350_SYSCFG(obj);

    s->proc_in_sync_bypass = 0;
    s->proc_in_sync_bypass_hi = 0;
    s->dbgforce = DBGFORCE_RESET;
    s->mempowerdown = 0;
    s->auxctrl = 0;
}

static void rp2350_syscfg_init(Object *obj)
{
    RP2350SysCfgState *s = RP2350_SYSCFG(obj);

    memory_region_init_io(&s->iomem, obj, &rp2350_syscfg_ops, s,
                          TYPE_RP2350_SYSCFG, RP2350_ATOMIC_REGION_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static const VMStateDescription vmstate_rp2350_syscfg = {
    .name = TYPE_RP2350_SYSCFG,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(proc_in_sync_bypass, RP2350SysCfgState),
        VMSTATE_UINT32(proc_in_sync_bypass_hi, RP2350SysCfgState),
        VMSTATE_UINT32(dbgforce, RP2350SysCfgState),
        VMSTATE_UINT32(mempowerdown, RP2350SysCfgState),
        VMSTATE_UINT32(auxctrl, RP2350SysCfgState),
        VMSTATE_END_OF_LIST()
    },
};

static void rp2350_syscfg_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = rp2350_syscfg_hold_reset;
    dc->vmsd = &vmstate_rp2350_syscfg;
}

static void rp2350_tbman_init(Object *obj)
{
    RP2350TBManState *s = RP2350_TBMAN(obj);

    memory_region_init_io(&s->iomem, obj, &rp2350_tbman_ops, s,
                          TYPE_RP2350_TBMAN, RP2350_ATOMIC_REGION_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static void rp2350_glitch_detector_hold_reset(Object *obj, ResetType type)
{
    RP2350GlitchDetectorState *s = RP2350_GLITCH_DETECTOR(obj);

    /*
     * A system reset stands for a reset of the whole switched core
     * domain, which clears the detectors and their registers.
     */
    s->arm = GD_ARM_NO;
    s->disarm = 0;
    s->sensitivity = 0;
    s->lock = 0;
    s->trig_status = 0;
}

/* OTP CRIT1.GLITCH_DETECTOR_ENABLE. */
static void rp2350_glitch_detector_set_otp_enable(void *opaque, int n,
                                                  int level)
{
    RP2350GlitchDetectorState *s = opaque;

    s->otp_enable = level;
}

static void rp2350_glitch_detector_init(Object *obj)
{
    RP2350GlitchDetectorState *s = RP2350_GLITCH_DETECTOR(obj);

    memory_region_init_io(&s->iomem, obj, &rp2350_glitch_detector_ops, s,
                          TYPE_RP2350_GLITCH_DETECTOR,
                          RP2350_ATOMIC_REGION_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    qdev_init_gpio_out_named(DEVICE(obj), &s->chip_reset, "chip-reset", 1);
    qdev_init_gpio_in_named(DEVICE(obj), rp2350_glitch_detector_set_otp_enable,
                            "otp-enable", 1);
}

static const VMStateDescription vmstate_rp2350_glitch_detector = {
    .name = TYPE_RP2350_GLITCH_DETECTOR,
    .version_id = 3,
    .minimum_version_id = 3,
    .fields = (const VMStateField[]) {
        VMSTATE_BOOL(otp_enable, RP2350GlitchDetectorState),
        VMSTATE_UINT32(arm, RP2350GlitchDetectorState),
        VMSTATE_UINT32(disarm, RP2350GlitchDetectorState),
        VMSTATE_UINT32(sensitivity, RP2350GlitchDetectorState),
        VMSTATE_UINT32(lock, RP2350GlitchDetectorState),
        VMSTATE_UINT32(trig_status, RP2350GlitchDetectorState),
        VMSTATE_END_OF_LIST()
    },
};

static void rp2350_glitch_detector_class_init(ObjectClass *klass,
                                              const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = rp2350_glitch_detector_hold_reset;
    dc->vmsd = &vmstate_rp2350_glitch_detector;
}

static void rp2350_dft_init(Object *obj)
{
    RP2350DFTState *s = RP2350_DFT(obj);

    memory_region_init_io(&s->iomem, obj, &rp2350_dft_ops, s,
                          TYPE_RP2350_DFT, RP2350_ATOMIC_REGION_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

/* [spec:nuos:req:emu.system-regs] */
static const TypeInfo rp2350_sysregs_types[] = {
    {
        .name          = TYPE_RP2350_SYSINFO,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(RP2350SysInfoState),
        .instance_init = rp2350_sysinfo_init,
        .class_init    = rp2350_sysinfo_class_init,
    }, {
        .name          = TYPE_RP2350_SYSCFG,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(RP2350SysCfgState),
        .instance_init = rp2350_syscfg_init,
        .class_init    = rp2350_syscfg_class_init,
    }, {
        .name          = TYPE_RP2350_TBMAN,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(RP2350TBManState),
        .instance_init = rp2350_tbman_init,
    }, {
        .name          = TYPE_RP2350_GLITCH_DETECTOR,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(RP2350GlitchDetectorState),
        .instance_init = rp2350_glitch_detector_init,
        .class_init    = rp2350_glitch_detector_class_init,
    }, {
        .name          = TYPE_RP2350_DFT,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(RP2350DFTState),
        .instance_init = rp2350_dft_init,
    },
};

DEFINE_TYPES(rp2350_sysregs_types)
