/*
 * ESP32-P4 debug assistant (ASSIST_DEBUG, the CPU bus monitor)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapter
 * 22 (Debug Assistant), and ESP-IDF's assist_debug_reg.h for v3, the
 * authority for the registers and their reset values, its
 * assist_debug_ll.h, whose description of the interrupt registers the
 * model follows, and debug_assist.c and the panic handler, which use them
 * for the hardware stack guard.
 *
 * For each HP core, at 0x80 * core:
 *
 * - The SP monitor: with INTR_ENA's SP_SPILL_MIN_ENA (bit 8) set, INTR_RAW
 *   bit 8 is high while the core's SP is below SP_MIN; with
 *   SP_SPILL_MAX_ENA (bit 9), bit 9 while it is above SP_MAX. Both are
 *   levels, following SP. SP_PC records the PC of the instruction that took
 *   SP out of bounds, as it leaves them. The core checks SP after every
 *   instruction that writes it while the monitor is on (a TB flag of the
 *   core's, so that code runs unchecked while it is off); a change of the
 *   bounds or enables compares the core's SP with them at once, or, made
 *   by the other core, at the end of this core's current block of
 *   instructions. Enabling the monitor takes effect from the end of the
 *   block of instructions the enabling store is in.
 *
 * - The region monitors: two regions of the data bus (DRAM0: the TCM, ROM,
 *   L2MEM and the external memory windows, 0x30000000 to 0x3FEFFFFF and
 *   0x3FF20000 to 0x4FFFFFFF) and two of the peripheral bus (PIF: the rest
 *   of the address space, but the core's CLIC and CLINT), each [MIN, MAX]
 *   inclusive (empty from reset, MIN 0xFFFFFFFF and MAX 0), watched for
 *   reads and for writes by INTR_ENA bits 0 to 7. A load or store by the
 *   core that its PMP and PMA allow and that touches a byte of a watched
 *   region on the bus the access is on sets the region's INTR_RAW bit,
 *   which holds until INTR_CLR clears it; AREA_PC and AREA_SP take the
 *   accessing instruction's PC and the core's SP when a region bit sets
 *   with none set before. Instruction fetches are not watched. An AMO
 *   counts as a write. A change of the regions or their enables takes
 *   effect at once, or, made by the other core, at the end of this core's
 *   current block of instructions.
 *
 * - The interrupt: as ESP-IDF's assist_debug_ll.h describes the hardware,
 *   each INTR_RAW bit that INTR_RLS (0x08, the TRM's INTR_ENA) enables
 *   latches an interrupt that INTR_CLR clears and INTR_RAW does not show;
 *   ASSIST_DEBUG_INTR (source 127) is high while either core has one
 *   latched. Clearing a latched SP interrupt while SP is still out of
 *   bounds latches it again. ESP-IDF's hardware stack guard enables the
 *   monitor with INTR_ENA and the interrupt with INTR_RLS, and its panic
 *   handler reads INTR_RAW, SP_MIN, SP_MAX and SP_PC.
 *
 * - PC logging: with RCD_EN's PDEBUGEN and RECORDEN set, RCD_PDEBUGPC and
 *   RCD_PDEBUGSP follow the core's PC and SP; when either is cleared they
 *   keep the values they had. A reset of the core clears RCD_EN, so that
 *   they keep the PC and SP at the reset, which the boot ROM prints as
 *   "Saved PC" and re-enables. They are not reset by any reset the
 *   machine makes, and are 0 at power-on. The PC the model follows is the
 *   core's as of its last block of instructions, or of an access the
 *   model makes on its behalf; a core reading its own record sees a PC
 *   near the load.
 *
 * - LASTPC_BEFORE_EXCEPTION: the PC at the core's last trap (the
 *   excepting instruction's, or the one an interrupt comes before).
 *   DEBUG_MODE: 0, as no debugger halts the cores.
 *
 * The IRAM0 and DRAM0 exception monitor records (0x50 to 0x6C of each
 * core) are in the header but not the TRM, which leaves what they record
 * undocumented: they read 0, logged as unimplemented; the two LIMIT_CYCLE
 * registers they go with (0x100, 0x104) hold what software writes.
 * CLOCK_GATE forces the register clock on and has no other effect.
 *
 * The block is not gated by HP_SYS_CLKRST_BUSMON_CPU_CLK_EN, which is
 * clear from reset: the boot ROM reads the PC records and enables PC
 * logging without setting it, and on silicon prints the saved PC after a
 * CPU reset.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "exec/cputlb.h"
#include "hw/core/cpu.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/esp32p4_assist_debug.h"
#include "migration/vmstate.h"
#include "system/tcg.h"
#include "target/riscv/esp32p4_cpu.h"

#define CORE_STRIDE             0x80

/* Each core's registers, from its base. */
#define A_INTR_ENA              0x00
#define A_INTR_RAW              0x04
#define A_INTR_RLS              0x08
#define A_INTR_CLR              0x0c
#define A_AREA_FIRST            0x10
#define A_AREA_LAST             0x2c
#define A_AREA_PC               0x30
#define A_AREA_SP               0x34
#define A_SP_MIN                0x38
#define A_SP_MAX                0x3c
#define A_SP_PC                 0x40
#define A_RCD_EN                0x44
#define A_RCD_PDEBUGPC          0x48
#define A_RCD_PDEBUGSP          0x4c
#define A_EXC_MON_FIRST         0x50
#define A_EXC_MON_LAST          0x6c
#define A_LASTPC_BEFORE_EXC     0x70
#define A_DEBUG_MODE            0x74

/* The registers of both cores. */
#define A_LIMIT_CYCLE_0         0x100
#define A_LIMIT_CYCLE_1         0x104
#define A_CLOCK_GATE            0x108
#define A_DATE                  0x3fc

#define INTR_MASK               0x3ff
#define INTR_REGIONS            0xff
#define INTR_SP                 (ESP32P4_ASSIST_SP_MIN | ESP32P4_ASSIST_SP_MAX)
#define RCD_RECORDEN            BIT(0)
#define RCD_PDEBUGEN            BIT(1)
#define RCD_RECORDING           (RCD_RECORDEN | RCD_PDEBUGEN)
#define LIMIT_CYCLE_MASK        0xfffff
#define DATE_MASK               0x0fffffff
#define DATE_RESET              0x2109130

/* What esp32p4_assist_debug_push() gives a core. */
#define PUSH_REGIONS            BIT(0)
#define PUSH_SP                 BIT(1)

/*
 * The latched interrupts: each raw bit RLS lets through sets one. A
 * report from the middle of a core's block of translated code drives the
 * interrupt as the block ends.
 */
/* [spec:nuos:req:emu.esp32p4.debug] */
static void esp32p4_assist_debug_update(ESP32P4AssistDebugState *s)
{
    bool level = false;

    for (int i = 0; i < ESP32P4_ASSIST_DEBUG_CORES; i++) {
        ESP32P4AssistDebugCore *c = &s->core[i];

        c->st |= (c->raw_region | c->raw_sp) & c->rls;
        level |= c->st != 0;
    }
    if (esp32p4_cpu_mid_block()) {
        timer_mod(s->irq_timer, 0);
        return;
    }
    qemu_set_irq(s->irq, level);
}

static void esp32p4_assist_debug_irq_timer(void *opaque)
{
    esp32p4_assist_debug_update(opaque);
}

/* The core's report of SP's place against the monitored bounds. */
/* [spec:nuos:req:emu.esp32p4.debug] */
static void esp32p4_assist_debug_sp(void *opaque, int core, uint32_t raw,
                                    uint32_t pc)
{
    ESP32P4AssistDebugState *s = opaque;
    ESP32P4AssistDebugCore *c = &s->core[core];

    raw &= c->ena & INTR_SP;
    if (raw && !c->raw_sp) {
        c->sp_pc = pc;
    }
    c->raw_sp = raw;
    esp32p4_assist_debug_update(s);
}

/* The core's report of an access to a watched region. */
/* [spec:nuos:req:emu.esp32p4.debug] */
static void esp32p4_assist_debug_region(void *opaque, int core,
                                        uint32_t hits, uint32_t pc,
                                        uint32_t sp)
{
    ESP32P4AssistDebugState *s = opaque;
    ESP32P4AssistDebugCore *c = &s->core[core];

    hits &= c->ena & INTR_REGIONS;
    if (!hits) {
        return;
    }
    if (!c->raw_region) {
        c->area_pc = pc;
        c->area_sp = sp;
    }
    c->raw_region |= hits;
    esp32p4_assist_debug_update(s);
}

static const ESP32P4AssistOps esp32p4_assist_debug_ops = {
    .sp = esp32p4_assist_debug_sp,
    .region = esp32p4_assist_debug_region,
};

/*
 * Give the core the regions to watch, flushing its TLB so that pages
 * holding them go back to being checked on every access.
 */
/* [spec:nuos:req:emu.esp32p4.debug] */
static void esp32p4_assist_debug_apply_regions(ESP32P4AssistDebugCore *c)
{
    ESP32P4CPU *cpu = c->cpu;

    for (int r = 0; r < ESP32P4_ASSIST_REGIONS; r++) {
        cpu->assist_lo[r] = c->area[2 * r];
        cpu->assist_hi[r] = c->area[2 * r + 1];
    }
    qatomic_set(&cpu->assist_watch, c->ena & INTR_REGIONS);
    if (tcg_enabled()) {
        tlb_flush(CPU(cpu));
    }
}

/*
 * Give the core the SP bounds, a bound not monitored being the whole
 * address space, and compare SP with them. A core whose monitor comes on
 * leaves its chain of translated blocks, which do not check SP, at the
 * next block.
 */
/* [spec:nuos:req:emu.esp32p4.debug] */
static void esp32p4_assist_debug_apply_sp(ESP32P4AssistDebugCore *c)
{
    ESP32P4CPU *cpu = c->cpu;
    bool en = c->ena & INTR_SP;

    qatomic_set(&cpu->spmon_min,
                c->ena & ESP32P4_ASSIST_SP_MIN ? c->sp_min : 0);
    qatomic_set(&cpu->spmon_max,
                c->ena & ESP32P4_ASSIST_SP_MAX ? c->sp_max : UINT32_MAX);
    if (en && !qatomic_read(&cpu->spmon_en)) {
        qatomic_set(&cpu->spmon_en, true);
        cpu_exit(CPU(cpu));
    } else if (!en) {
        qatomic_set(&cpu->spmon_en, false);
    }
    esp32p4_assist_sp_update(cpu);
}

static void esp32p4_assist_debug_apply(ESP32P4AssistDebugCore *c,
                                       unsigned what)
{
    if (what & PUSH_REGIONS) {
        esp32p4_assist_debug_apply_regions(c);
    }
    if (what & PUSH_SP) {
        esp32p4_assist_debug_apply_sp(c);
    }
}

/* On the core's thread: what is pending, as the registers are now. */
static void esp32p4_assist_debug_push_work(CPUState *cs, run_on_cpu_data data)
{
    ESP32P4AssistDebugCore *c = data.host_ptr;
    unsigned what = c->push_pending;

    c->push_pending = 0;
    esp32p4_assist_debug_apply(c, what);
}

/*
 * Give the core its regions (PUSH_REGIONS) or SP bounds (PUSH_SP). Its
 * TLB, and the SP it compares, are its own thread's: another thread
 * flushing the TLB while the core runs frees and refills the tables its
 * translated code is reading. So a push from elsewhere, another core's
 * store to this core's registers or a reset of the block from the main
 * loop (an HP core reset a watchdog's timer makes), is queued as work the
 * core does before it executes again, with the registers as they are
 * then. A push on the core's own thread (under icount, the one thread the
 * cores take turns on) takes effect at once, as does one to a core that
 * cannot be executing: one that is stopped (and stays so while the BQL,
 * which every push holds, is held), or any without TCG (under qtest).
 */
/* [spec:nuos:req:emu.esp32p4.debug] */
static void esp32p4_assist_debug_push(ESP32P4AssistDebugCore *c,
                                      unsigned what)
{
    CPUState *cs;

    if (!c->cpu || !what) {
        return;
    }
    cs = CPU(c->cpu);
    if (!tcg_enabled() || !cs->created || qemu_cpu_is_self(cs) ||
        cs->stopped) {
        esp32p4_assist_debug_apply(c, what);
        return;
    }
    if (!c->push_pending) {
        async_run_on_cpu(cs, esp32p4_assist_debug_push_work,
                         RUN_ON_CPU_HOST_PTR(c));
    }
    c->push_pending |= what;
}

/* PC logging: the core's PC and SP, while it sends them. */
/* [spec:nuos:req:emu.esp32p4.debug] */
static void esp32p4_assist_debug_record(ESP32P4AssistDebugCore *c)
{
    if ((c->rcd_en & RCD_RECORDING) == RCD_RECORDING && c->cpu &&
        !c->in_reset) {
        c->pdebug_pc = c->cpu->parent_obj.env.pc;
        c->pdebug_sp = c->cpu->parent_obj.env.gpr[2];
    }
}

static uint32_t esp32p4_assist_debug_core_read(ESP32P4AssistDebugCore *c,
                                               hwaddr reg)
{
    switch (reg) {
    case A_INTR_ENA:
        return c->ena;
    case A_INTR_RAW:
        return c->raw_region | c->raw_sp;
    case A_INTR_RLS:
        return c->rls;
    case A_INTR_CLR:
        return 0;
    case A_AREA_FIRST ... A_AREA_LAST:
        return c->area[(reg - A_AREA_FIRST) / 4];
    case A_AREA_PC:
        return c->area_pc;
    case A_AREA_SP:
        return c->area_sp;
    case A_SP_MIN:
        return c->sp_min;
    case A_SP_MAX:
        return c->sp_max;
    case A_SP_PC:
        return c->sp_pc;
    case A_RCD_EN:
        return c->rcd_en;
    case A_RCD_PDEBUGPC:
        esp32p4_assist_debug_record(c);
        return c->pdebug_pc;
    case A_RCD_PDEBUGSP:
        esp32p4_assist_debug_record(c);
        return c->pdebug_sp;
    case A_EXC_MON_FIRST ... A_EXC_MON_LAST:
        qemu_log_mask(LOG_UNIMP, "%s: the exception monitor records are "
                      "not documented; offset 0x%" HWADDR_PRIx " reads 0\n",
                      TYPE_ESP32P4_ASSIST_DEBUG, reg);
        return 0;
    case A_LASTPC_BEFORE_EXC:
        return c->cpu ? c->cpu->exc_pc : 0;
    case A_DEBUG_MODE:
        return 0;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of reserved offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_ASSIST_DEBUG, reg);
        return 0;
    }
}

static uint64_t esp32p4_assist_debug_read(void *opaque, hwaddr addr,
                                          unsigned size)
{
    ESP32P4AssistDebugState *s = opaque;
    hwaddr reg = addr & ~3;
    uint32_t v;

    switch (reg) {
    case A_LIMIT_CYCLE_0:
    case A_LIMIT_CYCLE_1:
        v = s->limit_cycle[(reg - A_LIMIT_CYCLE_0) / 4];
        break;
    case A_CLOCK_GATE:
        v = s->clock_gate;
        break;
    case A_DATE:
        v = s->date;
        break;
    default:
        if (reg < ESP32P4_ASSIST_DEBUG_CORES * CORE_STRIDE) {
            v = esp32p4_assist_debug_core_read(&s->core[reg / CORE_STRIDE],
                                               reg % CORE_STRIDE);
        } else {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: read of reserved offset 0x%"
                          HWADDR_PRIx "\n", TYPE_ESP32P4_ASSIST_DEBUG, addr);
            v = 0;
        }
        break;
    }
    return extract32(v, (addr & 3) * 8, size * 8);
}

/* [spec:nuos:req:emu.esp32p4.debug] */
static void esp32p4_assist_debug_core_write(ESP32P4AssistDebugState *s,
                                            ESP32P4AssistDebugCore *c,
                                            hwaddr reg, uint32_t v,
                                            uint32_t bytes)
{
    uint32_t old;

    switch (reg) {
    case A_INTR_ENA:
        old = c->ena;
        c->ena = ((c->ena & ~bytes) | (v & bytes)) & INTR_MASK;
        esp32p4_assist_debug_push(c,
            ((old ^ c->ena) & INTR_REGIONS ? PUSH_REGIONS : 0) |
            ((old ^ c->ena) & INTR_SP ? PUSH_SP : 0));
        break;
    case A_INTR_RLS:
        c->rls = ((c->rls & ~bytes) | (v & bytes)) & INTR_MASK;
        break;
    case A_INTR_CLR:
        c->st &= ~(v & bytes);
        c->raw_region &= ~(v & bytes);
        break;
    case A_AREA_FIRST ... A_AREA_LAST:
        old = (reg - A_AREA_FIRST) / 4;
        c->area[old] = (c->area[old] & ~bytes) | (v & bytes);
        esp32p4_assist_debug_push(c, PUSH_REGIONS);
        break;
    case A_SP_MIN:
        c->sp_min = (c->sp_min & ~bytes) | (v & bytes);
        esp32p4_assist_debug_push(c, PUSH_SP);
        break;
    case A_SP_MAX:
        c->sp_max = (c->sp_max & ~bytes) | (v & bytes);
        esp32p4_assist_debug_push(c, PUSH_SP);
        break;
    case A_RCD_EN:
        /* A record that stops keeps the PC and SP it last had. */
        esp32p4_assist_debug_record(c);
        c->rcd_en = ((c->rcd_en & ~bytes) | (v & bytes)) & RCD_RECORDING;
        break;
    case A_INTR_RAW:
    case A_AREA_PC:
    case A_AREA_SP:
    case A_SP_PC:
    case A_RCD_PDEBUGPC:
    case A_RCD_PDEBUGSP:
    case A_EXC_MON_FIRST ... A_EXC_MON_LAST:
    case A_LASTPC_BEFORE_EXC:
    case A_DEBUG_MODE:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_ASSIST_DEBUG, reg);
        return;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to reserved offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_ASSIST_DEBUG, reg);
        return;
    }
    esp32p4_assist_debug_update(s);
}

static void esp32p4_assist_debug_write(void *opaque, hwaddr addr,
                                       uint64_t value, unsigned size)
{
    ESP32P4AssistDebugState *s = opaque;
    hwaddr reg = addr & ~3;
    unsigned shift = (addr & 3) * 8;
    uint32_t bytes = MAKE_64BIT_MASK(shift, size * 8);
    uint32_t v = ((uint32_t)value << shift) & bytes;
    int i;

    switch (reg) {
    case A_LIMIT_CYCLE_0:
    case A_LIMIT_CYCLE_1:
        i = (reg - A_LIMIT_CYCLE_0) / 4;
        s->limit_cycle[i] = ((s->limit_cycle[i] & ~bytes) | v) &
                            LIMIT_CYCLE_MASK;
        break;
    case A_CLOCK_GATE:
        s->clock_gate = ((s->clock_gate & ~bytes) | v) & 1;
        break;
    case A_DATE:
        s->date = ((s->date & ~bytes) | v) & DATE_MASK;
        break;
    default:
        if (reg < ESP32P4_ASSIST_DEBUG_CORES * CORE_STRIDE) {
            esp32p4_assist_debug_core_write(s, &s->core[reg / CORE_STRIDE],
                                            reg % CORE_STRIDE, v, bytes);
        } else {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: write to reserved offset 0x%"
                          HWADDR_PRIx "\n", TYPE_ESP32P4_ASSIST_DEBUG, addr);
        }
        break;
    }
}

static const MemoryRegionOps esp32p4_assist_debug_ops_mmio = {
    .read = esp32p4_assist_debug_read,
    .write = esp32p4_assist_debug_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

/*
 * A core held in reset stops sending its PC: its record keeps the PC and
 * SP at the reset, and RCD_EN clears.
 */
/* [spec:nuos:req:emu.esp32p4.debug] */
static void esp32p4_assist_debug_core_reset(void *opaque, int n, int level)
{
    ESP32P4AssistDebugState *s = opaque;
    ESP32P4AssistDebugCore *c = &s->core[n];

    if (level && !c->in_reset) {
        esp32p4_assist_debug_record(c);
        c->rcd_en = 0;
    }
    c->in_reset = level;
}

/*
 * A machine reset, before the cores take their reset state: a core
 * sending its PC leaves it in its record, as at a reset of the core.
 */
static void esp32p4_assist_debug_reset_enter(Object *obj, ResetType type)
{
    ESP32P4AssistDebugState *s = ESP32P4_ASSIST_DEBUG(obj);

    for (int i = 0; i < ESP32P4_ASSIST_DEBUG_CORES; i++) {
        esp32p4_assist_debug_record(&s->core[i]);
    }
}

static void esp32p4_assist_debug_reset_hold(Object *obj, ResetType type)
{
    ESP32P4AssistDebugState *s = ESP32P4_ASSIST_DEBUG(obj);

    for (int i = 0; i < ESP32P4_ASSIST_DEBUG_CORES; i++) {
        ESP32P4AssistDebugCore *c = &s->core[i];

        c->ena = 0;
        c->rls = 0;
        c->raw_region = 0;
        c->raw_sp = 0;
        c->st = 0;
        for (int r = 0; r < 8; r += 2) {
            c->area[r] = UINT32_MAX;
            c->area[r + 1] = 0;
        }
        c->area_pc = 0;
        c->area_sp = 0;
        c->sp_min = 0;
        c->sp_max = UINT32_MAX;
        c->sp_pc = 0;
        c->rcd_en = 0;
    }
    s->limit_cycle[0] = 0;
    s->limit_cycle[1] = 0;
    s->clock_gate = 1;
    s->date = DATE_RESET;
}

static void esp32p4_assist_debug_reset_exit(Object *obj, ResetType type)
{
    ESP32P4AssistDebugState *s = ESP32P4_ASSIST_DEBUG(obj);

    for (int i = 0; i < ESP32P4_ASSIST_DEBUG_CORES; i++) {
        esp32p4_assist_debug_push(&s->core[i], PUSH_REGIONS | PUSH_SP);
    }
    esp32p4_assist_debug_update(s);
}

static void esp32p4_assist_debug_realize(DeviceState *dev, Error **errp)
{
    ESP32P4AssistDebugState *s = ESP32P4_ASSIST_DEBUG(dev);

    s->irq_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                esp32p4_assist_debug_irq_timer, s);
    for (int i = 0; i < ESP32P4_ASSIST_DEBUG_CORES; i++) {
        if (s->core[i].cpu) {
            esp32p4_assist_connect(s->core[i].cpu, &esp32p4_assist_debug_ops,
                                   s);
        }
    }
}

static void esp32p4_assist_debug_init(Object *obj)
{
    ESP32P4AssistDebugState *s = ESP32P4_ASSIST_DEBUG(obj);

    memory_region_init_io(&s->iomem, obj, &esp32p4_assist_debug_ops_mmio, s,
                          "esp32p4.assist-debug", ESP32P4_ASSIST_DEBUG_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    qdev_init_gpio_in_named(DEVICE(obj), esp32p4_assist_debug_core_reset,
                            ESP32P4_ASSIST_DEBUG_CORE_RESET,
                            ESP32P4_ASSIST_DEBUG_CORES);
}

static const VMStateDescription vmstate_esp32p4_assist_debug_core = {
    .name = "esp32p4-assist-debug/core",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(ena, ESP32P4AssistDebugCore),
        VMSTATE_UINT32(rls, ESP32P4AssistDebugCore),
        VMSTATE_UINT32(raw_region, ESP32P4AssistDebugCore),
        VMSTATE_UINT32(raw_sp, ESP32P4AssistDebugCore),
        VMSTATE_UINT32(st, ESP32P4AssistDebugCore),
        VMSTATE_UINT32_ARRAY(area, ESP32P4AssistDebugCore, 8),
        VMSTATE_UINT32(area_pc, ESP32P4AssistDebugCore),
        VMSTATE_UINT32(area_sp, ESP32P4AssistDebugCore),
        VMSTATE_UINT32(sp_min, ESP32P4AssistDebugCore),
        VMSTATE_UINT32(sp_max, ESP32P4AssistDebugCore),
        VMSTATE_UINT32(sp_pc, ESP32P4AssistDebugCore),
        VMSTATE_UINT32(rcd_en, ESP32P4AssistDebugCore),
        VMSTATE_UINT32(pdebug_pc, ESP32P4AssistDebugCore),
        VMSTATE_UINT32(pdebug_sp, ESP32P4AssistDebugCore),
        VMSTATE_BOOL(in_reset, ESP32P4AssistDebugCore),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_esp32p4_assist_debug = {
    .name = TYPE_ESP32P4_ASSIST_DEBUG,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(core, ESP32P4AssistDebugState,
                             ESP32P4_ASSIST_DEBUG_CORES, 1,
                             vmstate_esp32p4_assist_debug_core,
                             ESP32P4AssistDebugCore),
        VMSTATE_TIMER_PTR(irq_timer, ESP32P4AssistDebugState),
        VMSTATE_UINT32_ARRAY(limit_cycle, ESP32P4AssistDebugState, 2),
        VMSTATE_UINT32(clock_gate, ESP32P4AssistDebugState),
        VMSTATE_UINT32(date, ESP32P4AssistDebugState),
        VMSTATE_END_OF_LIST()
    },
};

static const Property esp32p4_assist_debug_properties[] = {
    DEFINE_PROP_LINK("cpu0", ESP32P4AssistDebugState, core[0].cpu,
                     TYPE_ESP32P4_CPU, ESP32P4CPU *),
    DEFINE_PROP_LINK("cpu1", ESP32P4AssistDebugState, core[1].cpu,
                     TYPE_ESP32P4_CPU, ESP32P4CPU *),
};

static void esp32p4_assist_debug_class_init(ObjectClass *klass,
                                            const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = esp32p4_assist_debug_realize;
    rc->phases.enter = esp32p4_assist_debug_reset_enter;
    rc->phases.hold = esp32p4_assist_debug_reset_hold;
    rc->phases.exit = esp32p4_assist_debug_reset_exit;
    dc->vmsd = &vmstate_esp32p4_assist_debug;
    device_class_set_props(dc, esp32p4_assist_debug_properties);
}

/* [spec:nuos:req:emu.esp32p4.debug] */
static const TypeInfo esp32p4_assist_debug_info = {
    .name          = TYPE_ESP32P4_ASSIST_DEBUG,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32P4AssistDebugState),
    .instance_init = esp32p4_assist_debug_init,
    .class_init    = esp32p4_assist_debug_class_init,
};

static void esp32p4_assist_debug_register_types(void)
{
    type_register_static(&esp32p4_assist_debug_info);
}
type_init(esp32p4_assist_debug_register_types)
