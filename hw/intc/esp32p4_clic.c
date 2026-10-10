/*
 * ESP32-P4 HP cores' core-local interrupt controllers (CLIC)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), 2.8
 * (Memory-Mapped Registers) and 2.9.2 (CLIC); Core-Local Interrupt
 * Controller (CLIC) RISC-V Privileged Architecture Extensions, 10/10/2023,
 * with its smclicshv, smclicconfig and suclic extensions; ESP-IDF's
 * clic_reg.h, which has the v3 layout of mcliccfg.
 *
 * Each HP core has a CLIC of 48 interrupts, of which the CLINT's software
 * (3) and timer (7) interrupts and the external interrupts 16 to 47 exist;
 * the others' registers read zero and ignore writes. Per interrupt, four
 * byte registers at 0x1000 + 4 * ID: clicintip, the pending bit;
 * clicintie, the enable; clicintattr, SHV (selective hardware vectoring),
 * TRIG (edge or level, and polarity) and MODE (machine or user); and
 * clicintctl, whose upper CLICINTCTLBITS = 3 bits hold the level and
 * priority, the lower five reading one. Byte, halfword and word accesses
 * are all atomic.
 *
 * mcliccfg (0x0, the v3 layout ESP-IDF gives) splits clicintctl between
 * level and priority: MNLBITS (3:0) upper bits are the level of a
 * machine-mode interrupt, UNLBITS (27:24) those of a user-mode one, each
 * clamped to 8, the level's missing lower bits reading one; with none,
 * every level is 255. NMBITS (5:4) has its lower bit implemented: with it
 * clear every interrupt is a machine-mode one, and with it set
 * clicintattr.MODE's upper bit gives the mode. clicinfo (0x4) gives 48
 * interrupts and CLICINTCTLBITS 3. mintthresh (0x8, TH in bits 31:24) is
 * the mintthresh CSR's threshold, which the core keeps. No interrupt
 * triggers (clicinttrig) exist; the rest of the 64 KiB window, beyond
 * the registers, answers with a bus error.
 *
 * A level-triggered interrupt is pending while its line is at its active
 * level (TRIG's upper bit inverting it), and software writes to clicintip
 * are ignored. An edge-triggered one becomes pending on an edge of its
 * polarity and stays so until software writes clicintip, or the core
 * services it vectored or through mnxti/unxti.
 *
 * Of the pending and enabled interrupts, the CLIC presents to its core
 * the highest-ranked: by mode, machine above user, then by clicintctl,
 * level then priority, then by ID; with its level and whether it is
 * vectored. What the core does with it is in
 * target/riscv/tcg/esp32p4_clic.c.
 *
 * The user-mode view (0x20B00000) has ucliccfg (0x0, UNLBITS) and the
 * interrupt registers of user-mode interrupts only, the others reading
 * zero and ignoring writes; it cannot make an interrupt machine mode.
 *
 * Address decoding (TRM table 2.8-1): each core sees its own CLIC at the
 * window's base and the other core's 64 KiB above it. An access that does
 * not come from an HP core, such as the LP core's or a DMA master's,
 * answers with a bus error; qtest and the debugger see core 0's view.
 *
 * The CLINT's interrupts (hw/intc/esp32p4_clint.c) arrive on lines 3 and
 * 7; the interrupt matrix drives lines 16 to 47.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/core/cpu.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/intc/esp32p4_clic.h"
#include "migration/vmstate.h"
#include "system/qtest.h"

#define A_MCLICCFG      0x0
#define A_CLICINFO      0x4
#define A_MINTTHRESH    0x8
#define A_CLICINT       0x1000
#define CLICINT_END     (A_CLICINT + 4 * ESP32P4_CLIC_NUM_IRQS)

#define CORE_WINDOW     0x10000

/* clicinfo: NUM_INTERRUPTS 48, VERSION 0, CLICINTCTLBITS 3. */
#define CLICINFO_VALUE  ((3u << 21) | ESP32P4_CLIC_NUM_IRQS)

/* mcliccfg */
#define CLICCFG_MNLBITS_SHIFT   0
#define CLICCFG_NMBITS_SHIFT    4
#define CLICCFG_UNLBITS_SHIFT   24
#define CLICCFG_NLBITS_MAX      8

/* clicintattr */
#define ATTR_SHV        BIT(0)
#define ATTR_TRIG_EDGE  BIT(1)
#define ATTR_TRIG_NEG   BIT(2)
#define ATTR_MODE_HI    BIT(7)
#define ATTR_MODE       (3u << 6)

/* clicintctl: the implemented bits; the rest read one. */
#define CTL_BITS        0xe0
#define CTL_ONES        0x1f

static bool clic_irq_exists(int id)
{
    return id == ESP32P4_CLIC_IRQ_MSIP || id == ESP32P4_CLIC_IRQ_MTIP ||
           (id >= ESP32P4_CLIC_IRQ_EXT0 && id < ESP32P4_CLIC_NUM_IRQS);
}

static bool clic_active(ESP32P4ClicCore *c)
{
    return !c->in_reset && clock_is_enabled(c->clk);
}

/* The interrupt's privilege mode, as NMBITS interprets clicintattr.MODE. */
static int clic_irq_priv(ESP32P4ClicCore *c, int id)
{
    return !c->nmbits || (c->attr[id] & ATTR_MODE_HI) ? PRV_M : PRV_U;
}

/*
 * The interrupt's level: the upper bits of clicintctl its mode's NLBITS
 * give, the lower bits reading one.
 */
static int clic_irq_level(ESP32P4ClicCore *c, int id)
{
    int nlbits = clic_irq_priv(c, id) == PRV_M ? c->mnlbits : c->unlbits;

    return c->ctl[id] | (0xff >> nlbits);
}

/*
 * Present the highest-ranked pending and enabled interrupt, by mode,
 * clicintctl and ID, to the core.
 */
/* [spec:nuos:req:emu.esp32p4.clic] */
static void clic_update(ESP32P4ClicCore *c)
{
    int best = -1;
    int best_rank = 0;
    uint32_t sel = 0;

    if (clic_active(c)) {
        for (int id = 0; id < ESP32P4_CLIC_NUM_IRQS; id++) {
            int rank;

            if (!clic_irq_exists(id) || !(c->ip[id] & 1) ||
                !(c->ie[id] & 1)) {
                continue;
            }
            rank = (clic_irq_priv(c, id) == PRV_M ? 0x100 : 0) | c->ctl[id];
            if (best < 0 || rank >= best_rank) {
                best = id;
                best_rank = rank;
            }
        }
    }
    if (best >= 0) {
        sel = esp32p4_clic_sel(c->attr[best] & ATTR_SHV,
                               clic_irq_priv(c, best),
                               clic_irq_level(c, best), best);
    }
    if (c->cpu) {
        esp32p4_cpu_clic_present(c->cpu, sel);
    }
}

/* A level-triggered interrupt's pending bit follows its line. */
static void clic_follow_line(ESP32P4ClicCore *c, int id)
{
    if (!(c->attr[id] & ATTR_TRIG_EDGE)) {
        c->ip[id] = c->line[id] ^ !!(c->attr[id] & ATTR_TRIG_NEG);
    }
}

/* [spec:nuos:req:emu.esp32p4.clic] */
static void clic_set_irq(void *opaque, int n, int level)
{
    ESP32P4ClicState *s = opaque;
    ESP32P4ClicCore *c = &s->core[n / ESP32P4_CLIC_NUM_IRQS];
    int id = n % ESP32P4_CLIC_NUM_IRQS;
    bool old = c->line[id];
    bool neg;

    if (!clic_irq_exists(id)) {
        return;
    }
    c->line[id] = level;
    if (!(c->attr[id] & ATTR_TRIG_EDGE)) {
        clic_follow_line(c, id);
    } else {
        neg = c->attr[id] & ATTR_TRIG_NEG;
        if (clic_active(c) && old != c->line[id] && c->line[id] != neg) {
            c->ip[id] = 1;
        }
    }
    clic_update(c);
}

/* The core services interrupt @id: an edge-triggered one stops pending. */
/* [spec:nuos:req:emu.esp32p4.clic] */
static void clic_claim(void *opaque, int id)
{
    ESP32P4ClicCore *c = opaque;

    if (c->attr[id] & ATTR_TRIG_EDGE) {
        c->ip[id] = 0;
        clic_update(c);
    }
}

static const ESP32P4ClicOps clic_ops = {
    .claim = clic_claim,
};

static uint32_t clic_cfg_read(ESP32P4ClicCore *c, bool user)
{
    uint32_t v = c->unlbits << CLICCFG_UNLBITS_SHIFT;

    if (!user) {
        v |= (c->mnlbits << CLICCFG_MNLBITS_SHIFT) |
             (c->nmbits << CLICCFG_NMBITS_SHIFT);
    }
    return v;
}

/* [spec:nuos:req:emu.esp32p4.clic] */
static void clic_cfg_write(ESP32P4ClicCore *c, uint32_t v, bool user)
{
    c->unlbits = MIN(extract32(v, CLICCFG_UNLBITS_SHIFT, 4),
                     CLICCFG_NLBITS_MAX);
    if (!user) {
        c->mnlbits = MIN(extract32(v, CLICCFG_MNLBITS_SHIFT, 4),
                         CLICCFG_NLBITS_MAX);
        c->nmbits = extract32(v, CLICCFG_NMBITS_SHIFT, 1);
    }
}

static uint8_t clic_byte_read(ESP32P4ClicCore *c, int id, int reg, bool user)
{
    if (!clic_irq_exists(id) || (user && clic_irq_priv(c, id) != PRV_U)) {
        return 0;
    }
    switch (reg) {
    case 0:
        return c->ip[id];
    case 1:
        return c->ie[id];
    case 2:
        return (c->attr[id] & (ATTR_SHV | ATTR_TRIG_EDGE | ATTR_TRIG_NEG)) |
               (clic_irq_priv(c, id) == PRV_M ? ATTR_MODE : 0);
    default:
        return c->ctl[id];
    }
}

/*
 * clicintattr: a level-triggered interrupt's pending bit follows its line
 * at once; the user-mode view keeps an interrupt in user mode.
 */
static void clic_attr_write(ESP32P4ClicCore *c, int id, uint8_t v, bool user)
{
    uint8_t mode = (v & ATTR_MODE_HI) && !user ? ATTR_MODE_HI : 0;

    c->attr[id] = (v & (ATTR_SHV | ATTR_TRIG_EDGE | ATTR_TRIG_NEG)) | mode;
    clic_follow_line(c, id);
}

/*
 * Write the registers of interrupt @id that @mask selects (bit n for byte
 * n), at once: the attributes, control and enable first, so that a
 * pending bit written with them follows the new trigger type.
 */
/* [spec:nuos:req:emu.esp32p4.clic] */
static void clic_int_write(ESP32P4ClicCore *c, int id, uint32_t v,
                           unsigned mask, bool user)
{
    if (!clic_irq_exists(id) || (user && clic_irq_priv(c, id) != PRV_U)) {
        return;
    }
    if (mask & BIT(2)) {
        clic_attr_write(c, id, extract32(v, 16, 8), user);
    }
    if (mask & BIT(3)) {
        c->ctl[id] = (extract32(v, 24, 8) & CTL_BITS) | CTL_ONES;
    }
    if (mask & BIT(1)) {
        c->ie[id] = extract32(v, 8, 1);
    }
    if ((mask & BIT(0)) && (c->attr[id] & ATTR_TRIG_EDGE)) {
        c->ip[id] = extract32(v, 0, 1);
    }
}

/*
 * The core whose view an access has: -1 if it does not come from an HP
 * core, as a DMA master's or the LP core's; core 0's for qtest and the
 * debugger.
 */
static int clic_accessor(ESP32P4ClicState *s, MemTxAttrs attrs)
{
    if (!current_cpu) {
        return qtest_enabled() || attrs.debug ? 0 : -1;
    }
    for (int i = 0; i < ESP32P4_CLIC_CORES; i++) {
        if (current_cpu == CPU(s->core[i].cpu)) {
            return i;
        }
    }
    return -1;
}

/* The register an offset in a core's window decodes to, if any. */
static bool clic_decodes(hwaddr off, bool user)
{
    if (off >= A_CLICINT && off < CLICINT_END) {
        return true;
    }
    switch (off & ~3) {
    case A_MCLICCFG:
        return true;
    case A_CLICINFO:
    case A_MINTTHRESH:
        return !user;
    default:
        return false;
    }
}

static MemTxResult clic_access(ESP32P4ClicState *s, hwaddr addr,
                               uint64_t *data, unsigned size, bool write,
                               bool user, MemTxAttrs attrs)
{
    int self = clic_accessor(s, attrs);
    ESP32P4ClicCore *c;
    hwaddr off = addr % CORE_WINDOW;
    unsigned shift = (off & 3) * 8;
    uint32_t mask32 = MAKE_64BIT_MASK(shift, size * 8);
    uint32_t v;

    if (self < 0) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: access at 0x%" HWADDR_PRIx
                      " from outside the HP cores\n", TYPE_ESP32P4_CLIC,
                      addr);
        return MEMTX_ERROR;
    }
    if (!clic_decodes(off, user)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: %s at 0x%" HWADDR_PRIx ", which "
                      "has no register\n", TYPE_ESP32P4_CLIC,
                      write ? "write" : "read", addr);
        return MEMTX_ERROR;
    }
    c = &s->core[addr >= CORE_WINDOW ? !self : self];
    if (write && !clic_active(c)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write at 0x%" HWADDR_PRIx
                      " to core %d's CLIC while its clock is stopped or "
                      "its core is in reset\n", TYPE_ESP32P4_CLIC, addr,
                      c->index);
        return MEMTX_OK;
    }

    if (off >= A_CLICINT) {
        int id = (off - A_CLICINT) / 4;

        if (write) {
            clic_int_write(c, id, *data << shift,
                           MAKE_64BIT_MASK(off & 3, size), user);
            clic_update(c);
            return MEMTX_OK;
        }
        v = 0;
        for (unsigned b = 0; b < 4; b++) {
            v |= clic_byte_read(c, id, b, user) << (8 * b);
        }
        *data = (v & mask32) >> shift;
        return MEMTX_OK;
    }

    switch (off & ~3) {
    case A_MCLICCFG:
        v = clic_cfg_read(c, user);
        if (write) {
            clic_cfg_write(c, (v & ~mask32) | ((*data << shift) & mask32),
                           user);
            clic_update(c);
            return MEMTX_OK;
        }
        break;
    case A_CLICINFO:
        v = CLICINFO_VALUE;
        if (write) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only "
                          "clicinfo\n", TYPE_ESP32P4_CLIC);
            return MEMTX_OK;
        }
        break;
    default:
        v = esp32p4_cpu_mintthresh(c->cpu) << 24;
        if (write) {
            v = (v & ~mask32) | ((*data << shift) & mask32);
            esp32p4_cpu_set_mintthresh(c->cpu, v >> 24);
            return MEMTX_OK;
        }
        break;
    }
    *data = (v & mask32) >> shift;
    return MEMTX_OK;
}

static MemTxResult clic_m_read(void *opaque, hwaddr addr, uint64_t *data,
                               unsigned size, MemTxAttrs attrs)
{
    return clic_access(opaque, addr, data, size, false, false, attrs);
}

/* [spec:nuos:req:emu.esp32p4.clic] */
static MemTxResult clic_m_write(void *opaque, hwaddr addr, uint64_t data,
                                unsigned size, MemTxAttrs attrs)
{
    return clic_access(opaque, addr, &data, size, true, false, attrs);
}

static MemTxResult clic_u_read(void *opaque, hwaddr addr, uint64_t *data,
                               unsigned size, MemTxAttrs attrs)
{
    return clic_access(opaque, addr, data, size, false, true, attrs);
}

/* [spec:nuos:req:emu.esp32p4.clic] */
static MemTxResult clic_u_write(void *opaque, hwaddr addr, uint64_t data,
                                unsigned size, MemTxAttrs attrs)
{
    return clic_access(opaque, addr, &data, size, true, true, attrs);
}

static const MemoryRegionOps clic_m_ops = {
    .read_with_attrs = clic_m_read,
    .write_with_attrs = clic_m_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

static const MemoryRegionOps clic_u_ops = {
    .read_with_attrs = clic_u_read,
    .write_with_attrs = clic_u_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

/* [spec:nuos:req:emu.esp32p4.clic] */
static void clic_core_reset(ESP32P4ClicCore *c)
{
    for (int id = 0; id < ESP32P4_CLIC_NUM_IRQS; id++) {
        c->ip[id] = 0;
        c->ie[id] = 0;
        c->attr[id] = ATTR_MODE_HI;
        c->ctl[id] = CTL_ONES;
        if (clic_irq_exists(id)) {
            clic_follow_line(c, id);
        }
    }
    c->mnlbits = 0;
    c->nmbits = 0;
    c->unlbits = 0;
}

static void clic_set_core_reset(void *opaque, int n, int level)
{
    ESP32P4ClicState *s = opaque;
    ESP32P4ClicCore *c = &s->core[n];

    if (c->in_reset == !!level) {
        return;
    }
    c->in_reset = level;
    clic_core_reset(c);
    clic_update(c);
}

static void clic_clock_update(void *opaque, ClockEvent event)
{
    clic_update(opaque);
}

static void esp32p4_clic_init(Object *obj)
{
    ESP32P4ClicState *s = ESP32P4_CLIC(obj);
    DeviceState *dev = DEVICE(obj);

    memory_region_init_io(&s->m_iomem, obj, &clic_m_ops, s, "esp32p4.clic",
                          ESP32P4_CLIC_WINDOW);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->m_iomem);
    memory_region_init_io(&s->u_iomem, obj, &clic_u_ops, s,
                          "esp32p4.clic-user", ESP32P4_CLIC_WINDOW);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->u_iomem);

    for (int i = 0; i < ESP32P4_CLIC_CORES; i++) {
        g_autofree char *name = g_strdup_printf("core%d", i);

        s->core[i].s = s;
        s->core[i].index = i;
        s->core[i].clk = qdev_init_clock_in(dev, name, clic_clock_update,
                                            &s->core[i], ClockUpdate);
    }
    qdev_init_gpio_in_named(dev, clic_set_irq, ESP32P4_CLIC_IRQ,
                            ESP32P4_CLIC_CORES * ESP32P4_CLIC_NUM_IRQS);
    qdev_init_gpio_in_named(dev, clic_set_core_reset,
                            ESP32P4_CLIC_CORE_RESET, ESP32P4_CLIC_CORES);
}

static void esp32p4_clic_realize(DeviceState *dev, Error **errp)
{
    ESP32P4ClicState *s = ESP32P4_CLIC(dev);

    for (int i = 0; i < ESP32P4_CLIC_CORES; i++) {
        if (!s->core[i].cpu) {
            error_setg(errp, "%s: cpu%d is not set", TYPE_ESP32P4_CLIC, i);
            return;
        }
        esp32p4_cpu_clic_connect(s->core[i].cpu, &clic_ops, &s->core[i]);
    }
}

static void esp32p4_clic_reset_hold(Object *obj, ResetType type)
{
    ESP32P4ClicState *s = ESP32P4_CLIC(obj);

    for (int i = 0; i < ESP32P4_CLIC_CORES; i++) {
        clic_core_reset(&s->core[i]);
    }
}

static void esp32p4_clic_reset_exit(Object *obj, ResetType type)
{
    ESP32P4ClicState *s = ESP32P4_CLIC(obj);

    for (int i = 0; i < ESP32P4_CLIC_CORES; i++) {
        clic_update(&s->core[i]);
    }
}

static int esp32p4_clic_post_load(void *opaque, int version_id)
{
    ESP32P4ClicState *s = opaque;

    for (int i = 0; i < ESP32P4_CLIC_CORES; i++) {
        clic_update(&s->core[i]);
    }
    return 0;
}

static const VMStateDescription vmstate_esp32p4_clic_core = {
    .name = TYPE_ESP32P4_CLIC "/core",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8_ARRAY(ip, ESP32P4ClicCore, ESP32P4_CLIC_NUM_IRQS),
        VMSTATE_UINT8_ARRAY(ie, ESP32P4ClicCore, ESP32P4_CLIC_NUM_IRQS),
        VMSTATE_UINT8_ARRAY(attr, ESP32P4ClicCore, ESP32P4_CLIC_NUM_IRQS),
        VMSTATE_UINT8_ARRAY(ctl, ESP32P4ClicCore, ESP32P4_CLIC_NUM_IRQS),
        VMSTATE_BOOL_ARRAY(line, ESP32P4ClicCore, ESP32P4_CLIC_NUM_IRQS),
        VMSTATE_UINT8(mnlbits, ESP32P4ClicCore),
        VMSTATE_UINT8(nmbits, ESP32P4ClicCore),
        VMSTATE_UINT8(unlbits, ESP32P4ClicCore),
        VMSTATE_BOOL(in_reset, ESP32P4ClicCore),
        VMSTATE_CLOCK(clk, ESP32P4ClicCore),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_esp32p4_clic = {
    .name = TYPE_ESP32P4_CLIC,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = esp32p4_clic_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(core, ESP32P4ClicState, ESP32P4_CLIC_CORES, 1,
                             vmstate_esp32p4_clic_core, ESP32P4ClicCore),
        VMSTATE_END_OF_LIST()
    },
};

static const Property esp32p4_clic_properties[] = {
    DEFINE_PROP_LINK("cpu0", ESP32P4ClicState, core[0].cpu, TYPE_ESP32P4_CPU,
                     ESP32P4CPU *),
    DEFINE_PROP_LINK("cpu1", ESP32P4ClicState, core[1].cpu, TYPE_ESP32P4_CPU,
                     ESP32P4CPU *),
};

static void esp32p4_clic_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = esp32p4_clic_realize;
    dc->vmsd = &vmstate_esp32p4_clic;
    rc->phases.hold = esp32p4_clic_reset_hold;
    rc->phases.exit = esp32p4_clic_reset_exit;
    device_class_set_props(dc, esp32p4_clic_properties);
}

/* [spec:nuos:req:emu.esp32p4.clic] */
static const TypeInfo esp32p4_clic_info = {
    .name          = TYPE_ESP32P4_CLIC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32P4ClicState),
    .instance_init = esp32p4_clic_init,
    .class_init    = esp32p4_clic_class_init,
};

static void esp32p4_clic_register_types(void)
{
    type_register_static(&esp32p4_clic_info);
}
type_init(esp32p4_clic_register_types)
