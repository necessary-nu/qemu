/*
 * ESP32-P4 interrupt matrix (INTMTX)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapter
 * 13 (Interrupt Matrix), and ESP-IDF's interrupt_core0_reg.h and
 * interrupt_core1_reg.h for v3, whose register set this follows where the
 * TRM's register summary leaves registers out.
 *
 * Each HP core has a matrix of its own, core 0's at 0x500D6000 and core
 * 1's at 0x500D6800, with the same registers:
 *
 *   - CORE<n>_<source>_INT_MAP_REG, one per source, at 4 * source for
 *     sources 0 to 127 and at 0x214 to 0x21C for 128 to 130. MAP (5:0)
 *     is the CPU interrupt the source drives, 16 to 47 being the core's
 *     CLIC interrupts of those IDs; 0 to 15 disable the source (0 being
 *     the TRM's recommended value), and 48 to 63, which the TRM calls
 *     invalid, route it nowhere. SRC_PASS_IN_SEC (6) lets the source take
 *     part in remapping, and SRC_IN_SEC_FLAG (7) makes it a secure source.
 *   - INTR_STATUS_REG_0 to _3 (0x200 to 0x20C) and _4 (0x220): each
 *     source's level, a bit per source, whatever its mapping: source n at
 *     bit n % 32 of register n / 32, sources 128 to 130 at bits 0 to 2 of
 *     _4 (include/hw/intc/esp32p4_intmatrix.h says why 130 is at bit 2).
 *   - CLOCK_GATE_REG (0x210): REG_CLK_EN, which forces the register
 *     file's own clock gate on; it has no effect software can see.
 *   - INTR_SIG_IDX_ASSERT_IN_SEC_REG (0x228), INTR_SEC_STATUS_REG (0x22C)
 *     and INTR_SRC_PASS_IN_SEC_STATUS_0 to _4 (0x230 to 0x240), for the
 *     remapping below.
 *   - INTERRUPT_REG_DATE_REG (0x3FC): a read-write 28-bit version, from
 *     reset 0x2502280 as ESP-IDF's v3 header gives it (the TRM's
 *     0x2003020 is an earlier revision's).
 *
 * The other offsets of each 2 KiB have no register: they read as zero and
 * ignore writes. Byte and halfword accesses read and write their bytes of
 * a register.
 *
 * Routing. A source is a level. A core's CLIC interrupt 16 + n is the OR
 * of the sources its matrix maps to it, so that sources can share one;
 * whether the CLIC takes it on a level or an edge is the CLIC's own
 * clicintattr.TRIG. A source mapped in both matrices reaches both cores.
 * The routing is combinational: the registers' bus clock clocks only the
 * registers. While the matrix is held in reset its registers hold their
 * reset values, which map every source to 0, so nothing reaches the
 * cores.
 *
 * Remapping (TRM 13.4.4). A core in machine mode does not take its
 * user-mode interrupts. While a core is in machine mode, a source with
 * SRC_PASS_IN_SEC set mapped to a CLIC interrupt that the core has in
 * user mode is redirected to the interrupt INTR_SIG_IDX_ASSERT_IN_SEC
 * names, if that is one of 16 to 47 and a machine-mode one, so that
 * machine-mode software sees it; INTR_SEC_STATUS has bit n set for each
 * user-mode interrupt 16 + n so redirected, to tell the handler which.
 * Back in user mode, the source drives its own interrupt again. The TRM
 * says the signal is redirected: while it is, the source's own interrupt
 * does not see it. INTR_SRC_PASS_IN_SEC_STATUS_<n> show each source's
 * SRC_PASS_IN_SEC.
 *
 * Secure fields. INTR_SIG_IDX_ASSERT_IN_SEC, SRC_PASS_IN_SEC and
 * SRC_IN_SEC_FLAG take writes only from machine mode, and so does MAP
 * while SRC_IN_SEC_FLAG is set: a user-mode write leaves them as they
 * were. An access's mode is its MemTxAttrs.user, which the HP cores set
 * for their user-mode accesses; an access from outside the cores, such as
 * qtest's or the debugger's, counts as machine mode.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "hw/core/irq.h"
#include "hw/intc/esp32p4_intmatrix.h"
#include "migration/vmstate.h"

#define A_STATUS0           0x200
#define A_STATUS3           0x20c
#define A_CLOCK_GATE        0x210
#define A_MAP128            0x214
#define A_STATUS4           0x220
#define A_SIG_IDX           0x228
#define A_SEC_STATUS        0x22c
#define A_PASS_STATUS0      0x230
#define A_PASS_STATUS4      0x240
#define A_DATE              0x3fc

/* The sources whose mapping registers are at 4 * source. */
#define LOW_SOURCES         128

#define MAP_LINE            0x3f
#define MAP_PASS_IN_SEC     BIT(6)
#define MAP_IN_SEC_FLAG     BIT(7)
#define MAP_BITS            0xff
#define SIG_IDX_MASK        0x3f
#define CLOCK_GATE_CLK_EN   BIT(0)
#define DATE_MASK           0x0fffffff
#define DATE_RESET          0x2502280

#define LINES               ESP32P4_INTMATRIX_LINES
#define LINE0               ESP32P4_INTMATRIX_LINE0

static const char *const intmatrix_core_name[ESP32P4_INTMATRIX_CORES] = {
    "core 0", "core 1",
};

/* The CPU interrupt index, 0 to 31, that @id names; or -1. */
static int intmatrix_line(int id)
{
    return id >= LINE0 && id < LINE0 + LINES ? id - LINE0 : -1;
}

/*
 * Drive core @n's interrupts from the sources' levels and its matrix,
 * redirecting what remapping takes.
 */
/* [spec:nuos:req:emu.esp32p4.interrupt-matrix] */
static void intmatrix_update(ESP32P4IntMatrixState *s, int n)
{
    ESP32P4IntMatrixCore *c = &s->core[n];
    int target = intmatrix_line(c->sig_idx);
    bool remap = c->m_mode && target >= 0 && !(c->line_user & BIT(target));
    uint32_t out = 0;
    uint32_t sec = 0;
    uint32_t changed;

    for (int w = 0; w < ESP32P4_INTMATRIX_SOURCE_WORDS; w++) {
        uint32_t active = s->level[w];

        while (active) {
            int i = w * 32 + ctz32(active);
            uint8_t map = c->map[i];
            int line = intmatrix_line(map & MAP_LINE);

            active &= active - 1;
            if (line < 0) {
                continue;
            }
            if (remap && (map & MAP_PASS_IN_SEC) &&
                (c->line_user & BIT(line))) {
                sec |= BIT(line);
            } else {
                out |= BIT(line);
            }
        }
    }
    if (sec) {
        out |= BIT(target);
    }
    c->sec_status = sec;
    changed = out ^ c->out;
    c->out = out;
    while (changed) {
        int line = ctz32(changed);

        changed &= changed - 1;
        qemu_set_irq(s->irq[n * LINES + line], extract32(out, line, 1));
    }
}

static void intmatrix_update_all(ESP32P4IntMatrixState *s)
{
    for (int n = 0; n < ESP32P4_INTMATRIX_CORES; n++) {
        intmatrix_update(s, n);
    }
}

/* [spec:nuos:req:emu.esp32p4.interrupt-matrix] */
static void intmatrix_set_source(void *opaque, int i, int level)
{
    ESP32P4IntMatrixState *s = opaque;
    uint32_t *word = &s->level[i / 32];
    uint32_t old = *word;

    *word = deposit32(old, i % 32, 1, level != 0);
    if (*word == old) {
        return;
    }
    qemu_set_irq(s->source_out[i], level != 0);
    intmatrix_update_all(s);
}

static void intmatrix_set_m_mode(void *opaque, int n, int level)
{
    ESP32P4IntMatrixState *s = opaque;

    if (s->core[n].m_mode != !!level) {
        s->core[n].m_mode = level;
        intmatrix_update(s, n);
    }
}

static void intmatrix_set_line_user(void *opaque, int i, int level)
{
    ESP32P4IntMatrixState *s = opaque;
    ESP32P4IntMatrixCore *c = &s->core[i / LINES];
    uint32_t old = c->line_user;

    c->line_user = deposit32(old, i % LINES, 1, level != 0);
    if (c->line_user != old) {
        intmatrix_update(s, i / LINES);
    }
}

/* The source whose mapping register is at @off in a core's matrix, or -1. */
static int intmatrix_map_source(hwaddr off)
{
    if (off < A_STATUS0) {
        return off / 4;
    }
    if (off >= A_MAP128 && off < A_STATUS4) {
        return LOW_SOURCES + (off - A_MAP128) / 4;
    }
    return -1;
}

/* INTR_SRC_PASS_IN_SEC_STATUS_@w: SRC_PASS_IN_SEC of sources 32w on. */
static uint32_t intmatrix_pass_status(ESP32P4IntMatrixCore *c, int w)
{
    uint32_t v = 0;

    for (int b = 0; b < 32 && w * 32 + b < ESP32P4_INTMATRIX_NUM_SOURCES;
         b++) {
        if (c->map[w * 32 + b] & MAP_PASS_IN_SEC) {
            v |= BIT(b);
        }
    }
    return v;
}

/* The register at word offset @off of core @c's matrix, if there is one. */
static bool intmatrix_read_reg(ESP32P4IntMatrixState *s,
                               ESP32P4IntMatrixCore *c, hwaddr off,
                               uint32_t *v)
{
    int src = intmatrix_map_source(off);

    if (src >= 0) {
        *v = c->map[src];
        return true;
    }
    switch (off) {
    case A_STATUS0 ... A_STATUS3:
        *v = s->level[(off - A_STATUS0) / 4];
        return true;
    case A_STATUS4:
        *v = s->level[4];
        return true;
    case A_CLOCK_GATE:
        *v = c->clk_en ? CLOCK_GATE_CLK_EN : 0;
        return true;
    case A_SIG_IDX:
        *v = c->sig_idx;
        return true;
    case A_SEC_STATUS:
        *v = c->sec_status;
        return true;
    case A_PASS_STATUS0 ... A_PASS_STATUS4:
        *v = intmatrix_pass_status(c, (off - A_PASS_STATUS0) / 4);
        return true;
    case A_DATE:
        *v = c->date;
        return true;
    default:
        return false;
    }
}

/*
 * Write a source's mapping register: from user mode, the secure fields
 * stay as they are, and MAP too while SRC_IN_SEC_FLAG is set.
 */
/* [spec:nuos:req:emu.esp32p4.interrupt-matrix] */
static void intmatrix_write_map(ESP32P4IntMatrixState *s, int n, int src,
                                uint8_t value, bool user)
{
    ESP32P4IntMatrixCore *c = &s->core[n];
    uint8_t old = c->map[src];

    if (user) {
        uint8_t keep = MAP_PASS_IN_SEC | MAP_IN_SEC_FLAG;

        if (old & MAP_IN_SEC_FLAG) {
            keep |= MAP_LINE;
        }
        if ((value ^ old) & keep) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: user-mode write to %s's "
                          "mapping of source %d leaves its secure fields "
                          "as they were\n", TYPE_ESP32P4_INTMATRIX,
                          intmatrix_core_name[n], src);
        }
        value = (value & ~keep) | (old & keep);
    }
    if ((value & MAP_LINE) >= LINE0 + LINES) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: %s maps source %d to CPU "
                      "interrupt %d, which it does not drive (16 to 47 are "
                      "valid)\n", TYPE_ESP32P4_INTMATRIX,
                      intmatrix_core_name[n], src, value & MAP_LINE);
    }
    c->map[src] = value;
    intmatrix_update(s, n);
}

/*
 * Write @value to the bits @mask selects of the register at word offset
 * @off of core @n's matrix.
 */
/* [spec:nuos:req:emu.esp32p4.interrupt-matrix] */
static void intmatrix_write_reg(ESP32P4IntMatrixState *s, int n, hwaddr off,
                                uint32_t value, uint32_t mask, bool user)
{
    ESP32P4IntMatrixCore *c = &s->core[n];
    int src = intmatrix_map_source(off);
    uint32_t old;

    if (!intmatrix_read_reg(s, c, off, &old)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to %s's matrix at 0x%"
                      HWADDR_PRIx ", which has no register\n",
                      TYPE_ESP32P4_INTMATRIX, intmatrix_core_name[n], off);
        return;
    }
    value = (old & ~mask) | (value & mask);
    if (src >= 0) {
        intmatrix_write_map(s, n, src, value & MAP_BITS, user);
        return;
    }
    switch (off) {
    case A_CLOCK_GATE:
        c->clk_en = value & CLOCK_GATE_CLK_EN;
        break;
    case A_SIG_IDX:
        if (user) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: user-mode write to %s's "
                          "INTR_SIG_IDX_ASSERT_IN_SEC ignored\n",
                          TYPE_ESP32P4_INTMATRIX, intmatrix_core_name[n]);
            break;
        }
        c->sig_idx = value & SIG_IDX_MASK;
        intmatrix_update(s, n);
        break;
    case A_DATE:
        c->date = value & DATE_MASK;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to %s's read-only status "
                      "register at 0x%" HWADDR_PRIx "\n",
                      TYPE_ESP32P4_INTMATRIX, intmatrix_core_name[n], off);
        break;
    }
}

static MemTxResult intmatrix_read(void *opaque, hwaddr addr, uint64_t *data,
                                  unsigned size, MemTxAttrs attrs)
{
    ESP32P4IntMatrixState *s = opaque;
    int n = addr / ESP32P4_INTMATRIX_CORE_SIZE;
    hwaddr off = addr % ESP32P4_INTMATRIX_CORE_SIZE;
    uint32_t v;

    if (!intmatrix_read_reg(s, &s->core[n], off & ~3, &v)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of %s's matrix at 0x%"
                      HWADDR_PRIx ", which has no register\n",
                      TYPE_ESP32P4_INTMATRIX, intmatrix_core_name[n], off);
        v = 0;
    }
    *data = extract32(v, (off & 3) * 8, size * 8);
    return MEMTX_OK;
}

/* [spec:nuos:req:emu.esp32p4.interrupt-matrix] */
static MemTxResult intmatrix_write(void *opaque, hwaddr addr, uint64_t data,
                                   unsigned size, MemTxAttrs attrs)
{
    ESP32P4IntMatrixState *s = opaque;
    int n = addr / ESP32P4_INTMATRIX_CORE_SIZE;
    hwaddr off = addr % ESP32P4_INTMATRIX_CORE_SIZE;
    unsigned shift = (off & 3) * 8;

    intmatrix_write_reg(s, n, off & ~3, data << shift,
                        MAKE_64BIT_MASK(shift, size * 8), attrs.user);
    return MEMTX_OK;
}

static const MemoryRegionOps intmatrix_ops = {
    .read_with_attrs = intmatrix_read,
    .write_with_attrs = intmatrix_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

/*
 * The registers take their reset values; the sources' levels and the
 * cores' modes are the inputs', which the reset leaves alone.
 */
/* [spec:nuos:req:emu.esp32p4.interrupt-matrix] */
static void esp32p4_intmatrix_reset_hold(Object *obj, ResetType type)
{
    ESP32P4IntMatrixState *s = ESP32P4_INTMATRIX(obj);

    for (int n = 0; n < ESP32P4_INTMATRIX_CORES; n++) {
        ESP32P4IntMatrixCore *c = &s->core[n];

        memset(c->map, 0, sizeof(c->map));
        c->sig_idx = 0;
        c->clk_en = true;
        c->date = DATE_RESET;
    }
}

static void esp32p4_intmatrix_reset_exit(Object *obj, ResetType type)
{
    intmatrix_update_all(ESP32P4_INTMATRIX(obj));
}

static void esp32p4_intmatrix_init(Object *obj)
{
    ESP32P4IntMatrixState *s = ESP32P4_INTMATRIX(obj);
    DeviceState *dev = DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &intmatrix_ops, s,
                          "esp32p4.intmtx", ESP32P4_INTMATRIX_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    qdev_init_gpio_in_named(dev, intmatrix_set_source,
                            ESP32P4_INTMATRIX_SOURCE,
                            ESP32P4_INTMATRIX_NUM_SOURCES);
    qdev_init_gpio_in_named(dev, intmatrix_set_m_mode,
                            ESP32P4_INTMATRIX_CORE_M_MODE,
                            ESP32P4_INTMATRIX_CORES);
    qdev_init_gpio_in_named(dev, intmatrix_set_line_user,
                            ESP32P4_INTMATRIX_LINE_USER,
                            ESP32P4_INTMATRIX_CORES * LINES);
    qdev_init_gpio_out_named(dev, s->irq, ESP32P4_INTMATRIX_IRQ,
                             ESP32P4_INTMATRIX_CORES * LINES);
    qdev_init_gpio_out_named(dev, s->source_out,
                             ESP32P4_INTMATRIX_SOURCE_OUT,
                             ESP32P4_INTMATRIX_NUM_SOURCES);
    /* The cores come out of reset in machine mode. */
    for (int n = 0; n < ESP32P4_INTMATRIX_CORES; n++) {
        s->core[n].m_mode = true;
    }
}

static const VMStateDescription vmstate_esp32p4_intmatrix_core = {
    .name = TYPE_ESP32P4_INTMATRIX "/core",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8_ARRAY(map, ESP32P4IntMatrixCore,
                            ESP32P4_INTMATRIX_NUM_SOURCES),
        VMSTATE_UINT8(sig_idx, ESP32P4IntMatrixCore),
        VMSTATE_BOOL(clk_en, ESP32P4IntMatrixCore),
        VMSTATE_UINT32(date, ESP32P4IntMatrixCore),
        VMSTATE_BOOL(m_mode, ESP32P4IntMatrixCore),
        VMSTATE_UINT32(line_user, ESP32P4IntMatrixCore),
        VMSTATE_UINT32(out, ESP32P4IntMatrixCore),
        VMSTATE_UINT32(sec_status, ESP32P4IntMatrixCore),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_esp32p4_intmatrix = {
    .name = TYPE_ESP32P4_INTMATRIX,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(level, ESP32P4IntMatrixState,
                             ESP32P4_INTMATRIX_SOURCE_WORDS),
        VMSTATE_STRUCT_ARRAY(core, ESP32P4IntMatrixState,
                             ESP32P4_INTMATRIX_CORES, 1,
                             vmstate_esp32p4_intmatrix_core,
                             ESP32P4IntMatrixCore),
        VMSTATE_END_OF_LIST()
    },
};

static void esp32p4_intmatrix_class_init(ObjectClass *klass,
                                         const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->vmsd = &vmstate_esp32p4_intmatrix;
    rc->phases.hold = esp32p4_intmatrix_reset_hold;
    rc->phases.exit = esp32p4_intmatrix_reset_exit;
}

/* [spec:nuos:req:emu.esp32p4.interrupt-matrix] */
static const TypeInfo esp32p4_intmatrix_info = {
    .name          = TYPE_ESP32P4_INTMATRIX,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32P4IntMatrixState),
    .instance_init = esp32p4_intmatrix_init,
    .class_init    = esp32p4_intmatrix_class_init,
};

static void esp32p4_intmatrix_register_types(void)
{
    type_register_static(&esp32p4_intmatrix_info);
}
type_init(esp32p4_intmatrix_register_types)
