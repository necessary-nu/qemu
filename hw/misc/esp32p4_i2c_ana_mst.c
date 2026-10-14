/*
 * ESP32-P4 analog I2C controller (LP_I2C_ANA_MST)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapter
 * 48 (Analog I2C Controller); ESP-IDF's lp_i2c_ana_mst_reg.h for v3 and
 * hal/esp32p4 regi2c_impl.c and regi2c_ctrl_ll.h, which show how software
 * drives it.
 *
 * Two I2C masters configure the analog blocks (esp32p4_ana.c), each
 * reaching the slaves ANA_CONF2 hands it: a slave whose bit is set
 * answers master 0 (I2C0), one whose bit is clear master 1 (I2C1). Writing
 * a master's CTRL register starts a transaction: to the slave in bits
 * 7:0, at the register in bits 15:8, writing the data in bits 23:16 if
 * bit 24 is set and reading into them if it is clear. The TRM gives bit
 * 24 the other way round; ESP-IDF, whose regi2c functions run on the
 * chip, sets it to write. BUSY reads 1 until the transaction is over.
 * A slave that is absent or on the other master does not acknowledge:
 * nothing is written, and a read gives 0.
 *
 * A transaction takes as long as its bits on the bus: a write the start
 * condition, the slave address, register and data, each with its
 * acknowledge, and the stop condition, 29 bit times; a read 39, with the
 * repeated start and second address. Each bit lasts SDA_SIDE_GUARD, then
 * SCL_PULSE_DUR, then SDA_SIDE_GUARD cycles of LP_FAST_CLK, or of
 * PLL_F160M_CLK with CLK_I2C_MST_SEL_160M set, at least one in all. While
 * that clock or the controller's own (LPPERI_CK_EN_LP_I2CMST) is stopped,
 * a transaction does not advance.
 *
 * ANA_CONF0, ANA_CONF1, the masters' CONF registers, the burst
 * registers, HW_I2C_CTRL, through which the PMU's own transactions are
 * timed, and NOUSE are undocumented and hold what is written; their
 * status fields read 0, and the burst mode they configure is not
 * modelled.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/host-utils.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/gpio/esp32_line.h"
#include "hw/misc/esp32p4_i2c_ana_mst.h"
#include "migration/vmstate.h"

enum {
    R_I2C0_CTRL,
    R_I2C1_CTRL,
    R_I2C0_CONF,
    R_I2C1_CONF,
    R_BURST_CONF,
    R_BURST_STATUS,
    R_ANA_CONF0,
    R_ANA_CONF1,
    R_ANA_CONF2,
    R_I2C0_CTRL1,
    R_I2C1_CTRL1,
    R_HW_I2C_CTRL,
    R_NOUSE,
    R_CLK160M,
    R_DATE,
};

#define CTRL_SLAVE_SHIFT  0
#define CTRL_REG_SHIFT    8
#define CTRL_DATA_SHIFT   16
#define CTRL_WRITE        BIT(24)
#define CTRL_BUSY         BIT(25)

#define CTRL1_SCL_PULSE_DUR(v)   extract32((v), 0, 6)
#define CTRL1_SDA_SIDE_GUARD(v)  extract32((v), 6, 5)

#define WRITE_BITS 29
#define READ_BITS  39

static const struct {
    const char *name;
    uint32_t reset;
    uint32_t rw;
} esp32p4_i2c_ana_mst_regs[ESP32P4_I2C_ANA_MST_NREGS] = {
    [R_I2C0_CTRL]    = { "I2C0_CTRL", 0, 0x01ffffff },
    [R_I2C1_CTRL]    = { "I2C1_CTRL", 0, 0x01ffffff },
    [R_I2C0_CONF]    = { "I2C0_CONF", 0, 0x00ffffff },
    [R_I2C1_CONF]    = { "I2C1_CONF", 0, 0x00ffffff },
    [R_BURST_CONF]   = { "I2C_BURST_CONF", 0, 0xffffffff },
    [R_BURST_STATUS] = { "I2C_BURST_STATUS", 0x40000000, 0xfff00000 },
    [R_ANA_CONF0]    = { "ANA_CONF0", 0, 0x00ffffff },
    [R_ANA_CONF1]    = { "ANA_CONF1", 0, 0x00ffffff },
    [R_ANA_CONF2]    = { "ANA_CONF2", 0, 0x00ffffff },
    [R_I2C0_CTRL1]   = { "I2C0_CTRL1", 0x42, 0x000007ff },
    [R_I2C1_CTRL1]   = { "I2C1_CTRL1", 0x42, 0x000007ff },
    [R_HW_I2C_CTRL]  = { "HW_I2C_CTRL", 0x42, 0x00000fff },
    [R_NOUSE]        = { "NOUSE", 0, 0xffffffff },
    [R_CLK160M]      = { "CLK160M", 0, 0x00000001 },
    [R_DATE]         = { "DATE", 0x02304230, 0x1fffffff },
};

static const uint8_t esp32p4_i2c_ana_mst_conf2_bit[ESP32P4_ANA_NUM_SLAVES] = {
    [ESP32P4_ANA_SLAVE_SYS_PLL]  = 5,
    [ESP32P4_ANA_SLAVE_SDIO_PLL] = 6,
    [ESP32P4_ANA_SLAVE_SAR]      = 7,
    [ESP32P4_ANA_SLAVE_PLLA]     = 8,
    [ESP32P4_ANA_SLAVE_MSPI]     = 9,
    [ESP32P4_ANA_SLAVE_DIG_REG]  = 10,
    [ESP32P4_ANA_SLAVE_CPU_PLL]  = 11,
    [ESP32P4_ANA_SLAVE_BIAS]     = 12,
};

/*
 * The edges of the clock the masters count, from @now if they are not
 * given; stopped while the bus clock is.
 */
static ESP32P4ClkEdges esp32p4_i2c_ana_mst_edges(ESP32P4I2CAnaMstState *s,
                                                 int64_t now)
{
    if (!clock_is_enabled(s->bus)) {
        return esp32p4_clk_edges_stopped();
    }
    return esp32p4_clk_edges_or_at((s->regs[R_CLK160M] & 1) ? s->pll_f160m
                                                             : s->lp_fast,
                                   now);
}

/*
 * Count the cycles master @n's transaction has had since the clock last
 * changed, at the clock's old period, and time the rest at its current
 * one.
 */
static void esp32p4_i2c_ana_mst_schedule(ESP32P4I2CAnaMstState *s, int n)
{
    int64_t now = esp32_line_time();
    uint64_t done;

    if (!s->busy[n]) {
        return;
    }
    done = esp32p4_clk_count_at(&s->xfer[n], now);
    s->xfer_left[n] -= MIN(done, s->xfer_left[n]);
    s->xfer[n].edges = esp32p4_i2c_ana_mst_edges(s, now);
    esp32p4_clk_count_rebase(&s->xfer[n], now);
    if (!esp32p4_clk_count_running(&s->xfer[n])) {
        timer_del(s->timer[n]);
        return;
    }
    timer_mod(s->timer[n], esp32p4_clk_count_time(&s->xfer[n],
                                                  s->xfer_left[n]));
}

/*
 * [spec:nuos:req:emu.esp32p4.clocks]
 * Master @n's transaction is over: the slave takes the write, or the
 * read lands in the data field.
 */
static void esp32p4_i2c_ana_mst_done(ESP32P4I2CAnaMstState *s, int n)
{
    uint32_t ctrl = s->regs[R_I2C0_CTRL + n];
    uint8_t addr = extract32(ctrl, CTRL_SLAVE_SHIFT, 8);
    uint8_t reg = extract32(ctrl, CTRL_REG_SHIFT, 8);
    uint8_t data = extract32(ctrl, CTRL_DATA_SHIFT, 8);
    int slave = esp32p4_ana_slave(addr);
    bool on_master, acked = false;

    s->busy[n] = false;
    timer_del(s->timer[n]);
    if (slave >= 0) {
        on_master = extract32(s->regs[R_ANA_CONF2],
                              esp32p4_i2c_ana_mst_conf2_bit[slave], 1) == !n;
        if (on_master) {
            acked = (ctrl & CTRL_WRITE) ?
                    esp32p4_ana_write(s->ana, slave, reg, data) :
                    esp32p4_ana_read(s->ana, slave, reg, &data);
        }
    }
    if (!acked) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: I2C%d: no analog slave at 0x%02x register %u "
                      "answers this master\n", TYPE_ESP32P4_I2C_ANA_MST, n,
                      addr, reg);
        data = 0;
    }
    if (!(ctrl & CTRL_WRITE)) {
        s->regs[R_I2C0_CTRL + n] = deposit32(ctrl, CTRL_DATA_SHIFT, 8, data);
    }
}

static void esp32p4_i2c_ana_mst_done_at(void *opaque)
{
    ESP32P4I2CAnaMstTimer *t = opaque;

    esp32p4_i2c_ana_mst_done(t->s, t->n);
}

/* When master @n's transaction ends, while it runs. */
static int64_t esp32p4_i2c_ana_mst_due(ESP32P4I2CAnaMstState *s, int n)
{
    return esp32p4_clk_count_time(&s->xfer[n], s->xfer_left[n]);
}

/*
 * A transaction ends at its time however late its timer runs: the slave
 * takes a write then, a PLL relocking from it (esp32_line_run_at()).
 */
static void esp32p4_i2c_ana_mst_timer(void *opaque)
{
    ESP32P4I2CAnaMstTimer *t = opaque;

    esp32_line_run_at(esp32p4_i2c_ana_mst_due(t->s, t->n),
                      esp32p4_i2c_ana_mst_done_at, t);
}

/*
 * [spec:nuos:req:emu.esp32p4.clocks]
 * End the transactions whose bits have run by the time of the access or
 * clock change being made, each at its own time: without icount their
 * timers can run well after that, while software polls BUSY, as
 * ESP-IDF's regi2c functions do, or reads the data a read brought.
 */
static void esp32p4_i2c_ana_mst_catch_up(ESP32P4I2CAnaMstState *s)
{
    int64_t now = esp32_line_time();

    for (;;) {
        int64_t due = now;
        int next = -1;

        for (int n = 0; n < ESP32P4_I2C_ANA_MST_MASTERS; n++) {
            if (timer_pending(s->timer[n]) &&
                esp32p4_i2c_ana_mst_due(s, n) <= due &&
                (next < 0 || esp32p4_i2c_ana_mst_due(s, n) < due)) {
                next = n;
                due = esp32p4_i2c_ana_mst_due(s, n);
            }
        }
        if (next < 0) {
            return;
        }
        timer_del(s->timer[next]);
        esp32_line_run_at(due, esp32p4_i2c_ana_mst_done_at,
                          &s->timer_ctx[next]);
    }
}

/* [spec:nuos:req:emu.esp32p4.clocks] */
static void esp32p4_i2c_ana_mst_start(ESP32P4I2CAnaMstState *s, int n)
{
    uint32_t ctrl1 = s->regs[R_I2C0_CTRL1 + n];
    uint32_t bit = CTRL1_SDA_SIDE_GUARD(ctrl1) * 2 +
                   CTRL1_SCL_PULSE_DUR(ctrl1);
    bool write = s->regs[R_I2C0_CTRL + n] & CTRL_WRITE;

    s->busy[n] = true;
    s->xfer_left[n] = (uint64_t)MAX(bit, 1) * (write ? WRITE_BITS : READ_BITS);
    s->xfer[n].edges = esp32p4_clk_edges_stopped();
    esp32p4_clk_count_rebase(&s->xfer[n], esp32_line_time());
    esp32p4_i2c_ana_mst_schedule(s, n);
}

static uint64_t esp32p4_i2c_ana_mst_read(void *opaque, hwaddr addr,
                                         unsigned size)
{
    ESP32P4I2CAnaMstState *s = opaque;
    unsigned idx = addr / 4;
    uint32_t v;

    if (idx >= ESP32P4_I2C_ANA_MST_NREGS) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of reserved address "
                      "0x%" HWADDR_PRIx "\n", TYPE_ESP32P4_I2C_ANA_MST, addr);
        return 0;
    }
    esp32p4_i2c_ana_mst_catch_up(s);
    v = s->regs[idx];
    if ((idx == R_I2C0_CTRL || idx == R_I2C1_CTRL) &&
        s->busy[idx - R_I2C0_CTRL]) {
        v |= CTRL_BUSY;
    }
    return extract32(v, (addr & 3) * 8, size * 8);
}

/* [spec:nuos:req:emu.esp32p4.clocks] */
static void esp32p4_i2c_ana_mst_write(void *opaque, hwaddr addr,
                                      uint64_t value, unsigned size)
{
    ESP32P4I2CAnaMstState *s = opaque;
    unsigned idx = addr / 4;
    uint32_t mask, v;
    int n;

    if (idx >= ESP32P4_I2C_ANA_MST_NREGS) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to reserved address "
                      "0x%" HWADDR_PRIx "\n", TYPE_ESP32P4_I2C_ANA_MST, addr);
        return;
    }
    esp32p4_i2c_ana_mst_catch_up(s);
    mask = deposit32(0, (addr & 3) * 8, size * 8, UINT32_MAX) &
           esp32p4_i2c_ana_mst_regs[idx].rw;
    v = deposit32(0, (addr & 3) * 8, size * 8, value);

    switch (idx) {
    case R_I2C0_CTRL:
    case R_I2C1_CTRL:
        n = idx - R_I2C0_CTRL;
        if (s->busy[n]) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: write to I2C%d_CTRL while "
                          "its transaction is in progress\n",
                          TYPE_ESP32P4_I2C_ANA_MST, n);
            return;
        }
        s->regs[idx] = (s->regs[idx] & ~mask) | (v & mask);
        esp32p4_i2c_ana_mst_start(s, n);
        return;
    case R_BURST_CONF:
        if (v & mask) {
            qemu_log_mask(LOG_UNIMP, "%s: the burst mode is not modelled\n",
                          TYPE_ESP32P4_I2C_ANA_MST);
        }
        break;
    default:
        break;
    }
    s->regs[idx] = (s->regs[idx] & ~mask) | (v & mask);
    if (idx == R_CLK160M) {
        for (n = 0; n < ESP32P4_I2C_ANA_MST_MASTERS; n++) {
            esp32p4_i2c_ana_mst_schedule(s, n);
        }
    }
}

static const MemoryRegionOps esp32p4_i2c_ana_mst_ops = {
    .read = esp32p4_i2c_ana_mst_read,
    .write = esp32p4_i2c_ana_mst_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

static void esp32p4_i2c_ana_mst_clock_update(void *opaque, ClockEvent event)
{
    ESP32P4I2CAnaMstState *s = opaque;
    int n;

    esp32p4_i2c_ana_mst_catch_up(s);
    for (n = 0; n < ESP32P4_I2C_ANA_MST_MASTERS; n++) {
        esp32p4_i2c_ana_mst_schedule(s, n);
    }
}

/* [spec:nuos:req:emu.esp32p4.clocks] */
static void esp32p4_i2c_ana_mst_reset_hold(Object *obj, ResetType type)
{
    ESP32P4I2CAnaMstState *s = ESP32P4_I2C_ANA_MST(obj);
    int n;

    for (n = 0; n < ESP32P4_I2C_ANA_MST_NREGS; n++) {
        s->regs[n] = esp32p4_i2c_ana_mst_regs[n].reset;
    }
    for (n = 0; n < ESP32P4_I2C_ANA_MST_MASTERS; n++) {
        s->busy[n] = false;
        timer_del(s->timer[n]);
        s->xfer_left[n] = 0;
        s->xfer[n].edges = esp32p4_clk_edges_stopped();
    }
}

static void esp32p4_i2c_ana_mst_init(Object *obj)
{
    ESP32P4I2CAnaMstState *s = ESP32P4_I2C_ANA_MST(obj);
    DeviceState *dev = DEVICE(obj);
    int n;

    memory_region_init_io(&s->iomem, obj, &esp32p4_i2c_ana_mst_ops, s,
                          "esp32p4.lp-i2c-ana-mst", ESP32P4_I2C_ANA_MST_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    s->bus = qdev_init_clock_in(dev, "bus", esp32p4_i2c_ana_mst_clock_update,
                                s, ClockUpdate);
    s->lp_fast = qdev_init_clock_in(dev, "lp-fast",
                                    esp32p4_i2c_ana_mst_clock_update, s,
                                    ClockUpdate);
    s->pll_f160m = qdev_init_clock_in(dev, "pll-f160m",
                                      esp32p4_i2c_ana_mst_clock_update, s,
                                      ClockUpdate);
    for (n = 0; n < ESP32P4_I2C_ANA_MST_MASTERS; n++) {
        s->timer_ctx[n].s = s;
        s->timer_ctx[n].n = n;
        s->timer[n] = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                   esp32p4_i2c_ana_mst_timer,
                                   &s->timer_ctx[n]);
    }
}

static void esp32p4_i2c_ana_mst_realize(DeviceState *dev, Error **errp)
{
    ESP32P4I2CAnaMstState *s = ESP32P4_I2C_ANA_MST(dev);

    if (!s->ana) {
        error_setg(errp, "%s: the \"ana\" link is not set",
                   TYPE_ESP32P4_I2C_ANA_MST);
    }
}

static const VMStateDescription vmstate_esp32p4_i2c_ana_mst = {
    .name = TYPE_ESP32P4_I2C_ANA_MST,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_CLOCK(bus, ESP32P4I2CAnaMstState),
        VMSTATE_CLOCK(lp_fast, ESP32P4I2CAnaMstState),
        VMSTATE_CLOCK(pll_f160m, ESP32P4I2CAnaMstState),
        VMSTATE_UINT32_ARRAY(regs, ESP32P4I2CAnaMstState,
                             ESP32P4_I2C_ANA_MST_NREGS),
        VMSTATE_BOOL_ARRAY(busy, ESP32P4I2CAnaMstState,
                           ESP32P4_I2C_ANA_MST_MASTERS),
        VMSTATE_TIMER_PTR_ARRAY(timer, ESP32P4I2CAnaMstState,
                                ESP32P4_I2C_ANA_MST_MASTERS),
        VMSTATE_UINT64_ARRAY(xfer_left, ESP32P4I2CAnaMstState,
                             ESP32P4_I2C_ANA_MST_MASTERS),
        VMSTATE_STRUCT_ARRAY(xfer, ESP32P4I2CAnaMstState,
                             ESP32P4_I2C_ANA_MST_MASTERS, 0,
                             vmstate_esp32p4_clk_count, ESP32P4ClkCount),
        VMSTATE_END_OF_LIST()
    },
};

static const Property esp32p4_i2c_ana_mst_properties[] = {
    DEFINE_PROP_LINK("ana", ESP32P4I2CAnaMstState, ana, TYPE_ESP32P4_ANA,
                     ESP32P4AnaState *),
};

static void esp32p4_i2c_ana_mst_class_init(ObjectClass *klass,
                                           const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32p4_i2c_ana_mst_reset_hold;
    dc->realize = esp32p4_i2c_ana_mst_realize;
    dc->vmsd = &vmstate_esp32p4_i2c_ana_mst;
    device_class_set_props(dc, esp32p4_i2c_ana_mst_properties);
}

/* [spec:nuos:req:emu.esp32p4.clocks] */
static const TypeInfo esp32p4_i2c_ana_mst_info = {
    .name          = TYPE_ESP32P4_I2C_ANA_MST,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32P4I2CAnaMstState),
    .instance_init = esp32p4_i2c_ana_mst_init,
    .class_init    = esp32p4_i2c_ana_mst_class_init,
};

static void esp32p4_i2c_ana_mst_register_types(void)
{
    type_register_static(&esp32p4_i2c_ana_mst_info);
}
type_init(esp32p4_i2c_ana_mst_register_types)
