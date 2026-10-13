/*
 * ESP32 eFuse emulation
 *
 * Copyright (c) 2019 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "system/system.h"
#include "chardev/char-fe.h"
#include "hw/core/registerfields.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/nvram/esp32_efuse.h"

/* EFUSE_CLK and EFUSE_DAC_CONF out of reset (TRM v5.8, registers 5.21, 5.28) */
#define EFUSE_CLK_RESET         0x4052
#define EFUSE_DAC_CONF_RESET    40

static void esp32_efuse_read_op(Esp32EfuseState *s);
static void esp32_efuse_program_op(Esp32EfuseState *s);
static void esp32_efuse_update_irq(Esp32EfuseState *s);
static void esp32_efuse_op_start(Esp32EfuseState *s, uint64_t cycles);
static uint64_t esp32_efuse_read_cycles(Esp32EfuseState *s);
static uint64_t esp32_efuse_program_cycles(Esp32EfuseState *s);

static void esp32_efuse_timer_cb(void *opaque);

/*
 * [spec:nuos:req:emu.esp32.clock-gating]
 * End the running read or program command if its cycles have run by the
 * access being made: without icount its timer can run well after that,
 * while software polls EFUSE_CMD, as the ROM and ESP-IDF do, or reads
 * INT_RAW. Only a timer that has come is run.
 */
static void esp32_efuse_catch_up(Esp32EfuseState *s)
{
    if (timer_pending(&s->op_timer) &&
        timer_expire_time_ns(&s->op_timer) <=
        qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)) {
        timer_del(&s->op_timer);
        esp32_efuse_timer_cb(s);
    }
}

static uint64_t esp32_efuse_read(void *opaque, hwaddr addr, unsigned int size)
{
    Esp32EfuseState *s = ESP32_EFUSE(opaque);
    uint64_t r = 0;
    int idx;

    esp32_efuse_catch_up(s);
    switch (addr) {
    case A_EFUSE_BLK0_RDATA0 ... A_EFUSE_BLK0_WDATA0 - 4:
        idx = (addr - A_EFUSE_BLK0_RDATA0) / 4;
        r = s->efuse_rd.blk0[idx] & ~(s->efuse_rd_dis.blk0[idx]);
        break;
    case A_EFUSE_BLK0_WDATA0 ... A_EFUSE_BLK1_RDATA0 - 4:
        r = s->efuse_wr.blk0[(addr - A_EFUSE_BLK0_WDATA0) / 4];
        break;
    case A_EFUSE_BLK1_RDATA0 ... A_EFUSE_BLK2_RDATA0 - 4:
        idx = (addr - A_EFUSE_BLK1_RDATA0) / 4;
        r = s->efuse_rd.blk1[idx] & ~(s->efuse_rd_dis.blk1[idx]);
        break;
    case A_EFUSE_BLK2_RDATA0 ... A_EFUSE_BLK3_RDATA0 - 4:
        idx = (addr - A_EFUSE_BLK2_RDATA0) / 4;
        r = s->efuse_rd.blk2[idx] & ~(s->efuse_rd_dis.blk2[idx]);
        break;
    case A_EFUSE_BLK3_RDATA0 ... A_EFUSE_BLK1_WDATA0 - 4:
        idx = (addr - A_EFUSE_BLK3_RDATA0) / 4;
        r = s->efuse_rd.blk3[idx] & ~(s->efuse_rd_dis.blk3[idx]);
        break;
    case A_EFUSE_BLK1_WDATA0 ... A_EFUSE_BLK2_WDATA0 - 4:
        r = s->efuse_wr.blk1[(addr - A_EFUSE_BLK1_WDATA0) / 4];
        break;
    case A_EFUSE_BLK2_WDATA0 ... A_EFUSE_BLK3_WDATA0 - 4:
        r = s->efuse_wr.blk2[(addr - A_EFUSE_BLK2_WDATA0) / 4];
        break;
    case A_EFUSE_BLK3_WDATA0 ... A_EFUSE_CLK - 4:
        r = s->efuse_wr.blk3[(addr - A_EFUSE_BLK3_WDATA0) / 4];
        break;
    case A_EFUSE_CLK:
        r = s->clk_reg;
        break;
    case A_EFUSE_CONF:
        r = s->conf_reg;
        break;
    case A_EFUSE_CMD:
        r = s->cmd_reg;
        break;
    case A_EFUSE_STATUS:
        r = 0;
        break;
    case A_EFUSE_DAC_CONF:
        r = s->dac_conf_reg;
        break;
    case A_EFUSE_INT_RAW:
        r = s->int_raw_reg;
        break;
    case A_EFUSE_INT_ST:
        r = s->int_st_reg;
        break;
    case A_EFUSE_INT_ENA:
        r = s->int_ena_reg;
        break;
    case A_EFUSE_DEC_STATUS:
        r = 0;
        break;
    case A_EFUSE_DATE:
        r = 0x16042600;
    }
    return r;
}

static void esp32_efuse_write(void *opaque, hwaddr addr,
                       uint64_t value, unsigned int size)
{
    Esp32EfuseState *s = ESP32_EFUSE(opaque);

    esp32_efuse_catch_up(s);
    switch (addr) {
    case A_EFUSE_CLK:
        s->clk_reg = value;
        break;
    case A_EFUSE_CONF:
        s->conf_reg = value;
        break;
    case A_EFUSE_CMD:
        if ((value & EFUSE_READ) && s->conf_reg == EFUSE_READ_OP_CODE) {
            esp32_efuse_read_op(s);
        }
        if ((value & EFUSE_PGM) && s->conf_reg == EFUSE_PGM_OP_CODE) {
            esp32_efuse_program_op(s);
        }
        break;
    case A_EFUSE_DAC_CONF:
        s->dac_conf_reg = value;
        break;
    case A_EFUSE_INT_ENA:
        s->int_ena_reg = value;
        esp32_efuse_update_irq(s);
        break;
    case A_EFUSE_INT_CLR:
        s->int_raw_reg &= ~value;
        esp32_efuse_update_irq(s);
        break;
    case A_EFUSE_BLK0_WDATA0 ... A_EFUSE_BLK1_RDATA0 - 4:
        s->efuse_wr.blk0[(addr - A_EFUSE_BLK0_WDATA0) / 4] = value;
        break;
    case A_EFUSE_BLK1_WDATA0 ... A_EFUSE_BLK2_WDATA0 - 4:
        s->efuse_wr.blk1[(addr - A_EFUSE_BLK1_WDATA0) / 4] = value;
        break;
    case A_EFUSE_BLK2_WDATA0 ... A_EFUSE_BLK3_WDATA0 - 4:
        s->efuse_wr.blk2[(addr - A_EFUSE_BLK2_WDATA0) / 4] = value;
        break;
    case A_EFUSE_BLK3_WDATA0 ... A_EFUSE_CLK - 4:
        s->efuse_wr.blk3[(addr - A_EFUSE_BLK3_WDATA0) / 4] = value;
        break;
    }
}

#define APPLY_DIS(rdwr_, ctrl_field_, dest_field_) \
    if (s->efuse_ ## rdwr_ .blk0_d0.ctrl_field_) { \
        memset(&s->efuse_ ## rdwr_ ## _dis.dest_field_, 0xff, sizeof(s->efuse_ ## rdwr_ ## _dis.dest_field_)); \
    }

#define APPLY_DIS_FIELD(rdwr_, ctrl_field_, dest_field_) \
    if (s->efuse_ ## rdwr_ .blk0_d0.ctrl_field_) { \
        s->efuse_ ## rdwr_ ## _dis.dest_field_ = 0; \
        s->efuse_ ## rdwr_ ## _dis.dest_field_ -= 1; \
    }


static void esp32_efuse_read_op(Esp32EfuseState *s)
{
    s->cmd_reg = EFUSE_READ;
    if (s->blk != NULL) {
        int ret = blk_pread(s->blk, 0, sizeof(s->efuse_rd), &s->efuse_rd, 0);
        if (ret < 0) {
            error_report("%s: failed to read the block device (%d)", __func__, ret);
            return;
        }
    }

    memset(&s->efuse_rd_dis, 0, sizeof(s->efuse_rd_dis));
    memset(&s->efuse_wr_dis, 0, sizeof(s->efuse_wr_dis));

    APPLY_DIS(rd, rd_dis_blk1, blk1);
    APPLY_DIS(rd, rd_dis_blk2, blk2);
    APPLY_DIS(rd, rd_dis_blk3, blk3);
    APPLY_DIS_FIELD(rd, rd_dis_blk0_partial, blk0_d5.flash_crypt_config);
    APPLY_DIS_FIELD(rd, rd_dis_blk0_partial, blk0_d6.coding_scheme);
    APPLY_DIS_FIELD(rd, rd_dis_blk0_partial, blk0_d6.key_status);

    APPLY_DIS(wr, wr_dis_blk1, blk1);
    APPLY_DIS(wr, wr_dis_blk2, blk2);
    APPLY_DIS(wr, wr_dis_blk3, blk3);

    /* Other wr_dis bits are not emulated, but can be handled here if necessary */

    esp32_efuse_op_start(s, esp32_efuse_read_cycles(s));
    qemu_irq_pulse(s->efuse_update_gpio);
}

static void esp32_efuse_program_op(Esp32EfuseState *s)
{
    s->cmd_reg = EFUSE_PGM;

    Esp32EfuseRegs result;
    uint32_t* dst = (uint32_t*) &result;
    uint32_t* rd = (uint32_t*) &s->efuse_rd;
    uint32_t* wr = (uint32_t*) &s->efuse_wr;
    uint32_t* wr_dis = (uint32_t*) &s->efuse_wr_dis;
    for (int i = 0; i < sizeof(result) / sizeof(uint32_t); ++i) {
        uint32_t wr_word = wr[i];
        uint32_t wr_dis_word = wr_dis[i];
        uint32_t rd_word = rd[i];
        dst[i] = (wr_word & (~wr_dis_word)) | rd_word;
    }

    if (s->blk != NULL) {
        int ret = blk_pwrite(s->blk, 0, sizeof(result), &result, 0);
        if (ret < 0) {
            error_report("%s: failed to write to block device (%d)", __func__, ret);
        }
    }

    esp32_efuse_op_start(s, esp32_efuse_program_cycles(s));
}

static void esp32_efuse_update_irq(Esp32EfuseState *s)
{
    s->int_st_reg = s->int_ena_reg & s->int_raw_reg;
    int level = s->int_st_reg != 0;
    qemu_set_irq(s->irq, level);
}

/*
 * How long a command takes
 * ========================
 *
 * The TRM (v5.8, 5.3.2 and table 5.3-4) has software set three fields
 * for the APB_CLK frequency before it programs the eFuses:
 *
 *                       26 MHz   40 MHz   80 MHz   reset
 *   EFUSE_CLK_SEL0        250      160       80     0x52
 *   EFUSE_CLK_SEL1        255      255      128     0x40
 *   EFUSE_DAC_CLK_DIV      52       80      100       40
 *
 * TRM v3.0 gives DAC_CLK_DIV 160 at 80 MHz, which is APB_CLK / 500 kHz as
 * at the other two; v5.8 and ESP-IDF's efuse_hal_set_timing() give 100.
 * The model takes whatever software writes. That the values scale with
 * the frequency makes them counts of APB_CLK cycles, the controller's
 * clock. Neither the TRM nor ESP-IDF says what they time, or how long a
 * read or a program takes. The boot ROM (v3, "ets Jul 29 2019") never
 * writes them, so the reads it makes as it boots
 * (_ResetHandler_efuse_check_patch's, and ets_efuse_read_op()'s) run at
 * the reset values.
 *
 * The model's interpretation, to be confirmed on silicon: CLK_SEL0 and
 * CLK_SEL1 are the two phases of the clock the array is accessed by, so
 * that an access takes CLK_SEL0 + CLK_SEL1 cycles. A read accesses each
 * of the array's 31 words (BLOCK0's 7, BLOCK1 to BLOCK3's 8 each) once.
 * A program raises the programming voltage through the DAC, DAC_CLK_DIV
 * cycles a step for 255 steps (an 8-bit DAC; EFUSE_DAC_NUM, the steps of
 * the same ramp on the later chips, resets to 255), accesses each of the
 * array's 992 bits once, and lowers the voltage the same way. At the
 * reset values and a 40 MHz APB_CLK a read takes 4526 cycles, 113 us; at
 * table 5.3-4's 40 MHz values a read takes 322 us and a program 11.3 ms.
 * On a board, CCOUNT counted across ets_efuse_read_op() with the CPU on
 * the 40 MHz XTAL_CLK, at the reset values and at the 40 MHz values,
 * checks this.
 *
 * A command takes the fields as it starts, and runs on APB_CLK as DPORT
 * gates it, standing still while that is off.
 *
 * The boot ROM reads EFUSE_CMD back one instruction after it issues a
 * read and software-resets the chip unless it is still busy: a bound of
 * a few CPU cycles, which silicon and icount keep, but the host's clock
 * does not when the host holds the CPU thread up between the two for
 * longer than the read.
 */
#define EFUSE_ARRAY_WORDS   31
#define EFUSE_ARRAY_BITS    (EFUSE_ARRAY_WORDS * 32)
#define EFUSE_DAC_STEPS     255

/* APB_CLK cycles an access to the array takes */
static uint64_t esp32_efuse_access_cycles(Esp32EfuseState *s)
{
    uint64_t c = FIELD_EX32(s->clk_reg, EFUSE_CLK, SEL0) +
                 FIELD_EX32(s->clk_reg, EFUSE_CLK, SEL1);

    if (c == 0) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: a command with EFUSE_CLK_SEL0 and EFUSE_CLK_SEL1 "
                      "both 0\n", TYPE_ESP32_EFUSE);
    }
    return c;
}

static uint64_t esp32_efuse_read_cycles(Esp32EfuseState *s)
{
    return EFUSE_ARRAY_WORDS * esp32_efuse_access_cycles(s);
}

static uint64_t esp32_efuse_program_cycles(Esp32EfuseState *s)
{
    uint64_t ramp = EFUSE_DAC_STEPS *
                    (uint64_t)FIELD_EX32(s->dac_conf_reg, EFUSE_DAC_CONF,
                                         DAC_CLK_DIV);

    return 2 * ramp + EFUSE_ARRAY_BITS * esp32_efuse_access_cycles(s);
}

/* Arm the timer for the rest of the running command, if APB_CLK runs. */
static void esp32_efuse_op_schedule(Esp32EfuseState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    s->op_anchor_ns = now;
    if (!clock_is_enabled(s->apb_clk)) {
        timer_del(&s->op_timer);
        return;
    }
    timer_mod_ns(&s->op_timer,
                 now + clock_ticks_to_ns(s->apb_clk,
                                         s->op_cycles - s->op_cycles_done));
}

/*
 * Start a command of @cycles. It runs for all of them from when it is
 * issued: a command left running by the reset that powers the digital
 * domain down for deep sleep must not end the next one early, as the ROM
 * issues a read on wake and software-resets the chip if EFUSE_CMD
 * already reads back as done.
 */
static void esp32_efuse_op_start(Esp32EfuseState *s, uint64_t cycles)
{
    s->op_cycles = cycles;
    s->op_cycles_done = 0;
    esp32_efuse_op_schedule(s);
}

/* APB_CLK changes: count the running command's cycles at each rate. */
static void esp32_efuse_apb_clk_update(void *opaque, ClockEvent event)
{
    Esp32EfuseState *s = ESP32_EFUSE(opaque);
    int64_t now;

    if (event == ClockPreUpdate) {
        esp32_efuse_catch_up(s);
    }
    if (!s->cmd_reg) {
        return;
    }
    if (event == ClockPreUpdate) {
        now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        s->op_cycles_done = MIN(s->op_cycles, s->op_cycles_done +
                                clock_ns_to_ticks(s->apb_clk,
                                                  now - s->op_anchor_ns));
    } else {
        esp32_efuse_op_schedule(s);
    }
}

static void esp32_efuse_timer_cb(void *opaque)
{
    Esp32EfuseState *s = ESP32_EFUSE(opaque);
    uint32_t cmd = s->cmd_reg;
    s->cmd_reg = 0;
    s->int_raw_reg |= cmd;
    esp32_efuse_update_irq(s);
}

static const MemoryRegionOps esp32_efuse_ops = {
    .read =  esp32_efuse_read,
    .write = esp32_efuse_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

/*
 * A reset: the registers take their reset values (TRM v5.8, 5.5), and
 * the controller reads the array, as at every reset.
 */
static void esp32_efuse_reset_hold(Object *obj, ResetType type)
{
    Esp32EfuseState *s = ESP32_EFUSE(obj);

    s->clk_reg = EFUSE_CLK_RESET;
    s->conf_reg = 0;
    s->dac_conf_reg = EFUSE_DAC_CONF_RESET;
    s->int_raw_reg = 0;
    s->int_ena_reg = 0;
    esp32_efuse_update_irq(s);
    esp32_efuse_read_op(s);
}

/* Fields of BLK0 that identify the chip */
#define EFUSE_BLK0_D3_CHIP_PACKAGE_D0WD (1 << 9)
#define EFUSE_BLK0_D3_CHIP_VER_REV1     (1 << 15)
#define EFUSE_BLK0_D5_CHIP_VER_REV2     (1 << 20)

static void esp32_efuse_realize(DeviceState *dev, Error **errp)
{
    Esp32EfuseState *s = ESP32_EFUSE(dev);

    /*
     * Without an eFuse drive, read as the ESP32-D0WD-V3 of an
     * ESP32-WROOM-32E module at revision v3.0: both revision fuses
     * burned, with APB_CTRL_DATE marking ECO3, and a zero MAC.
     */
    if (s->blk == NULL) {
        s->efuse_rd.blk0[3] = EFUSE_BLK0_D3_CHIP_PACKAGE_D0WD |
                              EFUSE_BLK0_D3_CHIP_VER_REV1;
        s->efuse_rd.blk0[5] = EFUSE_BLK0_D5_CHIP_VER_REV2;
    }
    if (s->blk != NULL) {
        if (!blk_supports_write_perm(s->blk)) {
            error_setg(errp, "%s: block device is not writeable", __func__);
            return;
        }
        uint64_t perm = BLK_PERM_CONSISTENT_READ | BLK_PERM_WRITE;
        int ret = blk_set_perm(s->blk, perm, BLK_PERM_ALL, NULL);
        if (ret != 0) {
            error_setg(errp, "%s: failed to set permission (%d)", __func__, ret);
            return;
        }
    }
}

static void esp32_efuse_init(Object *obj)
{
    Esp32EfuseState *s = ESP32_EFUSE(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32_efuse_ops, s,
                          TYPE_ESP32_EFUSE, A_EFUSE_DATE + 4);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);

    timer_init_ns(&s->op_timer, QEMU_CLOCK_VIRTUAL, esp32_efuse_timer_cb, s);
    s->apb_clk = qdev_init_clock_in(DEVICE(obj), ESP32_EFUSE_APB_CLK,
                                    esp32_efuse_apb_clk_update, s,
                                    ClockPreUpdate | ClockUpdate);
    qdev_init_gpio_out_named(DEVICE(sbd), &s->efuse_update_gpio, ESP32_EFUSE_UPDATE_GPIO, 1);

    memset(&s->efuse_rd, 0, sizeof(s->efuse_rd));
    memset(&s->efuse_wr, 0, sizeof(s->efuse_wr));
}

static const Property esp32_efuse_properties[] = {
    DEFINE_PROP_DRIVE("drive", Esp32EfuseState, blk),
};

static void esp32_efuse_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32_efuse_reset_hold;
    dc->realize = esp32_efuse_realize;
    device_class_set_props(dc, esp32_efuse_properties);
}

static const TypeInfo esp32_efuse_info = {
    .name = TYPE_ESP32_EFUSE,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32EfuseState),
    .instance_init = esp32_efuse_init,
    .class_init = esp32_efuse_class_init
};

static void esp32_efuse_register_types(void)
{
    type_register_static(&esp32_efuse_info);
}

type_init(esp32_efuse_register_types)
