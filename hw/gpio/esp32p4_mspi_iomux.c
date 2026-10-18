/*
 * ESP32-P4 IOMUX_MSPI_PIN: the flash and PSRAM pads' configuration
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP-IDF's v3 iomux_mspi_pin_reg.h and iomux_mspi_pin_struct.h,
 * the authority for the registers, as the TRM does not document the
 * block; its esp_hal_mspi mspi_ll.h, which drives them, with the
 * bootloader's bootloader_configure_spi_pins(), the hex PSRAM driver
 * (esp_psram_impl_ap_hex.c) and the DQS timing tuning
 * (mspi_timing_by_dqs.c, mspi_timing_tuning.c), which show what software
 * writes and what it reads back to choose.
 *
 * The registers, all read/write, at 0x500E1200 in the HP IO MUX's space,
 * which shares their power domain and sleep retention (ESP-IDF's
 * system_periph_retention.c keeps them with the IO MUX), so that the IO
 * MUX's APB clock and reset are theirs:
 *
 *   0x00       CLK_EN0, bit 0 (reset 1): the register file's clock gating,
 *              nothing software sees.
 *   0x04-0x18  FLASH_CS, _Q, _WP, _HOLD, _CK and _D_PIN0: HYS (bit 0), IE
 *              (1), WPU (2), WPD (3), DRV (5:4), the flash's dedicated pads.
 *   0x1C-0x38  PSRAM_D, _Q, _WP, _HOLD, _DQ4 to _DQ7_PIN0 (DQ0 to DQ7),
 *   0x40-0x64  PSRAM_CK, _CS, _DQ8 to _DQ15_PIN0: DLI (3:0), DLC (7:4),
 *              HYS (8), IE (9), WPU (10), WPD (11), DRV (13:12), the
 *              in-package PSRAM's pads.
 *   0x3C, 0x68 PSRAM_DQS_0 and _1_PIN0: XPD (0), PHASE (2:1), DLI (6:3),
 *              DELAY_90 (10:7), HYS (11), IE (12), WPU (13), WPD (14), DRV
 *              (16:15), DELAY_270 (20:17).
 *
 * Every field resets to 0 but CLK_EN0's. The rest of the space up to the
 * end of the IO MUX's 4 KiB has no register: it reads 0, and a write is
 * dropped as a guest error. Byte and halfword accesses read and write
 * their bytes of a register.
 *
 * [spec:nuos:req:emu.esp32p4.memory]
 * What software sees of them:
 *
 *   - The flash pads' pulls (WPU, WPD): a data line nothing on the bus
 *     drives (a board with no flash, or chip select 1, which has none)
 *     reads as they hold it: pulled up 1, down 0, floating or both (a
 *     45 kOhm divider at half the supply, between VIL and VIH; setting
 *     both is logged as a guest error) 0, as the GPIO block resolves its
 *     pads. The boot ROM's SelectSpiFunction() pulls every flash pad up
 *     as it attaches the flash, so that with no flash its reads find all
 *     1s (its status read a busy flash: "SPI flash busy detected(0xff)");
 *     ESP-IDF's bootloader_configure_spi_pins() clears the pulls again,
 *     leaving CS to the board's pull-up. A flash that is there drives its
 *     data lines whenever the controller reads them, so their pulls change
 *     nothing then.
 *   - The flash pads' IE, which the headers call reserved and reset to 0
 *     while the boot ROM reads the flash from reset: it gates nothing.
 *     The controller's own direction control enables a dedicated pad's
 *     input as it reads, as on the earlier chips' SPI0/1 pads. Held.
 *   - The PSRAM's DDR reads (with the controller's DDR_EN set and its
 *     DQS_LOOP clear, so that it captures each beat on an edge of the
 *     die's strobe, DQS0 for DQ0 to DQ7, DQS1 for DQ8 to DQ15): the
 *     strobe's input is powered by its XPD, which ESP-IDF's
 *     mspi_timing_ll_enable_dqs() sets before the driver reads anything;
 *     with it clear nothing is captured on the strobe's data lines, which
 *     read 0, logged as a guest error. The die launches each beat with
 *     its strobe edge, edge-aligned; the beat's bits are valid from
 *     tDQSQ, the skew between them, until tDQSQ before the next edge;
 *     no document at hand gives the die's tDQSQ, and the model takes
 *     400 ps, the figure of AP Memory's published OPI PSRAM parts. The
 *     controller samples each data line at its strobe's edge shifted by
 *     PHASE, 67.5, 78.75, 90 or 101.25 degrees of the SPI clock (ESP-IDF's
 *     mspi_ll_dqs_phase_t), and by the strobe's delay line, DELAY_90 on
 *     rising edges and DELAY_270 on falling ones, less the data line's
 *     own delay line, DLC, each a step of 100 ps (which no document
 *     gives; it makes the window ESP-IDF's delay line sweep looks for,
 *     -15 to +15 steps around its best phase, span part of the sweep at
 *     200 and 250 MHz and all of it at 80 MHz, so that the sweep finds
 *     its edges and their middle as on the chip). A line sampled before
 *     its beat is valid takes the bit of the beat before, after it the
 *     bit of the beat after (none beyond the burst: 0). ESP-IDF's tuning
 *     writes reference data, then reads it back at each phase and each
 *     delay line setting, and keeps the middle of the longest run that
 *     reads it right.
 *
 * Nothing else here changes what software reads: hysteresis (HYS) and
 * drive strength (DRV) shape the pads' edges, which no document relates
 * to the timing above (as for the GPIO pads, hw/gpio/esp32p4_gpio.c); the
 * PSRAM pads' pulls and IE, as the die drives every data line it is read
 * from and the strobe while it sends; the clock's and chip select's
 * delay lines, which shift the die's strobe and data alike; and the DLI
 * fields, which no document describes and ESP-IDF never sets: held,
 * logged as unimplemented when set. The flash's timing tuning is the
 * controller's (SPI_MEM_C's DIN_MODE and DIN_NUM), not these pads'.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/core/registerfields.h"
#include "hw/gpio/esp32p4_mspi_iomux.h"
#include "migration/vmstate.h"

#define R_CLK_EN0           0
#define R_FLASH_CS          1
#define R_FLASH_D           6
#define R_PSRAM_GROUP0      7
#define R_PSRAM_DQS0        15
#define R_PSRAM_GROUP1      16
#define R_PSRAM_DQ8         18
#define R_PSRAM_DQS1        26

/* The flash pads' registers by data line, IO0 (D) to IO3 (HOLD). */
static const uint8_t flash_io_reg[ESP32P4_MSPI_IOMUX_FLASH_IOS] = {
    R_FLASH_D, R_FLASH_CS + 1, R_FLASH_CS + 2, R_FLASH_CS + 3,
};

FIELD(FLASH, WPU, 2, 1)
FIELD(FLASH, WPD, 3, 1)
FIELD(PSRAM, DLI, 0, 4)
FIELD(PSRAM, DLC, 4, 4)
FIELD(DQS, XPD, 0, 1)
FIELD(DQS, PHASE, 1, 2)
FIELD(DQS, DLI, 3, 4)
FIELD(DQS, DELAY_90, 7, 4)
FIELD(DQS, DELAY_270, 17, 4)

#define CLK_EN0_WRITABLE    0x1u
#define FLASH_WRITABLE      0x3fu
#define PSRAM_WRITABLE      0x3fffu
#define DQS_WRITABLE        0x1fffffu

/* The DDR read window and delay lines (see the top of the file). */
#define TDQSQ_PS            400
#define DELAY_STEP_PS       100

static uint32_t reg_writable(unsigned r)
{
    switch (r) {
    case R_CLK_EN0:
        return CLK_EN0_WRITABLE;
    case R_FLASH_CS ... R_FLASH_D:
        return FLASH_WRITABLE;
    case R_PSRAM_DQS0:
    case R_PSRAM_DQS1:
        return DQS_WRITABLE;
    default:
        return PSRAM_WRITABLE;
    }
}

/* The register of PSRAM data line dq. */
static unsigned psram_dq_reg(unsigned dq)
{
    return dq < 8 ? R_PSRAM_GROUP0 + dq : R_PSRAM_DQ8 + (dq - 8);
}

/*
 * [spec:nuos:req:emu.esp32p4.memory]
 * The level flash data line io reads undriven, from its pad's pulls.
 */
bool esp32p4_mspi_iomux_flash_idle(ESP32P4MspiIomuxState *s, unsigned io)
{
    uint32_t v;
    bool up, down;

    if (io >= ESP32P4_MSPI_IOMUX_FLASH_IOS) {
        return false;
    }
    v = s->regs[flash_io_reg[io]];
    up = FIELD_EX32(v, FLASH, WPU);
    down = FIELD_EX32(v, FLASH, WPD);
    return up && !down;
}

/*
 * [spec:nuos:req:emu.esp32p4.memory]
 * Where PSRAM data line dq samples a DDR beat (see the top of the file).
 */
bool esp32p4_mspi_iomux_psram_strobe_on(ESP32P4MspiIomuxState *s,
                                        unsigned lane)
{
    return FIELD_EX32(s->regs[lane ? R_PSRAM_DQS1 : R_PSRAM_DQS0], DQS, XPD);
}

ESP32P4MspiSample esp32p4_mspi_iomux_psram_sample(ESP32P4MspiIomuxState *s,
                                                  unsigned dq, bool falling,
                                                  uint64_t period_ps)
{
    unsigned lane = dq >= 8;
    uint32_t dqs = s->regs[lane ? R_PSRAM_DQS1 : R_PSRAM_DQS0];
    uint32_t data = s->regs[psram_dq_reg(dq % ESP32P4_MSPI_IOMUX_PSRAM_DQ)];
    int64_t at, valid_from, valid_to;
    unsigned dqs_delay;

    if (!FIELD_EX32(dqs, DQS, XPD)) {
        if (!(s->dqs_off_logged & BIT(lane))) {
            s->dqs_off_logged |= BIT(lane);
            qemu_log_mask(LOG_GUEST_ERROR, "%s: a DDR read of the PSRAM "
                          "with DQS%u's input powered down "
                          "(IOMUX_MSPI_PIN_PSRAM_DQS_%u_XPD clear): DQ%u to "
                          "DQ%u capture nothing, and read 0\n",
                          TYPE_ESP32P4_MSPI_IOMUX, lane, lane, 8 * lane,
                          8 * lane + 7);
        }
        return ESP32P4_MSPI_SAMPLE_NONE;
    }
    s->dqs_off_logged &= ~BIT(lane);
    dqs_delay = falling ? FIELD_EX32(dqs, DQS, DELAY_270)
                        : FIELD_EX32(dqs, DQS, DELAY_90);
    /* PHASE k is 67.5 + 11.25 k degrees: (6 + k) 32nds of the period. */
    at = (int64_t)period_ps * (6 + FIELD_EX32(dqs, DQS, PHASE)) / 32 +
         ((int64_t)dqs_delay - (int64_t)FIELD_EX32(data, PSRAM, DLC)) *
         DELAY_STEP_PS;
    valid_from = TDQSQ_PS;
    valid_to = (int64_t)period_ps / 2 - TDQSQ_PS;
    if (at < valid_from) {
        return ESP32P4_MSPI_SAMPLE_EARLY;
    }
    if (at > valid_to) {
        return ESP32P4_MSPI_SAMPLE_LATE;
    }
    return ESP32P4_MSPI_SAMPLE_GOOD;
}

static uint64_t mspi_iomux_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4MspiIomuxState *s = opaque;
    unsigned r = addr / 4;

    if (r >= ESP32P4_MSPI_IOMUX_NREGS) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read at 0x%" HWADDR_PRIx
                      ", which has no register\n", TYPE_ESP32P4_MSPI_IOMUX,
                      addr);
        return 0;
    }
    return extract32(s->regs[r], (addr & 3) * 8, size * 8);
}

/* [spec:nuos:req:emu.esp32p4.memory] */
static void mspi_iomux_write(void *opaque, hwaddr addr, uint64_t value,
                             unsigned size)
{
    ESP32P4MspiIomuxState *s = opaque;
    unsigned r = addr / 4, shift = (addr & 3) * 8;
    uint32_t mask = MAKE_64BIT_MASK(shift, size * 8) & reg_writable(r);
    uint32_t v;

    if (r >= ESP32P4_MSPI_IOMUX_NREGS) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write at 0x%" HWADDR_PRIx
                      ", which has no register\n", TYPE_ESP32P4_MSPI_IOMUX,
                      addr);
        return;
    }
    v = (s->regs[r] & ~mask) | (((uint32_t)value << shift) & mask);
    s->regs[r] = v;
    if (r >= R_FLASH_CS && r <= R_FLASH_D &&
        FIELD_EX32(v, FLASH, WPU) && FIELD_EX32(v, FLASH, WPD)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: a flash pad's pull-up and "
                      "pull-down both on (offset 0x%" HWADDR_PRIx "): "
                      "undriven, it sits between VIL and VIH, and reads "
                      "low\n", TYPE_ESP32P4_MSPI_IOMUX, addr & ~3);
    }
    if (!s->dli_logged &&
        (((r == R_PSRAM_DQS0 || r == R_PSRAM_DQS1) &&
          FIELD_EX32(v, DQS, DLI)) ||
         (r >= R_PSRAM_GROUP0 && r != R_PSRAM_DQS0 && r != R_PSRAM_DQS1 &&
          FIELD_EX32(v, PSRAM, DLI)))) {
        s->dli_logged = true;
        qemu_log_mask(LOG_UNIMP, "%s: the PSRAM pads' DLI fields, which no "
                      "document describes, are not modelled\n",
                      TYPE_ESP32P4_MSPI_IOMUX);
    }
}

static const MemoryRegionOps mspi_iomux_ops = {
    .read = mspi_iomux_read,
    .write = mspi_iomux_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

/* [spec:nuos:req:emu.esp32p4.memory] */
static void mspi_iomux_reset_hold(Object *obj, ResetType type)
{
    ESP32P4MspiIomuxState *s = ESP32P4_MSPI_IOMUX(obj);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[R_CLK_EN0] = 1;
    s->dqs_off_logged = 0;
}

static void mspi_iomux_init(Object *obj)
{
    ESP32P4MspiIomuxState *s = ESP32P4_MSPI_IOMUX(obj);

    memory_region_init_io(&s->iomem, obj, &mspi_iomux_ops, s,
                          "esp32p4.iomux-mspi-pin", ESP32P4_MSPI_IOMUX_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static const VMStateDescription vmstate_mspi_iomux = {
    .name = TYPE_ESP32P4_MSPI_IOMUX,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, ESP32P4MspiIomuxState,
                             ESP32P4_MSPI_IOMUX_NREGS),
        VMSTATE_UINT8(dqs_off_logged, ESP32P4MspiIomuxState),
        VMSTATE_BOOL(dli_logged, ESP32P4MspiIomuxState),
        VMSTATE_END_OF_LIST()
    },
};

static void mspi_iomux_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = mspi_iomux_reset_hold;
    dc->vmsd = &vmstate_mspi_iomux;
}

/* [spec:nuos:req:emu.esp32p4.memory] */
static const TypeInfo mspi_iomux_info = {
    .name = TYPE_ESP32P4_MSPI_IOMUX,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32P4MspiIomuxState),
    .instance_init = mspi_iomux_init,
    .class_init = mspi_iomux_class_init,
};

static void mspi_iomux_register_types(void)
{
    type_register_static(&mspi_iomux_info);
}

type_init(mspi_iomux_register_types)
