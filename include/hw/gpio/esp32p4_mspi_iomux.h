/*
 * ESP32-P4 IOMUX_MSPI_PIN: the flash and PSRAM pads' configuration
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_GPIO_ESP32P4_MSPI_IOMUX_H
#define HW_GPIO_ESP32P4_MSPI_IOMUX_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_ESP32P4_MSPI_IOMUX "esp32p4-mspi-iomux"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4MspiIomuxState, ESP32P4_MSPI_IOMUX)

/* From IOMUX_MSPI_PIN_CLK_EN0 at 0x500E1200 to the HP IO MUX's 4 KiB end */
#define ESP32P4_MSPI_IOMUX_BASE     0x500e1200
#define ESP32P4_MSPI_IOMUX_SIZE     0xe00
#define ESP32P4_MSPI_IOMUX_NREGS    (0x6c / 4)

/*
 * The flash's data lines, by their quad-mode number: IO0 is D (MOSI),
 * IO1 Q (MISO), IO2 WP and IO3 HOLD.
 */
#define ESP32P4_MSPI_IOMUX_FLASH_IOS    4

/* The PSRAM's data lines, DQ0 to DQ15; DQS0 strobes DQ0 to DQ7. */
#define ESP32P4_MSPI_IOMUX_PSRAM_DQ     16

/* Where a PSRAM data pin's sample of a DDR beat falls. */
typedef enum ESP32P4MspiSample {
    /* In the beat's data valid window: the beat's bit. */
    ESP32P4_MSPI_SAMPLE_GOOD,
    /* Before it: the bit of the beat before. */
    ESP32P4_MSPI_SAMPLE_EARLY,
    /* After it: the bit of the beat after. */
    ESP32P4_MSPI_SAMPLE_LATE,
    /* The strobe's input is powered down: nothing is sampled. */
    ESP32P4_MSPI_SAMPLE_NONE,
} ESP32P4MspiSample;

/*
 * The device's registers are mapped behind the HP IO MUX's APB clock and
 * reset (hw/riscv/esp32p4.c). The MSPI controllers (hw/ssi/esp32p4_mspi.c)
 * hold a link to it and ask what their reads see through the pads:
 *
 * esp32p4_mspi_iomux_flash_idle(): the level flash data line io reads
 * while nothing on the bus drives it, as its pad's pulls hold it.
 *
 * esp32p4_mspi_iomux_psram_sample(): where PSRAM data pin dq samples the
 * DDR beat its strobe's rising (or with falling, falling) edge launches,
 * at an SPI clock period of period_ps.
 */
bool esp32p4_mspi_iomux_flash_idle(ESP32P4MspiIomuxState *s, unsigned io);
ESP32P4MspiSample esp32p4_mspi_iomux_psram_sample(ESP32P4MspiIomuxState *s,
                                                  unsigned dq, bool falling,
                                                  uint64_t period_ps);

struct ESP32P4MspiIomuxState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t regs[ESP32P4_MSPI_IOMUX_NREGS];
    /* The strobes found off, and the DLI fields found set, as logged. */
    uint8_t dqs_off_logged;
    bool dli_logged;
};

#endif
