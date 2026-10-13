/*
 * SD Memory Card emulation.  Mostly correct for MMC too.
 *
 * Copyright (c) 2006 Andrzej Zaborowski  <balrog@zabor.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef HW_SD_H
#define HW_SD_H

#include "hw/core/qdev.h"
#include "qom/object.h"

#define OUT_OF_RANGE            (1 << 31)
#define ADDRESS_ERROR           (1 << 30)
#define BLOCK_LEN_ERROR         (1 << 29)
#define ERASE_SEQ_ERROR         (1 << 28)
#define ERASE_PARAM             (1 << 27)
#define WP_VIOLATION            (1 << 26)
#define CARD_IS_LOCKED          (1 << 25)
#define LOCK_UNLOCK_FAILED      (1 << 24)
#define COM_CRC_ERROR           (1 << 23)
#define ILLEGAL_COMMAND         (1 << 22)
#define CARD_ECC_FAILED         (1 << 21)
#define CC_ERROR                (1 << 20)
#define SD_ERROR                (1 << 19)
#define CID_CSD_OVERWRITE       (1 << 16)
#define WP_ERASE_SKIP           (1 << 15)
#define CARD_ECC_DISABLED       (1 << 14)
#define ERASE_RESET             (1 << 13)
#define CURRENT_STATE           (7 << 9)
#define READY_FOR_DATA          (1 << 8)
#define APP_CMD                 (1 << 5)
#define AKE_SEQ_ERROR           (1 << 3)

enum SDPhySpecificationVersion {
    SD_PHY_SPECv2_00_VERS     = 2,
    SD_PHY_SPECv3_01_VERS     = 3,
};

typedef enum {
    SD_VOLTAGE_0_4V     = 400,  /* currently not supported */
    SD_VOLTAGE_1_8V     = 1800,
    SD_VOLTAGE_3_0V     = 3000,
    SD_VOLTAGE_3_3V     = 3300,
} sd_voltage_mv_t;

typedef enum  {
    UHS_NOT_SUPPORTED   = 0,
    UHS_I               = 1,
    UHS_II              = 2,    /* currently not supported */
    UHS_III             = 3,    /* currently not supported */
} sd_uhs_mode_t;

typedef struct {
    uint8_t cmd;
    uint32_t arg;
    uint8_t crc;
} SDRequest;

/*
 * When a card's output on CMD and DAT is valid, as the timing of its bus
 * speed mode gives it: a bit the card launches on an SDCLK edge (the
 * falling edge if @falling, else the rising edge) is valid from
 * *_delay_ps after that edge until *_hold_ps after the edge that launches
 * the next one, which on DAT with @ddr is the next edge of either kind.
 */
typedef struct SDOutputTiming {
    uint32_t cmd_delay_ps;
    uint32_t cmd_hold_ps;
    uint32_t dat_delay_ps;
    uint32_t dat_hold_ps;
    bool falling;
    bool ddr;
} SDOutputTiming;


#define TYPE_SD_CARD "sd-card"
OBJECT_DECLARE_TYPE(SDState, SDCardClass, SD_CARD)

#define TYPE_SD_CARD_SPI "sd-card-spi"
DECLARE_INSTANCE_CHECKER(SDState, SD_CARD_SPI, TYPE_SD_CARD_SPI)

#define TYPE_EMMC "emmc"
DECLARE_INSTANCE_CHECKER(SDState, EMMC, TYPE_EMMC)

struct SDCardClass {
    /*< private >*/
    DeviceClass parent_class;
    /*< public >*/

    /**
     * Process a SD command request.
     * @sd: card
     * @req: command request
     * @resp: buffer to receive the command response
     * @respsz: size of @resp buffer
     *
     * Return: size of the response
     */
    size_t (*do_command)(SDState *sd, SDRequest *req,
                         uint8_t *resp, size_t respsz);
    /**
     * Write data to a SD card.
     * @sd: card
     * @value: data to write
     * @len: length of data
     *
     * Write data on the data lines of a SD card. May write not all data, in
     * which case it should be called again. At least one byte must be consumed.
     *
     * Return: number of bytes actually written. >= 1
     */
    size_t (*write_data)(SDState *sd, const void *buf, size_t len);
    /**
     * Read a byte from a SD card.
     * @sd: card
     * @buf: buffer to receive the data
     * @len: size of data to read
     *
     * Read data from the data lines of a SD card. This may not read all
     * requested data, in this case it should be called again with the remaining
     * buffer. At least one byte must be read.
     *
     * Return: number of bytes actually read. >= 1
     */
    size_t (*read_data)(SDState *sd, void* buf, size_t len);
    /*
     * A block of a write has reached the card failing its CRC check, the
     * bits the card took off its DAT lines not those the host sent: the
     * card discards it, and in a multiple block write the blocks after it
     * until stopped.
     */
    void (*write_crc_error)(SDState *sd);
    bool (*receive_ready)(SDState *sd);
    bool (*data_ready)(SDState *sd);
    void (*set_voltage)(SDState *sd, uint16_t millivolts);
    uint8_t (*get_dat_lines)(SDState *sd);
    bool (*get_cmd_line)(SDState *sd);
    /*
     * The voltage, in mV, at which the card signals on CLK, CMD and
     * DAT: its I/O's supply, 3.3 V or, once switched, 1.8 V.
     */
    uint16_t (*get_signal_voltage)(SDState *sd);
    /* Whether SDCLK reaches the card (it runs, at a level the card sees). */
    void (*set_clock)(SDState *sd, bool running);
    /* The card's output timing in its present bus speed mode. */
    void (*get_output_timing)(SDState *sd, SDOutputTiming *t);
    bool (*get_inserted)(SDState *sd);
    bool (*get_readonly)(SDState *sd);
    void (*set_cid)(SDState *sd);
    void (*set_csd)(SDState *sd, uint64_t size);
    /*
     * For a card that takes time (sd-card's "timing" property), at the
     * present on its bus (SDBusClass's get_time): when its next read
     * block's start bit can be on DAT, INT64_MAX while it sends none (0:
     * the card does not time its data); and when it releases DAT0 from
     * busy, 0 if it is not busy.
     */
    int64_t (*data_ready_time)(SDState *sd);
    int64_t (*busy_end_time)(SDState *sd);
    /* The board switches the card's supply on or off. */
    void (*set_power)(SDState *sd, bool on);

    const struct SDProto *proto;
};

#define TYPE_SD_BUS "sd-bus"
OBJECT_DECLARE_TYPE(SDBus, SDBusClass,
                    SD_BUS)

struct SDBus {
    BusState qbus;
};

struct SDBusClass {
    /*< private >*/
    BusClass parent_class;
    /*< public >*/

    /* These methods are called by the SD device to notify the controller
     * when the card insertion or readonly status changes
     */
    void (*set_inserted)(DeviceState *dev, bool inserted);
    void (*set_readonly)(DeviceState *dev, bool readonly);
    /*
     * Called by an SDIO card to signal its interrupt to the controller:
     * the level of the card's interrupt request, which it gives on DAT1.
     */
    void (*set_irq)(DeviceState *dev, bool level);
    /*
     * Called by a card that changes the levels it drives on CMD or
     * DAT[3:0] on its own, not in answer to the controller, at the time
     * line_time() gives (hw/core/line-time.h).
     */
    void (*lines_changed)(DeviceState *dev);
    /*
     * For a card that takes time: the present on the bus, as the
     * controller times its events (by default, the virtual clock); and
     * when @cycles of the card clock from @from have passed (INT64_MAX
     * while it is stopped; by default @from: no clock is known).
     */
    int64_t (*get_time)(DeviceState *dev);
    int64_t (*card_clock_after)(DeviceState *dev, int64_t from,
                                uint64_t cycles);
};

/* Functions to be used by qdevified callers (working via
 * an SDBus rather than directly with SDState)
 */
void sdbus_set_voltage(SDBus *sdbus, uint16_t millivolts);
uint8_t sdbus_get_dat_lines(SDBus *sdbus);
bool sdbus_get_cmd_line(SDBus *sdbus);
/**
 * sdbus_get_signal_voltage: The card's signalling voltage, in mV.
 * @sdbus: bus
 *
 * Return: 3300 for a card that does not say, or no card.
 */
uint16_t sdbus_get_signal_voltage(SDBus *sdbus);
/**
 * sdbus_set_clock: Whether SDCLK reaches the card, as of line_time().
 * @sdbus: bus
 * @running: true while the clock runs at a level the card sees
 */
void sdbus_set_clock(SDBus *sdbus, bool running);
/**
 * sdbus_get_output_timing: The card's output timing in its bus mode.
 * @sdbus: bus
 * @t: filled in
 *
 * Return: false if there is no card or it gives none.
 */
bool sdbus_get_output_timing(SDBus *sdbus, SDOutputTiming *t);
/**
 * sdbus_do_command: Process a SD command request
 * @sd: card
 * @req: command request
 * @resp: buffer to receive the command response
 * @respsz: size of @resp buffer
 *
 * Return: size of the response
 */
size_t sdbus_do_command(SDBus *sd, SDRequest *req, uint8_t *resp, size_t respsz);
/**
 * Write a byte to a SD bus.
 * @sd: bus
 * @value: byte to write
 *
 * Write a byte on the data lines of a SD bus.
 */
void sdbus_write_byte(SDBus *sd, uint8_t value);
/**
 * Read a byte from a SD bus.
 * @sd: bus
 *
 * Read a byte from the data lines of a SD bus.
 *
 * Return: byte value read
 */
uint8_t sdbus_read_byte(SDBus *sd);
/**
 * Write data to a SD bus.
 * @sdbus: bus
 * @buf: data to write
 * @length: number of bytes to write
 *
 * Write multiple bytes of data on the data lines of a SD bus.
 */
void sdbus_write_data(SDBus *sdbus, const void *buf, size_t length);
/**
 * Read data from a SD bus.
 * @sdbus: bus
 * @buf: buffer to read data into
 * @length: number of bytes to read
 *
 * Read multiple bytes of data on the data lines of a SD bus.
 */
void sdbus_read_data(SDBus *sdbus, void *buf, size_t length);
/**
 * sdbus_write_crc_error: A write block fails the card's CRC check
 * @sdbus: bus
 *
 * A block of a write has crossed the bus, but the card took bits off its
 * DAT lines other than those the host sent: it discards the block (and,
 * in a multiple block write, the blocks after it until stopped), as a
 * card does whose CRC check fails. Instead of sdbus_write_data() for the
 * block's bytes.
 */
void sdbus_write_crc_error(SDBus *sdbus);
bool sdbus_receive_ready(SDBus *sd);
bool sdbus_data_ready(SDBus *sd);
bool sdbus_get_inserted(SDBus *sd);
bool sdbus_get_readonly(SDBus *sd);
/**
 * sdbus_data_ready_time: When the card's read data comes
 * @sd: bus
 *
 * Return: the time the card's next read block can start, at or before
 * the present once it can; INT64_MAX while the card sends none; 0 for a
 * card that does not time its data, or no card.
 */
int64_t sdbus_data_ready_time(SDBus *sd);
/**
 * sdbus_busy_end_time: When the card releases DAT0
 * @sd: bus
 *
 * Return: the time the card's busy (DAT0 held low while it programs or
 * erases) ends; 0 while it is not busy, or for no card.
 */
int64_t sdbus_busy_end_time(SDBus *sd);
/**
 * sdbus_set_power: The board switches the supply of the card on @sd
 * @sd: bus
 * @on: true to power the card
 *
 * A card switched off loses its state and does not answer; switched on,
 * it starts from power-up.
 */
void sdbus_set_power(SDBus *sd, bool on);
/**
 * sdbus_reparent_card: Reparent an SD card from one controller to another
 * @from: controller bus to remove card from
 * @to: controller bus to move card to
 *
 * Reparent an SD card, effectively unplugging it from one controller
 * and inserting it into another. This is useful for SoCs like the
 * bcm2835 which have two SD controllers and connect a single SD card
 * to them, selected by the guest reprogramming GPIO line routing.
 */
void sdbus_reparent_card(SDBus *from, SDBus *to);

/* Functions to be used by SD devices to report back to qdevified controllers */
void sdbus_set_inserted(SDBus *sd, bool inserted);
void sdbus_set_readonly(SDBus *sd, bool inserted);
/**
 * sdbus_set_irq: An SDIO card's interrupt request on its bus.
 * @sd: bus
 * @level: true while the card requests an interrupt
 */
void sdbus_set_irq(SDBus *sd, bool level);
/**
 * sdbus_lines_changed: The card changed what it drives on CMD or DAT.
 * @sd: bus
 */
void sdbus_lines_changed(SDBus *sd);
/**
 * sdbus_get_time: The present on the bus, for a card that takes time
 * @sd: bus
 */
int64_t sdbus_get_time(SDBus *sd);
/**
 * sdbus_card_clock_after: When @cycles of the card clock have passed
 * @sd: bus
 * @from: from when, in ns
 * @cycles: card clock cycles
 *
 * Return: the time, INT64_MAX while the clock is stopped, or @from when
 * the controller does not say.
 */
int64_t sdbus_card_clock_after(SDBus *sd, int64_t from, uint64_t cycles);

#endif /* HW_SD_H */
