/*
 * ESP32-P4 eFuse controller and its 4096-bit eFuse array
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_NVRAM_ESP32P4_EFUSE_H
#define HW_NVRAM_ESP32P4_EFUSE_H

#include "hw/core/clock.h"
#include "hw/core/sysbus.h"
#include "hw/misc/esp32p4_clk_period.h"
#include "qemu/timer.h"
#include "qom/object.h"

/*
 * Espressif's QEMU names its eFuse devices "nvram.<chip>.efuse", and
 * ESP-IDF's "idf.py qemu" attaches the eFuse file to the device of that
 * name, so this one follows them.
 */
#define TYPE_ESP32P4_EFUSE "nvram.esp32p4.efuse"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4EfuseState, ESP32P4_EFUSE)

#define ESP32P4_EFUSE_SIZE          0x1000

/* The named GPIO input that holds the controller in reset while high. */
#define ESP32P4_EFUSE_RESET         "reset"

/* BLOCK0 to BLOCK10. */
#define ESP32P4_EFUSE_BLOCKS        11
/* BLOCK0's EFUSE_WR_DIS and EFUSE_RD_REPEAT_DATA0..4. */
#define ESP32P4_EFUSE_BLK0_WORDS    6
/* The four stored copies of BLOCK0's parameters other than EFUSE_WR_DIS. */
#define ESP32P4_EFUSE_BLK0_BACKUPS  4
/* Each data word of a Reed-Solomon coded block, and its 12 parity bytes. */
#define ESP32P4_EFUSE_RS_WORDS      8
#define ESP32P4_EFUSE_RS_CHECK_WORDS 3
/*
 * The words the RD_* registers hold, BLOCK0 to BLOCK10 in order: 6 for
 * BLOCK0, 6 for BLOCK1, 8 for each of the others. This is also the layout
 * of the first 336 bytes of the backing file.
 */
#define ESP32P4_EFUSE_RD_WORDS      84

/* The key blocks: KEY0 to KEY5 are BLOCK4 to BLOCK9. */
#define ESP32P4_EFUSE_KEY_BLOCK0    4
#define ESP32P4_EFUSE_KEY_BLOCKS    6

struct ESP32P4EfuseState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    Clock *clk;
    BlockBackend *blk;
    QEMUTimer op_timer;

    /*
     * The eFuse array: EFUSE_WR_DIS, the four copies of BLOCK0's other
     * 152 bits, and the data and parity bytes of BLOCK1 to BLOCK10 (index
     * 0 of rs_data and rs_check is unused).
     */
    uint32_t wr_dis;
    uint32_t blk0[ESP32P4_EFUSE_BLK0_BACKUPS][ESP32P4_EFUSE_BLK0_WORDS - 1];
    uint32_t rs_data[ESP32P4_EFUSE_BLOCKS][ESP32P4_EFUSE_RS_WORDS];
    uint32_t rs_check[ESP32P4_EFUSE_BLOCKS][ESP32P4_EFUSE_RS_CHECK_WORDS];
    /* The file has room for the copies and parity, not only the data. */
    bool blk_has_raw;
    /*
     * The "chip-revision" property: the wafer version a factory-fresh
     * array holds, major * 100 + minor (300 for v3.0).
     */
    uint32_t chip_revision;

    /*
     * What the last read operation loaded: the decoded parameters, before
     * read protection hides any from software, and the error records.
     */
    uint32_t rd[ESP32P4_EFUSE_RD_WORDS];
    uint32_t repeat_err[ESP32P4_EFUSE_BLK0_WORDS - 1];
    uint32_t rs_err[2];

    uint32_t pgm_data[ESP32P4_EFUSE_RS_WORDS];
    uint32_t pgm_check[ESP32P4_EFUSE_RS_CHECK_WORDS];
    uint32_t clk_reg;
    uint32_t conf;
    uint32_t cmd;
    uint32_t int_raw;
    uint32_t int_ena;
    uint32_t dac_conf;
    uint32_t rd_tim_conf;
    uint32_t wr_tim_conf1;
    uint32_t wr_tim_conf2;
    uint32_t wr_tim_conf0;
    uint32_t date;
    uint32_t ecdsa;
    uint32_t apb2otp_en;

    /* The timing the state machine uses, taken by EFUSE_UPDATE. */
    uint32_t dac_conf_used;
    uint32_t rd_tim_conf_used;
    uint32_t wr_tim_conf1_used;
    uint32_t wr_tim_conf2_used;
    uint32_t wr_tim_conf0_used;

    /*
     * The running operation, timed in eFuse clock cycles: its length, the
     * cycles run up to the start of the current stretch of constant clock,
     * and the clock's cycles counted from when that stretch began.
     */
    uint32_t op;
    uint64_t op_cycles;
    uint64_t op_cycles_done;
    ESP32P4ClkCount op_edges;
    uint32_t op_pgm[ESP32P4_EFUSE_RS_WORDS + ESP32P4_EFUSE_RS_CHECK_WORDS];
    uint32_t op_blk;
    bool in_reset;
};

/*
 * Hooks for the blocks the eFuses configure, which read the parameters
 * as loaded by the last read operation, read protection notwithstanding:
 * the hardware cryptography modules take their keys this way.
 */

/* Word `word` of block `block` as loaded, in the RD_* registers' layout. */
uint32_t esp32p4_efuse_word(ESP32P4EfuseState *s, unsigned block,
                            unsigned word);

/* The 5-bit purpose of key block `block` (4 to 9, KEY0 to KEY5). */
uint32_t esp32p4_efuse_key_purpose(ESP32P4EfuseState *s, unsigned block);

/* The 32 bytes of key block `block` (4 to 9) as loaded. */
void esp32p4_efuse_key(ESP32P4EfuseState *s, unsigned block,
                       uint8_t key[32]);

/*
 * The first key block whose purpose is `purpose`, or -1 if there is none
 * (key purpose values are TRM table 9.4-2's, and ESP-IDF's
 * esp_efuse_purpose_t for the 5-bit fields of v3.x silicon).
 */
int esp32p4_efuse_find_purpose(ESP32P4EfuseState *s, uint32_t purpose);

#endif
