/*
 * Helpers for the ESP32-P4 boot ROM's QTest testcases: flash images, eFuse
 * files and ELFs for the ROM, a machine booted with serial port 0 into a
 * log, and esptool's download protocol on a serial port
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef ESP32P4_BOOT_UTILS_H
#define ESP32P4_BOOT_UTILS_H

#include "libqtest.h"

#define FLASH_SIZE          (4 * 1024 * 1024)
#define BOOTLOADER_OFFSET   0x2000
#define LOAD_ADDR           0x4ff80000
#define MAGIC_ADDR          0x5010ff00
#define MAGIC               0x600db007
#define GPIO_STRAP          0x500e0038
#define EFUSE_FILE_SIZE     336

/*
 * Block 0's eFuse bits the boot ROM reads, in the file's words (the RD_*
 * registers': RD_REPEAT_DATAn is word n + 1).
 */
#define EFUSE_DATA(n)                   ((n) + 1)
#define DIS_FORCE_DOWNLOAD              (1u << 12)      /* DATA0 */
#define SECURE_BOOT_EN                  (1u << 20)      /* DATA2 */
#define DIS_DOWNLOAD_MODE               (1u << 0)       /* DATA3 */
#define DIS_DIRECT_BOOT                 (1u << 1)       /* DATA3 */
#define ENABLE_SECURITY_DOWNLOAD        (1u << 5)       /* DATA3 */
#define UART_PRINT_CONTROL_OFF          (3u << 6)       /* DATA3 */
#define KEY_PURPOSE_0(p)                ((uint32_t)(p) << 24) /* DATA1 */
#define KEY_PURPOSE_SECURE_BOOT_DIGEST0 9
/* BLOCK_KEY0's words: after BLOCK0's 6, BLOCK1's 6, BLOCK2's and 3's 8. */
#define EFUSE_KEY0(n)                   (28 + (n))

/* Writes MAGIC to MAGIC_ADDR, then loops. */
extern const uint32_t prog_magic[6];

/*
 * A serial port esptool has open to the ROM: the bytes received on it and
 * not yet taken as a frame
 */
typedef struct EspPort {
    QTestState *qts;
    int fd;
    GByteArray *rx;
} EspPort;

typedef struct Boot {
    QTestState *qts;
    char *log;
} Boot;

/* A response from the ROM: its command, data and status. */
typedef struct EspReply {
    uint8_t cmd;
    uint16_t len;
    /* The header's value field: READ_REG's word. */
    uint32_t value;
    uint8_t data[64];
    uint8_t status;
    uint8_t error;
} EspReply;

char *write_tmp(const char *tmpl, const void *data, size_t len);
void put_image(uint8_t *flash, uint32_t offset, const uint32_t *code,
               size_t words, size_t filler, bool bad_digest);
char *write_flash(const uint32_t *code, size_t words, bool direct);
char *write_flash_filled(const uint32_t *code, size_t words, size_t filler,
                         bool bad_digest);
char *write_efuse_words(const uint32_t *words);
char *write_efuse(unsigned word, uint32_t bits);
char *write_elf(const uint32_t *code, size_t words);

Boot boot(uint32_t strap, const char *flash, const char *efuse,
          const char *kernel);
void boot_end(Boot *b);
char *output(Boot *b);
void await_output(Boot *b, const char *want);
void await_magic(Boot *b);
void expect_no_magic(Boot *b);

void esp_sync(int fd);
void esp_load_and_run(QTestState *qts, int fd);
void await_text(int fd, GString *rx, const char *want);
EspReply esp_command(int fd, uint8_t cmd, const uint8_t *data, size_t len,
                     uint32_t checksum);
uint32_t esp_read_reg(int fd, uint32_t addr);
void esp_write_reg(int fd, uint32_t addr, uint32_t value);

const char *esp_timing(bool icount);
void esp_port_await_banner(EspPort *p);
void esp_port_check_loader(EspPort *p, unsigned kib);

#endif
