/*
 * Helpers for the ESP32-P4 boot ROM's QTest testcases
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <poll.h>
#include "qemu/bswap.h"
#include "libqtest.h"
#include "esp32p4-boot-utils.h"

#define EM_RISCV 243
#define PT_LOAD 1

/* Writes MAGIC to MAGIC_ADDR, then loops. */
const uint32_t prog_magic[6] = {
    0x50110537,         /* lui  a0, 0x50110 */
    0xf0050513,         /* addi a0, a0, -256 */
    0x600db5b7,         /* lui  a1, 0x600db */
    0x00758593,         /* addi a1, a1, 7 */
    0x00b52023,         /* sw   a1, 0(a0) */
    0x0000006f,         /* j    . */
};

char *write_tmp(const char *tmpl, const void *data, size_t len)
{
    GError *err = NULL;
    char *path = NULL;
    int fd = g_file_open_tmp(tmpl, &path, &err);

    g_assert_no_error(err);
    g_assert_cmpint(write(fd, data, len), ==, len);
    close(fd);
    return path;
}

/*
 * An image in esptool's format for the ESP32-P4 (chip ID 18, chip
 * revisions v3.0 to v3.99, DIO at 80 MHz, the SHA-256 digest appended),
 * one segment of @words at LOAD_ADDR followed by @filler words of a
 * pattern, its entry point there, written into @flash at @offset; with
 * @bad_digest the digest's first bit flipped.
 */
void put_image(uint8_t *flash, uint32_t offset, const uint32_t *code,
               size_t words, size_t filler, bool bad_digest)
{
    uint8_t *img = flash + offset;
    size_t len = 24, i;
    uint8_t checksum = 0xef;
    gsize digest_len = 32;
    GChecksum *sha = g_checksum_new(G_CHECKSUM_SHA256);

    img[0] = 0xe9;
    img[1] = 1;
    img[2] = 2;                         /* DIO */
    img[3] = 0x2f;                      /* 4 MiB, 80 MHz */
    stl_le_p(img + 4, LOAD_ADDR);
    memset(img + 8, 0, 16);
    img[8] = 0xee;                      /* no WP pin */
    stw_le_p(img + 12, 18);             /* ESP32-P4 */
    stw_le_p(img + 15, 300);            /* min_chip_rev_full */
    stw_le_p(img + 17, 399);            /* max_chip_rev_full */
    img[23] = 1;                        /* digest appended */
    stl_le_p(img + len, LOAD_ADDR);
    stl_le_p(img + len + 4, (words + filler) * 4);
    len += 8;
    for (i = 0; i < words; i++) {
        stl_le_p(img + len, code[i]);
        len += 4;
    }
    for (i = 0; i < filler; i++) {
        stl_le_p(img + len, 0x9e3779b9u * (i + 1));
        len += 4;
    }
    for (i = 32; i < len; i++) {
        checksum ^= img[i];
    }
    while (len % 16 != 15) {
        img[len++] = 0;
    }
    img[len++] = checksum;
    g_checksum_update(sha, img, len);
    g_checksum_get_digest(sha, img + len, &digest_len);
    g_checksum_free(sha);
    if (bad_digest) {
        img[len] ^= 1;
    }
}

char *write_flash(const uint32_t *code, size_t words, bool direct)
{
    g_autofree uint8_t *flash = g_malloc(FLASH_SIZE);

    memset(flash, 0xff, FLASH_SIZE);
    if (direct) {
        /* A direct boot image: the magic words, then code at 0x40000008. */
        stl_le_p(flash, 0xaedb041d);
        stl_le_p(flash + 4, 0xaedb041d);
        for (size_t i = 0; i < words; i++) {
            stl_le_p(flash + 8 + 4 * i, code[i]);
        }
    } else {
        put_image(flash, BOOTLOADER_OFFSET, code, words, 0, false);
    }
    return write_tmp("esp32p4-boot-XXXXXX.img", flash, FLASH_SIZE);
}

/* A flash with put_image()'s image of @code and @filler words. */
char *write_flash_filled(const uint32_t *code, size_t words,
                         size_t filler, bool bad_digest)
{
    g_autofree uint8_t *flash = g_malloc(FLASH_SIZE);

    memset(flash, 0xff, FLASH_SIZE);
    put_image(flash, BOOTLOADER_OFFSET, code, words, filler, bad_digest);
    return write_tmp("esp32p4-boot-XXXXXX.img", flash, FLASH_SIZE);
}

/* An eFuse file, 336 bytes of the RD_* words. */
char *write_efuse_words(const uint32_t *words)
{
    uint8_t data[EFUSE_FILE_SIZE];

    for (int i = 0; i < EFUSE_FILE_SIZE / 4; i++) {
        stl_le_p(data + 4 * i, words[i]);
    }
    return write_tmp("esp32p4-efuse-XXXXXX.bin", data, sizeof(data));
}

/* An eFuse file with one bit field set. */
char *write_efuse(unsigned word, uint32_t bits)
{
    uint32_t words[EFUSE_FILE_SIZE / 4] = { 0 };

    words[word] = bits;
    return write_efuse_words(words);
}

/* Write a 32-bit RISC-V ELF of @words at LOAD_ADDR, its entry point. */
char *write_elf(const uint32_t *code, size_t words)
{
    uint8_t elf[84 + 64] = { 0 };

    g_assert(words * 4 <= 64);
    memcpy(elf, "\x7f" "ELF", 4);
    elf[4] = 1;                         /* ELFCLASS32 */
    elf[5] = 1;                         /* ELFDATA2LSB */
    elf[6] = 1;                         /* EV_CURRENT */
    stw_le_p(elf + 16, 2);              /* ET_EXEC */
    stw_le_p(elf + 18, EM_RISCV);
    stl_le_p(elf + 20, 1);              /* e_version */
    stl_le_p(elf + 24, LOAD_ADDR);      /* e_entry */
    stl_le_p(elf + 28, 52);             /* e_phoff */
    stw_le_p(elf + 40, 52);             /* e_ehsize */
    stw_le_p(elf + 42, 32);             /* e_phentsize */
    stw_le_p(elf + 44, 1);              /* e_phnum */
    stw_le_p(elf + 46, 40);             /* e_shentsize */
    stl_le_p(elf + 52, PT_LOAD);
    stl_le_p(elf + 56, 84);             /* p_offset */
    stl_le_p(elf + 60, LOAD_ADDR);      /* p_vaddr */
    stl_le_p(elf + 64, LOAD_ADDR);      /* p_paddr */
    stl_le_p(elf + 68, words * 4);      /* p_filesz */
    stl_le_p(elf + 72, words * 4);      /* p_memsz */
    stl_le_p(elf + 76, 5);              /* PF_R | PF_X */
    stl_le_p(elf + 80, 4);              /* p_align */
    for (size_t i = 0; i < words; i++) {
        stl_le_p(elf + 84 + 4 * i, code[i]);
    }
    return write_tmp("esp32p4-boot-XXXXXX.elf", elf, 84 + words * 4);
}

/*
 * Start the machine with serial port 0 into a log file: @strap the
 * strapping, @flash a flash image or NULL for a blank one, @efuse an eFuse
 * file or NULL for a factory-fresh chip, @kernel an ELF for -kernel or
 * NULL.
 */
Boot boot(uint32_t strap, const char *flash, const char *efuse,
          const char *kernel)
{
    Boot b;
    g_autofree char *drive = NULL;
    g_autofree char *fuses = NULL;
    g_autofree char *elf = NULL;
    int fd = g_file_open_tmp("esp32p4-boot-XXXXXX.log", &b.log, NULL);

    g_assert_cmpint(fd, >=, 0);
    close(fd);
    drive = flash ? g_strdup_printf("-drive file=%s,if=mtd,format=raw,"
                                    "snapshot=on", flash) : g_strdup("");
    fuses = efuse ? g_strdup_printf("-drive file=%s,if=none,format=raw,"
                                    "id=efuse,snapshot=on -global "
                                    "driver=nvram.esp32p4.efuse,"
                                    "property=drive,value=efuse", efuse)
                  : g_strdup("");
    elf = kernel ? g_strdup_printf("-kernel %s", kernel) : g_strdup("");
    b.qts = qtest_initf("-M esp32p4,flash-size=4M,strap-mode=0x%x "
                        "-accel tcg -icount shift=0,sleep=off %s %s %s "
                        "-chardev file,id=s0,path=%s -serial chardev:s0",
                        strap, drive, fuses, elf, b.log);
    return b;
}

void boot_end(Boot *b)
{
    qtest_quit(b->qts);
    unlink(b->log);
    g_free(b->log);
}

/* What serial port 0 has printed so far. */
char *output(Boot *b)
{
    char *text = NULL;

    g_assert(g_file_get_contents(b->log, &text, NULL, NULL));
    return text;
}

/* Wait up to 30 s for @want on serial port 0. */
void await_output(Boot *b, const char *want)
{
    for (int i = 0; i < 300; i++) {
        g_autofree char *text = output(b);

        if (strstr(text, want)) {
            return;
        }
        g_usleep(100000);
    }
    {
        g_autofree char *text = output(b);

        g_test_message("'%s' not in: %s", want, text);
        g_test_fail();
    }
}

/* Wait up to 30 s for the loaded program's MAGIC in LP SRAM. */
void await_magic(Boot *b)
{
    for (int i = 0; i < 300; i++) {
        if (qtest_readl(b->qts, MAGIC_ADDR) == MAGIC) {
            return;
        }
        g_usleep(100000);
    }
    g_test_message("the loaded program did not run");
    g_test_fail();
}

void expect_no_magic(Boot *b)
{
    g_assert_cmphex(qtest_readl(b->qts, MAGIC_ADDR), !=, MAGIC);
}

/* SLIP-frame and send an esptool command with its data and checksum. */
static void esp_send(int fd, uint8_t cmd, const uint8_t *data, size_t len,
                     uint32_t checksum)
{
    g_autofree uint8_t *raw = g_malloc(8 + len);
    GByteArray *out = g_byte_array_new();
    static const uint8_t end = 0xc0;

    raw[0] = 0;
    raw[1] = cmd;
    stw_le_p(raw + 2, len);
    stl_le_p(raw + 4, checksum);
    memcpy(raw + 8, data, len);
    g_byte_array_append(out, &end, 1);
    for (size_t i = 0; i < 8 + len; i++) {
        static const uint8_t esc_c0[] = { 0xdb, 0xdc };
        static const uint8_t esc_db[] = { 0xdb, 0xdd };

        if (raw[i] == 0xc0) {
            g_byte_array_append(out, esc_c0, 2);
        } else if (raw[i] == 0xdb) {
            g_byte_array_append(out, esc_db, 2);
        } else {
            g_byte_array_append(out, raw + i, 1);
        }
    }
    g_byte_array_append(out, &end, 1);
    g_assert_cmpint(send(fd, out->data, out->len, 0), ==, out->len);
    g_byte_array_free(out, true);
}

/*
 * How long, in the host's time, a wait for the ROM may last: a SYNC that
 * goes unanswered for SYNC_WAIT_MS is sent again, as esptool sends it; a
 * command's response, or text, comes within REPLY_WAIT_MS however loaded
 * the host is. The waits end as soon as what they wait for arrives. They
 * are bounded in time rather than in reads, as the ROM's other output,
 * such as its further answers to a SYNC, arrives in pieces meanwhile.
 */
#define SYNC_WAIT_MS        3000
#define REPLY_WAIT_MS       30000

static int64_t deadline_us(int ms)
{
    return g_get_monotonic_time() + ms * (int64_t)1000;
}

/*
 * Read from @fd for up to @ms ms until the response to @cmd: true with its
 * first status byte (0 for success) in *status.
 */
static bool esp_response(int fd, uint8_t cmd, uint8_t *status, int ms)
{
    int64_t end = deadline_us(ms);
    uint8_t buf[4096];
    size_t got = 0;

    while (g_get_monotonic_time() < end) {
        struct pollfd pfd = { .fd = fd, .events = POLLIN };

        if (poll(&pfd, 1, 100) > 0) {
            ssize_t n = recv(fd, buf + got, sizeof(buf) - got, 0);

            g_assert_cmpint(n, >, 0);
            got += n;
        }
        for (size_t s = 0; s + 10 < got; s++) {
            if (buf[s] == 0xc0 && buf[s + 1] == 0x01 && buf[s + 2] == cmd) {
                *status = buf[s + 9];
                return true;
            }
        }
        if (got == sizeof(buf)) {
            got = 0;
        }
    }
    return false;
}

static const uint8_t sync_data[36] = {
    0x07, 0x07, 0x12, 0x20,
    0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
    0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
    0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
    0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
};

/* SYNC, as esptool resends it, until the ROM answers on @fd. */
void esp_sync(int fd)
{
    uint8_t status = 0xff;
    bool synced = false;

    for (int i = 0; i < 20 && !synced; i++) {
        esp_send(fd, 0x08, sync_data, sizeof(sync_data), 0);
        synced = esp_response(fd, 0x08, &status, SYNC_WAIT_MS);
    }
    g_assert(synced);
    g_assert_cmpint(status, ==, 0);
    /* Let the ROM's further SYNC responses out before the next command. */
    g_usleep(200000);
}

/*
 * Over @fd, take the ROM's download into L2MEM: a program into L2MEM
 * (MEM_BEGIN, MEM_DATA) that it then runs (MEM_END), and wait for it.
 */
void esp_load_and_run(QTestState *qts, int fd)
{
    uint8_t begin[16], data[16 + sizeof(prog_magic)], end[8];
    uint32_t checksum = 0xef;
    uint8_t status = 0xff;

    esp_sync(fd);
    stl_le_p(begin, sizeof(prog_magic));
    stl_le_p(begin + 4, 1);
    stl_le_p(begin + 8, sizeof(prog_magic));
    stl_le_p(begin + 12, LOAD_ADDR);
    esp_send(fd, 0x05, begin, sizeof(begin), 0);
    g_assert(esp_response(fd, 0x05, &status, REPLY_WAIT_MS));
    g_assert_cmpint(status, ==, 0);

    stl_le_p(data, sizeof(prog_magic));
    stl_le_p(data + 4, 0);
    stl_le_p(data + 8, 0);
    stl_le_p(data + 12, 0);
    for (size_t i = 0; i < ARRAY_SIZE(prog_magic); i++) {
        stl_le_p(data + 16 + 4 * i, prog_magic[i]);
    }
    for (size_t i = 16; i < sizeof(data); i++) {
        checksum ^= data[i];
    }
    esp_send(fd, 0x07, data, sizeof(data), checksum);
    g_assert(esp_response(fd, 0x07, &status, REPLY_WAIT_MS));
    g_assert_cmpint(status, ==, 0);

    stl_le_p(end, 0);                   /* run it */
    stl_le_p(end + 4, LOAD_ADDR);
    esp_send(fd, 0x06, end, sizeof(end), 0);
    for (int i = 0; i < 300 && qtest_readl(qts, MAGIC_ADDR) != MAGIC; i++) {
        g_usleep(100000);
    }
    g_assert_cmphex(qtest_readl(qts, MAGIC_ADDR), ==, MAGIC);
}

/*
 * Read @fd for up to REPLY_WAIT_MS until the text received holds @want.
 * @rx holds what has arrived and not yet been matched: what follows the
 * match stays in it for the next call, as one read may bring several
 * lines.
 */
void await_text(int fd, GString *rx, const char *want)
{
    int64_t end = deadline_us(REPLY_WAIT_MS);
    const char *at;

    while (!strstr(rx->str, want) && g_get_monotonic_time() < end) {
        struct pollfd pfd = { .fd = fd, .events = POLLIN };
        char buf[256];

        if (poll(&pfd, 1, 100) > 0) {
            ssize_t n = recv(fd, buf, sizeof(buf), 0);

            g_assert_cmpint(n, >, 0);
            g_string_append_len(rx, buf, n);
        }
    }
    at = strstr(rx->str, want);
    if (!at) {
        g_test_message("'%s' not in: %s", want, rx->str);
        g_test_fail();
        return;
    }
    g_string_erase(rx, 0, at - rx->str + strlen(want));
}

/* Read and drop what @fd has, until it has been quiet for 200 ms. */
static void esp_drain(int fd)
{
    uint8_t buf[1024];
    struct pollfd pfd = { .fd = fd, .events = POLLIN };

    while (poll(&pfd, 1, 200) > 0) {
        g_assert_cmpint(recv(fd, buf, sizeof(buf), 0), >, 0);
    }
}

/*
 * Read the next response frame from @fd, SLIP-decoded, waiting up to
 * REPLY_WAIT_MS: its status bytes follow its data.
 */
static bool esp_reply(int fd, EspReply *r)
{
    int64_t end = deadline_us(REPLY_WAIT_MS);
    uint8_t frame[256];
    size_t n = 0;
    bool in = false;

    while (g_get_monotonic_time() < end) {
        struct pollfd pfd = { .fd = fd, .events = POLLIN };
        uint8_t c;

        if (poll(&pfd, 1, 10) <= 0) {
            continue;
        }
        while (recv(fd, &c, 1, MSG_DONTWAIT) == 1) {
            if (c == 0xc0) {
                if (in && n >= 10 && frame[0] == 1) {
                    r->cmd = frame[1];
                    r->len = lduw_le_p(frame + 2);
                    r->value = ldl_le_p(frame + 4);
                    g_assert_cmpint(r->len, <=, n - 8);
                    memcpy(r->data, frame + 8, MIN(r->len, sizeof(r->data)));
                    r->status = frame[8 + r->len - 4];
                    r->error = frame[8 + r->len - 3];
                    r->len -= 4;
                    return true;
                }
                in = true;
                n = 0;
            } else if (in && n < sizeof(frame)) {
                if (c == 0xdb && recv(fd, &c, 1, 0) == 1) {
                    c = c == 0xdc ? 0xc0 : 0xdb;
                }
                frame[n++] = c;
            }
        }
    }
    return false;
}

/* Send a command and take its response, after the ROM's earlier ones. */
EspReply esp_command(int fd, uint8_t cmd, const uint8_t *data,
                     size_t len, uint32_t checksum)
{
    EspReply r;

    esp_drain(fd);
    esp_send(fd, cmd, data, len, checksum);
    g_assert_true(esp_reply(fd, &r));
    return r;
}

/* READ_REG over the ROM's download protocol: the word at @addr. */
uint32_t esp_read_reg(int fd, uint32_t addr)
{
    uint8_t data[4];
    EspReply r;

    stl_le_p(data, addr);
    r = esp_command(fd, 0x0a, data, sizeof(data), 0);
    g_assert_cmpint(r.cmd, ==, 0x0a);
    g_assert_cmpint(r.status, ==, 0);
    return r.value;
}

/* WRITE_REG over the ROM's download protocol: all of @addr's bits. */
void esp_write_reg(int fd, uint32_t addr, uint32_t value)
{
    uint8_t data[16];
    EspReply r;

    stl_le_p(data, addr);
    stl_le_p(data + 4, value);
    stl_le_p(data + 8, 0xffffffff);
    stl_le_p(data + 12, 0);
    r = esp_command(fd, 0x09, data, sizeof(data), 0);
    g_assert_cmpint(r.cmd, ==, 0x09);
    g_assert_cmpint(r.status, ==, 0);
}
