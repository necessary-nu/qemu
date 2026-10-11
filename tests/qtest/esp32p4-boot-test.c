/*
 * QTest testcase for the ESP32-P4 boot: the v3.x boot ROM run from reset
 * in each boot mode the strapping and the eFuses select, and -kernel in
 * its place
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The CPUs run (TCG, icount); each test reads what the ROM prints on
 * serial port 0 (UART0) and what a small program the ROM loads leaves in
 * LP SRAM. The board's terminal on UART0's pads holds U0RXD (GPIO38) high,
 * so GPIO_STRAP reads the strapping with bit 0 set: 0x09 for SPI boot,
 * 0x05 for joint download boot and 0x01 for SPI download boot (TRM table
 * 12.2-2); 0x03 is a combination the TRM calls invalid.
 */

#include "qemu/osdep.h"
#include <poll.h>
#include "qemu/bswap.h"
#include "qemu/cutils.h"
#include "libqtest.h"

#define FLASH_SIZE          (4 * 1024 * 1024)
#define BOOTLOADER_OFFSET   0x2000
#define LOAD_ADDR           0x4ff80000
#define MAGIC_ADDR          0x5010ff00
#define MAGIC               0x600db007
#define GPIO_STRAP          0x500e0038
#define LP_SYS_SYS_CTRL     0x50110008
#define FORCE_DOWNLOAD_BOOT (1u << 2)
#define EFUSE_FILE_SIZE     336

#define EM_RISCV 243
#define PT_LOAD 1

/* Writes MAGIC to MAGIC_ADDR, then loops. */
static const uint32_t prog_magic[] = {
    0x50110537,         /* lui  a0, 0x50110 */
    0xf0050513,         /* addi a0, a0, -256 */
    0x600db5b7,         /* lui  a1, 0x600db */
    0x00758593,         /* addi a1, a1, 7 */
    0x00b52023,         /* sw   a1, 0(a0) */
    0x0000006f,         /* j    . */
};

/*
 * Waits 2M cycles (50 ms), for the ROM's messages to go out, sets
 * LP_SYSTEM_FORCE_DOWNLOAD_BOOT, then resets core 0 through
 * LP_CLKRST_HPCORE0_SW_RESET.
 */
static const uint32_t prog_force_download[] = {
    0x001006b7,         /* lui  a3, 0x100 */
    0xfff68693,         /* 1: addi a3, a3, -1 */
    0xfe069ee3,         /* bnez a3, 1b */
    0x50110537,         /* lui  a0, 0x50110 */
    0x00852583,         /* lw   a1, 8(a0) */
    0x0045e593,         /* ori  a1, a1, 4 */
    0x00b52423,         /* sw   a1, 8(a0) */
    0x50111537,         /* lui  a0, 0x50111 */
    0x01452583,         /* lw   a1, 20(a0) */
    0x00002637,         /* lui  a2, 0x2 */
    0x00c5e5b3,         /* or   a1, a1, a2 */
    0x00b52a23,         /* sw   a1, 20(a0) */
    0x0000006f,         /* j    . */
};

/*
 * Block 0's eFuse bits the boot ROM reads, in the file's words (the RD_*
 * registers': RD_REPEAT_DATAn is word n + 1).
 */
#define EFUSE_DATA(n)                   ((n) + 1)
#define DIS_FORCE_DOWNLOAD              (1u << 12)      /* DATA0 */
#define SECURE_BOOT_EN                  (1u << 20)      /* DATA2 */
#define DIS_DOWNLOAD_MODE               (1u << 0)       /* DATA3 */
#define DIS_DIRECT_BOOT                 (1u << 1)       /* DATA3 */
#define UART_PRINT_CONTROL_OFF          (3u << 6)       /* DATA3 */

static char *write_tmp(const char *tmpl, const void *data, size_t len)
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
 * one segment of @words at LOAD_ADDR, its entry point there, written into
 * @flash at @offset.
 */
static void put_image(uint8_t *flash, uint32_t offset, const uint32_t *code,
                      size_t words)
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
    stl_le_p(img + len + 4, words * 4);
    len += 8;
    for (i = 0; i < words; i++) {
        stl_le_p(img + len, code[i]);
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
}

static char *write_flash(const uint32_t *code, size_t words, bool direct)
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
        put_image(flash, BOOTLOADER_OFFSET, code, words);
    }
    return write_tmp("esp32p4-boot-XXXXXX.img", flash, FLASH_SIZE);
}

/* An eFuse file, 336 bytes of the RD_* words, with one bit field set. */
static char *write_efuse(unsigned word, uint32_t bits)
{
    uint8_t data[EFUSE_FILE_SIZE] = { 0 };

    stl_le_p(data + 4 * word, bits);
    return write_tmp("esp32p4-efuse-XXXXXX.bin", data, sizeof(data));
}

/* Write a 32-bit RISC-V ELF of @words at LOAD_ADDR, its entry point. */
static char *write_elf(const uint32_t *code, size_t words)
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

typedef struct Boot {
    QTestState *qts;
    char *log;
} Boot;

/*
 * Start the machine with serial port 0 into a log file: @strap the
 * strapping, @flash a flash image or NULL for a blank one, @efuse an eFuse
 * file or NULL for a factory-fresh chip, @kernel an ELF for -kernel or
 * NULL.
 */
static Boot boot(uint32_t strap, const char *flash, const char *efuse,
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

static void boot_end(Boot *b)
{
    qtest_quit(b->qts);
    unlink(b->log);
    g_free(b->log);
}

/* What serial port 0 has printed so far. */
static char *output(Boot *b)
{
    char *text = NULL;

    g_assert(g_file_get_contents(b->log, &text, NULL, NULL));
    return text;
}

/* Wait up to 30 s for @want on serial port 0. */
static void await_output(Boot *b, const char *want)
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
static void await_magic(Boot *b)
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

static void expect_no_magic(Boot *b)
{
    g_assert_cmphex(qtest_readl(b->qts, MAGIC_ADDR), !=, MAGIC);
}

/*
 * SPI boot (GPIO35 high): the ROM loads the image at flash offset 0x2000
 * into L2MEM, checks it and jumps to its entry point.
 */
/* [spec:nuos:req:emu.esp32p4.boot/test] */
static void test_spi_boot(void)
{
    g_autofree char *flash = write_flash(prog_magic,
                                         ARRAY_SIZE(prog_magic), false);
    Boot b = boot(0x08, flash, NULL, NULL);

    await_magic(&b);
    await_output(&b, "rst:0x1 (POWERON),boot:0x9 (SPI_FAST_FLASH_BOOT)");
    await_output(&b, "load:0x4ff80000,len:0x18");
    await_output(&b, "entry 0x4ff80000");
    boot_end(&b);
    unlink(flash);
}

/*
 * Direct boot: with no valid image at 0x2000 and the direct boot magic at
 * the flash's start, the ROM maps the flash and runs the code at
 * 0x40000008 from it, through the caches; EFUSE_DIS_DIRECT_BOOT stops it.
 */
/* [spec:nuos:req:emu.esp32p4.boot/test] */
static void test_direct_boot(void)
{
    g_autofree char *flash = write_flash(prog_magic,
                                         ARRAY_SIZE(prog_magic), true);
    g_autofree char *efuse = write_efuse(EFUSE_DATA(3), DIS_DIRECT_BOOT);
    Boot b = boot(0x08, flash, NULL, NULL);

    await_magic(&b);
    boot_end(&b);

    b = boot(0x08, flash, efuse, NULL);
    await_output(&b, "invalid header: 0xffffffff");
    await_output(&b, "ets_main.c 372");
    expect_no_magic(&b);
    boot_end(&b);
    unlink(flash);
    unlink(efuse);
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
 * Read from @fd for up to 3 s until the response to @cmd: true with its
 * first status byte (0 for success) in *status.
 */
static bool esp_response(int fd, uint8_t cmd, uint8_t *status)
{
    uint8_t buf[4096];
    size_t got = 0;

    for (int i = 0; i < 30; i++) {
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

/*
 * Joint download boot (GPIO35 low, GPIO36 high): the ROM waits for esptool
 * on UART0 (and USB Serial/JTAG and the SPI slave); over UART0 it syncs,
 * takes a program into L2MEM (MEM_BEGIN, MEM_DATA) and runs it (MEM_END).
 */
/* [spec:nuos:req:emu.esp32p4.boot/test] */
static void test_uart_download(void)
{
    static const uint8_t sync_data[36] = {
        0x07, 0x07, 0x12, 0x20,
        0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
        0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
        0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
        0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
    };
    uint8_t begin[16], data[16 + sizeof(prog_magic)], end[8];
    uint32_t checksum = 0xef;
    uint8_t status = 0xff;
    bool synced = false;
    int fd;
    QTestState *qts = qtest_init_with_serial(
        "-M esp32p4,strap-mode=0x04 -accel tcg -icount shift=0,sleep=off",
        &fd);

    for (int i = 0; i < 20 && !synced; i++) {
        esp_send(fd, 0x08, sync_data, sizeof(sync_data), 0);
        synced = esp_response(fd, 0x08, &status);
    }
    g_assert(synced);
    g_assert_cmpint(status, ==, 0);
    /* Let the ROM's further SYNC responses out before the next command. */
    g_usleep(200000);

    stl_le_p(begin, sizeof(prog_magic));
    stl_le_p(begin + 4, 1);
    stl_le_p(begin + 8, sizeof(prog_magic));
    stl_le_p(begin + 12, LOAD_ADDR);
    esp_send(fd, 0x05, begin, sizeof(begin), 0);
    g_assert(esp_response(fd, 0x05, &status));
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
    g_assert(esp_response(fd, 0x07, &status));
    g_assert_cmpint(status, ==, 0);

    stl_le_p(end, 0);                   /* run it */
    stl_le_p(end + 4, LOAD_ADDR);
    esp_send(fd, 0x06, end, sizeof(end), 0);
    for (int i = 0; i < 300 && qtest_readl(qts, MAGIC_ADDR) != MAGIC; i++) {
        g_usleep(100000);
    }
    g_assert_cmphex(qtest_readl(qts, MAGIC_ADDR), ==, MAGIC);
    close(fd);
    qtest_quit(qts);
}

/*
 * The download modes' banners: joint download boot, SPI download boot
 * (GPIO35 to GPIO37 low, GPIO38 high), and the combination the TRM calls
 * invalid, GPIO35 and GPIO36 low with GPIO37 high, on which the ROM stops.
 */
/* [spec:nuos:req:emu.esp32p4.boot/test] */
static void test_download_modes(void)
{
    Boot b = boot(0x04, NULL, NULL, NULL);

    await_output(&b, "boot:0x5 (DOWNLOAD(USB/UART0/SPI))");
    await_output(&b, "waiting for download");
    boot_end(&b);

    b = boot(0x00, NULL, NULL, NULL);
    await_output(&b, "boot:0x1 (SPI_DOWNLOAD_BOOT)");
    await_output(&b, "wait spi download");
    boot_end(&b);

    b = boot(0x02, NULL, NULL, NULL);
    await_output(&b, "ets_main.c 273");
    boot_end(&b);
}

/*
 * LP_SYSTEM_FORCE_DOWNLOAD_BOOT and a CPU reset take a chip strapped for
 * SPI boot into joint download boot; with EFUSE_DIS_FORCE_DOWNLOAD set it
 * boots from flash again.
 */
/* [spec:nuos:req:emu.esp32p4.boot/test] */
static void test_force_download(void)
{
    g_autofree char *flash = write_flash(prog_force_download,
                                         ARRAY_SIZE(prog_force_download),
                                         false);
    g_autofree char *efuse = write_efuse(EFUSE_DATA(0), DIS_FORCE_DOWNLOAD);
    Boot b = boot(0x08, flash, NULL, NULL);

    await_output(&b, "rst:0xc (SW_CPU_RESET),boot:0x5 "
                     "(DOWNLOAD(USB/UART0/SPI))");
    await_output(&b, "waiting for download");
    g_assert_cmphex(qtest_readl(b.qts, GPIO_STRAP), ==, 0x05);
    boot_end(&b);

    b = boot(0x08, flash, efuse, NULL);
    await_output(&b, "rst:0xc (SW_CPU_RESET),boot:0x9 (SPI_FAST_FLASH_BOOT)");
    g_assert_cmphex(qtest_readl(b.qts, LP_SYS_SYS_CTRL) &
                    FORCE_DOWNLOAD_BOOT, ==, FORCE_DOWNLOAD_BOOT);
    g_assert_cmphex(qtest_readl(b.qts, GPIO_STRAP), ==, 0x09);
    boot_end(&b);
    unlink(flash);
    unlink(efuse);
}

/* EFUSE_DIS_DOWNLOAD_MODE refuses the download modes. */
/* [spec:nuos:req:emu.esp32p4.boot/test] */
static void test_download_disabled(void)
{
    g_autofree char *efuse = write_efuse(EFUSE_DATA(3), DIS_DOWNLOAD_MODE);
    Boot b = boot(0x04, NULL, efuse, NULL);

    await_output(&b, "Download boot modes disabled");
    boot_end(&b);
    unlink(efuse);
}

/*
 * EFUSE_UART_PRINT_CONTROL 3: the ROM prints nothing on UART0, and boots
 * the image all the same.
 */
/* [spec:nuos:req:emu.esp32p4.boot/test] */
static void test_print_control(void)
{
    g_autofree char *flash = write_flash(prog_magic,
                                         ARRAY_SIZE(prog_magic), false);
    g_autofree char *efuse = write_efuse(EFUSE_DATA(3),
                                         UART_PRINT_CONTROL_OFF);
    g_autofree char *text = NULL;
    Boot b = boot(0x08, flash, efuse, NULL);

    await_magic(&b);
    text = output(&b);
    g_assert_cmpstr(text, ==, "");
    boot_end(&b);
    unlink(flash);
    unlink(efuse);
}

/* Core 0's PC, from "info registers". */
static uint32_t core0_pc(QTestState *qts)
{
    g_autofree char *regs = qtest_hmp(qts, "info registers");
    const char *p = strstr(regs, " pc ");
    const char *end;
    unsigned long pc;

    g_assert(p);
    g_assert_cmpint(qemu_strtoul(p + 4, &end, 16, &pc), ==, 0);
    return pc;
}

/*
 * -kernel takes the ROM's place only where the ROM would boot the image
 * from flash: strapped for SPI boot the ELF runs at once and the ROM
 * prints nothing; strapped for joint download boot core 0 runs the ROM,
 * which waits for a download; with Secure Boot enabled it runs the ROM,
 * which boots only signed images.
 */
/* [spec:nuos:req:emu.esp32p4.boot/test] */
static void test_kernel(void)
{
    g_autofree char *elf = write_elf(prog_magic, ARRAY_SIZE(prog_magic));
    g_autofree char *efuse = write_efuse(EFUSE_DATA(2), SECURE_BOOT_EN);
    g_autofree char *text = NULL;
    Boot b = boot(0x08, NULL, NULL, elf);

    await_magic(&b);
    text = output(&b);
    g_assert_null(strstr(text, "ESP-ROM"));
    boot_end(&b);

    b = boot(0x04, NULL, NULL, elf);
    await_output(&b, "waiting for download");
    expect_no_magic(&b);
    boot_end(&b);

    b = boot(0x08, NULL, efuse, elf);
    g_usleep(500000);
    g_assert_cmphex(core0_pc(b.qts) & 0xfffe0000, ==, 0x4fc00000);
    expect_no_magic(&b);
    boot_end(&b);
    unlink(elf);
    unlink(efuse);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/esp32p4/boot/spi-boot", test_spi_boot);
    qtest_add_func("/esp32p4/boot/direct-boot", test_direct_boot);
    qtest_add_func("/esp32p4/boot/uart-download", test_uart_download);
    qtest_add_func("/esp32p4/boot/download-modes", test_download_modes);
    qtest_add_func("/esp32p4/boot/force-download", test_force_download);
    qtest_add_func("/esp32p4/boot/download-disabled",
                   test_download_disabled);
    qtest_add_func("/esp32p4/boot/print-control", test_print_control);
    qtest_add_func("/esp32p4/boot/kernel", test_kernel);
    return g_test_run();
}
