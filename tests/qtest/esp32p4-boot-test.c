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
 * 12.2-2); 0x03 is a combination the TRM calls invalid. With the terminal
 * holding a break through the reset GPIO38 reads low: 0x02 and 0x00, which
 * the TRM calls invalid too, are the ROM's UART0_BOOT and USB_BOOT.
 *
 * The image's digest and Secure Boot (esp32p4-boot-secure-test), the
 * downloads over UART0 (esp32p4-boot-uart-test), over the USB OTG
 * (esp32p4-boot-usb-test) and esptool's commands in download mode
 * (esp32p4-boot-esptool-test), which each take seconds, are tested
 * apart, each program within the time a qtest has.
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qemu/cutils.h"
#include "libqtest.h"
#include "esp32p4-boot-utils.h"

#define LP_SYS_SYS_CTRL                 0x50110008
#define FORCE_DOWNLOAD_BOOT             (1u << 2)

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

/*
 * The SPI slave download. The ROM's SPI download boot makes GP-SPI2 a
 * half-duplex slave on its IO MUX pads (GPIO6 HD, 7 CS, 8 D, 9 CK, 10 Q,
 * 11 WP) and speaks the protocol of Espressif's esp-serial-flasher
 * (protocol_spi.c): status words in its buffer that the host reads and
 * writes with Rd_BUF and Wr_BUF (RXSTA at 4, TXSTA at 8: a toggle bit, an
 * init bit and the length the slave is ready for above them), commands
 * in esptool's framing without SLIP sent with Wr_DMA and closed with
 * CMD7, responses read with Rd_DMA and closed with CMD8.
 *
 * The host here is the chip's own GP-SPI3 as the master the board wires to
 * those pads: its CK, D and CS0 out on GPIO20, 21 and 23 and its Q in on
 * GPIO22, through the GPIO matrix, joined to GPIO9, 8, 7 and 10 by the
 * board's wires (the GPIO block's "pad-wires"). The ROM does not touch
 * GP-SPI3, which runs on its reset clocks.
 */
#define SPI3_BASE           0x500d1000
#define SPI_CMD             0x00
#define SPI_ADDR            0x04
#define SPI_USER            0x10
#define SPI_USER1           0x14
#define SPI_USER2           0x18
#define SPI_MS_DLEN         0x1c
#define SPI_W(n)            (0x98 + 4 * (n))
#define SPI_CMD_UPDATE      (1u << 23)
#define SPI_CMD_USR         (1u << 24)
#define SPI_CS_HOLD         (1u << 6)
#define SPI_CS_SETUP        (1u << 7)
#define SPI_USR_MOSI        (1u << 27)
#define SPI_USR_MISO        (1u << 28)
#define SPI_USR_DUMMY       (1u << 29)
#define SPI_USR_ADDR        (1u << 30)
#define SPI_USR_COMMAND     (1u << 31)
#define GPIO_FUNC_IN(s)     (0x500e0158 + 4 * (s))
#define GPIO_FUNC_OUT(n)    (0x500e0558 + 4 * (n))
#define IO_MUX_PAD(n)       (0x500e1004 + 4 * (n))
#define SIG_SPI3_CK         47
#define SIG_SPI3_Q          48
#define SIG_SPI3_D          49
#define SIG_SPI3_CS         52
#define SLAVE_RXSTA         4
#define SLAVE_TXSTA         8
#define SLAVE_CMD           12

/* Pad @pad carries GPIO matrix output @out and feeds input @in. */
static void spi_host_pad(QTestState *qts, int pad, int out, int in)
{
    qtest_writel(qts, IO_MUX_PAD(pad), (1u << 12) | (1u << 9) | (2u << 10));
    qtest_writel(qts, GPIO_FUNC_OUT(pad), out);
    if (in >= 0) {
        qtest_writel(qts, GPIO_FUNC_IN(in), pad | (1u << 7));
    }
}

/* A GP-SPI3 command register's bit, set and run until it clears. */
static void spi_host_cmd(QTestState *qts, uint32_t bit)
{
    qtest_writel(qts, SPI3_BASE + SPI_CMD, bit);
    for (int i = 0; i < 1000 && qtest_readl(qts, SPI3_BASE + SPI_CMD) & bit;
         i++) {
        g_usleep(1000);
    }
    g_assert_cmphex(qtest_readl(qts, SPI3_BASE + SPI_CMD) & bit, ==, 0);
}

/*
 * A half-duplex transaction: command, address, 8 dummy cycles, then
 * @len bytes out of @out or into @in.
 */
static void spi_host_hd(QTestState *qts, uint8_t cmd, uint8_t addr,
                        const uint8_t *out, uint8_t *in, size_t len)
{
    uint32_t user = SPI_USR_COMMAND | SPI_USR_ADDR | SPI_USR_DUMMY |
                    SPI_CS_SETUP | SPI_CS_HOLD;
    uint8_t buf[64] = { 0 };

    g_assert_cmpuint(len, <=, sizeof(buf));
    if (len) {
        user |= out ? SPI_USR_MOSI : SPI_USR_MISO;
        qtest_writel(qts, SPI3_BASE + SPI_MS_DLEN, len * 8 - 1);
    }
    if (out) {
        memcpy(buf, out, len);
    }
    for (int i = 0; i < 16; i++) {
        qtest_writel(qts, SPI3_BASE + SPI_W(i), ldl_le_p(buf + 4 * i));
    }
    qtest_writel(qts, SPI3_BASE + SPI_USER, user);
    qtest_writel(qts, SPI3_BASE + SPI_USER1,
                 (7u << 27) | (8 - 1) | (1u << 22));
    qtest_writel(qts, SPI3_BASE + SPI_USER2, (7u << 28) | cmd);
    qtest_writel(qts, SPI3_BASE + SPI_ADDR, (uint32_t)addr << 24);
    spi_host_cmd(qts, SPI_CMD_UPDATE);
    spi_host_cmd(qts, SPI_CMD_USR);
    if (in) {
        for (int i = 0; i < 16; i++) {
            stl_le_p(buf + 4 * i, qtest_readl(qts, SPI3_BASE + SPI_W(i)));
        }
        memcpy(in, buf, len);
    }
}

static uint32_t spi_slave_rd(QTestState *qts, uint8_t reg)
{
    uint8_t v[4];

    spi_host_hd(qts, 0x02, reg, NULL, v, 4);        /* Rd_BUF */
    return ldl_le_p(v);
}

static void spi_slave_wr(QTestState *qts, uint8_t reg, uint32_t value)
{
    uint8_t v[4];

    stl_le_p(v, value);
    spi_host_hd(qts, 0x01, reg, v, NULL, 4);        /* Wr_BUF */
}

/*
 * Wait, as esp-serial-flasher's handle_slave_state() does, for the slave
 * to be ready on the status word @reg, whose toggle bit was *seq: the
 * length it is ready for.
 */
static uint32_t spi_slave_ready(QTestState *qts, uint8_t reg, uint32_t *seq)
{
    for (int i = 0; i < 1000; i++) {
        uint32_t sta = spi_slave_rd(qts, reg);

        if ((sta & 3) == 3) {
            spi_slave_wr(qts, reg, 0);
        } else if ((sta & 3) == 2 || (sta & 1) != *seq) {
            *seq = sta & 1;
            return sta >> 2;
        }
        g_usleep(1000);
    }
    g_assert_not_reached();
}

/*
 * Send a command over the SPI slave protocol and read its response: its
 * status byte.
 *
 * The ROM starts the slave with TXSTA in its init state and posts its
 * first response there without waiting for the host to acknowledge that
 * state. esp-serial-flasher acknowledges it only as it waits for that
 * response, its write a few microseconds after its read; this host, each
 * of whose transactions is a round trip through qtest while the chip runs
 * on, can take long enough for the response's status to come in between,
 * which its write then clears. So it acknowledges TXSTA's init state
 * before it sends a command.
 */
static int spi_slave_command(QTestState *qts, uint32_t seq[2], uint8_t op,
                             const uint8_t *data, size_t len,
                             uint32_t checksum)
{
    uint8_t pkt[64], rsp[12];

    g_assert_cmpuint(8 + len, <=, sizeof(pkt));
    g_assert_cmpuint(spi_slave_ready(qts, SLAVE_RXSTA, &seq[0]), >=, 8 + len);
    if ((spi_slave_rd(qts, SLAVE_TXSTA) & 3) == 3) {
        spi_slave_wr(qts, SLAVE_TXSTA, 0);
    }
    pkt[0] = 0;
    pkt[1] = op;
    stw_le_p(pkt + 2, len);
    stl_le_p(pkt + 4, checksum);
    memcpy(pkt + 8, data, len);
    spi_host_hd(qts, 0x03, 0, pkt, NULL, 8 + len);  /* Wr_DMA */
    spi_host_hd(qts, 0x07, 0, NULL, NULL, 0);       /* CMD7: done */
    g_assert_cmpuint(spi_slave_ready(qts, SLAVE_TXSTA, &seq[1]), >=,
                     sizeof(rsp));
    spi_host_hd(qts, 0x04, 0, NULL, rsp, sizeof(rsp)); /* Rd_DMA */
    spi_host_hd(qts, 0x08, 0, NULL, NULL, 0);       /* CMD8: done */
    g_assert_cmpint(rsp[0], ==, 1);
    g_assert_cmpint(rsp[1], ==, op);
    return rsp[8];
}

/*
 * SPI download boot (GPIO35 to GPIO37 low, GPIO38 high): the ROM waits for
 * a master on GP-SPI2's pads, takes a program into L2MEM (MEM_BEGIN,
 * MEM_DATA) and runs it (MEM_END).
 */
/* [spec:nuos:req:emu.esp32p4.boot/test] */
static void test_spi_slave_download(void)
{
    uint8_t begin[16], data[16 + sizeof(prog_magic)], end[8];
    uint32_t checksum = 0xef;
    uint32_t seq[2] = { 0, 0 };
    Boot b;
    int fd = g_file_open_tmp("esp32p4-boot-XXXXXX.log", &b.log, NULL);

    close(fd);
    b.qts = qtest_initf("-M esp32p4,strap-mode=0x00 -accel tcg "
                        "-icount shift=0,sleep=off "
                        "-global esp32p4-gpio.pad-wires=20:9,21:8,22:10,23:7 "
                        "-chardev file,id=s0,path=%s -serial chardev:s0",
                        b.log);
    await_output(&b, "boot:0x1 (SPI_DOWNLOAD_BOOT)");
    await_output(&b, "wait spi download");
    spi_host_pad(b.qts, 20, SIG_SPI3_CK, -1);
    spi_host_pad(b.qts, 21, SIG_SPI3_D, -1);
    spi_host_pad(b.qts, 22, 0x100, SIG_SPI3_Q);
    spi_host_pad(b.qts, 23, SIG_SPI3_CS, -1);

    stl_le_p(begin, sizeof(prog_magic));
    stl_le_p(begin + 4, 1);
    stl_le_p(begin + 8, sizeof(prog_magic));
    stl_le_p(begin + 12, LOAD_ADDR);
    g_assert_cmpint(spi_slave_command(b.qts, seq, 0x05, begin, sizeof(begin),
                                      0), ==, 0);
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
    g_assert_cmpint(spi_slave_command(b.qts, seq, 0x07, data, sizeof(data),
                                      checksum), ==, 0);
    stl_le_p(end, 0);
    stl_le_p(end + 4, LOAD_ADDR);
    g_assert_cmpint(spi_slave_command(b.qts, seq, 0x06, end, sizeof(end), 0),
                    ==, 0);
    await_magic(&b);
    g_assert_cmphex(spi_slave_rd(b.qts, SLAVE_CMD), ==, 0x55);
    boot_end(&b);
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
    qtest_add_func("/esp32p4/boot/download-modes", test_download_modes);
    qtest_add_func("/esp32p4/boot/force-download", test_force_download);
    qtest_add_func("/esp32p4/boot/download-disabled", test_download_disabled);
    qtest_add_func("/esp32p4/boot/print-control", test_print_control);
    qtest_add_func("/esp32p4/boot/kernel", test_kernel);
    qtest_add_func("/esp32p4/boot/spi-slave-download", test_spi_slave_download);
    return g_test_run();
}
