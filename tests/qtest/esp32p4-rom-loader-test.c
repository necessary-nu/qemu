/*
 * QTest testcase for esptool's protocol with the ESP32-P4's boot ROM
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The boot ROM in joint download mode (strap-mode 0x04) waits for esptool
 * on serial port 0 (UART0, through the board's terminal) and on USB
 * Serial/JTAG. Over each, as esptool without its stub drives it: SYNC,
 * answered within a few tries as on silicon; READ_REG of the chip's
 * revision word; WRITE_REG and READ_REG of L2MEM; 4 KiB written with
 * MEM_BEGIN and MEM_DATA and read back word by word with READ_REG, as
 * esptool's dump-mem reads memory; and a program loaded and run with
 * MEM_END. Each runs under icount and on the host's clock, where the
 * models' timers run as late as the host makes them: the ROM must lose
 * no byte and no reply either way.
 */

#include "qemu/osdep.h"
#include <poll.h>
#include "qemu/bswap.h"
#include "libqtest.h"

#define USJ                 0x500d2000
#define USJ_CHIP_RST        (USJ + 0x4c)
#define CHIP_RST_DTR        (1u << 1)

#define SYSTIMER            0x500e2000
#define SYSTIMER_UNIT0_OP   (SYSTIMER + 0x04)
#define SYSTIMER_UNIT0_HI   (SYSTIMER + 0x40)
#define SYSTIMER_UNIT0_LO   (SYSTIMER + 0x44)
#define SYSTIMER_UPDATE     (1u << 30)
#define SYSTIMER_PER_MS     16000

/* Where the loaded program goes, and where data goes */
#define PROG_ADDR           0x4ff80000
#define DATA_ADDR           0x4ff84000
#define DATA_BYTES          4096
#define BLOCK_BYTES         1024
#define MAGIC_ADDR          0x5010ff00
#define MAGIC               0x600db007

#define CMD_MEM_BEGIN       0x05
#define CMD_MEM_END         0x06
#define CMD_MEM_DATA        0x07
#define CMD_SYNC            0x08
#define CMD_WRITE_REG       0x09
#define CMD_READ_REG        0x0a
#define CHECKSUM_SEED       0xef

/*
 * esptool's SYNC timeout, how many SYNCs its connect attempt sends, and
 * its timeout for other commands' responses
 */
#define SYNC_WAIT_MS        100
#define SYNC_TRIES          5
#define REPLY_WAIT_MS       3000

/* Writes MAGIC to MAGIC_ADDR, then loops. */
static const uint32_t prog_magic[] = {
    0x50110537,         /* lui  a0, 0x50110 */
    0xf0050513,         /* addi a0, a0, -256 */
    0x600db5b7,         /* lui  a1, 0x600db */
    0x00758593,         /* addi a1, a1, 7 */
    0x00b52023,         /* sw   a1, 0(a0) */
    0x0000006f,         /* j    . */
};

typedef struct Port {
    QTestState *qts;
    int fd;
    /* Bytes received and not yet taken as a frame */
    GByteArray *rx;
} Port;

typedef struct Reply {
    uint8_t cmd;
    uint32_t val;
    uint8_t status;
} Reply;

static int64_t ms_now(void)
{
    return g_get_monotonic_time() / 1000;
}

/*
 * The chip's time in milliseconds, from the system timer's unit 0, which
 * the boot ROM leaves alone. esptool's timeouts are the chip's time on
 * silicon; under icount on a busy host the chip runs slower than the
 * wall clock, so they are measured on the chip.
 */
static uint64_t chip_ms(Port *p)
{
    uint64_t ticks;

    qtest_writel(p->qts, SYSTIMER_UNIT0_OP, SYSTIMER_UPDATE);
    ticks = qtest_readl(p->qts, SYSTIMER_UNIT0_HI);
    ticks = ticks << 32 | qtest_readl(p->qts, SYSTIMER_UNIT0_LO);
    return ticks / SYSTIMER_PER_MS;
}

/* SLIP-frame and send an esptool command with its data and checksum. */
static void esp_send(Port *p, uint8_t cmd, const uint8_t *data, size_t len,
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
    g_assert_cmpint(send(p->fd, out->data, out->len, 0), ==, out->len);
    g_byte_array_free(out, true);
}

/*
 * Take the next SLIP frame from what has been received into @frame;
 * false if no whole frame is there yet. Bytes before a frame's start
 * (the ROM's banner) are dropped.
 */
static bool slip_take(Port *p, GByteArray *frame)
{
    guint start, i;

    for (start = 0; start < p->rx->len && p->rx->data[start] != 0xc0;
         start++) {
    }
    for (i = start + 1; i < p->rx->len && p->rx->data[i] != 0xc0; i++) {
    }
    if (i >= p->rx->len) {
        g_byte_array_remove_range(p->rx, 0, start);
        return false;
    }
    g_byte_array_set_size(frame, 0);
    for (guint j = start + 1; j < i; j++) {
        uint8_t b = p->rx->data[j];

        if (b == 0xdb && j + 1 < i) {
            b = p->rx->data[++j] == 0xdc ? 0xc0 : 0xdb;
        }
        g_byte_array_append(frame, &b, 1);
    }
    /* The closing 0xc0 may open the next frame: keep it. */
    g_byte_array_remove_range(p->rx, 0, i);
    return true;
}

/*
 * Wait up to @ms of the chip's time for the response to @cmd, skipping
 * others (SYNC's extra responses), as esptool does: true with its value
 * and first status byte in @r.
 */
static bool esp_reply(Port *p, uint8_t cmd, Reply *r, int ms)
{
    g_autoptr(GByteArray) frame = g_byte_array_new();
    uint64_t deadline = chip_ms(p) + ms;

    for (;;) {
        struct pollfd pfd = { .fd = p->fd, .events = POLLIN };

        while (slip_take(p, frame)) {
            if (frame->len >= 10 && frame->data[0] == 1 &&
                frame->data[1] == cmd) {
                r->cmd = cmd;
                r->val = ldl_le_p(frame->data + 4);
                r->status = frame->data[8];
                return true;
            }
        }
        if (chip_ms(p) >= deadline) {
            return false;
        }
        if (poll(&pfd, 1, 10) > 0) {
            uint8_t buf[4096];
            ssize_t n = recv(p->fd, buf, sizeof(buf), 0);

            g_assert_cmpint(n, >, 0);
            g_byte_array_append(p->rx, buf, n);
        }
    }
}

static Reply esp_command(Port *p, uint8_t cmd, const uint8_t *data,
                         size_t len, uint32_t checksum)
{
    Reply r;

    esp_send(p, cmd, data, len, checksum);
    if (!esp_reply(p, cmd, &r, REPLY_WAIT_MS)) {
        g_test_message("no response to command 0x%02x", cmd);
        g_assert_not_reached();
    }
    g_assert_cmpint(r.status, ==, 0);
    return r;
}

static const uint8_t sync_data[36] = {
    0x07, 0x07, 0x12, 0x20,
    0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
    0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
    0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
    0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
};

/*
 * SYNC as esptool sends it, waiting 100 ms for each response: the ROM
 * measures the baud rate from the first, so it answers within a few.
 * The ROM's further responses to it are skipped as later commands wait
 * for theirs, as esptool skips them.
 */
static void esp_sync(Port *p)
{
    Reply r;
    int tries;

    for (tries = 1; tries <= SYNC_TRIES; tries++) {
        esp_send(p, CMD_SYNC, sync_data, sizeof(sync_data), 0);
        if (esp_reply(p, CMD_SYNC, &r, SYNC_WAIT_MS)) {
            break;
        }
    }
    if (tries > SYNC_TRIES) {
        g_test_message("no response to %d SYNCs", SYNC_TRIES);
        g_assert_not_reached();
    }
    g_assert_cmpint(r.status, ==, 0);
    g_assert_cmphex(r.val, ==, 0x20120707);
}

static uint32_t read_reg(Port *p, uint32_t addr)
{
    uint8_t d[4];

    stl_le_p(d, addr);
    return esp_command(p, CMD_READ_REG, d, sizeof(d), 0).val;
}

static void write_reg(Port *p, uint32_t addr, uint32_t val)
{
    uint8_t d[16];

    stl_le_p(d, addr);
    stl_le_p(d + 4, val);
    stl_le_p(d + 8, 0xffffffff);
    stl_le_p(d + 12, 0);
    esp_command(p, CMD_WRITE_REG, d, sizeof(d), 0);
}

static void mem_begin(Port *p, uint32_t size, uint32_t blocks,
                      uint32_t block_size, uint32_t addr)
{
    uint8_t d[16];

    stl_le_p(d, size);
    stl_le_p(d + 4, blocks);
    stl_le_p(d + 8, block_size);
    stl_le_p(d + 12, addr);
    esp_command(p, CMD_MEM_BEGIN, d, sizeof(d), 0);
}

static void mem_data(Port *p, uint32_t seq, const uint8_t *buf, size_t len)
{
    g_autofree uint8_t *d = g_malloc(16 + len);
    uint32_t checksum = CHECKSUM_SEED;

    stl_le_p(d, len);
    stl_le_p(d + 4, seq);
    stl_le_p(d + 8, 0);
    stl_le_p(d + 12, 0);
    memcpy(d + 16, buf, len);
    for (size_t i = 0; i < len; i++) {
        checksum ^= buf[i];
    }
    esp_command(p, CMD_MEM_DATA, d, 16 + len, checksum);
}

/* The data word at byte offset @off: every byte value, 0xc0 and 0xdb too */
static uint32_t pattern(uint32_t off)
{
    return (off * 0x01010101u + 0x03020100u) ^ (off << 14);
}

static void check_loader(Port *p)
{
    uint8_t buf[DATA_BYTES];
    uint8_t end[8];

    esp_sync(p);

    /* The ROM's revision word, which esptool reads first */
    g_assert_cmphex(read_reg(p, 0x4fc00014), ==,
                    qtest_readl(p->qts, 0x4fc00014));

    write_reg(p, DATA_ADDR, 0xc0dbc0db);
    g_assert_cmphex(read_reg(p, DATA_ADDR), ==, 0xc0dbc0db);

    for (uint32_t off = 0; off < DATA_BYTES; off += 4) {
        stl_le_p(buf + off, pattern(off));
    }
    mem_begin(p, DATA_BYTES, DATA_BYTES / BLOCK_BYTES, BLOCK_BYTES,
              DATA_ADDR);
    for (uint32_t b = 0; b < DATA_BYTES / BLOCK_BYTES; b++) {
        mem_data(p, b, buf + b * BLOCK_BYTES, BLOCK_BYTES);
    }
    for (uint32_t off = 0; off < DATA_BYTES; off += 4) {
        uint32_t v = read_reg(p, DATA_ADDR + off);

        if (v != pattern(off)) {
            g_test_message("word %u of %u", off / 4, DATA_BYTES / 4);
            g_assert_cmphex(v, ==, pattern(off));
        }
    }

    /* A program, run by MEM_END */
    mem_begin(p, sizeof(prog_magic), 1, sizeof(prog_magic), PROG_ADDR);
    for (size_t i = 0; i < ARRAY_SIZE(prog_magic); i++) {
        stl_le_p(buf + 4 * i, prog_magic[i]);
    }
    mem_data(p, 0, buf, sizeof(prog_magic));
    stl_le_p(end, 0);
    stl_le_p(end + 4, PROG_ADDR);
    esp_send(p, CMD_MEM_END, end, sizeof(end), 0);
    for (int i = 0; i < 300 && qtest_readl(p->qts, MAGIC_ADDR) != MAGIC; i++) {
        g_usleep(100 * 1000);
    }
    g_assert_cmphex(qtest_readl(p->qts, MAGIC_ADDR), ==, MAGIC);
}

/*
 * Wait up to 30 s for the ROM's banner to end with "waiting for download"
 * on serial port 0, as one waits for the board to come up before esptool.
 */
static void await_banner(Port *p)
{
    static const char want[] = "waiting for download";
    int64_t deadline = ms_now() + 30000;

    while (!memmem(p->rx->data, p->rx->len, want, strlen(want))) {
        struct pollfd pfd = { .fd = p->fd, .events = POLLIN };
        uint8_t buf[256];

        g_assert_cmpint(ms_now(), <, deadline);
        if (poll(&pfd, 1, 100) > 0) {
            ssize_t n = recv(p->fd, buf, sizeof(buf), 0);

            g_assert_cmpint(n, >, 0);
            g_byte_array_append(p->rx, buf, n);
        }
    }
    g_byte_array_set_size(p->rx, 0);
}

static const char *timing(bool icount)
{
    return icount ? "-accel tcg -icount shift=0,sleep=off" : "-accel tcg";
}

/* [spec:nuos:req:emu.esp32p4.uart/test] */
/* [spec:nuos:req:emu.esp32p4.boot/test] */
static void run_uart0(bool icount)
{
    g_autofree char *args = g_strdup_printf("-M esp32p4,strap-mode=0x04 %s",
                                            timing(icount));
    Port p = { .rx = g_byte_array_new() };

    p.qts = qtest_init_with_serial(args, &p.fd);
    await_banner(&p);
    check_loader(&p);
    close(p.fd);
    qtest_quit(p.qts);
    g_byte_array_free(p.rx, true);
}

/*
 * USB Serial/JTAG: the host enumerates the port and opens it (DTR) before
 * esptool sends anything, as on a computer.
 */
/* [spec:nuos:req:emu.esp32p4.uart/test] */
/* [spec:nuos:req:emu.esp32p4.boot/test] */
static void run_usj(bool icount)
{
    g_autofree char *dir = g_dir_make_tmp("esp32p4-rom-loader-XXXXXX", NULL);
    g_autofree char *path = g_strdup_printf("%s/sock", dir);
    int srv = qtest_socket_server(path);
    Port p = { .rx = g_byte_array_new() };
    int i;

    p.qts = qtest_initf("-M esp32p4,strap-mode=0x04 %s -serial none "
                        "-chardev socket,id=usj,path=%s "
                        "-global esp32p4-usb-serial-jtag.chardev=usj",
                        timing(icount), path);
    p.fd = accept(srv, NULL, NULL);
    g_assert_cmpint(p.fd, >=, 0);
    close(srv);
    unlink(path);
    rmdir(dir);
    for (i = 0; i < 300; i++) {
        if (qtest_readl(p.qts, USJ_CHIP_RST) & CHIP_RST_DTR) {
            break;
        }
        g_usleep(100 * 1000);
    }
    g_assert_cmpint(i, <, 300);
    check_loader(&p);
    close(p.fd);
    qtest_quit(p.qts);
    g_byte_array_free(p.rx, true);
}

static void test_uart0_icount(void)
{
    run_uart0(true);
}

static void test_uart0_host_clock(void)
{
    run_uart0(false);
}

static void test_usj_icount(void)
{
    run_usj(true);
}

static void test_usj_host_clock(void)
{
    run_usj(false);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("esp32p4/rom-loader/uart0/icount", test_uart0_icount);
    qtest_add_func("esp32p4/rom-loader/uart0/host-clock",
                   test_uart0_host_clock);
    qtest_add_func("esp32p4/rom-loader/usj/icount", test_usj_icount);
    qtest_add_func("esp32p4/rom-loader/usj/host-clock", test_usj_host_clock);
    return g_test_run();
}
