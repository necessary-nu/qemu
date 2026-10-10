/*
 * QTest testcase for the RP2350 USB controller
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <sys/socket.h>
#include <sys/un.h>
#include "libqtest.h"
#include "qobject/qdict.h"
#include "rp2350-clocks.h"
#include "rp2350-resets.h"

#define XOR                     0x1000
#define SET                     0x2000
#define CLR                     0x3000

#define DPRAM                   0x50100000
#define REGS                    0x50110000
#define ADDR_ENDP(n)            (REGS + 0x00 + 4 * (n))
#define MAIN_CTRL               (REGS + 0x40)
#define SOF_WR                  (REGS + 0x44)
#define SOF_RD                  (REGS + 0x48)
#define SIE_CTRL                (REGS + 0x4c)
#define SIE_STATUS              (REGS + 0x50)
#define INT_EP_CTRL             (REGS + 0x54)
#define BUFF_STATUS             (REGS + 0x58)
#define BUFF_CPU_SHOULD_HANDLE  (REGS + 0x5c)
#define EP_ABORT                (REGS + 0x60)
#define EP_ABORT_DONE           (REGS + 0x64)
#define EP_STALL_ARM            (REGS + 0x68)
#define NAK_POLL                (REGS + 0x6c)
#define EP_STATUS_STALL_NAK     (REGS + 0x70)
#define USB_MUXING              (REGS + 0x74)
#define USB_PWR                 (REGS + 0x78)
#define USBPHY_DIRECT           (REGS + 0x7c)
#define USBPHY_DIRECT_OVERRIDE  (REGS + 0x80)
#define USBPHY_TRIM             (REGS + 0x84)
#define LINESTATE_TUNING        (REGS + 0x88)
#define INTR                    (REGS + 0x8c)
#define INTE                    (REGS + 0x90)
#define INTF                    (REGS + 0x94)
#define INTS                    (REGS + 0x98)
#define SOF_TIMESTAMP_RAW       (REGS + 0x100)
#define SOF_TIMESTAMP_LAST      (REGS + 0x104)
#define SM_STATE                (REGS + 0x108)
#define EP_TX_ERROR             (REGS + 0x10c)
#define EP_RX_ERROR             (REGS + 0x110)
#define DEV_SM_WATCHDOG         (REGS + 0x114)

#define MAIN_CTRL_EN            (1u << 0)
#define MAIN_CTRL_HOST          (1u << 1)
#define MAIN_CTRL_PHY_ISO       (1u << 2)

#define SIE_CTRL_START_TRANS    (1u << 0)
#define SIE_CTRL_SEND_SETUP     (1u << 1)
#define SIE_CTRL_SEND_DATA      (1u << 2)
#define SIE_CTRL_RECEIVE_DATA   (1u << 3)
#define SIE_CTRL_STOP_TRANS     (1u << 4)
#define SIE_CTRL_SOF_EN         (1u << 9)
#define SIE_CTRL_KEEP_ALIVE_EN  (1u << 10)
#define SIE_CTRL_VBUS_EN        (1u << 11)
#define SIE_CTRL_RESUME         (1u << 12)
#define SIE_CTRL_RESET_BUS      (1u << 13)
#define SIE_CTRL_PULLDOWN_EN    (1u << 15)
#define SIE_CTRL_PULLUP_EN      (1u << 16)
#define SIE_CTRL_EP0_INT_NAK    (1u << 27)
#define SIE_CTRL_EP0_INT_1BUF   (1u << 29)
#define SIE_CTRL_EP0_INT_STALL  (1u << 31)

#define ST_VBUS_DETECTED        (1u << 0)
#define ST_LINE_STATE(v)        (((v) >> 2) & 3)
#define ST_SUSPENDED            (1u << 4)
#define ST_SPEED(v)             (((v) >> 8) & 3)
#define ST_RESUME               (1u << 11)
#define ST_RX_SHORT_PACKET      (1u << 12)
#define ST_CONNECTED            (1u << 16)
#define ST_SETUP_REC            (1u << 17)
#define ST_TRANS_COMPLETE       (1u << 18)
#define ST_BUS_RESET            (1u << 19)
#define ST_RX_TIMEOUT           (1u << 27)
#define ST_NAK_REC              (1u << 28)
#define ST_STALL_REC            (1u << 29)
#define ST_ACK_REC              (1u << 30)
#define ST_DATA_SEQ_ERROR       (1u << 31)

#define INT_HOST_CONN_DIS       (1u << 0)
#define INT_HOST_SOF            (1u << 2)
#define INT_TRANS_COMPLETE      (1u << 3)
#define INT_BUFF_STATUS         (1u << 4)
#define INT_ERROR_DATA_SEQ      (1u << 5)
#define INT_STALL               (1u << 10)
#define INT_VBUS_DETECT         (1u << 11)
#define INT_BUS_RESET           (1u << 12)
#define INT_DEV_CONN_DIS        (1u << 13)
#define INT_DEV_SUSPEND         (1u << 14)
#define INT_DEV_RESUME          (1u << 15)
#define INT_SETUP_REQ           (1u << 16)
#define INT_DEV_SOF             (1u << 17)
#define INT_ABORT_DONE          (1u << 18)
#define INT_EP_STALL_NAK        (1u << 19)
#define INT_EPX_STOPPED_ON_NAK  (1u << 23)

#define EP_CTRL_ENABLE          (1u << 31)
#define EP_CTRL_DOUBLE          (1u << 30)
#define EP_CTRL_INT_1BUF        (1u << 29)
#define EP_CTRL_TYPE(t)         ((uint32_t)(t) << 26)
#define EP_TYPE_BULK            2
#define EP_TYPE_INTERRUPT       3

#define BUF_FULL                (1u << 15)
#define BUF_LAST                (1u << 14)
#define BUF_PID1                (1u << 13)
#define BUF_STALL               (1u << 11)
#define BUF_AVAIL               (1u << 10)

#define DP_EP_CTRL(n, out)      (DPRAM + 0x08 + ((n) - 1) * 8 + (out) * 4)
#define DP_BUF_CTRL(n, out)     (DPRAM + 0x80 + (n) * 8 + (out) * 4)
#define DP_EPX_CTRL             (DPRAM + 0x100)
#define DP_EPX_BUF_CTRL         (DPRAM + 0x80)

#define RESET_USBCTRL           (1u << 28)

static char *rom_path;

static QTestState *start(const char *extra)
{
    QTestState *qts = qtest_initf("-M rp2350 -bios %s %s", rom_path,
                                  extra ? extra : "");

    rp2350_clocks_init(qts);
    rp2350_unreset(qts, RESET_USBCTRL);
    return qts;
}

static void irq_intercept(QTestState *qts)
{
    qtest_irq_intercept_out_named(qts, "/machine/soc/usbctrl", "sysbus-irq");
}

static void dpram_write(QTestState *qts, uint32_t off, const void *buf,
                        size_t len)
{
    qtest_memwrite(qts, DPRAM + off, buf, len);
}

/* USBCTRL starts in reset: its registers and DPRAM read 0 and ignore writes. */
/* [spec:nuos:req:emu.usb/test] */
static void test_held_in_reset(void)
{
    QTestState *qts = qtest_initf("-M rp2350 -bios %s", rom_path);

    g_assert_cmphex(qtest_readl(qts, MAIN_CTRL), ==, 0);
    qtest_writel(qts, DPRAM + 0x200, 0x12345678);
    g_assert_cmphex(qtest_readl(qts, DPRAM + 0x200), ==, 0);
    rp2350_unreset(qts, RESET_USBCTRL);
    g_assert_cmphex(qtest_readl(qts, MAIN_CTRL), ==, MAIN_CTRL_PHY_ISO);
    g_assert_cmphex(qtest_readl(qts, DPRAM + 0x200), ==, 0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.usb/test] */
static void test_reset_values(void)
{
    static const struct {
        uint32_t addr;
        uint32_t value;
    } regs[] = {
        { ADDR_ENDP(0), 0 }, { ADDR_ENDP(15), 0 },
        { MAIN_CTRL, 0x00000004 }, { SOF_RD, 0 },
        { SIE_CTRL, 0x00008000 }, { SIE_STATUS, 0 }, { INT_EP_CTRL, 0 },
        { BUFF_STATUS, 0 }, { BUFF_CPU_SHOULD_HANDLE, 0 }, { EP_ABORT, 0 },
        { EP_ABORT_DONE, 0 }, { EP_STALL_ARM, 0 },
        { NAK_POLL, 0x00100010 }, { EP_STATUS_STALL_NAK, 0 },
        { USB_MUXING, 0x00000001 }, { USB_PWR, 0 }, { USBPHY_DIRECT, 0 },
        { USBPHY_DIRECT_OVERRIDE, 0 }, { USBPHY_TRIM, 0x00001f1f },
        { LINESTATE_TUNING, 0x000000f8 }, { INTR, 0 }, { INTE, 0 },
        { INTF, 0 }, { INTS, 0 }, { SOF_TIMESTAMP_LAST, 0 },
        { SM_STATE, 0 }, { EP_TX_ERROR, 0 }, { EP_RX_ERROR, 0 },
        { DEV_SM_WATCHDOG, 0 },
    };
    QTestState *qts = start(NULL);
    int i;

    for (i = 0; i < ARRAY_SIZE(regs); i++) {
        g_assert_cmphex(qtest_readl(qts, regs[i].addr), ==, regs[i].value);
    }
    for (i = 0; i < 0x1000; i += 4) {
        g_assert_cmphex(qtest_readl(qts, DPRAM + i), ==, 0);
    }
    qtest_quit(qts);
}

/* Read-write fields keep only their bits; read-only registers ignore writes. */
/* [spec:nuos:req:emu.usb/test] */
static void test_access(void)
{
    static const struct {
        uint32_t addr;
        uint32_t mask;
    } rw[] = {
        { ADDR_ENDP(0), 0x000f007f }, { ADDR_ENDP(1), 0x060f007f },
        { ADDR_ENDP(15), 0x060f007f }, { INT_EP_CTRL, 0x0000fffe },
        { EP_ABORT, 0xffffffff }, { EP_STALL_ARM, 0x00000003 },
        { USB_PWR, 0x0000003f }, { USBPHY_TRIM, 0x00001f1f },
        { LINESTATE_TUNING, 0x00000fff }, { INTE, 0x00ffffff },
        { DEV_SM_WATCHDOG, 0x000fffff },
    };
    QTestState *qts = start(NULL);
    int i;

    for (i = 0; i < ARRAY_SIZE(rw); i++) {
        qtest_writel(qts, rw[i].addr, 0xffffffff);
        g_assert_cmphex(qtest_readl(qts, rw[i].addr), ==, rw[i].mask);
        qtest_writel(qts, rw[i].addr, 0);
        g_assert_cmphex(qtest_readl(qts, rw[i].addr), ==, 0);
    }
    /* Setting EP_ABORT bits reported the endpoints idle. */
    g_assert_cmphex(qtest_readl(qts, EP_ABORT_DONE), ==, 0xffffffff);
    qtest_writel(qts, EP_ABORT_DONE, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, EP_ABORT_DONE), ==, 0);
    /* NAK_POLL: the retry count and EPX_STOPPED_ON_NAK are not writable. */
    qtest_writel(qts, NAK_POLL, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, NAK_POLL), ==, 0x07ff03ff);
    /* SIE_CTRL's self-clearing strobes read back as 0. */
    qtest_writel(qts, SIE_CTRL, SIE_CTRL_STOP_TRANS | SIE_CTRL_RESUME |
                 SIE_CTRL_VBUS_EN);
    g_assert_cmphex(qtest_readl(qts, SIE_CTRL), ==, SIE_CTRL_VBUS_EN);

    qtest_writel(qts, SOF_RD, 0x7ff);
    qtest_writel(qts, INTR, 0xffffff);
    qtest_writel(qts, BUFF_CPU_SHOULD_HANDLE, 0xffffffff);
    qtest_writel(qts, SM_STATE, 0xfff);
    g_assert_cmphex(qtest_readl(qts, SOF_RD), ==, 0);
    g_assert_cmphex(qtest_readl(qts, INTR), ==, 0);
    g_assert_cmphex(qtest_readl(qts, BUFF_CPU_SHOULD_HANDLE), ==, 0);
    g_assert_cmphex(qtest_readl(qts, SM_STATE), ==, 0);

    /* Atomic aliases, and narrow writes replicated across the register. */
    qtest_writel(qts, INTE, 0x00f0);
    qtest_writel(qts, INTE + SET, 0x0003);
    g_assert_cmphex(qtest_readl(qts, INTE), ==, 0x00f3);
    qtest_writel(qts, INTE + CLR, 0x0030);
    g_assert_cmphex(qtest_readl(qts, INTE), ==, 0x00c3);
    qtest_writel(qts, INTE + XOR, 0x0101);
    g_assert_cmphex(qtest_readl(qts, INTE), ==, 0x01c2);
    qtest_writeb(qts, INTE + 1, 0x5a);
    g_assert_cmphex(qtest_readl(qts, INTE), ==, 0x5a5a5a);
    g_assert_cmphex(qtest_readb(qts, INTE + 2), ==, 0x5a);

    /* The DPRAM takes byte, halfword and word accesses, without aliases. */
    qtest_writel(qts, DPRAM + 0x400, 0x11223344);
    qtest_writeb(qts, DPRAM + 0x401, 0xaa);
    qtest_writew(qts, DPRAM + 0x402, 0xbbcc);
    g_assert_cmphex(qtest_readl(qts, DPRAM + 0x400), ==, 0xbbccaa44);
    g_assert_cmphex(qtest_readb(qts, DPRAM + 0x403), ==, 0xbb);
    g_assert_cmphex(qtest_readw(qts, DPRAM + 0x400), ==, 0xaa44);
    qtest_quit(qts);
}

/* INTS is INTR or INTF, masked by INTE, and drives USBCTRL_IRQ. */
/* [spec:nuos:req:emu.usb/test] */
static void test_irq(void)
{
    QTestState *qts = start(NULL);

    irq_intercept(qts);
    g_assert_false(qtest_get_irq(qts, 0));
    qtest_writel(qts, INTF, INT_TRANS_COMPLETE);
    g_assert_cmphex(qtest_readl(qts, INTS), ==, 0);
    g_assert_false(qtest_get_irq(qts, 0));
    qtest_writel(qts, INTE, INT_TRANS_COMPLETE | INT_VBUS_DETECT);
    g_assert_cmphex(qtest_readl(qts, INTS), ==, INT_TRANS_COMPLETE);
    g_assert_true(qtest_get_irq(qts, 0));
    qtest_writel(qts, INTF, 0);
    g_assert_false(qtest_get_irq(qts, 0));

    /* VBUS_DETECTED follows USB_PWR's override; VBUS_DETECT is its level. */
    qtest_writel(qts, USB_PWR, 0x0c);
    g_assert_cmphex(qtest_readl(qts, SIE_STATUS) & ST_VBUS_DETECTED, ==,
                    ST_VBUS_DETECTED);
    g_assert_cmphex(qtest_readl(qts, INTR), ==, INT_VBUS_DETECT);
    g_assert_true(qtest_get_irq(qts, 0));
    qtest_writel(qts, USB_PWR, 0x08);
    g_assert_false(qtest_get_irq(qts, 0));
    qtest_quit(qts);
}

/* Device mode through the token port */

typedef struct {
    QTestState *qts;
    char *dir;
    char *path;
    int fd;
} TokenHost;

static void tok_send(TokenHost *h, char op, uint8_t addr, uint8_t ep,
                     uint8_t pid, uint16_t len, const void *data)
{
    uint8_t hdr[8] = { op, addr, ep, pid, len, len >> 8, 0, 0 };
    size_t n = op == 'S' || op == 'O' ? len : 0;

    g_assert_cmpint(write(h->fd, hdr, sizeof(hdr)), ==, sizeof(hdr));
    if (n) {
        g_assert_cmpint(write(h->fd, data, n), ==, n);
    }
}

static void read_full(int fd, uint8_t *buf, size_t len)
{
    while (len) {
        ssize_t n = read(fd, buf, len);

        g_assert_cmpint(n, >, 0);
        buf += n;
        len -= n;
    }
}

/* Run one transaction; returns the handshake, and IN data and PID. */
static char tok_xact(TokenHost *h, char op, uint8_t addr, uint8_t ep,
                     uint8_t pid, uint16_t len, const void *data,
                     uint8_t *in, uint16_t *in_len, uint8_t *b1)
{
    uint8_t rep[4];
    uint16_t n;

    tok_send(h, op, addr, ep, pid, len, data);
    read_full(h->fd, rep, sizeof(rep));
    n = rep[2] | (rep[3] << 8);
    if (n) {
        g_assert_nonnull(in);
        read_full(h->fd, in, n);
    }
    if (in_len) {
        *in_len = n;
    }
    if (b1) {
        *b1 = rep[1];
    }
    return rep[0];
}

static TokenHost *token_start(void)
{
    TokenHost *h = g_new0(TokenHost, 1);
    struct sockaddr_un sa = { .sun_family = AF_UNIX };
    g_autofree char *args = NULL;
    GError *err = NULL;
    int i;

    h->dir = g_dir_make_tmp("usbctrl-XXXXXX", &err);
    g_assert_no_error(err);
    h->path = g_strdup_printf("%s/t", h->dir);
    g_assert_cmpuint(strlen(h->path), <, sizeof(sa.sun_path));
    args = g_strdup_printf("-chardev socket,id=tok,path=%s,server=on,"
                           "wait=off -global rp2350-usbctrl.token-chardev=tok",
                           h->path);
    h->qts = start(args);
    h->fd = socket(AF_UNIX, SOCK_STREAM, 0);
    g_assert_cmpint(h->fd, >=, 0);
    strcpy(sa.sun_path, h->path);
    g_assert_cmpint(connect(h->fd, (struct sockaddr *)&sa, sizeof(sa)), ==, 0);
    /* The device sees the host once QEMU has taken the connection. */
    qtest_writel(h->qts, MAIN_CTRL, MAIN_CTRL_EN);
    qtest_writel(h->qts, SIE_CTRL, SIE_CTRL_EP0_INT_1BUF |
                 SIE_CTRL_PULLUP_EN);
    for (i = 0; i < 1000; i++) {
        if (qtest_readl(h->qts, SIE_STATUS) & ST_CONNECTED) {
            break;
        }
        g_usleep(1000);
    }
    g_assert_cmphex(qtest_readl(h->qts, SIE_STATUS) & ST_CONNECTED, ==,
                    ST_CONNECTED);
    return h;
}

static void token_stop(TokenHost *h)
{
    close(h->fd);
    qtest_quit(h->qts);
    unlink(h->path);
    rmdir(h->dir);
    g_free(h->path);
    g_free(h->dir);
    g_free(h);
}

/*
 * A connection raises DEV_CONN_DIS until SIE_STATUS.CONNECTED is
 * written; a bus reset sets BUS_RESET; SETUP lands in DPRAM.
 */
/* [spec:nuos:req:emu.usb/test] */
static void test_device_setup(void)
{
    static const uint8_t setup[8] = { 0x80, 6, 0, 1, 0, 0, 0x12, 0 };
    TokenHost *h = token_start();
    QTestState *qts = h->qts;
    uint8_t flags;

    irq_intercept(qts);
    qtest_writel(qts, INTE, INT_DEV_CONN_DIS | INT_BUS_RESET |
                 INT_SETUP_REQ);
    g_assert_true(qtest_get_irq(qts, 0));
    g_assert_cmphex(qtest_readl(qts, INTR) & INT_DEV_CONN_DIS, ==,
                    INT_DEV_CONN_DIS);
    qtest_writel(qts, SIE_STATUS + CLR, ST_CONNECTED);
    g_assert_cmphex(qtest_readl(qts, INTR) & INT_DEV_CONN_DIS, ==, 0);
    g_assert_cmpuint(ST_LINE_STATE(qtest_readl(qts, SIE_STATUS)), ==, 1);
    g_assert_false(qtest_get_irq(qts, 0));

    g_assert_cmpint(tok_xact(h, 'P', 0, 0, 0, 0, NULL, NULL, NULL, &flags),
                    ==, 'A');
    g_assert_cmpuint(flags & 1, ==, 1);

    g_assert_cmpint(tok_xact(h, 'R', 0, 0, 0, 0, NULL, NULL, NULL, NULL),
                    ==, 'A');
    g_assert_cmphex(qtest_readl(qts, SIE_STATUS) & ST_BUS_RESET, ==,
                    ST_BUS_RESET);
    g_assert_true(qtest_get_irq(qts, 0));
    qtest_writel(qts, SIE_STATUS, ST_BUS_RESET);
    g_assert_false(qtest_get_irq(qts, 0));

    /* SETUP clears the EP0 STALL arming. */
    qtest_writel(qts, EP_STALL_ARM, 3);
    g_assert_cmpint(tok_xact(h, 'S', 0, 0, 0, 8, setup, NULL, NULL, NULL),
                    ==, 'A');
    g_assert_cmphex(qtest_readl(qts, DPRAM), ==, 0x01000680);
    g_assert_cmphex(qtest_readl(qts, DPRAM + 4), ==, 0x00120000);
    g_assert_cmphex(qtest_readl(qts, EP_STALL_ARM), ==, 0);
    g_assert_cmphex(qtest_readl(qts, SIE_STATUS) & ST_SETUP_REC, ==,
                    ST_SETUP_REC);
    g_assert_true(qtest_get_irq(qts, 0));
    qtest_writel(qts, SIE_STATUS + CLR, ST_SETUP_REC);
    g_assert_false(qtest_get_irq(qts, 0));

    /* SETUP data must be DATA0. */
    g_assert_cmpint(tok_xact(h, 'S', 0, 0, 1, 8, setup, NULL, NULL, NULL),
                    ==, 'A');
    g_assert_cmphex(qtest_readl(qts, SIE_STATUS) & ST_DATA_SEQ_ERROR, ==,
                    ST_DATA_SEQ_ERROR);

    /* Only the device's address is answered. */
    g_assert_cmpint(tok_xact(h, 'S', 3, 0, 0, 8, setup, NULL, NULL, NULL),
                    ==, 'T');
    qtest_writel(qts, ADDR_ENDP(0), 3);
    g_assert_cmpint(tok_xact(h, 'S', 0, 0, 0, 8, setup, NULL, NULL, NULL),
                    ==, 'T');
    g_assert_cmpint(tok_xact(h, 'S', 3, 0, 0, 8, setup, NULL, NULL, NULL),
                    ==, 'A');
    token_stop(h);
}

/*
 * EP0 IN and OUT NAK until software hands a buffer over, then transfer
 * it, write its status back and raise BUFF_STATUS.
 */
/* [spec:nuos:req:emu.usb/test] */
static void test_device_ep0(void)
{
    static const uint8_t desc[8] = { 0x12, 1, 0, 2, 0, 0, 0, 64 };
    TokenHost *h = token_start();
    QTestState *qts = h->qts;
    uint8_t data[64], pid;
    uint16_t len;

    qtest_writel(qts, SIE_CTRL + SET, SIE_CTRL_EP0_INT_NAK |
                 SIE_CTRL_EP0_INT_STALL);
    g_assert_cmpint(tok_xact(h, 'I', 0, 0, 0, 64, NULL, data, &len, NULL),
                    ==, 'N');
    g_assert_cmphex(qtest_readl(qts, EP_STATUS_STALL_NAK), ==, 1);
    g_assert_cmphex(qtest_readl(qts, INTR) & INT_EP_STALL_NAK, ==,
                    INT_EP_STALL_NAK);
    qtest_writel(qts, EP_STATUS_STALL_NAK, 1);
    g_assert_cmphex(qtest_readl(qts, INTR) & INT_EP_STALL_NAK, ==, 0);

    dpram_write(qts, 0x100, desc, sizeof(desc));
    /* Full but not yet available: still NAKed. */
    qtest_writel(qts, DP_BUF_CTRL(0, 0), BUF_FULL | BUF_PID1 | 8);
    g_assert_cmpint(tok_xact(h, 'I', 0, 0, 0, 64, NULL, data, &len, NULL),
                    ==, 'N');
    qtest_writel(qts, DP_BUF_CTRL(0, 0), BUF_FULL | BUF_PID1 | BUF_LAST |
                 BUF_AVAIL | 8);
    g_assert_cmpint(tok_xact(h, 'I', 0, 0, 0, 64, NULL, data, &len, &pid),
                    ==, 'A');
    g_assert_cmpuint(len, ==, 8);
    g_assert_cmpuint(pid, ==, 1);
    g_assert_cmpmem(data, len, desc, sizeof(desc));
    /* Status: length, PID and LAST; FULL and AVAILABLE cleared. */
    g_assert_cmphex(qtest_readl(qts, DP_BUF_CTRL(0, 0)), ==,
                    BUF_PID1 | BUF_LAST | 8);
    g_assert_cmphex(qtest_readl(qts, BUFF_STATUS), ==, 1);
    g_assert_cmphex(qtest_readl(qts, BUFF_CPU_SHOULD_HANDLE), ==, 0);
    g_assert_cmphex(qtest_readl(qts, SIE_STATUS) &
                    (ST_TRANS_COMPLETE | ST_ACK_REC), ==,
                    ST_TRANS_COMPLETE | ST_ACK_REC);
    qtest_writel(qts, BUFF_STATUS + CLR, 1);
    g_assert_cmphex(qtest_readl(qts, BUFF_STATUS), ==, 0);

    /* Status stage: a zero-length OUT into an available, empty buffer. */
    g_assert_cmpint(tok_xact(h, 'O', 0, 0, 1, 0, NULL, NULL, NULL, NULL),
                    ==, 'N');
    qtest_writel(qts, DP_BUF_CTRL(0, 1), BUF_PID1 | BUF_AVAIL);
    g_assert_cmpint(tok_xact(h, 'O', 0, 0, 1, 0, NULL, NULL, NULL, NULL),
                    ==, 'A');
    g_assert_cmphex(qtest_readl(qts, DP_BUF_CTRL(0, 1)), ==,
                    BUF_FULL | BUF_PID1);
    g_assert_cmphex(qtest_readl(qts, BUFF_STATUS), ==, 2);

    /* EP0's STALL needs EP_STALL_ARM as well. */
    qtest_writel(qts, DP_BUF_CTRL(0, 0), BUF_STALL);
    g_assert_cmpint(tok_xact(h, 'I', 0, 0, 0, 64, NULL, data, &len, NULL),
                    ==, 'N');
    qtest_writel(qts, EP_STALL_ARM, 1);
    qtest_writel(qts, EP_STATUS_STALL_NAK, 0xffffffff);
    g_assert_cmpint(tok_xact(h, 'I', 0, 0, 0, 64, NULL, data, &len, NULL),
                    ==, 'S');
    g_assert_cmphex(qtest_readl(qts, EP_STATUS_STALL_NAK), ==, 1);

    /* An OUT whose PID does not match the buffer's is a sequence error. */
    qtest_writel(qts, DP_BUF_CTRL(0, 1), BUF_AVAIL | 64);
    g_assert_cmpint(tok_xact(h, 'O', 0, 0, 1, 4, "abcd", NULL, NULL, NULL),
                    ==, 'A');
    g_assert_cmphex(qtest_readl(qts, SIE_STATUS) &
                    (ST_DATA_SEQ_ERROR | ST_RX_SHORT_PACKET), ==,
                    ST_DATA_SEQ_ERROR | ST_RX_SHORT_PACKET);
    g_assert_cmphex(qtest_readl(qts, DP_BUF_CTRL(0, 1)), ==, BUF_FULL | 4);
    g_assert_cmphex(qtest_readl(qts, DPRAM + 0x100), ==, 0x64636261);
    token_stop(h);
}

/*
 * A double-buffered bulk endpoint alternates buffers; a buffer that
 * completes while BUFF_STATUS is still set raises it again once cleared,
 * with BUFF_CPU_SHOULD_HANDLE naming it.
 */
/* [spec:nuos:req:emu.usb/test] */
static void test_device_double_buffer(void)
{
    TokenHost *h = token_start();
    QTestState *qts = h->qts;
    uint8_t pkt[64];
    int i;

    for (i = 0; i < 64; i++) {
        pkt[i] = i;
    }
    /* A disabled endpoint does not answer. */
    g_assert_cmpint(tok_xact(h, 'O', 0, 1, 0, 64, pkt, NULL, NULL, NULL),
                    ==, 'T');
    qtest_writel(qts, DP_EP_CTRL(1, 1), EP_CTRL_ENABLE | EP_CTRL_DOUBLE |
                 EP_CTRL_INT_1BUF | EP_CTRL_TYPE(EP_TYPE_BULK) | 0x180);
    qtest_writel(qts, DP_BUF_CTRL(1, 1),
                 ((BUF_AVAIL | BUF_PID1 | 64) << 16) | BUF_AVAIL | 64);
    g_assert_cmpint(tok_xact(h, 'O', 0, 1, 0, 64, pkt, NULL, NULL, NULL),
                    ==, 'A');
    g_assert_cmphex(qtest_readl(qts, BUFF_STATUS), ==, 1u << 3);
    g_assert_cmphex(qtest_readl(qts, BUFF_CPU_SHOULD_HANDLE), ==, 0);
    pkt[0] = 0xaa;
    g_assert_cmpint(tok_xact(h, 'O', 0, 1, 1, 64, pkt, NULL, NULL, NULL),
                    ==, 'A');
    g_assert_cmphex(qtest_readl(qts, DP_BUF_CTRL(1, 1)), ==,
                    ((BUF_FULL | BUF_PID1 | 64) << 16) | BUF_FULL | 64);
    g_assert_cmphex(qtest_readl(qts, DPRAM + 0x180), ==, 0x03020100);
    g_assert_cmphex(qtest_readl(qts, DPRAM + 0x1c0), ==, 0x030201aa);
    /* Both buffers full: NAK. */
    g_assert_cmpint(tok_xact(h, 'O', 0, 1, 0, 64, pkt, NULL, NULL, NULL),
                    ==, 'N');
    qtest_writel(qts, BUFF_STATUS, 1u << 3);
    g_assert_cmphex(qtest_readl(qts, BUFF_STATUS), ==, 1u << 3);
    g_assert_cmphex(qtest_readl(qts, BUFF_CPU_SHOULD_HANDLE), ==, 1u << 3);
    qtest_writel(qts, BUFF_STATUS, 1u << 3);
    g_assert_cmphex(qtest_readl(qts, BUFF_STATUS), ==, 0);

    /* EP_ABORT NAKs the endpoint and reports it idle. */
    qtest_writel(qts, DP_BUF_CTRL(1, 1), BUF_AVAIL | 64);
    qtest_writel(qts, EP_ABORT, 1u << 3);
    g_assert_cmphex(qtest_readl(qts, EP_ABORT_DONE), ==, 1u << 3);
    g_assert_cmphex(qtest_readl(qts, INTR) & INT_ABORT_DONE, ==,
                    INT_ABORT_DONE);
    g_assert_cmpint(tok_xact(h, 'O', 0, 1, 0, 64, pkt, NULL, NULL, NULL),
                    ==, 'N');
    qtest_writel(qts, EP_ABORT_DONE, 1u << 3);
    qtest_writel(qts, EP_ABORT, 0);
    g_assert_cmphex(qtest_readl(qts, INTR) & INT_ABORT_DONE, ==, 0);
    token_stop(h);
}

/*
 * SOFs set SOF_RD and DEV_SOF, which reading SOF_RD clears. With no bus
 * activity for 3 ms the device suspends; activity resumes it.
 */
/* [spec:nuos:req:emu.usb/test] */
static void test_device_sof_suspend(void)
{
    TokenHost *h = token_start();
    QTestState *qts = h->qts;

    g_assert_cmpint(tok_xact(h, 'F', 0, 0, 0, 0x123, NULL, NULL, NULL, NULL),
                    ==, 'A');
    g_assert_cmphex(qtest_readl(qts, INTR) & INT_DEV_SOF, ==, INT_DEV_SOF);
    g_assert_cmphex(qtest_readl(qts, SOF_RD), ==, 0x123);
    g_assert_cmphex(qtest_readl(qts, INTR) & INT_DEV_SOF, ==, 0);

    qtest_clock_step(qts, 2900000);
    g_assert_cmphex(qtest_readl(qts, SIE_STATUS) & ST_SUSPENDED, ==, 0);
    qtest_clock_step(qts, 200000);
    g_assert_cmphex(qtest_readl(qts, SIE_STATUS) & ST_SUSPENDED, ==,
                    ST_SUSPENDED);
    g_assert_cmphex(qtest_readl(qts, INTR) & INT_DEV_SUSPEND, ==,
                    INT_DEV_SUSPEND);
    qtest_writel(qts, SIE_STATUS, ST_SUSPENDED);
    g_assert_cmphex(qtest_readl(qts, INTR) & INT_DEV_SUSPEND, ==, 0);

    g_assert_cmpint(tok_xact(h, 'F', 0, 0, 0, 0x124, NULL, NULL, NULL, NULL),
                    ==, 'A');
    g_assert_cmphex(qtest_readl(qts, SIE_STATUS) &
                    (ST_SUSPENDED | ST_RESUME), ==, ST_RESUME);
    g_assert_cmphex(qtest_readl(qts, INTR) &
                    (INT_DEV_SUSPEND | INT_DEV_RESUME), ==,
                    INT_DEV_SUSPEND | INT_DEV_RESUME);
    qtest_writel(qts, SIE_STATUS, ST_RESUME | ST_SUSPENDED);
    g_assert_cmphex(qtest_readl(qts, INTR) &
                    (INT_DEV_SUSPEND | INT_DEV_RESUME), ==, 0);

    /* Pulling the pull-up disconnects the device. */
    qtest_writel(qts, SIE_CTRL + CLR, SIE_CTRL_PULLUP_EN);
    g_assert_cmphex(qtest_readl(qts, SIE_STATUS) & ST_CONNECTED, ==, 0);
    g_assert_cmphex(qtest_readl(qts, INTR) & INT_DEV_CONN_DIS, ==,
                    INT_DEV_CONN_DIS);
    g_assert_cmpint(tok_xact(h, 'F', 0, 0, 0, 1, NULL, NULL, NULL, NULL),
                    ==, 'A');
    g_assert_cmphex(qtest_readl(qts, SOF_RD), ==, 0x124);
    token_stop(h);
}

/*
 * The CDC-ACM bridge attaches with USB timing: 100 ms of debounce, a
 * 10 ms bus reset and 10 ms of recovery, then SOFs and the first
 * GET_DESCRIPTOR at address 0, which it repeats while EP0 NAKs.
 */
/* [spec:nuos:req:emu.usb/test] */
static void test_device_bridge_attach(void)
{
    QTestState *qts = start("-chardev null,id=cdc "
                            "-global rp2350-usbctrl.cdc-chardev=cdc");

    qtest_writel(qts, MAIN_CTRL, MAIN_CTRL_EN);
    qtest_writel(qts, SIE_CTRL, SIE_CTRL_EP0_INT_1BUF | SIE_CTRL_PULLUP_EN);
    g_assert_cmphex(qtest_readl(qts, SIE_STATUS) & ST_CONNECTED, ==,
                    ST_CONNECTED);
    qtest_clock_step(qts, 99 * 1000000);
    g_assert_cmphex(qtest_readl(qts, SIE_STATUS) & ST_BUS_RESET, ==, 0);
    qtest_clock_step(qts, 2 * 1000000);
    g_assert_cmphex(qtest_readl(qts, SIE_STATUS) & ST_BUS_RESET, ==,
                    ST_BUS_RESET);
    g_assert_cmphex(qtest_readl(qts, SIE_STATUS) & ST_SETUP_REC, ==, 0);
    qtest_writel(qts, SIE_STATUS, ST_BUS_RESET);
    qtest_clock_step(qts, 20 * 1000000);
    g_assert_cmphex(qtest_readl(qts, SIE_STATUS) & ST_SETUP_REC, ==,
                    ST_SETUP_REC);
    g_assert_cmphex(qtest_readl(qts, DPRAM), ==, 0x01000680);
    g_assert_cmphex(qtest_readl(qts, DPRAM + 4), ==, 0x00080000);
    g_assert_cmphex(qtest_readl(qts, INTR) & INT_DEV_SOF, ==, INT_DEV_SOF);
    g_assert_cmphex(qtest_readl(qts, SIE_STATUS) & ST_SUSPENDED, ==, 0);

    /* The host retries the data stage until EP0 IN has a buffer. */
    qtest_writel(qts, SIE_STATUS, ST_SETUP_REC);
    qtest_writel(qts, DPRAM + 0x100, 0x02000112);
    qtest_writel(qts, DPRAM + 0x104, 0x40000000);
    qtest_writel(qts, DP_BUF_CTRL(0, 0), BUF_FULL | BUF_PID1 | 8);
    qtest_writel(qts, DP_BUF_CTRL(0, 0), BUF_FULL | BUF_PID1 | BUF_AVAIL | 8);
    qtest_clock_step(qts, 100000);
    g_assert_cmphex(qtest_readl(qts, DP_BUF_CTRL(0, 0)), ==, BUF_PID1 | 8);
    g_assert_cmphex(qtest_readl(qts, BUFF_STATUS), ==, 1);
    qtest_quit(qts);
}

/* Host mode with a QEMU USB keyboard on the port */

static QTestState *host_start(void)
{
    QTestState *qts = start("-device usb-kbd,port=1");

    irq_intercept(qts);
    qtest_writel(qts, MAIN_CTRL, MAIN_CTRL_EN | MAIN_CTRL_HOST);
    qtest_writel(qts, SIE_CTRL, SIE_CTRL_SOF_EN | SIE_CTRL_KEEP_ALIVE_EN |
                 SIE_CTRL_PULLDOWN_EN);
    qtest_writel(qts, DP_EPX_CTRL, EP_CTRL_ENABLE | EP_CTRL_INT_1BUF | 0x180);
    return qts;
}

static void host_trans(QTestState *qts, uint32_t flags)
{
    uint32_t base = SIE_CTRL_SOF_EN | SIE_CTRL_KEEP_ALIVE_EN |
                    SIE_CTRL_PULLDOWN_EN;

    qtest_writel(qts, SIE_CTRL, base | flags);
    qtest_writel(qts, SIE_CTRL, base | flags | SIE_CTRL_START_TRANS);
    qtest_clock_step(qts, 200000);
}

static void host_setup(QTestState *qts, uint8_t addr, const uint8_t *setup)
{
    qtest_writel(qts, ADDR_ENDP(0), addr);
    qtest_memwrite(qts, DPRAM, setup, 8);
    qtest_writel(qts, SIE_STATUS, 0xffffffff);
    host_trans(qts, SIE_CTRL_SEND_SETUP);
    g_assert_cmphex(qtest_readl(qts, SIE_STATUS) &
                    (ST_TRANS_COMPLETE | ST_ACK_REC), ==,
                    ST_TRANS_COMPLETE | ST_ACK_REC);
    qtest_writel(qts, SIE_STATUS, 0xffffffff);
}

/* One IN buffer on EPX; returns the buffer control word after it. */
static uint32_t host_in(QTestState *qts, uint8_t addr, uint8_t ep,
                        uint32_t bc)
{
    qtest_writel(qts, ADDR_ENDP(0), addr | (ep << 16));
    qtest_writel(qts, DP_EPX_BUF_CTRL, bc);
    host_trans(qts, SIE_CTRL_RECEIVE_DATA);
    return qtest_readl(qts, DP_EPX_BUF_CTRL);
}

static uint32_t host_out(QTestState *qts, uint8_t addr, uint8_t ep,
                         uint32_t bc)
{
    qtest_writel(qts, ADDR_ENDP(0), addr | (ep << 16));
    qtest_writel(qts, DP_EPX_BUF_CTRL, bc);
    host_trans(qts, SIE_CTRL_SEND_DATA);
    return qtest_readl(qts, DP_EPX_BUF_CTRL);
}

/*
 * The attached device shows as a full-speed connection; frames count
 * from SOF_WR's number.
 */
/* [spec:nuos:req:emu.usb/test] */
static void test_host_connect(void)
{
    QTestState *qts = start("-device usb-kbd,port=1");
    uint32_t st;

    irq_intercept(qts);
    qtest_writel(qts, INTE, INT_HOST_CONN_DIS);
    g_assert_cmpuint(ST_SPEED(qtest_readl(qts, SIE_STATUS)), ==, 0);
    qtest_writel(qts, MAIN_CTRL, MAIN_CTRL_EN | MAIN_CTRL_HOST);
    st = qtest_readl(qts, SIE_STATUS);
    g_assert_cmpuint(ST_SPEED(st), ==, 2);
    g_assert_cmpuint(ST_LINE_STATE(st), ==, 1);
    g_assert_true(qtest_get_irq(qts, 0));
    qtest_writel(qts, SIE_STATUS, 3u << 8);
    g_assert_false(qtest_get_irq(qts, 0));

    /* Without the host's pull-downs no connection is seen. */
    qtest_writel(qts, SIE_CTRL + CLR, SIE_CTRL_PULLDOWN_EN);
    g_assert_cmpuint(ST_SPEED(qtest_readl(qts, SIE_STATUS)), ==, 0);
    g_assert_true(qtest_get_irq(qts, 0));
    qtest_writel(qts, SIE_STATUS, 3u << 8);
    qtest_writel(qts, SIE_CTRL + SET, SIE_CTRL_PULLDOWN_EN);
    qtest_writel(qts, SIE_STATUS, 3u << 8);

    /* SOFs every millisecond, numbered from SOF_WR. */
    qtest_writel(qts, SOF_WR, 0x7fe);
    qtest_writel(qts, SIE_CTRL + SET, SIE_CTRL_SOF_EN);
    qtest_clock_step(qts, 1000000);
    g_assert_cmphex(qtest_readl(qts, INTR) & INT_HOST_SOF, ==, INT_HOST_SOF);
    g_assert_cmphex(qtest_readl(qts, SOF_RD), ==, 0x7fe);
    g_assert_cmphex(qtest_readl(qts, INTR) & INT_HOST_SOF, ==, 0);
    qtest_clock_step(qts, 2000000);
    g_assert_cmphex(qtest_readl(qts, SOF_RD), ==, 0);
    qtest_quit(qts);
}

/*
 * A control transfer on EPX: SETUP, IN data packets in buffers software
 * arms one at a time, and the OUT status stage.
 */
/* [spec:nuos:req:emu.usb/test] */
static void test_host_control(void)
{
    static const uint8_t get_dev[8] = { 0x80, 6, 0, 1, 0, 0, 18, 0 };
    static const uint8_t set_addr[8] = { 0x00, 5, 7, 0, 0, 0, 0, 0 };
    QTestState *qts = host_start();
    uint32_t bc;

    host_setup(qts, 0, get_dev);
    bc = host_in(qts, 0, 0, BUF_AVAIL | BUF_PID1 | 8);
    g_assert_cmphex(bc, ==, BUF_FULL | BUF_PID1 | 8);
    g_assert_cmphex(qtest_readl(qts, BUFF_STATUS), ==, 1);
    g_assert_cmphex(qtest_readl(qts, DPRAM + 0x180), ==, 0x02000112);
    g_assert_cmphex(qtest_readl(qts, SIE_STATUS) & ST_TRANS_COMPLETE, ==, 0);
    /* The transfer waits for the next buffer. */
    qtest_writel(qts, BUFF_STATUS, 1);
    qtest_writel(qts, DP_EPX_BUF_CTRL, BUF_AVAIL | 8);
    qtest_clock_step(qts, 200000);
    g_assert_cmphex(qtest_readl(qts, DP_EPX_BUF_CTRL), ==, BUF_FULL | 8);
    g_assert_cmphex(qtest_readl(qts, DPRAM + 0x180), ==, 0x00010627);
    qtest_writel(qts, BUFF_STATUS, 1);
    /* A short last packet ends the transfer. */
    qtest_writel(qts, DP_EPX_BUF_CTRL, BUF_AVAIL | BUF_PID1 | BUF_LAST | 8);
    qtest_clock_step(qts, 200000);
    g_assert_cmphex(qtest_readl(qts, DP_EPX_BUF_CTRL), ==,
                    BUF_FULL | BUF_PID1 | BUF_LAST | 2);
    g_assert_cmphex(qtest_readl(qts, SIE_STATUS) &
                    (ST_TRANS_COMPLETE | ST_RX_SHORT_PACKET), ==,
                    ST_TRANS_COMPLETE | ST_RX_SHORT_PACKET);
    qtest_writel(qts, SIE_STATUS, 0xffffffff);
    bc = host_out(qts, 0, 0, BUF_AVAIL | BUF_FULL | BUF_PID1 | BUF_LAST);
    g_assert_cmphex(bc, ==, BUF_PID1 | BUF_LAST);
    g_assert_cmphex(qtest_readl(qts, SIE_STATUS) &
                    (ST_TRANS_COMPLETE | ST_ACK_REC), ==,
                    ST_TRANS_COMPLETE | ST_ACK_REC);

    /* SET_ADDRESS moves the device to address 7. */
    host_setup(qts, 0, set_addr);
    host_in(qts, 0, 0, BUF_AVAIL | BUF_PID1 | BUF_LAST);
    host_setup(qts, 7, get_dev);
    bc = host_in(qts, 7, 0, BUF_AVAIL | BUF_PID1 | 8);
    g_assert_cmphex(bc, ==, BUF_FULL | BUF_PID1 | 8);

    /* No device at an address: RX_TIMEOUT. */
    qtest_writel(qts, SIE_STATUS, 0xffffffff);
    qtest_writel(qts, ADDR_ENDP(0), 9);
    host_trans(qts, SIE_CTRL_SEND_SETUP);
    g_assert_cmphex(qtest_readl(qts, SIE_STATUS) &
                    (ST_RX_TIMEOUT | ST_TRANS_COMPLETE), ==, ST_RX_TIMEOUT);

    /* RESET_BUS returns the device to address 0. */
    qtest_writel(qts, SIE_CTRL + SET, SIE_CTRL_RESET_BUS);
    g_assert_cmphex(qtest_readl(qts, SIE_CTRL) & SIE_CTRL_RESET_BUS, ==, 0);
    host_setup(qts, 0, get_dev);
    qtest_quit(qts);
}

/*
 * The device's data toggle: an IN with a PID software does not expect is
 * a sequence error; a request it refuses stalls the data stage.
 */
/* [spec:nuos:req:emu.usb/test] */
static void test_host_errors(void)
{
    static const uint8_t get_dev[8] = { 0x80, 6, 0, 1, 0, 0, 18, 0 };
    static const uint8_t bad_req[8] = { 0x80, 6, 0, 0x7f, 0, 0, 18, 0 };
    QTestState *qts = host_start();
    uint32_t bc;

    host_setup(qts, 0, get_dev);
    bc = host_in(qts, 0, 0, BUF_AVAIL | 8);
    g_assert_cmphex(qtest_readl(qts, SIE_STATUS) & ST_DATA_SEQ_ERROR, ==,
                    ST_DATA_SEQ_ERROR);
    g_assert_cmphex(bc & BUF_FULL, ==, 0);

    qtest_writel(qts, INTE, INT_STALL);
    host_setup(qts, 0, bad_req);
    bc = host_in(qts, 0, 0, BUF_AVAIL | BUF_PID1 | 8);
    g_assert_cmphex(qtest_readl(qts, SIE_STATUS) & ST_STALL_REC, ==,
                    ST_STALL_REC);
    g_assert_cmphex(bc & (BUF_STALL | BUF_AVAIL | BUF_FULL), ==, BUF_STALL);
    g_assert_true(qtest_get_irq(qts, 0));
    qtest_quit(qts);
}

static void send_key(QTestState *qts, const char *key, bool down)
{
    QDict *rsp = qtest_qmp(qts, "{ 'execute': 'input-send-event', "
                           "'arguments': { 'events': [ { 'type': 'key', "
                           "'data': { 'down': %i, 'key': { 'type': 'qcode', "
                           "'data': %s } } } ] } }", down, key);

    g_assert(qdict_haskey(rsp, "return"));
    qobject_unref(rsp);
}

/*
 * A NAKed EPX transaction is retried after NAK_POLL's delay, counting
 * retries, or stops with STOP_EPX_ON_NAK. An interrupt endpoint slot is
 * polled after SOFs at its interval and reports through BUFF_STATUS.
 */
/* [spec:nuos:req:emu.usb/test] */
static void test_host_interrupt(void)
{
    static const uint8_t set_config[8] = { 0x00, 9, 1, 0, 0, 0, 0, 0 };
    QTestState *qts = host_start();
    uint32_t bc, np;

    host_setup(qts, 0, set_config);
    host_in(qts, 0, 0, BUF_AVAIL | BUF_PID1 | BUF_LAST);

    /* The keyboard NAKs its interrupt endpoint with no key pressed. */
    qtest_writel(qts, DP_EPX_CTRL, EP_CTRL_ENABLE | EP_CTRL_INT_1BUF |
                 EP_CTRL_TYPE(EP_TYPE_INTERRUPT) | 0x180);
    bc = host_in(qts, 0, 1, BUF_AVAIL | BUF_LAST | 8);
    g_assert_cmphex(bc, ==, BUF_AVAIL | BUF_LAST | 8);
    g_assert_cmphex(qtest_readl(qts, SIE_STATUS) & ST_NAK_REC, ==,
                    ST_NAK_REC);
    np = qtest_readl(qts, NAK_POLL);
    g_assert_cmpuint(((np >> 10) & 0x3f) | ((np >> 28) << 6), >, 2);
    qtest_writel(qts, NAK_POLL + SET, 1u << 26);
    qtest_clock_step(qts, 100000);
    g_assert_cmphex(qtest_readl(qts, NAK_POLL) & (1u << 27), ==, 1u << 27);
    g_assert_cmphex(qtest_readl(qts, INTR) & INT_EPX_STOPPED_ON_NAK, ==,
                    INT_EPX_STOPPED_ON_NAK);
    qtest_writel(qts, NAK_POLL + CLR, (1u << 27) | (1u << 26));
    g_assert_cmphex(qtest_readl(qts, NAK_POLL) & 0x0fff03ff, ==, 0x00100010);
    g_assert_cmphex(qtest_readl(qts, INTR) & INT_EPX_STOPPED_ON_NAK, ==, 0);

    /* Interrupt endpoint slot 1, polled every 4 frames. */
    qtest_writel(qts, ADDR_ENDP(1), 0 | (1 << 16));
    qtest_writel(qts, DP_EP_CTRL(1, 0), EP_CTRL_ENABLE | EP_CTRL_INT_1BUF |
                 EP_CTRL_TYPE(EP_TYPE_INTERRUPT) | (3 << 16) | 0x200);
    qtest_writel(qts, DP_BUF_CTRL(1, 0), BUF_AVAIL | BUF_LAST | 8);
    qtest_writel(qts, BUFF_STATUS, 0xffffffff);
    qtest_writel(qts, SIE_STATUS, 0xffffffff);
    qtest_writel(qts, INT_EP_CTRL, 1u << 1);
    qtest_clock_step(qts, 5000000);
    g_assert_cmphex(qtest_readl(qts, BUFF_STATUS), ==, 0);
    send_key(qts, "a", true);
    qtest_clock_step(qts, 5000000);
    g_assert_cmphex(qtest_readl(qts, BUFF_STATUS), ==, 1u << 2);
    bc = qtest_readl(qts, DP_BUF_CTRL(1, 0));
    g_assert_cmphex(bc, ==, BUF_FULL | BUF_LAST | 8);
    /* Boot keyboard report: modifiers, reserved, then usage 0x04 'a'. */
    g_assert_cmphex(qtest_readl(qts, DPRAM + 0x200), ==, 0x00040000);
    /* Interrupt endpoint polls do not raise TRANS_COMPLETE. */
    g_assert_cmphex(qtest_readl(qts, SIE_STATUS) & ST_TRANS_COMPLETE, ==, 0);

    /* The next report needs the next PID. */
    qtest_writel(qts, BUFF_STATUS, 1u << 2);
    send_key(qts, "a", false);
    qtest_writel(qts, DP_BUF_CTRL(1, 0), BUF_AVAIL | BUF_PID1 | BUF_LAST | 8);
    qtest_clock_step(qts, 5000000);
    g_assert_cmphex(qtest_readl(qts, BUFF_STATUS), ==, 1u << 2);
    g_assert_cmphex(qtest_readl(qts, DPRAM + 0x200), ==, 0);
    g_assert_cmphex(qtest_readl(qts, SIE_STATUS) & ST_DATA_SEQ_ERROR, ==, 0);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    static const uint32_t blank[2];
    GError *err = NULL;
    int fd, ret;

    g_test_init(&argc, &argv, NULL);

    fd = g_file_open_tmp("rp2350-usbctrl-test-XXXXXX.bin", &rom_path, &err);
    g_assert_no_error(err);
    g_assert_cmpint(write(fd, blank, sizeof(blank)), ==, sizeof(blank));
    close(fd);

    qtest_add_func("/rp2350/usbctrl/held-in-reset", test_held_in_reset);
    qtest_add_func("/rp2350/usbctrl/reset-values", test_reset_values);
    qtest_add_func("/rp2350/usbctrl/access", test_access);
    qtest_add_func("/rp2350/usbctrl/irq", test_irq);
    qtest_add_func("/rp2350/usbctrl/device/setup", test_device_setup);
    qtest_add_func("/rp2350/usbctrl/device/ep0", test_device_ep0);
    qtest_add_func("/rp2350/usbctrl/device/double-buffer",
                   test_device_double_buffer);
    qtest_add_func("/rp2350/usbctrl/device/sof-suspend",
                   test_device_sof_suspend);
    qtest_add_func("/rp2350/usbctrl/device/bridge-attach",
                   test_device_bridge_attach);
    qtest_add_func("/rp2350/usbctrl/host/connect", test_host_connect);
    qtest_add_func("/rp2350/usbctrl/host/control", test_host_control);
    qtest_add_func("/rp2350/usbctrl/host/errors", test_host_errors);
    qtest_add_func("/rp2350/usbctrl/host/interrupt", test_host_interrupt);

    ret = g_test_run();
    unlink(rom_path);
    g_free(rom_path);
    return ret;
}
