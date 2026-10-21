/*
 * QTest: the ESP32-C6 co-processor running esp-hosted-mcu on the ESP32-P4
 * machine's SD slot 1
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Driven through the SD/MMC host as NuOS's wifid drives it: CMD5, CMD3 and
 * CMD7, the CCCR through CMD52, function 1's registers and FIFO through
 * CMD52 and CMD53, esp-hosted's packets and serial RPCs, and the
 * station's Ethernet frames, which a socket backend carries.
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "libqtest.h"
#include "esp32p4-sd-pins.h"

#define SD                  0x50083000
#define SD_CTRL             (SD + 0x000)
#define SD_CLKDIV           (SD + 0x008)
#define SD_CLKSRC           (SD + 0x00c)
#define SD_CLKENA           (SD + 0x010)
#define SD_CTYPE            (SD + 0x018)
#define SD_BLKSIZ           (SD + 0x01c)
#define SD_BYTCNT           (SD + 0x020)
#define SD_CMDARG           (SD + 0x028)
#define SD_CMD              (SD + 0x02c)
#define SD_RESP(n)          (SD + 0x030 + 4 * (n))
#define SD_RINTSTS          (SD + 0x044)
#define SD_FIFOTH           (SD + 0x04c)
#define SD_BUFFIFO          (SD + 0x200)

#define CTRL_FIFO_RESET     (1u << 1)

#define CMD_RESP            (1u << 6)
#define CMD_CRC             (1u << 8)
#define CMD_DATA            (1u << 9)
#define CMD_WRITE           (1u << 10)
#define CMD_WAIT            (1u << 13)
#define CMD_INIT            (1u << 15)
#define CMD_CARD(n)         ((n) << 16)
#define CMD_UPDATE_CLK      (1u << 21)
#define CMD_HOLD            (1u << 29)
#define CMD_START           (1u << 31)
#define R1                  (CMD_RESP | CMD_CRC)
#define R4                  CMD_RESP

#define INT_CMDD            (1u << 2)
#define INT_DTO             (1u << 3)
#define INT_DCRC            (1u << 7)
#define INT_RTO             (1u << 8)
#define INT_DRTO            (1u << 9)
#define INT_EBE             (1u << 15)
#define INT_SDIO(slot)      (1u << (16 + (slot)))
#define INT_ERRORS          (INT_DCRC | INT_RTO | INT_DRTO | INT_EBE)

/* SDMMC's clock and reset, and slot 0's pads' supply */
#define HP_CLKRST           0x500e6000
#define SOC_CLK_CTRL1       (HP_CLKRST + 0x18)
#define SDMMC_SYS_CLK_EN    (1u << 14)
#define PERI_CLK_CTRL01     (HP_CLKRST + 0x34)
#define PERI_CLK_CTRL02     (HP_CLKRST + 0x38)
#define PMU_EXT_LDO_VO4     0x501151d8
#define VO4_ON_3V3          ((1u << 7) | (1u << 8) | (1u << 14))

/* GPIO54, the C6's CHIP_PU, which the board pulls up */
#define GPIO_OUT1_W1TS      (SDP_GPIO + 0x014)
#define GPIO_OUT1_W1TC      (SDP_GPIO + 0x018)
#define CHIP_PU_PAD         54
#define CHIP_PU_BIT         (1u << (CHIP_PU_PAD - 32))

#define SLOT                1
#define CCLK_NS             25

/* The SDIO card */
#define IO_RW               (1u << 31)
#define IO_RAW              (1u << 27)
#define IO_OP_INC           (1u << 26)
#define R5_ERRORS           0xcb00
#define CCCR_REV            0x00
#define CCCR_IOE            0x02
#define CCCR_IOR            0x03
#define CCCR_IEN            0x04
#define CCCR_INT            0x05
#define CCCR_CIS_PTR        0x09
#define FBR1_BLKSIZE        0x110

/* esp-hosted's SDIO slave */
#define FN1_TOKEN_RDATA     0x44
#define FN1_INT_RAW         0x50
#define FN1_RX_TOTAL        0x60
#define FN1_HOST_INT        0x8c
#define FN1_INT_CLR         0xd4
#define FN1_INT_ENA         0xdc
#define FN1_FIFO_END        0x1f800
#define INT_NEW_PACKET      (1u << 23)
#define RX_TOTAL_MASK       0xfffff

#define HDR_LEN             12
#define PACKET              1536
#define IF_STA              1
#define IF_SERIAL           3
#define IF_PRIV             5
#define PRIV_KIND_INIT      0x33
#define PRIV_EVENT_INIT     0x22
#define TAG_CHIP_ID         0x12
#define TAG_RX_BUFS         0x14
#define TAG_FW_VERSION      0x17
#define RX_BUFS             20

#define RPC_TYPE_REQ        1
#define RPC_TYPE_RESP       2
#define RPC_TYPE_EVENT      3
#define RPC_GET_MAC         257
#define RPC_WIFI_START      280
#define RPC_WIFI_CONNECT    282
#define RPC_WIFI_DISCONNECT 283
#define RPC_SET_CONFIG      284
#define RPC_FW_VERSION      350
#define RPC_RESP_OFFSET     256
#define RPC_EVENT_ESP_INIT  769
#define RPC_EVENT_CONNECTED 775
#define RPC_EVENT_DISCONNECTED 776
#define ESP_ERR_NOT_SUPPORTED 0x106
#define REASON_ASSOC_LEAVE  8
#define REASON_4WAY_TIMEOUT 15
#define REASON_NO_AP_FOUND  201

/* The access point the model offers by default */
#define AP_SSID             "nuos-emulator"
#define AP_PASSWORD         "nuos-emulator"

static void clocks_on(QTestState *qts)
{
    uint32_t edges = (3u << 17) | (3u << 9) | (1u << 13) | (7u << 27);

    qtest_writel(qts, PMU_EXT_LDO_VO4,
                 qtest_readl(qts, PMU_EXT_LDO_VO4) | VO4_ON_3V3);
    sd_pins(qts, SLOT);
    qtest_writel(qts, SOC_CLK_CTRL1,
                 qtest_readl(qts, SOC_CLK_CTRL1) | SDMMC_SYS_CLK_EN);
    qtest_writel(qts, PERI_CLK_CTRL01, 0x401 | (1u << 24));
    qtest_writel(qts, PERI_CLK_CTRL02, edges);
    qtest_writel(qts, PERI_CLK_CTRL02, edges | (1u << 8));
    qtest_writel(qts, SD_FIFOTH, (7u << 16) | 8);
}

static void card_clock(QTestState *qts, unsigned div)
{
    qtest_writel(qts, SD_CLKDIV, div << (8 * SLOT));
    qtest_writel(qts, SD_CLKSRC, SLOT << (2 * SLOT));
    qtest_writel(qts, SD_CLKENA, 1u << SLOT);
    qtest_writel(qts, SD_CMD, CMD_START | CMD_UPDATE_CLK | CMD_WAIT |
                 CMD_HOLD | CMD_CARD(SLOT));
    qtest_clock_step(qts, 2 * CCLK_NS);
    g_assert_cmphex(qtest_readl(qts, SD_CMD) & CMD_START, ==, 0);
}

/* Run until a RINTSTS bit in @mask is set: the bits, cleared. */
static uint32_t wait_int(QTestState *qts, uint32_t mask, int64_t max_ns)
{
    for (int64_t t = 0; t <= max_ns; t += 1000) {
        uint32_t st = qtest_readl(qts, SD_RINTSTS);

        if (st & mask) {
            qtest_writel(qts, SD_RINTSTS, st);
            return st;
        }
        qtest_clock_step(qts, 1000);
    }
    g_assert_not_reached();
}

static uint32_t cmd(QTestState *qts, uint32_t flags, unsigned index,
                    uint32_t arg)
{
    qtest_writel(qts, SD_CMDARG, arg);
    qtest_writel(qts, SD_CMD, CMD_START | CMD_HOLD | CMD_CARD(SLOT) |
                 flags | index);
    return wait_int(qts, INT_CMDD, 100 * 1000 * 1000);
}

/* CHIP_PU driven low, holding the C6 in reset, or released to the pull-up. */
static void chip_pu(QTestState *qts, bool low)
{
    qtest_writel(qts, SDP_IO_MUX(CHIP_PU_PAD), SDP_MCU_SEL(1) | SDP_FUN_IE |
                 SDP_FUN_DRV(2));
    qtest_writel(qts, SDP_FUNC_OUT(CHIP_PU_PAD), SDP_SIG_GPIO_OUT);
    qtest_writel(qts, GPIO_OUT1_W1TC, CHIP_PU_BIT);
    qtest_writel(qts, low ? SDP_ENABLE1_W1TS : SDP_ENABLE1_W1TC, CHIP_PU_BIT);
    qtest_writel(qts, GPIO_OUT1_W1TS, low ? 0 : CHIP_PU_BIT);
}

/* CMD5, CMD3 and CMD7: the card selected, its RCA. */
static uint32_t card_init(QTestState *qts)
{
    uint32_t st, ocr, rca;

    card_clock(qts, 50);
    st = cmd(qts, CMD_INIT | R4, 5, 0);
    g_assert_cmphex(st & INT_RTO, ==, 0);
    ocr = qtest_readl(qts, SD_RESP(0));
    /* Ready, one I/O function, no memory, 2.7 to 3.6 V */
    g_assert_cmphex(ocr, ==, 0x90ff8000);
    st = cmd(qts, R4, 5, 0x00200000);
    g_assert_cmphex(st & INT_RTO, ==, 0);
    st = cmd(qts, R1, 3, 0);
    g_assert_cmphex(st & (INT_RTO | INT_ERRORS), ==, 0);
    rca = qtest_readl(qts, SD_RESP(0)) >> 16;
    g_assert_cmphex(rca, !=, 0);
    st = cmd(qts, R1 | CMD_WAIT, 7, rca << 16);
    g_assert_cmphex(st & INT_RTO, ==, 0);
    card_clock(qts, 2);
    return rca;
}

/* CMD52: the byte at @addr of function @fn, after writing @v if @write. */
static uint8_t io_rw(QTestState *qts, bool write, unsigned fn, uint32_t addr,
                     uint8_t v)
{
    uint32_t st, r5;

    st = cmd(qts, R1, 52, (write ? IO_RW | IO_RAW : 0) | (fn << 28) |
             (addr << 9) | v);
    g_assert_cmphex(st & (INT_RTO | INT_ERRORS), ==, 0);
    r5 = qtest_readl(qts, SD_RESP(0));
    g_assert_cmphex(r5 & R5_ERRORS, ==, 0);
    return r5 & 0xff;
}

static uint32_t reg_read(QTestState *qts, uint32_t addr)
{
    uint32_t v = 0;

    for (int i = 3; i >= 0; i--) {
        v = (v << 8) | io_rw(qts, false, 1, addr + i, 0);
    }
    return v;
}

/* CMD53 in byte mode, incrementing, with its data through the FIFO. */
static void io_data(QTestState *qts, bool write, uint32_t addr, uint8_t *buf,
                    size_t n)
{
    uint32_t st, word;

    g_assert(n > 0 && n <= 512);
    qtest_writel(qts, SD_CTRL, CTRL_FIFO_RESET);
    qtest_writel(qts, SD_BLKSIZ, n);
    qtest_writel(qts, SD_BYTCNT, n);
    qtest_writel(qts, SD_CMDARG, (write ? IO_RW : 0) | (1u << 28) |
                 IO_OP_INC | (addr << 9) | (n & 0x1ff));
    qtest_writel(qts, SD_CMD, CMD_START | CMD_HOLD | CMD_CARD(SLOT) | R1 |
                 CMD_WAIT | CMD_DATA | (write ? CMD_WRITE : 0) | 53);
    if (write) {
        for (size_t i = 0; i < n; i += 4) {
            word = 0;
            memcpy(&word, &buf[i], MIN(4, n - i));
            qtest_writel(qts, SD_BUFFIFO, word);
        }
    }
    st = wait_int(qts, INT_DTO, 100 * 1000 * 1000);
    g_assert_cmphex(st & INT_ERRORS, ==, 0);
    g_assert_cmphex(qtest_readl(qts, SD_RESP(0)) & R5_ERRORS, ==, 0);
    if (!write) {
        for (size_t i = 0; i < n; i += 4) {
            word = qtest_readl(qts, SD_BUFFIFO);
            memcpy(&buf[i], &word, MIN(4, n - i));
        }
    }
}

/* ---- esp-hosted's packets ---- */

typedef struct {
    QTestState *qts;
    uint32_t taken;
    uint8_t buf[16384];
    size_t len;
    uint16_t seq;
} Link;

typedef struct {
    uint8_t iface;
    uint8_t flags;
    uint8_t kind;
    uint16_t len;
    uint8_t data[PACKET];
} Packet;

static uint16_t checksum(const uint8_t *p, size_t n)
{
    uint16_t sum = 0;

    for (size_t i = 0; i < n; i++) {
        sum += i == 6 || i == 7 ? 0 : p[i];
    }
    return sum;
}

/* Take what the C6 has queued since the last read into the link's buffer. */
static size_t link_fill(Link *l)
{
    size_t avail = (reg_read(l->qts, FN1_RX_TOTAL) - l->taken) & RX_TOTAL_MASK;

    g_assert_cmpuint(l->len + avail, <=, sizeof(l->buf));
    for (size_t at = 0; at < avail;) {
        size_t n = MIN(avail - at, 512);

        io_data(l->qts, false, FN1_FIFO_END - n, &l->buf[l->len], n);
        l->len += n;
        at += n;
    }
    l->taken = (l->taken + avail) & RX_TOTAL_MASK;
    return avail;
}

/* Virtual time, read by the shortest step the clock takes. */
static int64_t now(QTestState *qts)
{
    return qtest_clock_step(qts, 1);
}

/*
 * The next packet the C6 sends, waiting until @max_ns of virtual time
 * has passed, the transfers that look for it included.
 */
static bool link_next(Link *l, Packet *p, int64_t max_ns)
{
    int64_t deadline = now(l->qts) + max_ns;

    for (;;) {
        if (l->len >= HDR_LEN && l->len >= HDR_LEN + lduw_le_p(&l->buf[2])) {
            size_t n = HDR_LEN + lduw_le_p(&l->buf[2]);

            g_assert_cmpuint(lduw_le_p(&l->buf[4]), ==, HDR_LEN);
            g_assert_cmphex(lduw_le_p(&l->buf[6]), ==, checksum(l->buf, n));
            p->iface = l->buf[0];
            p->flags = l->buf[1];
            p->kind = l->buf[11];
            p->len = n - HDR_LEN;
            memcpy(p->data, &l->buf[HDR_LEN], p->len);
            memmove(l->buf, &l->buf[n], l->len - n);
            l->len -= n;
            return true;
        }
        if (link_fill(l)) {
            continue;
        }
        if (now(l->qts) >= deadline) {
            return false;
        }
        qtest_clock_step(l->qts, 100 * 1000);
    }
}

static void link_send(Link *l, uint8_t iface, const uint8_t *payload,
                      size_t len)
{
    uint8_t pkt[512];

    g_assert_cmpuint(HDR_LEN + len, <=, sizeof(pkt));
    memset(pkt, 0, HDR_LEN);
    pkt[0] = iface;
    stw_le_p(&pkt[2], len);
    stw_le_p(&pkt[4], HDR_LEN);
    stw_le_p(&pkt[8], l->seq++);
    memcpy(&pkt[HDR_LEN], payload, len);
    stw_le_p(&pkt[6], checksum(pkt, HDR_LEN + len));
    io_data(l->qts, true, FN1_FIFO_END - (HDR_LEN + len), pkt, HDR_LEN + len);
}

/* ---- Protobuf ---- */

typedef struct {
    uint8_t buf[512];
    size_t len;
} Pb;

static void pb_varint(Pb *w, uint64_t v)
{
    do {
        g_assert_cmpuint(w->len, <, sizeof(w->buf));
        w->buf[w->len++] = (v & 0x7f) | (v > 0x7f ? 0x80 : 0);
        v >>= 7;
    } while (v);
}

static void pb_uint(Pb *w, uint32_t field, uint64_t v)
{
    pb_varint(w, (uint64_t)field << 3);
    pb_varint(w, v);
}

static void pb_bytes(Pb *w, uint32_t field, const void *p, size_t n)
{
    pb_varint(w, ((uint64_t)field << 3) | 2);
    pb_varint(w, n);
    g_assert_cmpuint(w->len + n, <=, sizeof(w->buf));
    memcpy(&w->buf[w->len], p, n);
    w->len += n;
}

static uint64_t pb_read_varint(const uint8_t *p, size_t n, size_t *at)
{
    uint64_t v = 0;

    for (unsigned shift = 0; shift < 64; shift += 7) {
        g_assert_cmpuint(*at, <, n);
        v |= (uint64_t)(p[*at] & 0x7f) << shift;
        if (!(p[(*at)++] & 0x80)) {
            return v;
        }
    }
    g_assert_not_reached();
}

/* Field @field of message @p: a varint's value, or a bytes field's bytes. */
static bool pb_field(const uint8_t *p, size_t n, uint32_t field, uint64_t *v,
                     const uint8_t **bytes, size_t *len)
{
    size_t at = 0;

    while (at < n) {
        uint64_t key = pb_read_varint(p, n, &at);
        uint64_t value = 0;
        size_t start = at;

        switch (key & 7) {
        case 0:
            value = pb_read_varint(p, n, &at);
            break;
        case 2:
            value = pb_read_varint(p, n, &at);
            start = at;
            g_assert_cmpuint(at + value, <=, n);
            at += value;
            break;
        default:
            g_assert_not_reached();
        }
        if ((key >> 3) == field) {
            if (v) {
                *v = value;
            }
            if (bytes) {
                g_assert_cmpuint(key & 7, ==, 2);
                *bytes = &p[start];
                *len = value;
            }
            return true;
        }
    }
    return false;
}

static uint64_t pb_get(const uint8_t *p, size_t n, uint32_t field)
{
    uint64_t v;

    g_assert_true(pb_field(p, n, field, &v, NULL, NULL));
    return v;
}

/* ---- RPCs ---- */

typedef struct {
    uint64_t type;
    uint64_t id;
    uint64_t uid;
    const uint8_t *body;
    size_t body_len;
    Packet packet;
} Rpc;

/* The next serial packet's RPC message, or false if none comes in @max_ns. */
static bool rpc_next(Link *l, Rpc *r, int64_t max_ns)
{
    Packet *p = &r->packet;
    const uint8_t *pb;
    size_t len;

    if (!link_next(l, p, max_ns)) {
        return false;
    }
    g_assert_cmpuint(p->iface, ==, IF_SERIAL);
    g_assert_cmpuint(p->flags, ==, 0);
    g_assert_cmpuint(p->len, >=, 12);
    g_assert_cmpuint(p->data[0], ==, 1);
    g_assert_cmpuint(lduw_le_p(&p->data[1]), ==, 6);
    g_assert_cmpuint(p->data[9], ==, 2);
    len = lduw_le_p(&p->data[10]);
    g_assert_cmpuint(12 + len, ==, p->len);
    pb = &p->data[12];
    r->type = pb_get(pb, len, 1);
    g_assert_true(memcmp(&p->data[3], r->type == RPC_TYPE_EVENT ?
                         "RPCEvt" : "RPCRsp", 6) == 0);
    r->id = pb_get(pb, len, 2);
    r->uid = r->type == RPC_TYPE_RESP ? pb_get(pb, len, 3) : 0;
    r->body = NULL;
    r->body_len = 0;
    pb_field(pb, len, r->id, NULL, &r->body, &r->body_len);
    return true;
}

/* Send request @id with @body and return its response's status. */
static uint64_t rpc_call(Link *l, uint32_t id, const Pb *body, Rpc *r)
{
    static uint32_t uid;
    uint8_t msg[512];
    Pb env = { };

    pb_uint(&env, 1, RPC_TYPE_REQ);
    pb_uint(&env, 2, id);
    pb_uint(&env, 3, ++uid);
    pb_bytes(&env, id, body ? body->buf : NULL, body ? body->len : 0);
    msg[0] = 1;
    stw_le_p(&msg[1], 6);
    memcpy(&msg[3], "RPCRsp", 6);
    msg[9] = 2;
    stw_le_p(&msg[10], env.len);
    memcpy(&msg[12], env.buf, env.len);
    link_send(l, IF_SERIAL, msg, 12 + env.len);
    g_assert_true(rpc_next(l, r, 10 * 1000 * 1000));
    g_assert_cmpuint(r->type, ==, RPC_TYPE_RESP);
    g_assert_cmpuint(r->id, ==, id + RPC_RESP_OFFSET);
    g_assert_cmpuint(r->uid, ==, uid);
    return id == RPC_GET_MAC ? pb_get(r->body, r->body_len, 2) :
                               pb_get(r->body, r->body_len, 1);
}

/* ---- The machine ---- */

static QTestState *start(int *sock)
{
    int fd[2];

    g_assert_cmpint(socketpair(PF_UNIX, SOCK_STREAM, 0, fd), ==, 0);
    *sock = fd[0];
    return qtest_initf("-M esp32p4 -nic socket,fd=%d", fd[1]);
}

/* The card up, its interrupt enabled, and the data path open. */
static void link_open(QTestState *qts, Link *l)
{
    Packet p;
    Rpc r;

    memset(l, 0, sizeof(*l));
    l->qts = qts;
    clocks_on(qts);
    card_init(qts);
    io_rw(qts, true, 0, CCCR_IOE, 2);
    g_assert_cmphex(io_rw(qts, false, 0, CCCR_IOR, 0), ==, 2);
    io_rw(qts, true, 0, CCCR_IEN, 3);
    io_rw(qts, true, 1, FN1_INT_ENA + 2, INT_NEW_PACKET >> 16);
    io_rw(qts, true, 1, FN1_HOST_INT, 1);
    g_assert_true(link_next(l, &p, 0));
    g_assert_cmpuint(p.iface, ==, IF_PRIV);
    g_assert_true(rpc_next(l, &r, 0));
    g_assert_cmpuint(r.type, ==, RPC_TYPE_EVENT);
    g_assert_cmpuint(r.id, ==, RPC_EVENT_ESP_INIT);
}

/* The station's configuration: an SSID and password, WPA2 or better. */
static void sta_config(Link *l, const char *ssid, const char *password)
{
    Pb threshold = { }, sta = { }, cfg = { }, body = { };
    Rpc r;

    pb_uint(&threshold, 2, 3);
    pb_bytes(&sta, 1, ssid, strlen(ssid));
    pb_bytes(&sta, 2, password, strlen(password));
    pb_bytes(&sta, 9, threshold.buf, threshold.len);
    pb_bytes(&cfg, 2, sta.buf, sta.len);
    pb_uint(&body, 1, 0);
    pb_bytes(&body, 2, cfg.buf, cfg.len);
    g_assert_cmpuint(rpc_call(l, RPC_SET_CONFIG, &body, &r), ==, 0);
}

/* Connect, and the event 20 ms of virtual time later: its reason, or 0. */
static uint64_t sta_connect(Link *l)
{
    const uint8_t *data;
    size_t len;
    int64_t sent = now(l->qts), took;
    Rpc r;

    g_assert_cmpuint(rpc_call(l, RPC_WIFI_CONNECT, NULL, &r), ==, 0);
    g_assert_true(rpc_next(l, &r, 30 * 1000 * 1000));
    took = now(l->qts) - sent;
    g_assert_cmpint(took, >=, 20 * 1000 * 1000);
    g_assert_cmpint(took, <, 22 * 1000 * 1000);
    g_assert_cmpuint(r.type, ==, RPC_TYPE_EVENT);
    g_assert_true(pb_field(r.body, r.body_len, 2, NULL, &data, &len));
    if (r.id == RPC_EVENT_CONNECTED) {
        const uint8_t *ssid;
        size_t ssid_len;

        g_assert_true(pb_field(data, len, 1, NULL, &ssid, &ssid_len));
        g_assert_cmpmem(ssid, ssid_len, AP_SSID, strlen(AP_SSID));
        g_assert_cmpuint(pb_get(data, len, 4), ==, 6);
        return 0;
    }
    g_assert_cmpuint(r.id, ==, RPC_EVENT_DISCONNECTED);
    return pb_get(data, len, 4);
}

/* ---- Frames on the socket backend ---- */

static void sock_send(int fd, const uint8_t *f, size_t len)
{
    uint32_t be = cpu_to_be32(len);

    g_assert_cmpint(send(fd, &be, 4, 0), ==, 4);
    g_assert_cmpint(send(fd, f, len, 0), ==, len);
}

static size_t sock_recv(QTestState *qts, int fd, uint8_t *f, size_t max)
{
    uint32_t be;
    size_t len;

    for (int i = 0; i < 100; i++) {
        if (recv(fd, &be, 4, MSG_DONTWAIT) == 4) {
            len = be32_to_cpu(be);
            g_assert_cmpuint(len, <=, max);
            g_assert_cmpint(recv(fd, f, len, MSG_WAITALL), ==, len);
            return len;
        }
        qtest_clock_step(qts, 10 * 1000);
    }
    return 0;
}

static void make_frame(uint8_t *f, size_t len, uint8_t tag)
{
    static const uint8_t da[6] = { 0x02, 0x11, 0x22, 0x33, 0x44, 0x55 };
    static const uint8_t sa[6] = { 0x02, 0, 0, 0, 0, 0x01 };

    memcpy(f, da, 6);
    memcpy(f + 6, sa, 6);
    stw_be_p(f + 12, 0x88b5);
    for (size_t i = 14; i < len; i++) {
        f[i] = i * 7 + tag;
    }
}

/* ---- Tests ---- */

/* [spec:nuos:req:emu.esp32p4.c6/test] */
/*
 * The card: CMD5's OCR, an RCA, the CCCR and CIS pointer; CHIP_PU low
 * holds the C6 in reset, answering nothing, and its rise starts it afresh.
 */
static void test_card(void)
{
    int sock;
    QTestState *qts = start(&sock);
    uint32_t st;

    clocks_on(qts);
    card_init(qts);
    g_assert_cmphex(io_rw(qts, false, 0, CCCR_REV, 0), ==, 0x43);
    g_assert_cmphex(io_rw(qts, false, 0, CCCR_CIS_PTR + 1, 0), ==, 0x10);
    g_assert_cmphex(io_rw(qts, false, 0, CCCR_IOE, 0), ==, 0);
    io_rw(qts, true, 0, CCCR_IOE, 2);
    g_assert_cmphex(io_rw(qts, false, 0, CCCR_IOR, 0), ==, 2);
    io_rw(qts, true, 0, FBR1_BLKSIZE, 0x00);
    io_rw(qts, true, 0, FBR1_BLKSIZE + 1, 0x02);
    g_assert_cmphex(io_rw(qts, false, 0, FBR1_BLKSIZE + 1, 0), ==, 0x02);
    /* The receive buffers the C6 offers the host */
    g_assert_cmphex(reg_read(qts, FN1_TOKEN_RDATA) >> 16, ==, RX_BUFS);

    chip_pu(qts, true);
    st = cmd(qts, R1, 52, CCCR_REV << 9);
    g_assert_cmphex(st & INT_RTO, ==, INT_RTO);
    st = cmd(qts, R4, 5, 0);
    g_assert_cmphex(st & INT_RTO, ==, INT_RTO);

    chip_pu(qts, false);
    card_init(qts);
    g_assert_cmphex(io_rw(qts, false, 0, CCCR_IOE, 0), ==, 0);
    qtest_quit(qts);
    close(sock);
}

/* [spec:nuos:req:emu.esp32p4.c6/test] */
/*
 * Opening the data path: the init event, its chip and buffers, then the
 * ESPInit event, each announced by the new-packet interrupt on DAT1.
 */
static void test_data_path(void)
{
    int sock;
    QTestState *qts = start(&sock);
    Link l = { .qts = qts };
    Packet p;
    Rpc r;
    bool chip = false, bufs = false, version = false;

    clocks_on(qts);
    card_init(qts);
    io_rw(qts, true, 0, CCCR_IOE, 2);
    io_rw(qts, true, 0, CCCR_IEN, 3);
    io_rw(qts, true, 1, FN1_INT_ENA + 2, INT_NEW_PACKET >> 16);
    g_assert_cmphex(reg_read(qts, FN1_RX_TOTAL), ==, 0);
    qtest_writel(qts, SD_RINTSTS, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, SD_RINTSTS) & INT_SDIO(SLOT), ==, 0);

    io_rw(qts, true, 1, FN1_HOST_INT, 1);
    g_assert_cmphex(reg_read(qts, FN1_INT_RAW) & INT_NEW_PACKET, ==,
                    INT_NEW_PACKET);
    g_assert_cmphex(io_rw(qts, false, 0, CCCR_INT, 0), ==, 2);
    g_assert_cmphex(qtest_readl(qts, SD_RINTSTS) & INT_SDIO(SLOT), ==,
                    INT_SDIO(SLOT));
    io_rw(qts, true, 1, FN1_INT_CLR + 2, INT_NEW_PACKET >> 16);
    g_assert_cmphex(io_rw(qts, false, 0, CCCR_INT, 0), ==, 0);

    g_assert_true(link_next(&l, &p, 0));
    g_assert_cmpuint(p.iface, ==, IF_PRIV);
    g_assert_cmpuint(p.kind, ==, PRIV_KIND_INIT);
    g_assert_cmpuint(p.data[0], ==, PRIV_EVENT_INIT);
    g_assert_cmpuint(p.data[1], ==, p.len - 2);
    for (size_t at = 2; at + 2 <= p.len; at += 2 + p.data[at + 1]) {
        const uint8_t *v = &p.data[at + 2];

        switch (p.data[at]) {
        case TAG_CHIP_ID:
            chip = v[0] == 0x0d;
            break;
        case TAG_RX_BUFS:
            bufs = v[0] == RX_BUFS;
            break;
        case TAG_FW_VERSION:
            version = ldl_le_p(v) == ((2 << 16) | (13 << 8));
            break;
        }
    }
    g_assert_true(chip && bufs && version);
    g_assert_true(rpc_next(&l, &r, 0));
    g_assert_cmpuint(r.type, ==, RPC_TYPE_EVENT);
    g_assert_cmpuint(r.id, ==, RPC_EVENT_ESP_INIT);
    g_assert_false(link_next(&l, &p, 1000 * 1000));
    qtest_quit(qts);
    close(sock);
}

/* [spec:nuos:req:emu.esp32p4.c6/test] */
/*
 * RPCs: the firmware version, the MAC, an unmodelled request refused, and
 * each host packet freeing a receive buffer, which the token counts.
 */
static void test_rpc(void)
{
    int sock;
    QTestState *qts = start(&sock);
    Link l;
    Rpc r;
    const uint8_t *name, *mac;
    size_t len;
    uint32_t token;

    link_open(qts, &l);
    token = reg_read(qts, FN1_TOKEN_RDATA) >> 16;
    g_assert_cmpuint(rpc_call(&l, RPC_FW_VERSION, NULL, &r), ==, 0);
    g_assert_cmpuint(pb_get(r.body, r.body_len, 2), ==, 2);
    g_assert_cmpuint(pb_get(r.body, r.body_len, 3), ==, 13);
    g_assert_true(pb_field(r.body, r.body_len, 9, NULL, &name, &len));
    g_assert_cmpmem(name, len, "esp32c6", 7);
    g_assert_cmpuint(reg_read(qts, FN1_TOKEN_RDATA) >> 16, ==, token + 1);

    g_assert_cmpuint(rpc_call(&l, RPC_GET_MAC, NULL, &r), ==, 0);
    g_assert_true(pb_field(r.body, r.body_len, 1, NULL, &mac, &len));
    g_assert_cmpuint(len, ==, 6);
    g_assert_cmpuint(mac[0] & 1, ==, 0);

    g_assert_cmpuint(rpc_call(&l, 400, NULL, &r), ==, ESP_ERR_NOT_SUPPORTED);
    g_assert_cmpuint(reg_read(qts, FN1_TOKEN_RDATA) >> 16, ==, token + 3);
    qtest_quit(qts);
    close(sock);
}

/* [spec:nuos:req:emu.esp32p4.c6/test] */
/*
 * The station: no association with the wrong SSID or password, each with
 * the firmware's reason; with the access point's, frames pass both ways
 * between the host and the network, as they do not before or after.
 */
static void test_station(void)
{
    int sock;
    QTestState *qts = start(&sock);
    Link l;
    Rpc r;
    Packet p;
    uint8_t frame[64], got[PACKET];

    link_open(qts, &l);
    g_assert_cmpuint(rpc_call(&l, RPC_WIFI_START, NULL, &r), ==, 0);

    sta_config(&l, "elsewhere", AP_PASSWORD);
    g_assert_cmpuint(sta_connect(&l), ==, REASON_NO_AP_FOUND);
    sta_config(&l, AP_SSID, "wrong password");
    g_assert_cmpuint(sta_connect(&l), ==, REASON_4WAY_TIMEOUT);

    /* Unassociated, the C6 passes no frames either way. */
    make_frame(frame, sizeof(frame), 1);
    link_send(&l, IF_STA, frame, sizeof(frame));
    g_assert_cmpuint(sock_recv(qts, sock, got, sizeof(got)), ==, 0);
    sock_send(sock, frame, sizeof(frame));
    g_assert_false(link_next(&l, &p, 1000 * 1000));

    sta_config(&l, AP_SSID, AP_PASSWORD);
    g_assert_cmpuint(sta_connect(&l), ==, 0);

    make_frame(frame, sizeof(frame), 2);
    link_send(&l, IF_STA, frame, sizeof(frame));
    g_assert_cmpuint(sock_recv(qts, sock, got, sizeof(got)), ==,
                     sizeof(frame));
    g_assert_cmpmem(got, sizeof(frame), frame, sizeof(frame));

    make_frame(frame, sizeof(frame), 3);
    sock_send(sock, frame, sizeof(frame));
    g_assert_true(link_next(&l, &p, 10 * 1000 * 1000));
    g_assert_cmpuint(p.iface, ==, IF_STA);
    g_assert_cmpmem(p.data, p.len, frame, sizeof(frame));

    g_assert_cmpuint(rpc_call(&l, RPC_WIFI_DISCONNECT, NULL, &r), ==, 0);
    g_assert_true(rpc_next(&l, &r, 1000 * 1000));
    g_assert_cmpuint(r.id, ==, RPC_EVENT_DISCONNECTED);
    link_send(&l, IF_STA, frame, sizeof(frame));
    g_assert_cmpuint(sock_recv(qts, sock, got, sizeof(got)), ==, 0);
    qtest_quit(qts);
    close(sock);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/esp32p4/c6/card", test_card);
    qtest_add_func("/esp32p4/c6/data-path", test_data_path);
    qtest_add_func("/esp32p4/c6/rpc", test_rpc);
    qtest_add_func("/esp32p4/c6/station", test_station);
    return g_test_run();
}
