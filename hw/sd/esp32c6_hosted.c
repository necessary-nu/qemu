/*
 * ESP32-C6 Wi-Fi co-processor running esp-hosted-mcu, at its SDIO host
 * interface
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: esp-hosted-mcu v2.13.0 (commit 721a5818), its SDIO host
 * driver and slave, and NuOS's docs/c6-wire-profile.md, which records the
 * protocol as the board's C6 speaks it.
 *
 * [spec:nuos:req:emu.esp32p4.c6]
 * The C6 is modelled at the interface its host sees, not as a RISC-V core
 * running the firmware: an SDIO card with function 1, whose registers and
 * FIFO window are esp-hosted's SDIO slave's, and the firmware's behaviour
 * behind them. On the board it sits on the P4's SD slot 1, its CHIP_PU on
 * GPIO54.
 *
 * Function 1, through CMD52 and CMD53:
 *
 *  - 0x44 TOKEN_RDATA: bits 27:16 count the receive buffers the C6 has
 *    made free, modulo 4096; the host may send one packet per buffer;
 *  - 0x50 interrupt raw state: bit 23 a new packet for the host;
 *  - 0x60 the bytes the C6 has queued for the host, modulo 2^20;
 *  - 0x8c host-to-C6 interrupt: bit 0 opens the data path;
 *  - 0xd4 interrupt clear, 0xdc interrupt enable;
 *  - 0x400..0x1f7ff the FIFO: a transfer is addressed so that its data
 *    ends at 0x1f800. Reads take the queued bytes; writes fill the packet
 *    the host is sending, which the byte at 0x1f7ff ends. The host's
 *    word-length padding past 0x1f800 reads 0 and is dropped.
 *
 * The C6 streams its packets: each is a 12-byte header (interface,
 * flags, length, offset 12, checksum, sequence, throttle, kind) and its
 * payload, back to back. Once the data path opens it sends its init event
 * (private interface, kind 0x33) and then the ESPInit RPC event. Station
 * packets carry Ethernet frames, which reach the machine's network
 * backend while the station is associated; serial packets carry RPC
 * messages, protobuf in esp-hosted's TLV framing, fragmented by
 * MORE_FRAGMENT.
 *
 * The radio is one access point, its SSID, password, security, channel
 * and signal the device's properties: a scan finds it, and a connection
 * to it with its SSID and password associates after 20 ms of virtual
 * time; any other ends in a disconnection event with the reason the
 * firmware would give.
 *
 * Not modelled: the C6's boot time after CHIP_PU rises (it answers at
 * once), throttling (it never throttles the host), power saving, other
 * interfaces than station and serial, other RPCs than the station's
 * (answered ESP_ERR_NOT_SUPPORTED), and the radio itself: no WPA
 * handshake runs and frames are not delayed or lost.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/bitops.h"
#include "qemu/bswap.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/resettable.h"
#include "hw/sd/esp32c6_hosted.h"
#include "migration/vmstate.h"
#include "net/net.h"
#include "system/reset.h"
#include "sdmmc-internal.h"
#include "trace.h"

#define SDIO_OCR                0x00ff8000u
#define R4_READY                BIT(31)
#define R4_NF_SHIFT             28
#define R5_FUNCTION_NUMBER      BIT(9)
#define R5_ERROR                BIT(11)
#define R5_STATE_CMD            (1u << 12)
#define R5_STATE_TRN            (2u << 12)

#define IO_RW                   BIT(31)
#define IO_FN(arg)              (((arg) >> 28) & 7)
#define IO_RAW                  BIT(27)
#define IO_BLOCK                BIT(27)
#define IO_OP_INC               BIT(26)
#define IO_ADDR(arg)            (((arg) >> 9) & 0x1ffff)

#define CCCR_REV                0x00
#define CCCR_SD_REV             0x01
#define CCCR_IOE                0x02
#define CCCR_IOR                0x03
#define CCCR_IEN                0x04
#define CCCR_INT                0x05
#define CCCR_ABORT              0x06
#define CCCR_BUS_IF             0x07
#define CCCR_CAP                0x08
#define CCCR_CIS_PTR            0x09
#define CCCR_BLKSIZE            0x10
#define CCCR_SPEED              0x13
#define ABORT_RES               BIT(3)
#define ABORT_AS_MASK           7
#define BUS_IF_WIDTH_MASK       3
/* The physical C6 reports the SCSI capability, read-only, in bit 6. */
#define BUS_IF_SCSI             BIT(6)
#define CAP_FIXED               0x13
#define SPEED_SHS               BIT(0)
#define FBR_CIS_PTR             0x09
#define FBR_BLKSIZE             0x10
#define CIS_COMMON              0x1000
#define CIS_FN1                 0x2000
#define CIS_END                 0x3000
#define CIS_MAX_BLOCK           512
#define CISTPL_MANFID           0x20
#define CISTPL_FUNCID           0x21
#define CISTPL_FUNCE            0x22
#define CISTPL_END              0xff
#define FUNCID_SDIO             0x0c
/* Espressif's SDIO vendor ID, and a device ID of the model's */
#define CIS_VENDOR              0x6666
#define CIS_DEVICE              0x2222

enum {
    CARD_INIT,
    CARD_READY,
    CARD_STBY,
    CARD_CMD,
    CARD_TRN,
    CARD_INACTIVE,
};

/* ---- esp-hosted's host interface ---- */

#define FN1_TOKEN_RDATA         0x44
#define FN1_INT_RAW             0x50
#define FN1_RX_TOTAL            0x60
#define FN1_HOST_INT            0x8c
#define FN1_INT_CLR             0xd4
#define FN1_INT_ENA             0xdc
#define FN1_REG_END             0x400
#define FN1_FIFO_END            0x1f800

#define INT_NEW_PACKET          BIT(23)
#define RX_TOTAL_MASK           0xfffff
#define TOKEN_MASK              0xfff
#define HOST_INT_OPEN_DATA_PATH BIT(0)

#define HDR_LEN                 12
#define HDR_MORE_FRAGMENT       BIT(0)
#define IF_STA                  1
#define IF_SERIAL               3
#define IF_PRIV                 5
#define PRIV_KIND_INIT          0x33
#define PRIV_EVENT_INIT         0x22

/* Init event tags and their values */
#define TAG_CAPABILITY          0x11
#define TAG_CHIP_ID             0x12
#define TAG_RAW_TP              0x13
#define TAG_RX_BUFS             0x14
#define TAG_TX_QUEUE            0x15
#define TAG_EXT_CAPABILITY      0x16
#define TAG_FW_VERSION          0x17
#define TAG_SDIO_MODE           0x18
#define TAG_TRANSFER_SIZE       0x1c
#define CAP_SDIO_WLAN           BIT(0)
#define CAP_CHECKSUM            BIT(7)
#define CHIP_ESP32C6            0x0d
#define FW_MAJOR                2
#define FW_MINOR                13
#define FW_PATCH                0
#define SDIO_MODE_STREAMING     1

/* RPC message types and identifiers (esp_hosted_rpc.proto) */
#define RPC_TYPE_REQ            1
#define RPC_TYPE_RESP           2
#define RPC_TYPE_EVENT          3
#define RPC_REQ_GET_MAC         257
#define RPC_REQ_SET_MODE        260
#define RPC_REQ_WIFI_INIT       278
#define RPC_REQ_WIFI_START      280
#define RPC_REQ_WIFI_STOP       281
#define RPC_REQ_WIFI_CONNECT    282
#define RPC_REQ_WIFI_DISCONNECT 283
#define RPC_REQ_SET_CONFIG      284
#define RPC_REQ_SCAN_START      286
#define RPC_REQ_SCAN_STOP       287
#define RPC_REQ_SCAN_AP_NUM     288
#define RPC_REQ_SCAN_AP_RECORDS 289
#define RPC_REQ_CLEAR_AP_LIST   290
#define RPC_REQ_STA_AP_INFO     294
#define RPC_REQ_SET_STORAGE     313
#define RPC_REQ_STA_RSSI        341
#define RPC_REQ_FW_VERSION      350
#define RPC_RESP_OFFSET         256
#define RPC_EVENT_ESP_INIT      769
#define RPC_EVENT_SCAN_DONE     774
#define RPC_EVENT_STA_CONNECTED 775
#define RPC_EVENT_STA_DISCONNECTED 776
#define ESP_ERR_NOT_SUPPORTED   0x106
#define ESP_ERR_WIFI_NOT_STARTED 0x3004

/* The firmware's disconnection reasons (esp_wifi_types.h) */
#define REASON_ASSOC_LEAVE      8
#define REASON_4WAY_TIMEOUT     15
#define REASON_NO_AP_FOUND      201
#define REASON_NO_AP_SECURITY   210

/* How long the radio takes, in virtual time */
#define CONNECT_NS              (20 * SCALE_MS)
#define SCAN_NS                 (50 * SCALE_MS)

enum {
    RADIO_IDLE,
    RADIO_CONNECT,
    RADIO_SCAN,
};

/* ---- Protobuf ---- */

typedef struct {
    uint8_t *buf;
    size_t cap;
    size_t len;
    bool overflow;
} PbWriter;

static void pb_byte(PbWriter *w, uint8_t b)
{
    if (w->len < w->cap) {
        w->buf[w->len++] = b;
    } else {
        w->overflow = true;
    }
}

static void pb_varint(PbWriter *w, uint64_t v)
{
    do {
        pb_byte(w, (v & 0x7f) | (v > 0x7f ? 0x80 : 0));
        v >>= 7;
    } while (v);
}

static void pb_uint(PbWriter *w, uint32_t field, uint64_t v)
{
    pb_varint(w, (uint64_t)field << 3);
    pb_varint(w, v);
}

/* A negative int32 is sent sign-extended to ten bytes, as protobuf does. */
static void pb_int32(PbWriter *w, uint32_t field, int32_t v)
{
    pb_uint(w, field, (uint64_t)(int64_t)v);
}

static void pb_bytes(PbWriter *w, uint32_t field, const void *p, size_t n)
{
    pb_varint(w, ((uint64_t)field << 3) | 2);
    pb_varint(w, n);
    for (size_t i = 0; i < n; i++) {
        pb_byte(w, ((const uint8_t *)p)[i]);
    }
}

typedef struct {
    uint32_t field;
    unsigned wire;
    uint64_t value;
    const uint8_t *bytes;
    size_t len;
} PbField;

static bool pb_read_varint(const uint8_t *p, size_t len, size_t *at,
                           uint64_t *v)
{
    *v = 0;
    for (unsigned shift = 0; shift < 64; shift += 7) {
        if (*at >= len) {
            return false;
        }
        uint8_t b = p[(*at)++];
        *v |= (uint64_t)(b & 0x7f) << shift;
        if (!(b & 0x80)) {
            return true;
        }
    }
    return false;
}

/* The next field at *at; false at the end or on a malformed message. */
static bool pb_next(const uint8_t *p, size_t len, size_t *at, PbField *f)
{
    uint64_t key;

    if (*at >= len || !pb_read_varint(p, len, at, &key)) {
        return false;
    }
    f->field = key >> 3;
    f->wire = key & 7;
    f->bytes = NULL;
    f->len = 0;
    switch (f->wire) {
    case 0:
        return pb_read_varint(p, len, at, &f->value);
    case 1:
    case 5: {
        size_t n = f->wire == 1 ? 8 : 4;

        if (len - *at < n) {
            return false;
        }
        f->value = 0;
        memcpy(&f->value, p + *at, n);
        f->value = le64_to_cpu(f->value);
        *at += n;
        return true;
    }
    case 2:
        if (!pb_read_varint(p, len, at, &f->value) || f->value > len - *at) {
            return false;
        }
        f->bytes = p + *at;
        f->len = f->value;
        *at += f->len;
        return true;
    default:
        return false;
    }
}

static bool pb_find(const uint8_t *p, size_t len, uint32_t field,
                    unsigned wire, PbField *out)
{
    size_t at = 0;
    PbField f;

    while (pb_next(p, len, &at, &f)) {
        if (f.field == field && f.wire == wire) {
            *out = f;
            return true;
        }
    }
    return false;
}

static bool pb_find_uint(const uint8_t *p, size_t len, uint32_t field,
                         uint64_t *v)
{
    PbField f;

    if (!pb_find(p, len, field, 0, &f)) {
        return false;
    }
    *v = f.value;
    return true;
}

static bool pb_find_bytes(const uint8_t *p, size_t len, uint32_t field,
                          const uint8_t **bytes, size_t *n)
{
    PbField f;

    if (!pb_find(p, len, field, 2, &f)) {
        return false;
    }
    *bytes = f.bytes;
    *n = f.len;
    return true;
}

/* ---- The card's interrupt ---- */

static bool c6_running(Esp32c6HostedState *s)
{
    return s->chip_pu;
}

/*
 * The card interrupts its host on DAT1 while the host has enabled it
 * (IEN's master and function 1 bits) and a raw interrupt it has enabled
 * (0xdc) is set.
 */
static void c6_update_irq(Esp32c6HostedState *s)
{
    bool level = c6_running(s) && s->data_path &&
                 (s->ien & BIT(0)) && (s->ien & BIT(1)) &&
                 (s->int_raw & s->int_ena);

    if (level != s->card_irq) {
        s->card_irq = level;
        trace_esp32c6_hosted_irq(level);
        if (s->bus) {
            sdbus_set_irq(s->bus, level);
        }
    }
}

/* ---- Packets to the host ---- */

#define FIFO_CAPACITY (ESP32C6_HOSTED_TX_QUEUE * ESP32C6_HOSTED_PACKET)

static void fifo_push(Esp32c6HostedState *s, const uint8_t *p, uint32_t n)
{
    uint32_t cap = FIFO_CAPACITY;

    for (uint32_t i = 0; i < n; i++) {
        s->fifo[(s->fifo_head + s->fifo_len) % cap] = p[i];
        s->fifo_len++;
    }
    s->rx_total = (s->rx_total + n) & RX_TOTAL_MASK;
    s->int_raw |= INT_NEW_PACKET;
    c6_update_irq(s);
}

static uint8_t fifo_pop(Esp32c6HostedState *s)
{
    uint8_t b;

    if (s->fifo_len == 0) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32c6-hosted: host reads past the "
                      "bytes queued for it\n");
        return 0;
    }
    b = s->fifo[s->fifo_head];
    s->fifo_head = (s->fifo_head + 1) % FIFO_CAPACITY;
    s->fifo_len--;
    return b;
}

/*
 * Queue one packet for the host, or none if the queue has no room: the
 * host's receive limit is the advertised queue, so the C6 holds no more.
 */
static bool c6_send_packet(Esp32c6HostedState *s, uint8_t iface, uint8_t flags,
                           uint16_t seq, uint8_t kind, const uint8_t *payload,
                           uint32_t len)
{
    uint8_t pkt[ESP32C6_HOSTED_PACKET];
    uint16_t sum = 0;

    assert(len > 0 && len <= ESP32C6_HOSTED_PACKET - HDR_LEN);
    if (FIFO_CAPACITY - s->fifo_len < HDR_LEN + len) {
        s->dropped++;
        return false;
    }
    memset(pkt, 0, HDR_LEN);
    pkt[0] = iface;
    pkt[1] = flags;
    stw_le_p(&pkt[2], len);
    stw_le_p(&pkt[4], HDR_LEN);
    stw_le_p(&pkt[8], seq);
    pkt[11] = kind;
    memcpy(&pkt[HDR_LEN], payload, len);
    for (uint32_t i = 0; i < HDR_LEN + len; i++) {
        sum += i == 6 || i == 7 ? 0 : pkt[i];
    }
    stw_le_p(&pkt[6], sum);
    fifo_push(s, pkt, HDR_LEN + len);
    trace_esp32c6_hosted_to_host(iface, HDR_LEN + len, s->fifo_len);
    return true;
}

/* The init event, which tells the host how to talk to the firmware. */
static void c6_send_init(Esp32c6HostedState *s)
{
    uint8_t e[64];
    unsigned n = 2;

#define TLV1(tag, v) do { e[n++] = (tag); e[n++] = 1; e[n++] = (v); } while (0)
#define TLV4(tag, v) do { e[n++] = (tag); e[n++] = 4; \
                          stl_le_p(&e[n], (v)); n += 4; } while (0)
    TLV1(TAG_CAPABILITY, CAP_SDIO_WLAN | CAP_CHECKSUM);
    TLV1(TAG_CHIP_ID, CHIP_ESP32C6);
    TLV1(TAG_RAW_TP, 0);
    TLV1(TAG_RX_BUFS, ESP32C6_HOSTED_RX_BUFS);
    TLV1(TAG_TX_QUEUE, ESP32C6_HOSTED_TX_QUEUE);
    TLV4(TAG_EXT_CAPABILITY, 0);
    TLV4(TAG_FW_VERSION, (FW_MAJOR << 16) | (FW_MINOR << 8) | FW_PATCH);
    TLV1(TAG_SDIO_MODE, SDIO_MODE_STREAMING);
    TLV4(TAG_TRANSFER_SIZE, ESP32C6_HOSTED_PACKET);
#undef TLV1
#undef TLV4
    e[0] = PRIV_EVENT_INIT;
    e[1] = n - 2;
    c6_send_packet(s, IF_PRIV, 0, 0, PRIV_KIND_INIT, e, n);
}

/*
 * An RPC message to the host, in esp-hosted's serial TLVs, fragmented: the
 * C6 numbers each fragment it sends, as its serial_write_data() does.
 */
static void c6_send_rpc(Esp32c6HostedState *s, bool event, uint32_t id,
                        uint32_t uid, const uint8_t *body, size_t body_len)
{
    uint8_t msg[ESP32C6_HOSTED_MESSAGE];
    uint8_t env[ESP32C6_HOSTED_MESSAGE - 12];
    PbWriter w = { .buf = env, .cap = sizeof(env) };
    size_t len, at;

    pb_uint(&w, 1, event ? RPC_TYPE_EVENT : RPC_TYPE_RESP);
    pb_uint(&w, 2, id);
    if (!event) {
        pb_uint(&w, 3, uid);
    }
    pb_bytes(&w, id, body, body_len);
    if (w.overflow) {
        qemu_log_mask(LOG_UNIMP, "esp32c6-hosted: RPC %u too large\n", id);
        return;
    }
    msg[0] = 1;
    stw_le_p(&msg[1], 6);
    memcpy(&msg[3], event ? "RPCEvt" : "RPCRsp", 6);
    msg[9] = 2;
    stw_le_p(&msg[10], w.len);
    memcpy(&msg[12], env, w.len);
    len = 12 + w.len;
    for (at = 0; at < len;) {
        size_t n = MIN(len - at, ESP32C6_HOSTED_PACKET - HDR_LEN);

        c6_send_packet(s, IF_SERIAL, at + n < len ? HDR_MORE_FRAGMENT : 0,
                       s->serial_seq++, 0, &msg[at], n);
        at += n;
    }
}

static void c6_send_status(Esp32c6HostedState *s, uint32_t req, uint32_t uid,
                           uint32_t status)
{
    uint8_t body[8];
    PbWriter w = { .buf = body, .cap = sizeof(body) };

    pb_uint(&w, 1, status);
    c6_send_rpc(s, false, req + RPC_RESP_OFFSET, uid, body, w.len);
}

/* ---- The radio ---- */

static void ap_bssid(Esp32c6HostedState *s, uint8_t bssid[6])
{
    /* A locally administered unicast address, apart from the station's */
    memcpy(bssid, s->conf.macaddr.a, 6);
    bssid[0] = 0x02;
    bssid[5] ^= 0x80;
}

static void ap_record(Esp32c6HostedState *s, PbWriter *w, uint32_t field)
{
    uint8_t rec[96];
    PbWriter r = { .buf = rec, .cap = sizeof(rec) };
    uint8_t bssid[6];
    size_t ssid_len = strlen(s->ap_ssid);
    uint8_t ssid[33];

    ap_bssid(s, bssid);
    /* esp-hosted sends a scan record's SSID with its terminator */
    memcpy(ssid, s->ap_ssid, ssid_len);
    ssid[ssid_len] = 0;
    pb_bytes(&r, 1, bssid, 6);
    pb_bytes(&r, 2, ssid, ssid_len + 1);
    pb_uint(&r, 3, s->ap_channel);
    pb_int32(&r, 5, s->ap_rssi);
    pb_uint(&r, 6, s->ap_authmode);
    pb_bytes(w, field, rec, r.len);
}

static void c6_event_connected(Esp32c6HostedState *s)
{
    uint8_t data[96], body[112];
    PbWriter d = { .buf = data, .cap = sizeof(data) };
    PbWriter w = { .buf = body, .cap = sizeof(body) };
    uint8_t bssid[6];

    ap_bssid(s, bssid);
    pb_bytes(&d, 1, s->ap_ssid, strlen(s->ap_ssid));
    pb_uint(&d, 2, strlen(s->ap_ssid));
    pb_bytes(&d, 3, bssid, 6);
    pb_uint(&d, 4, s->ap_channel);
    pb_uint(&d, 5, s->ap_authmode);
    pb_uint(&w, 1, 0);
    pb_bytes(&w, 2, data, d.len);
    c6_send_rpc(s, true, RPC_EVENT_STA_CONNECTED, 0, body, w.len);
}

static void c6_event_disconnected(Esp32c6HostedState *s, uint32_t reason)
{
    uint8_t data[96], body[112];
    PbWriter d = { .buf = data, .cap = sizeof(data) };
    PbWriter w = { .buf = body, .cap = sizeof(body) };
    uint8_t bssid[6];

    ap_bssid(s, bssid);
    pb_bytes(&d, 1, s->sta_ssid, s->sta_ssid_len);
    pb_uint(&d, 2, s->sta_ssid_len);
    pb_bytes(&d, 3, bssid, 6);
    pb_uint(&d, 4, reason);
    pb_int32(&d, 5, s->ap_rssi);
    pb_uint(&w, 1, 0);
    pb_bytes(&w, 2, data, d.len);
    c6_send_rpc(s, true, RPC_EVENT_STA_DISCONNECTED, 0, body, w.len);
}

static void c6_event_scan_done(Esp32c6HostedState *s)
{
    uint8_t data[16], body[32];
    PbWriter d = { .buf = data, .cap = sizeof(data) };
    PbWriter w = { .buf = body, .cap = sizeof(body) };

    pb_uint(&d, 1, 0);
    pb_uint(&d, 2, 1);
    pb_uint(&w, 1, 0);
    pb_bytes(&w, 2, data, d.len);
    c6_send_rpc(s, true, RPC_EVENT_SCAN_DONE, 0, body, w.len);
}

/*
 * The firmware's verdict on a connection: the AP must have the SSID asked
 * for, meet the security threshold, and take the password.
 */
static uint32_t c6_connect_reason(Esp32c6HostedState *s)
{
    size_t ssid_len = strlen(s->ap_ssid);
    size_t password_len = strlen(s->ap_password);

    if (s->sta_ssid_len != ssid_len ||
        memcmp(s->sta_ssid, s->ap_ssid, ssid_len)) {
        return REASON_NO_AP_FOUND;
    }
    if (s->sta_threshold > s->ap_authmode) {
        return REASON_NO_AP_SECURITY;
    }
    if (s->sta_password_len != password_len ||
        memcmp(s->sta_password, s->ap_password, password_len)) {
        return REASON_4WAY_TIMEOUT;
    }
    return 0;
}

static void c6_radio_cb(void *opaque)
{
    Esp32c6HostedState *s = opaque;
    int pending = s->radio_pending;
    uint32_t reason;

    s->radio_pending = RADIO_IDLE;
    if (!c6_running(s) || !s->started) {
        return;
    }
    switch (pending) {
    case RADIO_CONNECT:
        reason = c6_connect_reason(s);
        if (reason) {
            c6_event_disconnected(s, reason);
            break;
        }
        s->associated = true;
        c6_event_connected(s);
        qemu_flush_queued_packets(qemu_get_queue(s->nic));
        break;
    case RADIO_SCAN:
        c6_event_scan_done(s);
        break;
    default:
        break;
    }
}

static void c6_radio_start(Esp32c6HostedState *s, int what, int64_t ns)
{
    s->radio_pending = what;
    timer_mod(&s->radio_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + ns);
}

/* WifiSetConfig's station: SSID 1, password 2, threshold 9 (authmode 2). */
static void c6_set_config(Esp32c6HostedState *s, const uint8_t *body,
                          size_t len)
{
    const uint8_t *cfg, *sta, *ssid, *password = NULL, *threshold;
    size_t cfg_len, sta_len, ssid_len, password_len, threshold_len;
    uint64_t authmode = 0;

    if (!pb_find_bytes(body, len, 2, &cfg, &cfg_len) ||
        !pb_find_bytes(cfg, cfg_len, 2, &sta, &sta_len) ||
        !pb_find_bytes(sta, sta_len, 1, &ssid, &ssid_len) ||
        ssid_len > sizeof(s->sta_ssid)) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32c6-hosted: WifiSetConfig "
                      "without a station SSID\n");
        return;
    }
    if (!pb_find_bytes(sta, sta_len, 2, &password, &password_len)) {
        password_len = 0;
    }
    password_len = MIN(password_len, sizeof(s->sta_password));
    if (pb_find_bytes(sta, sta_len, 9, &threshold, &threshold_len)) {
        pb_find_uint(threshold, threshold_len, 2, &authmode);
    }
    memcpy(s->sta_ssid, ssid, ssid_len);
    s->sta_ssid_len = ssid_len;
    if (password_len) {
        memcpy(s->sta_password, password, password_len);
    }
    s->sta_password_len = password_len;
    s->sta_threshold = authmode;
}

static void c6_disassociate(Esp32c6HostedState *s)
{
    if (s->associated) {
        s->associated = false;
        c6_event_disconnected(s, REASON_ASSOC_LEAVE);
    }
}

/* [spec:nuos:req:emu.esp32p4.c6] */
static void c6_rpc_request(Esp32c6HostedState *s, const uint8_t *pb,
                           size_t len)
{
    uint8_t body[1024];
    PbWriter w = { .buf = body, .cap = sizeof(body) };
    uint64_t type, id, uid, n;
    const uint8_t *req;
    size_t req_len;

    if (!pb_find_uint(pb, len, 1, &type) || type != RPC_TYPE_REQ ||
        !pb_find_uint(pb, len, 2, &id) || id > UINT32_MAX ||
        !pb_find_uint(pb, len, 3, &uid) ||
        !pb_find_bytes(pb, len, id, &req, &req_len)) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32c6-hosted: malformed RPC "
                      "request\n");
        return;
    }
    trace_esp32c6_hosted_rpc(id, uid);
    switch (id) {
    case RPC_REQ_FW_VERSION:
        pb_uint(&w, 1, 0);
        pb_uint(&w, 2, FW_MAJOR);
        pb_uint(&w, 3, FW_MINOR);
        pb_uint(&w, 4, FW_PATCH);
        pb_uint(&w, 8, CHIP_ESP32C6);
        pb_bytes(&w, 9, "esp32c6", 7);
        break;
    case RPC_REQ_GET_MAC:
        /* The MAC response alone has its status in field 2. */
        pb_bytes(&w, 1, s->conf.macaddr.a, 6);
        pb_uint(&w, 2, 0);
        break;
    case RPC_REQ_SET_MODE:
    case RPC_REQ_WIFI_INIT:
    case RPC_REQ_SET_STORAGE:
    case RPC_REQ_CLEAR_AP_LIST:
    case RPC_REQ_SCAN_STOP:
        pb_uint(&w, 1, 0);
        break;
    case RPC_REQ_WIFI_START:
        s->started = true;
        pb_uint(&w, 1, 0);
        break;
    case RPC_REQ_WIFI_STOP:
        c6_disassociate(s);
        s->started = false;
        pb_uint(&w, 1, 0);
        break;
    case RPC_REQ_SET_CONFIG:
        c6_set_config(s, req, req_len);
        pb_uint(&w, 1, 0);
        break;
    case RPC_REQ_WIFI_CONNECT:
        if (!s->started) {
            pb_uint(&w, 1, ESP_ERR_WIFI_NOT_STARTED);
            break;
        }
        pb_uint(&w, 1, 0);
        c6_radio_start(s, RADIO_CONNECT, CONNECT_NS);
        break;
    case RPC_REQ_WIFI_DISCONNECT:
        pb_uint(&w, 1, 0);
        c6_send_rpc(s, false, id + RPC_RESP_OFFSET, uid, body, w.len);
        c6_disassociate(s);
        return;
    case RPC_REQ_SCAN_START:
        if (!s->started) {
            pb_uint(&w, 1, ESP_ERR_WIFI_NOT_STARTED);
            break;
        }
        pb_uint(&w, 1, 0);
        c6_radio_start(s, RADIO_SCAN, SCAN_NS);
        break;
    case RPC_REQ_SCAN_AP_NUM:
        pb_uint(&w, 1, 0);
        pb_uint(&w, 2, 1);
        break;
    case RPC_REQ_SCAN_AP_RECORDS:
        n = 0;
        pb_find_uint(req, req_len, 1, &n);
        pb_uint(&w, 1, 0);
        pb_uint(&w, 2, MIN(n, 1));
        if (n) {
            ap_record(s, &w, 3);
        }
        break;
    case RPC_REQ_STA_AP_INFO:
        pb_uint(&w, 1, 0);
        if (s->associated) {
            ap_record(s, &w, 2);
        }
        break;
    case RPC_REQ_STA_RSSI:
        pb_uint(&w, 1, 0);
        pb_int32(&w, 2, s->ap_rssi);
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "esp32c6-hosted: RPC request %" PRIu64
                      " is not modelled\n", id);
        c6_send_status(s, id, uid, ESP_ERR_NOT_SUPPORTED);
        return;
    }
    c6_send_rpc(s, false, id + RPC_RESP_OFFSET, uid, body, w.len);
}

/* A complete serial message: esp-hosted's TLVs around an RPC request. */
static void c6_serial_message(Esp32c6HostedState *s, const uint8_t *m,
                              size_t len)
{
    if (len < 12 || m[0] != 1 || lduw_le_p(&m[1]) != 6 ||
        memcmp(&m[3], "RPCRsp", 6) || m[9] != 2 ||
        lduw_le_p(&m[10]) != len - 12) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32c6-hosted: malformed serial "
                      "message\n");
        return;
    }
    c6_rpc_request(s, &m[12], len - 12);
}

/* [spec:nuos:req:emu.esp32p4.c6] */
static void c6_host_packet(Esp32c6HostedState *s)
{
    const uint8_t *p = s->rx;
    uint32_t len;
    uint16_t sum = 0;

    if (s->rx_overflow || s->rx_len < HDR_LEN) {
        goto bad;
    }
    len = lduw_le_p(&p[2]);
    if (len == 0 || lduw_le_p(&p[4]) != HDR_LEN || HDR_LEN + len != s->rx_len) {
        goto bad;
    }
    for (uint32_t i = 0; i < s->rx_len; i++) {
        sum += i == 6 || i == 7 ? 0 : p[i];
    }
    if (sum != lduw_le_p(&p[6])) {
        goto bad;
    }
    trace_esp32c6_hosted_from_host(p[0], s->rx_len);
    switch (p[0]) {
    case IF_STA:
        if (s->associated) {
            qemu_send_packet(qemu_get_queue(s->nic), &p[HDR_LEN], len);
        }
        break;
    case IF_SERIAL:
        if (s->message_len + len > sizeof(s->message)) {
            s->message_overflow = true;
        } else {
            memcpy(&s->message[s->message_len], &p[HDR_LEN], len);
            s->message_len += len;
        }
        if (!(p[1] & HDR_MORE_FRAGMENT)) {
            if (!s->message_overflow) {
                c6_serial_message(s, s->message, s->message_len);
            }
            s->message_len = 0;
            s->message_overflow = false;
        }
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "esp32c6-hosted: packet for interface %u "
                      "is not modelled\n", p[0]);
        break;
    }
    goto done;
bad:
    qemu_log_mask(LOG_GUEST_ERROR, "esp32c6-hosted: malformed packet from "
                  "the host (%u bytes)\n", s->rx_len);
done:
    s->rx_len = 0;
    s->rx_overflow = false;
    /* The packet's buffer is free again: one more credit for the host. */
    s->token = (s->token + 1) & TOKEN_MASK;
}

/* ---- The network ---- */

static bool c6_can_receive(NetClientState *nc)
{
    Esp32c6HostedState *s = qemu_get_nic_opaque(nc);

    return !s->associated ||
           FIFO_CAPACITY - s->fifo_len >= ESP32C6_HOSTED_PACKET;
}

static ssize_t c6_receive(NetClientState *nc, const uint8_t *buf, size_t size)
{
    Esp32c6HostedState *s = qemu_get_nic_opaque(nc);

    if (!c6_running(s) || !s->data_path || !s->associated ||
        size > ESP32C6_HOSTED_PACKET - HDR_LEN) {
        return size;
    }
    if (!c6_send_packet(s, IF_STA, 0, 0, 0, buf, size)) {
        return 0;
    }
    return size;
}

static NetClientInfo net_esp32c6_hosted_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = c6_can_receive,
    .receive = c6_receive,
};

/* ---- Function 0: CCCR, FBR and CIS ---- */

static uint8_t cis_byte(uint32_t addr)
{
    static const uint8_t common[] = {
        CISTPL_MANFID, 4, CIS_VENDOR & 0xff, CIS_VENDOR >> 8,
        CIS_DEVICE & 0xff, CIS_DEVICE >> 8,
        CISTPL_FUNCID, 2, FUNCID_SDIO, 0,
        CISTPL_FUNCE, 4, 0, CIS_MAX_BLOCK & 0xff, CIS_MAX_BLOCK >> 8, 0x32,
        CISTPL_END,
    };
    static const uint8_t fn1[] = {
        CISTPL_FUNCID, 2, FUNCID_SDIO, 0,
        CISTPL_END,
    };
    const uint8_t *t = addr < CIS_FN1 ? common : fn1;
    size_t n = addr < CIS_FN1 ? sizeof(common) : sizeof(fn1);
    uint32_t off = addr % 0x1000;

    return off < n ? t[off] : 0;
}

static uint8_t fn0_read(Esp32c6HostedState *s, uint32_t addr)
{
    if (addr >= CIS_COMMON && addr < CIS_END) {
        return cis_byte(addr);
    }
    if (addr >= 0x100 && addr < 0x200) {
        switch (addr & 0xff) {
        case FBR_CIS_PTR:
            return CIS_FN1 & 0xff;
        case FBR_CIS_PTR + 1:
            return CIS_FN1 >> 8;
        case FBR_BLKSIZE:
            return s->blksize[1];
        case FBR_BLKSIZE + 1:
            return s->blksize[1] >> 8;
        default:
            return 0;
        }
    }
    switch (addr) {
    case CCCR_REV:
        return 0x43;
    case CCCR_SD_REV:
        return 0x03;
    case CCCR_IOE:
        return s->ioe;
    case CCCR_IOR:
        return s->ioe;
    case CCCR_IEN:
        return s->ien;
    case CCCR_INT:
        return s->card_irq ? BIT(1) : 0;
    case CCCR_BUS_IF:
        return s->bus_if | BUS_IF_SCSI;
    case CCCR_CAP:
        return CAP_FIXED;
    case CCCR_CIS_PTR:
        return CIS_COMMON & 0xff;
    case CCCR_CIS_PTR + 1:
        return CIS_COMMON >> 8;
    case CCCR_BLKSIZE:
        return s->blksize[0];
    case CCCR_BLKSIZE + 1:
        return s->blksize[0] >> 8;
    case CCCR_SPEED:
        return SPEED_SHS;
    default:
        return 0;
    }
}

static void card_reset_state(Esp32c6HostedState *s)
{
    s->card_state = CARD_INIT;
    s->rca = 0;
    s->ioe = 0;
    s->ien = 0;
    s->bus_if = 0;
    memset(s->blksize, 0, sizeof(s->blksize));
    s->xfer_active = false;
}

static void set_blksize_byte(uint16_t *bs, unsigned hi, uint8_t v)
{
    *bs = hi ? (*bs & 0xff) | (v << 8) : (*bs & 0xff00) | v;
}

static void fn0_write(Esp32c6HostedState *s, uint32_t addr, uint8_t v)
{
    if (addr >= 0x100 && addr < 0x200) {
        if ((addr & 0xff) == FBR_BLKSIZE || (addr & 0xff) == FBR_BLKSIZE + 1) {
            set_blksize_byte(&s->blksize[1], addr & 1, v);
        }
        return;
    }
    switch (addr) {
    case CCCR_IOE:
        s->ioe = v & BIT(1);
        break;
    case CCCR_IEN:
        s->ien = v & (BIT(0) | BIT(1));
        break;
    case CCCR_ABORT:
        if (v & ABORT_RES) {
            card_reset_state(s);
        } else if (s->xfer_active && (v & ABORT_AS_MASK) == s->xfer_fn) {
            s->xfer_active = false;
            s->card_state = CARD_CMD;
        }
        break;
    case CCCR_BUS_IF:
        s->bus_if = v & ~BUS_IF_SCSI & (BUS_IF_WIDTH_MASK | BIT(5) | BIT(7));
        break;
    case CCCR_BLKSIZE:
    case CCCR_BLKSIZE + 1:
        set_blksize_byte(&s->blksize[0], addr & 1, v);
        break;
    default:
        break;
    }
    c6_update_irq(s);
}

/* ---- Function 1: esp-hosted's registers and FIFO ---- */

static uint32_t fn1_reg_value(Esp32c6HostedState *s, uint32_t addr)
{
    switch (addr) {
    case FN1_TOKEN_RDATA:
        return s->token << 16;
    case FN1_INT_RAW:
        return s->int_raw;
    case FN1_RX_TOTAL:
        return s->rx_total;
    case FN1_INT_ENA:
        return s->int_ena;
    default:
        return 0;
    }
}

static uint32_t fn1_reg(Esp32c6HostedState *s, uint32_t addr)
{
    uint32_t value = fn1_reg_value(s, addr);

    trace_esp32c6_hosted_reg_read(addr, value);
    return value;
}

/* [spec:nuos:req:emu.esp32p4.c6] */
static void fn1_reg_write(Esp32c6HostedState *s, uint32_t addr, uint8_t v)
{
    unsigned shift = (addr & 3) * 8;

    trace_esp32c6_hosted_reg_write(addr, v);
    switch (addr & ~3u) {
    case FN1_INT_CLR:
        s->int_raw &= ~((uint32_t)v << shift);
        break;
    case FN1_INT_ENA:
        s->int_ena = (s->int_ena & ~(0xffu << shift)) | ((uint32_t)v << shift);
        break;
    case FN1_HOST_INT:
        if (shift == 0 && (v & HOST_INT_OPEN_DATA_PATH) && !s->data_path) {
            s->data_path = true;
            c6_send_init(s);
            c6_send_rpc(s, true, RPC_EVENT_ESP_INIT, 0, NULL, 0);
        }
        break;
    default:
        break;
    }
    c6_update_irq(s);
}

static uint8_t fn_read(Esp32c6HostedState *s, unsigned fn, uint32_t addr)
{
    if (fn == 0) {
        return fn0_read(s, addr);
    }
    if (addr < FN1_REG_END) {
        return (addr & 3 ? fn1_reg_value(s, addr & ~3u)
                         : fn1_reg(s, addr)) >> ((addr & 3) * 8);
    }
    if (addr < FN1_FIFO_END) {
        return fifo_pop(s);
    }
    return 0;
}

static void fn_write(Esp32c6HostedState *s, unsigned fn, uint32_t addr,
                     uint8_t v)
{
    if (fn == 0) {
        fn0_write(s, addr, v);
        return;
    }
    if (addr < FN1_REG_END) {
        fn1_reg_write(s, addr, v);
        return;
    }
    if (addr >= FN1_FIFO_END) {
        return;
    }
    if (s->rx_len < sizeof(s->rx)) {
        s->rx[s->rx_len++] = v;
    } else {
        s->rx_overflow = true;
    }
    if (addr == FN1_FIFO_END - 1) {
        c6_host_packet(s);
    }
}

/* ---- The card's SD interface ---- */

static Esp32c6HostedState *card_c6(SDState *sd)
{
    return ESP32C6_HOSTED(OBJECT(sd)->parent);
}

static size_t card_resp32(uint8_t *resp, size_t respsz, uint32_t v)
{
    assert(respsz >= 4);
    stl_be_p(resp, v);
    return 4;
}

static uint32_t r5_state(Esp32c6HostedState *s)
{
    return s->card_state == CARD_TRN ? R5_STATE_TRN :
           s->card_state == CARD_CMD ? R5_STATE_CMD : 0;
}

static uint32_t card_cmd52(Esp32c6HostedState *s, uint32_t arg)
{
    unsigned fn = IO_FN(arg);
    uint32_t addr = IO_ADDR(arg);
    uint8_t data = arg & 0xff;
    uint32_t state = r5_state(s);

    if (fn > 1) {
        return state | R5_FUNCTION_NUMBER;
    }
    if (arg & IO_RW) {
        fn_write(s, fn, addr, data);
        if (arg & IO_RAW) {
            data = fn_read(s, fn, addr);
        }
    } else {
        data = fn_read(s, fn, addr);
    }
    return state | data;
}

static uint32_t card_cmd53(Esp32c6HostedState *s, uint32_t arg)
{
    unsigned fn = IO_FN(arg);
    uint32_t count = arg & 0x1ff;
    uint32_t state = r5_state(s);

    if (fn > 1) {
        return state | R5_FUNCTION_NUMBER;
    }
    s->xfer_endless = false;
    if (arg & IO_BLOCK) {
        if (s->blksize[fn] == 0 || s->blksize[fn] > CIS_MAX_BLOCK) {
            qemu_log_mask(LOG_GUEST_ERROR, "esp32c6-hosted: block mode "
                          "CMD53 with function %u's block size %u\n", fn,
                          s->blksize[fn]);
            return state | R5_ERROR;
        }
        s->xfer_endless = count == 0;
        s->xfer_left = count * s->blksize[fn];
    } else {
        s->xfer_left = count ? count : 512;
    }
    s->xfer_active = true;
    s->xfer_write = arg & IO_RW;
    s->xfer_inc = arg & IO_OP_INC;
    s->xfer_fn = fn;
    s->xfer_addr = IO_ADDR(arg);
    s->card_state = CARD_TRN;
    trace_esp32c6_hosted_xfer(s->xfer_write, fn, s->xfer_addr, s->xfer_left,
                              arg & IO_BLOCK);
    return state;
}

static void card_xfer_advance(Esp32c6HostedState *s)
{
    if (s->xfer_inc) {
        s->xfer_addr = (s->xfer_addr + 1) & 0x1ffff;
    }
    if (!s->xfer_endless && --s->xfer_left == 0) {
        s->xfer_active = false;
        s->card_state = CARD_CMD;
    }
}

/* [spec:nuos:req:emu.esp32p4.c6] */
/*
 * CMD5, CMD3 and CMD7 bring the card up and CMD52 and CMD53 reach its
 * functions; as an I/O-only card it answers no memory card command. Held
 * in reset by CHIP_PU it answers nothing.
 */
static size_t card_do_command(SDState *sd, SDRequest *req, uint8_t *resp,
                              size_t respsz)
{
    Esp32c6HostedState *s = card_c6(sd);
    size_t n = 0;

    if (!c6_running(s) || s->card_state == CARD_INACTIVE) {
        return 0;
    }
    switch (req->cmd) {
    case 5:
        if (s->card_state != CARD_INIT && s->card_state != CARD_READY) {
            break;
        }
        if ((req->arg & SDIO_OCR) == 0 && (req->arg & 0xffffff)) {
            s->card_state = CARD_INACTIVE;
            break;
        }
        if (req->arg & 0xffffff) {
            s->card_state = CARD_READY;
        }
        n = card_resp32(resp, respsz, R4_READY | (1u << R4_NF_SHIFT) |
                        SDIO_OCR);
        break;
    case 3:
        if (s->card_state != CARD_READY && s->card_state != CARD_STBY) {
            break;
        }
        s->rca = s->rca + 1 ? s->rca + 1 : 1;
        s->card_state = CARD_STBY;
        n = card_resp32(resp, respsz, ((uint32_t)s->rca << 16) | (3 << 9));
        break;
    case 7:
        if (s->card_state != CARD_STBY && s->card_state != CARD_CMD) {
            break;
        }
        if ((req->arg >> 16) == s->rca) {
            n = card_resp32(resp, respsz, (s->card_state == CARD_STBY ?
                                           3 : 4) << 9 | READY_FOR_DATA);
            s->card_state = CARD_CMD;
        } else {
            s->card_state = CARD_STBY;
        }
        break;
    case 15:
        if ((req->arg >> 16) == s->rca) {
            s->card_state = CARD_INACTIVE;
        }
        break;
    case 52:
        if (s->card_state == CARD_CMD || s->card_state == CARD_TRN) {
            n = card_resp32(resp, respsz, card_cmd52(s, req->arg));
        }
        break;
    case 53:
        if (s->card_state == CARD_CMD) {
            n = card_resp32(resp, respsz, card_cmd53(s, req->arg));
        }
        break;
    default:
        break;
    }
    trace_esp32c6_hosted_cmd(req->cmd, req->arg, s->card_state,
                             n ? ldl_be_p(resp) : 0);
    c6_update_irq(s);
    return n;
}

static size_t card_read_data(SDState *sd, void *buf, size_t length)
{
    Esp32c6HostedState *s = card_c6(sd);
    uint8_t *p = buf;

    for (size_t i = 0; i < length; i++) {
        if (!s->xfer_active || s->xfer_write) {
            qemu_log_mask(LOG_GUEST_ERROR, "esp32c6-hosted: host reads data "
                          "the card is not sending\n");
            memset(p + i, 0, length - i);
            break;
        }
        p[i] = fn_read(s, s->xfer_fn, s->xfer_addr);
        card_xfer_advance(s);
    }
    c6_update_irq(s);
    /* Room freed in the queue may take frames the network held back. */
    if (s->associated) {
        qemu_flush_queued_packets(qemu_get_queue(s->nic));
    }
    return length;
}

static size_t card_write_data(SDState *sd, const void *buf, size_t length)
{
    Esp32c6HostedState *s = card_c6(sd);
    const uint8_t *p = buf;

    for (size_t i = 0; i < length; i++) {
        if (!s->xfer_active || !s->xfer_write) {
            qemu_log_mask(LOG_GUEST_ERROR, "esp32c6-hosted: host writes data "
                          "the card is not receiving\n");
            break;
        }
        fn_write(s, s->xfer_fn, s->xfer_addr, p[i]);
        card_xfer_advance(s);
    }
    c6_update_irq(s);
    return length;
}

static bool card_data_ready(SDState *sd)
{
    Esp32c6HostedState *s = card_c6(sd);

    return s->xfer_active && !s->xfer_write;
}

static bool card_receive_ready(SDState *sd)
{
    Esp32c6HostedState *s = card_c6(sd);

    return s->xfer_active && s->xfer_write;
}

static bool card_get_inserted(SDState *sd)
{
    return true;
}

static bool card_get_readonly(SDState *sd)
{
    return false;
}

static void card_set_voltage(SDState *sd, uint16_t millivolts)
{
}

/* The card pulls DAT1 low to interrupt the host. */
static uint8_t card_get_dat_lines(SDState *sd)
{
    return card_c6(sd)->card_irq ? 0xd : 0xf;
}

static bool card_get_cmd_line(SDState *sd)
{
    return true;
}

static void card_realize(DeviceState *dev, Error **errp)
{
    if (!object_dynamic_cast(OBJECT(dev)->parent, TYPE_ESP32C6_HOSTED)) {
        error_setg(errp, "%s: only an %s can create its card",
                   TYPE_ESP32C6_HOSTED_CARD, TYPE_ESP32C6_HOSTED);
    }
}

/* The card's state is the C6's, which CHIP_PU resets. */
static void card_reset(DeviceState *dev)
{
}

static void card_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SDCardClass *sc = SDMMC_COMMON_CLASS(klass);

    dc->desc = "ESP32-C6 esp-hosted SDIO card";
    dc->realize = card_realize;
    dc->vmsd = NULL;
    dc->user_creatable = false;
    device_class_set_legacy_reset(dc, card_reset);

    sc->do_command = card_do_command;
    sc->read_data = card_read_data;
    sc->write_data = card_write_data;
    sc->data_ready = card_data_ready;
    sc->receive_ready = card_receive_ready;
    sc->get_inserted = card_get_inserted;
    sc->get_readonly = card_get_readonly;
    sc->set_voltage = card_set_voltage;
    sc->get_dat_lines = card_get_dat_lines;
    sc->get_cmd_line = card_get_cmd_line;
    sc->set_cid = NULL;
    sc->set_csd = NULL;
    sc->proto = NULL;
}

void esp32c6_hosted_attach(Esp32c6HostedState *s, SDBus *bus)
{
    Object *card = object_new(TYPE_ESP32C6_HOSTED_CARD);

    assert(!s->card);
    object_property_add_child(OBJECT(s), "card", card);
    object_unref(card);
    s->card = DEVICE(card);
    s->bus = bus;
    qdev_realize(s->card, BUS(bus), &error_fatal);
    sdbus_set_irq(bus, s->card_irq);
}

/* ---- The device ---- */

/* The C6 as CHIP_PU's rising edge leaves it: booted, its link closed. */
static void c6_power_on(Esp32c6HostedState *s)
{
    card_reset_state(s);
    timer_del(&s->radio_timer);
    s->radio_pending = RADIO_IDLE;
    s->data_path = false;
    s->int_raw = 0;
    s->int_ena = 0;
    s->token = ESP32C6_HOSTED_RX_BUFS;
    s->rx_total = 0;
    s->fifo_head = 0;
    s->fifo_len = 0;
    s->rx_len = 0;
    s->rx_overflow = false;
    s->serial_seq = 0;
    s->message_len = 0;
    s->message_overflow = false;
    s->started = false;
    s->associated = false;
    s->sta_ssid_len = 0;
    memset(s->sta_password, 0, sizeof(s->sta_password));
    s->sta_password_len = 0;
    s->sta_threshold = 0;
}

static void c6_chip_pu(void *opaque, int n, int level)
{
    Esp32c6HostedState *s = opaque;
    bool on = level;

    if (on == s->chip_pu) {
        return;
    }
    s->chip_pu = on;
    trace_esp32c6_hosted_chip_pu(on);
    c6_power_on(s);
    if (!on) {
        s->card_state = CARD_INACTIVE;
    }
    c6_update_irq(s);
}

static void esp32c6_hosted_reset_hold(Object *obj, ResetType type)
{
    Esp32c6HostedState *s = ESP32C6_HOSTED(obj);

    c6_power_on(s);
    c6_update_irq(s);
}

static void esp32c6_hosted_init(Object *obj)
{
    Esp32c6HostedState *s = ESP32C6_HOSTED(obj);

    qdev_init_gpio_in_named(DEVICE(obj), c6_chip_pu, ESP32C6_HOSTED_CHIP_PU,
                            1);
    timer_init_ns(&s->radio_timer, QEMU_CLOCK_VIRTUAL, c6_radio_cb, s);
    s->chip_pu = true;
    /* The access point a test image's saved network expects */
    s->ap_ssid = g_strdup("nuos-emulator");
    s->ap_password = g_strdup("nuos-emulator");
}

static void esp32c6_hosted_realize(DeviceState *dev, Error **errp)
{
    Esp32c6HostedState *s = ESP32C6_HOSTED(dev);

    if (!s->ap_ssid || !*s->ap_ssid || strlen(s->ap_ssid) > 32) {
        error_setg(errp, "%s: the access point's SSID must be 1 to 32 bytes",
                   TYPE_ESP32C6_HOSTED);
        return;
    }
    if (!s->ap_password || strlen(s->ap_password) > 64) {
        error_setg(errp, "%s: the access point's password must be at most "
                   "64 bytes", TYPE_ESP32C6_HOSTED);
        return;
    }
    if (s->ap_channel < 1 || s->ap_channel > 14 || s->ap_rssi > 0 ||
        s->ap_rssi < -127) {
        error_setg(errp, "%s: the channel must be 1 to 14 and the RSSI "
                   "-127 to 0", TYPE_ESP32C6_HOSTED);
        return;
    }
    qemu_macaddr_default_if_unset(&s->conf.macaddr);
    s->nic = qemu_new_nic(&net_esp32c6_hosted_info, &s->conf,
                          object_get_typename(OBJECT(dev)), dev->id,
                          &dev->mem_reentrancy_guard, s);
    qemu_format_nic_info_str(qemu_get_queue(s->nic), s->conf.macaddr.a);
    qemu_register_resettable(OBJECT(dev));
    c6_power_on(s);
}

static void esp32c6_hosted_unrealize(DeviceState *dev)
{
    Esp32c6HostedState *s = ESP32C6_HOSTED(dev);

    timer_del(&s->radio_timer);
    qemu_unregister_resettable(OBJECT(dev));
    qemu_del_nic(s->nic);
}

static const Property esp32c6_hosted_properties[] = {
    DEFINE_NIC_PROPERTIES(Esp32c6HostedState, conf),
    DEFINE_PROP_STRING("ssid", Esp32c6HostedState, ap_ssid),
    DEFINE_PROP_STRING("password", Esp32c6HostedState, ap_password),
    /* esp_wifi's authmode: WPA2-PSK 3, WPA3-PSK 6 */
    DEFINE_PROP_UINT8("authmode", Esp32c6HostedState, ap_authmode, 3),
    DEFINE_PROP_UINT8("channel", Esp32c6HostedState, ap_channel, 6),
    DEFINE_PROP_INT32("rssi", Esp32c6HostedState, ap_rssi, -40),
};

static const VMStateDescription vmstate_esp32c6_hosted = {
    .name = TYPE_ESP32C6_HOSTED,
    .unmigratable = 1,
};

static void esp32c6_hosted_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->desc = "ESP32-C6 Wi-Fi co-processor (esp-hosted-mcu, SDIO)";
    dc->realize = esp32c6_hosted_realize;
    dc->unrealize = esp32c6_hosted_unrealize;
    dc->vmsd = &vmstate_esp32c6_hosted;
    dc->user_creatable = false;
    rc->phases.hold = esp32c6_hosted_reset_hold;
    device_class_set_props(dc, esp32c6_hosted_properties);
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}

static const TypeInfo esp32c6_hosted_types[] = {
    {
        .name = TYPE_ESP32C6_HOSTED,
        .parent = TYPE_DEVICE,
        .instance_size = sizeof(Esp32c6HostedState),
        .instance_init = esp32c6_hosted_init,
        .class_init = esp32c6_hosted_class_init,
    },
    {
        .name = TYPE_ESP32C6_HOSTED_CARD,
        .parent = TYPE_SDMMC_COMMON,
        .class_init = card_class_init,
    },
};

DEFINE_TYPES(esp32c6_hosted_types)
