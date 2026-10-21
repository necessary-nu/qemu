/*
 * ESP32-C6 Wi-Fi co-processor running esp-hosted-mcu, at its SDIO host
 * interface
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_SD_ESP32C6_HOSTED_H
#define HW_SD_ESP32C6_HOSTED_H

#include "hw/core/qdev.h"
#include "hw/sd/sd.h"
#include "net/net.h"
#include "qemu/timer.h"

#define TYPE_ESP32C6_HOSTED "esp32c6-hosted"
OBJECT_DECLARE_SIMPLE_TYPE(Esp32c6HostedState, ESP32C6_HOSTED)

/* The SDIO card the co-processor presents on an SD bus, its child */
#define TYPE_ESP32C6_HOSTED_CARD "esp32c6-hosted-card"

/* CHIP_PU (input): low holds the C6 in reset; high, it runs */
#define ESP32C6_HOSTED_CHIP_PU "chip-pu"

/* esp-hosted's transport packet, header included */
#define ESP32C6_HOSTED_PACKET   1536
/* The receive buffers the C6 advertises for the host's packets */
#define ESP32C6_HOSTED_RX_BUFS  20
/* The packets it queues for the host */
#define ESP32C6_HOSTED_TX_QUEUE 20
/* The largest serial (RPC) message, fragments reassembled */
#define ESP32C6_HOSTED_MESSAGE  8192

struct Esp32c6HostedState {
    DeviceState parent_obj;

    DeviceState *card;
    SDBus *bus;
    NICState *nic;
    NICConf conf;
    QEMUTimer radio_timer;

    /* Properties: the one access point the radio finds */
    char *ap_ssid;
    char *ap_password;
    uint8_t ap_authmode;
    uint8_t ap_channel;
    int32_t ap_rssi;

    bool chip_pu;

    int card_state;
    uint16_t rca;
    uint8_t ioe;
    uint8_t ien;
    uint8_t bus_if;
    uint16_t blksize[2];
    bool xfer_active;
    bool xfer_write;
    bool xfer_inc;
    bool xfer_endless;
    uint8_t xfer_fn;
    uint32_t xfer_addr;
    uint32_t xfer_left;
    bool card_irq;

    /* esp-hosted's SDIO slave, as the host sees it */
    bool data_path;
    uint32_t int_raw;
    uint32_t int_ena;
    uint32_t token;
    uint32_t rx_total;
    uint8_t fifo[ESP32C6_HOSTED_TX_QUEUE * ESP32C6_HOSTED_PACKET];
    uint32_t fifo_head;
    uint32_t fifo_len;
    uint8_t rx[ESP32C6_HOSTED_PACKET];
    uint32_t rx_len;
    bool rx_overflow;
    uint16_t serial_seq;
    uint8_t message[ESP32C6_HOSTED_MESSAGE];
    uint32_t message_len;
    bool message_overflow;

    /* The station */
    bool started;
    bool associated;
    int radio_pending;
    uint8_t sta_ssid[32];
    uint8_t sta_ssid_len;
    uint8_t sta_password[64];
    uint8_t sta_password_len;
    uint8_t sta_threshold;
    uint64_t dropped;
};

/* Create the C6's card on @bus, as a board joins it to its host's slot. */
void esp32c6_hosted_attach(Esp32c6HostedState *s, SDBus *bus);

#endif
