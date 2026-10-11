/*
 * The Synopsys DesignWare GMAC of Espressif's Ethernet MACs
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_NET_ESP32_DWMAC_H
#define HW_NET_ESP32_DWMAC_H

#include "hw/core/sysbus.h"
#include "hw/core/clock.h"
#include "hw/net/ip101_phy.h"
#include "net/net.h"
#include "qemu/timer.h"
#include "system/memory.h"

/*
 * The GMAC core the ESP32 and ESP32-P4 EMACs share: its DMA, MTL and MAC
 * register sets, the descriptor engines, the MAC's transmit and receive
 * paths, MDIO, the IEEE 1588 system time, and the link to a QEMU network
 * backend through an IP101 PHY. It is abstract: each chip's EMAC is a
 * subclass that places the register sets in its own address map, clocks
 * them, and says how the DMA reaches memory and the MII reaches the PHY,
 * through the hooks in Esp32DwmacClass.
 *
 * Register offsets below are the core's own: the MAC's at 0x0000 and the
 * DMA's at 0x1000, the DesignWare layout, which the ESP32-P4 keeps and
 * the ESP32 rearranges.
 */
#define TYPE_ESP32_DWMAC "esp32-dwmac"
OBJECT_DECLARE_TYPE(Esp32DwmacState, Esp32DwmacClass, ESP32_DWMAC)

#define ESP32_DWMAC_MAC_BASE        0x0000
#define ESP32_DWMAC_DMA_BASE        0x1000
#define ESP32_DWMAC_BLOCK_SIZE      0x1000

/* The most MAC address registers a chip has (the ESP32-P4's nine) */
#define ESP32_DWMAC_ADDR_MAX        9
#define ESP32_DWMAC_RWUFFR_COUNT    8
/* The largest MTL receive FIFO, and the frames it can hold */
#define ESP32_DWMAC_RX_FIFO_MAX     2048
#define ESP32_DWMAC_RX_FIFO_FRAMES  (ESP32_DWMAC_RX_FIFO_MAX / 64)
/* Longest frame the MAC handles with its watchdog and jabber disabled */
#define ESP32_DWMAC_MAX_FRAME       16383
/* Room for a gathered frame plus inserted address or tag, padding, FCS */
#define ESP32_DWMAC_TX_BUF_SIZE     (ESP32_DWMAC_MAX_FRAME + 64)

/*
 * The sysbus IRQs: the main interrupt (sbd_intr_o, the chips' ETH_MAC_INTR
 * source), the power management interrupt (pmt_intr_o, PMT_INTR) and the
 * LPI interrupt (lpi_intr_o, LPI_INTR).
 */
enum {
    ESP32_DWMAC_IRQ_SBD,
    ESP32_DWMAC_IRQ_PMT,
    ESP32_DWMAC_IRQ_LPI,
    ESP32_DWMAC_IRQS,
};

/*
 * Named GPIO input ESP32_DWMAC_SBD_FLOWCTRL: the sideband flow control
 * request (sbd_flowctrl_i), high to have the MAC send pause frames.
 */
#define ESP32_DWMAC_SBD_FLOWCTRL    "sbd-flowctrl"

/*
 * Named GPIO output ESP32_DWMAC_PTP_PPS: the pulse-per-second output
 * (ptp_pps_o) the system time drives, for chips with the timestamp engine.
 */
#define ESP32_DWMAC_PTP_PPS         "ptp-pps"

/*
 * The path a management frame takes, as a subclass's mdio_path hook
 * reports it: whether the frame reaches the PHY's MDC and MDIO pins,
 * whether the MAC's MDIO input listens to the PHY's MDIO pin, and the
 * level that input sees when nothing drives it.
 */
typedef struct Esp32DwmacMdioPath {
    bool to_phy;
    bool from_phy;
    bool idle_level;
} Esp32DwmacMdioPath;

struct Esp32DwmacClass {
    SysBusDeviceClass parent_class;

    /* The prefix of the block's log messages */
    const char *log_name;
    /* The name of the clock input the registers and DMA run on */
    const char *bus_clock;
    /* The number of MAC address registers */
    unsigned addr_count;
    /* The MTL FIFOs' sizes in bytes */
    unsigned rx_fifo_size;
    unsigned tx_fifo_size;
    /*
     * DMABUSMODE's, EMACCONFIG's and EMACFF's writable bits and reset
     * values. A DMABUSMODE reset value with SW_RST set is a core that
     * leaves reset only once its MII clocks run.
     */
    uint32_t bus_mode_rw;
    uint32_t bus_mode_reset;
    uint32_t config_rw;
    uint32_t config_reset;
    uint32_t ff_rw;
    uint32_t ff_reset;
    /* EMACINTMASK's and EMACLPI_CSR's writable bits */
    uint32_t int_mask_rw;
    uint32_t lpi_rw;
    /* EMACCONFIG bits that decide which MII clocks reach the MAC */
    uint32_t config_clock_bits;
    /* DMAAHBSTATUS, the AHB master's state */
    bool has_ahb_status;
    /*
     * The IEEE 1588 advanced timestamp engine, with the system time
     * registers, TDES6/7 and RDES6/7 timestamps and RDES4's PTP status.
     */
    bool has_ptp;
    /* The receive VLAN tag register and transmit VLAN tag control */
    bool has_vlan;

    /*
     * Whether the MII's transmit and receive clocks reach the MAC. A
     * software reset finishes once both run.
     */
    bool (*tx_clk)(Esp32DwmacState *s);
    bool (*rx_clk)(Esp32DwmacState *s);
    /*
     * Whether the DMA may access [addr, addr + len) at all; NULL for a
     * bus that answers every access itself. An access the bus refuses
     * either way is a fatal bus error.
     */
    bool (*dma_ok)(Esp32DwmacState *s, uint32_t addr, uint32_t len);
    /* The attributes of the DMA's accesses; NULL for unspecified ones */
    MemTxAttrs (*dma_attrs)(Esp32DwmacState *s);
    /*
     * The MII's routing to the PHY, NULL for a PHY wired straight to the
     * MAC: where a management frame goes (mdio_path), whether the
     * transmit and receive data paths reach the PHY (tx_path, rx_path),
     * and the MAC's MDIO output enable and transmit enable, as a frame
     * starts and ends (mdio_drive, tx_drive).
     */
    void (*mdio_path)(Esp32DwmacState *s, Esp32DwmacMdioPath *path);
    void (*mdio_drive)(Esp32DwmacState *s, bool on);
    bool (*tx_path)(Esp32DwmacState *s);
    bool (*rx_path)(Esp32DwmacState *s);
    void (*tx_drive)(Esp32DwmacState *s, bool on);
    /* The core's reset state changed: held, or a software reset pending */
    void (*reset_changed)(Esp32DwmacState *s);
};

struct Esp32DwmacState {
    SysBusDevice parent_obj;

    qemu_irq irq[ESP32_DWMAC_IRQS];
    qemu_irq pps_out;
    Clock *bus_clk;
    /* The PTP reference clock, which the subclass provides */
    Clock *ptp_clk;
    NICState *nic;
    NICConf conf;
    IP101PhyState *phy;

    AddressSpace *dma_as;

    /* Held in reset by the chip's reset line, if the subclass takes it */
    bool held;

    /* DMA */
    uint32_t bus_mode;
    uint32_t rx_base;
    uint32_t tx_base;
    uint32_t status;
    uint32_t op_mode;
    uint32_t int_en;
    uint32_t missed_frames;
    uint32_t overflow_frames;
    bool missed_overflow;
    bool fifo_overflow;
    uint32_t rx_wdt;
    uint32_t tx_desc;
    uint32_t rx_desc;
    uint32_t tx_buf;
    uint32_t rx_buf;
    uint8_t tx_state;
    uint8_t rx_state;
    /* An engine that took a bus error stays off the bus until reset */
    bool tx_fatal;
    bool rx_fatal;

    /* MAC */
    uint32_t config;
    uint32_t frame_filter;
    uint32_t mii_addr;
    uint32_t mii_data;
    uint32_t flow_ctrl;
    uint32_t vlan_tag;
    uint32_t rwuffr[ESP32_DWMAC_RWUFFR_COUNT];
    uint32_t rwuffr_ptr;
    uint32_t pmt_csr;
    uint32_t lpi_csr;
    uint32_t lpi_timers;
    uint32_t int_mask;
    uint32_t addr_high[ESP32_DWMAC_ADDR_MAX];
    uint32_t addr_low[ESP32_DWMAC_ADDR_MAX];
    uint32_t wdog_to;
    uint32_t tx_vlan_ctrl;

    /* A software reset waits for the MII clocks */
    bool sw_reset_pending;

    /* MDIO: a management frame on the wire */
    QEMUTimer mdio_timer;

    /*
     * Transmit: the frame the DMA has gathered so far from its
     * descriptors, the first descriptor's control words, and the
     * descriptor holding its last segment. A complete frame waits in the
     * MTL FIFO (tx_ready) until the MAC can send it; then, rewritten in
     * place as the wire's tx_wire_len bytes, it occupies the wire
     * (tx_timer) until it reaches the receiver and the DMA closes its
     * descriptor. tx_ts is the system time its SFD left at.
     */
    uint8_t tx_frame[ESP32_DWMAC_TX_BUF_SIZE];
    uint32_t tx_len;
    uint32_t tx_wire_len;
    bool tx_gathering;
    bool tx_ready;
    bool tx_on_wire;
    bool tx_jabber;
    uint32_t tx_tdes0;
    uint32_t tx_tdes1;
    uint32_t tx_last_desc;
    uint32_t tx_status;
    uint64_t tx_ts;
    QEMUTimer tx_timer;
    /*
     * Pause frames: one the MAC owes the wire and its pause time, the one
     * on it, and the flow control request (FCB or the sideband input)
     * that has the MAC repeat them before the pause runs out (pause_
     * refresh_timer) and end the pause with a zero-quanta one.
     */
    bool pause_pending;
    uint16_t pause_quanta;
    bool pause_on_wire;
    bool sbd_flowctrl;
    bool fc_active;
    QEMUTimer pause_refresh_timer;
    /* The transmitter is paused by a received pause frame */
    QEMUTimer pause_timer;
    bool paused;

    /* Receive: frames in the MTL FIFO with their status words */
    uint8_t rx_fifo[ESP32_DWMAC_RX_FIFO_MAX];
    uint32_t rx_fifo_used;
    uint32_t rx_fifo_count;
    uint16_t rx_fifo_len[ESP32_DWMAC_RX_FIFO_FRAMES];
    uint32_t rx_fifo_rdes0[ESP32_DWMAC_RX_FIFO_FRAMES];
    uint32_t rx_fifo_rdes4[ESP32_DWMAC_RX_FIFO_FRAMES];
    uint64_t rx_fifo_ts[ESP32_DWMAC_RX_FIFO_FRAMES];
    /* RIWT: the receive interrupt held back by RDES1.DIC */
    QEMUTimer rx_wdt_timer;

    /*
     * IEEE 1588 system time. The time is kept as of an anchor: at
     * ptp_anchor_ns of QEMU_CLOCK_VIRTUAL it was ptp_sec (with the high
     * word ptp_sec_hi) and ptp_subsec, with ptp_acc in the fine update
     * accumulator, counting PTP reference clock cycles from there. The
     * anchor moves whenever the rate changes.
     */
    uint32_t ts_ctrl;
    uint32_t ts_ssinc;
    uint32_t ts_sec_upd;
    uint32_t ts_subsec_upd;
    uint32_t ts_addend;
    uint32_t ts_target_sec;
    uint32_t ts_target_subsec;
    uint32_t ts_status;
    uint32_t pps_ctrl;
    int64_t ptp_anchor_ns;
    uint32_t ptp_sec;
    uint16_t ptp_sec_hi;
    uint32_t ptp_subsec;
    uint32_t ptp_acc;
    /* The addend TSADDREG last loaded into the fine update */
    uint32_t ptp_addend;
    /* The next system time event: target time, overflow or PPS edge */
    QEMUTimer ptp_timer;
    bool pps_level;
};

/*
 * Read and write the register at core offset @off, the MAC's at 0x0000,
 * the DMA's at 0x1000; @addr is the guest's offset, for log messages.
 * Reads and writes the core does not decode are logged as guest errors.
 */
uint32_t esp32_dwmac_read(Esp32DwmacState *s, hwaddr off, hwaddr addr);
void esp32_dwmac_write(Esp32DwmacState *s, hwaddr off, uint32_t v,
                       hwaddr addr);

/*
 * The subclass's view of the MII clocks or the PHY's wiring changed:
 * finish a software reset waiting for them and restart what waits on
 * them.
 */
void esp32_dwmac_mii_changed(Esp32DwmacState *s);

/* Hold the core in reset, or release it, from the chip's reset line. */
void esp32_dwmac_set_held(Esp32DwmacState *s, bool held);

/* Whether a software reset is still waiting for the MII clocks. */
bool esp32_dwmac_sw_reset_pending(Esp32DwmacState *s);

/*
 * The system time now, as the PTP block outputs it (ptp_timestamp_o):
 * seconds in the high 32 bits, sub-seconds in the low.
 */
uint64_t esp32_dwmac_ptp_time(Esp32DwmacState *s);

extern const VMStateDescription vmstate_esp32_dwmac;

#endif
