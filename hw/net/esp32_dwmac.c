/*
 * The Synopsys DesignWare GMAC of Espressif's Ethernet MACs
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: the ESP32 TRM and the ESP32-P4 TRM (v3.x silicon), each in
 * its chapter "Ethernet Media Access Controller (EMAC)", with the field
 * layout of ESP-IDF's emac_dma_struct.h, emac_mac_struct.h and, for the
 * ESP32-P4, emac_ptp_struct.h and the v3 emac_reg.h, and the DesignWare
 * GMAC behaviour they describe. Both chips carry the same GMAC; they
 * differ in where its registers sit, its FIFO sizes, its clocking, how
 * its DMA reaches memory and its MII the PHY, and the ESP32-P4 has the
 * IEEE 1588 advanced timestamp engine and VLAN tag control. Those are
 * the subclasses' (esp32_emac.c, esp32p4_emac.c).
 *
 * The DMA moves frames between descriptor lists in memory and the MTL
 * FIFOs; the MAC moves them between the FIFOs and the PHY over MII or
 * RMII. The descriptors are the GMAC's enhanced ones, 16 bytes, or 32
 * with DMABUSMODE.ALT_DESC_SIZE, which adds RDES4's extended status and
 * the TDES6/7 and RDES6/7 timestamps; lists are rings (contiguous, with
 * DESC_SKIP_LEN words between entries, and an end-of-ring bit) or chains
 * (each descriptor's third address word points at the next).
 *
 * The PHY hangs off the MAC's MDIO bus, and the network backend is its
 * medium: transmitted frames leave through the PHY at the line rate set
 * by EMACCONFIG.FES, and the backend's frames arrive padded to the
 * minimum size, with their FCS. Frames carry their FCS inside the model,
 * so loopback, CRC errors and FCS stripping act on the wire's bytes.
 *
 * The MAC's transmit and receive paths run on the MII clocks the chip
 * gives them. Without them frames do not move, and a software reset,
 * which must reach every clock domain, does not finish.
 *
 * The system time (chips with the timestamp engine) counts the PTP
 * reference clock: every cycle in coarse update mode, or every carry out
 * of the 32-bit accumulator the addend is added to in fine mode, adds
 * SSINC to the sub-seconds, which roll over into the seconds at 2^31
 * (binary rollover) or 10^9 (digital rollover). Commands that change it
 * (TSINIT, TSUPDT, TSADDREG) take effect on the PTP clock, so they wait,
 * their bits still set, while it is stopped.
 *
 * Interpretations where the TRMs are silent or disagree:
 *
 *   - DMASTATUS.ERROR_BITS: the ESP32 TRM gives the GMAC's 3-bit code
 *     (bit 23 transmit, bit 24 read, bit 25 descriptor); the ESP32-P4 TRM
 *     numbers the same six cases 0 to 5. Both chips get the GMAC's code.
 *   - With the advanced timestamp engine RDES0 bit 7 says a timestamp was
 *     written, so a giant frame is reported only by ES, which the MAC's
 *     status word still sets; RDES0 bit 0 always means that RDES4 holds
 *     extended status.
 *   - TSUPDT subtracts the update registers' time, as ESP-IDF programs
 *     them, rather than adding a complement.
 *   - In store-and-forward mode a frame longer than the transmit FIFO
 *     starts once the FIFO is full, as in threshold mode.
 *   - A frame whose source address or VLAN tag the MAC changes gets a
 *     new FCS: appended, or with TDES0.DC in place of the last four
 *     bytes, as both TRMs say.
 *
 * Not modelled: VLAN tag insertion on the ESP32 (its TRM documents no tag
 * register), remote wake-up frame filters, the energy-efficient Ethernet
 * LPI states, collisions and the half-duplex backoff, automatic pause
 * frames when the receive FIFO fills, and the flexible PPS output modes.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/host-utils.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/gpio/esp32_line.h"
#include "hw/net/esp32_dwmac.h"
#include "migration/vmstate.h"
#include "net/checksum.h"
#include "net/eth.h"
#include <zlib.h>

/* MAC */
#define A_CONFIG            0x0000
#define A_FRAME_FILTER      0x0004
#define A_MII_ADDR          0x0010
#define A_MII_DATA          0x0014
#define A_FLOW_CTRL         0x0018
#define A_VLAN_TAG          0x001c
#define A_DEBUG             0x0024
#define A_RWUFFR            0x0028
#define A_PMT_CSR           0x002c
#define A_LPI_CSR           0x0030
#define A_LPI_TIMERS        0x0034
#define A_INTS              0x0038
#define A_INT_MASK          0x003c
#define A_ADDR0_HIGH        0x0040
#define A_CSTATUS           0x00d8
#define A_WDOG_TO           0x00dc
#define A_TX_VLAN           0x0584
#define A_TS_CTRL           0x0700
#define A_TS_SSINC          0x0704
#define A_TS_SEC            0x0708
#define A_TS_SUBSEC         0x070c
#define A_TS_SEC_UPD        0x0710
#define A_TS_SUBSEC_UPD     0x0714
#define A_TS_ADDEND         0x0718
#define A_TS_TARGET_SEC     0x071c
#define A_TS_TARGET_SUBSEC  0x0720
#define A_TS_SEC_HI         0x0724
#define A_TS_STATUS         0x0728
#define A_PPS_CTRL          0x072c

/* DMA */
#define A_BUS_MODE          0x1000
#define A_TX_POLL           0x1004
#define A_RX_POLL           0x1008
#define A_RX_BASE           0x100c
#define A_TX_BASE           0x1010
#define A_STATUS            0x1014
#define A_OP_MODE           0x1018
#define A_INT_EN            0x101c
#define A_MISSED            0x1020
#define A_RX_WDT            0x1024
#define A_AHB_STATUS        0x102c
#define A_TX_CUR_DESC       0x1048
#define A_RX_CUR_DESC       0x104c
#define A_TX_CUR_BUF        0x1050
#define A_RX_CUR_BUF        0x1054

/* DMABUSMODE */
#define BUS_MODE_SWR        (1u << 0)
#define BUS_MODE_DSL_SHIFT  2
#define BUS_MODE_DSL_MASK   0x1f
#define BUS_MODE_ATDS       (1u << 7)

/* DMASTATUS */
#define ST_TI               (1u << 0)
#define ST_TPS              (1u << 1)
#define ST_TU               (1u << 2)
#define ST_TJT              (1u << 3)
#define ST_OVF              (1u << 4)
#define ST_UNF              (1u << 5)
#define ST_RI               (1u << 6)
#define ST_RU               (1u << 7)
#define ST_RPS              (1u << 8)
#define ST_RWT              (1u << 9)
#define ST_ETI              (1u << 10)
#define ST_FBI              (1u << 13)
#define ST_ERI              (1u << 14)
#define ST_AIS              (1u << 15)
#define ST_NIS              (1u << 16)
#define ST_RS_SHIFT         17
#define ST_TS_SHIFT         20
#define ST_EB_SHIFT         23
#define ST_PMT              (1u << 28)
#define ST_TTI              (1u << 29)
#define ST_LPI              (1u << 30)
#define ST_W1C              0x0001e7ffu
#define ST_NIS_BITS         (ST_TI | ST_TU | ST_RI | ST_ERI)
#define ST_AIS_BITS         (ST_TPS | ST_TJT | ST_OVF | ST_UNF | ST_RU | \
                             ST_RPS | ST_RWT | ST_ETI | ST_FBI)

/* DMAIN_EN */
#define IE_AIE              (1u << 15)
#define IE_NIE              (1u << 16)
#define IE_RW               0x0001e7ffu

/* DMAOPERATION_MODE */
#define OP_SR               (1u << 1)
#define OP_DGF              (1u << 5)
#define OP_FUF              (1u << 6)
#define OP_FEF              (1u << 7)
#define OP_ST               (1u << 13)
#define OP_TTC_SHIFT        14
#define OP_FTF              (1u << 20)
#define OP_TSF              (1u << 21)
#define OP_DFF              (1u << 24)
#define OP_RSF              (1u << 25)
#define OP_DT               (1u << 26)
#define OP_RW               0x0731e0feu

/* DMAMISSEDFR */
#define MISSED_FC_MAX       0xffff
#define MISSED_BMFC         (1u << 16)
#define OVERFLOW_FC_SHIFT   17
#define OVERFLOW_FC_MAX     0x7ff
#define OVERFLOW_BFOC       (1u << 28)

/* DMA bus error codes, DMASTATUS.ERROR_BITS */
#define EB_RX_DATA          0
#define EB_TX_DATA          3
#define EB_RX_DESC_WRITE    4
#define EB_TX_DESC_WRITE    5
#define EB_RX_DESC_READ     6
#define EB_TX_DESC_READ     7

/*
 * Transmit process states. The ESP32 TRM's table repeats the receive
 * states here; the encoding is the DesignWare GMAC's.
 */
#define TS_STOPPED          0
#define TS_FETCHING         1
#define TS_WAITING          2
#define TS_SUSPENDED        6
#define TS_CLOSING          7

/* Receive process states */
#define RS_STOPPED          0
#define RS_WAITING          3
#define RS_SUSPENDED        4

/* EMACCONFIG */
#define CFG_PRELEN_MASK     0x3
#define CFG_RE              (1u << 2)
#define CFG_TE              (1u << 3)
#define CFG_ACS             (1u << 7)
#define CFG_IPC             (1u << 10)
#define CFG_DM              (1u << 11)
#define CFG_LM              (1u << 12)
#define CFG_DO              (1u << 13)
#define CFG_FES             (1u << 14)
#define CFG_DCRS            (1u << 16)
#define CFG_IFG_SHIFT       17
#define CFG_JE              (1u << 20)
#define CFG_JD              (1u << 22)
#define CFG_WD              (1u << 23)
#define CFG_CST             (1u << 25)
#define CFG_2KPE            (1u << 27)
#define CFG_SARC_SHIFT      28

/* EMACFF */
#define FF_PR               (1u << 0)
#define FF_DAIF             (1u << 3)
#define FF_PM               (1u << 4)
#define FF_DBF              (1u << 5)
#define FF_PCF_SHIFT        6
#define FF_SAIF             (1u << 8)
#define FF_SAF              (1u << 9)
#define FF_VTFE             (1u << 16)
#define FF_RA               (1u << 31)

/* EMACGMIIADDR */
#define MII_GB              (1u << 0)
#define MII_GW              (1u << 1)
#define MII_CR_SHIFT        2
#define MII_GR_SHIFT        6
#define MII_PA_SHIFT        11

/* EMACFC */
#define FC_FCB              (1u << 0)
#define FC_TFE              (1u << 1)
#define FC_RFE              (1u << 2)
#define FC_UP               (1u << 3)
#define FC_PLT_SHIFT        4
#define FC_DZPQ             (1u << 7)
#define FC_PT_SHIFT         16
#define FC_RW               0xffff00bfu

/* EMACVLANTAG */
#define VLAN_TAG_VL_MASK    0xffffu
#define VLAN_TAG_ETV        (1u << 16)
#define VLAN_TAG_VTIM       (1u << 17)
#define VLAN_TAG_ESVL       (1u << 18)
#define VLAN_TAG_RW         0x0007ffffu

/* EMACTXVLTCTRL */
#define TX_VLAN_VLT_MASK    0xffffu
#define TX_VLAN_VLC_SHIFT   16
#define TX_VLAN_VLP         (1u << 18)
#define TX_VLAN_CSVL        (1u << 19)
#define TX_VLAN_RW          0x000fffffu

/* PMT_CSR */
#define PMT_PWRDWN          (1u << 0)
#define PMT_MGKPKTEN        (1u << 1)
#define PMT_RWKPKTEN        (1u << 2)
#define PMT_MGKPRCVD        (1u << 5)
#define PMT_RWKPRCVD        (1u << 6)
#define PMT_GLBLUCAST       (1u << 9)
#define PMT_RWKPTR_SHIFT    24
#define PMT_RWKFILTRST      (1u << 31)
#define PMT_RW              0x00000207u

/* EMACLPI_CSR and EMACLPITIMERSCONTROL */
#define LPI_LPIEN           (1u << 16)
#define LPI_TIMERS_RW       0x03ffffffu
#define LPI_TIMERS_RESET    0x03e80000u

/* EMACINTS and EMACINTMASK */
#define INTS_PMT            (1u << 3)
#define INTS_TS             (1u << 9)
#define INTS_LPI            (1u << 10)

/* EMACADDRnHIGH */
#define ADDR_HIGH_MBC_SHIFT 24
#define ADDR_HIGH_SA        (1u << 30)
#define ADDR_HIGH_AE        (1u << 31)
#define ADDR0_HIGH_RW       0x0000ffffu
#define ADDRN_HIGH_RW       0xff00ffffu

#define WDOG_TO_PWE         (1u << 16)
#define WDOG_TO_RW          0x00013fffu

/* EMACTSTPCTRL */
#define TSC_TSENA           (1u << 0)
#define TSC_TSCFUPDT        (1u << 1)
#define TSC_TSINIT          (1u << 2)
#define TSC_TSUPDT          (1u << 3)
#define TSC_TSTRIG          (1u << 4)
#define TSC_TSADDREG        (1u << 5)
#define TSC_TSENALL         (1u << 8)
#define TSC_TSCTRLSSR       (1u << 9)
#define TSC_TSVER2ENA       (1u << 10)
#define TSC_TSIPENA         (1u << 11)
#define TSC_TSIPV6ENA       (1u << 12)
#define TSC_TSIPV4ENA       (1u << 13)
#define TSC_TSEVNTENA       (1u << 14)
#define TSC_TSMSTRENA       (1u << 15)
#define TSC_SNAPTYPSEL_SHIFT 16
#define TSC_TSENMACADDR     (1u << 18)
#define TSC_CMDS            (TSC_TSINIT | TSC_TSUPDT | TSC_TSADDREG)
/* The bits software sets and the block clears: writing 0 leaves them. */
#define TSC_WS              (TSC_CMDS | TSC_TSTRIG)
#define TSC_RW              0x0007ff3fu
#define TSC_RESET           TSC_TSIPV4ENA
#define TS_SSINC_MASK       0xffu
#define TS_SUBSEC_MASK      0x7fffffffu
#define TS_SUBSEC_ADDSUB    (1u << 31)
#define TS_SEC_HI_MASK      0xffffu
/* EMACTSTPSTATUS */
#define TSS_TSSOVF          (1u << 0)
#define TSS_TSTARGT         (1u << 1)
#define TSS_TSTRGTERR       (1u << 3)
/* PPSCONTROL, as ESP-IDF's v3 emac_ptp_struct.h lays it out */
#define PPS_CTRL0_MASK      0xfu
#define PPS_EN0             (1u << 4)
#define PPS_CTRL_RW         0x6767677fu

#define TS_BINARY_ROLLOVER  0x80000000ull
#define TS_DIGITAL_ROLLOVER 1000000000ull

/* Transmit descriptor */
#define TDES0_OWN           (1u << 31)
#define TDES0_IC            (1u << 30)
#define TDES0_LS            (1u << 29)
#define TDES0_FS            (1u << 28)
#define TDES0_DC            (1u << 27)
#define TDES0_DP            (1u << 26)
#define TDES0_TTSE          (1u << 25)
#define TDES0_CRCR          (1u << 24)
#define TDES0_CIC_SHIFT     22
#define TDES0_TER           (1u << 21)
#define TDES0_TCH           (1u << 20)
#define TDES0_VLIC_SHIFT    18
#define TDES0_TTSS          (1u << 17)
#define TDES0_IHE           (1u << 16)
#define TDES0_ES            (1u << 15)
#define TDES0_JT            (1u << 14)
#define TDES0_FF            (1u << 13)
#define TDES0_IPE           (1u << 12)
#define TDES0_NC            (1u << 10)
#define TDES0_VF            (1u << 7)
#define TDES0_UF            (1u << 1)
#define TDES0_STATUS        0x0003ffffu
#define TDES1_TBS2_SHIFT    16
#define TDES1_SAIC_SHIFT    29
#define TDES_BS_MASK        0x1fff

/* Receive descriptor */
#define RDES0_OWN           (1u << 31)
#define RDES0_AFM           (1u << 30)
#define RDES0_FL_SHIFT      16
#define RDES0_ES            (1u << 15)
#define RDES0_DE            (1u << 14)
#define RDES0_SAF           (1u << 13)
#define RDES0_LE            (1u << 12)
#define RDES0_VLAN          (1u << 10)
#define RDES0_FS            (1u << 9)
#define RDES0_LS            (1u << 8)
#define RDES0_GF            (1u << 7)
#define RDES0_TSA           (1u << 7)
#define RDES0_FT            (1u << 5)
#define RDES0_RWT           (1u << 4)
#define RDES0_CE            (1u << 1)
#define RDES0_ESA           (1u << 0)
#define RDES1_DIC           (1u << 31)
#define RDES1_RBS2_SHIFT    16
#define RDES1_RER           (1u << 15)
#define RDES1_RCH           (1u << 14)
#define RDES4_UDP           1
#define RDES4_TCP           2
#define RDES4_ICMP          3
#define RDES4_IPHE          (1u << 3)
#define RDES4_IPPE          (1u << 4)
#define RDES4_IPCB          (1u << 5)
#define RDES4_IPV4          (1u << 6)
#define RDES4_IPV6          (1u << 7)
#define RDES4_MSG_SHIFT     8
#define RDES4_PTP_ETH       (1u << 12)
#define RDES4_PTP_V2        (1u << 13)

/* Descriptor words 4, 6 and 7 of a 32-byte descriptor */
#define DES4_OFF            16
#define DES6_OFF            24
#define DES7_OFF            28

#define ETH_HLEN            14
#define ETH_FCS_LEN         4
#define ETH_MIN_LEN         60
#define ETH_TYPE_VLAN       0x8100
#define ETH_TYPE_SVLAN      0x88a8
#define ETH_TYPE_CONTROL    0x8808
#define ETH_TYPE_PTP        0x88f7
/* Smallest Length/Type value that is a type, not a length */
#define ETH_TYPE_MIN        0x0600
#define VLAN_TAG_LEN        4

/* The UDP ports PTP's event and general messages go to */
#define PTP_EVENT_PORT      319
#define PTP_GENERAL_PORT    320

/* RDES4's PTP message type codes */
#define PTP_MSG_SYNC        0x1
#define PTP_MSG_FOLLOW_UP   0x2
#define PTP_MSG_DELAY_REQ   0x3
#define PTP_MSG_DELAY_RESP  0x4
#define PTP_MSG_PDELAY_REQ  0x5
#define PTP_MSG_PDELAY_RESP 0x6
#define PTP_MSG_PDELAY_FUP  0x7
#define PTP_MSG_ANNOUNCE    0x8
#define PTP_MSG_MANAGEMENT  0x9
#define PTP_MSG_SIGNALING   0xa
#define PTP_MSG_RESERVED    0xf

/* Bound on the descriptors walked for one frame, against owned loops */
#define DESC_WALK_LIMIT     4096

/* The slot time pause times count, in bit times, at 10 and 100 Mbit/s */
#define SLOT_BITS           512

static const uint8_t pause_da[ETH_ALEN] = {
    0x01, 0x80, 0xc2, 0x00, 0x00, 0x01
};

/* The multicast addresses PTP over Ethernet uses */
static const uint8_t ptp_da[2][ETH_ALEN] = {
    { 0x01, 0x1b, 0x19, 0x00, 0x00, 0x00 },
    { 0x01, 0x80, 0xc2, 0x00, 0x00, 0x0e },
};

static void dwmac_tx_dma(Esp32DwmacState *s);
static void dwmac_rx_dma(Esp32DwmacState *s);
static void dwmac_tx_kick(Esp32DwmacState *s);
static void dwmac_mac_rx(Esp32DwmacState *s, const uint8_t *frame,
                         size_t len);
static void dwmac_ptp_schedule(Esp32DwmacState *s);

static Esp32DwmacClass *dwmac_class(Esp32DwmacState *s)
{
    return ESP32_DWMAC_GET_CLASS(s);
}

static const char *dwmac_name(Esp32DwmacState *s)
{
    return dwmac_class(s)->log_name;
}

static bool dwmac_live(Esp32DwmacState *s)
{
    return !s->held &&
           (!clock_has_source(s->bus_clk) || clock_get_hz(s->bus_clk) != 0);
}

static bool dwmac_tx_clk(Esp32DwmacState *s)
{
    return !s->held && dwmac_class(s)->tx_clk(s);
}

static bool dwmac_rx_clk(Esp32DwmacState *s)
{
    return !s->held && dwmac_class(s)->rx_clk(s);
}

/* The MAC's own interrupts, EMACINTS */
static uint32_t dwmac_mac_ints(Esp32DwmacState *s)
{
    uint32_t v = 0;

    if (s->pmt_csr & (PMT_MGKPRCVD | PMT_RWKPRCVD)) {
        v |= INTS_PMT;
    }
    if (dwmac_class(s)->has_ptp && s->ts_status) {
        v |= INTS_TS;
    }
    return v;
}

/*
 * [spec:nuos:req:emu.esp32.emac]
 * [spec:nuos:req:emu.esp32p4.emac]
 * The summary bits latch when an enabled status bit is set. The main
 * interrupt is raised by an enabled summary, or by one of the MAC's own
 * interrupts (PMT, timestamp) unless EMACINTMASK masks it; the power
 * management interrupt by a received magic packet or wake-up frame.
 */
static void dwmac_update_irq(Esp32DwmacState *s)
{
    bool level;

    if (s->status & s->int_en & ST_NIS_BITS) {
        s->status |= ST_NIS;
    }
    if (s->status & s->int_en & ST_AIS_BITS) {
        s->status |= ST_AIS;
    }
    level = ((s->status & ST_NIS) && (s->int_en & IE_NIE)) ||
            ((s->status & ST_AIS) && (s->int_en & IE_AIE)) ||
            (dwmac_mac_ints(s) & ~s->int_mask);
    qemu_set_irq(s->irq[ESP32_DWMAC_IRQ_SBD], level);
    qemu_set_irq(s->irq[ESP32_DWMAC_IRQ_PMT],
                 !!(s->pmt_csr & (PMT_MGKPRCVD | PMT_RWKPRCVD)));
}

static void dwmac_raise(Esp32DwmacState *s, uint32_t bits)
{
    s->status |= bits;
    if (bits & ST_RI) {
        s->status &= ~ST_ERI;
    }
    dwmac_update_irq(s);
}

/*
 * [spec:nuos:req:emu.esp32.emac]
 * [spec:nuos:req:emu.esp32p4.emac]
 * A DMA access the bus refuses is a fatal bus error: the engine that made
 * it sets FBI with the access's type in ERROR_BITS and stops using the
 * bus until a software reset.
 */
static void dwmac_bus_error(Esp32DwmacState *s, uint32_t addr, unsigned code)
{
    bool tx = code == EB_TX_DATA || code == EB_TX_DESC_WRITE ||
              code == EB_TX_DESC_READ;

    qemu_log_mask(LOG_GUEST_ERROR,
                  "%s: the bus refused a %s DMA access at 0x%08" PRIx32 "\n",
                  dwmac_name(s), tx ? "transmit" : "receive", addr);
    s->status = (s->status & ~(7u << ST_EB_SHIFT)) | (code << ST_EB_SHIFT);
    if (tx) {
        s->tx_fatal = true;
        s->tx_state = TS_STOPPED;
        s->tx_gathering = false;
    } else {
        s->rx_fatal = true;
        s->rx_state = RS_STOPPED;
    }
    dwmac_raise(s, ST_FBI);
}

static bool dwmac_dma_rw(Esp32DwmacState *s, uint32_t addr, void *buf,
                         uint32_t len, unsigned code, bool write)
{
    Esp32DwmacClass *k = dwmac_class(s);
    MemTxAttrs attrs = k->dma_attrs ? k->dma_attrs(s) : MEMTXATTRS_UNSPECIFIED;

    if (len == 0) {
        return true;
    }
    if ((k->dma_ok && !k->dma_ok(s, addr, len)) ||
        address_space_rw(s->dma_as, addr, attrs, buf, len, write) !=
        MEMTX_OK) {
        dwmac_bus_error(s, addr, code);
        return false;
    }
    return true;
}

static bool dwmac_dma_read(Esp32DwmacState *s, uint32_t addr, void *buf,
                           uint32_t len, unsigned code)
{
    return dwmac_dma_rw(s, addr, buf, len, code, false);
}

static bool dwmac_dma_write(Esp32DwmacState *s, uint32_t addr,
                            const void *buf, uint32_t len, unsigned code)
{
    return dwmac_dma_rw(s, addr, (void *)buf, len, code, true);
}

static bool dwmac_desc_read(Esp32DwmacState *s, uint32_t addr, uint32_t *d,
                            unsigned words, unsigned code)
{
    if (!dwmac_dma_read(s, addr, d, words * 4, code)) {
        return false;
    }
    for (unsigned i = 0; i < words; i++) {
        d[i] = le32_to_cpu(d[i]);
    }
    return true;
}

static bool dwmac_desc_write_word(Esp32DwmacState *s, uint32_t addr,
                                  uint32_t v, unsigned code)
{
    uint32_t le = cpu_to_le32(v);

    return dwmac_dma_write(s, addr, &le, 4, code);
}

/*
 * The descriptor after one: the list's base at the end of a ring, the
 * chained address, or the next ring entry DESC_SKIP_LEN words on.
 */
static uint32_t dwmac_next_desc(Esp32DwmacState *s, uint32_t addr,
                                bool end_of_ring, bool chained,
                                uint32_t next, uint32_t base)
{
    uint32_t size = (s->bus_mode & BUS_MODE_ATDS) ? 32 : 16;
    uint32_t skip = (s->bus_mode >> BUS_MODE_DSL_SHIFT) & BUS_MODE_DSL_MASK;

    if (end_of_ring) {
        return base;
    }
    if (chained) {
        return next & ~3u;
    }
    return addr + size + skip * 4;
}

static uint32_t dwmac_fcs(const uint8_t *frame, size_t len)
{
    return crc32(0, frame, len);
}

static bool dwmac_fcs_ok(const uint8_t *frame, size_t len)
{
    return len >= ETH_FCS_LEN &&
           dwmac_fcs(frame, len - ETH_FCS_LEN) ==
           ldl_le_p(frame + len - ETH_FCS_LEN);
}

static void dwmac_addr(Esp32DwmacState *s, unsigned n, uint8_t *mac)
{
    stl_le_p(mac, s->addr_low[n]);
    stw_le_p(mac + 4, s->addr_high[n] & 0xffff);
}

/* Bit time in ns at the speed EMACCONFIG.FES selects */
static int64_t dwmac_bit_ns(Esp32DwmacState *s)
{
    return (s->config & CFG_FES) ? 10 : 100;
}

/* Preamble and SFD bytes, shortened by PRELEN in full duplex */
static unsigned dwmac_preamble(Esp32DwmacState *s)
{
    static const unsigned prelen[] = { 8, 6, 4, 8 };

    return (s->config & CFG_DM) ? prelen[s->config & CFG_PRELEN_MASK] : 8;
}

/*
 * The time a frame of len bytes, FCS included, holds the wire: preamble
 * and SFD, the frame, and the interframe gap EMACCONFIG.IFG sets.
 */
static int64_t dwmac_wire_ns(Esp32DwmacState *s, size_t len)
{
    unsigned ifg_bits = 96 - 8 * ((s->config >> CFG_IFG_SHIFT) & 7);

    if (!(s->config & CFG_DM) && ifg_bits < 64) {
        ifg_bits = 64;
    }
    return ((dwmac_preamble(s) + len) * 8 + ifg_bits) * dwmac_bit_ns(s);
}

/* Whether an Ethernet type marks a VLAN tag the MAC recognises */
static bool dwmac_is_vlan_type(Esp32DwmacState *s, uint16_t type)
{
    return type == ETH_TYPE_VLAN ||
           (dwmac_class(s)->has_vlan && (s->vlan_tag & VLAN_TAG_ESVL) &&
            type == ETH_TYPE_SVLAN);
}

/* Checksum offload */

static uint32_t dwmac_pseudo_sum(const uint8_t *addrs, unsigned addr_len,
                                 uint8_t proto, uint32_t len)
{
    uint8_t tail[8];

    stl_be_p(tail, len);
    tail[4] = tail[5] = tail[6] = 0;
    tail[7] = proto;
    return net_checksum_add(addr_len * 2, (uint8_t *)addrs) +
           net_checksum_add(8, tail);
}

/* Offset of the payload checksum field for an IP protocol, or -1 */
static int dwmac_l4_csum_off(uint8_t proto, uint32_t len)
{
    switch (proto) {
    case IP_PROTO_TCP:
        return len >= 20 ? 16 : -1;
    case IP_PROTO_UDP:
        return len >= 8 ? 6 : -1;
    case 1:  /* ICMP */
    case 58: /* ICMPv6 */
        return len >= 4 ? 2 : -1;
    default:
        return -1;
    }
}

static unsigned dwmac_l3_off(Esp32DwmacState *s, const uint8_t *frame,
                             size_t len, uint16_t *type)
{
    unsigned off = 12;

    if (len < ETH_HLEN) {
        *type = 0;
        return len;
    }
    *type = lduw_be_p(frame + off);
    if (dwmac_is_vlan_type(s, *type) && len >= ETH_HLEN + VLAN_TAG_LEN) {
        off += VLAN_TAG_LEN;
        *type = lduw_be_p(frame + off);
    }
    return off + 2;
}

/*
 * [spec:nuos:req:emu.esp32.emac]
 * [spec:nuos:req:emu.esp32p4.emac]
 * Transmit checksum insertion, TDES0.CIC: 1 inserts the IPv4 header
 * checksum; 2 also the TCP, UDP or ICMP checksum, summing the payload
 * over the pseudo-header sum software left in the checksum field; 3
 * computes the pseudo-header sum too. Returns IHE and IPE status bits.
 */
static uint32_t dwmac_tx_csum(Esp32DwmacState *s, uint8_t *frame,
                              size_t len, unsigned cic)
{
    uint16_t type;
    unsigned l3 = dwmac_l3_off(s, frame, len, &type);
    uint8_t *ip = frame + l3;
    size_t avail = len - l3;
    uint32_t sum;
    uint8_t proto;
    uint32_t plen;
    uint8_t *payload;
    int csum_off;
    uint16_t c;

    if (type == ETH_P_IP) {
        unsigned ihl;
        uint32_t total;

        if (avail < 20 || (ip[0] >> 4) != 4) {
            return TDES0_IHE;
        }
        ihl = (ip[0] & 0xf) * 4;
        total = lduw_be_p(ip + 2);
        if (ihl < 20 || ihl > avail || total < ihl) {
            return TDES0_IHE;
        }
        stw_be_p(ip + 10, 0);
        stw_be_p(ip + 10, net_raw_checksum(ip, ihl));
        if (cic < 2) {
            return 0;
        }
        if (total > avail) {
            return TDES0_IPE;
        }
        if (lduw_be_p(ip + 6) & 0x3fff) {
            /* Fragments carry no whole payload to sum. */
            return 0;
        }
        proto = ip[9];
        plen = total - ihl;
        payload = ip + ihl;
        sum = proto == 1 ? 0 : dwmac_pseudo_sum(ip + 12, 4, proto, plen);
    } else if (type == ETH_P_IPV6) {
        if (avail < 40 || (ip[0] >> 4) != 6) {
            return TDES0_IHE;
        }
        if (cic < 2) {
            return 0;
        }
        proto = ip[6];
        plen = lduw_be_p(ip + 4);
        if (plen > avail - 40) {
            return TDES0_IPE;
        }
        payload = ip + 40;
        sum = dwmac_pseudo_sum(ip + 8, 16, proto, plen);
    } else {
        return 0;
    }

    csum_off = dwmac_l4_csum_off(proto, plen);
    if (csum_off < 0) {
        if (proto == IP_PROTO_TCP || proto == IP_PROTO_UDP ||
            proto == 1 || proto == 58) {
            return TDES0_IPE;
        }
        return 0;
    }
    if (cic == 2) {
        /* The field holds software's pseudo-header sum. */
        sum = 0;
    } else {
        stw_be_p(payload + csum_off, 0);
    }
    sum += net_checksum_add(plen, payload);
    c = net_checksum_finish(sum);
    if (proto == IP_PROTO_UDP && c == 0) {
        c = 0xffff;
    }
    stw_be_p(payload + csum_off, c);
    return 0;
}

/*
 * [spec:nuos:req:emu.esp32.emac]
 * [spec:nuos:req:emu.esp32p4.emac]
 * The receive checksum engine (EMACCONFIG.IPC) with the GMAC's full
 * offload: IPv4 header and TCP, UDP or ICMP payload checksums over IPv4
 * and IPv6, reported in RDES4. Returns RDES4.
 */
static uint32_t dwmac_rx_csum(Esp32DwmacState *s, const uint8_t *frame,
                              size_t len)
{
    uint16_t type;
    unsigned l3 = dwmac_l3_off(s, frame, len, &type);
    const uint8_t *ip = frame + l3;
    size_t avail = len - l3;
    uint32_t rdes4;
    uint32_t sum;
    uint8_t proto;
    uint32_t plen;
    const uint8_t *payload;
    int csum_off;

    if (type == ETH_P_IP) {
        unsigned ihl;
        uint32_t total;

        rdes4 = RDES4_IPV4;
        if (avail < 20 || (ip[0] >> 4) != 4) {
            return rdes4 | RDES4_IPHE;
        }
        ihl = (ip[0] & 0xf) * 4;
        total = lduw_be_p(ip + 2);
        if (ihl < 20 || ihl > avail || total < ihl ||
            net_raw_checksum((uint8_t *)ip, ihl) != 0) {
            return rdes4 | RDES4_IPHE;
        }
        if (lduw_be_p(ip + 6) & 0x3fff) {
            return rdes4;
        }
        proto = ip[9];
        plen = total - ihl;
        payload = ip + ihl;
        if (total > avail) {
            return rdes4 | RDES4_IPPE;
        }
        sum = proto == 1 ? 0 : dwmac_pseudo_sum(ip + 12, 4, proto, plen);
    } else if (type == ETH_P_IPV6) {
        rdes4 = RDES4_IPV6;
        if (avail < 40 || (ip[0] >> 4) != 6) {
            return rdes4 | RDES4_IPHE;
        }
        proto = ip[6];
        plen = lduw_be_p(ip + 4);
        payload = ip + 40;
        if (plen > avail - 40) {
            return rdes4 | RDES4_IPPE;
        }
        sum = dwmac_pseudo_sum(ip + 8, 16, proto, plen);
    } else {
        return RDES4_IPCB;
    }

    switch (proto) {
    case IP_PROTO_UDP:
        rdes4 |= RDES4_UDP;
        break;
    case IP_PROTO_TCP:
        rdes4 |= RDES4_TCP;
        break;
    case 1:
    case 58:
        rdes4 |= RDES4_ICMP;
        break;
    default:
        return rdes4 | RDES4_IPCB;
    }
    csum_off = dwmac_l4_csum_off(proto, plen);
    if (csum_off < 0) {
        return rdes4 | RDES4_IPPE;
    }
    if (proto == IP_PROTO_UDP && type == ETH_P_IP &&
        lduw_be_p(payload + csum_off) == 0) {
        return rdes4;
    }
    sum += net_checksum_add(plen, (uint8_t *)payload);
    if (net_checksum_finish(sum) != 0) {
        rdes4 |= RDES4_IPPE;
    }
    return rdes4;
}

/* The IEEE 1588 system time */

typedef struct DwmacPtpTime {
    /* Seconds with the high word above them, 48 bits */
    uint64_t sec;
    uint32_t subsec;
    uint32_t acc;
} DwmacPtpTime;

static uint32_t dwmac_ptp_hz(Esp32DwmacState *s)
{
    if (!s->ptp_clk || s->held) {
        return 0;
    }
    return clock_get_hz(s->ptp_clk);
}

/* Whether the system time advances: TSENA set, on a running clock */
static bool dwmac_ptp_running(Esp32DwmacState *s)
{
    return dwmac_class(s)->has_ptp && (s->ts_ctrl & TSC_TSENA) &&
           dwmac_ptp_hz(s) != 0;
}

static uint64_t dwmac_ptp_rollover(Esp32DwmacState *s)
{
    return (s->ts_ctrl & TSC_TSCTRLSSR) ? TS_DIGITAL_ROLLOVER
                                        : TS_BINARY_ROLLOVER;
}

static uint64_t dwmac_default_clock_edges(Esp32DwmacState *s, Clock *clk,
                                          int64_t ns)
{
    return muldiv64(MAX(ns, 0), clock_get_hz(clk), NANOSECONDS_PER_SECOND);
}

static int64_t dwmac_default_clock_edge_ns(Esp32DwmacState *s, Clock *clk,
                                           uint64_t edge)
{
    uint32_t hz = clock_get_hz(clk);

    return hz ? muldiv64_round_up(edge, NANOSECONDS_PER_SECOND, hz)
              : INT64_MAX;
}

/* The PTP reference clock's rising edges from time 0 to @ns */
static uint64_t dwmac_ptp_edges(Esp32DwmacState *s, int64_t ns)
{
    return dwmac_class(s)->clock_edges(s, s->ptp_clk, ns);
}

/* The time of the @edge'th rising edge */
static int64_t dwmac_ptp_edge_ns(Esp32DwmacState *s, uint64_t edge)
{
    return dwmac_class(s)->clock_edge_ns(s, s->ptp_clk, edge);
}

/* When @cycles of the bus clock, its edges after @from, have come */
static int64_t dwmac_bus_after(Esp32DwmacState *s, int64_t from,
                               uint64_t cycles)
{
    Esp32DwmacClass *k = dwmac_class(s);

    return k->clock_edge_ns(s, s->bus_clk,
                            k->clock_edges(s, s->bus_clk, from) + cycles);
}

/* The sub-second increments @cycles cycles of the PTP clock make */
static uint64_t dwmac_ptp_incs(Esp32DwmacState *s, DwmacPtpTime *t,
                               uint64_t cycles)
{
    uint64_t lo, hi;

    if (!(s->ts_ctrl & TSC_TSCFUPDT)) {
        return cycles;
    }
    /* Fine update: count the carries out of the 32-bit accumulator. */
    mulu64(&lo, &hi, cycles, s->ptp_addend);
    lo += t->acc;
    if (lo < t->acc) {
        hi++;
    }
    t->acc = lo;
    return (hi << 32) | (lo >> 32);
}

/* Advance @t by @units sub-second units, carrying into the seconds. */
static void dwmac_ptp_add_units(Esp32DwmacState *s, DwmacPtpTime *t,
                                uint64_t units)
{
    uint64_t roll = dwmac_ptp_rollover(s);
    uint64_t total = t->subsec + units;

    t->sec = (t->sec + total / roll) & MAKE_64BIT_MASK(0, 48);
    t->subsec = total % roll;
}

static void dwmac_ptp_get(Esp32DwmacState *s, DwmacPtpTime *t)
{
    t->sec = ((uint64_t)s->ptp_sec_hi << 32) | s->ptp_sec;
    t->subsec = s->ptp_subsec;
    t->acc = s->ptp_acc;
}

static void dwmac_ptp_put(Esp32DwmacState *s, const DwmacPtpTime *t)
{
    if ((t->sec >> 32) != s->ptp_sec_hi) {
        /* The seconds counter has wrapped. */
        s->ts_status |= TSS_TSSOVF;
    }
    s->ptp_sec = t->sec;
    s->ptp_sec_hi = t->sec >> 32;
    s->ptp_subsec = t->subsec;
    s->ptp_acc = t->acc;
}

/* The system time at @ns of QEMU_CLOCK_VIRTUAL, from the anchor on. */
static void dwmac_ptp_time_at(Esp32DwmacState *s, int64_t ns,
                              DwmacPtpTime *t)
{
    uint64_t cycles;

    dwmac_ptp_get(s, t);
    if (!dwmac_ptp_running(s) || ns <= s->ptp_anchor_ns) {
        return;
    }
    cycles = dwmac_ptp_edges(s, ns) - dwmac_ptp_edges(s, s->ptp_anchor_ns);
    dwmac_ptp_add_units(s, t, dwmac_ptp_incs(s, t, cycles) *
                              (s->ts_ssinc & TS_SSINC_MASK));
}

static bool dwmac_ptp_reached(const DwmacPtpTime *t, uint32_t sec,
                              uint32_t subsec)
{
    uint32_t tsec = t->sec;

    return tsec > sec || (tsec == sec && t->subsec >= subsec);
}

/*
 * [spec:nuos:req:emu.esp32p4.emac]
 * The PPS output in its fixed mode, PPSCTRL0 = n: at 0 a pulse from each
 * second's start until the next sub-second increment; otherwise a square
 * wave of 2^n Hz, high in the first half of each period, the periods
 * starting at the second.
 */
static bool dwmac_pps_level(Esp32DwmacState *s, const DwmacPtpTime *t,
                            uint64_t *next_units)
{
    unsigned n = s->pps_ctrl & PPS_CTRL0_MASK;
    uint64_t roll = dwmac_ptp_rollover(s);
    uint64_t ssinc = s->ts_ssinc & TS_SSINC_MASK;
    uint64_t half, k, edge;

    if (n == 0) {
        if (t->subsec < ssinc) {
            *next_units = ssinc - t->subsec;
            return true;
        }
        *next_units = roll - t->subsec;
        return false;
    }
    /* The half-periods, 2^(n + 1) to the second */
    half = n + 1;
    k = ((uint64_t)t->subsec << half) / roll;
    edge = ((k + 1) * roll + (1ull << half) - 1) >> half;
    *next_units = MIN(edge, roll) - t->subsec;
    return !(k & 1);
}

static void dwmac_ptp_update_pps(Esp32DwmacState *s, const DwmacPtpTime *t)
{
    uint64_t next;
    bool level = dwmac_pps_level(s, t, &next);

    if (s->pps_ctrl & PPS_EN0) {
        level = false;
    }
    if (level != s->pps_level) {
        s->pps_level = level;
        qemu_set_irq(s->pps_out, level);
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.emac]
 * Bring the system time up to now, then look for its events: the seconds
 * counter wrapping (TSSOVF), and the target time reached while TSTRIG is
 * set (TSTARGT), which clears TSTRIG.
 */
static void dwmac_ptp_sync(Esp32DwmacState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    DwmacPtpTime t;

    if (!dwmac_class(s)->has_ptp) {
        return;
    }
    dwmac_ptp_time_at(s, now, &t);
    dwmac_ptp_put(s, &t);
    s->ptp_anchor_ns = now;
    if ((s->ts_ctrl & TSC_TSTRIG) &&
        dwmac_ptp_reached(&t, s->ts_target_sec, s->ts_target_subsec)) {
        s->ts_ctrl &= ~TSC_TSTRIG;
        s->ts_status |= TSS_TSTARGT;
    }
    dwmac_ptp_update_pps(s, &t);
    dwmac_update_irq(s);
}

/* The units the time must advance before @sec, @subsec; 0 if reached */
static uint64_t dwmac_ptp_units_to(Esp32DwmacState *s, const DwmacPtpTime *t,
                                   uint64_t sec, uint64_t subsec)
{
    uint64_t roll = dwmac_ptp_rollover(s);
    uint64_t now_sec = t->sec & 0xffffffffu;

    if (now_sec > sec || (now_sec == sec && t->subsec >= subsec)) {
        return 0;
    }
    return (sec - now_sec) * roll + subsec - t->subsec;
}

/*
 * Arm the timer for the next event of the system time: the target time,
 * the seconds wrapping, or the PPS output's next edge. Far events are
 * approached in steps, so that the cycle counts stay in range.
 */
static void dwmac_ptp_schedule(Esp32DwmacState *s)
{
    uint64_t ssinc = s->ts_ssinc & TS_SSINC_MASK;
    uint64_t units = UINT64_MAX;
    uint64_t incs, cycles, next_pps;
    DwmacPtpTime t;

    timer_del(&s->ptp_timer);
    if (!dwmac_ptp_running(s) || !ssinc ||
        ((s->ts_ctrl & TSC_TSCFUPDT) && !s->ptp_addend)) {
        return;
    }
    dwmac_ptp_get(s, &t);
    if (s->ts_ctrl & TSC_TSTRIG) {
        units = MIN(units, dwmac_ptp_units_to(s, &t, s->ts_target_sec,
                                              s->ts_target_subsec));
    }
    units = MIN(units, dwmac_ptp_units_to(s, &t, 1ull << 32, 0));
    if (!(s->pps_ctrl & PPS_EN0)) {
        dwmac_pps_level(s, &t, &next_pps);
        units = MIN(units, next_pps);
    }
    incs = DIV_ROUND_UP(units, ssinc);
    incs = MIN(MAX(incs, 1), 1ull << 31);
    if (s->ts_ctrl & TSC_TSCFUPDT) {
        cycles = ((incs << 32) - t.acc + s->ptp_addend - 1) / s->ptp_addend;
    } else {
        cycles = incs;
    }
    timer_mod(&s->ptp_timer,
              dwmac_ptp_edge_ns(s, dwmac_ptp_edges(s, s->ptp_anchor_ns) +
                                cycles));
}

static void dwmac_ptp_timer_cb(void *opaque)
{
    Esp32DwmacState *s = opaque;

    dwmac_ptp_sync(s);
    dwmac_ptp_schedule(s);
}

/*
 * [spec:nuos:req:emu.esp32p4.emac]
 * The commands EMACTSTPCTRL starts, each on the PTP clock: TSINIT sets
 * the system time to the update registers', TSUPDT adds them to it or,
 * with ADDSUB, subtracts them, and TSADDREG loads the addend register
 * into the fine update. Each bit clears as its command completes; with
 * the PTP clock stopped they wait.
 */
static void dwmac_ptp_commands(Esp32DwmacState *s)
{
    uint64_t roll = dwmac_ptp_rollover(s);
    uint64_t upd = s->ts_subsec_upd & TS_SUBSEC_MASK;
    DwmacPtpTime t;

    if (!(s->ts_ctrl & TSC_CMDS) || !dwmac_ptp_hz(s)) {
        return;
    }
    if (upd >= roll && (s->ts_ctrl & (TSC_TSINIT | TSC_TSUPDT))) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: EMACNSUPT's sub-seconds 0x%"
                      PRIx64 " are past the rollover\n", dwmac_name(s), upd);
        upd %= roll;
    }
    dwmac_ptp_get(s, &t);
    if (s->ts_ctrl & TSC_TSINIT) {
        t.sec = (t.sec & ~0xffffffffull) | s->ts_sec_upd;
        t.subsec = upd;
    }
    if (s->ts_ctrl & TSC_TSUPDT) {
        uint64_t now = (t.sec & 0xffffffffu) * roll + t.subsec;
        uint64_t delta = (uint64_t)s->ts_sec_upd * roll + upd;
        uint64_t hi = t.sec & ~0xffffffffull;
        uint64_t span = (1ull << 32) * roll;

        now = (s->ts_subsec_upd & TS_SUBSEC_ADDSUB) ?
              (now + span - delta) % span : (now + delta) % span;
        t.sec = hi | (now / roll);
        t.subsec = now % roll;
    }
    if (s->ts_ctrl & TSC_TSADDREG) {
        s->ptp_addend = s->ts_addend;
    }
    /* Setting the time is not the seconds counter wrapping. */
    s->ptp_sec = t.sec;
    s->ptp_sec_hi = t.sec >> 32;
    s->ptp_subsec = t.subsec;
    s->ptp_acc = t.acc;
    s->ts_ctrl &= ~TSC_CMDS;
}

/*
 * [spec:nuos:req:emu.esp32p4.emac]
 * Arming the target time interrupt with TSTRIG for a time already passed
 * reports TSTRGTERR, and TSTARGT, at once.
 */
static void dwmac_ptp_arm_target(Esp32DwmacState *s)
{
    DwmacPtpTime t;

    dwmac_ptp_get(s, &t);
    if (dwmac_ptp_reached(&t, s->ts_target_sec, s->ts_target_subsec)) {
        s->ts_ctrl &= ~TSC_TSTRIG;
        s->ts_status |= TSS_TSTRGTERR | TSS_TSTARGT;
    }
}

static void dwmac_ptp_write_ctrl(Esp32DwmacState *s, uint32_t v)
{
    uint32_t old = s->ts_ctrl;

    dwmac_ptp_sync(s);
    s->ts_ctrl = (s->ts_ctrl & TSC_WS) | (v & TSC_RW);
    if ((v & TSC_TSTRIG) && !(old & TSC_TSTRIG)) {
        dwmac_ptp_arm_target(s);
    }
    dwmac_ptp_commands(s);
    dwmac_ptp_sync(s);
    dwmac_ptp_schedule(s);
}

/* Timestamp a frame: the system time at @ns, for a descriptor. */
static uint64_t dwmac_ptp_stamp(Esp32DwmacState *s, int64_t ns)
{
    DwmacPtpTime t;

    dwmac_ptp_time_at(s, ns, &t);
    return ((t.sec & 0xffffffffu) << 32) | t.subsec;
}

/*
 * The PTP message in a received frame, as the timestamp engine finds it
 * with EMACTSTPCTRL's settings: RDES4's message type, PTP frame type and
 * version bits, or 0 for a frame that carries none.
 */
static uint32_t dwmac_ptp_parse(Esp32DwmacState *s, const uint8_t *f,
                                size_t len, int da_match)
{
    uint32_t ctrl = s->ts_ctrl;
    uint16_t type;
    unsigned off = dwmac_l3_off(s, f, len, &type);
    const uint8_t *msg = NULL;
    size_t msg_len = 0;
    uint32_t rdes4 = 0;
    unsigned mt;

    if (type == ETH_TYPE_PTP && (ctrl & TSC_TSIPENA)) {
        if ((ctrl & TSC_TSENMACADDR) && da_match < 0 &&
            memcmp(f, ptp_da[0], ETH_ALEN) && memcmp(f, ptp_da[1], ETH_ALEN)) {
            return 0;
        }
        msg = f + off;
        msg_len = len - off;
        rdes4 |= RDES4_PTP_ETH;
    } else if ((type == ETH_P_IP && (ctrl & TSC_TSIPV4ENA)) ||
               (type == ETH_P_IPV6 && (ctrl & TSC_TSIPV6ENA))) {
        const uint8_t *ip = f + off;
        size_t avail = len - off;
        unsigned hl;
        uint16_t port;

        if (type == ETH_P_IP) {
            if (avail < 20 || (ip[0] >> 4) != 4 || ip[9] != IP_PROTO_UDP ||
                (lduw_be_p(ip + 6) & 0x3fff)) {
                return 0;
            }
            hl = (ip[0] & 0xf) * 4;
        } else {
            if (avail < 40 || (ip[0] >> 4) != 6 || ip[6] != IP_PROTO_UDP) {
                return 0;
            }
            hl = 40;
        }
        if (avail < hl + 8) {
            return 0;
        }
        port = lduw_be_p(ip + hl + 2);
        if (port != PTP_EVENT_PORT && port != PTP_GENERAL_PORT) {
            return 0;
        }
        msg = ip + hl + 8;
        msg_len = avail - hl - 8;
    } else {
        return 0;
    }

    if (ctrl & TSC_TSVER2ENA) {
        static const uint8_t v2_types[16] = {
            [0x0] = PTP_MSG_SYNC, [0x1] = PTP_MSG_DELAY_REQ,
            [0x2] = PTP_MSG_PDELAY_REQ, [0x3] = PTP_MSG_PDELAY_RESP,
            [0x8] = PTP_MSG_FOLLOW_UP, [0x9] = PTP_MSG_DELAY_RESP,
            [0xa] = PTP_MSG_PDELAY_FUP, [0xb] = PTP_MSG_ANNOUNCE,
            [0xc] = PTP_MSG_SIGNALING, [0xd] = PTP_MSG_MANAGEMENT,
        };

        if (msg_len < 2 || (msg[1] & 0xf) != 2) {
            return 0;
        }
        mt = v2_types[msg[0] & 0xf];
        rdes4 |= RDES4_PTP_V2;
    } else {
        static const uint8_t v1_types[5] = {
            PTP_MSG_SYNC, PTP_MSG_DELAY_REQ, PTP_MSG_FOLLOW_UP,
            PTP_MSG_DELAY_RESP, PTP_MSG_MANAGEMENT,
        };

        if (msg_len < 33 || lduw_be_p(msg) != 1) {
            return 0;
        }
        mt = msg[32] < ARRAY_SIZE(v1_types) ? v1_types[msg[32]] : 0;
    }
    if (!mt) {
        mt = PTP_MSG_RESERVED;
    }
    return rdes4 | (mt << RDES4_MSG_SHIFT);
}

/*
 * [spec:nuos:req:emu.esp32p4.emac]
 * Whether a received frame's timestamp is taken: every frame with
 * TSENALL, otherwise the PTP messages SNAPTYPSEL, TSMSTRENA and TSEVNTENA
 * select (EMACTSTPCTRL's table).
 */
static bool dwmac_ptp_snapshot(Esp32DwmacState *s, uint32_t ptp)
{
    unsigned mt = (ptp >> RDES4_MSG_SHIFT) & 0xf;
    unsigned sel = (s->ts_ctrl >> TSC_SNAPTYPSEL_SHIFT) & 3;
    bool mstr = s->ts_ctrl & TSC_TSMSTRENA;
    bool evnt = s->ts_ctrl & TSC_TSEVNTENA;
    uint32_t set;

    if (s->ts_ctrl & TSC_TSENALL) {
        return true;
    }
    if (!ptp) {
        return false;
    }
#define M(x) (1u << PTP_MSG_##x)
    switch (sel) {
    case 0:
        set = !evnt ? M(SYNC) | M(FOLLOW_UP) | M(DELAY_REQ) | M(DELAY_RESP) :
              mstr ? M(DELAY_REQ) : M(SYNC);
        break;
    case 1:
        set = !evnt ? M(SYNC) | M(FOLLOW_UP) | M(DELAY_REQ) | M(DELAY_RESP) |
                      M(PDELAY_REQ) | M(PDELAY_RESP) | M(PDELAY_FUP) :
              mstr ? M(DELAY_REQ) | M(PDELAY_REQ) | M(PDELAY_RESP) :
                     M(SYNC) | M(PDELAY_REQ) | M(PDELAY_RESP);
        break;
    case 2:
        set = M(SYNC) | M(DELAY_REQ);
        break;
    default:
        set = M(PDELAY_REQ) | M(PDELAY_RESP);
        break;
    }
#undef M
    return set & (1u << mt);
}

/* Transmit */

/*
 * [spec:nuos:req:emu.esp32.emac]
 * [spec:nuos:req:emu.esp32p4.emac]
 * A frame leaving the MAC: back to its own receiver in MAC loopback,
 * through the PHY otherwise, if the MII reaches it. The PHY returns it in
 * its own loopback, and puts it on the medium when the link is up; the
 * link partner discards runts and frames with a bad FCS.
 */
static void dwmac_wire_out(Esp32DwmacState *s, const uint8_t *wire,
                           size_t len)
{
    Esp32DwmacClass *k = dwmac_class(s);
    NetClientState *nc = qemu_get_queue(s->nic);

    if (s->config & CFG_LM) {
        if (dwmac_rx_clk(s)) {
            dwmac_mac_rx(s, wire, len);
        }
        return;
    }
    if (!s->phy || ip101_phy_isolated(s->phy) ||
        (k->tx_path && !k->tx_path(s))) {
        return;
    }
    if (ip101_phy_loopback(s->phy)) {
        if (dwmac_rx_clk(s) && (!k->rx_path || k->rx_path(s)) &&
            ((s->config & CFG_DM) || !(s->config & CFG_DO))) {
            dwmac_mac_rx(s, wire, len);
        }
        return;
    }
    if (!ip101_phy_link_up(s->phy)) {
        return;
    }
    if (len < ETH_MIN_LEN + ETH_FCS_LEN || !dwmac_fcs_ok(wire, len)) {
        return;
    }
    qemu_send_packet(nc, wire, len - ETH_FCS_LEN);
}

static bool dwmac_tx_can_send(Esp32DwmacState *s)
{
    return dwmac_live(s) && (s->config & CFG_TE) && dwmac_tx_clk(s);
}

static void dwmac_tx_drive(Esp32DwmacState *s, bool on)
{
    Esp32DwmacClass *k = dwmac_class(s);

    if (k->tx_drive) {
        k->tx_drive(s, on);
    }
}

/*
 * The source address control: EMACCONFIG.SARC, or else TDES1.SAIC.
 * Returns whether the frame changed.
 */
static bool dwmac_tx_source_addr(Esp32DwmacState *s, uint8_t *frame,
                                 size_t *len)
{
    unsigned sarc = (s->config >> CFG_SARC_SHIFT) & 7;
    unsigned saic = (s->tx_tdes1 >> TDES1_SAIC_SHIFT) & 7;
    unsigned op, which;
    uint8_t mac[ETH_ALEN];

    if (sarc & 2) {
        op = (sarc & 1) ? 2 : 1;
        which = sarc >> 2;
    } else {
        op = saic & 3;
        which = saic >> 2;
    }
    if (op == 0) {
        return false;
    }
    if (op == 3) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: reserved TDES1.SAIC value\n", dwmac_name(s));
        return false;
    }
    dwmac_addr(s, which, mac);
    if (op == 1) {
        if (*len + ETH_ALEN > ESP32_DWMAC_MAX_FRAME || *len < ETH_ALEN) {
            return false;
        }
        memmove(frame + 2 * ETH_ALEN, frame + ETH_ALEN, *len - ETH_ALEN);
        *len += ETH_ALEN;
    } else if (*len < 2 * ETH_ALEN) {
        return false;
    }
    memcpy(frame + ETH_ALEN, mac, ETH_ALEN);
    return true;
}

/*
 * [spec:nuos:req:emu.esp32p4.emac]
 * VLAN tag control, from EMACTXVLTCTRL.VLC when VLP is set, else from the
 * first descriptor's TDES0.VLIC: delete the tag of a tagged frame, insert
 * VLT under the C-VLAN or (CSVL) S-VLAN type, or replace a tagged frame's
 * type and tag so. Returns whether the frame changed.
 */
static bool dwmac_tx_vlan(Esp32DwmacState *s, uint8_t *frame, size_t *len)
{
    uint32_t ctrl = s->tx_vlan_ctrl;
    unsigned op = (ctrl & TX_VLAN_VLP) ? (ctrl >> TX_VLAN_VLC_SHIFT) & 3
                                       : (s->tx_tdes0 >> TDES0_VLIC_SHIFT) & 3;
    uint16_t type = (ctrl & TX_VLAN_CSVL) ? ETH_TYPE_SVLAN : ETH_TYPE_VLAN;
    bool tagged = *len >= ETH_HLEN + VLAN_TAG_LEN &&
                  (lduw_be_p(frame + 12) == ETH_TYPE_VLAN ||
                   lduw_be_p(frame + 12) == ETH_TYPE_SVLAN);

    if (!dwmac_class(s)->has_vlan) {
        if (op) {
            qemu_log_mask(LOG_UNIMP, "%s: VLAN tag insertion\n",
                          dwmac_name(s));
        }
        return false;
    }
    switch (op) {
    case 1:
        if (!tagged) {
            return false;
        }
        memmove(frame + 12, frame + 12 + VLAN_TAG_LEN,
                *len - 12 - VLAN_TAG_LEN);
        *len -= VLAN_TAG_LEN;
        return true;
    case 2:
        if (*len < 12 || *len + VLAN_TAG_LEN > ESP32_DWMAC_MAX_FRAME) {
            return false;
        }
        memmove(frame + 12 + VLAN_TAG_LEN, frame + 12, *len - 12);
        *len += VLAN_TAG_LEN;
        break;
    case 3:
        if (!tagged) {
            return false;
        }
        break;
    default:
        return false;
    }
    stw_be_p(frame + 12, type);
    stw_be_p(frame + 14, ctrl & TX_VLAN_VLT_MASK);
    return true;
}

/*
 * [spec:nuos:req:emu.esp32.emac]
 * [spec:nuos:req:emu.esp32p4.emac]
 * The MAC's timed events: a management frame's end, a frame's or pause
 * frame's end on the wire, a received pause's end, the pause refresh and
 * the receive watchdog. Each runs as of its due time however late its
 * QEMU timer runs (esp32_line_run_at()), so that what it starts counts
 * from then, and a register access first takes those due by its time,
 * in time order (dwmac_catch_up()).
 */
enum {
    DT_MDIO,
    DT_TX,
    DT_PAUSE,
    DT_PAUSE_REFRESH,
    DT_RX_WDT,
    DT_COUNT,
};

static void dwmac_mdio_end(void *opaque);
static void dwmac_tx_end(void *opaque);
static void dwmac_pause_end(void *opaque);
static void dwmac_pause_refresh_end(void *opaque);
static void dwmac_rx_wdt_end(void *opaque);

static QEMUTimer *dwmac_timer(Esp32DwmacState *s, int n)
{
    switch (n) {
    case DT_MDIO:
        return &s->mdio_timer;
    case DT_TX:
        return &s->tx_timer;
    case DT_PAUSE:
        return &s->pause_timer;
    case DT_PAUSE_REFRESH:
        return &s->pause_refresh_timer;
    default:
        return &s->rx_wdt_timer;
    }
}

static void (*const dwmac_timer_end[DT_COUNT])(void *opaque) = {
    [DT_MDIO] = dwmac_mdio_end,
    [DT_TX] = dwmac_tx_end,
    [DT_PAUSE] = dwmac_pause_end,
    [DT_PAUSE_REFRESH] = dwmac_pause_refresh_end,
    [DT_RX_WDT] = dwmac_rx_wdt_end,
};

static void dwmac_timer_at(Esp32DwmacState *s, int n, int64_t due)
{
    s->due[n] = due;
    timer_mod(dwmac_timer(s, n), due);
}

static void dwmac_mdio_cb(void *opaque)
{
    Esp32DwmacState *s = opaque;

    esp32_line_run_at(s->due[DT_MDIO], dwmac_mdio_end, s);
}

static void dwmac_tx_timer_cb(void *opaque)
{
    Esp32DwmacState *s = opaque;

    esp32_line_run_at(s->due[DT_TX], dwmac_tx_end, s);
}

static void dwmac_pause_timer_cb(void *opaque)
{
    Esp32DwmacState *s = opaque;

    esp32_line_run_at(s->due[DT_PAUSE], dwmac_pause_end, s);
}

static void dwmac_pause_refresh_cb(void *opaque)
{
    Esp32DwmacState *s = opaque;

    esp32_line_run_at(s->due[DT_PAUSE_REFRESH], dwmac_pause_refresh_end, s);
}

static void dwmac_rx_wdt_cb(void *opaque)
{
    Esp32DwmacState *s = opaque;

    esp32_line_run_at(s->due[DT_RX_WDT], dwmac_rx_wdt_end, s);
}

/*
 * Take the events due by now, earliest first: without icount their
 * timers can run well after, while software polls GMIIADDR's GB, as
 * ESP-IDF's MDIO functions do, reads GMIIDATA, the DMA's STATUS (TI, RI,
 * TPS, the transmit and receive states) or the descriptors it closes.
 * Only a timer that has come is run.
 */
static void dwmac_catch_up(Esp32DwmacState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    for (;;) {
        int64_t due = now;
        int next = -1;

        for (int n = 0; n < DT_COUNT; n++) {
            if (timer_pending(dwmac_timer(s, n)) && s->due[n] <= due &&
                (next < 0 || s->due[n] < due)) {
                next = n;
                due = s->due[n];
            }
        }
        if (next < 0) {
            return;
        }
        timer_del(dwmac_timer(s, next));
        esp32_line_run_at(due, dwmac_timer_end[next], s);
    }
}

/*
 * Turn the gathered frame into the bytes the wire carries and start
 * sending them; the frame has left, and reached a receiver, when the wire
 * time is over. A frame with TDES0.TTSE has its timestamp taken as its
 * SFD leaves.
 */
static void dwmac_tx_frame_out(Esp32DwmacState *s)
{
    uint32_t tdes0 = s->tx_tdes0;
    size_t len = s->tx_len;
    uint8_t *f = s->tx_frame;
    uint32_t status = 0;
    unsigned cic = (tdes0 >> TDES0_CIC_SHIFT) & 3;
    size_t jabber_max = (s->config & CFG_JE) ? 10240 : 2048;
    int64_t now = esp32_line_time();
    bool add_crc, changed;
    size_t wire_len;

    changed = dwmac_tx_source_addr(s, f, &len);
    changed |= dwmac_tx_vlan(s, f, &len);
    if (cic) {
        status |= dwmac_tx_csum(s, f, len, cic);
    }
    if (len >= ETH_HLEN &&
        (lduw_be_p(f + 12) == ETH_TYPE_VLAN ||
         (dwmac_class(s)->has_vlan && lduw_be_p(f + 12) == ETH_TYPE_SVLAN))) {
        status |= TDES0_VF;
    }

    wire_len = len;
    /* Without DP, short frames get padding and an FCS whatever DC says. */
    add_crc = !(tdes0 & TDES0_DC) ||
              (!(tdes0 & TDES0_DP) && len < ETH_MIN_LEN);
    if (!(tdes0 & TDES0_DP) && wire_len < ETH_MIN_LEN) {
        memset(f + wire_len, 0, ETH_MIN_LEN - wire_len);
        wire_len = ETH_MIN_LEN;
    }
    if (add_crc) {
        stl_le_p(f + wire_len, dwmac_fcs(f, wire_len));
        wire_len += ETH_FCS_LEN;
    } else if (((tdes0 & TDES0_CRCR) || changed) &&
               wire_len >= ETH_FCS_LEN) {
        stl_le_p(f + wire_len - ETH_FCS_LEN,
                 dwmac_fcs(f, wire_len - ETH_FCS_LEN));
    }

    if (s->tx_jabber || (!(s->config & CFG_JD) && wire_len > jabber_max)) {
        /* The jabber timer cuts the frame off; nothing usable leaves. */
        status |= TDES0_JT;
        s->tx_jabber = true;
        wire_len = MIN(wire_len, jabber_max);
    } else {
        if (!(s->config & (CFG_DM | CFG_DCRS | CFG_LM)) &&
            (!s->phy || !ip101_phy_link_up(s->phy)) &&
            !(s->phy && ip101_phy_loopback(s->phy))) {
            /* Half duplex: the PHY never raised carrier sense. */
            status |= TDES0_NC;
        }
    }
    if (status & (TDES0_JT | TDES0_NC | TDES0_IHE | TDES0_IPE)) {
        status |= TDES0_ES;
    }
    if ((tdes0 & TDES0_TTSE) && dwmac_class(s)->has_ptp &&
        (s->ts_ctrl & TSC_TSENA)) {
        if (s->bus_mode & BUS_MODE_ATDS) {
            s->tx_ts = dwmac_ptp_stamp(s, now + dwmac_preamble(s) * 8 *
                                          dwmac_bit_ns(s));
            status |= TDES0_TTSS;
        } else {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: TDES0.TTSE with 16-byte "
                          "descriptors, which have no timestamp words\n",
                          dwmac_name(s));
        }
    }
    s->tx_status = status;
    s->tx_wire_len = wire_len;
    s->tx_on_wire = true;
    s->tx_state = TS_WAITING;
    dwmac_tx_drive(s, true);
    dwmac_timer_at(s, DT_TX, now + dwmac_wire_ns(s, wire_len));
}

/*
 * [spec:nuos:req:emu.esp32.emac]
 * [spec:nuos:req:emu.esp32p4.emac]
 * Send a pause frame: to the reserved multicast address, from MAC
 * address 0, with the pause time it was queued with.
 */
static void dwmac_tx_pause_out(Esp32DwmacState *s)
{
    s->pause_on_wire = true;
    s->tx_on_wire = true;
    dwmac_tx_drive(s, true);
    dwmac_timer_at(s, DT_TX, esp32_line_time() +
                   dwmac_wire_ns(s, ETH_MIN_LEN + ETH_FCS_LEN));
}

static void dwmac_tx_pause_done(Esp32DwmacState *s)
{
    uint8_t wire[ETH_MIN_LEN + ETH_FCS_LEN] = { 0 };

    memcpy(wire, pause_da, ETH_ALEN);
    dwmac_addr(s, 0, wire + ETH_ALEN);
    stw_be_p(wire + 12, ETH_TYPE_CONTROL);
    stw_be_p(wire + 14, 1);
    stw_be_p(wire + 16, s->pause_quanta);
    stl_le_p(wire + ETH_MIN_LEN, dwmac_fcs(wire, ETH_MIN_LEN));
    dwmac_wire_out(s, wire, sizeof(wire));
}

/* Queue a pause frame with @quanta, if flow control may send one. */
static void dwmac_queue_pause(Esp32DwmacState *s, uint16_t quanta)
{
    if (!(s->config & CFG_DM) || !(s->flow_ctrl & FC_TFE)) {
        return;
    }
    s->pause_pending = true;
    s->pause_quanta = quanta;
    dwmac_tx_kick(s);
}

/*
 * [spec:nuos:req:emu.esp32p4.emac]
 * While the flow control request stays up, the MAC sends its pause frame
 * again PLT slot times before the pause it asked for runs out.
 */
static void dwmac_pause_refresh_arm(Esp32DwmacState *s)
{
    static const unsigned plt_slots[] = { 4, 28, 144, 256 };
    unsigned pt = s->flow_ctrl >> FC_PT_SHIFT;
    unsigned plt = plt_slots[(s->flow_ctrl >> FC_PLT_SHIFT) & 3];
    unsigned slots = pt > plt ? pt - plt : 1;

    dwmac_timer_at(s, DT_PAUSE_REFRESH, esp32_line_time() +
                   (int64_t)slots * SLOT_BITS * dwmac_bit_ns(s));
}

static void dwmac_pause_refresh_end(void *opaque)
{
    Esp32DwmacState *s = opaque;

    if (s->fc_active && s->sbd_flowctrl) {
        dwmac_queue_pause(s, s->flow_ctrl >> FC_PT_SHIFT);
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.emac]
 * The sideband flow control request: in full duplex with TFE set its
 * rise sends a pause frame with EMACFC's pause time, repeated while it
 * stays up, and its fall a zero-quanta pause frame unless DZPQ is set.
 */
static void dwmac_sbd_flowctrl(void *opaque, int n, int level)
{
    Esp32DwmacState *s = opaque;
    bool up = level;

    if (up == s->sbd_flowctrl) {
        return;
    }
    s->sbd_flowctrl = up;
    if (up) {
        if ((s->config & CFG_DM) && (s->flow_ctrl & FC_TFE)) {
            s->fc_active = true;
            dwmac_queue_pause(s, s->flow_ctrl >> FC_PT_SHIFT);
        }
    } else if (s->fc_active) {
        s->fc_active = false;
        timer_del(&s->pause_refresh_timer);
        if (!(s->flow_ctrl & FC_DZPQ)) {
            dwmac_queue_pause(s, 0);
        }
    }
}

/* Start whatever the MAC has to send, if the wire is free. */
static void dwmac_tx_kick(Esp32DwmacState *s)
{
    if (s->tx_on_wire || !dwmac_tx_can_send(s)) {
        return;
    }
    if (s->pause_pending) {
        s->pause_pending = false;
        dwmac_tx_pause_out(s);
        return;
    }
    if (s->tx_ready && !s->paused) {
        dwmac_tx_frame_out(s);
    }
}

/*
 * Close the descriptor holding a frame's last segment with its status,
 * and its timestamp in TDES6/7, raising TI if it asked for an interrupt
 * on completion.
 */
static void dwmac_tx_close(Esp32DwmacState *s, uint32_t status)
{
    uint32_t tdes0;

    s->tx_state = TS_CLOSING;
    if (!dwmac_desc_read(s, s->tx_last_desc, &tdes0, 1, EB_TX_DESC_READ)) {
        return;
    }
    if ((status & TDES0_TTSS) &&
        (!dwmac_desc_write_word(s, s->tx_last_desc + DES6_OFF,
                                (uint32_t)s->tx_ts, EB_TX_DESC_WRITE) ||
         !dwmac_desc_write_word(s, s->tx_last_desc + DES7_OFF,
                                s->tx_ts >> 32, EB_TX_DESC_WRITE))) {
        return;
    }
    tdes0 = (tdes0 & ~(TDES0_OWN | TDES0_STATUS)) | status;
    if (!dwmac_desc_write_word(s, s->tx_last_desc, tdes0, EB_TX_DESC_WRITE)) {
        return;
    }
    if (tdes0 & TDES0_IC) {
        dwmac_raise(s, ST_TI);
    }
}

/* Leave the running state when DMAOPERATION_MODE.ST has been cleared. */
static bool dwmac_tx_maybe_stop(Esp32DwmacState *s)
{
    if (s->op_mode & OP_ST) {
        return false;
    }
    s->tx_gathering = false;
    if (s->tx_state != TS_STOPPED) {
        s->tx_state = TS_STOPPED;
        dwmac_raise(s, ST_TPS);
    }
    return true;
}

static void dwmac_tx_end(void *opaque)
{
    Esp32DwmacState *s = opaque;

    s->tx_on_wire = false;
    dwmac_tx_drive(s, false);
    if (s->pause_on_wire) {
        s->pause_on_wire = false;
        s->flow_ctrl &= ~FC_FCB;
        dwmac_tx_pause_done(s);
        if (s->fc_active && s->sbd_flowctrl && s->pause_quanta) {
            dwmac_pause_refresh_arm(s);
        }
    } else if (s->tx_ready) {
        s->tx_ready = false;
        if (!s->tx_jabber) {
            dwmac_wire_out(s, s->tx_frame, s->tx_wire_len);
        }
        dwmac_tx_close(s, s->tx_status);
        if (s->tx_jabber) {
            /* A jabber timeout stops the transmit process. */
            s->tx_jabber = false;
            s->tx_gathering = false;
            if (!s->tx_fatal) {
                s->tx_state = TS_STOPPED;
            }
            dwmac_raise(s, ST_TJT | ST_TPS);
            dwmac_tx_kick(s);
            return;
        }
        if (!s->tx_fatal && !dwmac_tx_maybe_stop(s)) {
            s->tx_state = TS_FETCHING;
        }
    }
    dwmac_tx_kick(s);
    dwmac_tx_dma(s);
}

/*
 * [spec:nuos:req:emu.esp32.emac]
 * [spec:nuos:req:emu.esp32p4.emac]
 * The transmit DMA: gather a frame from the descriptors the DMA owns,
 * first segment to last, clearing OWN in each as its buffers are read,
 * and hand it to the MTL FIFO. A descriptor the host owns suspends the
 * process with TU; in the middle of a frame the MAC may already have
 * started it, past the threshold in threshold mode or with the FIFO full
 * in store-and-forward mode, and then the frame underflows.
 */
static void dwmac_tx_dma(Esp32DwmacState *s)
{
    static const uint32_t ttc_bytes[] = { 64, 128, 192, 256, 40, 32, 24, 16 };

    for (unsigned n = 0; n < DESC_WALK_LIMIT; n++) {
        uint32_t d[4];
        uint32_t addr = s->tx_desc;
        uint32_t b1, b2, next;

        if (s->tx_fatal || s->tx_state == TS_STOPPED || s->tx_ready ||
            !dwmac_live(s) || dwmac_tx_maybe_stop(s)) {
            return;
        }
        s->tx_state = TS_FETCHING;
        if (!dwmac_desc_read(s, addr, d, 4, EB_TX_DESC_READ)) {
            return;
        }
        if (!(d[0] & TDES0_OWN)) {
            uint32_t start = (s->op_mode & OP_TSF) ?
                             dwmac_class(s)->tx_fifo_size :
                             ttc_bytes[(s->op_mode >> OP_TTC_SHIFT) & 7];

            if (s->tx_gathering && s->tx_len >= start) {
                s->tx_gathering = false;
                dwmac_tx_close(s, TDES0_UF | TDES0_ES);
                dwmac_raise(s, ST_UNF | ST_TI);
            }
            if (s->tx_state != TS_SUSPENDED) {
                s->tx_state = TS_SUSPENDED;
                dwmac_raise(s, ST_TU);
            }
            return;
        }
        if (!s->tx_gathering || (d[0] & TDES0_FS)) {
            if (!(d[0] & TDES0_FS)) {
                qemu_log_mask(LOG_GUEST_ERROR, "%s: transmit descriptor "
                              "0x%08" PRIx32 " starts a frame without "
                              "TDES0.FS\n", dwmac_name(s), addr);
            } else if (s->tx_gathering) {
                qemu_log_mask(LOG_GUEST_ERROR, "%s: transmit descriptor "
                              "0x%08" PRIx32 " has TDES0.FS in the middle "
                              "of a frame\n", dwmac_name(s), addr);
            }
            s->tx_gathering = true;
            s->tx_len = 0;
            s->tx_jabber = false;
            s->tx_tdes0 = d[0];
            s->tx_tdes1 = d[1];
        }
        s->tx_state = TS_WAITING;

        b1 = d[1] & TDES_BS_MASK;
        b2 = (d[0] & TDES0_TCH) ? 0 : (d[1] >> TDES1_TBS2_SHIFT) & TDES_BS_MASK;
        for (unsigned i = 0; i < 2; i++) {
            uint32_t want = i ? b2 : b1;
            uint32_t buf = d[2 + i];
            uint32_t room = ESP32_DWMAC_MAX_FRAME - s->tx_len;
            uint32_t take = MIN(want, room);

            if (!want) {
                continue;
            }
            if (!dwmac_dma_read(s, buf, s->tx_frame + s->tx_len, take,
                                EB_TX_DATA)) {
                return;
            }
            s->tx_buf = buf;
            s->tx_len += take;
            if (take < want) {
                s->tx_jabber = true;
            }
        }

        next = dwmac_next_desc(s, addr, d[0] & TDES0_TER, d[0] & TDES0_TCH,
                               d[3], s->tx_base);
        s->tx_desc = next;
        if (d[0] & TDES0_LS) {
            s->tx_last_desc = addr;
            s->tx_gathering = false;
            s->tx_ready = true;
            dwmac_raise(s, ST_ETI);
            dwmac_tx_kick(s);
            return;
        }
        if (!dwmac_desc_write_word(s, addr, d[0] & ~TDES0_OWN,
                                   EB_TX_DESC_WRITE)) {
            return;
        }
    }
    qemu_log_mask(LOG_GUEST_ERROR,
                  "%s: transmit frame spans too many descriptors\n",
                  dwmac_name(s));
    s->tx_gathering = false;
}

/* Flush the transmit FIFO: a frame waiting there closes as flushed. */
static void dwmac_tx_flush(Esp32DwmacState *s)
{
    if (s->tx_ready && !s->tx_on_wire) {
        s->tx_ready = false;
        dwmac_tx_close(s, TDES0_FF | TDES0_ES);
        if (!s->tx_fatal && !dwmac_tx_maybe_stop(s)) {
            s->tx_state = TS_FETCHING;
        }
        dwmac_tx_dma(s);
    }
}

/* Receive */

static bool dwmac_is_multicast(const uint8_t *a)
{
    return a[0] & 1;
}

static bool dwmac_is_broadcast(const uint8_t *a)
{
    static const uint8_t bcast[ETH_ALEN] = {
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff
    };

    return !memcmp(a, bcast, ETH_ALEN);
}

/*
 * Perfect filtering: compare addr with the enabled address registers in
 * the given role (destination or source), honouring each register's byte
 * mask. Address 0 is always enabled, for destinations, unmasked. Returns
 * the matching register, or -1.
 */
static int dwmac_perfect_match(Esp32DwmacState *s, const uint8_t *addr,
                               bool source)
{
    for (unsigned n = source ? 1 : 0; n < dwmac_class(s)->addr_count; n++) {
        uint32_t high = s->addr_high[n];
        uint32_t mbc = n ? (high >> ADDR_HIGH_MBC_SHIFT) & 0x3f : 0;
        uint8_t mac[ETH_ALEN];
        bool match = true;

        if (n && (!(high & ADDR_HIGH_AE) ||
                  !!(high & ADDR_HIGH_SA) != source)) {
            continue;
        }
        dwmac_addr(s, n, mac);
        for (unsigned i = 0; i < ETH_ALEN; i++) {
            if (!(mbc & (1u << i)) && mac[i] != addr[i]) {
                match = false;
                break;
            }
        }
        if (match) {
            return n;
        }
    }
    return -1;
}

/*
 * [spec:nuos:req:emu.esp32.emac]
 * [spec:nuos:req:emu.esp32p4.emac]
 * Power-down mode drops every frame; a magic packet (six 0xff bytes, then
 * MAC address 0 sixteen times) or, with GLBLUCAST, a unicast frame passing
 * the destination filter wakes the MAC when enabled.
 */
static void dwmac_pmt_check(Esp32DwmacState *s, const uint8_t *f, size_t len,
                            bool da_pass)
{
    uint8_t mac[ETH_ALEN];

    if ((s->pmt_csr & PMT_RWKPKTEN) && (s->pmt_csr & PMT_GLBLUCAST) &&
        !dwmac_is_multicast(f) && da_pass) {
        s->pmt_csr = (s->pmt_csr | PMT_RWKPRCVD) & ~PMT_PWRDWN;
    }
    if (s->pmt_csr & PMT_MGKPKTEN) {
        dwmac_addr(s, 0, mac);
        for (size_t i = 0; i + 6 + 16 * ETH_ALEN <= len; i++) {
            bool found = true;

            for (unsigned j = 0; j < 6 && found; j++) {
                found = f[i + j] == 0xff;
            }
            for (unsigned j = 0; j < 16 && found; j++) {
                found = !memcmp(f + i + 6 + j * ETH_ALEN, mac, ETH_ALEN);
            }
            if (found) {
                s->pmt_csr = (s->pmt_csr | PMT_MGKPRCVD) & ~PMT_PWRDWN;
                break;
            }
        }
    }
    if ((s->pmt_csr & PMT_RWKPKTEN) && !(s->pmt_csr & PMT_GLBLUCAST)) {
        qemu_log_mask(LOG_UNIMP,
                      "%s: remote wake-up frame filters\n", dwmac_name(s));
    }
    dwmac_update_irq(s);
}

/* A received pause frame stops the transmitter for its pause time. */
static void dwmac_rx_pause(Esp32DwmacState *s, const uint8_t *f)
{
    uint16_t quanta = lduw_be_p(f + 16);

    if (quanta == 0) {
        s->paused = false;
        timer_del(&s->pause_timer);
        dwmac_tx_kick(s);
        return;
    }
    s->paused = true;
    dwmac_timer_at(s, DT_PAUSE, esp32_line_time() +
                   (int64_t)quanta * SLOT_BITS * dwmac_bit_ns(s));
}

static void dwmac_pause_end(void *opaque)
{
    Esp32DwmacState *s = opaque;

    s->paused = false;
    dwmac_tx_kick(s);
}

/*
 * Take a frame from the head of the receive FIFO into the descriptors,
 * with its timestamp in RDES6/7 if it has one. Returns false if the
 * receive process is suspended for want of a descriptor.
 */
static bool dwmac_rx_dma_frame(Esp32DwmacState *s, const uint8_t *frame,
                               uint32_t len, uint32_t status, uint32_t rdes4,
                               uint64_t ts)
{
    bool atds = s->bus_mode & BUS_MODE_ATDS;
    bool ptp = dwmac_class(s)->has_ptp;
    uint32_t addr = s->rx_desc;
    uint32_t d[4];
    uint32_t pos = 0;
    bool first = true;

    if (!dwmac_desc_read(s, addr, d, 4, EB_RX_DESC_READ)) {
        return true;
    }
    if (!(d[0] & RDES0_OWN)) {
        if (s->rx_state != RS_SUSPENDED) {
            s->rx_state = RS_SUSPENDED;
            dwmac_raise(s, ST_RU);
        }
        return false;
    }
    s->rx_state = RS_WAITING;
    if (ptp && !atds) {
        /* No words for the timestamp in a 16-byte descriptor */
        status &= ~RDES0_TSA;
    }

    for (unsigned n = 0; n < DESC_WALK_LIMIT; n++) {
        uint32_t b1 = d[1] & TDES_BS_MASK;
        uint32_t b2 = (d[1] & RDES1_RCH) ? 0 :
                      (d[1] >> RDES1_RBS2_SHIFT) & TDES_BS_MASK;
        uint32_t next, rdes0, nd;

        if ((b1 | b2) & 3) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: receive buffer size not a "
                          "multiple of 4 in descriptor 0x%08" PRIx32 "\n",
                          dwmac_name(s), addr);
        }
        for (unsigned i = 0; i < 2; i++) {
            uint32_t n2 = MIN(i ? b2 : b1, len - pos);

            if (!n2) {
                continue;
            }
            if (!dwmac_dma_write(s, d[2 + i], frame + pos, n2, EB_RX_DATA)) {
                return true;
            }
            s->rx_buf = d[2 + i];
            pos += n2;
            if (first && i == 0 && pos < len) {
                dwmac_raise(s, ST_ERI);
            }
        }
        next = dwmac_next_desc(s, addr, d[1] & RDES1_RER, d[1] & RDES1_RCH,
                               d[3], s->rx_base);

        if (pos == len) {
            rdes0 = status | RDES0_LS | (first ? RDES0_FS : 0) |
                    (len << RDES0_FL_SHIFT);
        } else {
            if (!dwmac_desc_read(s, next, &nd, 1, EB_RX_DESC_READ)) {
                return true;
            }
            if (nd & RDES0_OWN) {
                if (!dwmac_desc_write_word(s, addr, first ? RDES0_FS : 0,
                                           EB_RX_DESC_WRITE)) {
                    return true;
                }
                addr = next;
                first = false;
                if (!dwmac_desc_read(s, addr, d, 4, EB_RX_DESC_READ)) {
                    return true;
                }
                continue;
            }
            /* No descriptor for the rest: truncate the frame here. */
            rdes0 = (status & ~RDES0_ESA) | RDES0_LS | RDES0_DE |
                    RDES0_ES | (first ? RDES0_FS : 0) |
                    (pos << RDES0_FL_SHIFT);
        }

        if (atds && (rdes0 & RDES0_ESA) && (ptp || (s->config & CFG_IPC)) &&
            !dwmac_desc_write_word(s, addr + DES4_OFF, rdes4,
                                   EB_RX_DESC_WRITE)) {
            return true;
        }
        if (ptp && (rdes0 & RDES0_TSA) &&
            (!dwmac_desc_write_word(s, addr + DES6_OFF, (uint32_t)ts,
                                    EB_RX_DESC_WRITE) ||
             !dwmac_desc_write_word(s, addr + DES7_OFF, ts >> 32,
                                    EB_RX_DESC_WRITE))) {
            return true;
        }
        if (!dwmac_desc_write_word(s, addr, rdes0, EB_RX_DESC_WRITE)) {
            return true;
        }
        s->rx_desc = next;
        if (d[1] & RDES1_DIC) {
            uint32_t riwt = s->rx_wdt & 0xff;

            if (riwt && clock_get_hz(s->bus_clk)) {
                dwmac_timer_at(s, DT_RX_WDT,
                               dwmac_bus_after(s, esp32_line_time(),
                                               riwt * 256));
            }
        } else {
            timer_del(&s->rx_wdt_timer);
            dwmac_raise(s, ST_RI);
        }
        if (pos < len && s->rx_state != RS_SUSPENDED) {
            s->rx_state = RS_SUSPENDED;
            dwmac_raise(s, ST_RU);
        }
        return true;
    }
    qemu_log_mask(LOG_GUEST_ERROR,
                  "%s: receive frame spans too many descriptors\n",
                  dwmac_name(s));
    return true;
}

static void dwmac_rx_wdt_end(void *opaque)
{
    Esp32DwmacState *s = opaque;

    dwmac_raise(s, ST_RI);
}

static void dwmac_rx_fifo_pop(Esp32DwmacState *s)
{
    uint32_t len = s->rx_fifo_len[0];
    uint32_t n = s->rx_fifo_count - 1;

    memmove(s->rx_fifo, s->rx_fifo + len, s->rx_fifo_used - len);
    s->rx_fifo_used -= len;
    memmove(s->rx_fifo_len, s->rx_fifo_len + 1, n * sizeof(s->rx_fifo_len[0]));
    memmove(s->rx_fifo_rdes0, s->rx_fifo_rdes0 + 1,
            n * sizeof(s->rx_fifo_rdes0[0]));
    memmove(s->rx_fifo_rdes4, s->rx_fifo_rdes4 + 1,
            n * sizeof(s->rx_fifo_rdes4[0]));
    memmove(s->rx_fifo_ts, s->rx_fifo_ts + 1, n * sizeof(s->rx_fifo_ts[0]));
    s->rx_fifo_count = n;
}

/*
 * [spec:nuos:req:emu.esp32.emac]
 * [spec:nuos:req:emu.esp32p4.emac]
 * The receive DMA drains the FIFO into the descriptor list. Without a
 * descriptor the process suspends with RU, and unless DFF is set the
 * frame at the FIFO's head is flushed and counted as missed.
 */
static void dwmac_rx_dma(Esp32DwmacState *s)
{
    while (s->rx_fifo_count && !s->rx_fatal && s->rx_state != RS_STOPPED &&
           (s->op_mode & OP_SR) && dwmac_live(s)) {
        if (!dwmac_rx_dma_frame(s, s->rx_fifo, s->rx_fifo_len[0],
                                s->rx_fifo_rdes0[0], s->rx_fifo_rdes4[0],
                                s->rx_fifo_ts[0])) {
            if (!(s->op_mode & OP_DFF)) {
                dwmac_rx_fifo_pop(s);
                if (s->missed_frames == MISSED_FC_MAX) {
                    s->missed_frames = 0;
                    s->missed_overflow = true;
                } else {
                    s->missed_frames++;
                }
            }
            return;
        }
        if (s->rx_fatal) {
            return;
        }
        dwmac_rx_fifo_pop(s);
    }
}

static void dwmac_rx_overflow(Esp32DwmacState *s)
{
    if (s->overflow_frames == OVERFLOW_FC_MAX) {
        s->overflow_frames = 0;
        s->fifo_overflow = true;
    } else {
        s->overflow_frames++;
    }
    dwmac_raise(s, ST_OVF);
}

/*
 * [spec:nuos:req:emu.esp32p4.emac]
 * The receive VLAN tag filter: a tagged frame matches EMACVLANTAG.VL, in
 * its 12-bit VID with ETV, or any tag while VL is zero; VTIM inverts the
 * match.
 */
static bool dwmac_vlan_match(Esp32DwmacState *s, const uint8_t *f)
{
    uint16_t mask = (s->vlan_tag & VLAN_TAG_ETV) ? 0x0fff : 0xffff;
    uint16_t vl = s->vlan_tag & VLAN_TAG_VL_MASK & mask;
    bool match = !vl || (lduw_be_p(f + 14) & mask) == vl;

    return match != !!(s->vlan_tag & VLAN_TAG_VTIM);
}

/*
 * [spec:nuos:req:emu.esp32.emac]
 * [spec:nuos:req:emu.esp32p4.emac]
 * The MAC's receive path for a frame off the MII, FCS included: the
 * power-management and pause-frame checks, the address and VLAN filters,
 * the receive status and timestamp, the dropping of frames the FIFO
 * configuration rejects, and the FIFO.
 */
static void dwmac_mac_rx(Esp32DwmacState *s, const uint8_t *f, size_t len)
{
    Esp32DwmacClass *k = dwmac_class(s);
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint32_t ff = s->frame_filter;
    uint32_t status = 0;
    uint32_t rdes4 = 0;
    uint64_t ts = 0;
    uint16_t type;
    bool vlan, da_pass, sa_pass, control, pause, deliver, csum_err;
    bool crc_err, giant, wdt;
    int da_match, sa_match;
    size_t data_len = len;
    size_t giant_len, wdog;

    if (!dwmac_live(s) || !(s->config & CFG_RE) || len < ETH_HLEN) {
        return;
    }
    type = lduw_be_p(f + 12);
    vlan = dwmac_is_vlan_type(s, type) && len >= ETH_HLEN + VLAN_TAG_LEN;

    /* Destination address filter */
    da_match = dwmac_perfect_match(s, f, false);
    if (dwmac_is_broadcast(f)) {
        da_pass = !(ff & FF_DBF);
    } else if (dwmac_is_multicast(f)) {
        da_pass = (ff & FF_PM) || ((da_match >= 0) != !!(ff & FF_DAIF));
    } else {
        da_pass = (da_match >= 0) != !!(ff & FF_DAIF);
    }
    /* Source address filter, against registers in source mode */
    sa_match = dwmac_perfect_match(s, f + ETH_ALEN, true);
    sa_pass = (sa_match >= 0) != !!(ff & FF_SAIF);
    if (ff & FF_PR) {
        da_pass = sa_pass = true;
    }

    if (s->pmt_csr & PMT_PWRDWN) {
        dwmac_pmt_check(s, f, len, da_pass);
        return;
    }

    control = type == ETH_TYPE_CONTROL;
    pause = control && len == ETH_MIN_LEN + ETH_FCS_LEN &&
            lduw_be_p(f + 14) == 1 && dwmac_fcs_ok(f, len);
    if (pause && (s->flow_ctrl & FC_RFE) && (s->config & CFG_DM)) {
        uint8_t mac[ETH_ALEN];

        dwmac_addr(s, 0, mac);
        if (!memcmp(f, pause_da, ETH_ALEN) ||
            ((s->flow_ctrl & FC_UP) && !memcmp(f, mac, ETH_ALEN))) {
            dwmac_rx_pause(s, f);
        }
    }

    if (control && !(ff & FF_RA)) {
        switch ((ff >> FF_PCF_SHIFT) & 3) {
        case 0:
            deliver = false;
            break;
        case 1:
            deliver = !pause;
            break;
        case 2:
            deliver = true;
            break;
        default:
            deliver = da_pass && (sa_pass || !(ff & FF_SAF));
            break;
        }
    } else {
        deliver = (ff & FF_RA) ||
                  (da_pass && (sa_pass || !(ff & FF_SAF)));
    }
    if (k->has_vlan && vlan && (ff & FF_VTFE) && !(ff & (FF_RA | FF_PR)) &&
        !dwmac_vlan_match(s, f)) {
        deliver = false;
    }
    if (!deliver) {
        return;
    }

    if (!da_pass) {
        status |= RDES0_AFM;
    }
    if (!sa_pass) {
        status |= RDES0_SAF;
    }
    crc_err = !dwmac_fcs_ok(f, len);
    if (crc_err) {
        status |= RDES0_CE;
    }
    if (type >= ETH_TYPE_MIN) {
        status |= RDES0_FT;
    }
    if (vlan) {
        status |= RDES0_VLAN;
    }

    giant_len = (s->config & CFG_JE) ? (vlan ? 9022 : 9018) :
                (s->config & CFG_2KPE) ? 2000 : (vlan ? 1522 : 1518);
    giant = len > giant_len;
    if (giant && !k->has_ptp) {
        status |= RDES0_GF;
    }
    if (s->config & CFG_WD) {
        wdog = ESP32_DWMAC_MAX_FRAME;
    } else if (s->wdog_to & WDOG_TO_PWE) {
        wdog = s->wdog_to & 0x3fff;
    } else {
        wdog = (s->config & CFG_JE) ? 10240 : 2048;
    }
    wdt = data_len > wdog;
    if (wdt) {
        data_len = wdog;
        status |= RDES0_RWT;
    }

    if (type < ETH_TYPE_MIN && len >= ETH_HLEN + ETH_FCS_LEN) {
        size_t payload = len - ETH_HLEN - ETH_FCS_LEN;

        if (type >= 46 ? type != payload : payload != 46) {
            status |= RDES0_LE;
        }
        if ((s->config & CFG_ACS) && ETH_HLEN + type <= data_len) {
            /* Strip the pad and the FCS from a length-field frame. */
            data_len = ETH_HLEN + type;
        }
    } else if (type >= ETH_TYPE_MIN && (s->config & CFG_CST) && !wdt &&
               data_len >= ETH_FCS_LEN) {
        /* Strip the FCS from a type frame. */
        data_len -= ETH_FCS_LEN;
    }

    csum_err = false;
    if (s->config & CFG_IPC) {
        rdes4 = dwmac_rx_csum(s, f, len - ETH_FCS_LEN);
        status |= RDES0_ESA;
        csum_err = rdes4 & (RDES4_IPHE | RDES4_IPPE);
    } else if (da_match > 0 && !k->has_ptp) {
        status |= RDES0_ESA;
    }
    if (k->has_ptp && (s->ts_ctrl & TSC_TSENA)) {
        uint32_t ptp = dwmac_ptp_parse(s, f, len - ETH_FCS_LEN, da_match);

        if (ptp) {
            rdes4 |= ptp;
            status |= RDES0_ESA;
        }
        if (dwmac_ptp_snapshot(s, ptp)) {
            ts = dwmac_ptp_stamp(s, now);
            status |= RDES0_TSA;
        }
    }
    if (crc_err || wdt || giant || csum_err) {
        status |= RDES0_ES;
    }

    /* The FIFO's drop rules */
    if (!(s->op_mode & OP_FEF)) {
        if (crc_err || wdt || giant) {
            return;
        }
        if (csum_err && !(s->op_mode & OP_DT)) {
            return;
        }
    }
    if (giant && (s->op_mode & OP_DGF)) {
        return;
    }
    if (len < ETH_MIN_LEN + ETH_FCS_LEN && !(status & RDES0_ES) &&
        !(s->op_mode & OP_FUF)) {
        return;
    }

    if (data_len > k->rx_fifo_size - s->rx_fifo_used ||
        s->rx_fifo_count == k->rx_fifo_size / 64) {
        if (data_len > k->rx_fifo_size && !s->rx_fifo_count &&
            !(s->op_mode & OP_RSF) && (s->op_mode & OP_SR) &&
            s->rx_state != RS_STOPPED && !s->rx_fatal) {
            /* Cut-through: a frame longer than the FIFO streams out. */
            if (dwmac_rx_dma_frame(s, f, data_len, status, rdes4, ts)) {
                return;
            }
        }
        dwmac_rx_overflow(s);
        return;
    }
    memcpy(s->rx_fifo + s->rx_fifo_used, f, data_len);
    s->rx_fifo_used += data_len;
    s->rx_fifo_len[s->rx_fifo_count] = data_len;
    s->rx_fifo_rdes0[s->rx_fifo_count] = status;
    s->rx_fifo_rdes4[s->rx_fifo_count] = rdes4;
    s->rx_fifo_ts[s->rx_fifo_count] = ts;
    s->rx_fifo_count++;
    dwmac_rx_dma(s);
}

/* The network backend's frames, arriving on the medium */
static ssize_t dwmac_receive(NetClientState *nc, const uint8_t *buf,
                             size_t size)
{
    Esp32DwmacState *s = qemu_get_nic_opaque(nc);
    Esp32DwmacClass *k = dwmac_class(s);
    g_autofree uint8_t *wire = NULL;
    size_t len = MAX(size, ETH_MIN_LEN);

    if (!s->phy || !ip101_phy_link_up(s->phy) || (s->config & CFG_LM) ||
        !dwmac_rx_clk(s) || (k->rx_path && !k->rx_path(s))) {
        return size;
    }
    if (len + ETH_FCS_LEN > ESP32_DWMAC_MAX_FRAME) {
        return size;
    }
    wire = g_malloc0(len + ETH_FCS_LEN);
    memcpy(wire, buf, size);
    stl_le_p(wire + len, dwmac_fcs(wire, len));
    dwmac_mac_rx(s, wire, len + ETH_FCS_LEN);
    return size;
}

static void dwmac_link_status_changed(NetClientState *nc)
{
    Esp32DwmacState *s = qemu_get_nic_opaque(nc);

    if (s->phy) {
        ip101_phy_set_medium(s->phy, nc->peer && !nc->link_down);
    }
}

/* MDIO */

/*
 * [spec:nuos:req:emu.esp32.emac]
 * [spec:nuos:req:emu.esp32p4.emac]
 * A management frame takes 64 MDC cycles (32 preamble bits, 32 frame
 * bits); MDC is the CSR clock divided as EMACGMIIADDR.MIICSRCLK selects.
 */
static void dwmac_mdio_start(Esp32DwmacState *s)
{
    static const uint8_t div[16] = {
        42, 62, 16, 26, 102, 124, 0, 0, 4, 6, 8, 10, 12, 14, 16, 18
    };
    Esp32DwmacClass *k = dwmac_class(s);
    unsigned cr = (s->mii_addr >> MII_CR_SHIFT) & 0xf;
    uint32_t d = div[cr];
    int64_t now = esp32_line_time(), end;

    if (!d) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: reserved MIICSRCLK value %u\n", dwmac_name(s), cr);
        d = 42;
    }
    /* Without its clock the frame takes the time it would at 80 MHz. */
    end = clock_get_hz(s->bus_clk) ? dwmac_bus_after(s, now, 64 * d)
                                   : now + muldiv64(64 * d,
                                                    NANOSECONDS_PER_SECOND,
                                                    80000000);
    if (k->mdio_drive) {
        k->mdio_drive(s, true);
    }
    dwmac_timer_at(s, DT_MDIO, end);
}

/*
 * [spec:nuos:req:emu.esp32.emac]
 * [spec:nuos:req:emu.esp32p4.emac]
 * The frame ends: the PHY at its address takes a write or answers a read,
 * if the frame reached it and the MAC listens to its MDIO pin; a read no
 * PHY answers returns the idle level of the MAC's MDIO input in every
 * bit, the pulled-up 0xffff on a board's MDIO line.
 */
static void dwmac_mdio_end(void *opaque)
{
    Esp32DwmacState *s = opaque;
    Esp32DwmacClass *k = dwmac_class(s);
    Esp32DwmacMdioPath path = { true, true, true };
    unsigned pa = (s->mii_addr >> MII_PA_SHIFT) & 0x1f;
    unsigned gr = (s->mii_addr >> MII_GR_SHIFT) & 0x1f;
    bool present;

    if (k->mdio_path) {
        k->mdio_path(s, &path);
    }
    present = path.to_phy && s->phy && ip101_phy_responds(s->phy, pa);
    if (s->mii_addr & MII_GW) {
        if (present) {
            ip101_phy_mdio_write(s->phy, gr, s->mii_data & 0xffff);
        }
    } else if (present && path.from_phy) {
        s->mii_data = ip101_phy_mdio_read(s->phy, gr);
    } else {
        s->mii_data = path.idle_level ? 0xffff : 0;
    }
    s->mii_addr &= ~MII_GB;
    if (k->mdio_drive) {
        k->mdio_drive(s, false);
    }
}

/* Registers */

static void dwmac_regs_reset(Esp32DwmacState *s)
{
    Esp32DwmacClass *k = dwmac_class(s);

    s->bus_mode = k->bus_mode_reset;
    s->rx_base = 0;
    s->tx_base = 0;
    s->status = 0;
    s->op_mode = 0;
    s->int_en = 0;
    s->missed_frames = 0;
    s->overflow_frames = 0;
    s->missed_overflow = false;
    s->fifo_overflow = false;
    s->rx_wdt = 0;
    s->tx_desc = 0;
    s->rx_desc = 0;
    s->tx_buf = 0;
    s->rx_buf = 0;
    s->tx_state = TS_STOPPED;
    s->rx_state = RS_STOPPED;
    s->tx_fatal = false;
    s->rx_fatal = false;

    s->config = k->config_reset;
    s->frame_filter = k->ff_reset;
    s->mii_addr = 0;
    s->mii_data = 0;
    s->flow_ctrl = 0;
    s->vlan_tag = 0;
    memset(s->rwuffr, 0, sizeof(s->rwuffr));
    s->rwuffr_ptr = 0;
    s->pmt_csr = 0;
    s->lpi_csr = 0;
    s->lpi_timers = LPI_TIMERS_RESET;
    s->int_mask = 0;
    s->addr_high[0] = ADDR_HIGH_AE | 0xffff;
    s->addr_low[0] = 0xffffffff;
    for (unsigned i = 1; i < ESP32_DWMAC_ADDR_MAX; i++) {
        s->addr_high[i] = 0xffff;
        s->addr_low[i] = 0xffffffff;
    }
    s->wdog_to = 0;
    s->tx_vlan_ctrl = 0;

    s->ts_ctrl = TSC_RESET;
    s->ts_ssinc = 0;
    s->ts_sec_upd = 0;
    s->ts_subsec_upd = 0;
    s->ts_addend = 0;
    s->ts_target_sec = 0;
    s->ts_target_subsec = 0;
    s->ts_status = 0;
    s->pps_ctrl = 0;
    s->ptp_anchor_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->ptp_sec = 0;
    s->ptp_sec_hi = 0;
    s->ptp_subsec = 0;
    s->ptp_acc = 0;
    s->ptp_addend = 0;

    timer_del(&s->mdio_timer);
    timer_del(&s->tx_timer);
    timer_del(&s->pause_timer);
    timer_del(&s->pause_refresh_timer);
    timer_del(&s->rx_wdt_timer);
    timer_del(&s->ptp_timer);
    s->tx_len = 0;
    s->tx_wire_len = 0;
    s->tx_gathering = false;
    s->tx_ready = false;
    s->tx_on_wire = false;
    s->tx_jabber = false;
    s->tx_tdes0 = 0;
    s->tx_tdes1 = 0;
    s->tx_last_desc = 0;
    s->tx_status = 0;
    s->tx_ts = 0;
    s->pause_pending = false;
    s->pause_quanta = 0;
    s->pause_on_wire = false;
    s->fc_active = false;
    s->paused = false;
    s->rx_fifo_used = 0;
    s->rx_fifo_count = 0;
    s->sw_reset_pending = s->bus_mode & BUS_MODE_SWR;
}

static void dwmac_reset_changed(Esp32DwmacState *s)
{
    Esp32DwmacClass *k = dwmac_class(s);

    if (k->reset_changed) {
        k->reset_changed(s);
    }
}

/* A software reset finishes once every clock domain has a clock. */
static void dwmac_sw_reset_check(Esp32DwmacState *s)
{
    if (s->sw_reset_pending && dwmac_tx_clk(s) && dwmac_rx_clk(s)) {
        s->sw_reset_pending = false;
        s->bus_mode &= ~BUS_MODE_SWR;
        dwmac_reset_changed(s);
    }
}

static uint32_t dwmac_debug(Esp32DwmacState *s)
{
    unsigned size = dwmac_class(s)->rx_fifo_size;
    uint32_t v = 0;

    if (s->rx_fifo_used == size) {
        v |= 3u << 8;
    } else if (s->rx_fifo_used >= size * 3 / 4) {
        v |= 2u << 8;
    } else if (s->rx_fifo_used) {
        v |= 1u << 8;
    }
    if (s->tx_on_wire) {
        v |= 1u << 16;
        v |= (s->pause_on_wire ? 2u : 3u) << 17;
        if (!s->pause_on_wire) {
            v |= 1u << 20;
        }
    }
    if (s->paused) {
        v |= 1u << 19;
    }
    if (s->tx_on_wire || s->tx_ready || s->tx_gathering) {
        v |= 1u << 24;
    }
    return v;
}

/* Whether core offset @off is one of the chip's MAC address registers */
static bool dwmac_is_addr(Esp32DwmacState *s, hwaddr off)
{
    return off >= A_ADDR0_HIGH &&
           off < A_ADDR0_HIGH + 8 * dwmac_class(s)->addr_count;
}

static bool dwmac_is_ptp(Esp32DwmacState *s, hwaddr off)
{
    return dwmac_class(s)->has_ptp && off >= A_TS_CTRL && off <= A_PPS_CTRL;
}

static uint32_t dwmac_ptp_read(Esp32DwmacState *s, hwaddr off)
{
    uint32_t v;

    dwmac_ptp_sync(s);
    switch (off) {
    case A_TS_CTRL:
        return s->ts_ctrl;
    case A_TS_SSINC:
        return s->ts_ssinc;
    case A_TS_SEC:
        return s->ptp_sec;
    case A_TS_SUBSEC:
        return s->ptp_subsec;
    case A_TS_SEC_UPD:
        return s->ts_sec_upd;
    case A_TS_SUBSEC_UPD:
        return s->ts_subsec_upd;
    case A_TS_ADDEND:
        return s->ts_addend;
    case A_TS_TARGET_SEC:
        return s->ts_target_sec;
    case A_TS_TARGET_SUBSEC:
        return s->ts_target_subsec;
    case A_TS_SEC_HI:
        return s->ptp_sec_hi;
    case A_TS_STATUS:
        v = s->ts_status;
        if (dwmac_live(s)) {
            s->ts_status = 0;
            dwmac_update_irq(s);
        }
        return v;
    default:
        return s->pps_ctrl;
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.emac]
 * The system time registers; a write that changes how the time counts
 * brings it up to date first.
 */
static void dwmac_ptp_write(Esp32DwmacState *s, hwaddr off, uint32_t v)
{
    if (off == A_TS_CTRL) {
        dwmac_ptp_write_ctrl(s, v);
        return;
    }
    dwmac_ptp_sync(s);
    switch (off) {
    case A_TS_SSINC:
        s->ts_ssinc = v & TS_SSINC_MASK;
        break;
    case A_TS_SEC_UPD:
        s->ts_sec_upd = v;
        break;
    case A_TS_SUBSEC_UPD:
        s->ts_subsec_upd = v;
        break;
    case A_TS_ADDEND:
        s->ts_addend = v;
        break;
    case A_TS_TARGET_SEC:
        s->ts_target_sec = v;
        break;
    case A_TS_TARGET_SUBSEC:
        s->ts_target_subsec = v & TS_SUBSEC_MASK;
        break;
    case A_TS_SEC_HI:
        s->ptp_sec_hi = v & TS_SEC_HI_MASK;
        break;
    case A_PPS_CTRL:
        s->pps_ctrl = v & PPS_CTRL_RW;
        if (s->pps_ctrl & ~PPS_CTRL0_MASK) {
            qemu_log_mask(LOG_UNIMP, "%s: flexible PPS output and PPS1 to "
                          "PPS3 (PPSCONTROL 0x%08" PRIx32 ")\n",
                          dwmac_name(s), s->pps_ctrl);
        }
        break;
    case A_TS_SEC:
    case A_TS_SUBSEC:
    case A_TS_STATUS:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only register "
                      "at 0x%04" HWADDR_PRIx "\n", dwmac_name(s), off);
        return;
    }
    dwmac_ptp_sync(s);
    dwmac_ptp_schedule(s);
}

uint32_t esp32_dwmac_read(Esp32DwmacState *s, hwaddr off, hwaddr addr)
{
    Esp32DwmacClass *k = dwmac_class(s);
    bool live;
    uint32_t v;

    dwmac_catch_up(s);
    live = dwmac_live(s);
    if (dwmac_is_addr(s, off)) {
        if (off & 4) {
            return s->addr_low[(off - A_ADDR0_HIGH) / 8];
        }
        return s->addr_high[(off - A_ADDR0_HIGH) / 8];
    }
    if (dwmac_is_ptp(s, off)) {
        return dwmac_ptp_read(s, off);
    }
    switch (off) {
    case A_BUS_MODE:
        return s->bus_mode;
    case A_TX_POLL:
    case A_RX_POLL:
        return 0;
    case A_RX_BASE:
        return s->rx_base;
    case A_TX_BASE:
        return s->tx_base;
    case A_STATUS:
        v = s->status | ((uint32_t)s->rx_state << ST_RS_SHIFT) |
            ((uint32_t)s->tx_state << ST_TS_SHIFT);
        if (dwmac_mac_ints(s) & INTS_PMT) {
            v |= ST_PMT;
        }
        if (dwmac_mac_ints(s) & INTS_TS) {
            v |= ST_TTI;
        }
        return v;
    case A_OP_MODE:
        return s->op_mode;
    case A_INT_EN:
        return s->int_en;
    case A_MISSED:
        if (!live) {
            return 0;
        }
        v = s->missed_frames | (s->missed_overflow ? MISSED_BMFC : 0) |
            (s->overflow_frames << OVERFLOW_FC_SHIFT) |
            (s->fifo_overflow ? OVERFLOW_BFOC : 0);
        s->missed_frames = 0;
        s->overflow_frames = 0;
        s->missed_overflow = false;
        s->fifo_overflow = false;
        return v;
    case A_RX_WDT:
        return s->rx_wdt;
    case A_AHB_STATUS:
        if (!k->has_ahb_status) {
            break;
        }
        /* The DMA's bursts finish within the access that starts them. */
        return 0;
    case A_TX_CUR_DESC:
        return s->tx_desc;
    case A_RX_CUR_DESC:
        return s->rx_desc;
    case A_TX_CUR_BUF:
        return s->tx_buf;
    case A_RX_CUR_BUF:
        return s->rx_buf;

    case A_CONFIG:
        return s->config;
    case A_FRAME_FILTER:
        return s->frame_filter;
    case A_MII_ADDR:
        return s->mii_addr;
    case A_MII_DATA:
        return s->mii_data;
    case A_FLOW_CTRL:
        return s->flow_ctrl;
    case A_VLAN_TAG:
        if (!k->has_vlan) {
            break;
        }
        return s->vlan_tag;
    case A_DEBUG:
        return dwmac_debug(s);
    case A_RWUFFR:
        if (!live) {
            return 0;
        }
        v = s->rwuffr[s->rwuffr_ptr];
        s->rwuffr_ptr = (s->rwuffr_ptr + 1) % ESP32_DWMAC_RWUFFR_COUNT;
        return v;
    case A_PMT_CSR:
        v = s->pmt_csr | (s->rwuffr_ptr << PMT_RWKPTR_SHIFT);
        if (!live) {
            return 0;
        }
        s->pmt_csr &= ~(PMT_MGKPRCVD | PMT_RWKPRCVD);
        dwmac_update_irq(s);
        return v;
    case A_LPI_CSR:
        return s->lpi_csr;
    case A_LPI_TIMERS:
        return s->lpi_timers;
    case A_INTS:
        return dwmac_mac_ints(s);
    case A_INT_MASK:
        return s->int_mask;
    case A_CSTATUS:
        return 0;
    case A_WDOG_TO:
        return s->wdog_to;
    case A_TX_VLAN:
        if (!k->has_vlan) {
            break;
        }
        return s->tx_vlan_ctrl;
    default:
        break;
    }
    qemu_log_mask(LOG_GUEST_ERROR,
                  "%s: read of reserved offset 0x%04" HWADDR_PRIx "\n",
                  dwmac_name(s), addr);
    return 0;
}

static void dwmac_write_op_mode(Esp32DwmacState *s, uint32_t v)
{
    uint32_t old = s->op_mode;

    s->op_mode = v & OP_RW & ~OP_FTF;
    if (v & OP_FTF) {
        dwmac_tx_flush(s);
    }

    if ((v & OP_ST) && !(old & OP_ST) && !s->tx_fatal) {
        s->tx_state = TS_FETCHING;
        dwmac_tx_dma(s);
    } else if (!(v & OP_ST) && (old & OP_ST)) {
        if (!s->tx_on_wire && !s->tx_ready) {
            dwmac_tx_maybe_stop(s);
        }
    }

    if ((v & OP_SR) && !(old & OP_SR) && !s->rx_fatal) {
        s->rx_state = RS_WAITING;
        dwmac_rx_dma(s);
    } else if (!(v & OP_SR) && (old & OP_SR)) {
        if (s->rx_state != RS_STOPPED) {
            s->rx_state = RS_STOPPED;
            dwmac_raise(s, ST_RPS);
        }
    }
}

void esp32_dwmac_write(Esp32DwmacState *s, hwaddr off, uint32_t v,
                       hwaddr addr)
{
    Esp32DwmacClass *k = dwmac_class(s);
    uint32_t old;

    dwmac_catch_up(s);
    if (dwmac_is_addr(s, off)) {
        unsigned n = (off - A_ADDR0_HIGH) / 8;

        if (off & 4) {
            s->addr_low[n] = v;
        } else if (n == 0) {
            s->addr_high[0] = ADDR_HIGH_AE | (v & ADDR0_HIGH_RW);
        } else {
            s->addr_high[n] = v & ADDRN_HIGH_RW;
        }
        return;
    }
    if (dwmac_is_ptp(s, off)) {
        dwmac_ptp_write(s, off, v);
        return;
    }
    switch (off) {
    case A_BUS_MODE:
        if (v & BUS_MODE_SWR) {
            /* The chip's wrapper sits outside the GMAC and keeps its state. */
            dwmac_regs_reset(s);
            s->bus_mode |= BUS_MODE_SWR;
            s->sw_reset_pending = true;
            qemu_set_irq(s->pps_out, 0);
            s->pps_level = false;
            dwmac_reset_changed(s);
            dwmac_sw_reset_check(s);
            dwmac_update_irq(s);
            return;
        }
        if (s->sw_reset_pending) {
            return;
        }
        s->bus_mode = v & k->bus_mode_rw;
        return;
    case A_TX_POLL:
        if (s->tx_state == TS_SUSPENDED) {
            s->tx_state = TS_FETCHING;
        }
        dwmac_tx_dma(s);
        return;
    case A_RX_POLL:
        if (s->rx_state == RS_SUSPENDED) {
            s->rx_state = RS_WAITING;
        }
        dwmac_rx_dma(s);
        return;
    case A_RX_BASE:
        s->rx_base = v & ~3u;
        if (s->rx_state == RS_STOPPED) {
            s->rx_desc = s->rx_base;
        } else {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: DMARXBASEADDR written while "
                          "the receive process runs\n", dwmac_name(s));
        }
        return;
    case A_TX_BASE:
        s->tx_base = v & ~3u;
        if (s->tx_state == TS_STOPPED) {
            s->tx_desc = s->tx_base;
        } else {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: DMATXBASEADDR written while "
                          "the transmit process runs\n", dwmac_name(s));
        }
        return;
    case A_STATUS:
        s->status &= ~(v & ST_W1C);
        dwmac_update_irq(s);
        return;
    case A_OP_MODE:
        dwmac_write_op_mode(s, v);
        return;
    case A_INT_EN:
        s->int_en = v & IE_RW;
        dwmac_update_irq(s);
        return;
    case A_RX_WDT:
        s->rx_wdt = v & 0xff;
        return;

    case A_CONFIG:
        old = s->config;
        s->config = v & k->config_rw;
        if ((s->config & CFG_TE) && !(old & CFG_TE)) {
            dwmac_tx_kick(s);
        }
        if ((s->config ^ old) & k->config_clock_bits) {
            esp32_dwmac_mii_changed(s);
        }
        return;
    case A_FRAME_FILTER:
        s->frame_filter = v & k->ff_rw;
        return;
    case A_MII_ADDR:
        if (s->mii_addr & MII_GB) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: EMACGMIIADDR written while "
                          "a management frame is busy\n", dwmac_name(s));
            return;
        }
        s->mii_addr = v & 0xffff;
        if (v & MII_GB) {
            dwmac_mdio_start(s);
        }
        return;
    case A_MII_DATA:
        if (s->mii_addr & MII_GB) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: EMACMIIDATA written while a "
                          "management frame is busy\n", dwmac_name(s));
        }
        s->mii_data = v & 0xffff;
        return;
    case A_FLOW_CTRL:
        if (s->flow_ctrl & FC_FCB) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: EMACFC written while a pause "
                          "frame is pending\n", dwmac_name(s));
            return;
        }
        s->flow_ctrl = v & FC_RW;
        if ((v & FC_FCB) && (s->config & CFG_DM)) {
            if (s->flow_ctrl & FC_TFE) {
                dwmac_queue_pause(s, s->flow_ctrl >> FC_PT_SHIFT);
            } else {
                s->flow_ctrl &= ~FC_FCB;
            }
        }
        return;
    case A_VLAN_TAG:
        if (!k->has_vlan) {
            break;
        }
        s->vlan_tag = v & VLAN_TAG_RW;
        return;
    case A_RWUFFR:
        s->rwuffr[s->rwuffr_ptr] = v;
        s->rwuffr_ptr = (s->rwuffr_ptr + 1) % ESP32_DWMAC_RWUFFR_COUNT;
        return;
    case A_PMT_CSR:
        if (v & PMT_RWKFILTRST) {
            s->rwuffr_ptr = 0;
        }
        s->pmt_csr = (s->pmt_csr & ~PMT_RW) | (v & PMT_RW);
        return;
    case A_LPI_CSR:
        s->lpi_csr = v & k->lpi_rw;
        if (v & LPI_LPIEN) {
            qemu_log_mask(LOG_UNIMP, "%s: low-power idle\n", dwmac_name(s));
        }
        return;
    case A_LPI_TIMERS:
        s->lpi_timers = v & LPI_TIMERS_RW;
        return;
    case A_INT_MASK:
        s->int_mask = v & k->int_mask_rw;
        dwmac_update_irq(s);
        return;
    case A_WDOG_TO:
        s->wdog_to = v & WDOG_TO_RW;
        return;
    case A_TX_VLAN:
        if (!k->has_vlan) {
            break;
        }
        s->tx_vlan_ctrl = v & TX_VLAN_RW;
        return;
    case A_AHB_STATUS:
        if (!k->has_ahb_status) {
            break;
        }
        /* fall through */
    case A_MISSED:
    case A_TX_CUR_DESC:
    case A_RX_CUR_DESC:
    case A_TX_CUR_BUF:
    case A_RX_CUR_BUF:
    case A_DEBUG:
    case A_INTS:
    case A_CSTATUS:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only register at "
                      "0x%04" HWADDR_PRIx "\n", dwmac_name(s), addr);
        return;
    default:
        break;
    }
    qemu_log_mask(LOG_GUEST_ERROR,
                  "%s: write to reserved offset 0x%04" HWADDR_PRIx "\n",
                  dwmac_name(s), addr);
}

/* [spec:nuos:req:emu.esp32.emac] */
/* [spec:nuos:req:emu.esp32p4.emac] */
void esp32_dwmac_mii_changed(Esp32DwmacState *s)
{
    dwmac_sw_reset_check(s);
    if (dwmac_live(s)) {
        dwmac_tx_kick(s);
        dwmac_tx_dma(s);
        dwmac_rx_dma(s);
    }
}

bool esp32_dwmac_sw_reset_pending(Esp32DwmacState *s)
{
    return s->sw_reset_pending;
}

uint64_t esp32_dwmac_ptp_time(Esp32DwmacState *s)
{
    DwmacPtpTime t;

    dwmac_ptp_time_at(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL), &t);
    return ((t.sec & 0xffffffffu) << 32) | t.subsec;
}

/*
 * [spec:nuos:req:emu.esp32p4.emac]
 * The chip's reset line resets the core as it rises and again as it
 * falls; while it is high the core does nothing.
 */
void esp32_dwmac_set_held(Esp32DwmacState *s, bool held)
{
    if (held == s->held) {
        return;
    }
    device_cold_reset(DEVICE(s));
    s->held = held;
    s->ptp_anchor_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    dwmac_reset_changed(s);
    if (!held) {
        esp32_dwmac_mii_changed(s);
    }
}

static void dwmac_bus_clk_update(void *opaque, ClockEvent event)
{
    Esp32DwmacState *s = opaque;

    if (dwmac_live(s)) {
        dwmac_tx_kick(s);
        dwmac_tx_dma(s);
        dwmac_rx_dma(s);
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.emac]
 * The PTP reference clock: the time counts at its old rate up to the
 * change, and commands waiting for it complete once it runs.
 */
static void dwmac_ptp_clk_update(void *opaque, ClockEvent event)
{
    Esp32DwmacState *s = opaque;

    if (event == ClockPreUpdate) {
        dwmac_ptp_sync(s);
        return;
    }
    s->ptp_anchor_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    dwmac_ptp_commands(s);
    dwmac_ptp_sync(s);
    dwmac_ptp_schedule(s);
}

static NetClientInfo net_esp32_dwmac_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .receive = dwmac_receive,
    .link_status_changed = dwmac_link_status_changed,
};

static void esp32_dwmac_reset_hold(Object *obj, ResetType type)
{
    Esp32DwmacState *s = ESP32_DWMAC(obj);

    dwmac_regs_reset(s);
    s->pps_level = false;
    qemu_set_irq(s->pps_out, 0);
    for (int i = 0; i < ESP32_DWMAC_IRQS; i++) {
        qemu_set_irq(s->irq[i], 0);
    }
}

static void esp32_dwmac_init(Object *obj)
{
    Esp32DwmacState *s = ESP32_DWMAC(obj);
    Esp32DwmacClass *k = ESP32_DWMAC_GET_CLASS(obj);
    DeviceState *dev = DEVICE(obj);

    for (int i = 0; i < ESP32_DWMAC_IRQS; i++) {
        sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq[i]);
    }
    s->bus_clk = qdev_init_clock_in(dev, k->bus_clock, dwmac_bus_clk_update,
                                    s, ClockUpdate);
    if (k->has_ptp) {
        s->ptp_clk = qdev_init_clock_in(dev, "ptp-ref", dwmac_ptp_clk_update,
                                        s, ClockPreUpdate | ClockUpdate);
        qdev_init_gpio_out_named(dev, &s->pps_out, ESP32_DWMAC_PTP_PPS, 1);
    }
    qdev_init_gpio_in_named(dev, dwmac_sbd_flowctrl, ESP32_DWMAC_SBD_FLOWCTRL,
                            1);
    timer_init_ns(&s->mdio_timer, QEMU_CLOCK_VIRTUAL, dwmac_mdio_cb, s);
    timer_init_ns(&s->tx_timer, QEMU_CLOCK_VIRTUAL, dwmac_tx_timer_cb, s);
    timer_init_ns(&s->pause_timer, QEMU_CLOCK_VIRTUAL, dwmac_pause_timer_cb,
                  s);
    timer_init_ns(&s->pause_refresh_timer, QEMU_CLOCK_VIRTUAL,
                  dwmac_pause_refresh_cb, s);
    timer_init_ns(&s->rx_wdt_timer, QEMU_CLOCK_VIRTUAL, dwmac_rx_wdt_cb, s);
    timer_init_ns(&s->ptp_timer, QEMU_CLOCK_VIRTUAL, dwmac_ptp_timer_cb, s);
}

static void esp32_dwmac_realize(DeviceState *dev, Error **errp)
{
    Esp32DwmacState *s = ESP32_DWMAC(dev);
    NetClientState *nc;

    if (!s->dma_as) {
        error_setg(errp, "%s: no address space for the DMA", dwmac_name(s));
        return;
    }
    qemu_macaddr_default_if_unset(&s->conf.macaddr);
    s->nic = qemu_new_nic(&net_esp32_dwmac_info, &s->conf,
                          object_get_typename(OBJECT(dev)), dev->id,
                          &dev->mem_reentrancy_guard, s);
    nc = qemu_get_queue(s->nic);
    qemu_format_nic_info_str(nc, s->conf.macaddr.a);
    dwmac_link_status_changed(nc);
}

static void esp32_dwmac_unrealize(DeviceState *dev)
{
    Esp32DwmacState *s = ESP32_DWMAC(dev);

    qemu_del_nic(s->nic);
}

static const Property esp32_dwmac_properties[] = {
    DEFINE_NIC_PROPERTIES(Esp32DwmacState, conf),
    DEFINE_PROP_LINK("phy", Esp32DwmacState, phy, TYPE_IP101_PHY,
                     IP101PhyState *),
};

const VMStateDescription vmstate_esp32_dwmac = {
    .name = TYPE_ESP32_DWMAC,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_BOOL(held, Esp32DwmacState),
        VMSTATE_UINT32(bus_mode, Esp32DwmacState),
        VMSTATE_UINT32(rx_base, Esp32DwmacState),
        VMSTATE_UINT32(tx_base, Esp32DwmacState),
        VMSTATE_UINT32(status, Esp32DwmacState),
        VMSTATE_UINT32(op_mode, Esp32DwmacState),
        VMSTATE_UINT32(int_en, Esp32DwmacState),
        VMSTATE_UINT32(missed_frames, Esp32DwmacState),
        VMSTATE_UINT32(overflow_frames, Esp32DwmacState),
        VMSTATE_BOOL(missed_overflow, Esp32DwmacState),
        VMSTATE_BOOL(fifo_overflow, Esp32DwmacState),
        VMSTATE_UINT32(rx_wdt, Esp32DwmacState),
        VMSTATE_UINT32(tx_desc, Esp32DwmacState),
        VMSTATE_UINT32(rx_desc, Esp32DwmacState),
        VMSTATE_UINT32(tx_buf, Esp32DwmacState),
        VMSTATE_UINT32(rx_buf, Esp32DwmacState),
        VMSTATE_UINT8(tx_state, Esp32DwmacState),
        VMSTATE_UINT8(rx_state, Esp32DwmacState),
        VMSTATE_BOOL(tx_fatal, Esp32DwmacState),
        VMSTATE_BOOL(rx_fatal, Esp32DwmacState),
        VMSTATE_UINT32(config, Esp32DwmacState),
        VMSTATE_UINT32(frame_filter, Esp32DwmacState),
        VMSTATE_UINT32(mii_addr, Esp32DwmacState),
        VMSTATE_UINT32(mii_data, Esp32DwmacState),
        VMSTATE_UINT32(flow_ctrl, Esp32DwmacState),
        VMSTATE_UINT32(vlan_tag, Esp32DwmacState),
        VMSTATE_UINT32_ARRAY(rwuffr, Esp32DwmacState,
                             ESP32_DWMAC_RWUFFR_COUNT),
        VMSTATE_UINT32(rwuffr_ptr, Esp32DwmacState),
        VMSTATE_UINT32(pmt_csr, Esp32DwmacState),
        VMSTATE_UINT32(lpi_csr, Esp32DwmacState),
        VMSTATE_UINT32(lpi_timers, Esp32DwmacState),
        VMSTATE_UINT32(int_mask, Esp32DwmacState),
        VMSTATE_UINT32_ARRAY(addr_high, Esp32DwmacState,
                             ESP32_DWMAC_ADDR_MAX),
        VMSTATE_UINT32_ARRAY(addr_low, Esp32DwmacState,
                             ESP32_DWMAC_ADDR_MAX),
        VMSTATE_UINT32(wdog_to, Esp32DwmacState),
        VMSTATE_UINT32(tx_vlan_ctrl, Esp32DwmacState),
        VMSTATE_BOOL(sw_reset_pending, Esp32DwmacState),
        VMSTATE_TIMER(mdio_timer, Esp32DwmacState),
        VMSTATE_UINT8_ARRAY(tx_frame, Esp32DwmacState,
                            ESP32_DWMAC_TX_BUF_SIZE),
        VMSTATE_UINT32(tx_len, Esp32DwmacState),
        VMSTATE_UINT32(tx_wire_len, Esp32DwmacState),
        VMSTATE_BOOL(tx_gathering, Esp32DwmacState),
        VMSTATE_BOOL(tx_ready, Esp32DwmacState),
        VMSTATE_BOOL(tx_on_wire, Esp32DwmacState),
        VMSTATE_BOOL(tx_jabber, Esp32DwmacState),
        VMSTATE_UINT32(tx_tdes0, Esp32DwmacState),
        VMSTATE_UINT32(tx_tdes1, Esp32DwmacState),
        VMSTATE_UINT32(tx_last_desc, Esp32DwmacState),
        VMSTATE_UINT32(tx_status, Esp32DwmacState),
        VMSTATE_UINT64(tx_ts, Esp32DwmacState),
        VMSTATE_TIMER(tx_timer, Esp32DwmacState),
        VMSTATE_BOOL(pause_pending, Esp32DwmacState),
        VMSTATE_UINT16(pause_quanta, Esp32DwmacState),
        VMSTATE_BOOL(pause_on_wire, Esp32DwmacState),
        VMSTATE_BOOL(sbd_flowctrl, Esp32DwmacState),
        VMSTATE_BOOL(fc_active, Esp32DwmacState),
        VMSTATE_TIMER(pause_refresh_timer, Esp32DwmacState),
        VMSTATE_TIMER(pause_timer, Esp32DwmacState),
        VMSTATE_BOOL(paused, Esp32DwmacState),
        VMSTATE_UINT8_ARRAY(rx_fifo, Esp32DwmacState,
                            ESP32_DWMAC_RX_FIFO_MAX),
        VMSTATE_UINT32(rx_fifo_used, Esp32DwmacState),
        VMSTATE_UINT32(rx_fifo_count, Esp32DwmacState),
        VMSTATE_UINT16_ARRAY(rx_fifo_len, Esp32DwmacState,
                             ESP32_DWMAC_RX_FIFO_FRAMES),
        VMSTATE_UINT32_ARRAY(rx_fifo_rdes0, Esp32DwmacState,
                             ESP32_DWMAC_RX_FIFO_FRAMES),
        VMSTATE_UINT32_ARRAY(rx_fifo_rdes4, Esp32DwmacState,
                             ESP32_DWMAC_RX_FIFO_FRAMES),
        VMSTATE_UINT64_ARRAY(rx_fifo_ts, Esp32DwmacState,
                             ESP32_DWMAC_RX_FIFO_FRAMES),
        VMSTATE_TIMER(rx_wdt_timer, Esp32DwmacState),
        VMSTATE_UINT32(ts_ctrl, Esp32DwmacState),
        VMSTATE_UINT32(ts_ssinc, Esp32DwmacState),
        VMSTATE_UINT32(ts_sec_upd, Esp32DwmacState),
        VMSTATE_UINT32(ts_subsec_upd, Esp32DwmacState),
        VMSTATE_UINT32(ts_addend, Esp32DwmacState),
        VMSTATE_UINT32(ts_target_sec, Esp32DwmacState),
        VMSTATE_UINT32(ts_target_subsec, Esp32DwmacState),
        VMSTATE_UINT32(ts_status, Esp32DwmacState),
        VMSTATE_UINT32(pps_ctrl, Esp32DwmacState),
        VMSTATE_INT64(ptp_anchor_ns, Esp32DwmacState),
        VMSTATE_UINT32(ptp_sec, Esp32DwmacState),
        VMSTATE_UINT16(ptp_sec_hi, Esp32DwmacState),
        VMSTATE_UINT32(ptp_subsec, Esp32DwmacState),
        VMSTATE_UINT32(ptp_acc, Esp32DwmacState),
        VMSTATE_UINT32(ptp_addend, Esp32DwmacState),
        VMSTATE_TIMER(ptp_timer, Esp32DwmacState),
        VMSTATE_BOOL(pps_level, Esp32DwmacState),
        VMSTATE_CLOCK(bus_clk, Esp32DwmacState),
        VMSTATE_INT64_ARRAY(due, Esp32DwmacState, 5),
        VMSTATE_END_OF_LIST()
    }
};

static void esp32_dwmac_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    Esp32DwmacClass *k = ESP32_DWMAC_CLASS(klass);

    rc->phases.hold = esp32_dwmac_reset_hold;
    dc->realize = esp32_dwmac_realize;
    dc->unrealize = esp32_dwmac_unrealize;
    k->clock_edges = dwmac_default_clock_edges;
    k->clock_edge_ns = dwmac_default_clock_edge_ns;
    device_class_set_props(dc, esp32_dwmac_properties);
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}

/* [spec:nuos:req:emu.esp32.emac] */
/* [spec:nuos:req:emu.esp32p4.emac] */
static const TypeInfo esp32_dwmac_info = {
    .name = TYPE_ESP32_DWMAC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32DwmacState),
    .instance_init = esp32_dwmac_init,
    .class_size = sizeof(Esp32DwmacClass),
    .class_init = esp32_dwmac_class_init,
    .abstract = true,
};

static void esp32_dwmac_register_types(void)
{
    type_register_static(&esp32_dwmac_info);
}

type_init(esp32_dwmac_register_types)
