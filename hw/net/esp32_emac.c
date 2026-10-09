/*
 * ESP32 Ethernet MAC (EMAC), a Synopsys DesignWare GMAC
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32 TRM, "Ethernet Media Access Controller (EMAC)", with the
 * field layout of ESP-IDF's soc/esp32 emac_dma_struct.h, emac_mac_struct.h
 * and emac_ext_struct.h, and the DesignWare GMAC behaviour they describe.
 *
 * One 8 KiB block: the DMA's registers at 0x0000, the ESP32's clock and
 * PHY interface wrapper at 0x0800 and the MAC's at 0x1000. The DMA moves
 * frames between descriptor lists in internal SRAM and the MTL FIFOs; the
 * MAC moves them between the FIFOs and the PHY over MII or RMII. The
 * descriptors are the GMAC's enhanced ones, 16 bytes, or 32 with
 * DMABUSMODE.ALT_DESC_SIZE, which adds RDES4's extended receive status;
 * lists are rings (contiguous, with DESC_SKIP_LEN words between entries,
 * and an end-of-ring bit) or chains (each descriptor's third address
 * word points at the next).
 *
 * The PHY hangs off the MAC's MDIO bus, and the network backend is its
 * medium: transmitted frames leave through the PHY at the line rate set
 * by EMACCONFIG.FES, and the backend's frames arrive padded to the
 * minimum size, with their FCS. Frames carry their FCS inside the model,
 * so loopback, CRC errors and FCS stripping act on the wire's bytes.
 *
 * The MAC's transmit and receive paths run on the MII clocks the wrapper
 * enables: TX_CLK and RX_CLK in MII mode, the 50 MHz reference in RMII
 * mode. Without them frames do not move, and a software reset, which must
 * reach every clock domain, does not finish.
 *
 * Not modelled: the IEEE 1588 timestamp engine (the TRM documents none of
 * its registers, so TDES0.TTSE has no effect), VLAN tag insertion (the
 * TRM documents no VLAN tag register), remote wake-up frame filters, the
 * energy-efficient Ethernet LPI states, collisions and the half-duplex
 * backoff, and automatic pause frames when the receive FIFO fills.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/net/esp32_emac.h"
#include "migration/vmstate.h"
#include "net/checksum.h"
#include "net/eth.h"
#include <zlib.h>

/* DMA */
#define A_BUS_MODE          0x0000
#define A_TX_POLL           0x0004
#define A_RX_POLL           0x0008
#define A_RX_BASE           0x000c
#define A_TX_BASE           0x0010
#define A_STATUS            0x0014
#define A_OP_MODE           0x0018
#define A_INT_EN            0x001c
#define A_MISSED            0x0020
#define A_RX_WDT            0x0024
#define A_TX_CUR_DESC       0x0048
#define A_RX_CUR_DESC       0x004c
#define A_TX_CUR_BUF        0x0050
#define A_RX_CUR_BUF        0x0054

/* Clock and PHY interface wrapper */
#define A_EX_CLKOUT_CONF    0x0800
#define A_EX_OSCCLK_CONF    0x0804
#define A_EX_CLK_CTRL       0x0808
#define A_EX_PHYINF_CONF    0x080c
#define A_EX_PD_SEL         0x0810
#define A_EX_DATE           0x08fc

/* MAC */
#define A_CONFIG            0x1000
#define A_FRAME_FILTER      0x1004
#define A_MII_ADDR          0x1010
#define A_MII_DATA          0x1014
#define A_FLOW_CTRL         0x1018
#define A_DEBUG             0x1024
#define A_RWUFFR            0x1028
#define A_PMT_CSR           0x102c
#define A_LPI_CSR           0x1030
#define A_LPI_TIMERS        0x1034
#define A_INTS              0x1038
#define A_INT_MASK          0x103c
#define A_ADDR0_HIGH        0x1040
#define A_ADDR7_LOW         0x107c
#define A_CSTATUS           0x10d8
#define A_WDOG_TO           0x10dc

/* DMABUSMODE */
#define BUS_MODE_SWR        (1u << 0)
#define BUS_MODE_DSL_SHIFT  2
#define BUS_MODE_DSL_MASK   0x1f
#define BUS_MODE_ATDS       (1u << 7)
#define BUS_MODE_RW         0x07ffffffu
#define BUS_MODE_RESET      0x00020100u

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
 * Transmit process states. The TRM's table repeats the receive states
 * here; the encoding is the DesignWare GMAC's.
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
#define CFG_2KPE            (1u << 27)
#define CFG_SARC_SHIFT      28
#define CFG_RW              0x78dffeffu

/* EMACFF */
#define FF_PR               (1u << 0)
#define FF_DAIF             (1u << 3)
#define FF_PM               (1u << 4)
#define FF_DBF              (1u << 5)
#define FF_PCF_SHIFT        6
#define FF_SAIF             (1u << 8)
#define FF_SAF              (1u << 9)
#define FF_RA               (1u << 31)
#define FF_RW               0x800003f9u

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
#define FC_PT_SHIFT         16
#define FC_RW               0xffff00bfu

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
#define LPI_RW              0x000b0300u
#define LPI_TIMERS_RW       0x03ffffffu
#define LPI_TIMERS_RESET    0x03e80000u

/* EMACINTS and EMACINTMASK */
#define INTS_PMT            (1u << 3)
#define INT_MASK_RW         0x00000408u

/* EMACADDRnHIGH */
#define ADDR_HIGH_MBC_SHIFT 24
#define ADDR_HIGH_SA        (1u << 30)
#define ADDR_HIGH_AE        (1u << 31)
#define ADDR0_HIGH_RW       0x0000ffffu
#define ADDRN_HIGH_RW       0xff00ffffu

#define WDOG_TO_PWE         (1u << 16)
#define WDOG_TO_RW          0x00013fffu

/* EMAC_EX_CLK_CTRL */
#define EX_CLK_EXT_EN       (1u << 0)
#define EX_CLK_INT_EN       (1u << 1)
#define EX_CLK_MII_TX_EN    (1u << 3)
#define EX_CLK_MII_RX_EN    (1u << 4)
/* EMAC_EX_PHYINF_CONF */
#define EX_PHY_INTF_SHIFT   13
#define EX_PHY_INTF_MII     0
#define EX_PHY_INTF_RMII    4

#define EX_CLKOUT_RESET     0x00000024u
#define EX_CLKOUT_RW        0x000003ffu
#define EX_OSCCLK_RESET     0x00001253u
#define EX_OSCCLK_RW        0x01ffffffu
#define EX_CLK_CTRL_RW      0x0000003fu
#define EX_PHYINF_RW        0x001fffffu
#define EX_PD_SEL_RW        0x00000003u
#define EX_DATE_RESET       0x16042200u

/* Transmit descriptor */
#define TDES0_OWN           (1u << 31)
#define TDES0_IC            (1u << 30)
#define TDES0_LS            (1u << 29)
#define TDES0_FS            (1u << 28)
#define TDES0_DC            (1u << 27)
#define TDES0_DP            (1u << 26)
#define TDES0_CRCR          (1u << 24)
#define TDES0_CIC_SHIFT     22
#define TDES0_TER           (1u << 21)
#define TDES0_TCH           (1u << 20)
#define TDES0_VLIC_SHIFT    18
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

/* The SRAM the EMAC's AHB master reaches: internal SRAM 1 and 2 */
#define DMA_RAM_START       0x3ffae000u
#define DMA_RAM_END         0x40000000u

#define ETH_HLEN            14
#define ETH_FCS_LEN         4
#define ETH_MIN_LEN         60
#define ETH_TYPE_VLAN       0x8100
#define ETH_TYPE_CONTROL    0x8808
/* Smallest Length/Type value that is a type, not a length */
#define ETH_TYPE_MIN        0x0600

/* Bound on the descriptors walked for one frame, against owned loops */
#define DESC_WALK_LIMIT     4096

static const uint8_t pause_da[ETH_ALEN] = {
    0x01, 0x80, 0xc2, 0x00, 0x00, 0x01
};

static void emac_tx_dma(Esp32EmacState *s);
static void emac_rx_dma(Esp32EmacState *s);
static void emac_tx_kick(Esp32EmacState *s);
static void emac_mac_rx(Esp32EmacState *s, const uint8_t *frame, size_t len);

static bool emac_live(Esp32EmacState *s)
{
    return !clock_has_source(s->apb_clk) || clock_get_hz(s->apb_clk) != 0;
}

/* The MII transmit and receive clocks the wrapper passes to the MAC */
static bool emac_tx_clk(Esp32EmacState *s)
{
    switch ((s->ex_phyinf_conf >> EX_PHY_INTF_SHIFT) & 7) {
    case EX_PHY_INTF_MII:
        return s->ex_clk_ctrl & EX_CLK_MII_TX_EN;
    case EX_PHY_INTF_RMII:
        return s->ex_clk_ctrl & (EX_CLK_EXT_EN | EX_CLK_INT_EN);
    default:
        return false;
    }
}

static bool emac_rx_clk(Esp32EmacState *s)
{
    switch ((s->ex_phyinf_conf >> EX_PHY_INTF_SHIFT) & 7) {
    case EX_PHY_INTF_MII:
        return s->ex_clk_ctrl & EX_CLK_MII_RX_EN;
    case EX_PHY_INTF_RMII:
        return s->ex_clk_ctrl & (EX_CLK_EXT_EN | EX_CLK_INT_EN);
    default:
        return false;
    }
}

static uint32_t emac_pmt_ints(Esp32EmacState *s)
{
    return (s->pmt_csr & (PMT_MGKPRCVD | PMT_RWKPRCVD)) ? INTS_PMT : 0;
}

/*
 * [spec:nuos:req:emu.esp32.emac]
 * The summary bits latch when an enabled status bit is set; the line is
 * raised by an enabled summary, or by the MAC's PMT interrupt unless
 * EMACINTMASK masks it.
 */
static void emac_update_irq(Esp32EmacState *s)
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
            (emac_pmt_ints(s) & ~s->int_mask);
    qemu_set_irq(s->irq, level);
}

static void emac_raise(Esp32EmacState *s, uint32_t bits)
{
    s->status |= bits;
    if (bits & ST_RI) {
        s->status &= ~ST_ERI;
    }
    emac_update_irq(s);
}

/*
 * [spec:nuos:req:emu.esp32.emac]
 * A DMA access outside the SRAM the AHB master reaches is answered with
 * an error: the engine that made it sets FBI with the access's type in
 * ERROR_BITS and stops using the bus until a software reset.
 */
static void emac_bus_error(Esp32EmacState *s, uint32_t addr, unsigned code)
{
    bool tx = code == EB_TX_DATA || code == EB_TX_DESC_WRITE ||
              code == EB_TX_DESC_READ;

    qemu_log_mask(LOG_GUEST_ERROR,
                  "esp32.emac: %s DMA access at 0x%08" PRIx32
                  " outside internal SRAM\n", tx ? "transmit" : "receive",
                  addr);
    s->status = (s->status & ~(7u << ST_EB_SHIFT)) | (code << ST_EB_SHIFT);
    if (tx) {
        s->tx_fatal = true;
        s->tx_state = TS_STOPPED;
        s->tx_gathering = false;
    } else {
        s->rx_fatal = true;
        s->rx_state = RS_STOPPED;
    }
    emac_raise(s, ST_FBI);
}

static bool emac_dma_ok(uint32_t addr, uint32_t len)
{
    return addr >= DMA_RAM_START && addr < DMA_RAM_END &&
           len <= DMA_RAM_END - addr;
}

static bool emac_dma_read(Esp32EmacState *s, uint32_t addr, void *buf,
                          uint32_t len, unsigned code)
{
    if (len == 0) {
        return true;
    }
    if (!emac_dma_ok(addr, len)) {
        emac_bus_error(s, addr, code);
        return false;
    }
    address_space_read(&s->dma_as, addr, MEMTXATTRS_UNSPECIFIED, buf, len);
    return true;
}

static bool emac_dma_write(Esp32EmacState *s, uint32_t addr,
                           const void *buf, uint32_t len, unsigned code)
{
    if (len == 0) {
        return true;
    }
    if (!emac_dma_ok(addr, len)) {
        emac_bus_error(s, addr, code);
        return false;
    }
    address_space_write(&s->dma_as, addr, MEMTXATTRS_UNSPECIFIED, buf, len);
    return true;
}

static bool emac_desc_read(Esp32EmacState *s, uint32_t addr, uint32_t *d,
                           unsigned words, unsigned code)
{
    if (!emac_dma_read(s, addr, d, words * 4, code)) {
        return false;
    }
    for (unsigned i = 0; i < words; i++) {
        d[i] = le32_to_cpu(d[i]);
    }
    return true;
}

static bool emac_desc_write_word(Esp32EmacState *s, uint32_t addr,
                                 uint32_t v, unsigned code)
{
    uint32_t le = cpu_to_le32(v);

    return emac_dma_write(s, addr, &le, 4, code);
}

/*
 * The descriptor after one: the list's base at the end of a ring, the
 * chained address, or the next ring entry DESC_SKIP_LEN words on.
 */
static uint32_t emac_next_desc(Esp32EmacState *s, uint32_t addr,
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

static uint32_t emac_fcs(const uint8_t *frame, size_t len)
{
    return crc32(0, frame, len);
}

static bool emac_fcs_ok(const uint8_t *frame, size_t len)
{
    return len >= ETH_FCS_LEN &&
           emac_fcs(frame, len - ETH_FCS_LEN) ==
           ldl_le_p(frame + len - ETH_FCS_LEN);
}

static void emac_addr(Esp32EmacState *s, unsigned n, uint8_t *mac)
{
    stl_le_p(mac, s->addr_low[n]);
    stw_le_p(mac + 4, s->addr_high[n] & 0xffff);
}

/* Bit time in ns at the speed EMACCONFIG.FES selects */
static int64_t emac_bit_ns(Esp32EmacState *s)
{
    return (s->config & CFG_FES) ? 10 : 100;
}

/*
 * The time a frame of len bytes, FCS included, holds the wire: preamble
 * and SFD (shortened by PRELEN in full duplex), the frame, and the
 * interframe gap EMACCONFIG.IFG sets.
 */
static int64_t emac_wire_ns(Esp32EmacState *s, size_t len)
{
    unsigned preamble = 8;
    unsigned ifg_bits = 96 - 8 * ((s->config >> CFG_IFG_SHIFT) & 7);

    if (s->config & CFG_DM) {
        static const unsigned prelen[] = { 8, 6, 4, 8 };

        preamble = prelen[s->config & CFG_PRELEN_MASK];
    } else if (ifg_bits < 64) {
        ifg_bits = 64;
    }
    return ((preamble + len) * 8 + ifg_bits) * emac_bit_ns(s);
}

/* Checksum offload */

static uint32_t emac_pseudo_sum(const uint8_t *addrs, unsigned addr_len,
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
static int emac_l4_csum_off(uint8_t proto, uint32_t len)
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

static unsigned emac_l3_off(const uint8_t *frame, size_t len, uint16_t *type)
{
    unsigned off = 12;

    if (len < ETH_HLEN) {
        *type = 0;
        return len;
    }
    *type = lduw_be_p(frame + off);
    if (*type == ETH_TYPE_VLAN && len >= ETH_HLEN + 4) {
        off += 4;
        *type = lduw_be_p(frame + off);
    }
    return off + 2;
}

/*
 * [spec:nuos:req:emu.esp32.emac]
 * Transmit checksum insertion, TDES0.CIC: 1 inserts the IPv4 header
 * checksum; 2 also the TCP, UDP or ICMP checksum, summing the payload
 * over the pseudo-header sum software left in the checksum field; 3
 * computes the pseudo-header sum too. Returns IHE and IPE status bits.
 */
static uint32_t emac_tx_csum(uint8_t *frame, size_t len, unsigned cic)
{
    uint16_t type;
    unsigned l3 = emac_l3_off(frame, len, &type);
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
        sum = proto == 1 ? 0 : emac_pseudo_sum(ip + 12, 4, proto, plen);
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
        sum = emac_pseudo_sum(ip + 8, 16, proto, plen);
    } else {
        return 0;
    }

    csum_off = emac_l4_csum_off(proto, plen);
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
 * The receive checksum engine (EMACCONFIG.IPC) with the GMAC's full
 * offload: IPv4 header and TCP, UDP or ICMP payload checksums over IPv4
 * and IPv6, reported in RDES4. Returns RDES4.
 */
static uint32_t emac_rx_csum(const uint8_t *frame, size_t len)
{
    uint16_t type;
    unsigned l3 = emac_l3_off(frame, len, &type);
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
        sum = proto == 1 ? 0 : emac_pseudo_sum(ip + 12, 4, proto, plen);
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
        sum = emac_pseudo_sum(ip + 8, 16, proto, plen);
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
    csum_off = emac_l4_csum_off(proto, plen);
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

/* Transmit */

/*
 * [spec:nuos:req:emu.esp32.emac]
 * A frame leaving the MAC: back to its own receiver in MAC loopback,
 * through the PHY otherwise. The PHY returns it in its own loopback, and
 * puts it on the medium when the link is up; the link partner discards
 * runts and frames with a bad FCS.
 */
static void emac_wire_out(Esp32EmacState *s, const uint8_t *wire, size_t len)
{
    NetClientState *nc = qemu_get_queue(s->nic);

    if (s->config & CFG_LM) {
        if (emac_rx_clk(s)) {
            emac_mac_rx(s, wire, len);
        }
        return;
    }
    if (!s->phy || ip101_phy_isolated(s->phy)) {
        return;
    }
    if (ip101_phy_loopback(s->phy)) {
        if (emac_rx_clk(s) &&
            ((s->config & CFG_DM) || !(s->config & CFG_DO))) {
            emac_mac_rx(s, wire, len);
        }
        return;
    }
    if (!ip101_phy_link_up(s->phy)) {
        return;
    }
    if (len < ETH_MIN_LEN + ETH_FCS_LEN || !emac_fcs_ok(wire, len)) {
        return;
    }
    qemu_send_packet(nc, wire, len - ETH_FCS_LEN);
}

static bool emac_tx_can_send(Esp32EmacState *s)
{
    return emac_live(s) && (s->config & CFG_TE) && emac_tx_clk(s);
}

/* The source address control: EMACCONFIG.SARC, or else TDES1.SAIC */
static void emac_tx_source_addr(Esp32EmacState *s, uint8_t *frame,
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
        return;
    }
    if (op == 3) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32.emac: reserved TDES1.SAIC value\n");
        return;
    }
    emac_addr(s, which, mac);
    if (op == 1) {
        if (*len + ETH_ALEN > ESP32_EMAC_MAX_FRAME || *len < ETH_ALEN) {
            return;
        }
        memmove(frame + 2 * ETH_ALEN, frame + ETH_ALEN, *len - ETH_ALEN);
        *len += ETH_ALEN;
    } else if (*len < 2 * ETH_ALEN) {
        return;
    }
    memcpy(frame + ETH_ALEN, mac, ETH_ALEN);
}

/*
 * Turn the gathered frame into the bytes the wire carries and start
 * sending them; the frame has left, and reached a receiver, when the wire
 * time is over.
 */
static void emac_tx_frame_out(Esp32EmacState *s)
{
    uint32_t tdes0 = s->tx_tdes0;
    size_t len = s->tx_len;
    uint8_t *f = s->tx_frame;
    uint32_t status = 0;
    unsigned cic = (tdes0 >> TDES0_CIC_SHIFT) & 3;
    size_t jabber_max = (s->config & CFG_JE) ? 10240 : 2048;
    bool add_crc;
    size_t wire_len;

    emac_tx_source_addr(s, f, &len);
    if ((tdes0 >> TDES0_VLIC_SHIFT) & 3) {
        qemu_log_mask(LOG_UNIMP, "esp32.emac: VLAN tag insertion\n");
    }
    if (cic) {
        status |= emac_tx_csum(f, len, cic);
    }
    if (len >= ETH_HLEN && lduw_be_p(f + 12) == ETH_TYPE_VLAN) {
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
        stl_le_p(f + wire_len, emac_fcs(f, wire_len));
        wire_len += ETH_FCS_LEN;
    } else if ((tdes0 & TDES0_CRCR) && wire_len >= ETH_FCS_LEN) {
        stl_le_p(f + wire_len - ETH_FCS_LEN,
                 emac_fcs(f, wire_len - ETH_FCS_LEN));
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
    s->tx_status = status;
    s->tx_wire_len = wire_len;
    s->tx_on_wire = true;
    s->tx_state = TS_WAITING;
    timer_mod(&s->tx_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              emac_wire_ns(s, wire_len));
}

/*
 * [spec:nuos:req:emu.esp32.emac]
 * Send a pause frame for EMACFC.FCB: to the reserved multicast address,
 * from MAC address 0, with EMACFC's pause time.
 */
static void emac_tx_pause_out(Esp32EmacState *s)
{
    s->pause_on_wire = true;
    s->tx_on_wire = true;
    timer_mod(&s->tx_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              emac_wire_ns(s, ETH_MIN_LEN + ETH_FCS_LEN));
}

static void emac_tx_pause_done(Esp32EmacState *s)
{
    uint8_t wire[ETH_MIN_LEN + ETH_FCS_LEN] = { 0 };

    memcpy(wire, pause_da, ETH_ALEN);
    emac_addr(s, 0, wire + ETH_ALEN);
    stw_be_p(wire + 12, ETH_TYPE_CONTROL);
    stw_be_p(wire + 14, 1);
    stw_be_p(wire + 16, s->flow_ctrl >> FC_PT_SHIFT);
    stl_le_p(wire + ETH_MIN_LEN, emac_fcs(wire, ETH_MIN_LEN));
    emac_wire_out(s, wire, sizeof(wire));
}

/* Start whatever the MAC has to send, if the wire is free. */
static void emac_tx_kick(Esp32EmacState *s)
{
    if (s->tx_on_wire || !emac_tx_can_send(s)) {
        return;
    }
    if (s->pause_pending) {
        s->pause_pending = false;
        emac_tx_pause_out(s);
        return;
    }
    if (s->tx_ready && !s->paused) {
        emac_tx_frame_out(s);
    }
}

/*
 * Close the descriptor holding a frame's last segment with its status,
 * raising TI if it asked for an interrupt on completion.
 */
static void emac_tx_close(Esp32EmacState *s, uint32_t status)
{
    uint32_t tdes0;

    s->tx_state = TS_CLOSING;
    if (!emac_desc_read(s, s->tx_last_desc, &tdes0, 1, EB_TX_DESC_READ)) {
        return;
    }
    tdes0 = (tdes0 & ~(TDES0_OWN | TDES0_STATUS)) | status;
    if (!emac_desc_write_word(s, s->tx_last_desc, tdes0, EB_TX_DESC_WRITE)) {
        return;
    }
    if (tdes0 & TDES0_IC) {
        emac_raise(s, ST_TI);
    }
}

/* Leave the running state when DMAOPERATION_MODE.ST has been cleared. */
static bool emac_tx_maybe_stop(Esp32EmacState *s)
{
    if (s->op_mode & OP_ST) {
        return false;
    }
    s->tx_gathering = false;
    if (s->tx_state != TS_STOPPED) {
        s->tx_state = TS_STOPPED;
        emac_raise(s, ST_TPS);
    }
    return true;
}

static void emac_tx_timer_cb(void *opaque)
{
    Esp32EmacState *s = opaque;

    s->tx_on_wire = false;
    if (s->pause_on_wire) {
        s->pause_on_wire = false;
        s->flow_ctrl &= ~FC_FCB;
        emac_tx_pause_done(s);
    } else if (s->tx_ready) {
        s->tx_ready = false;
        if (!s->tx_jabber) {
            emac_wire_out(s, s->tx_frame, s->tx_wire_len);
        }
        emac_tx_close(s, s->tx_status);
        if (s->tx_jabber) {
            /* A jabber timeout stops the transmit process. */
            s->tx_jabber = false;
            s->tx_gathering = false;
            if (!s->tx_fatal) {
                s->tx_state = TS_STOPPED;
            }
            emac_raise(s, ST_TJT | ST_TPS);
            emac_tx_kick(s);
            return;
        }
        if (!s->tx_fatal && !emac_tx_maybe_stop(s)) {
            s->tx_state = TS_FETCHING;
        }
    }
    emac_tx_kick(s);
    emac_tx_dma(s);
}

/*
 * [spec:nuos:req:emu.esp32.emac]
 * The transmit DMA: gather a frame from the descriptors the DMA owns,
 * first segment to last, clearing OWN in each as its buffers are read,
 * and hand it to the MTL FIFO. A descriptor the host owns suspends the
 * process with TU; in the middle of a frame, past the FIFO's threshold in
 * threshold mode, the MAC has already started and the frame underflows.
 */
static void emac_tx_dma(Esp32EmacState *s)
{
    static const uint32_t ttc_bytes[] = { 64, 128, 192, 256, 40, 32, 24, 16 };

    for (unsigned n = 0; n < DESC_WALK_LIMIT; n++) {
        uint32_t d[4];
        uint32_t addr = s->tx_desc;
        uint32_t b1, b2, next;

        if (s->tx_fatal || s->tx_state == TS_STOPPED || s->tx_ready ||
            !emac_live(s) || emac_tx_maybe_stop(s)) {
            return;
        }
        s->tx_state = TS_FETCHING;
        if (!emac_desc_read(s, addr, d, 4, EB_TX_DESC_READ)) {
            return;
        }
        if (!(d[0] & TDES0_OWN)) {
            uint32_t ttc = ttc_bytes[(s->op_mode >> OP_TTC_SHIFT) & 7];

            if (s->tx_gathering && !(s->op_mode & OP_TSF) &&
                s->tx_len >= ttc) {
                s->tx_gathering = false;
                emac_tx_close(s, TDES0_UF | TDES0_ES);
                emac_raise(s, ST_UNF | ST_TI);
            }
            if (s->tx_state != TS_SUSPENDED) {
                s->tx_state = TS_SUSPENDED;
                emac_raise(s, ST_TU);
            }
            return;
        }
        if (!s->tx_gathering || (d[0] & TDES0_FS)) {
            if (!(d[0] & TDES0_FS)) {
                qemu_log_mask(LOG_GUEST_ERROR, "esp32.emac: transmit "
                              "descriptor 0x%08" PRIx32 " starts a frame "
                              "without TDES0.FS\n", addr);
            } else if (s->tx_gathering) {
                qemu_log_mask(LOG_GUEST_ERROR, "esp32.emac: transmit "
                              "descriptor 0x%08" PRIx32 " has TDES0.FS in "
                              "the middle of a frame\n", addr);
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
            uint32_t room = ESP32_EMAC_MAX_FRAME - s->tx_len;
            uint32_t take = MIN(want, room);

            if (!want) {
                continue;
            }
            if (!emac_dma_read(s, buf, s->tx_frame + s->tx_len, take,
                               EB_TX_DATA)) {
                return;
            }
            s->tx_buf = buf;
            s->tx_len += take;
            if (take < want) {
                s->tx_jabber = true;
            }
        }

        next = emac_next_desc(s, addr, d[0] & TDES0_TER, d[0] & TDES0_TCH,
                              d[3], s->tx_base);
        s->tx_desc = next;
        if (d[0] & TDES0_LS) {
            s->tx_last_desc = addr;
            s->tx_gathering = false;
            s->tx_ready = true;
            emac_raise(s, ST_ETI);
            emac_tx_kick(s);
            return;
        }
        if (!emac_desc_write_word(s, addr, d[0] & ~TDES0_OWN,
                                  EB_TX_DESC_WRITE)) {
            return;
        }
    }
    qemu_log_mask(LOG_GUEST_ERROR,
                  "esp32.emac: transmit frame spans too many descriptors\n");
    s->tx_gathering = false;
}

/* Flush the transmit FIFO: a frame waiting there closes as flushed. */
static void emac_tx_flush(Esp32EmacState *s)
{
    if (s->tx_ready && !s->tx_on_wire) {
        s->tx_ready = false;
        emac_tx_close(s, TDES0_FF | TDES0_ES);
        if (!s->tx_fatal && !emac_tx_maybe_stop(s)) {
            s->tx_state = TS_FETCHING;
        }
        emac_tx_dma(s);
    }
}

/* Receive */

static bool emac_is_multicast(const uint8_t *a)
{
    return a[0] & 1;
}

static bool emac_is_broadcast(const uint8_t *a)
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
static int emac_perfect_match(Esp32EmacState *s, const uint8_t *addr,
                              bool source)
{
    for (unsigned n = source ? 1 : 0; n < ESP32_EMAC_ADDR_COUNT; n++) {
        uint32_t high = s->addr_high[n];
        uint32_t mbc = n ? (high >> ADDR_HIGH_MBC_SHIFT) & 0x3f : 0;
        uint8_t mac[ETH_ALEN];
        bool match = true;

        if (n && (!(high & ADDR_HIGH_AE) ||
                  !!(high & ADDR_HIGH_SA) != source)) {
            continue;
        }
        emac_addr(s, n, mac);
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
 * Power-down mode drops every frame; a magic packet (six 0xff bytes, then
 * MAC address 0 sixteen times) or, with GLBLUCAST, a unicast frame passing
 * the destination filter wakes the MAC when enabled.
 */
static void emac_pmt_check(Esp32EmacState *s, const uint8_t *f, size_t len,
                           bool da_pass)
{
    uint8_t mac[ETH_ALEN];

    if ((s->pmt_csr & PMT_RWKPKTEN) && (s->pmt_csr & PMT_GLBLUCAST) &&
        !emac_is_multicast(f) && da_pass) {
        s->pmt_csr = (s->pmt_csr | PMT_RWKPRCVD) & ~PMT_PWRDWN;
    }
    if (s->pmt_csr & PMT_MGKPKTEN) {
        emac_addr(s, 0, mac);
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
                      "esp32.emac: remote wake-up frame filters\n");
    }
    emac_update_irq(s);
}

/* A received pause frame stops the transmitter for its pause time. */
static void emac_rx_pause(Esp32EmacState *s, const uint8_t *f)
{
    uint16_t quanta = lduw_be_p(f + 16);

    if (quanta == 0) {
        s->paused = false;
        timer_del(&s->pause_timer);
        emac_tx_kick(s);
        return;
    }
    s->paused = true;
    timer_mod(&s->pause_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              (int64_t)quanta * 512 * emac_bit_ns(s));
}

static void emac_pause_timer_cb(void *opaque)
{
    Esp32EmacState *s = opaque;

    s->paused = false;
    emac_tx_kick(s);
}

/*
 * Take a frame from the head of the receive FIFO into the descriptors.
 * Returns false if the receive process is suspended for want of one.
 */
static bool emac_rx_dma_frame(Esp32EmacState *s, const uint8_t *frame,
                              uint32_t len, uint32_t status, uint32_t rdes4)
{
    uint32_t addr = s->rx_desc;
    uint32_t d[4];
    uint32_t pos = 0;
    bool first = true;

    if (!emac_desc_read(s, addr, d, 4, EB_RX_DESC_READ)) {
        return true;
    }
    if (!(d[0] & RDES0_OWN)) {
        if (s->rx_state != RS_SUSPENDED) {
            s->rx_state = RS_SUSPENDED;
            emac_raise(s, ST_RU);
        }
        return false;
    }
    s->rx_state = RS_WAITING;

    for (unsigned n = 0; n < DESC_WALK_LIMIT; n++) {
        uint32_t b1 = d[1] & TDES_BS_MASK;
        uint32_t b2 = (d[1] & RDES1_RCH) ? 0 :
                      (d[1] >> RDES1_RBS2_SHIFT) & TDES_BS_MASK;
        uint32_t next, rdes0, nd;

        if ((b1 | b2) & 3) {
            qemu_log_mask(LOG_GUEST_ERROR, "esp32.emac: receive buffer "
                          "size not a multiple of 4 in descriptor 0x%08"
                          PRIx32 "\n", addr);
        }
        for (unsigned i = 0; i < 2; i++) {
            uint32_t n2 = MIN(i ? b2 : b1, len - pos);

            if (!n2) {
                continue;
            }
            if (!emac_dma_write(s, d[2 + i], frame + pos, n2, EB_RX_DATA)) {
                return true;
            }
            s->rx_buf = d[2 + i];
            pos += n2;
            if (first && i == 0 && pos < len) {
                emac_raise(s, ST_ERI);
            }
        }
        next = emac_next_desc(s, addr, d[1] & RDES1_RER, d[1] & RDES1_RCH,
                              d[3], s->rx_base);

        if (pos == len) {
            rdes0 = status | RDES0_LS | (first ? RDES0_FS : 0) |
                    (len << RDES0_FL_SHIFT);
        } else {
            if (!emac_desc_read(s, next, &nd, 1, EB_RX_DESC_READ)) {
                return true;
            }
            if (nd & RDES0_OWN) {
                if (!emac_desc_write_word(s, addr, first ? RDES0_FS : 0,
                                          EB_RX_DESC_WRITE)) {
                    return true;
                }
                addr = next;
                first = false;
                if (!emac_desc_read(s, addr, d, 4, EB_RX_DESC_READ)) {
                    return true;
                }
                continue;
            }
            /* No descriptor for the rest: truncate the frame here. */
            rdes0 = (status & ~RDES0_ESA) | RDES0_LS | RDES0_DE |
                    RDES0_ES | (first ? RDES0_FS : 0) |
                    (pos << RDES0_FL_SHIFT);
        }

        if ((s->bus_mode & BUS_MODE_ATDS) && (s->config & CFG_IPC) &&
            (rdes0 & RDES0_ESA) &&
            !emac_desc_write_word(s, addr + 16, rdes4, EB_RX_DESC_WRITE)) {
            return true;
        }
        if (!emac_desc_write_word(s, addr, rdes0, EB_RX_DESC_WRITE)) {
            return true;
        }
        s->rx_desc = next;
        if (d[1] & RDES1_DIC) {
            uint32_t riwt = s->rx_wdt & 0xff;

            if (riwt && clock_get_hz(s->apb_clk)) {
                timer_mod(&s->rx_wdt_timer,
                          qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                          muldiv64(riwt * 256, NANOSECONDS_PER_SECOND,
                                   clock_get_hz(s->apb_clk)));
            }
        } else {
            timer_del(&s->rx_wdt_timer);
            emac_raise(s, ST_RI);
        }
        if (pos < len && s->rx_state != RS_SUSPENDED) {
            s->rx_state = RS_SUSPENDED;
            emac_raise(s, ST_RU);
        }
        return true;
    }
    qemu_log_mask(LOG_GUEST_ERROR,
                  "esp32.emac: receive frame spans too many descriptors\n");
    return true;
}

static void emac_rx_wdt_cb(void *opaque)
{
    Esp32EmacState *s = opaque;

    emac_raise(s, ST_RI);
}

static void emac_rx_fifo_pop(Esp32EmacState *s)
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
    s->rx_fifo_count = n;
}

/*
 * [spec:nuos:req:emu.esp32.emac]
 * The receive DMA drains the FIFO into the descriptor list. Without a
 * descriptor the process suspends with RU, and unless DFF is set the
 * frame at the FIFO's head is flushed and counted as missed.
 */
static void emac_rx_dma(Esp32EmacState *s)
{
    while (s->rx_fifo_count && !s->rx_fatal && s->rx_state != RS_STOPPED &&
           (s->op_mode & OP_SR) && emac_live(s)) {
        if (!emac_rx_dma_frame(s, s->rx_fifo, s->rx_fifo_len[0],
                               s->rx_fifo_rdes0[0], s->rx_fifo_rdes4[0])) {
            if (!(s->op_mode & OP_DFF)) {
                emac_rx_fifo_pop(s);
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
        emac_rx_fifo_pop(s);
    }
}

static void emac_rx_overflow(Esp32EmacState *s)
{
    if (s->overflow_frames == OVERFLOW_FC_MAX) {
        s->overflow_frames = 0;
        s->fifo_overflow = true;
    } else {
        s->overflow_frames++;
    }
    emac_raise(s, ST_OVF);
}

/*
 * [spec:nuos:req:emu.esp32.emac]
 * The MAC's receive path for a frame off the MII, FCS included: the
 * power-management and pause-frame checks, the address filter, the
 * receive status, the dropping of frames the FIFO configuration rejects,
 * and the FIFO.
 */
static void emac_mac_rx(Esp32EmacState *s, const uint8_t *f, size_t len)
{
    uint32_t ff = s->frame_filter;
    uint32_t status = 0;
    uint32_t rdes4 = 0;
    uint16_t type;
    bool vlan, da_pass, sa_pass, control, pause, deliver, csum_err;
    int da_match, sa_match;
    size_t data_len = len;
    size_t giant, wdog;

    if (!emac_live(s) || !(s->config & CFG_RE) || len < ETH_HLEN) {
        return;
    }
    type = lduw_be_p(f + 12);
    vlan = type == ETH_TYPE_VLAN;

    /* Destination address filter */
    da_match = emac_perfect_match(s, f, false);
    if (emac_is_broadcast(f)) {
        da_pass = !(ff & FF_DBF);
    } else if (emac_is_multicast(f)) {
        da_pass = (ff & FF_PM) || ((da_match >= 0) != !!(ff & FF_DAIF));
    } else {
        da_pass = (da_match >= 0) != !!(ff & FF_DAIF);
    }
    /* Source address filter, against registers in source mode */
    sa_match = emac_perfect_match(s, f + ETH_ALEN, true);
    sa_pass = (sa_match >= 0) != !!(ff & FF_SAIF);
    if (ff & FF_PR) {
        da_pass = sa_pass = true;
    }

    if (s->pmt_csr & PMT_PWRDWN) {
        emac_pmt_check(s, f, len, da_pass);
        return;
    }

    control = type == ETH_TYPE_CONTROL;
    pause = control && len == ETH_MIN_LEN + ETH_FCS_LEN &&
            lduw_be_p(f + 14) == 1 && emac_fcs_ok(f, len);
    if (pause && (s->flow_ctrl & FC_RFE) && (s->config & CFG_DM)) {
        uint8_t mac[ETH_ALEN];

        emac_addr(s, 0, mac);
        if (!memcmp(f, pause_da, ETH_ALEN) ||
            ((s->flow_ctrl & FC_UP) && !memcmp(f, mac, ETH_ALEN))) {
            emac_rx_pause(s, f);
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
    if (!deliver) {
        return;
    }

    if (!da_pass) {
        status |= RDES0_AFM;
    }
    if (!sa_pass) {
        status |= RDES0_SAF;
    }
    if (!emac_fcs_ok(f, len)) {
        status |= RDES0_CE;
    }
    if (type >= ETH_TYPE_MIN) {
        status |= RDES0_FT;
    }
    if (vlan) {
        status |= RDES0_VLAN;
    }

    giant = (s->config & CFG_JE) ? (vlan ? 9022 : 9018) :
            (s->config & CFG_2KPE) ? 2000 : (vlan ? 1522 : 1518);
    if (len > giant) {
        status |= RDES0_GF;
    }
    if (s->config & CFG_WD) {
        wdog = ESP32_EMAC_MAX_FRAME;
    } else if (s->wdog_to & WDOG_TO_PWE) {
        wdog = s->wdog_to & 0x3fff;
    } else {
        wdog = (s->config & CFG_JE) ? 10240 : 2048;
    }
    if (data_len > wdog) {
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
    }

    csum_err = false;
    if (s->config & CFG_IPC) {
        rdes4 = emac_rx_csum(f, len - ETH_FCS_LEN);
        status |= RDES0_ESA;
        csum_err = rdes4 & (RDES4_IPHE | RDES4_IPPE);
    } else if (da_match > 0) {
        status |= RDES0_ESA;
    }
    if (status & (RDES0_CE | RDES0_RWT | RDES0_GF) || csum_err) {
        status |= RDES0_ES;
    }

    /* The FIFO's drop rules */
    if (!(s->op_mode & OP_FEF)) {
        if (status & (RDES0_CE | RDES0_RWT | RDES0_GF)) {
            return;
        }
        if (csum_err && !(s->op_mode & OP_DT)) {
            return;
        }
    }
    if ((status & RDES0_GF) && (s->op_mode & OP_DGF)) {
        return;
    }
    if (len < ETH_MIN_LEN + ETH_FCS_LEN && !(status & RDES0_ES) &&
        !(s->op_mode & OP_FUF)) {
        return;
    }

    if (data_len > ESP32_EMAC_RX_FIFO_SIZE - s->rx_fifo_used ||
        s->rx_fifo_count == ESP32_EMAC_RX_FIFO_MAX_FRAMES) {
        if (data_len > ESP32_EMAC_RX_FIFO_SIZE && !s->rx_fifo_count &&
            !(s->op_mode & OP_RSF) && (s->op_mode & OP_SR) &&
            s->rx_state != RS_STOPPED && !s->rx_fatal) {
            /* Cut-through: a frame longer than the FIFO streams out. */
            if (emac_rx_dma_frame(s, f, data_len, status, rdes4)) {
                return;
            }
        }
        emac_rx_overflow(s);
        return;
    }
    memcpy(s->rx_fifo + s->rx_fifo_used, f, data_len);
    s->rx_fifo_used += data_len;
    s->rx_fifo_len[s->rx_fifo_count] = data_len;
    s->rx_fifo_rdes0[s->rx_fifo_count] = status;
    s->rx_fifo_rdes4[s->rx_fifo_count] = rdes4;
    s->rx_fifo_count++;
    emac_rx_dma(s);
}

/* The network backend's frames, arriving on the medium */
static ssize_t emac_receive(NetClientState *nc, const uint8_t *buf,
                            size_t size)
{
    Esp32EmacState *s = qemu_get_nic_opaque(nc);
    g_autofree uint8_t *wire = NULL;
    size_t len = MAX(size, ETH_MIN_LEN);

    if (!s->phy || !ip101_phy_link_up(s->phy) || (s->config & CFG_LM) ||
        !emac_rx_clk(s)) {
        return size;
    }
    if (len + ETH_FCS_LEN > ESP32_EMAC_MAX_FRAME) {
        return size;
    }
    wire = g_malloc0(len + ETH_FCS_LEN);
    memcpy(wire, buf, size);
    stl_le_p(wire + len, emac_fcs(wire, len));
    emac_mac_rx(s, wire, len + ETH_FCS_LEN);
    return size;
}

static void emac_link_status_changed(NetClientState *nc)
{
    Esp32EmacState *s = qemu_get_nic_opaque(nc);

    if (s->phy) {
        ip101_phy_set_medium(s->phy, nc->peer && !nc->link_down);
    }
}

/* MDIO */

/*
 * [spec:nuos:req:emu.esp32.emac]
 * A management frame takes 64 MDC cycles (32 preamble bits, 32 frame
 * bits); MDC is APB_CLK divided as EMACGMIIADDR.MIICSRCLK selects. A
 * read of an address no PHY answers returns the pulled-up 0xffff.
 */
static void emac_mdio_start(Esp32EmacState *s)
{
    static const uint8_t div[16] = {
        42, 62, 16, 26, 102, 124, 0, 0, 4, 6, 8, 10, 12, 14, 16, 18
    };
    unsigned cr = (s->mii_addr >> MII_CR_SHIFT) & 0xf;
    uint64_t hz = clock_get_hz(s->apb_clk);
    uint32_t d = div[cr];

    if (!d) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32.emac: reserved MIICSRCLK value %u\n", cr);
        d = 42;
    }
    if (!hz) {
        hz = 80000000;
    }
    timer_mod(&s->mdio_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              muldiv64(64 * d, NANOSECONDS_PER_SECOND, hz));
}

static void emac_mdio_cb(void *opaque)
{
    Esp32EmacState *s = opaque;
    unsigned pa = (s->mii_addr >> MII_PA_SHIFT) & 0x1f;
    unsigned gr = (s->mii_addr >> MII_GR_SHIFT) & 0x1f;
    bool present = s->phy && s->phy->addr == pa;

    if (s->mii_addr & MII_GW) {
        if (present) {
            ip101_phy_mdio_write(s->phy, gr, s->mii_data & 0xffff);
        }
    } else {
        s->mii_data = present ? ip101_phy_mdio_read(s->phy, gr) : 0xffff;
    }
    s->mii_addr &= ~MII_GB;
}

/* Registers */

static void emac_regs_reset(Esp32EmacState *s)
{
    s->bus_mode = BUS_MODE_RESET;
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

    s->config = 0;
    s->frame_filter = 0;
    s->mii_addr = 0;
    s->mii_data = 0;
    s->flow_ctrl = 0;
    memset(s->rwuffr, 0, sizeof(s->rwuffr));
    s->rwuffr_ptr = 0;
    s->pmt_csr = 0;
    s->lpi_csr = 0;
    s->lpi_timers = LPI_TIMERS_RESET;
    s->int_mask = 0;
    s->addr_high[0] = ADDR_HIGH_AE | 0xffff;
    s->addr_low[0] = 0xffffffff;
    for (unsigned i = 1; i < ESP32_EMAC_ADDR_COUNT; i++) {
        s->addr_high[i] = 0xffff;
        s->addr_low[i] = 0xffffffff;
    }
    s->wdog_to = 0;

    timer_del(&s->mdio_timer);
    timer_del(&s->tx_timer);
    timer_del(&s->pause_timer);
    timer_del(&s->rx_wdt_timer);
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
    s->pause_pending = false;
    s->pause_on_wire = false;
    s->paused = false;
    s->rx_fifo_used = 0;
    s->rx_fifo_count = 0;
    s->sw_reset_pending = false;
}

/* A software reset finishes once every clock domain has a clock. */
static void emac_sw_reset_check(Esp32EmacState *s)
{
    if (s->sw_reset_pending && emac_tx_clk(s) && emac_rx_clk(s)) {
        s->sw_reset_pending = false;
        s->bus_mode &= ~BUS_MODE_SWR;
    }
}

static uint32_t emac_debug(Esp32EmacState *s)
{
    uint32_t v = 0;

    if (s->rx_fifo_used == ESP32_EMAC_RX_FIFO_SIZE) {
        v |= 3u << 8;
    } else if (s->rx_fifo_used >= ESP32_EMAC_RX_FIFO_SIZE * 3 / 4) {
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

static uint64_t emac_read(void *opaque, hwaddr addr, unsigned size)
{
    Esp32EmacState *s = opaque;
    bool live = emac_live(s);
    uint32_t v;

    switch (addr) {
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
        return s->status | ((uint32_t)s->rx_state << ST_RS_SHIFT) |
               ((uint32_t)s->tx_state << ST_TS_SHIFT) |
               (emac_pmt_ints(s) ? ST_PMT : 0);
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
    case A_TX_CUR_DESC:
        return s->tx_desc;
    case A_RX_CUR_DESC:
        return s->rx_desc;
    case A_TX_CUR_BUF:
        return s->tx_buf;
    case A_RX_CUR_BUF:
        return s->rx_buf;

    case A_EX_CLKOUT_CONF:
        return s->ex_clkout_conf;
    case A_EX_OSCCLK_CONF:
        return s->ex_oscclk_conf;
    case A_EX_CLK_CTRL:
        return s->ex_clk_ctrl;
    case A_EX_PHYINF_CONF:
        return s->ex_phyinf_conf;
    case A_EX_PD_SEL:
        return s->ex_pd_sel;
    case A_EX_DATE:
        return s->ex_date;

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
    case A_DEBUG:
        return emac_debug(s);
    case A_RWUFFR:
        if (!live) {
            return 0;
        }
        v = s->rwuffr[s->rwuffr_ptr];
        s->rwuffr_ptr = (s->rwuffr_ptr + 1) % ESP32_EMAC_RWUFFR_COUNT;
        return v;
    case A_PMT_CSR:
        v = s->pmt_csr | (s->rwuffr_ptr << PMT_RWKPTR_SHIFT);
        if (!live) {
            return 0;
        }
        s->pmt_csr &= ~(PMT_MGKPRCVD | PMT_RWKPRCVD);
        emac_update_irq(s);
        return v;
    case A_LPI_CSR:
        return s->lpi_csr;
    case A_LPI_TIMERS:
        return s->lpi_timers;
    case A_INTS:
        return emac_pmt_ints(s);
    case A_INT_MASK:
        return s->int_mask;
    case A_ADDR0_HIGH ... A_ADDR7_LOW:
        if (addr & 4) {
            return s->addr_low[(addr - A_ADDR0_HIGH) / 8];
        }
        return s->addr_high[(addr - A_ADDR0_HIGH) / 8];
    case A_CSTATUS:
        return 0;
    case A_WDOG_TO:
        return s->wdog_to;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32.emac: read of reserved offset 0x%04" HWADDR_PRIx
                      "\n", addr);
        return 0;
    }
}

static void emac_write_op_mode(Esp32EmacState *s, uint32_t v)
{
    uint32_t old = s->op_mode;

    s->op_mode = v & OP_RW & ~OP_FTF;
    if (v & OP_FTF) {
        emac_tx_flush(s);
    }

    if ((v & OP_ST) && !(old & OP_ST) && !s->tx_fatal) {
        s->tx_state = TS_FETCHING;
        emac_tx_dma(s);
    } else if (!(v & OP_ST) && (old & OP_ST)) {
        if (!s->tx_on_wire && !s->tx_ready) {
            emac_tx_maybe_stop(s);
        }
    }

    if ((v & OP_SR) && !(old & OP_SR) && !s->rx_fatal) {
        s->rx_state = RS_WAITING;
        emac_rx_dma(s);
    } else if (!(v & OP_SR) && (old & OP_SR)) {
        if (s->rx_state != RS_STOPPED) {
            s->rx_state = RS_STOPPED;
            emac_raise(s, ST_RPS);
        }
    }
}

static void emac_write(void *opaque, hwaddr addr, uint64_t value,
                       unsigned size)
{
    Esp32EmacState *s = opaque;
    uint32_t v = value;
    uint32_t old;

    switch (addr) {
    case A_BUS_MODE:
        if (v & BUS_MODE_SWR) {
            /* The wrapper sits outside the GMAC and keeps its state. */
            emac_regs_reset(s);
            s->bus_mode |= BUS_MODE_SWR;
            s->sw_reset_pending = true;
            emac_sw_reset_check(s);
            emac_update_irq(s);
            return;
        }
        if (s->sw_reset_pending) {
            return;
        }
        s->bus_mode = v & BUS_MODE_RW;
        return;
    case A_TX_POLL:
        if (s->tx_state == TS_SUSPENDED) {
            s->tx_state = TS_FETCHING;
        }
        emac_tx_dma(s);
        return;
    case A_RX_POLL:
        if (s->rx_state == RS_SUSPENDED) {
            s->rx_state = RS_WAITING;
        }
        emac_rx_dma(s);
        return;
    case A_RX_BASE:
        s->rx_base = v & ~3u;
        if (s->rx_state == RS_STOPPED) {
            s->rx_desc = s->rx_base;
        } else {
            qemu_log_mask(LOG_GUEST_ERROR, "esp32.emac: DMARXBASEADDR "
                          "written while the receive process runs\n");
        }
        return;
    case A_TX_BASE:
        s->tx_base = v & ~3u;
        if (s->tx_state == TS_STOPPED) {
            s->tx_desc = s->tx_base;
        } else {
            qemu_log_mask(LOG_GUEST_ERROR, "esp32.emac: DMATXBASEADDR "
                          "written while the transmit process runs\n");
        }
        return;
    case A_STATUS:
        s->status &= ~(v & ST_W1C);
        emac_update_irq(s);
        return;
    case A_OP_MODE:
        emac_write_op_mode(s, v);
        return;
    case A_INT_EN:
        s->int_en = v & IE_RW;
        emac_update_irq(s);
        return;
    case A_RX_WDT:
        s->rx_wdt = v & 0xff;
        return;

    case A_EX_CLKOUT_CONF:
        s->ex_clkout_conf = v & EX_CLKOUT_RW;
        return;
    case A_EX_OSCCLK_CONF:
        s->ex_oscclk_conf = v & EX_OSCCLK_RW;
        return;
    case A_EX_CLK_CTRL:
        s->ex_clk_ctrl = v & EX_CLK_CTRL_RW;
        emac_sw_reset_check(s);
        emac_tx_kick(s);
        return;
    case A_EX_PHYINF_CONF:
        old = (v >> EX_PHY_INTF_SHIFT) & 7;
        if (old != EX_PHY_INTF_MII && old != EX_PHY_INTF_RMII) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "esp32.emac: reserved PHY interface %u\n", old);
        }
        s->ex_phyinf_conf = v & EX_PHYINF_RW;
        emac_sw_reset_check(s);
        emac_tx_kick(s);
        return;
    case A_EX_PD_SEL:
        s->ex_pd_sel = v & EX_PD_SEL_RW;
        if (s->ex_pd_sel) {
            qemu_log_mask(LOG_UNIMP,
                          "esp32.emac: FIFO RAM power-down\n");
        }
        return;
    case A_EX_DATE:
        s->ex_date = v;
        return;

    case A_CONFIG:
        old = s->config;
        s->config = v & CFG_RW;
        if ((s->config & CFG_TE) && !(old & CFG_TE)) {
            emac_tx_kick(s);
        }
        return;
    case A_FRAME_FILTER:
        s->frame_filter = v & FF_RW;
        return;
    case A_MII_ADDR:
        if (s->mii_addr & MII_GB) {
            qemu_log_mask(LOG_GUEST_ERROR, "esp32.emac: EMACGMIIADDR "
                          "written while a management frame is busy\n");
            return;
        }
        s->mii_addr = v & 0xffff;
        if (v & MII_GB) {
            emac_mdio_start(s);
        }
        return;
    case A_MII_DATA:
        if (s->mii_addr & MII_GB) {
            qemu_log_mask(LOG_GUEST_ERROR, "esp32.emac: EMACMIIDATA "
                          "written while a management frame is busy\n");
        }
        s->mii_data = v & 0xffff;
        return;
    case A_FLOW_CTRL:
        if (s->flow_ctrl & FC_FCB) {
            qemu_log_mask(LOG_GUEST_ERROR, "esp32.emac: EMACFC written "
                          "while a pause frame is pending\n");
            return;
        }
        s->flow_ctrl = v & FC_RW;
        if ((v & FC_FCB) && (s->config & CFG_DM)) {
            if (s->flow_ctrl & FC_TFE) {
                s->pause_pending = true;
                emac_tx_kick(s);
            } else {
                s->flow_ctrl &= ~FC_FCB;
            }
        }
        return;
    case A_RWUFFR:
        s->rwuffr[s->rwuffr_ptr] = v;
        s->rwuffr_ptr = (s->rwuffr_ptr + 1) % ESP32_EMAC_RWUFFR_COUNT;
        return;
    case A_PMT_CSR:
        if (v & PMT_RWKFILTRST) {
            s->rwuffr_ptr = 0;
        }
        s->pmt_csr = (s->pmt_csr & ~PMT_RW) | (v & PMT_RW);
        return;
    case A_LPI_CSR:
        s->lpi_csr = v & LPI_RW;
        if (v & LPI_LPIEN) {
            qemu_log_mask(LOG_UNIMP, "esp32.emac: low-power idle\n");
        }
        return;
    case A_LPI_TIMERS:
        s->lpi_timers = v & LPI_TIMERS_RW;
        return;
    case A_INT_MASK:
        s->int_mask = v & INT_MASK_RW;
        emac_update_irq(s);
        return;
    case A_ADDR0_HIGH ... A_ADDR7_LOW: {
        unsigned n = (addr - A_ADDR0_HIGH) / 8;

        if (addr & 4) {
            s->addr_low[n] = v;
        } else if (n == 0) {
            s->addr_high[0] = ADDR_HIGH_AE | (v & ADDR0_HIGH_RW);
        } else {
            s->addr_high[n] = v & ADDRN_HIGH_RW;
        }
        return;
    }
    case A_WDOG_TO:
        s->wdog_to = v & WDOG_TO_RW;
        return;
    case A_MISSED:
    case A_TX_CUR_DESC:
    case A_RX_CUR_DESC:
    case A_TX_CUR_BUF:
    case A_RX_CUR_BUF:
    case A_DEBUG:
    case A_INTS:
    case A_CSTATUS:
        qemu_log_mask(LOG_GUEST_ERROR, "esp32.emac: write to read-only "
                      "register at 0x%04" HWADDR_PRIx "\n", addr);
        return;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32.emac: write to reserved offset 0x%04"
                      HWADDR_PRIx "\n", addr);
        return;
    }
}

static const MemoryRegionOps emac_ops = {
    .read = emac_read,
    .write = emac_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void emac_clk_update(void *opaque, ClockEvent event)
{
    Esp32EmacState *s = opaque;

    if (emac_live(s)) {
        emac_tx_kick(s);
        emac_tx_dma(s);
        emac_rx_dma(s);
    }
}

static NetClientInfo net_esp32_emac_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .receive = emac_receive,
    .link_status_changed = emac_link_status_changed,
};

static void esp32_emac_reset_hold(Object *obj, ResetType type)
{
    Esp32EmacState *s = ESP32_EMAC(obj);

    emac_regs_reset(s);
    s->ex_clkout_conf = EX_CLKOUT_RESET;
    s->ex_oscclk_conf = EX_OSCCLK_RESET;
    s->ex_clk_ctrl = 0;
    s->ex_phyinf_conf = 0;
    s->ex_pd_sel = 0;
    s->ex_date = EX_DATE_RESET;
    qemu_set_irq(s->irq, 0);
}

static void esp32_emac_init(Object *obj)
{
    Esp32EmacState *s = ESP32_EMAC(obj);

    memory_region_init_io(&s->iomem, obj, &emac_ops, s, TYPE_ESP32_EMAC,
                          ESP32_EMAC_REG_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->apb_clk = qdev_init_clock_in(DEVICE(obj), "apb", emac_clk_update, s,
                                    ClockUpdate);
    timer_init_ns(&s->mdio_timer, QEMU_CLOCK_VIRTUAL, emac_mdio_cb, s);
    timer_init_ns(&s->tx_timer, QEMU_CLOCK_VIRTUAL, emac_tx_timer_cb, s);
    timer_init_ns(&s->pause_timer, QEMU_CLOCK_VIRTUAL, emac_pause_timer_cb,
                  s);
    timer_init_ns(&s->rx_wdt_timer, QEMU_CLOCK_VIRTUAL, emac_rx_wdt_cb, s);
}

static void esp32_emac_realize(DeviceState *dev, Error **errp)
{
    Esp32EmacState *s = ESP32_EMAC(dev);
    NetClientState *nc;

    if (!s->dma_mr) {
        error_setg(errp, "esp32.emac: 'dma-mr' link not set");
        return;
    }
    address_space_init(&s->dma_as, s->dma_mr, "esp32-emac-dma");
    qemu_macaddr_default_if_unset(&s->conf.macaddr);
    s->nic = qemu_new_nic(&net_esp32_emac_info, &s->conf,
                          object_get_typename(OBJECT(dev)), dev->id,
                          &dev->mem_reentrancy_guard, s);
    nc = qemu_get_queue(s->nic);
    qemu_format_nic_info_str(nc, s->conf.macaddr.a);
    emac_link_status_changed(nc);
}

static void esp32_emac_unrealize(DeviceState *dev)
{
    Esp32EmacState *s = ESP32_EMAC(dev);

    qemu_del_nic(s->nic);
    address_space_destroy(&s->dma_as);
}

static const Property esp32_emac_properties[] = {
    DEFINE_NIC_PROPERTIES(Esp32EmacState, conf),
    DEFINE_PROP_LINK("dma-mr", Esp32EmacState, dma_mr, TYPE_MEMORY_REGION,
                     MemoryRegion *),
    DEFINE_PROP_LINK("phy", Esp32EmacState, phy, TYPE_IP101_PHY,
                     IP101PhyState *),
};

static const VMStateDescription vmstate_esp32_emac = {
    .name = TYPE_ESP32_EMAC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(bus_mode, Esp32EmacState),
        VMSTATE_UINT32(rx_base, Esp32EmacState),
        VMSTATE_UINT32(tx_base, Esp32EmacState),
        VMSTATE_UINT32(status, Esp32EmacState),
        VMSTATE_UINT32(op_mode, Esp32EmacState),
        VMSTATE_UINT32(int_en, Esp32EmacState),
        VMSTATE_UINT32(missed_frames, Esp32EmacState),
        VMSTATE_UINT32(overflow_frames, Esp32EmacState),
        VMSTATE_BOOL(missed_overflow, Esp32EmacState),
        VMSTATE_BOOL(fifo_overflow, Esp32EmacState),
        VMSTATE_UINT32(rx_wdt, Esp32EmacState),
        VMSTATE_UINT32(tx_desc, Esp32EmacState),
        VMSTATE_UINT32(rx_desc, Esp32EmacState),
        VMSTATE_UINT32(tx_buf, Esp32EmacState),
        VMSTATE_UINT32(rx_buf, Esp32EmacState),
        VMSTATE_UINT8(tx_state, Esp32EmacState),
        VMSTATE_UINT8(rx_state, Esp32EmacState),
        VMSTATE_BOOL(tx_fatal, Esp32EmacState),
        VMSTATE_BOOL(rx_fatal, Esp32EmacState),
        VMSTATE_UINT32(config, Esp32EmacState),
        VMSTATE_UINT32(frame_filter, Esp32EmacState),
        VMSTATE_UINT32(mii_addr, Esp32EmacState),
        VMSTATE_UINT32(mii_data, Esp32EmacState),
        VMSTATE_UINT32(flow_ctrl, Esp32EmacState),
        VMSTATE_UINT32_ARRAY(rwuffr, Esp32EmacState,
                             ESP32_EMAC_RWUFFR_COUNT),
        VMSTATE_UINT32(rwuffr_ptr, Esp32EmacState),
        VMSTATE_UINT32(pmt_csr, Esp32EmacState),
        VMSTATE_UINT32(lpi_csr, Esp32EmacState),
        VMSTATE_UINT32(lpi_timers, Esp32EmacState),
        VMSTATE_UINT32(int_mask, Esp32EmacState),
        VMSTATE_UINT32_ARRAY(addr_high, Esp32EmacState,
                             ESP32_EMAC_ADDR_COUNT),
        VMSTATE_UINT32_ARRAY(addr_low, Esp32EmacState,
                             ESP32_EMAC_ADDR_COUNT),
        VMSTATE_UINT32(wdog_to, Esp32EmacState),
        VMSTATE_UINT32(ex_clkout_conf, Esp32EmacState),
        VMSTATE_UINT32(ex_oscclk_conf, Esp32EmacState),
        VMSTATE_UINT32(ex_clk_ctrl, Esp32EmacState),
        VMSTATE_UINT32(ex_phyinf_conf, Esp32EmacState),
        VMSTATE_UINT32(ex_pd_sel, Esp32EmacState),
        VMSTATE_UINT32(ex_date, Esp32EmacState),
        VMSTATE_BOOL(sw_reset_pending, Esp32EmacState),
        VMSTATE_TIMER(mdio_timer, Esp32EmacState),
        VMSTATE_UINT8_ARRAY(tx_frame, Esp32EmacState,
                            ESP32_EMAC_TX_BUF_SIZE),
        VMSTATE_UINT32(tx_wire_len, Esp32EmacState),
        VMSTATE_UINT32(tx_len, Esp32EmacState),
        VMSTATE_BOOL(tx_gathering, Esp32EmacState),
        VMSTATE_BOOL(tx_ready, Esp32EmacState),
        VMSTATE_BOOL(tx_on_wire, Esp32EmacState),
        VMSTATE_BOOL(tx_jabber, Esp32EmacState),
        VMSTATE_UINT32(tx_tdes0, Esp32EmacState),
        VMSTATE_UINT32(tx_tdes1, Esp32EmacState),
        VMSTATE_UINT32(tx_last_desc, Esp32EmacState),
        VMSTATE_UINT32(tx_status, Esp32EmacState),
        VMSTATE_TIMER(tx_timer, Esp32EmacState),
        VMSTATE_BOOL(pause_pending, Esp32EmacState),
        VMSTATE_BOOL(pause_on_wire, Esp32EmacState),
        VMSTATE_TIMER(pause_timer, Esp32EmacState),
        VMSTATE_BOOL(paused, Esp32EmacState),
        VMSTATE_UINT8_ARRAY(rx_fifo, Esp32EmacState,
                            ESP32_EMAC_RX_FIFO_SIZE),
        VMSTATE_UINT32(rx_fifo_used, Esp32EmacState),
        VMSTATE_UINT32(rx_fifo_count, Esp32EmacState),
        VMSTATE_UINT16_ARRAY(rx_fifo_len, Esp32EmacState,
                             ESP32_EMAC_RX_FIFO_MAX_FRAMES),
        VMSTATE_UINT32_ARRAY(rx_fifo_rdes0, Esp32EmacState,
                             ESP32_EMAC_RX_FIFO_MAX_FRAMES),
        VMSTATE_UINT32_ARRAY(rx_fifo_rdes4, Esp32EmacState,
                             ESP32_EMAC_RX_FIFO_MAX_FRAMES),
        VMSTATE_TIMER(rx_wdt_timer, Esp32EmacState),
        VMSTATE_CLOCK(apb_clk, Esp32EmacState),
        VMSTATE_END_OF_LIST()
    }
};

static void esp32_emac_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32_emac_reset_hold;
    dc->realize = esp32_emac_realize;
    dc->unrealize = esp32_emac_unrealize;
    dc->vmsd = &vmstate_esp32_emac;
    device_class_set_props(dc, esp32_emac_properties);
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}

/* [spec:nuos:req:emu.esp32.emac] */
static const TypeInfo esp32_emac_info = {
    .name = TYPE_ESP32_EMAC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32EmacState),
    .instance_init = esp32_emac_init,
    .class_init = esp32_emac_class_init,
};

static void esp32_emac_register_types(void)
{
    type_register_static(&esp32_emac_info);
}

type_init(esp32_emac_register_types)
