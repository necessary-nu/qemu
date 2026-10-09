/*
 * ESP32 SDIO slave: HINF, SLC and SLCHOST, and the SDIO card they present
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32 TRM, "SDIO Slave Controller", with the register layout
 * of ESP-IDF's soc/esp32 sdio_slc_reg.h, sdio_slc_host_reg.h and
 * sdio_hinf_reg.h, and the protocol ESP-IDF's sdio_slave driver and the
 * esp_serial_slave_link host driver follow.
 *
 * The slave is one SDIO card with I/O functions 1 and 2. The host reaches
 * the card's function 0 (CCCR, FBRs and CIS) and function 1 through
 * CMD52 and CMD53:
 *
 *  - function 1 addresses 0x00000..0x003ff are the SLCHOST registers
 *    (interrupt status and enables, the shared CONF_W registers, the
 *    packet length and the receive buffer count);
 *  - addresses 0x00400..0x1f7ff are SLC0's data: reads take the packet
 *    the slave sends, writes fill the slave's receive buffers. A transfer
 *    is addressed so that it ends at 0x1f800; the slave pads reads and
 *    drops writes at 0x1f800 and above, and a write that reaches 0x1f800
 *    ends the packet.
 *
 * SLC0's two DMA links use the ESP32 DMA descriptor: DW0 holds owner
 * (31), eof (30), length (23:12) and size (11:0); DW1 the buffer address;
 * DW2 the next descriptor's address, 0 at the end of the list. The DMA
 * reaches internal SRAM 0x3ffae000..0x3fffffff; the link registers hold
 * the low 20 bits of a descriptor address in 0x3ffxxxxx.
 *
 * Sending (the SLC's "RX" link): the DMA moves the packet's buffers into
 * its FIFO ahead of the host (RX_DONE as each buffer is taken), stops at
 * the end of the packet, raises the host's RX_NEW_PACKET when it starts a
 * packet and RX_EOF when the host has read the packet's last byte, then
 * goes on to the next packet. A host read with the FIFO empty underflows
 * (RX_UDF) and reads 0.
 *
 * Receiving (the SLC's "TX" link): host data fills the descriptors in
 * turn; each is written back with its length and, at the end of the
 * packet, EOF (TX_DONE, TX_SUC_EOF). Data with no descriptor to take it
 * overflows (TX_OVF) and is lost. The receive buffer count (TOKEN1) and
 * the packet length (SLC0_LEN) are the counters software keeps for the
 * host to read.
 *
 * The TRM documents only part of the register file. Where it is silent,
 * the model takes what ESP-IDF relies on and states its inference beside
 * the code: function 2 exists (HINF has its IDs and enables) but its data
 * path and SLC1's DMA are undocumented and logged as unimplemented; the
 * CIS is the model's, built from HINF's IDs and its 32 configurable bytes.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/sd/esp32_sdio_slave.h"
#include "migration/vmstate.h"
#include "sdmmc-internal.h"

/* ---- SLC registers ---- */
#define A_SLC_CONF0             0x000
#define A_SLC_0INT_RAW          0x004
#define A_SLC_0INT_ST           0x008
#define A_SLC_0INT_ENA          0x00c
#define A_SLC_0INT_CLR          0x010
#define A_SLC_1INT_RAW          0x014
#define A_SLC_1INT_ST           0x018
#define A_SLC_1INT_ENA          0x01c
#define A_SLC_1INT_CLR          0x020
#define A_SLC_RX_STATUS         0x024
#define A_SLC_0RXFIFO_PUSH      0x028
#define A_SLC_1RXFIFO_PUSH      0x02c
#define A_SLC_TX_STATUS         0x030
#define A_SLC_0TXFIFO_POP       0x034
#define A_SLC_1TXFIFO_POP       0x038
#define A_SLC_0RX_LINK          0x03c
#define A_SLC_0TX_LINK          0x040
#define A_SLC_1RX_LINK          0x044
#define A_SLC_1TX_LINK          0x048
#define A_SLC_INTVEC_TOHOST     0x04c
#define A_SLC_0TOKEN0           0x050
#define A_SLC_0TOKEN1           0x054
#define A_SLC_1TOKEN0           0x058
#define A_SLC_1TOKEN1           0x05c
#define A_SLC_CONF1             0x060
#define A_SLC_BRIDGE_CONF       0x074
#define A_SLC_0TO_EOF_DES_ADDR  0x078
#define A_SLC_0TX_EOF_DES_ADDR  0x07c
#define A_SLC_0TO_EOF_BFR_DES_ADDR 0x080
#define A_SLC_AHB_TEST          0x090
#define A_SLC_RX_DSCR_CONF      0x098
#define A_SLC_0TXLINK_DSCR      0x09c
#define A_SLC_0RXLINK_DSCR      0x0a8
#define A_SLC_TOKEN_LAT         0x0d4
#define A_SLC_TX_DSCR_CONF      0x0d8
#define A_SLC_0_LEN_CONF        0x0e4
#define A_SLC_0_LENGTH          0x0e8
#define A_SLC_SEQ_POSITION      0x114
#define A_SLC_0_DSCR_REC_CONF   0x118
#define A_SLC_SDIO_CRC_ST1      0x120
#define A_SLC_0_LEN_LIM_CONF    0x138
#define A_SLC_0INT_ST1          0x13c
#define A_SLC_0INT_ENA1         0x140
#define A_SLC_1INT_ST1          0x144
#define A_SLC_1INT_ENA1         0x148

/* SLCCONF0 */
#define CONF0_SLC0_TX_RST       BIT(0)
#define CONF0_SLC0_RX_RST       BIT(1)
#define CONF0_AHBM_FIFO_RST     BIT(2)
#define CONF0_AHBM_RST          BIT(3)
#define CONF0_SLC0_TX_LOOP_TEST BIT(4)
#define CONF0_SLC0_RX_LOOP_TEST BIT(5)
#define CONF0_SLC0_RX_AUTO_WRBACK BIT(6)
#define CONF0_SLC0_TOKEN_AUTO_CLR BIT(14)
#define CONF0_SLC0_TOKEN_SEL    BIT(15)
#define CONF0_SLC0_WR_RETRY_MASK_EN BIT(18)
/* TRM register 26.1: TOKEN_AUTO_CLR, RX_LOOP_TEST and TX_LOOP_TEST */
#define CONF0_RESET             0x00004030u

/* SLC0INT_* (SLC1INT_* has the same layout in its low 25 bits) */
#define SLCINT_FRHOST_MASK      0xffu
#define SLCINT_RX_START         BIT(8)
#define SLCINT_TX_START         BIT(9)
#define SLCINT_RX_UDF           BIT(10)
#define SLCINT_TX_OVF           BIT(11)
#define SLCINT_TX_DONE          BIT(14)
#define SLCINT_TX_SUC_EOF       BIT(15)
#define SLCINT_RX_DONE          BIT(16)
#define SLCINT_RX_EOF           BIT(17)
#define SLCINT_TX_DSCR_ERR      BIT(19)
#define SLCINT_RX_DSCR_ERR      BIT(20)
#define SLC0INT_MASK            0x07ffffffu
#define SLC1INT_MASK            0x01ffffffu

/* SLC0RX_LINK, SLC0TX_LINK and the SLC1 links */
#define LINK_ADDR_MASK          0xfffffu
#define LINK_SLC1_BT_PACKET     BIT(20)
#define LINK_STOP               BIT(28)
#define LINK_START              BIT(29)
#define LINK_RESTART            BIT(30)
#define LINK_PARK               BIT(31)
#define LINK_DESC_BASE          0x3ff00000u

/* SLC0TOKEN0/1, SLC1TOKEN0/1 */
#define TOKEN_WDATA_MASK        0xfffu
#define TOKEN_WR                BIT(12)
#define TOKEN_INC               BIT(13)
#define TOKEN_INC_MORE          BIT(14)
#define TOKEN_SHIFT             16

/* SLCCONF1 */
#define CONF1_SLC0_CHECK_OWNER  BIT(0)
#define CONF1_SLC0_CHECK_SUM    (BIT(1) | BIT(2))
#define CONF1_CMD_HOLD_EN       BIT(3)
#define CONF1_SLC0_LEN_AUTO_CLR BIT(4)
#define CONF1_SLC0_STITCH       (BIT(5) | BIT(6))
#define CONF1_MASK              0x007f007fu
/* TRM register 26.10: CMD_HOLD_EN, LEN_AUTO_CLR, TX_ and RX_STITCH_EN */
#define CONF1_RESET             0x00000078u

/* SLC_RX_DSCR_CONF: fields other than TOKEN_NO_REPLACE change the DMA */
#define RX_DSCR_CONF_SLC0_UNMODELLED 0x0000fffcu

/* SLC0_LEN_CONF */
#define LEN_WDATA_MASK          0xfffffu
#define LEN_WR                  BIT(20)
#define LEN_INC                 BIT(21)
#define LEN_INC_MORE            BIT(22)
#define LEN_CONF_UNMODELLED     0x1f800000u

/* RX_STATUS and TX_STATUS */
#define FIFO_FULL               BIT(0)
#define FIFO_EMPTY              BIT(1)
#define SLC1_FIFO_EMPTY         BIT(17)

#define RXFIFO_PUSH             BIT(16)
#define TXFIFO_POP              BIT(16)

/* ---- SLCHOST registers ---- */
#define A_HOST_FUNC2_0          0x010
#define A_HOST_FUNC2_1          0x014
#define A_HOST_FUNC2_2          0x020
#define A_HOST_SLC0_TOKEN_RDATA 0x044
#define A_HOST_SLC0_INT_RAW     0x050
#define A_HOST_SLC1_INT_RAW     0x054
#define A_HOST_SLC0_INT_ST      0x058
#define A_HOST_SLC1_INT_ST      0x05c
#define A_HOST_PKT_LEN          0x060
#define A_HOST_STATE_W0         0x064
#define A_HOST_STATE_W1         0x068
#define A_HOST_CONF_W0          0x06c
#define A_HOST_CONF_W5          0x080
#define A_HOST_WIN_CMD          0x084
#define A_HOST_CONF_W6          0x088
#define A_HOST_CONF_W7          0x08c
#define A_HOST_CONF_W8          0x09c
#define A_HOST_CONF_W15         0x0b8
#define A_HOST_SLC1_TOKEN_RDATA 0x0c4
#define A_HOST_SLC0_TOKEN_WDATA 0x0c8
#define A_HOST_SLC1_TOKEN_WDATA 0x0cc
#define A_HOST_TOKEN_CON        0x0d0
#define A_HOST_SLC0_INT_CLR     0x0d4
#define A_HOST_SLC1_INT_CLR     0x0d8
#define A_HOST_SLC0_FUNC1_INT_ENA 0x0dc
#define A_HOST_SLC1_FUNC1_INT_ENA 0x0e0
#define A_HOST_SLC0_FUNC2_INT_ENA 0x0e4
#define A_HOST_SLC1_FUNC2_INT_ENA 0x0e8
#define A_HOST_SLC1_INT_ENA     0x0f0
#define A_HOST_SLC0_RX_INFOR    0x0f4
#define A_HOST_SLC1_RX_INFOR    0x0f8
#define A_HOST_SLC0_LEN_WD      0x0fc
#define A_HOST_APBWIN_WDATA     0x100
#define A_HOST_APBWIN_CONF      0x104
#define A_HOST_SLC0_RDCLR       0x10c
#define A_HOST_SLC1_RDCLR       0x110
#define A_HOST_SLC0_INT_ENA1    0x114
#define A_HOST_SLC1_INT_ENA1    0x118
#define A_HOST_CONF             0x1f0

/* SLC0HOST_INT_* (SLC1HOST_INT_* matches in the bits modelled) */
#define HOSTINT_TOHOST_MASK     0xffu
#define HOSTINT_RX_UDF          BIT(16)
#define HOSTINT_TX_OVF          BIT(17)
#define HOSTINT_RX_NEW_PACKET   BIT(23)
#define HOSTINT_MASK            0x03ffffffu

/* SLCHOST_TOKEN_CON */
#define TOKEN_CON_SLC0_TOKEN0_DEC BIT(0)
#define TOKEN_CON_SLC0_TOKEN0_WR BIT(2)
#define TOKEN_CON_SLC1_SHIFT    4
#define TOKEN_CON_SLC0_LEN_WR   BIT(8)

#define APBWIN_START            BIT(29)
#define PKT_LEN_CHECK_SHIFT     20

/* ---- HINF registers ---- */
#define A_HINF_CFG_DATA0        0x000
#define A_HINF_CFG_DATA1        0x004
#define A_HINF_CFG_DATA7        0x01c
#define A_HINF_CIS_CONF0        0x020
#define A_HINF_CIS_CONF7        0x03c
#define A_HINF_CFG_DATA16       0x040

/* HINF_CFG_DATA1 */
#define CFG1_SDIO_IOREADY1      BIT(1)
#define CFG1_HIGHSPEED_ENABLE   BIT(2)
#define CFG1_HIGHSPEED_MODE     BIT(3)
#define CFG1_SDIO_IOREADY2      BIT(5)
#define CFG1_IOENABLE2          BIT(7)
#define CFG1_IOENABLE1          BIT(11)
#define CFG1_SDIO_VER_SHIFT     16
/* Bits that show the host's settings rather than store the CPU's */
#define CFG1_HOST_STATUS        (CFG1_HIGHSPEED_MODE | CFG1_IOENABLE2 | \
                                 CFG1_IOENABLE1)

#define CFG7_MASK               0x0003ffffu
#define CFG7_SDIO_RST           BIT(16)

/*
 * The function IDs the CIS presents until software changes them. ESP-IDF
 * never writes them; hosts know the ESP32 by vendor 0x6666, device 0x2222
 * (function 1) and 0x3333 (function 2).
 */
#define CFG_DATA0_RESET         0x22226666u
#define CFG_DATA16_RESET        0x33336666u

/* ---- Descriptors and DMA ---- */
#define DESC_SIZE(dw0)          ((dw0) & 0xfffu)
#define DESC_LENGTH(dw0)        (((dw0) >> 12) & 0xfffu)
#define DESC_LENGTH_SHIFT       12
#define DESC_EOF                BIT(30)
#define DESC_OWNER              BIT(31)

#define DMA_RAM_START           0x3ffae000u
#define DMA_RAM_END             0x40000000u

/* ---- The SDIO card ---- */
#define SDIO_FUNCTIONS          2
/* 2.7..3.6 V */
#define SDIO_OCR                0x00ff8000u
#define R4_READY                BIT(31)
#define R4_NF_SHIFT             28

/* R5 response flags, bits 15:8 of the response */
#define R5_FUNCTION_NUMBER      BIT(9)
#define R5_ERROR                BIT(11)
#define R5_STATE_CMD            (1u << 12)
#define R5_STATE_TRN            (2u << 12)

/* CMD52 and CMD53 arguments */
#define IO_RW                   BIT(31)
#define IO_FN(arg)              (((arg) >> 28) & 7)
#define IO_RAW                  BIT(27)
#define IO_BLOCK                BIT(27)
#define IO_OP_INC               BIT(26)
#define IO_ADDR(arg)            (((arg) >> 9) & 0x1ffff)

/* Function 1's address map */
#define FN1_REG_END             0x400
#define FN1_DATA_END            0x1f800

/* CCCR */
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

#define ABORT_AS_MASK           7
#define ABORT_RES               BIT(3)
#define BUS_IF_WIDTH_MASK       3
#define BUS_IF_WIDTH_4          2
#define BUS_IF_ECSI             BIT(5)
#define BUS_IF_CD_DISABLE       BIT(7)
#define BUS_IF_WR_MASK          (BUS_IF_WIDTH_MASK | BUS_IF_ECSI | \
                                 BUS_IF_CD_DISABLE)
/* Card capability: SDC, SMB and S4MI; E4MI is the host's */
#define CAP_FIXED               0x13
#define CAP_E4MI                BIT(5)
#define SPEED_SHS               BIT(0)
#define SPEED_EHS               BIT(1)

#define FBR_CIS_PTR             0x09
#define FBR_BLKSIZE             0x10

/* The CIS: the common CIS and one per function */
#define CIS_COMMON              0x1000
#define CIS_FN_BASE(fn)         (0x1000 + 0x1000 * (fn))
#define CIS_END                 0x4000
#define CIS_MAX_BLOCK           512
#define CISTPL_MANFID           0x20
#define CISTPL_FUNCID           0x21
#define CISTPL_FUNCE            0x22
#define CISTPL_END              0xff
#define FUNCID_SDIO             0x0c

typedef enum {
    CARD_INIT,
    CARD_READY,
    CARD_STBY,
    CARD_CMD,
    CARD_TRN,
    CARD_INACTIVE,
} Esp32SdioCardState;

static void sdio_kick_send(Esp32SdioSlaveState *s);

static bool sdio_running(Esp32SdioSlaveState *s)
{
    return clock_is_enabled(s->apb_clk);
}

static uint32_t *slc_reg(Esp32SdioSlaveState *s, hwaddr a)
{
    return &s->slc[a / 4];
}

static uint32_t *host_reg(Esp32SdioSlaveState *s, hwaddr a)
{
    return &s->host[a / 4];
}

/* ---- Interrupts ---- */

static bool sdio_fn_pending(Esp32SdioSlaveState *s, unsigned fn)
{
    hwaddr ena0 = fn == 1 ? A_HOST_SLC0_FUNC1_INT_ENA :
                            A_HOST_SLC0_FUNC2_INT_ENA;
    hwaddr ena1 = fn == 1 ? A_HOST_SLC1_FUNC1_INT_ENA :
                            A_HOST_SLC1_FUNC2_INT_ENA;

    return (*host_reg(s, A_HOST_SLC0_INT_RAW) & *host_reg(s, ena0)) ||
           (*host_reg(s, A_HOST_SLC1_INT_RAW) & *host_reg(s, ena1));
}

static SDBus *sdio_card_bus(Esp32SdioSlaveState *s)
{
    BusState *bus = s->card ? qdev_get_parent_bus(s->card) : NULL;

    return bus ? SD_BUS(bus) : NULL;
}

/*
 * [spec:nuos:req:emu.esp32.sdio-slave]
 * The SLC interrupts go to the interrupt matrix. The host's interrupt
 * is function 1's or function 2's pending SLCHOST interrupts, as their
 * FUNCn_INT_ENA select them, passed on as the CCCR's IENx and IENM
 * allow: the card signals it on DAT1.
 */
static void sdio_update_irq(Esp32SdioSlaveState *s)
{
    bool level = false;
    SDBus *bus;

    qemu_set_irq(s->irq[ESP32_SDIO_IRQ_SLC0],
                 (*slc_reg(s, A_SLC_0INT_RAW) &
                  *slc_reg(s, A_SLC_0INT_ENA)) != 0);
    qemu_set_irq(s->irq[ESP32_SDIO_IRQ_SLC1],
                 (*slc_reg(s, A_SLC_1INT_RAW) &
                  *slc_reg(s, A_SLC_1INT_ENA)) != 0);

    if (sdio_running(s) && (s->ien & 1)) {
        level = ((s->ien & BIT(1)) && sdio_fn_pending(s, 1)) ||
                ((s->ien & BIT(2)) && sdio_fn_pending(s, 2));
    }
    bus = sdio_card_bus(s);
    if (level != s->card_irq) {
        s->card_irq = level;
        if (bus) {
            sdbus_set_irq(bus, level);
        }
    }
}

static void slc0_raise(Esp32SdioSlaveState *s, uint32_t bits)
{
    *slc_reg(s, A_SLC_0INT_RAW) |= bits;
}

static void host0_raise(Esp32SdioSlaveState *s, uint32_t bits)
{
    *host_reg(s, A_HOST_SLC0_INT_RAW) |= bits;
}

/* ---- DMA ---- */

static bool sdio_dma_range_ok(uint32_t addr, uint32_t len)
{
    return addr >= DMA_RAM_START && addr < DMA_RAM_END &&
           len <= DMA_RAM_END - addr;
}

static bool sdio_ldl(Esp32SdioSlaveState *s, uint32_t addr, uint32_t *v)
{
    MemTxResult r;

    *v = address_space_ldl_le(&s->dma_as, addr, MEMTXATTRS_UNSPECIFIED, &r);
    return r == MEMTX_OK;
}

static void sdio_stl(Esp32SdioSlaveState *s, uint32_t addr, uint32_t v)
{
    address_space_stl_le(&s->dma_as, addr, v, MEMTXATTRS_UNSPECIFIED, NULL);
}

/*
 * Read the descriptor at addr. It must be word aligned in DMA RAM, with a
 * buffer of len bytes there too, and, with CONF1.SLC0_CHECK_OWNER, owned
 * by the DMA.
 */
static bool sdio_load_desc(Esp32SdioSlaveState *s, uint32_t addr,
                           uint32_t *dw0, uint32_t *buf, uint32_t *next,
                           bool send)
{
    const char *dir = send ? "sending" : "receiving";
    uint32_t len;

    if ((addr & 3) || !sdio_dma_range_ok(addr, 12)) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32_sdio: %s descriptor at "
                      "0x%08x is not word-aligned DMA RAM\n", dir, addr);
        return false;
    }
    if (!sdio_ldl(s, addr, dw0) || !sdio_ldl(s, addr + 4, buf) ||
        !sdio_ldl(s, addr + 8, next)) {
        return false;
    }
    if ((*slc_reg(s, A_SLC_CONF1) & CONF1_SLC0_CHECK_OWNER) &&
        !(*dw0 & DESC_OWNER)) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32_sdio: %s descriptor at "
                      "0x%08x is owned by the CPU\n", dir, addr);
        return false;
    }
    len = send ? DESC_LENGTH(*dw0) : DESC_SIZE(*dw0);
    if ((!send && len == 0) || (len && !sdio_dma_range_ok(*buf, len))) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32_sdio: %s descriptor at "
                      "0x%08x has no buffer of %u bytes in DMA RAM\n",
                      dir, addr, len);
        return false;
    }
    return true;
}

/* The DMA's descriptor registers: the current one and the two before */
static void sdio_shift_dscr(Esp32SdioSlaveState *s, hwaddr reg,
                            uint32_t addr)
{
    *slc_reg(s, reg + 8) = *slc_reg(s, reg + 4);
    *slc_reg(s, reg + 4) = *slc_reg(s, reg);
    *slc_reg(s, reg) = addr;
}

static void sdio_log_undocumented(Esp32SdioSlaveState *s)
{
    uint32_t conf0 = *slc_reg(s, A_SLC_CONF0) &
                     (CONF0_SLC0_TOKEN_AUTO_CLR | CONF0_SLC0_TOKEN_SEL |
                      CONF0_SLC0_WR_RETRY_MASK_EN);
    uint32_t conf1 = *slc_reg(s, A_SLC_CONF1) &
                     (CONF1_SLC0_CHECK_SUM | CONF1_CMD_HOLD_EN |
                      CONF1_SLC0_LEN_AUTO_CLR | CONF1_SLC0_STITCH);
    uint32_t dscr = *slc_reg(s, A_SLC_RX_DSCR_CONF) &
                    RX_DSCR_CONF_SLC0_UNMODELLED;

    if ((conf0 || conf1 || dscr) && !s->undoc_logged) {
        s->undoc_logged = true;
        qemu_log_mask(LOG_UNIMP, "esp32_sdio: SLC0 runs with modes the TRM "
                      "says to clear and does not describe (CONF0 0x%x, "
                      "CONF1 0x%x, RX_DSCR_CONF 0x%x); running without "
                      "them\n", conf0, conf1, dscr);
    }
}

/* ---- Sending: SLC0's RX link ---- */

static void send_fifo_reset(Esp32SdioSendLink *l)
{
    l->head = 0;
    l->num = 0;
    l->eof_in_fifo = false;
}

static void send_reset(Esp32SdioSlaveState *s)
{
    Esp32SdioSendLink *l = &s->send;

    l->active = false;
    l->parked = false;
    l->have_desc = false;
    l->have_last = false;
    l->new_packet = false;
    l->desc_addr = 0;
    l->dw0 = 0;
    l->buf = 0;
    l->next = 0;
    l->pos = 0;
    l->last_addr = 0;
    l->eof_desc = 0;
    l->eof_buf = 0;
    send_fifo_reset(l);
}

static void send_park(Esp32SdioSlaveState *s)
{
    s->send.active = false;
    s->send.parked = true;
}

/* The DMA has taken the whole of the descriptor's buffer. */
static void send_finish_desc(Esp32SdioSlaveState *s)
{
    Esp32SdioSendLink *l = &s->send;
    uint32_t conf0 = *slc_reg(s, A_SLC_CONF0);

    if ((conf0 & CONF0_SLC0_RX_AUTO_WRBACK) &&
        !(conf0 & CONF0_SLC0_RX_LOOP_TEST)) {
        sdio_stl(s, l->desc_addr, l->dw0 & ~DESC_OWNER);
    }
    slc0_raise(s, SLCINT_RX_DONE);
    l->have_desc = false;
    l->have_last = true;
    l->last_addr = l->desc_addr;
    if (l->dw0 & DESC_EOF) {
        l->eof_in_fifo = true;
        l->eof_desc = l->desc_addr;
        l->eof_buf = l->buf;
    } else if (l->next == 0) {
        /* The list ends inside a packet: wait for more descriptors. */
        send_park(s);
    }
}

/*
 * [spec:nuos:req:emu.esp32.sdio-slave]
 * Move the packet into the FIFO as far as it has room, stopping at the
 * end of the packet until the host has read it.
 */
static void send_fill(Esp32SdioSlaveState *s)
{
    Esp32SdioSendLink *l = &s->send;
    uint32_t dw0, buf, next;

    while (l->active && !l->eof_in_fifo &&
           l->num < ESP32_SDIO_FIFO_DEPTH) {
        if (!l->have_desc) {
            uint32_t addr = l->next;

            if (addr == 0) {
                send_park(s);
                break;
            }
            if (!sdio_load_desc(s, addr, &dw0, &buf, &next, true)) {
                slc0_raise(s, SLCINT_RX_DSCR_ERR);
                send_park(s);
                break;
            }
            sdio_shift_dscr(s, A_SLC_0RXLINK_DSCR, addr);
            l->desc_addr = addr;
            l->dw0 = dw0;
            l->buf = buf;
            l->next = next;
            l->pos = 0;
            l->have_desc = true;
            if (l->new_packet) {
                l->new_packet = false;
                host0_raise(s, HOSTINT_RX_NEW_PACKET);
            }
        }
        while (l->pos < DESC_LENGTH(l->dw0) &&
               l->num < ESP32_SDIO_FIFO_DEPTH) {
            uint32_t n = MIN(DESC_LENGTH(l->dw0) - l->pos,
                             ESP32_SDIO_FIFO_DEPTH - l->num);
            uint8_t tmp[ESP32_SDIO_FIFO_DEPTH];

            address_space_read(&s->dma_as, l->buf + l->pos,
                               MEMTXATTRS_UNSPECIFIED, tmp, n);
            for (uint32_t i = 0; i < n; i++) {
                l->fifo[(l->head + l->num) % ESP32_SDIO_FIFO_DEPTH] = tmp[i];
                l->num++;
            }
            l->pos += n;
        }
        if (l->pos == DESC_LENGTH(l->dw0)) {
            send_finish_desc(s);
        }
    }
}

/*
 * [spec:nuos:req:emu.esp32.sdio-slave]
 * The host has read the packet's last byte: RX_EOF, and the DMA goes on
 * to the next packet, if the list has one.
 */
static void send_check_eof(Esp32SdioSlaveState *s)
{
    Esp32SdioSendLink *l = &s->send;

    if (!l->eof_in_fifo || l->num != 0) {
        return;
    }
    l->eof_in_fifo = false;
    *slc_reg(s, A_SLC_0TO_EOF_DES_ADDR) = l->eof_desc;
    *slc_reg(s, A_SLC_0TO_EOF_BFR_DES_ADDR) = l->eof_buf;
    slc0_raise(s, SLCINT_RX_EOF);
    l->new_packet = true;
    if (l->next == 0) {
        send_park(s);
    }
}

static void sdio_kick_send(Esp32SdioSlaveState *s)
{
    if (!sdio_running(s) ||
        (*slc_reg(s, A_SLC_CONF0) &
         (CONF0_SLC0_RX_RST | CONF0_AHBM_RST | CONF0_AHBM_FIFO_RST))) {
        return;
    }
    do {
        send_fill(s);
        send_check_eof(s);
    } while (s->send.active && !s->send.eof_in_fifo &&
             s->send.num < ESP32_SDIO_FIFO_DEPTH);
}

/* [spec:nuos:req:emu.esp32.sdio-slave] */
static uint8_t send_pop(Esp32SdioSlaveState *s)
{
    Esp32SdioSendLink *l = &s->send;
    uint8_t b;

    if (l->num == 0) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32_sdio: host read with the "
                      "slave's send FIFO empty\n");
        slc0_raise(s, SLCINT_RX_UDF);
        host0_raise(s, HOSTINT_RX_UDF);
        return 0;
    }
    b = l->fifo[l->head];
    l->head = (l->head + 1) % ESP32_SDIO_FIFO_DEPTH;
    l->num--;
    send_check_eof(s);
    sdio_kick_send(s);
    return b;
}

/* ---- Receiving: SLC0's TX link ---- */

static void recv_reset(Esp32SdioSlaveState *s)
{
    Esp32SdioRecvLink *l = &s->recv;

    l->active = false;
    l->parked = false;
    l->have_desc = false;
    l->have_last = false;
    l->desc_addr = 0;
    l->dw0 = 0;
    l->buf = 0;
    l->next = 0;
    l->len = 0;
    l->last_addr = 0;
}

/* Hand the descriptor back with its data, and go on to the next. */
static void recv_complete(Esp32SdioSlaveState *s, bool eof)
{
    Esp32SdioRecvLink *l = &s->recv;
    uint32_t dw0 = l->dw0 & ~(DESC_EOF | (0xfffu << DESC_LENGTH_SHIFT));

    dw0 |= l->len << DESC_LENGTH_SHIFT;
    if (eof) {
        dw0 |= DESC_EOF;
    }
    if (!(*slc_reg(s, A_SLC_CONF0) & CONF0_SLC0_TX_LOOP_TEST)) {
        dw0 &= ~DESC_OWNER;
    }
    sdio_stl(s, l->desc_addr, dw0);
    slc0_raise(s, SLCINT_TX_DONE);
    if (eof) {
        *slc_reg(s, A_SLC_0TX_EOF_DES_ADDR) = l->desc_addr;
        slc0_raise(s, SLCINT_TX_SUC_EOF);
    }
    l->have_desc = false;
    l->have_last = true;
    l->last_addr = l->desc_addr;
}

static bool recv_load(Esp32SdioSlaveState *s)
{
    Esp32SdioRecvLink *l = &s->recv;
    uint32_t addr = l->next;
    uint32_t dw0, buf, next;

    if (!l->active || addr == 0) {
        return false;
    }
    if (!sdio_load_desc(s, addr, &dw0, &buf, &next, false)) {
        slc0_raise(s, SLCINT_TX_DSCR_ERR);
        l->active = false;
        l->parked = true;
        return false;
    }
    sdio_shift_dscr(s, A_SLC_0TXLINK_DSCR, addr);
    l->desc_addr = addr;
    l->dw0 = dw0;
    l->buf = buf;
    l->next = next;
    l->len = 0;
    l->have_desc = true;
    return true;
}

/*
 * [spec:nuos:req:emu.esp32.sdio-slave]
 * Store a byte the host writes into the receive buffers. A full buffer is
 * handed back once the next byte shows that the packet goes on, or the
 * packet ends.
 */
static void recv_store(Esp32SdioSlaveState *s, uint8_t b)
{
    Esp32SdioRecvLink *l = &s->recv;

    if (!sdio_running(s) ||
        (*slc_reg(s, A_SLC_CONF0) &
         (CONF0_SLC0_TX_RST | CONF0_AHBM_RST | CONF0_AHBM_FIFO_RST))) {
        return;
    }
    if (l->have_desc && l->len == DESC_SIZE(l->dw0)) {
        recv_complete(s, false);
    }
    if (!l->have_desc && !recv_load(s)) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32_sdio: host data with no "
                      "receive buffer to take it\n");
        slc0_raise(s, SLCINT_TX_OVF);
        host0_raise(s, HOSTINT_TX_OVF);
        return;
    }
    address_space_stb(&s->dma_as, l->buf + l->len, b,
                      MEMTXATTRS_UNSPECIFIED, NULL);
    l->len++;
}

/* The host's write reached the end of the data window. */
static void recv_end_packet(Esp32SdioSlaveState *s)
{
    if (s->recv.have_desc) {
        recv_complete(s, true);
    }
}

/* ---- Link registers ---- */

/* [spec:nuos:req:emu.esp32.sdio-slave] */
static void slc0_rx_link_write(Esp32SdioSlaveState *s, uint32_t value)
{
    Esp32SdioSendLink *l = &s->send;
    uint32_t next;

    *slc_reg(s, A_SLC_0RX_LINK) = value & LINK_ADDR_MASK;
    if (value & LINK_STOP) {
        if (l->active) {
            send_park(s);
        }
    }
    if (value & LINK_START) {
        sdio_log_undocumented(s);
        l->have_desc = false;
        l->new_packet = true;
        l->next = LINK_DESC_BASE | (value & LINK_ADDR_MASK);
        l->active = true;
        l->parked = false;
        slc0_raise(s, SLCINT_RX_START);
    }
    if ((value & LINK_RESTART) && !l->active) {
        /*
         * Go on from where the list stopped: within the descriptor in
         * hand, or at the next field of the last one taken, which
         * software may have pointed at descriptors added since.
         */
        if (l->have_desc) {
            l->active = true;
        } else if (l->have_last && sdio_ldl(s, l->last_addr + 8, &next) &&
                   next) {
            l->next = next;
            l->active = true;
        }
        if (l->active) {
            l->parked = false;
        }
    }
    sdio_kick_send(s);
}

/* [spec:nuos:req:emu.esp32.sdio-slave] */
static void slc0_tx_link_write(Esp32SdioSlaveState *s, uint32_t value)
{
    Esp32SdioRecvLink *l = &s->recv;
    uint32_t next;

    *slc_reg(s, A_SLC_0TX_LINK) = value & LINK_ADDR_MASK;
    if (value & LINK_STOP) {
        if (l->active) {
            l->active = false;
            l->parked = true;
        }
    }
    if (value & LINK_START) {
        sdio_log_undocumented(s);
        l->have_desc = false;
        l->next = LINK_DESC_BASE | (value & LINK_ADDR_MASK);
        l->active = true;
        l->parked = false;
        slc0_raise(s, SLCINT_TX_START);
    }
    if (value & LINK_RESTART) {
        /*
         * As for sending; a link still running but out of descriptors
         * mounts those linked on since.
         */
        if (!l->active) {
            if (l->have_desc) {
                l->active = true;
            } else if (l->have_last &&
                       sdio_ldl(s, l->last_addr + 8, &next) && next) {
                l->next = next;
                l->active = true;
            }
            if (l->active) {
                l->parked = false;
            }
        } else if (!l->have_desc && l->next == 0 && l->have_last &&
                   sdio_ldl(s, l->last_addr + 8, &next)) {
            l->next = next;
        }
    }
}

static uint32_t token_write(uint32_t token, uint32_t value)
{
    uint32_t wdata = value & TOKEN_WDATA_MASK;

    if (value & TOKEN_WR) {
        token = wdata;
    }
    if (value & TOKEN_INC) {
        token++;
    }
    if (value & TOKEN_INC_MORE) {
        token += wdata;
    }
    return token & TOKEN_WDATA_MASK;
}

/* ---- SLC register access ---- */

/* Bits software can write, for the registers that only store */
static uint32_t slc_wmask(hwaddr a)
{
    switch (a) {
    case A_SLC_CONF0:
    case A_SLC_RX_DSCR_CONF:
        return 0xffffffffu;
    case A_SLC_0INT_ENA:
    case A_SLC_0INT_ENA1:
        return SLC0INT_MASK;
    case A_SLC_1INT_ENA:
    case A_SLC_1INT_ENA1:
        return SLC1INT_MASK;
    case A_SLC_CONF1:
        return CONF1_MASK;
    case A_SLC_BRIDGE_CONF:
        return 0xffff7f3fu;
    case A_SLC_AHB_TEST:
        return 0x37;
    case A_SLC_TX_DSCR_CONF:
        return 0x7ff;
    case A_SLC_SEQ_POSITION:
        return 0xffff;
    case A_SLC_0_DSCR_REC_CONF:
        return 0x3ff;
    case A_SLC_0_LEN_LIM_CONF:
        return LEN_WDATA_MASK;
    default:
        return 0;
    }
}

/* [spec:nuos:req:emu.esp32.sdio-slave] */
static uint32_t slc_read_word(Esp32SdioSlaveState *s, hwaddr a)
{
    switch (a) {
    case A_SLC_0INT_ST:
        return *slc_reg(s, A_SLC_0INT_RAW) & *slc_reg(s, A_SLC_0INT_ENA);
    case A_SLC_1INT_ST:
        return *slc_reg(s, A_SLC_1INT_RAW) & *slc_reg(s, A_SLC_1INT_ENA);
    case A_SLC_0INT_ST1:
        return *slc_reg(s, A_SLC_0INT_RAW) & *slc_reg(s, A_SLC_0INT_ENA1);
    case A_SLC_1INT_ST1:
        return *slc_reg(s, A_SLC_1INT_RAW) & *slc_reg(s, A_SLC_1INT_ENA1);
    case A_SLC_RX_STATUS:
        return (s->send.num == ESP32_SDIO_FIFO_DEPTH ? FIFO_FULL : 0) |
               (s->send.num == 0 ? FIFO_EMPTY : 0) | SLC1_FIFO_EMPTY;
    case A_SLC_TX_STATUS:
        /* Received data goes straight to memory. */
        return FIFO_EMPTY | SLC1_FIFO_EMPTY;
    case A_SLC_0RX_LINK:
        return *slc_reg(s, a) | (s->send.parked ? LINK_PARK : 0);
    case A_SLC_0TX_LINK:
        return *slc_reg(s, a) | (s->recv.parked ? LINK_PARK : 0);
    case A_SLC_0TOKEN0:
        return s->slc0_token[0] << TOKEN_SHIFT;
    case A_SLC_0TOKEN1:
        return s->slc0_token[1] << TOKEN_SHIFT;
    case A_SLC_1TOKEN0:
        return s->slc1_token[0] << TOKEN_SHIFT;
    case A_SLC_1TOKEN1:
        return s->slc1_token[1] << TOKEN_SHIFT;
    case A_SLC_0_LENGTH:
        return s->slc0_len;
    case A_SLC_0INT_CLR:
    case A_SLC_1INT_CLR:
    case A_SLC_INTVEC_TOHOST:
    case A_SLC_0_LEN_CONF:
        return 0;
    default:
        return *slc_reg(s, a);
    }
}

/* [spec:nuos:req:emu.esp32.sdio-slave] */
static void slc_write_word(Esp32SdioSlaveState *s, hwaddr a, uint32_t v,
                           uint32_t mask)
{
    uint32_t *r = slc_reg(s, a);

    switch (a) {
    case A_SLC_CONF0:
        *r = (*r & ~mask) | (v & mask);
        /* The resets hold their FSMs while set. */
        if (*r & (CONF0_SLC0_RX_RST | CONF0_AHBM_RST | CONF0_AHBM_FIFO_RST)) {
            send_reset(s);
        }
        if (*r & (CONF0_SLC0_TX_RST | CONF0_AHBM_RST | CONF0_AHBM_FIFO_RST)) {
            recv_reset(s);
        }
        break;
    case A_SLC_0INT_CLR:
        *slc_reg(s, A_SLC_0INT_RAW) &= ~(v & mask);
        break;
    case A_SLC_1INT_CLR:
        *slc_reg(s, A_SLC_1INT_RAW) &= ~(v & mask);
        break;
    case A_SLC_0RX_LINK:
        slc0_rx_link_write(s, (*r & ~mask) | (v & mask));
        break;
    case A_SLC_0TX_LINK:
        slc0_tx_link_write(s, (*r & ~mask) | (v & mask));
        break;
    case A_SLC_1RX_LINK:
    case A_SLC_1TX_LINK:
        v = (*r & ~mask) | (v & mask);
        *r = v & (LINK_ADDR_MASK |
                  (a == A_SLC_1RX_LINK ? LINK_SLC1_BT_PACKET : 0));
        if (v & (LINK_START | LINK_RESTART)) {
            qemu_log_mask(LOG_UNIMP, "esp32_sdio: SLC1's DMA serves "
                          "function 2, whose data path is undocumented; "
                          "its links do not run\n");
        }
        break;
    case A_SLC_INTVEC_TOHOST:
        /* Slave to host interrupts; the vector clears itself. */
        v &= mask;
        *host_reg(s, A_HOST_SLC0_INT_RAW) |= v & HOSTINT_TOHOST_MASK;
        *host_reg(s, A_HOST_SLC1_INT_RAW) |= (v >> 16) & HOSTINT_TOHOST_MASK;
        break;
    case A_SLC_0TOKEN0:
        s->slc0_token[0] = token_write(s->slc0_token[0], v & mask);
        break;
    case A_SLC_0TOKEN1:
        s->slc0_token[1] = token_write(s->slc0_token[1], v & mask);
        break;
    case A_SLC_1TOKEN0:
        s->slc1_token[0] = token_write(s->slc1_token[0], v & mask);
        break;
    case A_SLC_1TOKEN1:
        s->slc1_token[1] = token_write(s->slc1_token[1], v & mask);
        break;
    case A_SLC_0_LEN_CONF:
        v &= mask;
        if (v & LEN_WR) {
            s->slc0_len = v & LEN_WDATA_MASK;
        }
        if (v & LEN_INC) {
            s->slc0_len++;
        }
        if (v & LEN_INC_MORE) {
            s->slc0_len += v & LEN_WDATA_MASK;
        }
        s->slc0_len &= LEN_WDATA_MASK;
        if (v & LEN_CONF_UNMODELLED) {
            qemu_log_mask(LOG_UNIMP, "esp32_sdio: SLC0_LEN_CONF 0x%08x "
                          "requests undocumented packet-load operations\n",
                          v);
        }
        break;
    case A_SLC_0RXFIFO_PUSH:
    case A_SLC_1RXFIFO_PUSH:
    case A_SLC_0TXFIFO_POP:
    case A_SLC_1TXFIFO_POP:
        if (v & mask & (RXFIFO_PUSH | TXFIFO_POP)) {
            qemu_log_mask(LOG_UNIMP, "esp32_sdio: direct FIFO access "
                          "through 0x%03" HWADDR_PRIx " is undocumented\n", a);
        }
        break;
    case A_SLC_SDIO_CRC_ST1:
        /* ERR_CNT_CLR: the CRC error counts are always 0. */
        break;
    default:
        if (slc_wmask(a)) {
            *r = (*r & ~(mask & slc_wmask(a))) | (v & mask & slc_wmask(a));
        } else {
            qemu_log_mask(LOG_GUEST_ERROR, "esp32_sdio: write to read-only "
                          "or reserved SLC register 0x%03" HWADDR_PRIx "\n",
                          a);
        }
        break;
    }
}

/* ---- SLCHOST register access, from the CPU and the host ---- */

static uint32_t host_wmask(hwaddr a)
{
    switch (a) {
    case A_HOST_FUNC2_0:
        return BIT(24);
    case A_HOST_FUNC2_1:
    case A_HOST_FUNC2_2:
        return BIT(0);
    case A_HOST_STATE_W0:
    case A_HOST_STATE_W1:
    case A_HOST_CONF_W0 ... A_HOST_CONF_W5:
    case A_HOST_WIN_CMD:
    case A_HOST_CONF_W6:
    case A_HOST_CONF_W8 ... A_HOST_CONF_W15:
    case A_HOST_SLC0_LEN_WD:
    case A_HOST_APBWIN_WDATA:
        return 0xffffffffu;
    case A_HOST_SLC0_TOKEN_WDATA:
    case A_HOST_SLC1_TOKEN_WDATA:
        return 0x0fff0fffu;
    case A_HOST_SLC0_FUNC1_INT_ENA ... A_HOST_SLC1_INT_ENA:
    case A_HOST_SLC0_INT_ENA1:
    case A_HOST_SLC1_INT_ENA1:
        return HOSTINT_MASK;
    case A_HOST_SLC0_RX_INFOR:
    case A_HOST_SLC1_RX_INFOR:
        return LEN_WDATA_MASK;
    case A_HOST_APBWIN_CONF:
        return 0x1fffffffu;
    case A_HOST_SLC0_RDCLR:
    case A_HOST_SLC1_RDCLR:
        return 0x3ffff;
    case A_HOST_CONF:
        return 0x0fffffffu;
    default:
        return 0;
    }
}

static uint32_t host_token_rdata(uint32_t *token)
{
    return token[0] | (token[1] << TOKEN_SHIFT);
}

static uint32_t pkt_len_value(uint32_t len)
{
    uint32_t check = ((len & 0x3ff) + (len >> 10)) & 0xfff;

    return len | (check << PKT_LEN_CHECK_SHIFT);
}

/* [spec:nuos:req:emu.esp32.sdio-slave] */
static uint32_t host_read_word(Esp32SdioSlaveState *s, hwaddr a)
{
    switch (a) {
    case A_HOST_SLC0_TOKEN_RDATA:
        return host_token_rdata(s->slc0_token);
    case A_HOST_SLC1_TOKEN_RDATA:
        return host_token_rdata(s->slc1_token);
    case A_HOST_SLC0_INT_ST:
        return *host_reg(s, A_HOST_SLC0_INT_RAW) &
               *host_reg(s, A_HOST_SLC0_FUNC1_INT_ENA);
    case A_HOST_SLC1_INT_ST:
        return *host_reg(s, A_HOST_SLC1_INT_RAW) &
               *host_reg(s, A_HOST_SLC1_FUNC1_INT_ENA);
    case A_HOST_PKT_LEN:
        return pkt_len_value(s->pkt_len_latch);
    case A_HOST_TOKEN_CON:
    case A_HOST_SLC0_INT_CLR:
    case A_HOST_SLC1_INT_CLR:
        return 0;
    default:
        return *host_reg(s, a);
    }
}

/*
 * [spec:nuos:req:emu.esp32.sdio-slave]
 * The host reads SLCHOST a byte at a time. Reading the first byte of
 * PKT_LEN takes SLC0_LEN into it, so that the host reads one value
 * whole; the TRM has the register update only when the host reads it.
 * Reading TOKEN_RDATA likewise latches the counts into SLC_TOKEN_LAT.
 */
static void host_read_side_effects(Esp32SdioSlaveState *s, hwaddr a)
{
    if (a == A_HOST_PKT_LEN) {
        s->pkt_len_latch = s->slc0_len;
    } else if (a == A_HOST_SLC0_TOKEN_RDATA || a == A_HOST_SLC1_TOKEN_RDATA) {
        *slc_reg(s, A_SLC_TOKEN_LAT) = s->slc0_token[1] |
                                       (s->slc1_token[1] << TOKEN_SHIFT);
    }
}

static uint32_t token_dec(uint32_t token)
{
    return (token - 1) & TOKEN_WDATA_MASK;
}

/* [spec:nuos:req:emu.esp32.sdio-slave] */
static void host_write_word(Esp32SdioSlaveState *s, hwaddr a, uint32_t v,
                            uint32_t mask, bool from_host)
{
    uint32_t *r = host_reg(s, a);

    switch (a) {
    case A_HOST_CONF_W7:
        /*
         * Byte 0 is the vector by which the host interrupts the slave,
         * SLC0's FRHOST bits; byte 1 is SLC1's (its FRHOST_BIT8..15).
         * The vector clears itself; bytes 2 and 3 store.
         */
        v &= mask;
        *slc_reg(s, A_SLC_0INT_RAW) |= v & SLCINT_FRHOST_MASK;
        *slc_reg(s, A_SLC_1INT_RAW) |= (v >> 8) & SLCINT_FRHOST_MASK;
        *r = (*r & ~(mask & 0xffff0000u)) | (v & 0xffff0000u);
        break;
    case A_HOST_SLC0_INT_CLR:
        *host_reg(s, A_HOST_SLC0_INT_RAW) &= ~(v & mask);
        break;
    case A_HOST_SLC1_INT_CLR:
        *host_reg(s, A_HOST_SLC1_INT_RAW) &= ~(v & mask);
        break;
    case A_HOST_TOKEN_CON: {
        /*
         * The host's side of the counters: DEC takes one off a count,
         * WR loads it from TOKEN_WDATA, SLC0_LEN_WR loads SLC0_LEN from
         * SLC0_LEN_WD. The TRM does not describe these; the model reads
         * them as the counterparts of SLC_TOKEN's INC and WR.
         */
        uint32_t *tok[2] = { s->slc0_token, s->slc1_token };
        hwaddr wd[2] = { A_HOST_SLC0_TOKEN_WDATA, A_HOST_SLC1_TOKEN_WDATA };

        v &= mask;
        for (int c = 0; c < 2; c++) {
            uint32_t bits = v >> (c * TOKEN_CON_SLC1_SHIFT);
            uint32_t wdata = *host_reg(s, wd[c]);

            for (int t = 0; t < 2; t++) {
                if (bits & (TOKEN_CON_SLC0_TOKEN0_DEC << t)) {
                    tok[c][t] = token_dec(tok[c][t]);
                }
                if (bits & (TOKEN_CON_SLC0_TOKEN0_WR << t)) {
                    tok[c][t] = (wdata >> (t * TOKEN_SHIFT)) &
                                TOKEN_WDATA_MASK;
                }
            }
        }
        if (v & TOKEN_CON_SLC0_LEN_WR) {
            s->slc0_len = *host_reg(s, A_HOST_SLC0_LEN_WD) & LEN_WDATA_MASK;
        }
        break;
    }
    case A_HOST_APBWIN_CONF:
        *r = (*r & ~(mask & host_wmask(a))) | (v & mask & host_wmask(a));
        if (v & mask & APBWIN_START) {
            qemu_log_mask(LOG_UNIMP, "esp32_sdio: the host's APB window "
                          "is undocumented and does not run\n");
        }
        break;
    default:
        if (host_wmask(a)) {
            *r = (*r & ~(mask & host_wmask(a))) | (v & mask & host_wmask(a));
        } else {
            qemu_log_mask(LOG_GUEST_ERROR, "esp32_sdio: %s write to "
                          "read-only or reserved SLCHOST register 0x%03"
                          HWADDR_PRIx "\n", from_host ? "host" : "CPU", a);
        }
        break;
    }
}

/* ---- HINF register access ---- */

static uint32_t hinf_wmask(hwaddr a)
{
    switch (a) {
    case A_HINF_CFG_DATA0:
    case A_HINF_CIS_CONF0 ... A_HINF_CIS_CONF7:
    case A_HINF_CFG_DATA16:
        return 0xffffffffu;
    case A_HINF_CFG_DATA1:
        return (uint32_t)~CFG1_HOST_STATUS;
    case A_HINF_CFG_DATA7:
        return CFG7_MASK;
    default:
        return 0;
    }
}

/*
 * [spec:nuos:req:emu.esp32.sdio-slave]
 * HINF_CFG_DATA1's IOENABLE1, IOENABLE2 and HIGHSPEED_MODE show the
 * host's function enables and high-speed selection, from the CCCR.
 */
static uint32_t hinf_read_word(Esp32SdioSlaveState *s, hwaddr a)
{
    uint32_t v = s->hinf[a / 4];

    if (a == A_HINF_CFG_DATA1) {
        v &= ~CFG1_HOST_STATUS;
        v |= (s->ioe & BIT(1)) ? CFG1_IOENABLE1 : 0;
        v |= (s->ioe & BIT(2)) ? CFG1_IOENABLE2 : 0;
        v |= (s->speed_wr & SPEED_EHS) ? CFG1_HIGHSPEED_MODE : 0;
    }
    return v;
}

static void hinf_write_word(Esp32SdioSlaveState *s, hwaddr a, uint32_t v,
                            uint32_t mask)
{
    uint32_t m = mask & hinf_wmask(a);

    if (!hinf_wmask(a)) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32_sdio: write to read-only or "
                      "reserved HINF register 0x%03" HWADDR_PRIx "\n", a);
        return;
    }
    s->hinf[a / 4] = (s->hinf[a / 4] & ~m) | (v & m);
    if (a == A_HINF_CFG_DATA7 && (v & m & CFG7_SDIO_RST)) {
        qemu_log_mask(LOG_UNIMP, "esp32_sdio: HINF_SDIO_RST is "
                      "undocumented and has no effect\n");
    }
}

/* ---- CPU access ---- */

typedef enum {
    BLOCK_SLC,
    BLOCK_HOST,
    BLOCK_HINF,
} Esp32SdioBlock;

static uint32_t block_read_word(Esp32SdioSlaveState *s, Esp32SdioBlock b,
                                hwaddr a)
{
    switch (b) {
    case BLOCK_SLC:
        return a < ESP32_SDIO_SLC_WORDS * 4 ? slc_read_word(s, a) : 0;
    case BLOCK_HOST:
        return a < ESP32_SDIO_HOST_WORDS * 4 ? host_read_word(s, a) : 0;
    case BLOCK_HINF:
    default:
        return a < ESP32_SDIO_HINF_WORDS * 4 ? hinf_read_word(s, a) : 0;
    }
}

static void block_write_word(Esp32SdioSlaveState *s, Esp32SdioBlock b,
                             hwaddr a, uint32_t v, uint32_t mask,
                             bool from_host)
{
    switch (b) {
    case BLOCK_SLC:
        if (a < ESP32_SDIO_SLC_WORDS * 4) {
            slc_write_word(s, a, v, mask);
        }
        break;
    case BLOCK_HOST:
        if (a < ESP32_SDIO_HOST_WORDS * 4) {
            host_write_word(s, a, v, mask, from_host);
        }
        break;
    case BLOCK_HINF:
    default:
        if (a < ESP32_SDIO_HINF_WORDS * 4) {
            hinf_write_word(s, a, v, mask);
        }
        break;
    }
}

/*
 * Accesses narrower than a word read or write their lanes of the word;
 * outside the registers, the 4 KiB blocks read 0 and ignore writes.
 */
static uint64_t sdio_block_read(Esp32SdioSlaveState *s, Esp32SdioBlock b,
                                hwaddr addr, unsigned size)
{
    hwaddr a = addr & ~3;
    unsigned shift = (addr & 3) * 8;
    uint64_t v;

    v = block_read_word(s, b, a) >> shift;
    return size == 4 ? v : v & MAKE_64BIT_MASK(0, size * 8);
}

static void sdio_block_write(Esp32SdioSlaveState *s, Esp32SdioBlock b,
                             hwaddr addr, uint64_t value, unsigned size)
{
    hwaddr a = addr & ~3;
    unsigned shift = (addr & 3) * 8;
    uint32_t mask = MAKE_64BIT_MASK(shift, size * 8);

    block_write_word(s, b, a, value << shift, mask, false);
    sdio_kick_send(s);
    sdio_update_irq(s);
}

static uint64_t sdio_slc_read(void *opaque, hwaddr addr, unsigned size)
{
    return sdio_block_read(opaque, BLOCK_SLC, addr, size);
}

static void sdio_slc_write(void *opaque, hwaddr addr, uint64_t value,
                           unsigned size)
{
    sdio_block_write(opaque, BLOCK_SLC, addr, value, size);
}

static uint64_t sdio_host_read(void *opaque, hwaddr addr, unsigned size)
{
    return sdio_block_read(opaque, BLOCK_HOST, addr, size);
}

static void sdio_host_write(void *opaque, hwaddr addr, uint64_t value,
                            unsigned size)
{
    sdio_block_write(opaque, BLOCK_HOST, addr, value, size);
}

static uint64_t sdio_hinf_read(void *opaque, hwaddr addr, unsigned size)
{
    return sdio_block_read(opaque, BLOCK_HINF, addr, size);
}

static void sdio_hinf_write(void *opaque, hwaddr addr, uint64_t value,
                            unsigned size)
{
    sdio_block_write(opaque, BLOCK_HINF, addr, value, size);
}

#define SDIO_OPS(name) \
    static const MemoryRegionOps sdio_##name##_ops = { \
        .read = sdio_##name##_read, \
        .write = sdio_##name##_write, \
        .endianness = DEVICE_LITTLE_ENDIAN, \
        .valid.min_access_size = 1, \
        .valid.max_access_size = 4, \
        .valid.unaligned = false, \
        .impl.min_access_size = 1, \
        .impl.max_access_size = 4, \
    }

SDIO_OPS(slc);
SDIO_OPS(host);
SDIO_OPS(hinf);

/* ---- The card: function 0 ---- */

static uint8_t cccr_ior(Esp32SdioSlaveState *s)
{
    uint32_t cfg1 = s->hinf[A_HINF_CFG_DATA1 / 4];
    uint8_t ready = 0;

    ready |= (cfg1 & CFG1_SDIO_IOREADY1) ? BIT(1) : 0;
    ready |= (cfg1 & CFG1_SDIO_IOREADY2) ? BIT(2) : 0;
    return ready & s->ioe;
}

static uint16_t fn_id(Esp32SdioSlaveState *s, unsigned fn, bool device)
{
    uint32_t cfg = s->hinf[(fn == 2 ? A_HINF_CFG_DATA16 :
                                      A_HINF_CFG_DATA0) / 4];

    return device ? cfg >> 16 : cfg & 0xffff;
}

/*
 * [spec:nuos:req:emu.esp32.sdio-slave]
 * The CIS. The common CIS gives the manufacturer ID (function 1's IDs in
 * HINF_CFG_DATA0), the SDIO function ID and function 0's extension, then
 * the 32 bytes of HINF_CIS_CONF0..7 for software's own tuples (zeros are
 * null tuples), then the end tuple. Each function's CIS gives its IDs,
 * the function ID and the SDIO 1.1 function extension.
 */
static uint8_t cis_byte(Esp32SdioSlaveState *s, uint32_t addr)
{
    uint8_t t[64];
    unsigned n = 0;
    unsigned fn = addr / 0x1000 - 1;
    uint32_t off = addr % 0x1000;
    uint16_t vendor = fn_id(s, fn == 2 ? 2 : 1, false);
    uint16_t device = fn_id(s, fn == 2 ? 2 : 1, true);

    t[n++] = CISTPL_MANFID;
    t[n++] = 4;
    t[n++] = vendor;
    t[n++] = vendor >> 8;
    t[n++] = device;
    t[n++] = device >> 8;
    t[n++] = CISTPL_FUNCID;
    t[n++] = 2;
    t[n++] = FUNCID_SDIO;
    t[n++] = 0;
    if (fn == 0) {
        /* Function 0's extension: maximum block size and 25 MHz */
        t[n++] = CISTPL_FUNCE;
        t[n++] = 4;
        t[n++] = 0;
        t[n++] = CIS_MAX_BLOCK & 0xff;
        t[n++] = CIS_MAX_BLOCK >> 8;
        t[n++] = 0x32;
        if (off >= n && off < n + 32) {
            return s->hinf[A_HINF_CIS_CONF0 / 4 + (off - n) / 4] >>
                   ((off - n) % 4 * 8);
        }
        if (off >= n + 32) {
            return off == n + 32 ? CISTPL_END : 0;
        }
    } else {
        /*
         * The function extension, 42 bytes: no standard function, no
         * CSA, the maximum block size, the OCR, and an enable timeout
         * of 10 ms; the power fields are 0.
         */
        uint8_t funce[42] = { 1 };

        funce[1] = 0x01;
        funce[2] = 0x20;
        funce[12] = CIS_MAX_BLOCK & 0xff;
        funce[13] = CIS_MAX_BLOCK >> 8;
        stl_le_p(&funce[14], SDIO_OCR);
        funce[28] = 1;
        t[n++] = CISTPL_FUNCE;
        t[n++] = sizeof(funce);
        memcpy(&t[n], funce, sizeof(funce));
        n += sizeof(funce);
        t[n++] = CISTPL_END;
    }
    return off < n ? t[off] : 0;
}

/* [spec:nuos:req:emu.esp32.sdio-slave] */
static uint8_t fn0_read(Esp32SdioSlaveState *s, uint32_t addr)
{
    uint32_t cfg1 = s->hinf[A_HINF_CFG_DATA1 / 4];
    unsigned fn;

    if (addr >= CIS_COMMON && addr < CIS_END) {
        return cis_byte(s, addr);
    }
    if (addr >= 0x100 && addr < 0x100 * (SDIO_FUNCTIONS + 1)) {
        fn = addr >> 8;
        switch (addr & 0xff) {
        case FBR_CIS_PTR:
            return CIS_FN_BASE(fn) & 0xff;
        case FBR_CIS_PTR + 1:
            return CIS_FN_BASE(fn) >> 8;
        case FBR_CIS_PTR + 2:
            return CIS_FN_BASE(fn) >> 16;
        case FBR_BLKSIZE:
            return s->blksize[fn];
        case FBR_BLKSIZE + 1:
            return s->blksize[fn] >> 8;
        default:
            return 0;
        }
    }
    switch (addr) {
    case CCCR_REV:
        return cfg1 >> CFG1_SDIO_VER_SHIFT;
    case CCCR_SD_REV:
        return (cfg1 >> (CFG1_SDIO_VER_SHIFT + 8)) & 0xf;
    case CCCR_IOE:
        return s->ioe;
    case CCCR_IOR:
        return cccr_ior(s);
    case CCCR_IEN:
        return s->ien;
    case CCCR_INT:
        return (sdio_fn_pending(s, 1) ? BIT(1) : 0) |
               (sdio_fn_pending(s, 2) ? BIT(2) : 0);
    case CCCR_BUS_IF:
        return s->bus_if;
    case CCCR_CAP:
        return CAP_FIXED | s->cap_wr;
    case CCCR_CIS_PTR:
        return CIS_COMMON & 0xff;
    case CCCR_CIS_PTR + 1:
        return CIS_COMMON >> 8;
    case CCCR_CIS_PTR + 2:
        return CIS_COMMON >> 16;
    case CCCR_BLKSIZE:
        return s->blksize[0];
    case CCCR_BLKSIZE + 1:
        return s->blksize[0] >> 8;
    case CCCR_SPEED:
        return ((cfg1 & CFG1_HIGHSPEED_ENABLE) ? SPEED_SHS : 0) |
               s->speed_wr;
    default:
        return 0;
    }
}

static void sdio_card_reset_state(Esp32SdioSlaveState *s)
{
    s->card_state = CARD_INIT;
    s->rca = 0;
    s->ioe = 0;
    s->ien = 0;
    s->bus_if = 0;
    s->cap_wr = 0;
    s->speed_wr = 0;
    memset(s->blksize, 0, sizeof(s->blksize));
    s->xfer_active = false;
    s->xfer_write = false;
    s->xfer_inc = false;
    s->xfer_endless = false;
    s->xfer_fn = 0;
    s->xfer_addr = 0;
    s->xfer_left = 0;
}

static void set_blksize_byte(uint16_t *bs, unsigned hi, uint8_t v)
{
    *bs = hi ? (*bs & 0xff) | (v << 8) : (*bs & 0xff00) | v;
}

/* [spec:nuos:req:emu.esp32.sdio-slave] */
static void fn0_write(Esp32SdioSlaveState *s, uint32_t addr, uint8_t v)
{
    if (addr >= 0x100 && addr < 0x100 * (SDIO_FUNCTIONS + 1)) {
        if ((addr & 0xff) == FBR_BLKSIZE || (addr & 0xff) == FBR_BLKSIZE + 1) {
            set_blksize_byte(&s->blksize[addr >> 8], addr & 1, v);
        }
        return;
    }
    switch (addr) {
    case CCCR_IOE:
        s->ioe = v & (BIT(1) | BIT(2));
        break;
    case CCCR_IEN:
        s->ien = v & (BIT(0) | BIT(1) | BIT(2));
        break;
    case CCCR_ABORT:
        if (v & ABORT_RES) {
            /* The I/O reset: the card starts again from its initialization */
            sdio_card_reset_state(s);
        } else if (s->xfer_active && (v & ABORT_AS_MASK) == s->xfer_fn) {
            s->xfer_active = false;
            s->card_state = CARD_CMD;
        }
        break;
    case CCCR_BUS_IF:
        if ((v & BUS_IF_WIDTH_MASK) != 0 &&
            (v & BUS_IF_WIDTH_MASK) != BUS_IF_WIDTH_4) {
            qemu_log_mask(LOG_GUEST_ERROR, "esp32_sdio: host selects a "
                          "reserved bus width %u\n", v & BUS_IF_WIDTH_MASK);
            v &= ~BUS_IF_WIDTH_MASK;
        }
        s->bus_if = v & BUS_IF_WR_MASK;
        break;
    case CCCR_CAP:
        s->cap_wr = v & CAP_E4MI;
        break;
    case CCCR_BLKSIZE:
    case CCCR_BLKSIZE + 1:
        set_blksize_byte(&s->blksize[0], addr & 1, v);
        break;
    case CCCR_SPEED:
        if (s->hinf[A_HINF_CFG_DATA1 / 4] & CFG1_HIGHSPEED_ENABLE) {
            s->speed_wr = v & SPEED_EHS;
        }
        break;
    default:
        break;
    }
}

/* ---- The card: functions 1 and 2 ---- */

static uint8_t fn_read(Esp32SdioSlaveState *s, unsigned fn, uint32_t addr)
{
    hwaddr a;

    switch (fn) {
    case 0:
        return fn0_read(s, addr);
    case 1:
        if (addr < FN1_REG_END) {
            a = addr & 0x3fc;
            if ((addr & 3) == 0) {
                host_read_side_effects(s, a);
            }
            return block_read_word(s, BLOCK_HOST, a) >> ((addr & 3) * 8);
        }
        if (addr < FN1_DATA_END) {
            return send_pop(s);
        }
        /* Padding past the end of the data */
        return 0;
    default:
        qemu_log_mask(LOG_UNIMP, "esp32_sdio: function 2 read at 0x%05x: "
                      "its address map is undocumented\n", addr);
        return 0;
    }
}

static void fn_write(Esp32SdioSlaveState *s, unsigned fn, uint32_t addr,
                     uint8_t v)
{
    unsigned shift = (addr & 3) * 8;

    switch (fn) {
    case 0:
        fn0_write(s, addr, v);
        break;
    case 1:
        if (addr < FN1_REG_END) {
            block_write_word(s, BLOCK_HOST, addr & 0x3fc, (uint32_t)v << shift,
                             0xffu << shift, true);
        } else if (addr < FN1_DATA_END) {
            recv_store(s, v);
            if (addr == FN1_DATA_END - 1) {
                recv_end_packet(s);
            }
        }
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "esp32_sdio: function 2 write at 0x%05x: "
                      "its address map is undocumented\n", addr);
        break;
    }
}

/* ---- The card's SD interface ---- */

static Esp32SdioSlaveState *card_slave(SDState *sd)
{
    return ESP32_SDIO_SLAVE(OBJECT(sd)->parent);
}

static size_t card_resp32(uint8_t *resp, size_t respsz, uint32_t v)
{
    assert(respsz >= 4);
    stl_be_p(resp, v);
    return 4;
}

static uint32_t r5_state(Esp32SdioSlaveState *s)
{
    switch (s->card_state) {
    case CARD_CMD:
        return R5_STATE_CMD;
    case CARD_TRN:
        return R5_STATE_TRN;
    default:
        return 0;
    }
}

/* [spec:nuos:req:emu.esp32.sdio-slave] */
static uint32_t card_cmd52(Esp32SdioSlaveState *s, uint32_t arg)
{
    unsigned fn = IO_FN(arg);
    uint32_t addr = IO_ADDR(arg);
    uint8_t data = arg & 0xff;
    uint32_t state = r5_state(s);

    if (fn > SDIO_FUNCTIONS) {
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

/* [spec:nuos:req:emu.esp32.sdio-slave] */
static uint32_t card_cmd53(Esp32SdioSlaveState *s, uint32_t arg)
{
    unsigned fn = IO_FN(arg);
    uint32_t count = arg & 0x1ff;
    uint32_t state = r5_state(s);

    if (fn > SDIO_FUNCTIONS) {
        return state | R5_FUNCTION_NUMBER;
    }
    s->xfer_endless = false;
    if (arg & IO_BLOCK) {
        if (s->blksize[fn] == 0 || s->blksize[fn] > CIS_MAX_BLOCK) {
            qemu_log_mask(LOG_GUEST_ERROR, "esp32_sdio: block mode CMD53 "
                          "with function %u's block size %u\n", fn,
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
    return state;
}

static void card_xfer_advance(Esp32SdioSlaveState *s)
{
    if (s->xfer_inc) {
        s->xfer_addr = (s->xfer_addr + 1) & 0x1ffff;
    }
    if (!s->xfer_endless && --s->xfer_left == 0) {
        s->xfer_active = false;
        s->card_state = CARD_CMD;
    }
}

/*
 * [spec:nuos:req:emu.esp32.sdio-slave]
 * The card's commands: CMD5, CMD3 and CMD7 bring it up, CMD52 and CMD53
 * reach its functions. As an I/O-only card it does not answer the memory
 * card commands. With its clock stopped or held in reset it answers
 * nothing.
 */
static size_t card_do_command(SDState *sd, SDRequest *req, uint8_t *resp,
                              size_t respsz)
{
    Esp32SdioSlaveState *s = card_slave(sd);
    size_t n = 0;

    if (!sdio_running(s) || s->card_state == CARD_INACTIVE) {
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
        n = card_resp32(resp, respsz, R4_READY |
                        (SDIO_FUNCTIONS << R4_NF_SHIFT) | SDIO_OCR);
        break;
    case 3:
        if (s->card_state != CARD_READY && s->card_state != CARD_STBY) {
            break;
        }
        s->rca = s->rca + 1 ? s->rca + 1 : 1;
        s->card_state = CARD_STBY;
        /* R6: the RCA and the status, in the stand-by state */
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
    sdio_kick_send(s);
    sdio_update_irq(s);
    return n;
}

/* [spec:nuos:req:emu.esp32.sdio-slave] */
static size_t card_read_data(SDState *sd, void *buf, size_t length)
{
    Esp32SdioSlaveState *s = card_slave(sd);
    uint8_t *p = buf;

    for (size_t i = 0; i < length; i++) {
        if (!s->xfer_active || s->xfer_write) {
            qemu_log_mask(LOG_GUEST_ERROR, "esp32_sdio: host reads data "
                          "the card is not sending\n");
            memset(p + i, 0, length - i);
            break;
        }
        p[i] = fn_read(s, s->xfer_fn, s->xfer_addr);
        card_xfer_advance(s);
    }
    sdio_update_irq(s);
    return length;
}

/* [spec:nuos:req:emu.esp32.sdio-slave] */
static size_t card_write_data(SDState *sd, const void *buf, size_t length)
{
    Esp32SdioSlaveState *s = card_slave(sd);
    const uint8_t *p = buf;

    for (size_t i = 0; i < length; i++) {
        if (!s->xfer_active || !s->xfer_write) {
            qemu_log_mask(LOG_GUEST_ERROR, "esp32_sdio: host writes data "
                          "the card is not receiving\n");
            break;
        }
        fn_write(s, s->xfer_fn, s->xfer_addr, p[i]);
        card_xfer_advance(s);
    }
    sdio_kick_send(s);
    sdio_update_irq(s);
    return length;
}

static bool card_data_ready(SDState *sd)
{
    Esp32SdioSlaveState *s = card_slave(sd);

    return s->xfer_active && !s->xfer_write;
}

static bool card_receive_ready(SDState *sd)
{
    Esp32SdioSlaveState *s = card_slave(sd);

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
    return card_slave(sd)->card_irq ? 0xd : 0xf;
}

static bool card_get_cmd_line(SDState *sd)
{
    return true;
}

static void card_realize(DeviceState *dev, Error **errp)
{
    if (!object_dynamic_cast(OBJECT(dev)->parent, TYPE_ESP32_SDIO_SLAVE)) {
        error_setg(errp, "%s: only an ESP32 SDIO slave can create its card",
                   TYPE_ESP32_SDIO_CARD);
    }
}

/* The card's state is the slave's, which the SoC resets. */
static void card_reset(DeviceState *dev)
{
}

static void card_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SDCardClass *sc = SDMMC_COMMON_CLASS(klass);

    dc->desc = "ESP32 SDIO slave card";
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

void esp32_sdio_slave_attach(Esp32SdioSlaveState *s, SDBus *bus)
{
    Object *card = object_new(TYPE_ESP32_SDIO_CARD);

    assert(!s->card);
    object_property_add_child(OBJECT(s), "card", card);
    object_unref(card);
    s->card = DEVICE(card);
    qdev_realize(s->card, BUS(bus), &error_fatal);
    sdbus_set_irq(bus, s->card_irq);
}

/* ---- The device ---- */

/* [spec:nuos:req:emu.esp32.sdio-slave] */
static void esp32_sdio_reset_hold(Object *obj, ResetType type)
{
    Esp32SdioSlaveState *s = ESP32_SDIO_SLAVE(obj);

    memset(s->slc, 0, sizeof(s->slc));
    memset(s->host, 0, sizeof(s->host));
    memset(s->hinf, 0, sizeof(s->hinf));
    *slc_reg(s, A_SLC_CONF0) = CONF0_RESET;
    *slc_reg(s, A_SLC_CONF1) = CONF1_RESET;
    s->hinf[A_HINF_CFG_DATA0 / 4] = CFG_DATA0_RESET;
    s->hinf[A_HINF_CFG_DATA16 / 4] = CFG_DATA16_RESET;
    memset(s->slc0_token, 0, sizeof(s->slc0_token));
    memset(s->slc1_token, 0, sizeof(s->slc1_token));
    s->slc0_len = 0;
    s->pkt_len_latch = 0;
    s->undoc_logged = false;
    send_reset(s);
    recv_reset(s);
    sdio_card_reset_state(s);
    sdio_update_irq(s);
}

static void esp32_sdio_clk_update(void *opaque, ClockEvent event)
{
    Esp32SdioSlaveState *s = opaque;

    sdio_kick_send(s);
    sdio_update_irq(s);
}

static void esp32_sdio_init(Object *obj)
{
    Esp32SdioSlaveState *s = ESP32_SDIO_SLAVE(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->slc_mr, obj, &sdio_slc_ops, s,
                          "esp32.slc", ESP32_SDIO_BLOCK_SIZE);
    memory_region_init_io(&s->host_mr, obj, &sdio_host_ops, s,
                          "esp32.slchost", ESP32_SDIO_BLOCK_SIZE);
    memory_region_init_io(&s->hinf_mr, obj, &sdio_hinf_ops, s,
                          "esp32.hinf", ESP32_SDIO_BLOCK_SIZE);
    sysbus_init_mmio(sbd, &s->slc_mr);
    sysbus_init_mmio(sbd, &s->host_mr);
    sysbus_init_mmio(sbd, &s->hinf_mr);
    for (int i = 0; i < ESP32_SDIO_IRQ_COUNT; i++) {
        sysbus_init_irq(sbd, &s->irq[i]);
    }
    s->apb_clk = qdev_init_clock_in(DEVICE(obj), "apb", esp32_sdio_clk_update,
                                    s, ClockUpdate);
}

static void esp32_sdio_realize(DeviceState *dev, Error **errp)
{
    Esp32SdioSlaveState *s = ESP32_SDIO_SLAVE(dev);

    if (!s->dma_mr) {
        error_setg(errp, "esp32_sdio: 'dma-mr' link not set");
        return;
    }
    address_space_init(&s->dma_as, s->dma_mr, "esp32-sdio-dma");
}

static const Property esp32_sdio_properties[] = {
    DEFINE_PROP_LINK("dma-mr", Esp32SdioSlaveState, dma_mr,
                     TYPE_MEMORY_REGION, MemoryRegion *),
};

static const VMStateDescription vmstate_esp32_sdio_send = {
    .name = "esp32-sdio-send",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_BOOL(active, Esp32SdioSendLink),
        VMSTATE_BOOL(parked, Esp32SdioSendLink),
        VMSTATE_BOOL(have_desc, Esp32SdioSendLink),
        VMSTATE_BOOL(have_last, Esp32SdioSendLink),
        VMSTATE_BOOL(new_packet, Esp32SdioSendLink),
        VMSTATE_BOOL(eof_in_fifo, Esp32SdioSendLink),
        VMSTATE_UINT32(desc_addr, Esp32SdioSendLink),
        VMSTATE_UINT32(dw0, Esp32SdioSendLink),
        VMSTATE_UINT32(buf, Esp32SdioSendLink),
        VMSTATE_UINT32(next, Esp32SdioSendLink),
        VMSTATE_UINT32(pos, Esp32SdioSendLink),
        VMSTATE_UINT32(last_addr, Esp32SdioSendLink),
        VMSTATE_UINT32(eof_desc, Esp32SdioSendLink),
        VMSTATE_UINT32(eof_buf, Esp32SdioSendLink),
        VMSTATE_UINT8_ARRAY(fifo, Esp32SdioSendLink, ESP32_SDIO_FIFO_DEPTH),
        VMSTATE_UINT32(head, Esp32SdioSendLink),
        VMSTATE_UINT32(num, Esp32SdioSendLink),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_esp32_sdio_recv = {
    .name = "esp32-sdio-recv",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_BOOL(active, Esp32SdioRecvLink),
        VMSTATE_BOOL(parked, Esp32SdioRecvLink),
        VMSTATE_BOOL(have_desc, Esp32SdioRecvLink),
        VMSTATE_BOOL(have_last, Esp32SdioRecvLink),
        VMSTATE_UINT32(desc_addr, Esp32SdioRecvLink),
        VMSTATE_UINT32(dw0, Esp32SdioRecvLink),
        VMSTATE_UINT32(buf, Esp32SdioRecvLink),
        VMSTATE_UINT32(next, Esp32SdioRecvLink),
        VMSTATE_UINT32(len, Esp32SdioRecvLink),
        VMSTATE_UINT32(last_addr, Esp32SdioRecvLink),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_esp32_sdio = {
    .name = TYPE_ESP32_SDIO_SLAVE,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(slc, Esp32SdioSlaveState, ESP32_SDIO_SLC_WORDS),
        VMSTATE_UINT32_ARRAY(host, Esp32SdioSlaveState,
                             ESP32_SDIO_HOST_WORDS),
        VMSTATE_UINT32_ARRAY(hinf, Esp32SdioSlaveState,
                             ESP32_SDIO_HINF_WORDS),
        VMSTATE_UINT32_ARRAY(slc0_token, Esp32SdioSlaveState, 2),
        VMSTATE_UINT32_ARRAY(slc1_token, Esp32SdioSlaveState, 2),
        VMSTATE_UINT32(slc0_len, Esp32SdioSlaveState),
        VMSTATE_UINT32(pkt_len_latch, Esp32SdioSlaveState),
        VMSTATE_STRUCT(send, Esp32SdioSlaveState, 1, vmstate_esp32_sdio_send,
                       Esp32SdioSendLink),
        VMSTATE_STRUCT(recv, Esp32SdioSlaveState, 1, vmstate_esp32_sdio_recv,
                       Esp32SdioRecvLink),
        VMSTATE_BOOL(undoc_logged, Esp32SdioSlaveState),
        VMSTATE_UINT8(card_state, Esp32SdioSlaveState),
        VMSTATE_UINT16(rca, Esp32SdioSlaveState),
        VMSTATE_UINT8(ioe, Esp32SdioSlaveState),
        VMSTATE_UINT8(ien, Esp32SdioSlaveState),
        VMSTATE_UINT8(bus_if, Esp32SdioSlaveState),
        VMSTATE_UINT8(cap_wr, Esp32SdioSlaveState),
        VMSTATE_UINT8(speed_wr, Esp32SdioSlaveState),
        VMSTATE_UINT16_ARRAY(blksize, Esp32SdioSlaveState, 3),
        VMSTATE_BOOL(xfer_active, Esp32SdioSlaveState),
        VMSTATE_BOOL(xfer_write, Esp32SdioSlaveState),
        VMSTATE_BOOL(xfer_inc, Esp32SdioSlaveState),
        VMSTATE_BOOL(xfer_endless, Esp32SdioSlaveState),
        VMSTATE_UINT8(xfer_fn, Esp32SdioSlaveState),
        VMSTATE_UINT32(xfer_addr, Esp32SdioSlaveState),
        VMSTATE_UINT32(xfer_left, Esp32SdioSlaveState),
        VMSTATE_BOOL(card_irq, Esp32SdioSlaveState),
        VMSTATE_CLOCK(apb_clk, Esp32SdioSlaveState),
        VMSTATE_END_OF_LIST()
    }
};

static void esp32_sdio_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32_sdio_reset_hold;
    dc->realize = esp32_sdio_realize;
    dc->vmsd = &vmstate_esp32_sdio;
    device_class_set_props(dc, esp32_sdio_properties);
}

static const TypeInfo esp32_sdio_types[] = {
    /* [spec:nuos:req:emu.esp32.sdio-slave] */
    {
        .name = TYPE_ESP32_SDIO_SLAVE,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(Esp32SdioSlaveState),
        .instance_init = esp32_sdio_init,
        .class_init = esp32_sdio_class_init,
    },
    /* [spec:nuos:req:emu.esp32.sdio-slave] */
    {
        .name = TYPE_ESP32_SDIO_CARD,
        .parent = TYPE_SDMMC_COMMON,
        .class_init = card_class_init,
    },
};

DEFINE_TYPES(esp32_sdio_types)
