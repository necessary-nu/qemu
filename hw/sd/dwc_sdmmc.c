/*
 * Synopsys DesignWare SD/MMC host controller (DWC_mobile_storage), as
 * Espressif integrates it in the ESP32 and ESP32-P4 (SDHOST)
 *
 * Copyright (c) 2021 Espressif Systems (Shanghai) Co. Ltd.
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * Based on Allwinner SD Host controller emulation (allwinner-sdhost.c),
 *   Copyright (C) 2019 Niek Linnenbank <nieklinnenbank@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapter
 * 57 (SD/MMC Host Controller), and the ESP32's, chapter 27; ESP-IDF's
 * sdmmc_reg.h and sdmmc_struct.h for the P4 (v3) and the ESP32, which are
 * the authority for registers and reset values, and its sdmmc_ll.h and
 * esp_driver_sdmmc for how software drives the block.
 *
 * The controller has a bus interface unit (BIU: the registers, the data
 * FIFO, the internal DMA controller and the interrupts) and a card
 * interface unit (CIU: the command and data paths and the card clocks),
 * and up to two card slots, each a QEMU SD bus with at most one card.
 *
 * Clocks. The CIU runs at cclk_in; each card's clock is cclk_in divided
 * by 2 * CLKDIV's divider that CLKSRC selects for the card, or undivided
 * for a divider of 0, while CLKENA enables it. CLKDIV, CLKSRC and CLKENA
 * reach the CIU only with a command with UPDATE_CLOCK_REGISTERS_ONLY set.
 * Everything on the card bus is timed in card clocks: a command is 48 of
 * them (and 80 more for SEND_INITIALIZATION), the card answers after 2,
 * its response is 48 or 136; a data block is its bits over the bus width
 * (halved in DDR mode), with a start bit, 16 CRC bits and an end bit, and
 * 2 clocks of access time before it; a written block is followed by the
 * card's CRC status, 8 clocks. A card's response timeout (TMOUT's
 * RESPONSE_TIMEOUT) and data timeout (DATA_TIMEOUT) count its clock. The
 * register writes software makes, the IDMAC's descriptor and buffer
 * accesses and the FIFO's moves take no time: the bus clock is many times
 * the card clock.
 *
 * Commands. Writing CMD with START_CMD set hands the command to the CIU,
 * which takes it (clearing START_CMD) after two cclk_in cycles once its
 * clock runs, the command path is free, and, with
 * WAIT_PRVDATA_COMPLETE, the data path and the card are not busy. While
 * START_CMD is set, writes to CMD, CMDARG, BYTCNT, BLKSIZ, CLKDIV,
 * CLKSRC, CLKENA, TMOUT and CTYPE are refused with a hardware locked
 * write error (HLE). The command goes to the card in slot CARD_NUMBER;
 * an empty slot, or a card that does not answer, gives a response
 * timeout (RTO). Command done (CMDD) ends every command but a clock
 * update. With CHECK_RESPONSE_CRC, an R3 or R4 response (ACMD41, CMD1,
 * CMD5), whose CRC field is all ones, fails its CRC check (RCRC), and a
 * response of the other length than RESPONSE_LENGTH expects fails it
 * too. A card that answers CMD11 (VOLT_SWITCH set) by holding CMD and
 * DAT[3:0] low raises the voltage switch interrupt (HTO); the clock
 * update that re-enables its clock raises it again, with CMDD, once the
 * card releases DAT[3:0].
 *
 * Data. A command with DATA_EXPECTED starts the data path: a read takes
 * BYTCNT bytes (or, with BYTCNT 0, until a stop command) from the card
 * into the FIFO after the command; a write sends them from the FIFO
 * after CMDD, and not at all after a response timeout. The card's clock
 * stops while a read finds the FIFO full or a write finds it empty, and
 * if that lasts the data timeout, data starvation by host timeout (HTO)
 * is raised. A read whose card sends no data within the data timeout
 * ends with a data read timeout (DRTO); a write the card does not take
 * ends with an end bit error (EBE, the missing CRC status). Data transfer
 * over (DTO) ends every data transfer; with SEND_AUTO_STOP the CIU then
 * sends CMD12 itself, its response in RESP1, and raises auto command done
 * (ACD). A command with STOP_ABORT_CMD ends the transfer in progress.
 * With CARDTHRCTL's CARDRDTHREN, a read block starts only when the FIFO
 * has CARDTHRESHOLD bytes free; with BUSY_CLR_INT_EN a write raises the
 * busy clear interrupt (BCI) as it ends.
 *
 * The FIFO is FIFO_DEPTH words. With CTRL.DMA_ENABLE clear, software
 * moves the data through BUFFIFO (0x200): the receive data request (RXDR)
 * is raised while it holds more than RX_WMARK words, the transmit data
 * request (TXDR) while a write needs data and it holds at most TX_WMARK;
 * reading it empty or writing it full is a FIFO error (FRUN). With
 * DMA_ENABLE set the requests go to the internal DMA controller instead.
 *
 * The IDMAC (with CTRL.USE_INTERNAL_DMAC and BMOD.DE) runs a list of
 * descriptors from DBADDR, four words each (DES0 to DES3), chained
 * through DES3 or, in ring mode, the next after this one (and BMOD's
 * DSL words, where present) or DBADDR after one with END_OF_RING. It
 * starts on PLDMND or a data command; a descriptor the host owns
 * suspends it with descriptor unavailable (DU) until PLDMND. It moves
 * data as the FIFO requests it: for a read once it holds more than
 * RX_WMARK words, or at the end of the data; for a write, after CMDD,
 * while it holds at most TX_WMARK words. It closes a descriptor when
 * its buffers are done, or at the end of the data, handing it back to
 * the host (OWN clear, CES set after a card error) and raising the
 * receive (RI) or transmit (TI) interrupt unless DIC is set; after the
 * descriptor with LAST_DESCRIPTOR, or at the end of the data, it stops
 * and starts its next list at DBADDR. A bus error on any of its accesses
 * (an access the bus refuses) stops it with a fatal bus error (FBE), its
 * code 1 while transmitting and 2 while receiving, until a DMA reset or
 * BMOD.SWR. IDSTS's normal and abnormal interrupt summaries latch their
 * enabled causes.
 *
 * Interrupts. The interrupt is high while CTRL.INT_ENABLE is set and a
 * RINTSTS bit INTMASK enables is set, or IDSTS's NIS or AIS with
 * IDINTEN's NI or AI. A slot's SDIO interrupt (RINTSTS bit 16 + slot) is
 * set while its card requests one on DAT1 or its card-int-n input is
 * low. The card-detect input is debounced for DEBNCE bus clock cycles
 * into CDETECT, each change raising card detect (CD).
 *
 * CTRL's resets: CONTROLLER_RESET aborts the command and data paths and
 * clears START_CMD, FIFO_RESET empties the FIFO, DMA_RESET stops the
 * IDMAC; each bit clears after two bus clock cycles, and, for the first
 * two, two cclk_in cycles more.
 *
 * Time. Every timed event (a command taken, sent or answered, a chunk of
 * data on the bus, a timeout, a reset's end, a card detect debounced)
 * happens at its own time, however late the QEMU timer that runs it: a
 * late event runs as of its time (esp32_line_run_at(), as the Espressif
 * integrations time their lines), and what it times from there it times
 * from that. Register accesses, clock changes and the card-side inputs
 * first take the events that fell due before them, in time order, so
 * that software sees the controller as it is at the access.
 *
 * Not modelled: boot mode, CE-ATA (CCS, CCSD) and MMC CMD40's interrupt
 * response, which are logged as unimplemented; the phase and delay
 * settings, which shape signals the model does not draw; the SD signals
 * at the pads, so that a slot's card answers however the pads are set;
 * and CRC and start or end bit errors on the bus, which a QEMU card never
 * makes.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "hw/sd/dwc_sdmmc.h"
#include "hw/core/irq.h"
#include "hw/gpio/esp32_line.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "system/dma.h"

#define A_CTRL          0x000
#define A_PWREN         0x004
#define A_CLKDIV        0x008
#define A_CLKSRC        0x00c
#define A_CLKENA        0x010
#define A_TMOUT         0x014
#define A_CTYPE         0x018
#define A_BLKSIZ        0x01c
#define A_BYTCNT        0x020
#define A_INTMASK       0x024
#define A_CMDARG        0x028
#define A_CMD           0x02c
#define A_RESP0         0x030
#define A_RESP3         0x03c
#define A_MINTSTS       0x040
#define A_RINTSTS       0x044
#define A_STATUS        0x048
#define A_FIFOTH        0x04c
#define A_CDETECT       0x050
#define A_WRTPRT        0x054
#define A_TCBCNT        0x05c
#define A_TBBCNT        0x060
#define A_DEBNCE        0x064
#define A_USRID         0x068
#define A_VERID         0x06c
#define A_HCON          0x070
#define A_UHS           0x074
#define A_RST_N         0x078
#define A_BMOD          0x080
#define A_PLDMND        0x084
#define A_DBADDR        0x088
#define A_IDSTS         0x08c
#define A_IDINTEN       0x090
#define A_DSCADDR       0x094
#define A_BUFADDR       0x098
#define A_CARDTHRCTL    0x100
#define A_BACK_END_POWER 0x104
#define A_EMMCDDR       0x10c
#define A_ENSHIFT       0x110
#define A_BUFFIFO       0x200
#define A_CLK_EDGE_SEL  0x800

#define CTRL_CONTROLLER_RESET   BIT(0)
#define CTRL_FIFO_RESET         BIT(1)
#define CTRL_DMA_RESET          BIT(2)
#define CTRL_INT_ENABLE         BIT(4)
#define CTRL_DMA_ENABLE         BIT(5)
#define CTRL_READ_WAIT          BIT(6)
#define CTRL_SEND_IRQ_RESPONSE  BIT(7)
#define CTRL_ABORT_READ_DATA    BIT(8)
#define CTRL_SEND_CCSD          BIT(9)
#define CTRL_SEND_AUTO_STOP_CCSD BIT(10)
#define CTRL_OD_PULLUP          BIT(24)
#define CTRL_USE_IDMAC          BIT(25)
#define CTRL_RESETS             (CTRL_CONTROLLER_RESET | CTRL_FIFO_RESET | \
                                 CTRL_DMA_RESET)
#define CTRL_WRITABLE           0x03ff0ff7u

#define CMD_INDEX(c)            ((c) & 0x3f)
#define CMD_RESP_EXPECT         BIT(6)
#define CMD_RESP_LONG           BIT(7)
#define CMD_CHECK_CRC           BIT(8)
#define CMD_DATA_EXPECTED       BIT(9)
#define CMD_WRITE               BIT(10)
#define CMD_STREAM              BIT(11)
#define CMD_SEND_AUTO_STOP      BIT(12)
#define CMD_WAIT_PRVDATA        BIT(13)
#define CMD_STOP_ABORT          BIT(14)
#define CMD_SEND_INIT           BIT(15)
#define CMD_CARD_NUM(c)         extract32((c), 16, 5)
#define CMD_UPDATE_CLK          BIT(21)
#define CMD_READ_CEATA          BIT(22)
#define CMD_CCS_EXPECTED        BIT(23)
#define CMD_BOOT_BITS           (0xfu << 24)
#define CMD_VOLT_SWITCH         BIT(28)
#define CMD_USE_HOLD_REG        BIT(29)
#define CMD_START               BIT(31)
#define CMD_RESET               CMD_USE_HOLD_REG

#define INT_CD                  BIT(0)
#define INT_RE                  BIT(1)
#define INT_CMDD                BIT(2)
#define INT_DTO                 BIT(3)
#define INT_TXDR                BIT(4)
#define INT_RXDR                BIT(5)
#define INT_RCRC                BIT(6)
#define INT_DCRC                BIT(7)
#define INT_RTO                 BIT(8)
#define INT_DRTO                BIT(9)
#define INT_HTO                 BIT(10)
#define INT_FRUN                BIT(11)
#define INT_HLE                 BIT(12)
#define INT_SBE_BCI             BIT(13)
#define INT_ACD                 BIT(14)
#define INT_EBE                 BIT(15)
#define INT_SDIO(slot)          BIT(16 + (slot))
#define INT_ALL                 0x3ffffu
/* The card errors IDSTS.CES and a descriptor's CES sum up. */
#define INT_CARD_ERRORS         (INT_EBE | INT_RTO | INT_RCRC | INT_SBE_BCI | \
                                 INT_DRTO | INT_DCRC | INT_RE)

#define STATUS_RX_WMARK         BIT(0)
#define STATUS_TX_WMARK         BIT(1)
#define STATUS_FIFO_EMPTY       BIT(2)
#define STATUS_FIFO_FULL        BIT(3)
#define STATUS_DATA3            BIT(8)
#define STATUS_DATA_BUSY        BIT(9)
#define STATUS_DATA_MC_BUSY     BIT(10)

#define BMOD_SWR                BIT(0)
#define BMOD_FB                 BIT(1)
#define BMOD_DSL_SHIFT          2
#define BMOD_DSL_MASK           (0x1fu << BMOD_DSL_SHIFT)
#define BMOD_DE                 BIT(7)
#define BMOD_PBL_SHIFT          8

#define IDSTS_TI                BIT(0)
#define IDSTS_RI                BIT(1)
#define IDSTS_FBE               BIT(2)
#define IDSTS_DU                BIT(4)
#define IDSTS_CES               BIT(5)
#define IDSTS_NIS               BIT(8)
#define IDSTS_AIS               BIT(9)
#define IDSTS_W1C               0x337u
#define IDSTS_FBE_CODE_SHIFT    10
#define IDSTS_FSM_SHIFT         13
#define IDINTEN_WRITABLE        0x337u

#define FBE_CODE_TX             1
#define FBE_CODE_RX             2

/* IDSTS.FSM's states that the model shows */
#define FSM_IDLE                0
#define FSM_SUSPEND             1
#define FSM_RD_REQ_WAIT         4
#define FSM_WR_REQ_WAIT         5

#define DES0_DIC                BIT(1)
#define DES0_LD                 BIT(2)
#define DES0_CH                 BIT(4)
#define DES0_ER                 BIT(5)
#define DES0_CES                BIT(30)
#define DES0_OWN                BIT(31)
#define DES1_BS1(d)             extract32((d), 0, 13)
#define DES1_BS2(d)             extract32((d), 13, 13)

#define CARDTHRCTL_RD_EN        BIT(0)
#define CARDTHRCTL_BUSY_CLR_EN  BIT(1)
#define CARDTHRCTL_THRESHOLD(v) extract32((v), 16, 16)

#define CLK_EDGE_N(v)           extract32((v), 17, 4)

/* Card bus timing, in card clock cycles */
#define CIU_SYNC_CYCLES         2
#define CMD_CYCLES              48
#define INIT_CYCLES             80
#define NCR_CYCLES              2
#define RESP_SHORT_CYCLES       48
#define RESP_LONG_CYCLES        136
#define NAC_CYCLES              2
#define NWR_CYCLES              2
#define CRC_CYCLES              16
#define CRC_STATUS_CYCLES       8
#define RESET_CYCLES            2

/* The command index a short response with no CRC (R3, R4) carries */
#define RESP_INDEX_NONE         0x3f

/* What the timeout timer waits for */
enum {
    TIMEOUT_NONE,
    /* A read's next block from the card */
    TIMEOUT_DRTO,
    /* Software or the IDMAC to service a full or empty FIFO */
    TIMEOUT_HTO,
    /* HTO raised for the stall still going on */
    TIMEOUT_HTO_RAISED,
};

/* The host side of the data transfer in progress */
enum {
    XFER_NONE,
    XFER_READ,
    XFER_WRITE,
};

static void dwc_sdmmc_update(DWCSDMMCState *s);

static uint32_t dwc_fifo_cap(DWCSDMMCState *s)
{
    return s->fifo_depth * 4;
}

static uint32_t dwc_fifo_words(DWCSDMMCState *s)
{
    return DIV_ROUND_UP(s->fifo_count, 4);
}

static uint32_t dwc_fifo_free(DWCSDMMCState *s)
{
    return dwc_fifo_cap(s) - s->fifo_count;
}

static void dwc_fifo_push(DWCSDMMCState *s, const uint8_t *buf, uint32_t n)
{
    uint32_t cap = dwc_fifo_cap(s);
    uint32_t tail = (s->fifo_head + s->fifo_count) % cap;

    assert(n <= dwc_fifo_free(s));
    for (uint32_t i = 0; i < n; i++) {
        s->fifo[(tail + i) % cap] = buf[i];
    }
    s->fifo_count += n;
}

static void dwc_fifo_pop(DWCSDMMCState *s, uint8_t *buf, uint32_t n)
{
    uint32_t cap = dwc_fifo_cap(s);

    assert(n <= s->fifo_count);
    for (uint32_t i = 0; i < n; i++) {
        buf[i] = s->fifo[(s->fifo_head + i) % cap];
    }
    s->fifo_head = (s->fifo_head + n) % cap;
    s->fifo_count -= n;
}

static uint32_t dwc_rx_wmark(DWCSDMMCState *s)
{
    return extract32(s->fifoth, 16, 12);
}

static uint32_t dwc_tx_wmark(DWCSDMMCState *s)
{
    return extract32(s->fifoth, 0, 12);
}

/*
 * @cycles of a clock of @period (in Clock's units, 2^-32 ns), in ns,
 * rounded up: what they time cannot happen before they have passed.
 */
static int64_t dwc_cycles_ns(uint64_t period, uint64_t cycles)
{
    uint64_t lo, hi;

    mulu64(&lo, &hi, period, cycles);
    if (hi & MAKE_64BIT_MASK(31, 33)) {
        return INT64_MAX;
    }
    return MAX((lo >> 32 | hi << 32) + !!(uint32_t)lo, 1);
}

/*
 * The present: the time of the event being taken, or of the change being
 * made (include/hw/gpio/esp32_line.h).
 */
static int64_t dwc_now(void)
{
    return esp32_line_time();
}

static int64_t dwc_default_clock_after(DWCSDMMCState *s, Clock *clk,
                                       int64_t from, uint64_t cycles)
{
    int64_t ns = dwc_cycles_ns(clock_get(clk), cycles);

    return ns > INT64_MAX - from ? INT64_MAX : from + ns;
}

/* cclk_in's cycles in one of the CIU's clock. */
static uint64_t dwc_ciu_div(DWCSDMMCState *s)
{
    return s->clk_edge_div ? CLK_EDGE_N(s->clk_edge_sel) + 1 : 1;
}

/* Arm @t for @when, which *@due keeps for when the timer runs late. */
static void dwc_arm(QEMUTimer *t, int64_t *due, int64_t when)
{
    *due = when;
    timer_mod(t, when);
}

static void dwc_catch_up(DWCSDMMCState *s, int64_t now);

/* Take the timed event @fn at @when, its changes catching up nothing. */
static void dwc_run_event(DWCSDMMCState *s, int64_t when,
                          void (*fn)(void *opaque), void *opaque)
{
    bool was = s->in_event;

    s->in_event = true;
    esp32_line_run_at(when, fn, opaque);
    s->in_event = was;
}

static uint64_t dwc_default_cclk_period(DWCSDMMCState *s)
{
    return clock_get(s->cclk) * dwc_ciu_div(s);
}

static uint64_t dwc_cclk_period(DWCSDMMCState *s)
{
    return DWC_SDMMC_GET_CLASS(s)->cclk_period(s);
}

/*
 * When @cycles of the CIU's clock from @from have elapsed: the clock must
 * be running.
 */
static int64_t dwc_cclk_after(DWCSDMMCState *s, int64_t from,
                              uint64_t cycles)
{
    return DWC_SDMMC_GET_CLASS(s)->clock_after(s, s->cclk, from,
                                               cycles * dwc_ciu_div(s));
}

static uint64_t dwc_hclk_period(DWCSDMMCState *s)
{
    return clock_get(s->hclk);
}

/* When @cycles of the bus clock from @from have elapsed: it must run. */
static int64_t dwc_hclk_after(DWCSDMMCState *s, int64_t from,
                              uint64_t cycles)
{
    return DWC_SDMMC_GET_CLASS(s)->clock_after(s, s->hclk, from, cycles);
}

/*
 * The card clock of @slot as the CIU has it configured, in cycles of the
 * CIU's clock; 0 if stopped.
 */
static uint64_t dwc_card_div(DWCSDMMCState *s, unsigned slot)
{
    unsigned src, div;

    if (slot >= s->num_slots || !(s->ciu_clkena & BIT(slot)) ||
        !dwc_cclk_period(s)) {
        return 0;
    }
    src = extract32(s->ciu_clksrc, 2 * slot, 2);
    div = extract32(s->ciu_clkdiv, 8 * src, 8);
    return div ? 2 * div : 1;
}

/* @cycles of a card clock of @div from now. */
static int64_t dwc_card_after(DWCSDMMCState *s, uint64_t div, uint64_t cycles)
{
    return dwc_cclk_after(s, dwc_now(), cycles * div);
}

static SDBus *dwc_bus(DWCSDMMCState *s, unsigned slot)
{
    return slot < s->num_slots ? &s->sdbus[slot] : NULL;
}

static unsigned dwc_bus_width(DWCSDMMCState *s, unsigned slot)
{
    if (s->ctype & BIT(16 + slot)) {
        return 8;
    }
    return s->ctype & BIT(slot) ? 4 : 1;
}

/* Card clocks to move @bytes over @slot's data lines. */
static uint64_t dwc_data_cycles(DWCSDMMCState *s, unsigned slot,
                                uint32_t bytes)
{
    uint64_t cycles = DIV_ROUND_UP((uint64_t)bytes * 8,
                                   dwc_bus_width(s, slot));

    if (s->uhs & BIT(16 + slot)) {
        cycles = DIV_ROUND_UP(cycles, 2);
    }
    return cycles;
}

static void dwc_sdmmc_update_irq(DWCSDMMCState *s)
{
    bool level = false;

    if (s->ctrl & CTRL_INT_ENABLE) {
        level = (s->rintsts & s->intmask & INT_ALL) ||
                ((s->idsts & IDSTS_NIS) && (s->idinten & IDSTS_NIS)) ||
                ((s->idsts & IDSTS_AIS) && (s->idinten & IDSTS_AIS));
    }
    qemu_set_irq(s->irq, level);
}

/* Set IDSTS bits, latching the summaries their enables let through. */
static void dwc_idsts_set(DWCSDMMCState *s, uint32_t bits)
{
    uint32_t enabled;

    s->idsts |= bits;
    enabled = s->idsts & s->idinten;
    if (enabled & (IDSTS_TI | IDSTS_RI)) {
        s->idsts |= IDSTS_NIS;
    }
    if (enabled & (IDSTS_FBE | IDSTS_DU)) {
        s->idsts |= IDSTS_AIS;
    }
}

/* Raise RINTSTS bits; a card error also sums up into IDSTS.CES. */
static void dwc_raise(DWCSDMMCState *s, uint32_t bits)
{
    s->rintsts |= bits;
    if ((bits & INT_CARD_ERRORS) && s->data_dma &&
        (s->data_active || s->autostop_busy)) {
        s->card_error = true;
        dwc_idsts_set(s, IDSTS_CES);
    }
}

/* A slot's SDIO interrupt request: its card's on DAT1, or card-int-n. */
static void dwc_sdio_irq_update(DWCSDMMCState *s)
{
    for (unsigned slot = 0; slot < s->num_slots; slot++) {
        if ((s->card_irq & BIT(slot)) || !(s->int_n_in & BIT(slot))) {
            s->rintsts |= INT_SDIO(slot);
        }
    }
}

static bool dwc_card_busy(DWCSDMMCState *s, unsigned slot)
{
    SDBus *bus = dwc_bus(s, slot);

    return bus && sdbus_get_inserted(bus) && !(sdbus_get_dat_lines(bus) & 1);
}

/* The data path, or the card on it, is busy: a waiting command waits. */
static bool dwc_data_busy(DWCSDMMCState *s)
{
    return s->data_active || s->autostop_pending || s->autostop_busy ||
           dwc_card_busy(s, CMD_CARD_NUM(s->cmd));
}

/* The command path */

static void dwc_cmd_try_take(DWCSDMMCState *s)
{
    uint64_t period;

    if (s->cmd_phase != DWC_CMD_IDLE ||
        (s->resets_pending & CTRL_CONTROLLER_RESET)) {
        return;
    }
    if (!s->autostop_pending && !(s->cmd & CMD_START)) {
        return;
    }
    period = dwc_cclk_period(s);
    if (!period) {
        return;
    }
    if (!s->autostop_pending && !(s->cmd & CMD_UPDATE_CLK) &&
        (s->cmd & CMD_WAIT_PRVDATA) && dwc_data_busy(s)) {
        return;
    }
    s->cmd_phase = DWC_CMD_TAKING;
    dwc_arm(&s->cmd_timer, &s->cmd_due,
            dwc_cclk_after(s, dwc_now(), CIU_SYNC_CYCLES));
}

static void dwc_idmac_start(DWCSDMMCState *s);

/* Start the data path for the data command just taken. */
static void dwc_data_setup(DWCSDMMCState *s, uint32_t cmd)
{
    if (s->data_active) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: data command CMD%u while a data "
                      "transfer is in progress\n", TYPE_DWC_SDMMC,
                      CMD_INDEX(cmd));
    }
    s->data_active = true;
    s->data_write = cmd & CMD_WRITE;
    s->data_stream = cmd & CMD_STREAM;
    s->data_autostop = cmd & CMD_SEND_AUTO_STOP;
    s->data_dma = s->ctrl & CTRL_DMA_ENABLE;
    s->data_slot = CMD_CARD_NUM(cmd);
    s->data_started = false;
    s->data_blksz = s->blksiz & 0xffff;
    s->data_total = s->bytcnt;
    s->data_block_pos = 0;
    s->chunk_busy = false;
    s->card_error = false;
    s->tcbcnt = 0;
    s->tbbcnt = 0;
    s->xfer_dir = s->data_write ? XFER_WRITE : XFER_READ;
    if (s->data_stream) {
        qemu_log_mask(LOG_UNIMP, "%s: stream transfers are taken as one "
                      "block without CRC\n", TYPE_DWC_SDMMC);
    }
    if (!s->data_blksz && !s->data_stream) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: data command with BLKSIZ 0\n",
                      TYPE_DWC_SDMMC);
    }
    if (s->data_dma) {
        dwc_idmac_start(s);
    }
}

/* Whether a short response to @index is R3 or R4, with no CRC. */
static bool dwc_resp_no_crc(DWCSDMMCState *s, unsigned slot, unsigned index)
{
    return index == 1 || index == 5 ||
           (index == 41 && (s->app_cmd & BIT(slot)));
}

/* [spec:nuos:req:emu.esp32p4.sdmmc] */
/* Put command @cmd on the bus of its slot, as the CIU takes it. */
static void dwc_cmd_issue(DWCSDMMCState *s, uint32_t cmd, uint32_t arg,
                          bool autostop)
{
    unsigned slot = CMD_CARD_NUM(cmd);
    unsigned index = CMD_INDEX(cmd);
    uint64_t div = dwc_card_div(s, slot);
    SDBus *bus = dwc_bus(s, slot);
    uint64_t cycles, sent;
    uint8_t resp[16];
    SDRequest req;
    size_t rlen = 0;

    s->ciu_cmd = cmd;
    s->ciu_cmdarg = arg;
    s->autostop_busy = autostop;
    if (slot >= s->num_slots) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: CMD%u to card %u, which the "
                      "controller does not have\n", TYPE_DWC_SDMMC, index,
                      slot);
        s->cmd_phase = DWC_CMD_STALLED;
        return;
    }
    if (!div) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: CMD%u to card %u, whose clock "
                      "is stopped, goes nowhere\n", TYPE_DWC_SDMMC, index,
                      slot);
        s->cmd_phase = DWC_CMD_STALLED;
        return;
    }
    if (cmd & (CMD_READ_CEATA | CMD_CCS_EXPECTED | CMD_BOOT_BITS)) {
        qemu_log_mask(LOG_UNIMP, "%s: CE-ATA and boot operation are not "
                      "modelled\n", TYPE_DWC_SDMMC);
    }
    if (!autostop && s->data_active && slot != s->data_slot) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: CMD%u to card %u during a data "
                      "transfer with card %u\n", TYPE_DWC_SDMMC, index, slot,
                      s->data_slot);
    }

    req.cmd = index;
    req.arg = arg;
    req.crc = 0;
    if (bus) {
        rlen = sdbus_do_command(bus, &req, resp, sizeof(resp));
    }

    sent = (cmd & CMD_SEND_INIT ? INIT_CYCLES : 0) + CMD_CYCLES;
    cycles = sent;
    s->pend_rint = autostop ? INT_ACD : INT_CMDD;
    s->pend_resp_valid = false;
    if (cmd & CMD_RESP_EXPECT) {
        bool want_long = cmd & CMD_RESP_LONG;

        if (!rlen) {
            s->pend_rint |= INT_RTO;
            cycles += extract32(s->tmout, 0, 8);
        } else {
            bool no_crc = rlen == 4 && dwc_resp_no_crc(s, slot, index);

            cycles += NCR_CYCLES + (rlen == 16 ? RESP_LONG_CYCLES
                                               : RESP_SHORT_CYCLES);
            s->pend_resp_valid = true;
            memset(s->pend_resp, 0, sizeof(s->pend_resp));
            if (rlen == 16) {
                s->pend_resp[0] = ldl_be_p(&resp[12]);
                s->pend_resp[1] = ldl_be_p(&resp[8]);
                s->pend_resp[2] = ldl_be_p(&resp[4]);
                s->pend_resp[3] = ldl_be_p(&resp[0]);
                s->pend_resp_index = RESP_INDEX_NONE;
            } else {
                s->pend_resp[0] = ldl_be_p(&resp[0]);
                s->pend_resp_index = no_crc ? RESP_INDEX_NONE : index;
            }
            if ((cmd & CMD_CHECK_CRC) &&
                (no_crc || want_long != (rlen == 16))) {
                s->pend_rint |= INT_RCRC;
            }
            if (want_long != (rlen == 16)) {
                qemu_log_mask(LOG_GUEST_ERROR, "%s: CMD%u expects a %s "
                              "response; the card gives a %s one\n",
                              TYPE_DWC_SDMMC, index,
                              want_long ? "long" : "short",
                              want_long ? "short" : "long");
            }
        }
    }

    /* Which of the card's commands are application commands */
    if (bus) {
        bool app = index == 55 && rlen == 4 && (ldl_be_p(resp) & BIT(5));

        s->app_cmd = deposit32(s->app_cmd, slot, 1, app);
    }

    /* A card switching to 1.8 V holds CMD and DAT[3:0] low after CMD11. */
    if ((cmd & CMD_VOLT_SWITCH) && index == 11 && rlen && bus &&
        !(sdbus_get_dat_lines(bus) & 0xf) && !sdbus_get_cmd_line(bus)) {
        s->pend_rint |= INT_HTO;
    }

    s->cmd_sent_ns = dwc_card_after(s, div, sent);
    s->cmd_phase = DWC_CMD_BUSY;
    dwc_arm(&s->cmd_timer, &s->cmd_due, dwc_card_after(s, div, cycles));

    if (!autostop && (cmd & CMD_DATA_EXPECTED)) {
        dwc_data_setup(s, cmd);
    }
}

/* A clock update: CLKDIV, CLKSRC and CLKENA reach the CIU. */
static void dwc_cmd_update_clock(DWCSDMMCState *s, uint32_t cmd)
{
    unsigned slot = CMD_CARD_NUM(cmd);
    bool was_on = s->ciu_clkena & BIT(slot);
    SDBus *bus = dwc_bus(s, slot);

    s->ciu_clkdiv = s->clkdiv;
    s->ciu_clksrc = s->clksrc;
    s->ciu_clkena = s->clkena;
    /*
     * In a voltage switch, the card releases DAT[3:0] once its clock runs
     * again: the second voltage switch interrupt, with command done.
     */
    if ((cmd & CMD_VOLT_SWITCH) && !was_on && (s->ciu_clkena & BIT(slot)) &&
        bus && (sdbus_get_dat_lines(bus) & 0xf) == 0xf) {
        dwc_raise(s, INT_HTO | INT_CMDD);
    }
}

static void dwc_data_end(DWCSDMMCState *s, uint32_t rint, bool normal);

/* The command path's next step, at its time, dwc_now(). */
static void dwc_cmd_run(void *opaque)
{
    DWCSDMMCState *s = opaque;

    if (s->cmd_phase == DWC_CMD_TAKING) {
        s->cmd_phase = DWC_CMD_IDLE;
        if (s->autostop_pending) {
            uint32_t stop = 12 | CMD_RESP_EXPECT | CMD_CHECK_CRC |
                            CMD_STOP_ABORT | CMD_USE_HOLD_REG |
                            (s->data_slot << 16);

            s->autostop_pending = false;
            dwc_cmd_issue(s, stop, 0, true);
        } else if (s->cmd & CMD_START) {
            uint32_t cmd = s->cmd;

            s->cmd &= ~CMD_START;
            if (cmd & CMD_UPDATE_CLK) {
                dwc_cmd_update_clock(s, cmd);
            } else {
                dwc_cmd_issue(s, cmd, s->cmdarg, false);
            }
        }
    } else if (s->cmd_phase == DWC_CMD_BUSY) {
        bool autostop = s->autostop_busy;
        uint32_t cmd = s->ciu_cmd;

        s->cmd_phase = DWC_CMD_IDLE;
        if (s->pend_resp_valid) {
            if (autostop) {
                s->resp[1] = s->pend_resp[0];
            } else {
                memcpy(s->resp, s->pend_resp, sizeof(s->resp));
            }
            s->resp_index = s->pend_resp_index;
        }
        dwc_raise(s, s->pend_rint);
        s->autostop_busy = false;
        if (!autostop && (cmd & CMD_DATA_EXPECTED) && s->data_active) {
            if (s->data_write && (s->pend_rint & INT_RTO)) {
                /* No response: no data goes to the card. */
                s->data_active = false;
                s->xfer_dir = XFER_NONE;
            } else {
                s->data_started = true;
            }
        } else if (!autostop && (cmd & CMD_STOP_ABORT) && s->data_active) {
            dwc_data_end(s, 0, false);
        }
    }
    dwc_sdmmc_update(s);
}

static void dwc_cmd_timer(void *opaque)
{
    DWCSDMMCState *s = opaque;

    dwc_run_event(s, s->cmd_due, dwc_cmd_run, s);
}

/* The data path */

static void dwc_timeout_arm(DWCSDMMCState *s, int kind)
{
    uint64_t div = dwc_card_div(s, s->data_slot);

    if (s->timeout_kind == kind ||
        (kind == TIMEOUT_HTO && s->timeout_kind == TIMEOUT_HTO_RAISED)) {
        return;
    }
    s->timeout_kind = kind;
    if (div) {
        dwc_arm(&s->timeout_timer, &s->timeout_due,
                dwc_card_after(s, div, extract32(s->tmout, 8, 24)));
    } else {
        timer_del(&s->timeout_timer);
    }
}

static void dwc_timeout_cancel(DWCSDMMCState *s)
{
    s->timeout_kind = TIMEOUT_NONE;
    timer_del(&s->timeout_timer);
}

/*
 * The data path finishes, @normal when all its data has moved: data
 * transfer over, then any auto stop.
 */
static void dwc_data_end(DWCSDMMCState *s, uint32_t rint, bool normal)
{
    s->data_active = false;
    s->chunk_busy = false;
    timer_del(&s->data_timer);
    dwc_timeout_cancel(s);
    if (s->data_write && (s->cardthrctl & CARDTHRCTL_BUSY_CLR_EN)) {
        rint |= INT_SBE_BCI;
    }
    dwc_raise(s, rint | INT_DTO);
    if (normal && s->data_autostop) {
        s->autostop_pending = true;
    }
    /* A write's host side is done; a read's once the FIFO is drained. */
    if (s->xfer_dir == XFER_WRITE ||
        (s->xfer_dir == XFER_READ && !s->fifo_count)) {
        s->xfer_dir = XFER_NONE;
    }
}

static uint32_t dwc_data_remaining(DWCSDMMCState *s)
{
    return s->data_total ? s->data_total - s->tcbcnt : UINT32_MAX;
}

/* The bytes left in the current block (a stream is one long block) */
static uint32_t dwc_block_remaining(DWCSDMMCState *s)
{
    if (s->data_stream || !s->data_blksz) {
        return dwc_data_remaining(s);
    }
    return s->data_blksz - s->data_block_pos;
}

/* [spec:nuos:req:emu.esp32p4.sdmmc] */
/* Start the next chunk on the card bus, or wait for what it needs. */
static void dwc_data_kick(DWCSDMMCState *s)
{
    uint32_t n, room, block_left;
    uint64_t div, cycles;
    SDBus *bus = dwc_bus(s, s->data_slot);
    bool block_start, block_end, stalled;

    if (!s->data_active || !s->data_started || s->chunk_busy) {
        return;
    }
    if (s->data_total && s->tcbcnt >= s->data_total) {
        dwc_data_end(s, 0, true);
        return;
    }
    div = dwc_card_div(s, s->data_slot);
    if (!div) {
        return;
    }
    block_left = dwc_block_remaining(s);
    n = MIN(MIN(block_left, dwc_data_remaining(s)), sizeof(s->chunk));
    block_start = s->data_block_pos == 0;

    if (!s->data_write) {
        if (block_start && !(bus && sdbus_data_ready(bus))) {
            dwc_timeout_arm(s, TIMEOUT_DRTO);
            return;
        }
        room = dwc_fifo_free(s);
        stalled = room < MIN(n, 4) ||
                  (block_start && (s->cardthrctl & CARDTHRCTL_RD_EN) &&
                   room < CARDTHRCTL_THRESHOLD(s->cardthrctl));
        if (stalled) {
            dwc_timeout_arm(s, TIMEOUT_HTO);
            return;
        }
        n = MIN(n, room);
        if (n > 4 && n < block_left) {
            n &= ~3;
        }
        if (!block_start && !sdbus_data_ready(bus)) {
            /* The card stopped mid-block: the host samples idle lines. */
            memset(s->chunk, 0xff, n);
            s->card_error = true;
            dwc_raise(s, INT_DCRC);
        } else {
            sdbus_read_data(bus, s->chunk, n);
        }
    } else {
        if (!s->fifo_count) {
            dwc_timeout_arm(s, TIMEOUT_HTO);
            return;
        }
        n = MIN(n, s->fifo_count);
        dwc_fifo_pop(s, s->chunk, n);
    }
    dwc_timeout_cancel(s);

    block_end = n == block_left;
    cycles = dwc_data_cycles(s, s->data_slot, n);
    if (block_start) {
        cycles += (s->data_write ? NWR_CYCLES : NAC_CYCLES) + 1;
    }
    if (block_end && !s->data_stream) {
        cycles += CRC_CYCLES + 1 + (s->data_write ? CRC_STATUS_CYCLES : 0);
    }
    s->chunk_len = n;
    s->chunk_busy = true;
    dwc_arm(&s->data_timer, &s->data_due, dwc_card_after(s, div, cycles));
}

/* The chunk on the bus has moved, at its time, dwc_now(). */
static void dwc_data_run(void *opaque)
{
    DWCSDMMCState *s = opaque;
    SDBus *bus = dwc_bus(s, s->data_slot);
    uint32_t n = s->chunk_len;

    if (!s->chunk_busy) {
        return;
    }
    s->chunk_busy = false;
    if (!s->data_write) {
        dwc_fifo_push(s, s->chunk, n);
    } else {
        if (s->data_block_pos == 0 && !(bus && sdbus_receive_ready(bus))) {
            /* The card takes no data, and sends no CRC status. */
            s->tcbcnt += n;
            dwc_data_end(s, INT_EBE, false);
            dwc_sdmmc_update(s);
            return;
        }
        sdbus_write_data(bus, s->chunk, n);
    }
    s->tcbcnt += n;
    s->data_block_pos += n;
    if (!s->data_stream && s->data_blksz &&
        s->data_block_pos >= s->data_blksz) {
        s->data_block_pos = 0;
    }
    if (s->data_total && s->tcbcnt >= s->data_total) {
        dwc_data_end(s, 0, true);
    }
    dwc_sdmmc_update(s);
}

static void dwc_data_timer(void *opaque)
{
    DWCSDMMCState *s = opaque;

    dwc_run_event(s, s->data_due, dwc_data_run, s);
}

/* The data timeout, at its time, dwc_now(). */
static void dwc_timeout_run(void *opaque)
{
    DWCSDMMCState *s = opaque;
    int kind = s->timeout_kind;

    s->timeout_kind = TIMEOUT_NONE;
    if (!s->data_active) {
        return;
    }
    if (kind == TIMEOUT_DRTO) {
        dwc_data_end(s, INT_DRTO, false);
    } else if (kind == TIMEOUT_HTO) {
        /* Once per stall: the FIFO's next move ends it. */
        s->timeout_kind = TIMEOUT_HTO_RAISED;
        dwc_raise(s, INT_HTO);
    }
    dwc_sdmmc_update(s);
}

static void dwc_timeout_timer(void *opaque)
{
    DWCSDMMCState *s = opaque;

    dwc_run_event(s, s->timeout_due, dwc_timeout_run, s);
}

/* The IDMAC */

static bool dwc_idmac_enabled(DWCSDMMCState *s)
{
    return (s->ctrl & CTRL_USE_IDMAC) && (s->bmod & BMOD_DE) &&
           dwc_hclk_period(s);
}

static void dwc_idmac_fatal(DWCSDMMCState *s, unsigned code)
{
    qemu_log_mask(LOG_GUEST_ERROR, "%s: IDMAC bus error\n", TYPE_DWC_SDMMC);
    s->idsts = deposit32(s->idsts, IDSTS_FBE_CODE_SHIFT, 3, code);
    dwc_idsts_set(s, IDSTS_FBE);
    s->idmac_state = DWC_IDMAC_FATAL;
}

static unsigned dwc_idmac_code(DWCSDMMCState *s)
{
    return s->xfer_dir == XFER_READ ? FBE_CODE_RX : FBE_CODE_TX;
}

static void dwc_idmac_fetch(DWCSDMMCState *s)
{
    uint32_t d[4];

    if (address_space_read(s->dma_as, s->dscaddr, s->dma_attrs, d,
                           sizeof(d)) != MEMTX_OK) {
        dwc_idmac_fatal(s, dwc_idmac_code(s));
        return;
    }
    for (int i = 0; i < 4; i++) {
        s->idmac_desc[i] = le32_to_cpu(d[i]);
    }
    if (!(s->idmac_desc[0] & DES0_OWN)) {
        s->idmac_state = DWC_IDMAC_SUSPEND;
        dwc_idsts_set(s, IDSTS_DU);
        return;
    }
    s->idmac_state = DWC_IDMAC_ACTIVE;
    s->idmac_buf = 0;
    s->idmac_buf_off = 0;
    s->bufaddr = s->idmac_desc[2];
}

/* Start the list from where the IDMAC's next one begins. */
static void dwc_idmac_start(DWCSDMMCState *s)
{
    if (s->idmac_state != DWC_IDMAC_IDLE || !dwc_idmac_enabled(s)) {
        return;
    }
    s->dscaddr = s->idmac_next;
    dwc_idmac_fetch(s);
}

/* Hand the descriptor back to the host and go on to the next, if any. */
static void dwc_idmac_close(DWCSDMMCState *s, bool end_of_data)
{
    uint32_t des0 = s->idmac_desc[0] & ~DES0_OWN;
    uint32_t le;

    if (s->card_error) {
        des0 |= DES0_CES;
    }
    le = cpu_to_le32(des0);
    if (address_space_write(s->dma_as, s->dscaddr, s->dma_attrs, &le,
                            sizeof(le)) != MEMTX_OK) {
        dwc_idmac_fatal(s, dwc_idmac_code(s));
        return;
    }
    if (!(des0 & DES0_DIC)) {
        dwc_idsts_set(s, s->xfer_dir == XFER_READ ? IDSTS_RI : IDSTS_TI);
    }
    if (end_of_data || (des0 & DES0_LD)) {
        s->idmac_state = DWC_IDMAC_IDLE;
        s->idmac_next = s->dbaddr;
        return;
    }
    if (des0 & DES0_CH) {
        s->dscaddr = s->idmac_desc[3];
    } else if (des0 & DES0_ER) {
        s->dscaddr = s->dbaddr;
    } else {
        uint32_t skip = s->has_dsl ?
            extract32(s->bmod, BMOD_DSL_SHIFT, 5) * 4 : 0;

        s->dscaddr += 16 + skip;
    }
    dwc_idmac_fetch(s);
}

/* [spec:nuos:req:emu.esp32p4.sdmmc] */
/* One step of the IDMAC's work; false when it has none to do. */
static bool dwc_idmac_step(DWCSDMMCState *s)
{
    uint32_t des0 = s->idmac_desc[0];
    uint32_t des1 = s->idmac_desc[1];
    bool chained = des0 & DES0_CH;
    uint32_t size = s->idmac_buf == 0 ? DES1_BS1(des1)
                                      : (chained ? 0 : DES1_BS2(des1));
    uint32_t addr = s->idmac_desc[2 + s->idmac_buf] & ~3u;
    uint8_t buf[512];
    uint32_t n;

    if (!s->data_dma || s->xfer_dir == XFER_NONE) {
        return false;
    }
    if (s->idmac_buf_off >= size) {
        if (s->idmac_buf == 0 && !chained) {
            s->idmac_buf = 1;
            s->idmac_buf_off = 0;
        } else {
            dwc_idmac_close(s, false);
        }
        return true;
    }

    if (s->xfer_dir == XFER_READ) {
        if (!s->fifo_count ||
            (s->data_active && dwc_fifo_words(s) <= dwc_rx_wmark(s))) {
            return false;
        }
        n = MIN(MIN(s->fifo_count, size - s->idmac_buf_off), sizeof(buf));
        dwc_fifo_pop(s, buf, n);
        if (address_space_write(s->dma_as, addr + s->idmac_buf_off,
                                s->dma_attrs, buf, n) != MEMTX_OK) {
            dwc_idmac_fatal(s, FBE_CODE_RX);
            return false;
        }
        s->idmac_buf_off += n;
        s->tbbcnt += n;
        s->bufaddr = addr + s->idmac_buf_off;
        if ((s->data_total && s->tbbcnt >= s->data_total) ||
            (!s->data_active && !s->fifo_count)) {
            dwc_idmac_close(s, true);
            s->xfer_dir = XFER_NONE;
        }
        return true;
    }

    /* A write's data goes to the FIFO only once its command is done. */
    if (!s->data_started || dwc_fifo_words(s) > dwc_tx_wmark(s)) {
        return false;
    }
    n = s->data_total ? s->data_total - s->tbbcnt : UINT32_MAX;
    n = MIN(MIN(n, size - s->idmac_buf_off), dwc_fifo_free(s));
    n = MIN(n, sizeof(buf));
    if (!n) {
        return false;
    }
    if (address_space_read(s->dma_as, addr + s->idmac_buf_off, s->dma_attrs,
                           buf, n) != MEMTX_OK) {
        dwc_idmac_fatal(s, FBE_CODE_TX);
        return false;
    }
    dwc_fifo_push(s, buf, n);
    s->idmac_buf_off += n;
    s->tbbcnt += n;
    s->bufaddr = addr + s->idmac_buf_off;
    if (s->data_total && s->tbbcnt >= s->data_total) {
        dwc_idmac_close(s, true);
    }
    return true;
}

static void dwc_idmac_run(DWCSDMMCState *s)
{
    while (dwc_idmac_enabled(s) && s->idmac_state == DWC_IDMAC_ACTIVE &&
           dwc_idmac_step(s)) {
        /* Each step moves a buffer's worth or closes a descriptor. */
    }
}

/* Software's side of the FIFO, with CTRL.DMA_ENABLE clear */
static void dwc_fifo_requests(DWCSDMMCState *s)
{
    if (s->data_dma) {
        return;
    }
    if (s->xfer_dir == XFER_READ && dwc_fifo_words(s) > dwc_rx_wmark(s)) {
        s->rintsts |= INT_RXDR;
    }
    if (s->xfer_dir == XFER_WRITE && s->data_active &&
        dwc_fifo_words(s) <= dwc_tx_wmark(s) &&
        (!s->data_total || s->tbbcnt < s->data_total)) {
        s->rintsts |= INT_TXDR;
    }
}

/*
 * Bring every path up to date after a change: the command waiting for
 * the CIU, the data path, the IDMAC and the FIFO requests, until none of
 * them moves on.
 */
static void dwc_sdmmc_update(DWCSDMMCState *s)
{
    uint32_t snap[6];

    do {
        snap[0] = s->fifo_count;
        snap[1] = s->tbbcnt;
        snap[2] = s->idmac_state;
        snap[3] = s->chunk_busy;
        snap[4] = s->data_active;
        snap[5] = s->dscaddr;
        dwc_idmac_run(s);
        dwc_data_kick(s);
        if (s->xfer_dir == XFER_READ && !s->data_active &&
            !s->fifo_count) {
            s->xfer_dir = XFER_NONE;
        }
    } while (snap[0] != s->fifo_count || snap[1] != s->tbbcnt ||
             snap[2] != s->idmac_state || snap[3] != s->chunk_busy ||
             snap[4] != s->data_active || snap[5] != s->dscaddr);
    dwc_fifo_requests(s);
    dwc_cmd_try_take(s);
    dwc_sdio_irq_update(s);
    dwc_sdmmc_update_irq(s);
}

/* CTRL's self-clearing resets */

static void dwc_resets_schedule(DWCSDMMCState *s)
{
    uint64_t hclk = dwc_hclk_period(s), cclk = dwc_cclk_period(s);
    int64_t when;

    if (!s->resets_pending || !hclk ||
        (!(s->resets_pending & CTRL_DMA_RESET) && !cclk)) {
        timer_del(&s->reset_timer);
        return;
    }
    when = dwc_hclk_after(s, dwc_now(), RESET_CYCLES);
    if ((s->resets_pending & (CTRL_CONTROLLER_RESET | CTRL_FIFO_RESET)) &&
        cclk) {
        when = dwc_cclk_after(s, when, RESET_CYCLES);
    }
    dwc_arm(&s->reset_timer, &s->reset_due, when);
}

/*
 * The bus clock has run two cycles: DMA_RESET clears; the other two once
 * cclk_in has run two cycles as well, and wait for it while it is stopped.
 */
static void dwc_reset_run(void *opaque)
{
    DWCSDMMCState *s = opaque;
    uint32_t done = CTRL_DMA_RESET;

    if (dwc_cclk_period(s)) {
        done |= CTRL_CONTROLLER_RESET | CTRL_FIFO_RESET;
    }
    s->resets_pending &= ~done;
    s->ctrl &= ~done;
    if (s->resets_pending) {
        dwc_resets_schedule(s);
    }
    dwc_sdmmc_update(s);
}

static void dwc_reset_timer(void *opaque)
{
    DWCSDMMCState *s = opaque;

    dwc_run_event(s, s->reset_due, dwc_reset_run, s);
}

static void dwc_controller_reset(DWCSDMMCState *s)
{
    timer_del(&s->cmd_timer);
    timer_del(&s->data_timer);
    dwc_timeout_cancel(s);
    s->cmd_phase = DWC_CMD_IDLE;
    s->cmd &= ~CMD_START;
    s->autostop_pending = false;
    s->autostop_busy = false;
    s->data_active = false;
    s->chunk_busy = false;
    s->xfer_dir = XFER_NONE;
    s->ctrl &= ~(CTRL_READ_WAIT | CTRL_SEND_IRQ_RESPONSE |
                 CTRL_ABORT_READ_DATA);
}

static void dwc_fifo_reset(DWCSDMMCState *s)
{
    s->fifo_head = 0;
    s->fifo_count = 0;
}

static void dwc_idmac_reset(DWCSDMMCState *s)
{
    s->idmac_state = DWC_IDMAC_IDLE;
    s->idmac_next = s->dbaddr;
}

static void dwc_write_ctrl(DWCSDMMCState *s, uint32_t value)
{
    uint32_t resets = value & CTRL_RESETS;
    uint32_t old = s->ctrl;

    s->ctrl = (value & CTRL_WRITABLE) | (s->ctrl & CTRL_RESETS);
    if (resets & CTRL_CONTROLLER_RESET) {
        dwc_controller_reset(s);
    }
    if (resets & CTRL_FIFO_RESET) {
        dwc_fifo_reset(s);
    }
    if (resets & CTRL_DMA_RESET) {
        dwc_idmac_reset(s);
    }
    s->resets_pending |= resets;
    s->ctrl |= resets;
    dwc_resets_schedule(s);

    if (s->ctrl & (CTRL_SEND_CCSD | CTRL_SEND_AUTO_STOP_CCSD)) {
        qemu_log_mask(LOG_UNIMP, "%s: CE-ATA command completion signal "
                      "disable is not modelled\n", TYPE_DWC_SDMMC);
        s->ctrl &= ~(CTRL_SEND_CCSD | CTRL_SEND_AUTO_STOP_CCSD);
    }
    if (s->ctrl & CTRL_SEND_IRQ_RESPONSE) {
        qemu_log_mask(LOG_UNIMP, "%s: the MMC interrupt (CMD40) response "
                      "is not modelled\n", TYPE_DWC_SDMMC);
        s->ctrl &= ~CTRL_SEND_IRQ_RESPONSE;
    }
    if (s->ctrl & CTRL_ABORT_READ_DATA) {
        /* The read data state machine goes back to idle. */
        if (s->data_active && !s->data_write) {
            s->data_active = false;
            s->chunk_busy = false;
            timer_del(&s->data_timer);
            dwc_timeout_cancel(s);
        }
        s->ctrl &= ~CTRL_ABORT_READ_DATA;
    }
    if ((old ^ s->ctrl) & CTRL_OD_PULLUP) {
        qemu_set_irq(s->od_pullup_en_n, !(s->ctrl & CTRL_OD_PULLUP));
    }
    dwc_sdmmc_update(s);
}

static void dwc_write_bmod(DWCSDMMCState *s, uint32_t value)
{
    uint32_t writable = BMOD_FB | BMOD_DE |
                        (s->has_dsl ? BMOD_DSL_MASK : 0);

    s->bmod = value & writable;
    if (value & BMOD_SWR) {
        /* The IDMAC's own registers reset; SWR clears a cycle later. */
        dwc_idmac_reset(s);
        s->idsts = 0;
        s->dscaddr = 0;
        s->bufaddr = 0;
    }
    dwc_sdmmc_update(s);
}

/* Registers */

static bool dwc_cmd_locked(DWCSDMMCState *s, hwaddr offset)
{
    if (!(s->cmd & CMD_START)) {
        return false;
    }
    qemu_log_mask(LOG_GUEST_ERROR, "%s: write to 0x%03" HWADDR_PRIx " while "
                  "CMD.START_CMD is set: hardware locked write error\n",
                  TYPE_DWC_SDMMC, offset);
    s->rintsts |= INT_HLE;
    dwc_sdmmc_update_irq(s);
    return true;
}

static uint32_t dwc_status(DWCSDMMCState *s)
{
    unsigned slot = CMD_CARD_NUM(s->cmd);
    SDBus *bus = dwc_bus(s, slot);
    uint32_t words = dwc_fifo_words(s);
    uint32_t st = 0;
    unsigned fsm = 0;

    if (words > dwc_rx_wmark(s)) {
        st |= STATUS_RX_WMARK;
    }
    if (words <= dwc_tx_wmark(s)) {
        st |= STATUS_TX_WMARK;
    }
    if (!words) {
        st |= STATUS_FIFO_EMPTY;
    }
    if (words >= s->fifo_depth) {
        st |= STATUS_FIFO_FULL;
    }
    st = deposit32(st, 11, 6, s->resp_index);
    st = deposit32(st, 17, 13, words);

    if (!dwc_cclk_period(s)) {
        /* Without cclk_in the CIU's state holds its reset value. */
        return st | (1 << 4) | STATUS_DATA3 | STATUS_DATA_BUSY |
               STATUS_DATA_MC_BUSY;
    }
    if (s->cmd_phase == DWC_CMD_BUSY) {
        if (dwc_now() < s->cmd_sent_ns) {
            fsm = 4;                /* Send cmd index + arg */
        } else if (s->ciu_cmd & CMD_RESP_EXPECT) {
            fsm = 11;               /* Receive resp data */
        } else {
            fsm = 14;               /* Cmd path wait NCC */
        }
    } else if (s->cmd_phase == DWC_CMD_STALLED) {
        fsm = 2;                    /* Send cmd start bit */
    }
    st = deposit32(st, 4, 4, fsm);
    if (bus && sdbus_get_inserted(bus) && (sdbus_get_dat_lines(bus) & 8)) {
        st |= STATUS_DATA3;
    }
    if (dwc_card_busy(s, slot)) {
        st |= STATUS_DATA_BUSY;
    }
    if (s->data_active || s->chunk_busy) {
        st |= STATUS_DATA_MC_BUSY;
    }
    return st;
}

static uint32_t dwc_buffifo_read(DWCSDMMCState *s)
{
    uint8_t b[4] = { 0 };
    uint32_t n = MIN(s->fifo_count, 4);

    if (!n) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: BUFFIFO read with the FIFO "
                      "empty: FIFO underrun\n", TYPE_DWC_SDMMC);
        s->rintsts |= INT_FRUN;
        dwc_sdmmc_update(s);
        return 0;
    }
    dwc_fifo_pop(s, b, n);
    if (s->xfer_dir != XFER_NONE) {
        s->tbbcnt += n;
    }
    dwc_sdmmc_update(s);
    return ldl_le_p(b);
}

static void dwc_buffifo_write(DWCSDMMCState *s, uint32_t value)
{
    uint8_t b[4];
    uint32_t n = 4;

    if (dwc_fifo_free(s) < 4) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: BUFFIFO write with the FIFO "
                      "full: FIFO overrun\n", TYPE_DWC_SDMMC);
        s->rintsts |= INT_FRUN;
        dwc_sdmmc_update(s);
        return;
    }
    if (s->xfer_dir == XFER_WRITE && s->data_total) {
        n = MIN(n, s->data_total - MIN(s->tbbcnt, s->data_total));
        n = n ? n : 4;
    }
    stl_le_p(b, value);
    dwc_fifo_push(s, b, n);
    if (s->xfer_dir != XFER_NONE) {
        s->tbbcnt += n;
    }
    dwc_sdmmc_update(s);
}

static bool dwc_reg_read(DWCSDMMCState *s, hwaddr offset, uint32_t *v)
{
    switch (offset) {
    case A_CTRL:
        *v = s->ctrl;
        break;
    case A_PWREN:
        if (!s->has_power_regs) {
            return false;
        }
        *v = s->pwren;
        break;
    case A_CLKDIV:
        *v = s->clkdiv;
        break;
    case A_CLKSRC:
        *v = s->clksrc;
        break;
    case A_CLKENA:
        *v = s->clkena;
        break;
    case A_TMOUT:
        *v = s->tmout;
        break;
    case A_CTYPE:
        *v = s->ctype;
        break;
    case A_BLKSIZ:
        *v = s->blksiz;
        break;
    case A_BYTCNT:
        *v = s->bytcnt;
        break;
    case A_INTMASK:
        *v = s->intmask;
        break;
    case A_CMDARG:
        *v = s->cmdarg;
        break;
    case A_CMD:
        *v = s->cmd;
        break;
    case A_RESP0 ... A_RESP3:
        *v = s->resp[(offset - A_RESP0) / 4];
        break;
    case A_MINTSTS:
        *v = s->rintsts & s->intmask & INT_ALL;
        break;
    case A_RINTSTS:
        *v = s->rintsts;
        break;
    case A_STATUS:
        *v = dwc_status(s);
        break;
    case A_FIFOTH:
        *v = s->fifoth;
        break;
    case A_CDETECT:
        *v = s->cd_n & MAKE_64BIT_MASK(0, s->num_slots);
        break;
    case A_WRTPRT:
        *v = s->wp_in & MAKE_64BIT_MASK(0, s->num_slots);
        break;
    case A_TCBCNT:
        *v = s->tcbcnt;
        break;
    case A_TBBCNT:
        *v = s->tbbcnt;
        break;
    case A_DEBNCE:
        *v = s->debnce;
        break;
    case A_USRID:
        *v = s->usrid;
        break;
    case A_VERID:
        *v = s->verid;
        break;
    case A_HCON:
        *v = s->hcon;
        break;
    case A_UHS:
        *v = s->uhs;
        break;
    case A_RST_N:
        *v = s->rst_n;
        break;
    case A_BMOD:
        /* PBL mirrors FIFOTH's DMA_MULTIPLE_TRANSACTION_SIZE. */
        *v = s->bmod | (extract32(s->fifoth, 28, 3) << BMOD_PBL_SHIFT);
        break;
    case A_PLDMND:
        *v = 0;
        break;
    case A_DBADDR:
        *v = s->dbaddr;
        break;
    case A_IDSTS: {
        unsigned fsm = FSM_IDLE;

        if (s->idmac_state == DWC_IDMAC_SUSPEND) {
            fsm = FSM_SUSPEND;
        } else if (s->idmac_state == DWC_IDMAC_ACTIVE) {
            fsm = s->xfer_dir == XFER_READ ? FSM_WR_REQ_WAIT
                                           : FSM_RD_REQ_WAIT;
        }
        *v = deposit32(s->idsts, IDSTS_FSM_SHIFT, 4, fsm);
        break;
    }
    case A_IDINTEN:
        *v = s->idinten;
        break;
    case A_DSCADDR:
        *v = s->dscaddr;
        break;
    case A_BUFADDR:
        *v = s->bufaddr;
        break;
    case A_CARDTHRCTL:
        *v = s->cardthrctl;
        break;
    case A_BACK_END_POWER:
        if (!s->has_power_regs) {
            return false;
        }
        *v = s->back_end_power;
        break;
    case A_EMMCDDR:
        *v = s->emmcddr;
        break;
    case A_ENSHIFT:
        *v = s->enshift;
        break;
    case A_CLK_EDGE_SEL:
        if (!s->clk_edge_mask) {
            return false;
        }
        *v = s->clk_edge_sel;
        break;
    default:
        return false;
    }
    return true;
}

/* [spec:nuos:req:emu.esp32p4.sdmmc] */
static uint64_t dwc_sdmmc_read(void *opaque, hwaddr offset, unsigned size)
{
    DWCSDMMCState *s = DWC_SDMMC(opaque);
    DWCSDMMCClass *dc = DWC_SDMMC_GET_CLASS(s);
    hwaddr reg = offset & ~3;
    uint32_t v;

    dwc_catch_up(s, dwc_now());
    if (reg == A_BUFFIFO) {
        if (size != 4) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: %u-byte access to BUFFIFO\n",
                          TYPE_DWC_SDMMC, size);
            return 0;
        }
        return dwc_buffifo_read(s);
    }
    if (!dwc_reg_read(s, reg, &v) &&
        !(reg >= A_CLK_EDGE_SEL && dc->vendor_read &&
          dc->vendor_read(s, reg, &v))) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read at 0x%03" HWADDR_PRIx
                      ", which has no register\n", TYPE_DWC_SDMMC, offset);
        return 0;
    }
    return extract32(v, (offset & 3) * 8, size * 8);
}

/* Write @value's bits under @mask to register @offset. */
static bool dwc_reg_write(DWCSDMMCState *s, hwaddr offset, uint32_t value,
                          uint32_t mask)
{
    uint32_t old;

#define MERGE(reg, writable) \
    ((((reg) & ~(mask)) | ((value) & (mask))) & (writable))

    switch (offset) {
    case A_CTRL:
        dwc_write_ctrl(s, MERGE(s->ctrl & ~CTRL_RESETS, 0xffffffffu));
        break;
    case A_PWREN:
        if (!s->has_power_regs) {
            return false;
        }
        s->pwren = MERGE(s->pwren, MAKE_64BIT_MASK(0, s->num_slots));
        break;
    case A_CLKDIV:
        if (!dwc_cmd_locked(s, offset)) {
            s->clkdiv = MERGE(s->clkdiv, 0xffffffffu);
        }
        break;
    case A_CLKSRC:
        if (!dwc_cmd_locked(s, offset)) {
            s->clksrc = MERGE(s->clksrc, 0xf);
        }
        break;
    case A_CLKENA:
        if (!dwc_cmd_locked(s, offset)) {
            s->clkena = MERGE(s->clkena, 0x00030003);
        }
        break;
    case A_TMOUT:
        if (!dwc_cmd_locked(s, offset)) {
            s->tmout = MERGE(s->tmout, 0xffffffffu);
        }
        break;
    case A_CTYPE:
        if (!dwc_cmd_locked(s, offset)) {
            s->ctype = MERGE(s->ctype, 0x00030003);
        }
        break;
    case A_BLKSIZ:
        if (!dwc_cmd_locked(s, offset)) {
            s->blksiz = MERGE(s->blksiz, 0xffff);
        }
        break;
    case A_BYTCNT:
        if (!dwc_cmd_locked(s, offset)) {
            s->bytcnt = MERGE(s->bytcnt, 0xffffffffu);
        }
        break;
    case A_INTMASK:
        s->intmask = MERGE(s->intmask, INT_ALL);
        dwc_sdmmc_update_irq(s);
        break;
    case A_CMDARG:
        if (!dwc_cmd_locked(s, offset)) {
            s->cmdarg = MERGE(s->cmdarg, 0xffffffffu);
        }
        break;
    case A_CMD:
        if (!dwc_cmd_locked(s, offset)) {
            s->cmd = MERGE(s->cmd, s->cmd_mask);
            dwc_sdmmc_update(s);
        }
        break;
    case A_RINTSTS:
        s->rintsts &= ~(value & mask & INT_ALL);
        /* A slot's SDIO request still held sets its bit again. */
        dwc_sdio_irq_update(s);
        dwc_sdmmc_update_irq(s);
        break;
    case A_FIFOTH:
        s->fifoth = MERGE(s->fifoth, s->fifoth_mask);
        dwc_sdmmc_update(s);
        break;
    case A_DEBNCE:
        s->debnce = MERGE(s->debnce, 0xffffff);
        break;
    case A_USRID:
        s->usrid = MERGE(s->usrid, 0xffffffffu);
        break;
    case A_UHS:
        s->uhs = MERGE(s->uhs, 0x00030003);
        break;
    case A_RST_N:
        old = s->rst_n;
        s->rst_n = MERGE(s->rst_n, MAKE_64BIT_MASK(0, s->num_slots));
        for (unsigned slot = 0; slot < s->num_slots; slot++) {
            if ((old ^ s->rst_n) & BIT(slot)) {
                qemu_set_irq(s->card_rst_n[slot], s->rst_n & BIT(slot));
            }
        }
        break;
    case A_BMOD:
        dwc_write_bmod(s, MERGE(s->bmod, 0xffffffffu));
        break;
    case A_PLDMND:
        if (s->idmac_state == DWC_IDMAC_SUSPEND && dwc_idmac_enabled(s)) {
            dwc_idmac_fetch(s);
        } else {
            dwc_idmac_start(s);
        }
        dwc_sdmmc_update(s);
        break;
    case A_DBADDR:
        /* Bits 1:0 are taken as zero. */
        s->dbaddr = MERGE(s->dbaddr, ~3u);
        if (s->idmac_state == DWC_IDMAC_IDLE) {
            s->idmac_next = s->dbaddr;
        }
        break;
    case A_IDSTS:
        s->idsts &= ~(value & mask & IDSTS_W1C);
        dwc_sdmmc_update_irq(s);
        break;
    case A_IDINTEN:
        s->idinten = MERGE(s->idinten, IDINTEN_WRITABLE);
        dwc_idsts_set(s, 0);
        dwc_sdmmc_update_irq(s);
        break;
    case A_CARDTHRCTL:
        s->cardthrctl = MERGE(s->cardthrctl, 0xffff0007u);
        dwc_sdmmc_update(s);
        break;
    case A_BACK_END_POWER:
        if (!s->has_power_regs) {
            return false;
        }
        s->back_end_power = MERGE(s->back_end_power,
                                  MAKE_64BIT_MASK(0, s->num_slots));
        break;
    case A_EMMCDDR:
        s->emmcddr = MERGE(s->emmcddr, 0x80000003u);
        break;
    case A_ENSHIFT:
        s->enshift = MERGE(s->enshift, 0xf);
        break;
    case A_CLK_EDGE_SEL:
        if (!s->clk_edge_mask) {
            return false;
        }
        s->clk_edge_sel = MERGE(s->clk_edge_sel, s->clk_edge_mask);
        dwc_resets_schedule(s);
        dwc_sdmmc_update(s);
        break;
    case A_RESP0 ... A_RESP3:
    case A_MINTSTS:
    case A_STATUS:
    case A_CDETECT:
    case A_WRTPRT:
    case A_TCBCNT:
    case A_TBBCNT:
    case A_VERID:
    case A_HCON:
    case A_DSCADDR:
    case A_BUFADDR:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only register "
                      "0x%03" HWADDR_PRIx "\n", TYPE_DWC_SDMMC, offset);
        break;
    default:
        return false;
    }
#undef MERGE
    return true;
}

/* [spec:nuos:req:emu.esp32p4.sdmmc] */
static void dwc_sdmmc_write(void *opaque, hwaddr offset, uint64_t value,
                            unsigned size)
{
    DWCSDMMCState *s = DWC_SDMMC(opaque);
    DWCSDMMCClass *dc = DWC_SDMMC_GET_CLASS(s);
    hwaddr reg = offset & ~3;
    unsigned shift = (offset & 3) * 8;
    uint32_t mask = MAKE_64BIT_MASK(shift, size * 8);

    dwc_catch_up(s, dwc_now());
    if (reg == A_BUFFIFO) {
        if (size != 4) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: %u-byte access to BUFFIFO\n",
                          TYPE_DWC_SDMMC, size);
            return;
        }
        dwc_buffifo_write(s, value);
        return;
    }
    if (!dwc_reg_write(s, reg, value << shift, mask) &&
        !(reg >= A_CLK_EDGE_SEL && dc->vendor_write &&
          dc->vendor_write(s, reg, value << shift, mask))) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write at 0x%03" HWADDR_PRIx
                      ", which has no register\n", TYPE_DWC_SDMMC, offset);
    }
}

static const MemoryRegionOps dwc_sdmmc_ops = {
    .read = dwc_sdmmc_read,
    .write = dwc_sdmmc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

/* The card-side inputs */

static void dwc_debounce_arm(DWCSDMMCState *s, unsigned slot)
{
    uint64_t hclk = dwc_hclk_period(s);

    if (!((s->cd_n_in ^ s->cd_n) & BIT(slot))) {
        timer_del(&s->debounce_timer[slot]);
        return;
    }
    if (hclk) {
        dwc_arm(&s->debounce_timer[slot], &s->debounce_due[slot],
                dwc_hclk_after(s, dwc_now(), extract32(s->debnce, 0, 24)));
    }
}

static void dwc_debounce_run(DWCSDMMCState *s, unsigned slot)
{
    if ((s->cd_n_in ^ s->cd_n) & BIT(slot)) {
        s->cd_n ^= BIT(slot);
        s->rintsts |= INT_CD;
        dwc_sdmmc_update_irq(s);
    }
}

static void dwc_debounce_run0(void *opaque)
{
    dwc_debounce_run(opaque, 0);
}

static void dwc_debounce_run1(void *opaque)
{
    dwc_debounce_run(opaque, 1);
}

static void dwc_debounce_timer0(void *opaque)
{
    DWCSDMMCState *s = opaque;

    dwc_run_event(s, s->debounce_due[0], dwc_debounce_run0, s);
}

static void dwc_debounce_timer1(void *opaque)
{
    DWCSDMMCState *s = opaque;

    dwc_run_event(s, s->debounce_due[1], dwc_debounce_run1, s);
}

/*
 * [spec:nuos:req:emu.esp32p4.sdmmc]
 * Take the timed events that fell due by @now, in time order, each at its
 * own time, before a change at @now. A change an event makes is part of
 * it and catches up nothing.
 */
static void dwc_catch_up(DWCSDMMCState *s, int64_t now)
{
    static void (*const debounce_run[DWC_SDMMC_MAX_SLOTS])(void *) = {
        dwc_debounce_run0, dwc_debounce_run1,
    };

    if (s->in_event) {
        return;
    }
    for (;;) {
        QEMUTimer *timer = NULL;
        void (*run)(void *opaque) = NULL;
        int64_t due = INT64_MAX;

#define DWC_EARLIEST(t, when, fn) \
        do { \
            if (timer_pending(t) && (when) < due) { \
                timer = (t); \
                due = (when); \
                run = (fn); \
            } \
        } while (0)

        DWC_EARLIEST(&s->reset_timer, s->reset_due, dwc_reset_run);
        DWC_EARLIEST(&s->cmd_timer, s->cmd_due, dwc_cmd_run);
        DWC_EARLIEST(&s->data_timer, s->data_due, dwc_data_run);
        DWC_EARLIEST(&s->timeout_timer, s->timeout_due, dwc_timeout_run);
        for (unsigned slot = 0; slot < DWC_SDMMC_MAX_SLOTS; slot++) {
            DWC_EARLIEST(&s->debounce_timer[slot], s->debounce_due[slot],
                         debounce_run[slot]);
        }
#undef DWC_EARLIEST
        if (!timer || due > now) {
            return;
        }
        timer_del(timer);
        dwc_run_event(s, due, run, s);
    }
}

void dwc_sdmmc_catch_up(DWCSDMMCState *s)
{
    dwc_catch_up(s, dwc_now());
}

static void dwc_card_detect_n(void *opaque, int slot, int level)
{
    DWCSDMMCState *s = opaque;

    dwc_catch_up(s, dwc_now());
    s->cd_n_in = deposit32(s->cd_n_in, slot, 1, level != 0);
    dwc_debounce_arm(s, slot);
}

static void dwc_card_write_prt(void *opaque, int slot, int level)
{
    DWCSDMMCState *s = opaque;

    s->wp_in = deposit32(s->wp_in, slot, 1, level != 0);
}

static void dwc_card_int_n(void *opaque, int slot, int level)
{
    DWCSDMMCState *s = opaque;

    dwc_catch_up(s, dwc_now());
    s->int_n_in = deposit32(s->int_n_in, slot, 1, level != 0);
    dwc_sdio_irq_update(s);
    dwc_sdmmc_update_irq(s);
}

static void dwc_card_irq(DeviceState *dev, unsigned slot, bool level)
{
    DWCSDMMCState *s = DWC_SDMMC(dev);

    dwc_catch_up(s, dwc_now());
    s->card_irq = deposit32(s->card_irq, slot, 1, level);
    dwc_sdio_irq_update(s);
    dwc_sdmmc_update_irq(s);
}

static void dwc_card_irq0(DeviceState *dev, bool level)
{
    dwc_card_irq(dev, 0, level);
}

static void dwc_card_irq1(DeviceState *dev, bool level)
{
    dwc_card_irq(dev, 1, level);
}

static void dwc_clock_update(void *opaque, ClockEvent event)
{
    DWCSDMMCState *s = opaque;

    if (event == ClockPreUpdate) {
        dwc_catch_up(s, dwc_now());
        return;
    }
    for (unsigned slot = 0; slot < s->num_slots; slot++) {
        if (!timer_pending(&s->debounce_timer[slot])) {
            dwc_debounce_arm(s, slot);
        }
    }
    if (s->resets_pending && !timer_pending(&s->reset_timer)) {
        dwc_resets_schedule(s);
    }
    dwc_sdmmc_update(s);
}

void dwc_sdmmc_clock_changed(DWCSDMMCState *s)
{
    dwc_clock_update(s, ClockUpdate);
}

static void dwc_sdmmc_init(Object *obj)
{
    DWCSDMMCState *s = DWC_SDMMC(obj);
    DeviceState *dev = DEVICE(obj);

    for (unsigned slot = 0; slot < DWC_SDMMC_MAX_SLOTS; slot++) {
        g_autofree char *name = g_strdup_printf("sd-bus%u", slot);
        g_autofree char *type = g_strdup_printf(TYPE_DWC_SDMMC_BUS "%u",
                                                slot);

        qbus_init(&s->sdbus[slot], sizeof(s->sdbus[slot]), type, dev, name);
    }
    memory_region_init_io(&s->iomem, obj, &dwc_sdmmc_ops, s, TYPE_DWC_SDMMC,
                          DWC_SDMMC_MMIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq);
    s->hclk = qdev_init_clock_in(dev, "hclk", dwc_clock_update, s,
                                 ClockPreUpdate | ClockUpdate);
    s->cclk = qdev_init_clock_in(dev, "cclk-in", dwc_clock_update, s,
                                 ClockPreUpdate | ClockUpdate);
    qdev_init_gpio_in_named(dev, dwc_card_detect_n, DWC_SDMMC_CARD_DETECT_N,
                            DWC_SDMMC_MAX_SLOTS);
    qdev_init_gpio_in_named(dev, dwc_card_write_prt,
                            DWC_SDMMC_CARD_WRITE_PRT, DWC_SDMMC_MAX_SLOTS);
    qdev_init_gpio_in_named(dev, dwc_card_int_n, DWC_SDMMC_CARD_INT_N,
                            DWC_SDMMC_MAX_SLOTS);
    qdev_init_gpio_out_named(dev, s->card_rst_n, DWC_SDMMC_CARD_RST_N,
                             DWC_SDMMC_MAX_SLOTS);
    qdev_init_gpio_out_named(dev, &s->od_pullup_en_n,
                             DWC_SDMMC_CCMD_OD_PULLUP_EN_N, 1);
    timer_init_ns(&s->cmd_timer, QEMU_CLOCK_VIRTUAL, dwc_cmd_timer, s);
    timer_init_ns(&s->data_timer, QEMU_CLOCK_VIRTUAL, dwc_data_timer, s);
    timer_init_ns(&s->timeout_timer, QEMU_CLOCK_VIRTUAL, dwc_timeout_timer,
                  s);
    timer_init_ns(&s->reset_timer, QEMU_CLOCK_VIRTUAL, dwc_reset_timer, s);
    timer_init_ns(&s->debounce_timer[0], QEMU_CLOCK_VIRTUAL,
                  dwc_debounce_timer0, s);
    timer_init_ns(&s->debounce_timer[1], QEMU_CLOCK_VIRTUAL,
                  dwc_debounce_timer1, s);
    /* Inputs no one drives: card present, writable, no SDIO interrupt. */
    s->cd_n_in = 0;
    s->wp_in = 0;
    s->int_n_in = MAKE_64BIT_MASK(0, DWC_SDMMC_MAX_SLOTS);
}

static void dwc_sdmmc_realize(DeviceState *dev, Error **errp)
{
    DWCSDMMCState *s = DWC_SDMMC(dev);

    if (!s->num_slots || s->num_slots > DWC_SDMMC_MAX_SLOTS) {
        error_setg(errp, "num-slots must be 1 or 2");
        return;
    }
    if (!s->fifo_depth || s->fifo_depth > DWC_SDMMC_MAX_FIFO_WORDS) {
        error_setg(errp, "fifo-depth must be 1 to %u words",
                   DWC_SDMMC_MAX_FIFO_WORDS);
        return;
    }
    if (!s->dma_as) {
        s->dma_as = &address_space_memory;
        s->dma_attrs = MEMTXATTRS_UNSPECIFIED;
    }
}

static void dwc_sdmmc_reset_hold(Object *obj, ResetType type)
{
    DWCSDMMCState *s = DWC_SDMMC(obj);

    timer_del(&s->cmd_timer);
    timer_del(&s->data_timer);
    timer_del(&s->timeout_timer);
    timer_del(&s->reset_timer);
    for (unsigned slot = 0; slot < DWC_SDMMC_MAX_SLOTS; slot++) {
        timer_del(&s->debounce_timer[slot]);
    }
    s->in_event = false;

    s->ctrl = s->ctrl_reset;
    s->pwren = 0;
    s->clkdiv = 0;
    s->clksrc = 0;
    s->clkena = 0;
    s->tmout = 0xffffff40;
    s->ctype = 0;
    s->blksiz = 0x200;
    s->bytcnt = 0x200;
    s->intmask = 0;
    s->cmdarg = 0;
    s->cmd = CMD_RESET;
    memset(s->resp, 0, sizeof(s->resp));
    s->rintsts = 0;
    s->fifoth = 0;
    s->tcbcnt = 0;
    s->tbbcnt = 0;
    s->debnce = 0;
    s->usrid = 0;
    s->uhs = 0;
    s->rst_n = 1;
    s->bmod = 0;
    s->dbaddr = 0;
    s->idsts = 0;
    s->idinten = 0;
    s->dscaddr = 0;
    s->bufaddr = 0;
    s->cardthrctl = 0;
    s->back_end_power = 0;
    s->emmcddr = 0;
    s->enshift = 0;
    s->clk_edge_sel = s->clk_edge_reset;

    s->ciu_clkdiv = 0;
    s->ciu_clksrc = 0;
    s->ciu_clkena = 0;
    /* The debounced detect starts as the input stands. */
    s->cd_n = s->cd_n_in;
    s->app_cmd = 0;

    s->cmd_phase = DWC_CMD_IDLE;
    s->ciu_cmd = 0;
    s->ciu_cmdarg = 0;
    s->autostop_pending = false;
    s->autostop_busy = false;
    s->pend_resp_valid = false;
    s->pend_rint = 0;
    s->resp_index = 0;
    s->cmd_sent_ns = 0;

    s->data_active = false;
    s->data_started = false;
    s->chunk_busy = false;
    s->timeout_kind = TIMEOUT_NONE;
    s->xfer_dir = XFER_NONE;
    s->card_error = false;
    dwc_fifo_reset(s);

    s->idmac_state = DWC_IDMAC_IDLE;
    s->idmac_next = 0;
    s->idmac_buf = 0;
    s->idmac_buf_off = 0;
    memset(s->idmac_desc, 0, sizeof(s->idmac_desc));
    s->resets_pending = 0;
}

static void dwc_sdmmc_reset_exit(Object *obj, ResetType type)
{
    DWCSDMMCState *s = DWC_SDMMC(obj);

    for (unsigned slot = 0; slot < DWC_SDMMC_MAX_SLOTS; slot++) {
        qemu_set_irq(s->card_rst_n[slot], s->rst_n & BIT(slot));
    }
    qemu_set_irq(s->od_pullup_en_n, !(s->ctrl & CTRL_OD_PULLUP));
    dwc_sdio_irq_update(s);
    dwc_sdmmc_update_irq(s);
}

const VMStateDescription vmstate_dwc_sdmmc = {
    .name = TYPE_DWC_SDMMC,
    .version_id = 3,
    .minimum_version_id = 3,
    .fields = (const VMStateField[]) {
        VMSTATE_TIMER(cmd_timer, DWCSDMMCState),
        VMSTATE_TIMER(data_timer, DWCSDMMCState),
        VMSTATE_TIMER(timeout_timer, DWCSDMMCState),
        VMSTATE_TIMER(reset_timer, DWCSDMMCState),
        VMSTATE_TIMER_ARRAY(debounce_timer, DWCSDMMCState,
                            DWC_SDMMC_MAX_SLOTS),
        VMSTATE_INT64(cmd_due, DWCSDMMCState),
        VMSTATE_INT64(data_due, DWCSDMMCState),
        VMSTATE_INT64(timeout_due, DWCSDMMCState),
        VMSTATE_INT64(reset_due, DWCSDMMCState),
        VMSTATE_INT64_ARRAY(debounce_due, DWCSDMMCState,
                            DWC_SDMMC_MAX_SLOTS),
        VMSTATE_UINT32(ctrl, DWCSDMMCState),
        VMSTATE_UINT32(pwren, DWCSDMMCState),
        VMSTATE_UINT32(clkdiv, DWCSDMMCState),
        VMSTATE_UINT32(clksrc, DWCSDMMCState),
        VMSTATE_UINT32(clkena, DWCSDMMCState),
        VMSTATE_UINT32(tmout, DWCSDMMCState),
        VMSTATE_UINT32(ctype, DWCSDMMCState),
        VMSTATE_UINT32(blksiz, DWCSDMMCState),
        VMSTATE_UINT32(bytcnt, DWCSDMMCState),
        VMSTATE_UINT32(intmask, DWCSDMMCState),
        VMSTATE_UINT32(cmdarg, DWCSDMMCState),
        VMSTATE_UINT32(cmd, DWCSDMMCState),
        VMSTATE_UINT32_ARRAY(resp, DWCSDMMCState, 4),
        VMSTATE_UINT32(rintsts, DWCSDMMCState),
        VMSTATE_UINT32(fifoth, DWCSDMMCState),
        VMSTATE_UINT32(tcbcnt, DWCSDMMCState),
        VMSTATE_UINT32(tbbcnt, DWCSDMMCState),
        VMSTATE_UINT32(debnce, DWCSDMMCState),
        VMSTATE_UINT32(usrid, DWCSDMMCState),
        VMSTATE_UINT32(uhs, DWCSDMMCState),
        VMSTATE_UINT32(rst_n, DWCSDMMCState),
        VMSTATE_UINT32(bmod, DWCSDMMCState),
        VMSTATE_UINT32(dbaddr, DWCSDMMCState),
        VMSTATE_UINT32(idsts, DWCSDMMCState),
        VMSTATE_UINT32(idinten, DWCSDMMCState),
        VMSTATE_UINT32(dscaddr, DWCSDMMCState),
        VMSTATE_UINT32(bufaddr, DWCSDMMCState),
        VMSTATE_UINT32(cardthrctl, DWCSDMMCState),
        VMSTATE_UINT32(back_end_power, DWCSDMMCState),
        VMSTATE_UINT32(emmcddr, DWCSDMMCState),
        VMSTATE_UINT32(enshift, DWCSDMMCState),
        VMSTATE_UINT32(clk_edge_sel, DWCSDMMCState),
        VMSTATE_UINT32(ciu_clkdiv, DWCSDMMCState),
        VMSTATE_UINT32(ciu_clksrc, DWCSDMMCState),
        VMSTATE_UINT32(ciu_clkena, DWCSDMMCState),
        VMSTATE_UINT8(cd_n_in, DWCSDMMCState),
        VMSTATE_UINT8(cd_n, DWCSDMMCState),
        VMSTATE_UINT8(wp_in, DWCSDMMCState),
        VMSTATE_UINT8(int_n_in, DWCSDMMCState),
        VMSTATE_UINT8(card_irq, DWCSDMMCState),
        VMSTATE_UINT8(app_cmd, DWCSDMMCState),
        VMSTATE_UINT32(cmd_phase, DWCSDMMCState),
        VMSTATE_UINT32(ciu_cmd, DWCSDMMCState),
        VMSTATE_UINT32(ciu_cmdarg, DWCSDMMCState),
        VMSTATE_BOOL(autostop_pending, DWCSDMMCState),
        VMSTATE_BOOL(autostop_busy, DWCSDMMCState),
        VMSTATE_UINT32_ARRAY(pend_resp, DWCSDMMCState, 4),
        VMSTATE_UINT32(pend_rint, DWCSDMMCState),
        VMSTATE_UINT8(pend_resp_index, DWCSDMMCState),
        VMSTATE_BOOL(pend_resp_valid, DWCSDMMCState),
        VMSTATE_UINT8(resp_index, DWCSDMMCState),
        VMSTATE_INT64(cmd_sent_ns, DWCSDMMCState),
        VMSTATE_BOOL(data_active, DWCSDMMCState),
        VMSTATE_BOOL(data_write, DWCSDMMCState),
        VMSTATE_BOOL(data_stream, DWCSDMMCState),
        VMSTATE_BOOL(data_started, DWCSDMMCState),
        VMSTATE_BOOL(data_autostop, DWCSDMMCState),
        VMSTATE_BOOL(data_dma, DWCSDMMCState),
        VMSTATE_UINT8(data_slot, DWCSDMMCState),
        VMSTATE_UINT32(data_blksz, DWCSDMMCState),
        VMSTATE_UINT32(data_total, DWCSDMMCState),
        VMSTATE_UINT32(data_block_pos, DWCSDMMCState),
        VMSTATE_BOOL(chunk_busy, DWCSDMMCState),
        VMSTATE_UINT32(chunk_len, DWCSDMMCState),
        VMSTATE_UINT8_ARRAY(chunk, DWCSDMMCState, 512),
        VMSTATE_UINT32(timeout_kind, DWCSDMMCState),
        VMSTATE_UINT32(xfer_dir, DWCSDMMCState),
        VMSTATE_BOOL(card_error, DWCSDMMCState),
        VMSTATE_UINT8_ARRAY(fifo, DWCSDMMCState,
                            DWC_SDMMC_MAX_FIFO_WORDS * 4),
        VMSTATE_UINT32(fifo_head, DWCSDMMCState),
        VMSTATE_UINT32(fifo_count, DWCSDMMCState),
        VMSTATE_UINT32(idmac_state, DWCSDMMCState),
        VMSTATE_UINT32(idmac_next, DWCSDMMCState),
        VMSTATE_UINT32_ARRAY(idmac_desc, DWCSDMMCState, 4),
        VMSTATE_UINT32(idmac_buf, DWCSDMMCState),
        VMSTATE_UINT32(idmac_buf_off, DWCSDMMCState),
        VMSTATE_UINT32(resets_pending, DWCSDMMCState),
        VMSTATE_CLOCK(hclk, DWCSDMMCState),
        VMSTATE_CLOCK(cclk, DWCSDMMCState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property dwc_sdmmc_properties[] = {
    DEFINE_PROP_UINT8("num-slots", DWCSDMMCState, num_slots, 2),
    DEFINE_PROP_UINT32("fifo-depth", DWCSDMMCState, fifo_depth, 512),
    DEFINE_PROP_UINT32("verid", DWCSDMMCState, verid, 0x5432270a),
    DEFINE_PROP_UINT32("hcon", DWCSDMMCState, hcon, 0x03444cc3),
    DEFINE_PROP_UINT32("ctrl-reset", DWCSDMMCState, ctrl_reset, 0),
    DEFINE_PROP_UINT32("cmd-mask", DWCSDMMCState, cmd_mask, 0xbfffffff),
    DEFINE_PROP_UINT32("fifoth-mask", DWCSDMMCState, fifoth_mask,
                       0x7fff0fff),
    DEFINE_PROP_BOOL("power-regs", DWCSDMMCState, has_power_regs, true),
    DEFINE_PROP_BOOL("bmod-dsl", DWCSDMMCState, has_dsl, true),
    DEFINE_PROP_UINT32("clk-edge-mask", DWCSDMMCState, clk_edge_mask, 0),
    DEFINE_PROP_UINT32("clk-edge-reset", DWCSDMMCState, clk_edge_reset, 0),
    DEFINE_PROP_BOOL("clk-edge-div", DWCSDMMCState, clk_edge_div, false),
};

static void dwc_sdmmc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    DWCSDMMCClass *sc = DWC_SDMMC_CLASS(klass);

    dc->realize = dwc_sdmmc_realize;
    dc->vmsd = &vmstate_dwc_sdmmc;
    device_class_set_props(dc, dwc_sdmmc_properties);
    rc->phases.hold = dwc_sdmmc_reset_hold;
    rc->phases.exit = dwc_sdmmc_reset_exit;
    sc->cclk_period = dwc_default_cclk_period;
    sc->clock_after = dwc_default_clock_after;
}

static void dwc_sdmmc_bus_class_init(ObjectClass *klass, const void *data)
{
    SDBusClass *sbc = SD_BUS_CLASS(klass);

    sbc->set_irq = data ? dwc_card_irq1 : dwc_card_irq0;
}

/* [spec:nuos:req:emu.esp32p4.sdmmc] */
static const TypeInfo dwc_sdmmc_types[] = {
    {
        .name          = TYPE_DWC_SDMMC,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_init = dwc_sdmmc_init,
        .instance_size = sizeof(DWCSDMMCState),
        .class_size    = sizeof(DWCSDMMCClass),
        .class_init    = dwc_sdmmc_class_init,
    },
    {
        .name          = TYPE_DWC_SDMMC_BUS "0",
        .parent        = TYPE_SD_BUS,
        .instance_size = sizeof(SDBus),
        .class_init    = dwc_sdmmc_bus_class_init,
        .class_data    = NULL,
    },
    {
        .name          = TYPE_DWC_SDMMC_BUS "1",
        .parent        = TYPE_SD_BUS,
        .instance_size = sizeof(SDBus),
        .class_init    = dwc_sdmmc_bus_class_init,
        .class_data    = (void *)1,
    },
};

DEFINE_TYPES(dwc_sdmmc_types)
