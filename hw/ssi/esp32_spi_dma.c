/*
 * ESP32 SPI DMA: the linked-list DMA engine of SPI1, SPI2 and SPI3
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

/*
 * The two SPI DMA channels are shared by SPI1, SPI2 and SPI3; each
 * controller's two-bit field in DPORT_SPI_DMA_CHAN_SEL_REG connects it to
 * channel 1 or 2, or to none (TRM 2.5). The controller's own register block
 * holds the DMA registers at 0x100..0x14c, so one instance of this device
 * per controller holds that state, and the engine runs only while the
 * controller has a channel and the SPI_DMA clock in DPORT_PERIP_CLK_EN is
 * running.
 *
 * Descriptors are the three-word lldesc_t of TRM 2.3.2:
 *   DW0: size[11:0], length[23:12], reserved[29:24], eof[30], owner[31]
 *   DW1: buffer address
 *   DW2: next descriptor address, 0 at the end of the list
 * The engine reads and writes internal SRAM 1 and 2 only, the 328 KB DMA
 * address space at 0x3ffae000..0x3fffffff (TRM 3.3.2.6); a link register
 * holds the low 20 bits of its first descriptor's address in that window.
 *
 * Data moves as the SPI controller's data phases run: an SPI transaction
 * completes at once in this model, so the FIFOs between the engine and the
 * shifter never hold data between accesses, and both SPI_OUT_EOF_MODE
 * settings raise the out-EOF flag at the same point.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "hw/ssi/esp32_spi_dma.h"
#include "hw/misc/esp32_reg.h"
#include "hw/misc/esp32_dport.h"

#define DMA_WINDOW_BASE     0x3ff00000u
#define DMA_RAM_START       0x3ffae000u
#define DMA_RAM_END         0x40000000u

#define DW0_SIZE(w)         ((w) & 0xfff)
#define DW0_LENGTH(w)       (((w) >> 12) & 0xfff)
#define DW0_EOF             (1u << 30)
#define DW0_OWNER           (1u << 31)

/* Writable bits of SPI_DMA_CONF: 2..12 and 14..16 */
#define DMA_CONF_MASK       0x0001dffcu
#define DMA_CONF_RESET      R_SPI_DMA_CONF_OUT_EOF_MODE_MASK
#define DMA_INT_MASK        0x1ffu
#define IN_LINK_MASK        (R_SPI_DMA_IN_LINK_ADDR_MASK | \
                             R_SPI_DMA_IN_LINK_AUTO_RET_MASK)

/* Bound on descriptors walked without moving data: a loop of empty ones. */
#define MAX_EMPTY_DSCRS     4096

static void esp32_spi_dma_update_irq(Esp32SpiDmaState *s)
{
    qemu_set_irq(s->irq, (s->int_raw & s->int_ena) != 0);
}

static void esp32_spi_dma_raise(Esp32SpiDmaState *s, uint32_t mask)
{
    s->int_raw |= mask;
    esp32_spi_dma_update_irq(s);
}

/*
 * The DMA channel the controller is connected to, or 0. Value 3 is
 * reserved (TRM 2.5); a channel selected by two controllers is taken to
 * serve neither, as one engine cannot follow two descriptor lists.
 */
static unsigned esp32_spi_dma_channel(Esp32SpiDmaState *s)
{
    uint32_t sel = s->dport->spi_dma_chan_sel;
    unsigned ch = (sel >> (2 * (s->host - 1))) & 3;

    if (ch == 0) {
        return 0;
    }
    if (ch == 3) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_spi_dma: SPI%u selects reserved DMA channel 3\n",
                      s->host);
        return 0;
    }
    for (unsigned h = 1; h <= 3; h++) {
        if (h != s->host && ((sel >> (2 * (h - 1))) & 3) == ch) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "esp32_spi_dma: SPI%u and SPI%u both select DMA "
                          "channel %u\n", s->host, h, ch);
            return 0;
        }
    }
    return ch;
}

/* The engine runs with its clock on, out of reset, and with a channel. */
static bool esp32_spi_dma_running(Esp32SpiDmaState *s)
{
    return clock_is_enabled(s->clk) && esp32_spi_dma_channel(s) != 0;
}

static bool esp32_spi_dma_in_ram(uint32_t addr, uint32_t len)
{
    return addr >= DMA_RAM_START && addr < DMA_RAM_END &&
           len <= DMA_RAM_END - addr;
}

static bool esp32_spi_dma_read_word(uint32_t addr, uint32_t *val)
{
    return address_space_read(&address_space_memory, addr,
                              MEMTXATTRS_UNSPECIFIED, val, 4) == MEMTX_OK;
}

static void esp32_spi_dma_write_word(uint32_t addr, uint32_t val)
{
    uint32_t le = cpu_to_le32(val);

    address_space_write(&address_space_memory, addr, MEMTXATTRS_UNSPECIFIED,
                        &le, 4);
}

static void esp32_spi_dma_show(Esp32SpiDmaState *s, bool out)
{
    Esp32SpiDmaLink *l = out ? &s->out : &s->in;

    if (out) {
        s->outlink_dscr = l->dscr;
        s->outlink_dscr_bf0 = l->next;
        s->outlink_dscr_bf1 = l->buf;
    } else {
        s->inlink_dscr = l->dscr;
        s->inlink_dscr_bf0 = l->next;
        s->inlink_dscr_bf1 = l->buf;
    }
}

/*
 * [spec:nuos:req:emu.esp32.spi-dma]
 * Fetch the descriptor at addr. A descriptor outside the DMA address space
 * or not word-aligned, one the CPU still owns, or one whose buffer runs out
 * of the DMA address space is a descriptor error: the link stops and raises
 * its DSCR_ERROR interrupt. The engine moves data in words, so a buffer
 * address's low two bits are not used.
 */
static bool esp32_spi_dma_load(Esp32SpiDmaState *s, bool out, uint32_t addr)
{
    Esp32SpiDmaLink *l = out ? &s->out : &s->in;
    uint32_t err = out ? R_SPI_DMA_INT_OUTLINK_DSCR_ERROR_MASK
                       : R_SPI_DMA_INT_INLINK_DSCR_ERROR_MASK;
    const char *dir = out ? "outlink" : "inlink";
    uint32_t w[3], n;

    l->active = false;
    l->dscr = addr;
    l->pos = 0;
    if ((addr & 3) || !esp32_spi_dma_in_ram(addr, 12) ||
        !esp32_spi_dma_read_word(addr, &w[0]) ||
        !esp32_spi_dma_read_word(addr + 4, &w[1]) ||
        !esp32_spi_dma_read_word(addr + 8, &w[2])) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_spi_dma: SPI%u %s descriptor at 0x%08x is "
                      "outside DMA-capable SRAM\n", s->host, dir, addr);
        l->dw0 = l->buf = l->next = 0;
        esp32_spi_dma_show(s, out);
        esp32_spi_dma_raise(s, err);
        return false;
    }
    l->dw0 = le32_to_cpu(w[0]);
    l->buf = le32_to_cpu(w[1]);
    l->next = le32_to_cpu(w[2]);
    esp32_spi_dma_show(s, out);

    if (!(l->dw0 & DW0_OWNER)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_spi_dma: SPI%u %s descriptor at 0x%08x is "
                      "owned by the CPU\n", s->host, dir, addr);
        esp32_spi_dma_raise(s, err);
        return false;
    }
    n = out ? DW0_LENGTH(l->dw0) : DW0_SIZE(l->dw0);
    if (l->buf & 3) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_spi_dma: SPI%u %s buffer 0x%08x is not "
                      "word-aligned\n", s->host, dir, l->buf);
        l->buf &= ~3u;
    }
    if (n && !esp32_spi_dma_in_ram(l->buf, n)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_spi_dma: SPI%u %s buffer 0x%08x+%u is outside "
                      "DMA-capable SRAM\n", s->host, dir, l->buf, n);
        esp32_spi_dma_raise(s, err);
        return false;
    }
    l->active = true;
    return true;
}

/*
 * [spec:nuos:req:emu.esp32.spi-dma]
 * The outlink has sent all of a descriptor's length bytes. With
 * SPI_OUT_AUTO_WRBACK the engine hands the descriptor back to the CPU.
 * A descriptor with eof ends a packet; one with no next descriptor ends
 * the list, which SPI_OUTLINK_RESTART can extend.
 */
static void esp32_spi_dma_out_settle(Esp32SpiDmaState *s)
{
    Esp32SpiDmaLink *l = &s->out;

    for (int i = 0; i < MAX_EMPTY_DSCRS; i++) {
        uint32_t ints = R_SPI_DMA_INT_OUT_DONE_MASK;

        if (!l->active || l->pos < DW0_LENGTH(l->dw0)) {
            return;
        }
        if (s->conf & R_SPI_DMA_CONF_OUT_AUTO_WRBACK_MASK) {
            l->dw0 &= ~DW0_OWNER;
            esp32_spi_dma_write_word(l->dscr, l->dw0);
        }
        if (l->dw0 & DW0_EOF) {
            s->out_eof_des_addr = l->dscr;
            s->out_eof_bfr_des_addr = l->buf;
            ints |= R_SPI_DMA_INT_OUT_EOF_MASK;
            if (l->next == 0) {
                ints |= R_SPI_DMA_INT_OUT_TOTAL_EOF_MASK;
            }
        }
        if (l->next == 0) {
            l->active = false;
            esp32_spi_dma_raise(s, ints);
            return;
        }
        esp32_spi_dma_raise(s, ints);
        esp32_spi_dma_load(s, true, l->next);
    }
    qemu_log_mask(LOG_GUEST_ERROR,
                  "esp32_spi_dma: SPI%u outlink loops through empty "
                  "descriptors\n", s->host);
    l->active = false;
}

/*
 * [spec:nuos:req:emu.esp32.spi-dma]
 * The inlink is done with a descriptor: the engine writes back the number
 * of bytes received into its length, sets eof on the one that ends the
 * transfer, and hands it back to the CPU.
 */
static void esp32_spi_dma_in_complete(Esp32SpiDmaState *s, bool eof)
{
    Esp32SpiDmaLink *l = &s->in;
    uint32_t ints = R_SPI_DMA_INT_IN_DONE_MASK;

    l->dw0 = (l->dw0 & 0x3f000fffu) | (l->pos << 12) | (eof ? DW0_EOF : 0);
    esp32_spi_dma_write_word(l->dscr, l->dw0);
    if (eof) {
        s->in_suc_eof_des_addr = l->dscr;
        ints |= R_SPI_DMA_INT_IN_SUC_EOF_MASK;
        l->active = false;
    }
    esp32_spi_dma_raise(s, ints);
}

/* A link's FSM is held while its reset bit, or the AHB master's, is set. */
static bool esp32_spi_dma_link_held(Esp32SpiDmaState *s, bool out)
{
    uint32_t rst = R_SPI_DMA_CONF_AHBM_RST_MASK |
                   (out ? R_SPI_DMA_CONF_OUT_RST_MASK
                        : R_SPI_DMA_CONF_IN_RST_MASK);

    return (s->conf & rst) != 0;
}

/*
 * [spec:nuos:req:emu.esp32.spi-dma]
 * SPI_OUTLINK_START / SPI_INLINK_START: fetch the first descriptor and
 * enable the link. STOP disables it. RESTART re-reads the next pointer of
 * the descriptor the link ended on and carries on from there, so software
 * can append descriptors to a finished list.
 */
static void esp32_spi_dma_link_ctrl(Esp32SpiDmaState *s, bool out,
                                    uint32_t val)
{
    Esp32SpiDmaLink *l = out ? &s->out : &s->in;
    const char *dir = out ? "outlink" : "inlink";
    bool start = val & R_SPI_DMA_OUT_LINK_START_MASK;
    bool restart = val & R_SPI_DMA_OUT_LINK_RESTART_MASK;
    uint32_t next;

    if (val & R_SPI_DMA_OUT_LINK_STOP_MASK) {
        l->active = false;
    }
    if (!start && !restart) {
        return;
    }
    if (!esp32_spi_dma_running(s)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_spi_dma: SPI%u %s started without a DMA channel "
                      "or with the SPI DMA clock off\n", s->host, dir);
        return;
    }
    if (esp32_spi_dma_link_held(s, out)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_spi_dma: SPI%u %s started while held in reset\n",
                      s->host, dir);
        return;
    }
    if (start) {
        esp32_spi_dma_load(s, out, DMA_WINDOW_BASE |
                           (val & R_SPI_DMA_OUT_LINK_ADDR_MASK));
    } else if (!l->active) {
        if (l->dscr == 0 || !esp32_spi_dma_read_word(l->dscr + 8, &next)) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "esp32_spi_dma: SPI%u %s restarted with no "
                          "descriptor to continue from\n", s->host, dir);
            return;
        }
        next = le32_to_cpu(next);
        if (next == 0) {
            return;
        }
        esp32_spi_dma_load(s, out, next);
    }
    if (out) {
        esp32_spi_dma_out_settle(s);
    }
}

static void esp32_spi_dma_link_reset(Esp32SpiDmaLink *l)
{
    memset(l, 0, sizeof(*l));
}

bool esp32_spi_dma_tx_enabled(Esp32SpiDmaState *s)
{
    return s->out.active;
}

bool esp32_spi_dma_rx_enabled(Esp32SpiDmaState *s)
{
    return s->in.active;
}

/*
 * [spec:nuos:req:emu.esp32.spi-dma]
 * The next MOSI byte from the outlink. Once the list is exhausted, or with
 * the engine stopped, the TX FIFO underflows and the shifter sends zeros.
 */
uint8_t esp32_spi_dma_pop(Esp32SpiDmaState *s)
{
    Esp32SpiDmaLink *l = &s->out;
    uint8_t byte = 0;

    if (!l->active || !esp32_spi_dma_running(s)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_spi_dma: SPI%u TX FIFO underflow\n", s->host);
        return 0;
    }
    address_space_read(&address_space_memory, l->buf + l->pos,
                       MEMTXATTRS_UNSPECIFIED, &byte, 1);
    l->pos++;
    esp32_spi_dma_out_settle(s);
    return byte;
}

/*
 * [spec:nuos:req:emu.esp32.spi-dma]
 * Store a MISO byte through the inlink. A full descriptor is completed
 * when the next byte arrives, so the descriptor that takes the last byte
 * of a transfer is the one marked eof. Running out of descriptors raises
 * SPI_INLINK_DSCR_EMPTY_INT and drops the rest of the data.
 */
void esp32_spi_dma_push(Esp32SpiDmaState *s, uint8_t byte)
{
    Esp32SpiDmaLink *l = &s->in;
    int empty = 0;

    if (!l->active || !esp32_spi_dma_running(s)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_spi_dma: SPI%u RX data with no inlink\n",
                      s->host);
        return;
    }
    while (l->pos >= DW0_SIZE(l->dw0)) {
        if (++empty > MAX_EMPTY_DSCRS) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "esp32_spi_dma: SPI%u inlink loops through empty "
                          "descriptors\n", s->host);
            l->active = false;
            return;
        }
        esp32_spi_dma_in_complete(s, false);
        if (l->next == 0) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "esp32_spi_dma: SPI%u inlink out of descriptors\n",
                          s->host);
            l->active = false;
            esp32_spi_dma_raise(s, R_SPI_DMA_INT_INLINK_DSCR_EMPTY_MASK);
            return;
        }
        if (!esp32_spi_dma_load(s, false, l->next)) {
            return;
        }
    }
    address_space_write(&address_space_memory, l->buf + l->pos,
                        MEMTXATTRS_UNSPECIFIED, &byte, 1);
    l->pos++;
}

/*
 * [spec:nuos:req:emu.esp32.spi-dma]
 * The end of an SPI transaction ends the received packet: the descriptor
 * holding its last byte is closed with eof and SPI_IN_SUC_EOF_INT is
 * raised, which ends the inlink.
 */
void esp32_spi_dma_trans_done(Esp32SpiDmaState *s)
{
    if (s->in.active && s->in.pos > 0) {
        esp32_spi_dma_in_complete(s, true);
    }
}

static uint64_t esp32_spi_dma_read(void *opaque, hwaddr addr, unsigned size)
{
    Esp32SpiDmaState *s = ESP32_SPI_DMA(opaque);

    switch (addr) {
    case A_SPI_DMA_CONF:
        return s->conf;
    case A_SPI_DMA_OUT_LINK:
        return s->out_link;
    case A_SPI_DMA_IN_LINK:
        return s->in_link;
    case A_SPI_DMA_STATUS:
        return (s->out.active ? R_SPI_DMA_STATUS_TX_EN_MASK : 0) |
               (s->in.active ? R_SPI_DMA_STATUS_RX_EN_MASK : 0);
    case A_SPI_DMA_INT_ENA:
        return s->int_ena;
    case A_SPI_DMA_INT_RAW:
        return s->int_raw;
    case A_SPI_DMA_INT_ST:
        return s->int_raw & s->int_ena;
    case A_SPI_DMA_INT_CLR:
        return 0;
    case A_SPI_IN_ERR_EOF_DES_ADDR:
        return s->in_err_eof_des_addr;
    case A_SPI_IN_SUC_EOF_DES_ADDR:
        return s->in_suc_eof_des_addr;
    case A_SPI_INLINK_DSCR:
        return s->inlink_dscr;
    case A_SPI_INLINK_DSCR_BF0:
        return s->inlink_dscr_bf0;
    case A_SPI_INLINK_DSCR_BF1:
        return s->inlink_dscr_bf1;
    case A_SPI_OUT_EOF_BFR_DES_ADDR:
        return s->out_eof_bfr_des_addr;
    case A_SPI_OUT_EOF_DES_ADDR:
        return s->out_eof_des_addr;
    case A_SPI_OUTLINK_DSCR:
        return s->outlink_dscr;
    case A_SPI_OUTLINK_DSCR_BF0:
        return s->outlink_dscr_bf0;
    case A_SPI_OUTLINK_DSCR_BF1:
        return s->outlink_dscr_bf1;
    case A_SPI_DMA_RSTATUS:
        /* The TX FIFO holds data from the outlink while it is enabled. */
        return (s->outlink_dscr & R_SPI_DMA_RSTATUS_DES_ADDRESS_MASK) |
               (s->out.active ? 0 : R_SPI_DMA_RSTATUS_FIFO_EMPTY_MASK);
    case A_SPI_DMA_TSTATUS:
        /* Received data goes straight through the RX FIFO to memory. */
        return (s->inlink_dscr & R_SPI_DMA_RSTATUS_DES_ADDRESS_MASK) |
               R_SPI_DMA_RSTATUS_FIFO_EMPTY_MASK;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_spi_dma: read of reserved offset 0x%" HWADDR_PRIx
                      "\n", addr + ESP32_SPI_DMA_REG_OFFSET);
        return 0;
    }
}

static void esp32_spi_dma_write(void *opaque, hwaddr addr, uint64_t value,
                                unsigned size)
{
    Esp32SpiDmaState *s = ESP32_SPI_DMA(opaque);
    uint32_t val = value;

    switch (addr) {
    case A_SPI_DMA_CONF:
        s->conf = val & DMA_CONF_MASK;
        if (s->conf & (R_SPI_DMA_CONF_OUT_RST_MASK |
                       R_SPI_DMA_CONF_AHBM_RST_MASK)) {
            esp32_spi_dma_link_reset(&s->out);
        }
        if (s->conf & (R_SPI_DMA_CONF_IN_RST_MASK |
                       R_SPI_DMA_CONF_AHBM_RST_MASK)) {
            esp32_spi_dma_link_reset(&s->in);
        }
        if (s->conf & (R_SPI_DMA_CONF_DMA_CONTINUE_MASK |
                       R_SPI_DMA_CONF_DMA_TX_STOP_MASK |
                       R_SPI_DMA_CONF_DMA_RX_STOP_MASK)) {
            qemu_log_mask(LOG_UNIMP,
                          "esp32_spi_dma: continuous TX/RX mode is not "
                          "modelled\n");
        }
        if (s->conf & (R_SPI_DMA_CONF_IN_LOOP_TEST_MASK |
                       R_SPI_DMA_CONF_OUT_LOOP_TEST_MASK)) {
            qemu_log_mask(LOG_UNIMP,
                          "esp32_spi_dma: link loop test is not modelled\n");
        }
        break;
    case A_SPI_DMA_OUT_LINK:
        /* START, STOP and RESTART clear themselves. */
        s->out_link = val & R_SPI_DMA_OUT_LINK_ADDR_MASK;
        esp32_spi_dma_link_ctrl(s, true, val);
        break;
    case A_SPI_DMA_IN_LINK:
        s->in_link = val & IN_LINK_MASK;
        esp32_spi_dma_link_ctrl(s, false, val);
        break;
    case A_SPI_DMA_INT_ENA:
        s->int_ena = val & DMA_INT_MASK;
        esp32_spi_dma_update_irq(s);
        break;
    case A_SPI_DMA_INT_CLR:
        s->int_raw &= ~(val & DMA_INT_MASK);
        esp32_spi_dma_update_irq(s);
        break;
    case A_SPI_DMA_STATUS:
    case A_SPI_DMA_INT_RAW:
    case A_SPI_DMA_INT_ST:
    case A_SPI_IN_ERR_EOF_DES_ADDR ... A_SPI_DMA_TSTATUS:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_spi_dma: write to read-only offset 0x%"
                      HWADDR_PRIx "\n", addr + ESP32_SPI_DMA_REG_OFFSET);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_spi_dma: write to reserved offset 0x%"
                      HWADDR_PRIx "\n", addr + ESP32_SPI_DMA_REG_OFFSET);
        break;
    }
}

static const MemoryRegionOps esp32_spi_dma_ops = {
    .read = esp32_spi_dma_read,
    .write = esp32_spi_dma_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void esp32_spi_dma_reset_hold(Object *obj, ResetType type)
{
    Esp32SpiDmaState *s = ESP32_SPI_DMA(obj);

    s->conf = DMA_CONF_RESET;
    s->out_link = 0;
    s->in_link = 0;
    s->int_ena = 0;
    s->int_raw = 0;
    s->in_err_eof_des_addr = 0;
    s->in_suc_eof_des_addr = 0;
    s->inlink_dscr = 0;
    s->inlink_dscr_bf0 = 0;
    s->inlink_dscr_bf1 = 0;
    s->out_eof_bfr_des_addr = 0;
    s->out_eof_des_addr = 0;
    s->outlink_dscr = 0;
    s->outlink_dscr_bf0 = 0;
    s->outlink_dscr_bf1 = 0;
    esp32_spi_dma_link_reset(&s->out);
    esp32_spi_dma_link_reset(&s->in);
    esp32_spi_dma_update_irq(s);
}

static void esp32_spi_dma_realize(DeviceState *dev, Error **errp)
{
    Esp32SpiDmaState *s = ESP32_SPI_DMA(dev);

    if (!s->dport) {
        error_setg(errp, "esp32_spi_dma: the dport link is not set");
        return;
    }
    if (s->host < 1 || s->host > 3) {
        error_setg(errp, "esp32_spi_dma: host must be 1, 2 or 3");
        return;
    }
}

static void esp32_spi_dma_init(Object *obj)
{
    Esp32SpiDmaState *s = ESP32_SPI_DMA(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32_spi_dma_ops, s,
                          TYPE_ESP32_SPI_DMA, ESP32_SPI_DMA_REG_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    s->clk = qdev_init_clock_in(DEVICE(obj), "apb", NULL, NULL, 0);
}

static const VMStateDescription vmstate_esp32_spi_dma_link = {
    .name = "esp32.spi_dma.link",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_BOOL(active, Esp32SpiDmaLink),
        VMSTATE_UINT32(dscr, Esp32SpiDmaLink),
        VMSTATE_UINT32(dw0, Esp32SpiDmaLink),
        VMSTATE_UINT32(buf, Esp32SpiDmaLink),
        VMSTATE_UINT32(next, Esp32SpiDmaLink),
        VMSTATE_UINT32(pos, Esp32SpiDmaLink),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_esp32_spi_dma = {
    .name = TYPE_ESP32_SPI_DMA,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_CLOCK(clk, Esp32SpiDmaState),
        VMSTATE_UINT32(conf, Esp32SpiDmaState),
        VMSTATE_UINT32(out_link, Esp32SpiDmaState),
        VMSTATE_UINT32(in_link, Esp32SpiDmaState),
        VMSTATE_UINT32(int_ena, Esp32SpiDmaState),
        VMSTATE_UINT32(int_raw, Esp32SpiDmaState),
        VMSTATE_UINT32(in_err_eof_des_addr, Esp32SpiDmaState),
        VMSTATE_UINT32(in_suc_eof_des_addr, Esp32SpiDmaState),
        VMSTATE_UINT32(inlink_dscr, Esp32SpiDmaState),
        VMSTATE_UINT32(inlink_dscr_bf0, Esp32SpiDmaState),
        VMSTATE_UINT32(inlink_dscr_bf1, Esp32SpiDmaState),
        VMSTATE_UINT32(out_eof_bfr_des_addr, Esp32SpiDmaState),
        VMSTATE_UINT32(out_eof_des_addr, Esp32SpiDmaState),
        VMSTATE_UINT32(outlink_dscr, Esp32SpiDmaState),
        VMSTATE_UINT32(outlink_dscr_bf0, Esp32SpiDmaState),
        VMSTATE_UINT32(outlink_dscr_bf1, Esp32SpiDmaState),
        VMSTATE_STRUCT(out, Esp32SpiDmaState, 1, vmstate_esp32_spi_dma_link,
                       Esp32SpiDmaLink),
        VMSTATE_STRUCT(in, Esp32SpiDmaState, 1, vmstate_esp32_spi_dma_link,
                       Esp32SpiDmaLink),
        VMSTATE_END_OF_LIST()
    }
};

static const Property esp32_spi_dma_properties[] = {
    DEFINE_PROP_UINT8("host", Esp32SpiDmaState, host, 0),
    DEFINE_PROP_LINK("dport", Esp32SpiDmaState, dport, TYPE_ESP32_DPORT,
                     Esp32DportState *),
};

static void esp32_spi_dma_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32_spi_dma_reset_hold;
    dc->realize = esp32_spi_dma_realize;
    dc->vmsd = &vmstate_esp32_spi_dma;
    device_class_set_props(dc, esp32_spi_dma_properties);
}

/* [spec:nuos:req:emu.esp32.spi-dma] */
static const TypeInfo esp32_spi_dma_info = {
    .name = TYPE_ESP32_SPI_DMA,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32SpiDmaState),
    .instance_init = esp32_spi_dma_init,
    .class_init = esp32_spi_dma_class_init,
};

static void esp32_spi_dma_register_types(void)
{
    type_register_static(&esp32_spi_dma_info);
}

type_init(esp32_spi_dma_register_types)
