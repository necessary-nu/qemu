/*
 * Synopsys DesignWare SD/MMC host controller (DWC_mobile_storage), as
 * Espressif integrates it in the ESP32 and ESP32-P4 (SDHOST)
 *
 * Copyright (c) 2021 Espressif Systems (Shanghai) Co. Ltd.
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef DWC_SDMMC_H
#define DWC_SDMMC_H

#include "qom/object.h"
#include "hw/core/clock.h"
#include "hw/core/sysbus.h"
#include "hw/sd/sd.h"
#include "qemu/timer.h"
#include "system/memory.h"
#include "migration/vmstate.h"

#define TYPE_DWC_SDMMC "dwc_sdmmc"
OBJECT_DECLARE_TYPE(DWCSDMMCState, DWCSDMMCClass, DWC_SDMMC)

/* The slot buses, one per card the controller has: "sd-bus0", "sd-bus1". */
#define TYPE_DWC_SDMMC_BUS "dwc-sdmmc-bus"
#define DWC_SDMMC_MAX_SLOTS 2

/* The largest data FIFO the model takes, in 32-bit words. */
#define DWC_SDMMC_MAX_FIFO_WORDS 1024

/* The register block, 4 KiB, its data FIFO at 0x200. */
#define DWC_SDMMC_MMIO_SIZE 0x1000

/*
 * How the controller connects
 * ===========================
 *
 *   sysbus MMIO 0: the registers. sysbus IRQ 0: the interrupt, high while
 *     CTRL.INT_ENABLE is set and an unmasked RINTSTS bit, or an enabled
 *     IDMAC interrupt summary, is.
 *   Clock inputs: "hclk", the bus interface's clock (its registers, the
 *     IDMAC and the card-detect debounce), and "cclk-in", the card
 *     interface's, which CLKDIV divides for each card. A stopped clock
 *     stops what it times.
 *   GPIO inputs, one line per slot: "card-detect-n" (low while a card is
 *     present), "card-write-prt" (high while it is write protected) and
 *     "card-int-n" (an SDIO interrupt request, low while asserted).
 *   GPIO outputs, one line per slot: "card-rst-n", RST_N's bit for the
 *     slot; and "ccmd-od-pullup-en-n", CTRL.ENABLE_OD_PULLUP inverted.
 *   The slots' buses "sd-bus0" and "sd-bus1", each taking one card.
 *   The IDMAC's accesses go to the AddressSpace dma_as with the
 *     attributes dma_attrs, which a subclass sets before realizing; by
 *     default system memory, unattributed.
 */
#define DWC_SDMMC_CARD_DETECT_N     "card-detect-n"
#define DWC_SDMMC_CARD_WRITE_PRT    "card-write-prt"
#define DWC_SDMMC_CARD_INT_N        "card-int-n"
#define DWC_SDMMC_CARD_RST_N        "card-rst-n"
#define DWC_SDMMC_CCMD_OD_PULLUP_EN_N "ccmd-od-pullup-en-n"

/* An integration's own registers, at 0x800 up, for a subclass to decode. */
struct DWCSDMMCClass {
    SysBusDeviceClass parent_class;

    DeviceRealize parent_realize;
    /*
     * Read or write register @offset of the integration's own; return
     * false for an offset that has none.
     */
    bool (*vendor_read)(DWCSDMMCState *s, hwaddr offset, uint32_t *value);
    bool (*vendor_write)(DWCSDMMCState *s, hwaddr offset, uint32_t value,
                         uint32_t mask);
    /*
     * The card interface clock's period (as Clock periods are), 0 while
     * it is stopped; by default "cclk-in"'s, divided by CLK_EDGE_SEL's N
     * where the integration divides it there.
     */
    uint64_t (*cclk_period)(DWCSDMMCState *s);
    /*
     * The first whole ns by which @cycles of @clk, "cclk-in" or "hclk",
     * have elapsed from @from ns, at least a ns later: by default from
     * @clk's Clock period; an integration whose clocks carry exact
     * periods and edges of their own counts its edges after @from.
     */
    int64_t (*clock_after)(DWCSDMMCState *s, Clock *clk, int64_t from,
                           uint64_t cycles);
};

/* The command the CIU has on the bus, or is about to take. */
typedef enum DWCSDMMCCmdPhase {
    DWC_CMD_IDLE,
    /* A command waits for the CIU to take it: CMD.START_CMD is set. */
    DWC_CMD_TAKING,
    /* On the bus, until its response or timeout. */
    DWC_CMD_BUSY,
    /* Taken, but its card's clock is stopped: it goes nowhere. */
    DWC_CMD_STALLED,
} DWCSDMMCCmdPhase;

typedef enum DWCSDMMCIdmacState {
    DWC_IDMAC_IDLE,
    /* Stopped at a descriptor the host owns, until PLDMND. */
    DWC_IDMAC_SUSPEND,
    /* Holding a descriptor, moving data as the FIFO asks. */
    DWC_IDMAC_ACTIVE,
    /* Stopped by a bus error, until a DMA reset. */
    DWC_IDMAC_FATAL,
} DWCSDMMCIdmacState;

struct DWCSDMMCState {
    SysBusDevice parent_obj;

    SDBus sdbus[DWC_SDMMC_MAX_SLOTS];
    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq card_rst_n[DWC_SDMMC_MAX_SLOTS];
    qemu_irq od_pullup_en_n;
    Clock *hclk;
    Clock *cclk;
    QEMUTimer cmd_timer;
    QEMUTimer data_timer;
    QEMUTimer timeout_timer;
    QEMUTimer reset_timer;
    QEMUTimer debounce_timer[DWC_SDMMC_MAX_SLOTS];
    /* A card that takes time: its data coming, or its busy ending. */
    QEMUTimer card_timer;
    /* When the event each timer runs falls due. */
    int64_t cmd_due;
    int64_t data_due;
    int64_t timeout_due;
    int64_t reset_due;
    int64_t debounce_due[DWC_SDMMC_MAX_SLOTS];
    int64_t card_due;
    /* A timed event is being taken: a change it makes catches up none. */
    bool in_event;

    /* Where the IDMAC's accesses go, and as whom. */
    AddressSpace *dma_as;
    MemTxAttrs dma_attrs;

    /* The integration, as properties */
    uint8_t num_slots;
    uint32_t fifo_depth;
    uint32_t verid;
    uint32_t hcon;
    uint32_t ctrl_reset;
    /* CMD bits that hold what is written (the boot bits, where present) */
    uint32_t cmd_mask;
    /* FIFOTH's writable bits: RX_WMARK is 11 or 12 bits wide */
    uint32_t fifoth_mask;
    /* PWREN (0x04) and BACK_END_POWER (0x104) are present. */
    bool has_power_regs;
    /* BMOD.DSL, the descriptor skip length of ring mode, is present. */
    bool has_dsl;
    /* CLK_EDGE_SEL (0x800): its writable bits (none: absent), reset value */
    uint32_t clk_edge_mask;
    uint32_t clk_edge_reset;
    /* CLK_EDGE_SEL's N divides "cclk-in" by N + 1 for the CIU. */
    bool clk_edge_div;

    /* Registers */
    uint32_t ctrl;
    uint32_t pwren;
    uint32_t clkdiv;
    uint32_t clksrc;
    uint32_t clkena;
    uint32_t tmout;
    uint32_t ctype;
    uint32_t blksiz;
    uint32_t bytcnt;
    uint32_t intmask;
    uint32_t cmdarg;
    uint32_t cmd;
    uint32_t resp[4];
    uint32_t rintsts;
    uint32_t fifoth;
    uint32_t tcbcnt;
    uint32_t tbbcnt;
    uint32_t debnce;
    uint32_t usrid;
    uint32_t uhs;
    uint32_t rst_n;
    uint32_t bmod;
    uint32_t dbaddr;
    uint32_t idsts;
    uint32_t idinten;
    uint32_t dscaddr;
    uint32_t bufaddr;
    uint32_t cardthrctl;
    uint32_t back_end_power;
    uint32_t emmcddr;
    uint32_t enshift;
    uint32_t clk_edge_sel;

    /* The clock registers as the last clock update gave them to the CIU */
    uint32_t ciu_clkdiv;
    uint32_t ciu_clksrc;
    uint32_t ciu_clkena;

    /* The card-side inputs, a bit per slot, and the debounced detect */
    uint8_t cd_n_in;
    uint8_t cd_n;
    uint8_t wp_in;
    uint8_t int_n_in;
    /* SDIO cards' interrupt requests on their buses */
    uint8_t card_irq;
    /* Each slot's card has taken CMD55: its next command is an ACMD. */
    uint8_t app_cmd;

    /* The command path */
    uint32_t cmd_phase;
    /* The command on the bus, as the CIU took it */
    uint32_t ciu_cmd;
    uint32_t ciu_cmdarg;
    /* The CIU is sending the auto-stop CMD12 it owes a data transfer. */
    bool autostop_pending;
    bool autostop_busy;
    /* The response the command on the bus will complete with */
    uint32_t pend_resp[4];
    uint32_t pend_rint;
    uint8_t pend_resp_index;
    bool pend_resp_valid;
    /* The index of the last response, STATUS.RESPONSE_INDEX */
    uint8_t resp_index;
    /* When the command on the bus finishes sending, for STATUS */
    int64_t cmd_sent_ns;

    /* The data path */
    bool data_active;
    bool data_write;
    bool data_stream;
    bool data_started;
    bool data_autostop;
    bool data_dma;
    uint8_t data_slot;
    uint32_t data_blksz;
    uint32_t data_total;
    uint32_t data_block_pos;
    /* The chunk on the bus, delivered when data_timer fires */
    bool chunk_busy;
    uint32_t chunk_len;
    uint8_t chunk[512];
    /* A read chunk's bytes are in chunk[], taken from the card. */
    bool chunk_fetched;
    /* A write is over; BCI waits for its card's busy to end. */
    bool bci_pending;
    /* What the timeout timer waits for: DRTO, HTO or nothing */
    uint32_t timeout_kind;
    /* The host side of the transfer: none, a read or a write */
    uint32_t xfer_dir;
    /* A card error was flagged during the transfer: descriptor CES */
    bool card_error;

    /* The FIFO, a byte ring */
    uint8_t fifo[DWC_SDMMC_MAX_FIFO_WORDS * 4];
    uint32_t fifo_head;
    uint32_t fifo_count;

    /* The IDMAC */
    uint32_t idmac_state;
    uint32_t idmac_next;
    uint32_t idmac_desc[4];
    uint32_t idmac_buf;
    uint32_t idmac_buf_off;

    /* CTRL reset bits waiting to clear */
    uint32_t resets_pending;
};

extern const VMStateDescription vmstate_dwc_sdmmc;

/*
 * For a subclass whose cclk_period() depends on clocks of its own: they
 * are about to change, so the events due before run at the old rate
 * (dwc_sdmmc_catch_up()); they changed, so whatever waits on the card
 * interface clock is re-timed (dwc_sdmmc_clock_changed()).
 */
void dwc_sdmmc_catch_up(DWCSDMMCState *s);
void dwc_sdmmc_clock_changed(DWCSDMMCState *s);

#endif /* DWC_SDMMC_H */
