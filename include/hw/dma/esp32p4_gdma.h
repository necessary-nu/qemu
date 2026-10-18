/*
 * ESP32-P4 general-purpose DMA controllers, GDMA-AHB and GDMA-AXI
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_DMA_ESP32P4_GDMA_H
#define HW_DMA_ESP32P4_GDMA_H

#include "hw/core/sysbus.h"
#include "hw/misc/esp32p4_clk_period.h"
#include "hw/core/clock.h"
#include "qemu/timer.h"
#include "qemu/notify.h"
#include "qom/object.h"
#include "system/memory.h"

#define TYPE_ESP32P4_GDMA "esp32p4-gdma"
#define TYPE_ESP32P4_AHB_GDMA "esp32p4-ahb-gdma"
#define TYPE_ESP32P4_AXI_GDMA "esp32p4-axi-gdma"
OBJECT_DECLARE_TYPE(ESP32P4GdmaState, ESP32P4GdmaClass, ESP32P4_GDMA)

#define ESP32P4_AHB_GDMA_BASE   0x50085000
#define ESP32P4_AHB_GDMA_SIZE   0x1000
#define ESP32P4_AXI_GDMA_BASE   0x5008a000
#define ESP32P4_AXI_GDMA_SIZE   0x2000

/* Each controller has three receive and three transmit channels. */
#define ESP32P4_GDMA_CHANNELS   3
#define ESP32P4_GDMA_IN         0
#define ESP32P4_GDMA_OUT        1

/*
 * The values of *_PERI_IN_SEL_CHn and *_PERI_OUT_SEL_CHn (TRM tables
 * 6.4-1 and 6.4-2). 0 to 15 select a peripheral or, where none is
 * connected, a dummy for memory-to-memory transfers; 16 to 62 are invalid
 * and 63, the reset value, connects the channel to nothing.
 */
#define ESP32P4_GDMA_PERIPHS    16
#define ESP32P4_GDMA_PERI_NONE  63

enum {
    ESP32P4_GDMA_AHB_I3C = 0,
    ESP32P4_GDMA_AHB_UHCI0 = 2,
    ESP32P4_GDMA_AHB_I2S0 = 3,
    ESP32P4_GDMA_AHB_I2S1 = 4,
    ESP32P4_GDMA_AHB_I2S2 = 5,
    ESP32P4_GDMA_AHB_ADC = 8,
    ESP32P4_GDMA_AHB_RMT = 10,
};

enum {
    ESP32P4_GDMA_AXI_LCD_CAM = 0,
    ESP32P4_GDMA_AXI_SPI2 = 1,
    ESP32P4_GDMA_AXI_SPI3 = 2,
    ESP32P4_GDMA_AXI_PARLIO = 3,
    ESP32P4_GDMA_AXI_AES = 4,
    ESP32P4_GDMA_AXI_SHA = 5,
};

/*
 * How a peripheral moves data through GDMA
 * ========================================
 *
 * A peripheral served by GDMA (UHCI, I2S, SPI, the crypto blocks, ...)
 * takes a QOM link to its controller and is known to it by its selection
 * value above. Software connects a channel to it by writing that value to
 * the channel's PERI_SEL; the peripheral need not know which channel.
 *
 *   - At realize, the peripheral attaches with esp32p4_gdma_attach(),
 *     giving the callbacks below.
 *
 *   - Transmit (memory to peripheral): the TX channel that selects the
 *     peripheral reads its outlink and the buffers' data into its FIFO.
 *     The peripheral takes bytes from that FIFO as its own hardware
 *     consumes them, with esp32p4_gdma_pull(), which returns how many it
 *     got (0 when the FIFO is empty or no running channel selects it) and
 *     stops after the last byte of a descriptor whose suc_eof is set,
 *     reporting that in *eof: the end of the frame the peripheral is to
 *     mark (TRM 6.4.1). Taking bytes is what completes the descriptors:
 *     OUT_DONE, and OUT_EOF and OUT_TOTAL_EOF where OUT_EOF_MODE says so.
 *     ops->tx_ready is called whenever bytes arrive in an empty FIFO.
 *
 *   - Receive (peripheral to memory): the peripheral gives the bytes it
 *     receives to the RX channel that selects it with esp32p4_gdma_push(),
 *     which takes as many as the FIFO has room for (none while no running
 *     channel selects the peripheral) and returns that count. flags marks
 *     the last byte given: ESP32P4_GDMA_EOF ends the frame, which closes
 *     the descriptor with suc_eof (IN_SUC_EOF); ESP32P4_GDMA_ERR_EOF ends it
 *     in error (err_eof, IN_ERR_EOF), which only UHCI and PARLIO report. A
 *     push that takes fewer bytes than given takes no flag. ops->rx_ready
 *     is called whenever room appears in a full FIFO and when the channel
 *     starts.
 *
 *   - esp32p4_gdma_tx_pending() and esp32p4_gdma_rx_room() give the
 *     bytes a pull would get and a push could give at the moment, and
 *     esp32p4_gdma_tx_flowing() and esp32p4_gdma_rx_flowing() whether
 *     more will come without software's help.
 *
 * The channel reads and writes memory as the bus arbitration lets it, at
 * the controller's bus clock, through the access permission managers as
 * the DMA master of the peripheral it serves. A qtest can play any
 * peripheral through the controller's "periph" property
 * (hw/dma/esp32p4_gdma.c).
 *
 * Time. The controller's bus rounds happen at the times they fall due,
 * however late the QEMU timer that runs them: each runs as of its own
 * time (esp32_line_run_at(), include/hw/gpio/esp32_line.h), and a pull,
 * push or event a round causes carries that time. A peripheral that takes
 * its own events late in the same way, and moves data at them, keeps the
 * two in step: before an event of its own at time t it calls
 * esp32p4_gdma_sync(s, t), which runs what an interposer on the
 * controller's paths has due by t and the rounds that fell due by then,
 * so the event sees the path as it was at t. A register access or a bus
 * clock change at the controller first runs, for each attached peripheral
 * with ops->sync, ops->sync(opaque, now), which takes the peripheral's
 * events due by then (syncing the controller before each), and then the
 * interposers' events and the controller's own rounds due by then.
 */
#define ESP32P4_GDMA_EOF        (1u << 0)
#define ESP32P4_GDMA_ERR_EOF    (1u << 1)

typedef struct ESP32P4GdmaPeriphOps {
    void (*tx_ready)(void *opaque);
    void (*rx_ready)(void *opaque);
    void (*sync)(void *opaque, int64_t when);
} ESP32P4GdmaPeriphOps;

void esp32p4_gdma_attach(ESP32P4GdmaState *s, unsigned periph,
                         const ESP32P4GdmaPeriphOps *ops, void *opaque);
void esp32p4_gdma_sync(ESP32P4GdmaState *s, int64_t when);
size_t esp32p4_gdma_pull(ESP32P4GdmaState *s, unsigned periph,
                         uint8_t *buf, size_t len, bool *eof);
size_t esp32p4_gdma_push(ESP32P4GdmaState *s, unsigned periph,
                         const uint8_t *buf, size_t len, unsigned flags);
size_t esp32p4_gdma_tx_pending(ESP32P4GdmaState *s, unsigned periph);
size_t esp32p4_gdma_rx_room(ESP32P4GdmaState *s, unsigned periph);

/*
 * Whether the channel serving @periph will go on moving data by itself: a
 * TX channel with bytes in its FIFO, or running (and clocked) with
 * descriptors still to read; an RX channel taking bytes, with room in its
 * FIFO or running with descriptors still to fill. A peripheral whose own
 * FIFO meets an empty or full DMA FIFO waits for the DMA while this holds,
 * and under- or overflows when it does not.
 */
bool esp32p4_gdma_tx_flowing(ESP32P4GdmaState *s, unsigned periph);
bool esp32p4_gdma_rx_flowing(ESP32P4GdmaState *s, unsigned periph);

/*
 * A block in a peripheral's DMA stream: the BitScrambler
 * ======================================================
 *
 * The BitScrambler (include/hw/dma/esp32p4_bitscrambler.h) sits between
 * a peripheral and the channels that serve it, on the paths
 * HP_SYSTEM_BITSCRAMBLER_PERI_SEL attaches it to (TRM 62.4.2), and
 * rewrites the bytes that pass. It interposes on one path of one
 * peripheral with esp32p4_gdma_interpose(s, dir, periph, ops, opaque),
 * @dir ESP32P4_GDMA_OUT for the TX path or ESP32P4_GDMA_IN for the RX
 * path; NULL ops restore the direct path. While it does:
 *
 *   - the peripheral's own calls on that path reach the interposer's ops
 *     in place of the channel: esp32p4_gdma_pull() and _tx_pending() call
 *     ops->pull and ops->tx_pending, esp32p4_gdma_push() and _rx_room()
 *     call ops->push and ops->rx_room, with the same meaning;
 *   - the channel's readiness that would have reached the peripheral
 *     (bytes arriving in an empty TX FIFO; room in a full RX FIFO, or an
 *     RX channel starting) calls ops->chan_ready(opaque, dir) instead;
 *   - the interposer reaches the channel with esp32p4_gdma_chan_pull(),
 *     _chan_push(), _chan_tx_pending() and _chan_rx_room(), which act on
 *     the channel exactly as the peripheral's calls do on a direct path,
 *     and tells the peripheral of bytes it has for it, or room it has
 *     made, with esp32p4_gdma_periph_ready(), which calls the
 *     peripheral's tx_ready or rx_ready;
 *   - esp32p4_gdma_sync(s, t) calls ops->sync(opaque, t), with which an
 *     interposer that takes its own steps late takes those due by t,
 *     each at its own time, running the channel's rounds due before each
 *     with esp32p4_gdma_chan_sync().
 *
 * The interposition is wiring, not controller state: a reset of the
 * controller keeps it.
 */
typedef struct ESP32P4GdmaInterposerOps {
    size_t (*pull)(void *opaque, uint8_t *buf, size_t len, bool *eof);
    size_t (*tx_pending)(void *opaque);
    size_t (*push)(void *opaque, const uint8_t *buf, size_t len,
                   unsigned flags);
    size_t (*rx_room)(void *opaque);
    void (*chan_ready)(void *opaque, int dir);
    void (*sync)(void *opaque, int64_t when);
} ESP32P4GdmaInterposerOps;

void esp32p4_gdma_interpose(ESP32P4GdmaState *s, int dir, unsigned periph,
                            const ESP32P4GdmaInterposerOps *ops,
                            void *opaque);
size_t esp32p4_gdma_chan_pull(ESP32P4GdmaState *s, unsigned periph,
                              uint8_t *buf, size_t len, bool *eof);
size_t esp32p4_gdma_chan_push(ESP32P4GdmaState *s, unsigned periph,
                              const uint8_t *buf, size_t len,
                              unsigned flags);
size_t esp32p4_gdma_chan_tx_pending(ESP32P4GdmaState *s, unsigned periph);
size_t esp32p4_gdma_chan_rx_room(ESP32P4GdmaState *s, unsigned periph);
void esp32p4_gdma_chan_sync(ESP32P4GdmaState *s, int64_t when);
void esp32p4_gdma_periph_ready(ESP32P4GdmaState *s, unsigned periph,
                               int dir);

/*
 * Named GPIOs.
 *
 * ESP32P4_GDMA_ETM_EVENT, "etm-event" (output, 27 lines): the ETM events,
 *   each pulsed as it occurs, line ESP32P4_GDMA_ETM_EVT(kind, channel)
 *   for the kinds below, in the order of ESP-IDF's soc_etm_source.h
 *   (PDMA_AHB_EVT_IN_DONE_CH0 + line, or PDMA_AXI_...).
 * ESP32P4_GDMA_ETM_TASK, "etm-task" (input, 6 lines): the ETM tasks
 *   IN_START_CH0..2 then OUT_START_CH0..2 (PDMA_AHB_TASK_IN_START_CH0 +
 *   line), each acting on a rising edge as *LINK_START does, but only
 *   while the channel's *_ETM_EN is set.
 *
 * The interrupts are the sysbus IRQs IN_CH0..2 then OUT_CH0..2, the
 * interrupt matrix's AHB_PDMA_* or AXI_PDMA_* sources in that order.
 */
#define ESP32P4_GDMA_ETM_EVENT  "etm-event"
#define ESP32P4_GDMA_ETM_TASK   "etm-task"

enum {
    ESP32P4_GDMA_EVT_IN_DONE,
    ESP32P4_GDMA_EVT_IN_SUC_EOF,
    ESP32P4_GDMA_EVT_IN_FIFO_EMPTY,
    ESP32P4_GDMA_EVT_IN_FIFO_FULL,
    ESP32P4_GDMA_EVT_OUT_DONE,
    ESP32P4_GDMA_EVT_OUT_EOF,
    ESP32P4_GDMA_EVT_OUT_TOTAL_EOF,
    ESP32P4_GDMA_EVT_OUT_FIFO_EMPTY,
    ESP32P4_GDMA_EVT_OUT_FIFO_FULL,
    ESP32P4_GDMA_EVT_KINDS,
};
#define ESP32P4_GDMA_ETM_EVT(kind, ch) ((kind) * ESP32P4_GDMA_CHANNELS + (ch))
#define ESP32P4_GDMA_ETM_EVENTS (ESP32P4_GDMA_EVT_KINDS * ESP32P4_GDMA_CHANNELS)
#define ESP32P4_GDMA_ETM_TASKS  (2 * ESP32P4_GDMA_CHANNELS)

/* A channel's FIFO, in bytes: the AXI one as the model has it. */
#define ESP32P4_GDMA_FIFO_MAX   64

/* The bytes in a channel's FIFO, each with what its leaving completes. */
typedef struct ESP32P4GdmaFifo {
    uint8_t data[ESP32P4_GDMA_FIFO_MAX];
    uint8_t flags[ESP32P4_GDMA_FIFO_MAX];
    uint32_t desc[ESP32P4_GDMA_FIFO_MAX];
    uint32_t head;
    uint32_t num;
} ESP32P4GdmaFifo;

typedef struct ESP32P4GdmaChan {
    /* Registers. */
    uint32_t int_raw;
    uint32_t int_ena;
    uint32_t conf0;
    uint32_t conf1;
    uint32_t link;          /* INLINK_AUTO_RET; 0 for a TX channel */
    uint32_t link_addr;
    uint32_t pri;           /* AXI: priority, weight and opt-dis bits */
    uint32_t peri_sel;
    uint32_t weight;        /* AHB only */
    uint32_t weight_opt_dis;
    uint32_t eof_des_addr;  /* IN_SUC_EOF_DES_ADDR / OUT_EOF_DES_ADDR */
    uint32_t eof_bfr_des_addr; /* IN_ERR_EOF_DES_ADDR / OUT_EOF_BFR_ */
    uint32_t dscr;
    uint32_t dscr_bf0;
    uint32_t dscr_bf1;
    uint32_t done_des_addr;
    uint32_t pop;           /* IN_POP's RDATA; OUT_PUSH's WDATA */
    uint32_t crc_init;
    uint32_t crc_width;
    uint32_t crc_clear;
    uint32_t crc_en_wr_data;
    uint32_t crc_en_addr;
    uint32_t crc_data_en_wr_data;
    uint32_t crc_data_en_addr;
    uint32_t crc_lfsr_mask[32];
    uint32_t crc_data_mask[32];
    uint32_t crc;

    /* The link FSM and the descriptor it holds. */
    bool active;
    bool has_desc;
    bool end;               /* the last descriptor's data is all moved */
    bool restart;           /* a RESTART to take at the list's end */
    bool switch_pending;    /* the next fetch is of a new linked list */
    bool empty_raised;      /* IN_DSCR_EMPTY raised at this list's end */
    uint32_t desc_addr;
    uint32_t desc_dw0;
    uint32_t desc_buf;
    uint32_t desc_next;
    uint32_t pos;
    uint32_t prev_desc;
    uint32_t tokens;

    ESP32P4GdmaFifo fifo;
} ESP32P4GdmaChan;

struct ESP32P4GdmaState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq[2 * ESP32P4_GDMA_CHANNELS];
    qemu_irq etm_event[ESP32P4_GDMA_ETM_EVENTS];
    Clock *clk;
    QEMUTimer *timer;
    /*
     * The bus clock's cycles, counted from its last change, and the cycle
     * of the round @timer runs, which falls due at @due_ns.
     */
    ESP32P4ClkCount bus_edges;
    int64_t due_cycle;
    int64_t due_ns;
    /* The bus as the channels reach it, through the APMs. */
    MemoryRegion *bus;
    AddressSpace *as;
    Object *pms;

    ESP32P4GdmaChan ch[2][ESP32P4_GDMA_CHANNELS];

    uint32_t ahb_test;
    uint32_t misc_conf;
    uint32_t date;
    uint32_t intr_mem_start;
    uint32_t intr_mem_end;
    uint32_t extr_mem_start;
    uint32_t extr_mem_end;
    uint32_t arb_timeout;
    uint32_t arb_timeout_tx;    /* AHB's separate TX and RX registers */
    uint32_t arb_timeout_rx;
    uint32_t weight_en;
    uint32_t weight_en_tx;
    uint32_t weight_en_rx;
    uint32_t module_clk_en;
    uint32_t resp_err_addr;
    uint32_t resp_err_info;
    uint32_t in_mem_conf;
    uint32_t rdn_result;
    uint32_t rdn_eco_high;
    uint32_t rdn_eco_low;
    uint32_t link_switch_state;
    bool etm_task_level[ESP32P4_GDMA_ETM_TASKS];

    /*
     * The arbiters: the last channel granted and their time slots' starts,
     * as cycles of @bus_edges.
     */
    uint32_t rr_last[2];
    int64_t slot_start[2];

    /* The peripherals to tell of data or room, after the engine's run. */
    uint32_t notify;

    /*
     * The transaction each arbiter granted that a memory controller's AXI
     * port holds (esp32p4_mspi_dma_wait()), as the arbiter's member, or
     * -1: the arbiter's bus waits, and makes it once the port takes it,
     * which @bus_wait is told of.
     */
    int32_t stall_grant[2];
    Notifier bus_wait;

    const ESP32P4GdmaPeriphOps *periph_ops[ESP32P4_GDMA_PERIPHS];
    void *periph_opaque[ESP32P4_GDMA_PERIPHS];
    /* The interposers on each path, by direction and peripheral. */
    const ESP32P4GdmaInterposerOps *ip_ops[2][ESP32P4_GDMA_PERIPHS];
    void *ip_opaque[2][ESP32P4_GDMA_PERIPHS];

    /* The qtest hook's last result. */
    char *periph_probe;
};

struct ESP32P4GdmaClass {
    SysBusDeviceClass parent_class;

    bool axi;
};

#endif
