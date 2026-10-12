/*
 * ESP32-P4 2D-DMA controller (DMA2D)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_DMA_ESP32P4_DMA2D_H
#define HW_DMA_ESP32P4_DMA2D_H

#include "hw/core/sysbus.h"
#include "hw/core/clock.h"
#include "hw/misc/esp32p4_clk_period.h"
#include "qemu/timer.h"
#include "qom/object.h"
#include "system/memory.h"

#define TYPE_ESP32P4_DMA2D "esp32p4-dma2d"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4Dma2dState, ESP32P4_DMA2D)

#define ESP32P4_DMA2D_BASE      0x50088000
#define ESP32P4_DMA2D_SIZE      0x1000

/* Four transmit (memory to peripheral) and three receive channels. */
#define ESP32P4_DMA2D_TX_CHANNELS   4
#define ESP32P4_DMA2D_RX_CHANNELS   3
#define ESP32P4_DMA2D_CHANNELS      4
#define ESP32P4_DMA2D_IN            0
#define ESP32P4_DMA2D_OUT           1

/*
 * The values of DMA2D_OUT_PERI_SEL_CHn and DMA2D_IN_PERI_SEL_CHn (TRM
 * tables 8.4-2 and 8.4-3): a peripheral, or a dummy for memory-to-memory
 * transfers (TX 4 to 7, RX 3 to 7); 7, the reset value, is also what
 * ESP-IDF writes to connect a channel to nothing.
 */
#define ESP32P4_DMA2D_PERIPHS       8
#define ESP32P4_DMA2D_PERI_NONE     7

enum {
    ESP32P4_DMA2D_TX_JPEG = 0,
    ESP32P4_DMA2D_TX_PPA_SRM = 1,
    ESP32P4_DMA2D_TX_PPA_BLEND_BG = 2,
    ESP32P4_DMA2D_TX_PPA_BLEND_FG = 3,
};

enum {
    ESP32P4_DMA2D_RX_JPEG = 0,
    ESP32P4_DMA2D_RX_PPA_SRM = 1,
    ESP32P4_DMA2D_RX_PPA_BLEND = 2,
};

/*
 * How a peripheral moves data through the 2D-DMA
 * ==============================================
 *
 * A peripheral served by the 2D-DMA (the JPEG codec, the PPA's SRM and
 * BLEND engines) takes a QOM link to the controller and is known to it,
 * in each direction, by its selection value above. Software connects a
 * channel to it by writing that value to the channel's PERI_SEL.
 *
 *   - At realize, the peripheral attaches each of its paths with
 *     esp32p4_dma2d_attach(s, dir, sel, ops, opaque).
 *
 *   - What a path carries is a byte stream, which the channel serving it
 *     maps to memory by its descriptors (TRM 8.4.1): a buffer (1D mode),
 *     one hb x vb block of an HA x VA picture (2D-MOD0), or the picture
 *     in hb x vb blocks (2D-MOD1, in macroblock order where reordering is
 *     on), converted on the way where the channel's color space
 *     conversion is set, and padded where 2D-MOD1 or DSCR-PORT pad.
 *     esp32p4_dma2d_desc() gives the peripheral the descriptor its next
 *     bytes belong to, once the channel serving it has fetched it: on the
 *     transmit side the one the FIFO's oldest byte came from (or, with
 *     the FIFO holding none, the one being read), so that a peripheral
 *     started after the channel has run ahead still sees its block's
 *     dimensions; on the receive side the one being filled.
 *
 *   - Transmit: the peripheral takes the bytes with esp32p4_dma2d_pull(),
 *     which returns how many it got (0 while the FIFO is empty or no
 *     running channel selects the peripheral) and stops after a
 *     descriptor's last byte when its eof is set, reporting that in *eof.
 *     Taking the bytes is what completes the descriptors (OUT_DONE, and
 *     OUT_EOF and OUT_TOTAL_EOF as OUT_EOF_MODE says). ops->tx_ready is
 *     called when bytes arrive in an empty FIFO and when the channel has
 *     fetched a descriptor.
 *
 *   - Receive: the peripheral gives its bytes with esp32p4_dma2d_push(),
 *     which takes as many as the FIFO has room for and returns that
 *     count; ESP32P4_DMA2D_EOF on the last byte given ends the frame, which
 *     closes the descriptor with suc_eof (IN_SUC_EOF); ESP32P4_DMA2D_ERR_EOF
 *     ends it in error (err_eof, IN_ERR_EOF: the JPEG decoder only). A
 *     descriptor whose block or buffer fills closes without the flag and
 *     the stream goes on in the next. ops->rx_ready is called when room
 *     appears in a full FIFO, when the channel starts and when it has
 *     fetched a descriptor.
 *
 *   - DSCR-PORT (TRM 39.5.2.1): a channel with *_DSCR_PORT_EN set takes the
 *     picture and the image block from its descriptor, and the pixel
 *     blocks within it from the peripheral. On the transmit side
 *     esp32p4_dma2d_port_tx() asks for the next pixel block, of the size
 *     DMA2D_OUT_DSCR_PORT_BLK_H/V set (esp32p4_dma2d_port_block()), with
 *     its top-left pixel at (x, y) of the image block (x and y may be
 *     negative): the channel reads it and pads what lies outside the
 *     image block, as for 2D-MOD1, with the image block's edge. On the
 *     receive side esp32p4_dma2d_port_rx() says where the next w x h
 *     pixels pushed go, at (x, y) of the image block, and how many
 *     nibbles a pixel has there. Each queues up to two blocks and returns
 *     false when the queue is full; the last block of the descriptor has
 *     @last set.
 *
 *   - A peripheral that times its own processing (the PPA's engines at
 *     their clock) does so on the controller's timers,
 *     esp32p4_dma2d_timer_new(), _mod() and _del(): the controller runs
 *     them in order with its own bus rounds, each at its due time, so
 *     that the data moving between them is seen as it was at that time
 *     whichever runs late. A register access at the peripheral calls
 *     esp32p4_dma2d_sync(s, now) first, which runs everything due by then.
 *
 * The channels read and write memory at the controller's bus clock
 * (DMA2D_SYS_CLK), through the access permission managers as the 2D-DMA
 * master of the peripheral they serve (DMA2D_JPEG, DMA2D_PPA, or
 * DMA2D_DUMMY for memory-to-memory transfers).
 *
 * The JPEG decoder's output (TRM figure 8.4-4): a receive channel in
 * 2D-MOD1 with a macroblock size takes its stream as MCUs of 8 x 8
 * component blocks, 64 bytes each, row by row: GRAY one block; YUV444 Y,
 * U, V; YUV422 two Y blocks (left, right), U, V; YUV420 four Y blocks
 * (top-left, top-right, bottom-left, bottom-right), U, V. The decoder
 * says which with esp32p4_dma2d_rx_mcu_format() before it pushes; until
 * then the macroblock size implies it (8 x 8 YUV444, 16 x 8 YUV422,
 * 16 x 16 YUV420).
 */
#define ESP32P4_DMA2D_EOF       (1u << 0)
#define ESP32P4_DMA2D_ERR_EOF   (1u << 1)

enum {
    ESP32P4_DMA2D_MCU_GRAY,
    ESP32P4_DMA2D_MCU_YUV444,
    ESP32P4_DMA2D_MCU_YUV422,
    ESP32P4_DMA2D_MCU_YUV420,
    ESP32P4_DMA2D_MCU_AUTO,
};

typedef struct ESP32P4Dma2dPeriphOps {
    void (*tx_ready)(void *opaque, unsigned sel);
    void (*rx_ready)(void *opaque, unsigned sel);
    void (*sync)(void *opaque, int64_t when);
} ESP32P4Dma2dPeriphOps;

/* A descriptor as the channel holds it (TRM table 8.4-1). */
typedef struct ESP32P4Dma2dDesc {
    uint32_t addr;
    bool owner;
    bool eof;
    bool en_2d;
    bool mod1;
    /* 2D: in pixels. */
    uint32_t hb;
    uint32_t vb;
    uint32_t ha;
    uint32_t va;
    uint32_t x;
    uint32_t y;
    /* The pbyte field; esp32p4_dma2d_pbyte_nibbles() gives its size. */
    uint32_t pbyte;
    /* 1D: in bytes. */
    uint32_t length;
    uint32_t size;
    uint32_t buf;
    uint32_t next;
} ESP32P4Dma2dDesc;

/* The nibbles of a pixel of the pbyte field's value @pbyte; 0 if none. */
static inline unsigned esp32p4_dma2d_pbyte_nibbles(uint32_t pbyte)
{
    static const uint8_t n[6] = { 1, 2, 3, 4, 6, 8 };

    return pbyte < 6 ? n[pbyte] : 0;
}

void esp32p4_dma2d_attach(ESP32P4Dma2dState *s, int dir, unsigned sel,
                          const ESP32P4Dma2dPeriphOps *ops, void *opaque);
bool esp32p4_dma2d_desc(ESP32P4Dma2dState *s, int dir, unsigned sel,
                        ESP32P4Dma2dDesc *d);
size_t esp32p4_dma2d_pull(ESP32P4Dma2dState *s, unsigned sel, uint8_t *buf,
                          size_t len, bool *eof);
size_t esp32p4_dma2d_push(ESP32P4Dma2dState *s, unsigned sel,
                          const uint8_t *buf, size_t len, unsigned flags);
size_t esp32p4_dma2d_tx_pending(ESP32P4Dma2dState *s, unsigned sel);
size_t esp32p4_dma2d_rx_room(ESP32P4Dma2dState *s, unsigned sel);
void esp32p4_dma2d_port_block(ESP32P4Dma2dState *s, unsigned sel,
                              uint32_t *w, uint32_t *h);
bool esp32p4_dma2d_port_tx(ESP32P4Dma2dState *s, unsigned sel, int32_t x,
                           int32_t y, bool last);
bool esp32p4_dma2d_port_rx(ESP32P4Dma2dState *s, unsigned sel, uint32_t x,
                           uint32_t y, uint32_t w, uint32_t h,
                           unsigned nibbles, bool last);
void esp32p4_dma2d_rx_mcu_format(ESP32P4Dma2dState *s, unsigned sel,
                                 int format);

/* The controller's timers for the peripherals' own processing. */
#define ESP32P4_DMA2D_TIMERS    8
typedef void ESP32P4Dma2dTimerFn(void *opaque);
int esp32p4_dma2d_timer_new(ESP32P4Dma2dState *s, ESP32P4Dma2dTimerFn *fn,
                            void *opaque);
void esp32p4_dma2d_timer_mod(ESP32P4Dma2dState *s, int t, int64_t when);
void esp32p4_dma2d_timer_del(ESP32P4Dma2dState *s, int t);
void esp32p4_dma2d_sync(ESP32P4Dma2dState *s, int64_t when);

/*
 * Named GPIOs.
 *
 * ESP32P4_DMA2D_ETM_EVENT, "etm-event" (output, 13 lines): the ETM events
 *   in the order of ESP-IDF's soc_etm_source.h, DMA2D_EVT_IN_DONE_CH0 +
 *   line: IN_DONE_CH0..1, IN_SUC_EOF_CH0..1, OUT_DONE_CH0..2,
 *   OUT_EOF_CH0..2, OUT_TOTAL_EOF_CH0..2; each pulsed as it occurs.
 * ESP32P4_DMA2D_ETM_TASK, "etm-task" (input, 10 lines): DMA2D_TASK_IN_
 *   START_CH0 + line: IN_START_CH0..1, IN_DSCR_READY_CH0..1,
 *   OUT_START_CH0..2, OUT_DSCR_READY_CH0..2, acting on a rising edge.
 *   The channels the P4's ETM has no IDs for (RX 2, TX 3) take none.
 *
 * The interrupts are the sysbus IRQs IN_CH0..2 then OUT_CH0..3, the
 * interrupt matrix's DMA2D_IN_CHn and DMA2D_OUT_CHn sources.
 */
#define ESP32P4_DMA2D_ETM_EVENT     "etm-event"
#define ESP32P4_DMA2D_ETM_TASK      "etm-task"
#define ESP32P4_DMA2D_ETM_EVENTS    13
#define ESP32P4_DMA2D_ETM_TASKS     10
#define ESP32P4_DMA2D_IRQS  (ESP32P4_DMA2D_RX_CHANNELS + \
                             ESP32P4_DMA2D_TX_CHANNELS)

/* The FIFOs as the model has them, in bytes. */
#define ESP32P4_DMA2D_FIFO_MAX      512
/* The receive reorder buffer: one block of macroblocks. */
#define ESP32P4_DMA2D_RO_MAX        (64 * 1024)
/* RX: the bytes converted for the bus a channel holds, a burst's worth. */
#define ESP32P4_DMA2D_STAGE         256
/* The port blocks a channel queues. */
#define ESP32P4_DMA2D_PORT_QUEUE    2
/*
 * TX: the descriptors a channel keeps for its peripheral, the one being
 * read and those with bytes still in the FIFO; it fetches no further
 * while they are this many.
 */
#define ESP32P4_DMA2D_DESC_QUEUE    8

typedef struct ESP32P4Dma2dFifo {
    uint8_t data[ESP32P4_DMA2D_FIFO_MAX];
    uint8_t flags[ESP32P4_DMA2D_FIFO_MAX];
    uint32_t desc[ESP32P4_DMA2D_FIFO_MAX];
    uint32_t head;
    uint32_t num;
} ESP32P4Dma2dFifo;

typedef struct ESP32P4Dma2dPortBlock {
    int32_t x;
    int32_t y;
    uint32_t w;
    uint32_t h;
    uint32_t nibbles;
    uint32_t last;
} ESP32P4Dma2dPortBlock;

typedef struct ESP32P4Dma2dChan {
    /* Registers. */
    uint32_t conf0;
    uint32_t int_raw;
    uint32_t int_ena;
    uint32_t link;          /* RX: INLINK_AUTO_RET */
    uint32_t link_addr;
    uint32_t peri_sel;
    uint32_t arb;
    uint32_t ro_pd_conf;
    uint32_t color_convert;
    uint32_t scramble;
    uint32_t color_param[6];
    uint32_t etm_conf;
    uint32_t port_blk;
    uint32_t pushpop;       /* OUT_PUSH's WDATA; IN_POP's RDATA */
    uint32_t eof_des_addr;
    uint32_t err_eof_des_addr;
    uint32_t dscr;
    uint32_t dscr_bf0;      /* also the descriptor the channel holds */
    uint32_t dscr_bf1;

    /* The link FSM and the descriptor it holds. */
    bool active;
    bool has_desc;
    bool end;               /* the list's last descriptor is done */
    bool restart;           /* a RESTART to take at the list's end */
    bool stop;              /* a STOP to take after this descriptor */
    bool wait_ready;        /* ETM loop: waiting for DSCR_READY */
    bool empty_raised;
    uint32_t tasks;         /* DSCR_READY tasks kept */
    uint32_t prev_desc;
    uint32_t desc_next;
    uint32_t dw[5];

    /*
     * Where the transfer is in its descriptor: 1D, the byte; 2D, the
     * block (2D-MOD1: its column and row in the picture), the macroblock
     * within it, the row, and the part of the row (left padding, data,
     * right padding) with the bytes done of it.
     */
    uint32_t pos;
    uint32_t blk_x;
    uint32_t blk_y;
    uint32_t mb;
    uint32_t row;
    uint32_t phase;
    uint32_t rpos;
    /* RX DSCR-PORT: the extent the pixel blocks reached. */
    uint32_t port_w;
    uint32_t port_h;

    /*
     * The stream's conversion: a partial pixel, a pixel waiting for its
     * pair, and the row for YUV420's parity.
     */
    uint8_t carry[8];
    uint32_t ncarry;
    uint32_t csc_pend;
    uint32_t csc_row;
    /* RX: converted bytes waiting for the bus, and the frame's end. */
    uint8_t stage[ESP32P4_DMA2D_STAGE];
    uint32_t nstage;
    uint8_t stage_flags;

    ESP32P4Dma2dPortBlock port[ESP32P4_DMA2D_PORT_QUEUE];
    uint32_t nport;

    uint32_t tokens;
    ESP32P4Dma2dFifo fifo;
    /*
     * TX: the descriptors from the one the FIFO's oldest byte came from to
     * the one being read, oldest first: each its address and five words.
     */
    uint32_t dq[ESP32P4_DMA2D_DESC_QUEUE][6];
    uint32_t dq_head;
    uint32_t dq_num;
} ESP32P4Dma2dChan;

struct ESP32P4Dma2dState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq[ESP32P4_DMA2D_IRQS];
    qemu_irq etm_event[ESP32P4_DMA2D_ETM_EVENTS];
    Clock *clk;
    QEMUTimer *timer;
    /*
     * The bus clock's cycles, counted from its last change, and the round
     * the engine runs next, at cycle @due_cycle, due at @due_ns.
     */
    ESP32P4ClkCount bus_edges;
    bool round_due;
    int64_t due_cycle;
    int64_t due_ns;
    AddressSpace *as;
    Object *pms;

    ESP32P4Dma2dChan ch[2][ESP32P4_DMA2D_CHANNELS];

    uint32_t axi_err;
    uint32_t rst_conf;
    uint32_t intr_mem_start;
    uint32_t intr_mem_end;
    uint32_t extr_mem_start;
    uint32_t extr_mem_end;
    uint32_t arb_config[2];
    uint32_t rdn_result;
    uint32_t rdn_eco_high;
    uint32_t rdn_eco_low;
    uint32_t date;
    bool etm_task_level[ESP32P4_DMA2D_ETM_TASKS];

    /* The arbiters: the last channel granted and their slots' starts. */
    uint32_t rr_last[2];
    int64_t slot_start[2];
    /* The peripherals to tell of data, room or a descriptor. */
    uint32_t notify;

    /*
     * RX channel 0's reorder RAM: the MCUs of a block coming in, and the
     * block's pixels going out.
     */
    uint8_t *ro_buf;
    uint8_t *ro_out;
    uint32_t ro_buf_size;
    uint32_t ro_num;
    uint32_t ro_out_len;
    int32_t mcu_format;

    /* The peripherals' timers: their due times, -1 when not armed. */
    int64_t ptimer_due[ESP32P4_DMA2D_TIMERS];
    ESP32P4Dma2dTimerFn *ptimer_fn[ESP32P4_DMA2D_TIMERS];
    void *ptimer_opaque[ESP32P4_DMA2D_TIMERS];
    int nptimers;
    bool running;

    const ESP32P4Dma2dPeriphOps *periph_ops[2][ESP32P4_DMA2D_PERIPHS];
    void *periph_opaque[2][ESP32P4_DMA2D_PERIPHS];

    /* The qtest hook's last result. */
    char *periph_probe;
};

#endif
