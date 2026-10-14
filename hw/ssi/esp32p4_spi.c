/*
 * ESP32-P4 general-purpose SPI controllers, GP-SPI2 and GP-SPI3
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapter
 * 46 (SPI Controller), with the layout, access types and reset values of
 * ESP-IDF's spi_reg.h and spi_struct.h for v3, and its spi_ll.h, spi_hal
 * and esp_driver_spi (master, slave and slave half-duplex) for how
 * software drives the controllers. Where those disagree the model follows
 * spi_reg.h and the TRM's register descriptions (SPI_DATE resets to
 * 0x2207202, as both say; spi_struct.h has 37761424).
 *
 * GP-SPI2 has six chip selects, eight data lanes, DTR and the configurable
 * segmented transfer; GP-SPI3 has three chip selects and four lanes. The
 * fields the other lacks read as zero and ignore writes.
 *
 * Master. Setting SPI_USR starts a transaction through the states of TRM
 * figure 46.5-5: PREP (CS setup), CMD, ADDR, DUMMY, DOUT and DIN (or, with
 * SPI_DOUTDIN, the full-duplex data state, both at once) and DONE (CS
 * hold), each as enabled, with its own lanes (1, 2, 4 or 8; SPI_QPI_MODE
 * and SPI_OPI_MODE put every state on 4 or 8) and length. A transaction
 * takes the time its SPI_CLK cycles take: clk_spi_mst ("mst"), divided by
 * (SPI_CLKDIV_PRE + 1)(SPI_CLKCNT_N + 1) unless SPI_CLK_EQU_SYSCLK, with
 * CS setup and hold of (SPI_CS_SETUP_TIME + 1.5) and (SPI_CS_HOLD_TIME +
 * 1.5) cycles when enabled and half a cycle otherwise (TRM 46.6). DTR moves
 * two bits per lane in each cycle of an output clock halved for it (as
 * ESP-IDF configures it), so a state takes the same time either way. At
 * its end SPI_USR clears and SPI_TRANS_DONE_INT rises. The half cycles
 * are counted at clk_spi_mst's exact period (esp32p4_clk_period.h) from
 * the transaction's start, each step at the first nanosecond its time
 * has come; the segments of a segmented transfer at one clock follow
 * each other on the same count.
 *
 * The master works from its own copy of its configuration registers
 * (SPI_ADDR, _CTRL, _CLOCK, _USER, _USER1, _USER2, _MS_DLEN, _MISC, the
 * timing registers and SPI_CONF_BITLEN), which SPI_UPDATE copies across
 * from the bus side two clk_spi_mst cycles after it is set (it reads 1
 * until then): software that does not set it runs with the configuration
 * it last synchronised, as on the chip (TRM 46.5.9.2's note).
 *
 * Bit order (TRM 46.5.3, table 46.5-6): a byte is split into chunks of as
 * many bits as there are lanes, sent high chunk first unless the order's
 * bit 0 is set; within a chunk lane n carries the chunk's bit n, or bit
 * (lanes - 1 - n) with the order's bit 1, except that in 8-bit mode it is
 * bit 0 that reverses the lanes. The command is the bytes
 * SPI_USR_COMMAND_VALUE[7:0] then [15:8], the address SPI_ADDR[31:24]
 * down to [7:0], each sent so (tables 46.5-11 and 46.5-12), and the
 * states send only their first bit length's chunks. The CPU buffer is
 * byte-addressed, SPI_W0[7:0] first; past its 64 bytes (32 with a high
 * part) a transfer repeats SPI_W15[31:24] until byte 256, and starts over
 * (TRM 46.5.6.1). A last byte of fewer than eight bits is stored with its
 * bits where a whole byte's would be.
 *
 * DMA. With SPI_DMA_TX_ENA, data out comes from GDMA-AXI (the channel whose
 * PERI_SEL selects the controller) through the controller's TX FIFO, which
 * takes what the DMA gives while the bit is set; with SPI_DMA_RX_ENA, data
 * in goes to it through the RX FIFO. A transfer waits while the DMA is
 * still moving data and has none yet, or no room yet: the SPI_CLK cycles
 * pause, which only the model's own timing makes happen, as the DMA is
 * faster than the bus. When the DMA has no more to give (its list ended or
 * it is stopped) the FIFO under- or overflows: a master repeats the last
 * byte it sent, a slave sends zeros, received data is lost, and
 * SPI_DMA_OUTFIFO_EMPTY_ERR_INT or _INFIFO_FULL_ERR_INT rises (with the
 * master's SPI_MST_TX_AFIFO_REMPTY_ERR_INT or _RX_AFIFO_WFULL_ERR_INT,
 * which end the transfer under SPI_MST_REMPTY_ERR_END_EN and
 * _WFULL_ERR_END_EN). With SPI_MST_FD_WAIT_DMA_TX_DATA a transfer with a
 * DMA data-out state starts once the DMA has given data. The received data
 * ends its DMA frame (suc_eof) at the end of the transfer that received it
 * (TRM table 46.5-9), or with SPI_RX_EOF_EN once SPI_MS_DATA_BITLEN + 1
 * bits have gone to the DMA.
 *
 * Configurable segmented transfer (GP-SPI2, TRM 46.5.9.5): with
 * SPI_USR_CONF, each segment starts with the CONF state, CS inactive for
 * SPI_CONF_BITLEN + 1 cycles, in which the controller takes from its TX DMA
 * a bitmap word, whose bits 31:28 must match SPI_DMA_SEG_MAGIC_VALUE, and a
 * word for each register its bits 0 to 13 name (table 46.5-13), which it
 * writes; a segment whose SPI_USR_CONF_NXT is set is followed by another.
 * A bad magic value ends the transfer with SPI_SEG_MAGIC_ERR_INT; the end
 * raises SPI_DMA_SEG_TRANS_DONE_INT, and SPI_TRANS_DONE_INT ends each
 * segment.
 *
 * Slave. With SPI_SLAVE_MODE the controller answers a master that clocks
 * it while its CS0 input is active (inverted by SPI_SLAVE_CS_POL), in full
 * duplex (SPI_DOUTDIN: data in on D and out on Q, from and to the CPU
 * buffer from byte 0 or the DMA) or half duplex (TRM 46.5.10): a command
 * byte (in 1-bit mode, or 4-bit in QPI mode), an address byte and eight
 * dummy cycles, then data in the lanes the command's bits 7:4 give; Wr_BUF
 * and Rd_BUF move data to and from the CPU buffer at the address, Wr_DMA
 * and Rd_DMA through the DMA, CMD7 to CMDA raise their interrupts, End_SEG
 * ends a slave segmented transfer (SPI_DMA_SLV_SEG_TRANS_EN), En_QPI and
 * Ex_QPI set and clear SPI_QPI_MODE; any other command raises
 * SPI_SLV_CMD_ERR_INT and the rest of its transfer is ignored. When CS
 * goes inactive the slave records the command, address and data bit length
 * (as SPI_SLV_*_BITLEN_EN select), raises its interrupts and
 * SPI_TRANS_DONE_INT and clears SPI_USR. A slave needs no SPI_UPDATE.
 *
 * The bus is modelled a clock cycle at a time, between the two
 * controllers and towards the board's QEMU SSI devices a byte of the
 * stream of bits at a time (include/hw/ssi/esp32p4_spi.h): the routing
 * through the GPIO matrix and IO MUX decides who hears whom, including a
 * controller hearing its own output. The device's bit stream carries each
 * cycle's lanes, the highest lane first, and a dummy cycle as many bits as
 * the address (or command) state has lanes, as QEMU's flash models count
 * dummy cycles. The clock and data lines are driven at their idle levels
 * (SPI_CK_IDLE_EDGE, SPI_D_POL, ...) and their output enables follow the
 * states, but their levels do not toggle with each bit; the chip selects
 * do, at the states' times.
 *
 * Not modelled: the clock phase and polarity (SPI_CK_OUT_EDGE,
 * SPI_TSCK_I_EDGE, SPI_RSCK_I_EDGE, SPI_CLK_MODE_13): both ends of a
 * transfer see the same bits whatever their modes; the timing compensation
 * (SPI_DIN_MODE, _DIN_NUM, _DOUT_MODE, SPI_CLK_EDGE_SEL, SPI_RSCK_DATA_OUT),
 * kept as written; the data strobe (DQS) and SPI_CLK_MODE; the matrix's
 * inversion of an input from the board's devices; the slave segmented
 * transfer's SPI_SLV_TX_SEG_TRANS_CLR_EN (a slave out of DMA data sends
 * zeros). SPI_CLK_GATE is kept as written: the P4's controller takes its
 * clocks from HP_SYS_CLKRST, and ESP-IDF never sets it.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/bitops.h"
#include "qemu/host-utils.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/registerfields.h"
#include "hw/gpio/esp32_line.h"
#include "hw/gpio/esp32p4_gpio_sig.h"
#include "hw/ssi/esp32p4_spi.h"
#include "migration/vmstate.h"

REG32(CMD, 0x00)
    FIELD(CMD, CONF_BITLEN, 0, 18)
    FIELD(CMD, UPDATE, 23, 1)
    FIELD(CMD, USR, 24, 1)
REG32(ADDR, 0x04)
REG32(CTRL, 0x08)
    FIELD(CTRL, DUMMY_OUT, 3, 1)
    FIELD(CTRL, FADDR_DUAL, 5, 1)
    FIELD(CTRL, FADDR_QUAD, 6, 1)
    FIELD(CTRL, FADDR_OCT, 7, 1)
    FIELD(CTRL, FCMD_DUAL, 8, 1)
    FIELD(CTRL, FCMD_QUAD, 9, 1)
    FIELD(CTRL, FCMD_OCT, 10, 1)
    FIELD(CTRL, FREAD_DUAL, 14, 1)
    FIELD(CTRL, FREAD_QUAD, 15, 1)
    FIELD(CTRL, FREAD_OCT, 16, 1)
    FIELD(CTRL, Q_POL, 18, 1)
    FIELD(CTRL, D_POL, 19, 1)
    FIELD(CTRL, HOLD_POL, 20, 1)
    FIELD(CTRL, WP_POL, 21, 1)
    FIELD(CTRL, RD_BIT_ORDER, 23, 2)
    FIELD(CTRL, WR_BIT_ORDER, 25, 2)
REG32(CLOCK, 0x0c)
    FIELD(CLOCK, CLKCNT_L, 0, 6)
    FIELD(CLOCK, CLKCNT_H, 6, 6)
    FIELD(CLOCK, CLKCNT_N, 12, 6)
    FIELD(CLOCK, CLKDIV_PRE, 18, 4)
    FIELD(CLOCK, CLK_EDGE_SEL, 30, 1)
    FIELD(CLOCK, CLK_EQU_SYSCLK, 31, 1)
REG32(USER, 0x10)
    FIELD(USER, DOUTDIN, 0, 1)
    FIELD(USER, QPI_MODE, 3, 1)
    FIELD(USER, OPI_MODE, 4, 1)
    FIELD(USER, TSCK_I_EDGE, 5, 1)
    FIELD(USER, CS_HOLD, 6, 1)
    FIELD(USER, CS_SETUP, 7, 1)
    FIELD(USER, RSCK_I_EDGE, 8, 1)
    FIELD(USER, CK_OUT_EDGE, 9, 1)
    FIELD(USER, FWRITE_DUAL, 12, 1)
    FIELD(USER, FWRITE_QUAD, 13, 1)
    FIELD(USER, FWRITE_OCT, 14, 1)
    FIELD(USER, USR_CONF_NXT, 15, 1)
    FIELD(USER, SIO, 17, 1)
    FIELD(USER, USR_MISO_HIGHPART, 24, 1)
    FIELD(USER, USR_MOSI_HIGHPART, 25, 1)
    FIELD(USER, USR_DUMMY_IDLE, 26, 1)
    FIELD(USER, USR_MOSI, 27, 1)
    FIELD(USER, USR_MISO, 28, 1)
    FIELD(USER, USR_DUMMY, 29, 1)
    FIELD(USER, USR_ADDR, 30, 1)
    FIELD(USER, USR_COMMAND, 31, 1)
REG32(USER1, 0x14)
    FIELD(USER1, USR_DUMMY_CYCLELEN, 0, 8)
    FIELD(USER1, MST_WFULL_ERR_END_EN, 16, 1)
    FIELD(USER1, CS_SETUP_TIME, 17, 5)
    FIELD(USER1, CS_HOLD_TIME, 22, 5)
    FIELD(USER1, USR_ADDR_BITLEN, 27, 5)
REG32(USER2, 0x18)
    FIELD(USER2, USR_COMMAND_VALUE, 0, 16)
    FIELD(USER2, MST_REMPTY_ERR_END_EN, 27, 1)
    FIELD(USER2, USR_COMMAND_BITLEN, 28, 4)
REG32(MS_DLEN, 0x1c)
    FIELD(MS_DLEN, MS_DATA_BITLEN, 0, 18)
REG32(MISC, 0x20)
    FIELD(MISC, CS_DIS, 0, 6)
    FIELD(MISC, CK_DIS, 6, 1)
    FIELD(MISC, MASTER_CS_POL, 7, 6)
    FIELD(MISC, CLK_DATA_DTR_EN, 16, 1)
    FIELD(MISC, DATA_DTR_EN, 17, 1)
    FIELD(MISC, ADDR_DTR_EN, 18, 1)
    FIELD(MISC, CMD_DTR_EN, 19, 1)
    FIELD(MISC, SLAVE_CS_POL, 23, 1)
    FIELD(MISC, DQS_IDLE_EDGE, 24, 1)
    FIELD(MISC, CK_IDLE_EDGE, 29, 1)
    FIELD(MISC, CS_KEEP_ACTIVE, 30, 1)
    FIELD(MISC, QUAD_DIN_PIN_SWAP, 31, 1)
REG32(DIN_MODE, 0x24)
REG32(DIN_NUM, 0x28)
REG32(DOUT_MODE, 0x2c)
REG32(DMA_CONF, 0x30)
    FIELD(DMA_CONF, DMA_OUTFIFO_EMPTY, 0, 1)
    FIELD(DMA_CONF, DMA_INFIFO_FULL, 1, 1)
    FIELD(DMA_CONF, DMA_SLV_SEG_TRANS_EN, 18, 1)
    FIELD(DMA_CONF, SLV_RX_SEG_TRANS_CLR_EN, 19, 1)
    FIELD(DMA_CONF, SLV_TX_SEG_TRANS_CLR_EN, 20, 1)
    FIELD(DMA_CONF, RX_EOF_EN, 21, 1)
    FIELD(DMA_CONF, DMA_RX_ENA, 27, 1)
    FIELD(DMA_CONF, DMA_TX_ENA, 28, 1)
    FIELD(DMA_CONF, RX_AFIFO_RST, 29, 1)
    FIELD(DMA_CONF, BUF_AFIFO_RST, 30, 1)
    FIELD(DMA_CONF, DMA_AFIFO_RST, 31, 1)
REG32(DMA_INT_ENA, 0x34)
REG32(DMA_INT_CLR, 0x38)
REG32(DMA_INT_RAW, 0x3c)
REG32(DMA_INT_ST, 0x40)
REG32(DMA_INT_SET, 0x44)
REG32(W0, 0x98)
REG32(W15, 0xd4)
REG32(SLAVE, 0xe0)
    FIELD(SLAVE, CLK_MODE, 0, 2)
    FIELD(SLAVE, CLK_MODE_13, 2, 1)
    FIELD(SLAVE, RSCK_DATA_OUT, 3, 1)
    FIELD(SLAVE, SLV_RDDMA_BITLEN_EN, 8, 1)
    FIELD(SLAVE, SLV_WRDMA_BITLEN_EN, 9, 1)
    FIELD(SLAVE, SLV_RDBUF_BITLEN_EN, 10, 1)
    FIELD(SLAVE, SLV_WRBUF_BITLEN_EN, 11, 1)
    FIELD(SLAVE, SLV_LAST_BYTE_STRB, 12, 8)
    FIELD(SLAVE, DMA_SEG_MAGIC_VALUE, 22, 4)
    FIELD(SLAVE, SLAVE_MODE, 26, 1)
    FIELD(SLAVE, SOFT_RESET, 27, 1)
    FIELD(SLAVE, USR_CONF, 28, 1)
    FIELD(SLAVE, MST_FD_WAIT_DMA_TX_DATA, 29, 1)
REG32(SLAVE1, 0xe4)
    FIELD(SLAVE1, SLV_DATA_BITLEN, 0, 18)
    FIELD(SLAVE1, SLV_LAST_COMMAND, 18, 8)
    FIELD(SLAVE1, SLV_LAST_ADDR, 26, 6)
REG32(CLK_GATE, 0xe8)
REG32(DATE, 0xf0)

/* SPI_DMA_INT_* bits. */
#define INT_DMA_INFIFO_FULL_ERR     BIT(0)
#define INT_DMA_OUTFIFO_EMPTY_ERR   BIT(1)
#define INT_SLV_EX_QPI              BIT(2)
#define INT_SLV_EN_QPI              BIT(3)
#define INT_SLV_CMD7                BIT(4)
#define INT_SLV_CMD8                BIT(5)
#define INT_SLV_CMD9                BIT(6)
#define INT_SLV_CMDA                BIT(7)
#define INT_SLV_RD_DMA_DONE         BIT(8)
#define INT_SLV_WR_DMA_DONE         BIT(9)
#define INT_SLV_RD_BUF_DONE         BIT(10)
#define INT_SLV_WR_BUF_DONE         BIT(11)
#define INT_TRANS_DONE              BIT(12)
#define INT_DMA_SEG_TRANS_DONE      BIT(13)
#define INT_SEG_MAGIC_ERR           BIT(14)
#define INT_SLV_BUF_ADDR_ERR        BIT(15)
#define INT_SLV_CMD_ERR             BIT(16)
#define INT_MST_RX_AFIFO_WFULL_ERR  BIT(17)
#define INT_MST_TX_AFIFO_REMPTY_ERR BIT(18)
#define INT_ALL                     0x1fffff

/* Reset values (spi_reg.h). */
#define CTRL_RESET      0x003c0000
#define CLOCK_RESET     0x80003043
#define USER_RESET      0x800000c0
#define USER1_RESET     0xb8410007
#define USER2_RESET     0x78000000
#define DMA_CONF_RESET  0x00000003
#define SLAVE_MAGIC_RESET (10u << R_SLAVE_DMA_SEG_MAGIC_VALUE_SHIFT)
#define DATE_RESET      0x2207202

#define DMA_CONF_RW     (R_DMA_CONF_DMA_SLV_SEG_TRANS_EN_MASK | \
                         R_DMA_CONF_SLV_RX_SEG_TRANS_CLR_EN_MASK | \
                         R_DMA_CONF_SLV_TX_SEG_TRANS_CLR_EN_MASK | \
                         R_DMA_CONF_RX_EOF_EN_MASK | \
                         R_DMA_CONF_DMA_RX_ENA_MASK | \
                         R_DMA_CONF_DMA_TX_ENA_MASK)

/* The CONF state's bitmap (TRM table 46.5-13). */
#define CONF_REGS       14
#define CONF_MAGIC(w)   ((w) >> 28)

/* SPI_CLK cycles a transfer runs between looks at the time. */
#define BATCH_BEATS     32

/* The master's states. */
enum {
    PH_PREP,
    PH_CMD,
    PH_ADDR,
    PH_DUMMY,
    PH_DOUT,
    PH_DIN,
    PH_DATA,        /* full duplex: out on D and in on Q at once */
    PH_DONE,
    PH_CONF,
};

/* Where a data input comes from. */
enum {
    SRC_CONST,      /* the level the matrix delivers, whatever drives it */
    SRC_SELF,       /* one of the controller's own data outputs */
    SRC_PEER,       /* one of the other controller's */
    SRC_EXT,        /* the board's SSI devices */
};

/* The slave's states. */
enum {
    S_IDLE,
    S_FD,
    S_CMD,
    S_ADDR,
    S_DUMMY,
    S_WR,
    S_RD,
    S_IGNORE,
};

/* The half-duplex slave commands (TRM tables 46.5-16 and 46.5-17). */
enum {
    K_NONE,
    K_WRBUF,
    K_RDBUF,
    K_WRDMA,
    K_RDDMA,
    K_CMD7,
    K_CMD8,
    K_CMD9,
    K_CMDA,
    K_SEGEND,
    K_ENQPI,
    K_EXQPI,
    K_ERR,
};

/* An RX FIFO entry's marks. */
#define F_EOF           BIT(0)
#define F_NODATA        BIT(1)

static void master_run(ESP32P4SpiState *s);
static void master_kick(ESP32P4SpiState *s);
static void master_resume(ESP32P4SpiState *s);
static void master_try_start(ESP32P4SpiState *s);
static void spi_drive_lines(ESP32P4SpiState *s);

/* Signals */

static const int16_t line_sig[2][ESP32P4_SPI_LINES] = {
    {
        ESP32P4_SIG_SPI2_D_PAD_OUT, ESP32P4_SIG_SPI2_Q_PAD_OUT,
        ESP32P4_SIG_SPI2_WP_PAD_OUT, ESP32P4_SIG_SPI2_HOLD_PAD_OUT,
        ESP32P4_SIG_SPI2_IO4_PAD_OUT, ESP32P4_SIG_SPI2_IO5_PAD_OUT,
        ESP32P4_SIG_SPI2_IO6_PAD_OUT, ESP32P4_SIG_SPI2_IO7_PAD_OUT,
        ESP32P4_SIG_SPI2_CK_PAD_OUT,
        ESP32P4_SIG_SPI2_CS_PAD_OUT, ESP32P4_SIG_SPI2_CS1_PAD_OUT,
        ESP32P4_SIG_SPI2_CS2_PAD_OUT, ESP32P4_SIG_SPI2_CS3_PAD_OUT,
        ESP32P4_SIG_SPI2_CS4_PAD_OUT, ESP32P4_SIG_SPI2_CS5_PAD_OUT,
        ESP32P4_SIG_SPI2_DQS_PAD_OUT,
    }, {
        ESP32P4_SIG_SPI3_D_PAD_OUT, ESP32P4_SIG_SPI3_QO_PAD_OUT,
        ESP32P4_SIG_SPI3_WP_PAD_OUT, ESP32P4_SIG_SPI3_HOLD_PAD_OUT,
        -1, -1, -1, -1,
        ESP32P4_SIG_SPI3_CK_PAD_OUT,
        ESP32P4_SIG_SPI3_CS_PAD_OUT, ESP32P4_SIG_SPI3_CS1_PAD_OUT,
        ESP32P4_SIG_SPI3_CS2_PAD_OUT, -1, -1, -1,
        -1,
    },
};

int esp32p4_spi_line_signal(unsigned host, unsigned line)
{
    if ((host != 2 && host != 3) || line >= ESP32P4_SPI_LINES) {
        return -1;
    }
    return line_sig[host - 2][line];
}

bool esp32p4_spi_line_has_input(unsigned host, unsigned line)
{
    return esp32p4_spi_line_signal(host, line) >= 0 &&
           (line < ESP32P4_SPI_LANES || line == ESP32P4_SPI_LINE_CK ||
            line == ESP32P4_SPI_LINE_CS0);
}

static bool spi2(ESP32P4SpiState *s)
{
    return s->host == 2;
}

static unsigned ncs(ESP32P4SpiState *s)
{
    return spi2(s) ? 6 : 3;
}

static unsigned nlanes(ESP32P4SpiState *s)
{
    return spi2(s) ? 8 : 4;
}

static int sig_of(ESP32P4SpiState *s, unsigned line)
{
    return esp32p4_spi_line_signal(s->host, line);
}

/* The data lane of @s whose output is matrix signal @sig, or -1. */
static int lane_of_signal(ESP32P4SpiState *s, unsigned sig)
{
    for (unsigned l = 0; l < nlanes(s); l++) {
        if (sig_of(s, l) == (int)sig) {
            return l;
        }
    }
    return -1;
}

/* Register masks, which differ between the two controllers. */

static uint32_t cmd_mask(ESP32P4SpiState *s)
{
    return R_CMD_USR_MASK | (spi2(s) ? R_CMD_CONF_BITLEN_MASK : 0);
}

static uint32_t ctrl_mask(ESP32P4SpiState *s)
{
    uint32_t m = R_CTRL_DUMMY_OUT_MASK | R_CTRL_FADDR_DUAL_MASK |
                 R_CTRL_FADDR_QUAD_MASK | R_CTRL_FCMD_DUAL_MASK |
                 R_CTRL_FCMD_QUAD_MASK | R_CTRL_FREAD_DUAL_MASK |
                 R_CTRL_FREAD_QUAD_MASK | R_CTRL_Q_POL_MASK |
                 R_CTRL_D_POL_MASK | R_CTRL_HOLD_POL_MASK |
                 R_CTRL_WP_POL_MASK | R_CTRL_RD_BIT_ORDER_MASK |
                 R_CTRL_WR_BIT_ORDER_MASK;

    if (spi2(s)) {
        m |= R_CTRL_FADDR_OCT_MASK | R_CTRL_FCMD_OCT_MASK |
             R_CTRL_FREAD_OCT_MASK;
    }
    return m;
}

static uint32_t clock_mask(void)
{
    return R_CLOCK_CLKCNT_L_MASK | R_CLOCK_CLKCNT_H_MASK |
           R_CLOCK_CLKCNT_N_MASK | R_CLOCK_CLKDIV_PRE_MASK |
           R_CLOCK_CLK_EDGE_SEL_MASK | R_CLOCK_CLK_EQU_SYSCLK_MASK;
}

static uint32_t user_mask(ESP32P4SpiState *s)
{
    uint32_t m = R_USER_DOUTDIN_MASK | R_USER_QPI_MODE_MASK |
                 R_USER_TSCK_I_EDGE_MASK | R_USER_CS_HOLD_MASK |
                 R_USER_CS_SETUP_MASK | R_USER_RSCK_I_EDGE_MASK |
                 R_USER_CK_OUT_EDGE_MASK | R_USER_FWRITE_DUAL_MASK |
                 R_USER_FWRITE_QUAD_MASK | R_USER_SIO_MASK | 0xff000000u;

    if (spi2(s)) {
        m |= R_USER_OPI_MODE_MASK | R_USER_FWRITE_OCT_MASK |
             R_USER_USR_CONF_NXT_MASK;
    }
    return m;
}

#define USER1_MASK      0xffff00ffu
#define USER2_MASK      0xf800ffffu

static uint32_t misc_mask(ESP32P4SpiState *s)
{
    uint32_t m = R_MISC_CK_DIS_MASK | R_MISC_SLAVE_CS_POL_MASK |
                 R_MISC_CK_IDLE_EDGE_MASK | R_MISC_CS_KEEP_ACTIVE_MASK |
                 R_MISC_QUAD_DIN_PIN_SWAP_MASK;

    if (spi2(s)) {
        m |= R_MISC_CS_DIS_MASK | R_MISC_MASTER_CS_POL_MASK |
             R_MISC_CLK_DATA_DTR_EN_MASK | R_MISC_DATA_DTR_EN_MASK |
             R_MISC_ADDR_DTR_EN_MASK | R_MISC_CMD_DTR_EN_MASK |
             R_MISC_DQS_IDLE_EDGE_MASK;
    } else {
        m |= 0x7u | (0x7u << R_MISC_MASTER_CS_POL_SHIFT);
    }
    return m;
}

static uint32_t misc_reset(ESP32P4SpiState *s)
{
    /* CS1 and up disabled. */
    return spi2(s) ? 0x3e : 0x06;
}

static uint32_t din_mode_mask(ESP32P4SpiState *s)
{
    return spi2(s) ? 0x1ffff : 0x100ff;
}

static uint32_t din_num_mask(ESP32P4SpiState *s)
{
    return spi2(s) ? 0xffff : 0xff;
}

static uint32_t dout_mode_mask(ESP32P4SpiState *s)
{
    return spi2(s) ? 0x1ff : 0xf;
}

static uint32_t int_mask(ESP32P4SpiState *s)
{
    return spi2(s) ? INT_ALL : INT_ALL & ~INT_SEG_MAGIC_ERR;
}

static uint32_t slave_mask(ESP32P4SpiState *s)
{
    uint32_t m = R_SLAVE_CLK_MODE_MASK | R_SLAVE_CLK_MODE_13_MASK |
                 R_SLAVE_RSCK_DATA_OUT_MASK |
                 R_SLAVE_SLV_RDDMA_BITLEN_EN_MASK |
                 R_SLAVE_SLV_WRDMA_BITLEN_EN_MASK |
                 R_SLAVE_SLV_RDBUF_BITLEN_EN_MASK |
                 R_SLAVE_SLV_WRBUF_BITLEN_EN_MASK |
                 R_SLAVE_SLAVE_MODE_MASK |
                 R_SLAVE_MST_FD_WAIT_DMA_TX_DATA_MASK;

    if (spi2(s)) {
        m |= R_SLAVE_DMA_SEG_MAGIC_VALUE_MASK | R_SLAVE_USR_CONF_MASK;
    }
    return m;
}

/* The bus side's value of each register the master copies. */
static uint32_t *mreg_src(ESP32P4SpiState *s, unsigned i)
{
    switch (i) {
    case ESP32P4_SPI_M_ADDR:
        return &s->addr;
    case ESP32P4_SPI_M_CTRL:
        return &s->ctrl;
    case ESP32P4_SPI_M_CLOCK:
        return &s->clock;
    case ESP32P4_SPI_M_USER:
        return &s->user;
    case ESP32P4_SPI_M_USER1:
        return &s->user1;
    case ESP32P4_SPI_M_USER2:
        return &s->user2;
    case ESP32P4_SPI_M_MS_DLEN:
        return &s->ms_dlen;
    case ESP32P4_SPI_M_MISC:
        return &s->misc;
    case ESP32P4_SPI_M_DIN_MODE:
        return &s->din_mode;
    case ESP32P4_SPI_M_DIN_NUM:
        return &s->din_num;
    case ESP32P4_SPI_M_DOUT_MODE:
        return &s->dout_mode;
    default:
        g_assert_not_reached();
    }
}

static bool slave_mode(ESP32P4SpiState *s)
{
    return s->slave & R_SLAVE_SLAVE_MODE_MASK;
}

/* Interrupts */

static void spi_update_irq(ESP32P4SpiState *s)
{
    qemu_set_irq(s->irq, (s->int_raw & s->int_ena) != 0);
}

static void spi_raise(ESP32P4SpiState *s, uint32_t bits)
{
    s->int_raw |= bits & int_mask(s);
    spi_update_irq(s);
}

/* FIFOs */

static void fifo_reset(ESP32P4SpiFifo *f)
{
    f->head = 0;
    f->num = 0;
}

static bool fifo_full(ESP32P4SpiFifo *f)
{
    return f->num == ESP32P4_SPI_AFIFO;
}

static void fifo_put(ESP32P4SpiFifo *f, uint8_t byte, uint8_t flags)
{
    uint32_t i = (f->head + f->num) % ESP32P4_SPI_AFIFO;

    assert(f->num < ESP32P4_SPI_AFIFO);
    f->data[i] = byte;
    f->flags[i] = flags;
    f->num++;
}

static void fifo_drop(ESP32P4SpiFifo *f, uint32_t n)
{
    f->head = (f->head + n) % ESP32P4_SPI_AFIFO;
    f->num -= n;
}

static unsigned periph(ESP32P4SpiState *s)
{
    return spi2(s) ? ESP32P4_GDMA_AXI_SPI2 : ESP32P4_GDMA_AXI_SPI3;
}

static bool sys_running(ESP32P4SpiState *s)
{
    return clock_is_enabled(s->sys_clk);
}

/* Take what the TX DMA has into the TX FIFO, while SPI_DMA_TX_ENA is set. */
static void txf_fill(ESP32P4SpiState *s)
{
    uint8_t buf[ESP32P4_SPI_AFIFO];

    if (!(s->dma_conf & R_DMA_CONF_DMA_TX_ENA_MASK) || !sys_running(s)) {
        return;
    }
    while (!fifo_full(&s->txf)) {
        uint32_t room = ESP32P4_SPI_AFIFO - s->txf.num;
        bool eof;
        size_t n = esp32p4_gdma_pull(s->gdma, periph(s), buf, room, &eof);

        for (size_t i = 0; i < n; i++) {
            fifo_put(&s->txf, buf[i], 0);
        }
        if (!n && !eof) {
            break;
        }
    }
}

static bool txf_pop(ESP32P4SpiState *s, uint8_t *byte)
{
    if (!s->txf.num) {
        txf_fill(s);
    }
    if (!s->txf.num) {
        return false;
    }
    *byte = s->txf.data[s->txf.head];
    fifo_drop(&s->txf, 1);
    txf_fill(s);
    return true;
}

/* Whether the TX DMA has no data now but more is on its way. */
static bool tx_waits(ESP32P4SpiState *s)
{
    return !s->txf.num && sys_running(s) &&
           esp32p4_gdma_tx_flowing(s->gdma, periph(s));
}

static bool slave_seg(ESP32P4SpiState *s);

/*
 * Whether the end of the data in the RX FIFO is still to be told: its
 * last byte stays in the FIFO until the transfer that received it ends,
 * which marks it as the end of the DMA frame (or, in a slave segmented
 * transfer, until CMD7 or End_SEG_TRANS does).
 */
static bool rx_holding(ESP32P4SpiState *s)
{
    if (s->dma_conf & R_DMA_CONF_RX_EOF_EN_MASK) {
        return false;
    }
    if (slave_mode(s)) {
        return s->s_state != S_IDLE || slave_seg(s);
    }
    return s->busy;
}

/* Give the RX DMA what the RX FIFO holds, frame by frame. */
static void rxf_drain(ESP32P4SpiState *s)
{
    uint8_t buf[ESP32P4_SPI_AFIFO];

    if (!sys_running(s)) {
        return;
    }
    while (s->rxf.num) {
        uint32_t n = 0, head = s->rxf.head, avail = s->rxf.num;
        uint8_t flags = 0;
        size_t took;

        if (rx_holding(s) &&
            !(s->rxf.flags[(head + avail - 1) % ESP32P4_SPI_AFIFO] & F_EOF)) {
            avail--;
            if (!avail) {
                return;
            }
        }

        if (s->rxf.flags[head] & F_NODATA) {
            if (!esp32p4_gdma_rx_room(s->gdma, periph(s))) {
                return;
            }
            esp32p4_gdma_push(s->gdma, periph(s), NULL, 0, ESP32P4_GDMA_EOF);
            fifo_drop(&s->rxf, 1);
            continue;
        }
        while (n < avail) {
            uint32_t i = (head + n) % ESP32P4_SPI_AFIFO;

            if (s->rxf.flags[i] & F_NODATA) {
                break;
            }
            buf[n++] = s->rxf.data[i];
            if (s->rxf.flags[i] & F_EOF) {
                flags = ESP32P4_GDMA_EOF;
                break;
            }
        }
        took = esp32p4_gdma_push(s->gdma, periph(s), buf, n, flags);
        fifo_drop(&s->rxf, took);
        if (took < n) {
            return;
        }
    }
}

/* Whether the RX FIFO is full but the RX DMA will make room. */
static bool rx_waits(ESP32P4SpiState *s)
{
    return fifo_full(&s->rxf) && sys_running(s) &&
           esp32p4_gdma_rx_flowing(s->gdma, periph(s));
}

/* End the DMA frame of the data received so far, if there is any. */
static void rx_eof(ESP32P4SpiState *s)
{
    ESP32P4SpiFifo *f = &s->rxf;

    if (f->num) {
        f->flags[(f->head + f->num - 1) % ESP32P4_SPI_AFIFO] |= F_EOF;
    } else if (s->rx_bits) {
        fifo_put(f, 0, F_NODATA | F_EOF);
    }
    s->rx_bits = 0;
    rxf_drain(s);
}

/*
 * A received byte (or the @bits of a last, partial one) for the RX DMA:
 * false if the FIFO overflows. With SPI_RX_EOF_EN the byte that makes up
 * SPI_MS_DATA_BITLEN + 1 bits ends the frame.
 */
static bool rx_put(ESP32P4SpiState *s, uint8_t byte, unsigned bits,
                   uint32_t ms_dlen)
{
    uint8_t flags = 0;

    if (fifo_full(&s->rxf)) {
        rxf_drain(s);
    }
    if (fifo_full(&s->rxf)) {
        spi_raise(s, INT_DMA_INFIFO_FULL_ERR);
        return false;
    }
    s->rx_bits += bits;
    if ((s->dma_conf & R_DMA_CONF_RX_EOF_EN_MASK) &&
        s->rx_bits >= FIELD_EX32(ms_dlen, MS_DLEN, MS_DATA_BITLEN) + 1) {
        flags = F_EOF;
        s->rx_bits = 0;
    }
    fifo_put(&s->rxf, byte, flags);
    rxf_drain(s);
    return true;
}

/* Bit order */

static uint8_t bitrev(uint8_t v, unsigned bits)
{
    uint8_t r = 0;

    for (unsigned i = 0; i < bits; i++) {
        r |= ((v >> i) & 1) << (bits - 1 - i);
    }
    return r;
}

static bool lanes_reversed(unsigned lanes, unsigned order)
{
    return lanes == 8 ? order & 1 : lanes > 1 && (order & 2);
}

/* The chunk of @byte the @c'th cycle of a byte carries on @lanes lanes. */
static uint8_t chunk_out(uint8_t byte, unsigned lanes, unsigned c,
                         unsigned order)
{
    unsigned n = 8 / lanes;
    unsigned g = (order & 1) ? c : n - 1 - c;
    uint8_t v = (byte >> (g * lanes)) & MAKE_64BIT_MASK(0, lanes);

    return lanes_reversed(lanes, order) ? bitrev(v, lanes) : v;
}

/* Put the chunk @v the @c'th cycle of a byte received into *@byte. */
static void chunk_in(uint8_t *byte, uint8_t v, unsigned lanes, unsigned c,
                     unsigned order)
{
    unsigned n = 8 / lanes;
    unsigned g = (order & 1) ? c : n - 1 - c;

    if (lanes_reversed(lanes, order)) {
        v = bitrev(v, lanes);
    }
    *byte |= v << (g * lanes);
}

/* The bits a byte's first @k chunks carry, in the byte's positions. */
static uint8_t chunk_mask(unsigned lanes, unsigned k, unsigned order)
{
    uint8_t m = 0;

    for (unsigned c = 0; c < k; c++) {
        chunk_in(&m, MAKE_64BIT_MASK(0, lanes), lanes, c, order);
    }
    return m;
}

/* The CPU buffer */

static uint8_t wbuf_get(ESP32P4SpiState *s, unsigned idx)
{
    return s->w[idx / 4] >> (8 * (idx % 4));
}

static void wbuf_set(ESP32P4SpiState *s, unsigned idx, uint8_t v)
{
    s->w[idx / 4] = deposit32(s->w[idx / 4], 8 * (idx % 4), 8, v);
}

/*
 * The buffer byte a master transfer's @n'th byte uses (TRM 46.5.6.1): the
 * 64 bytes from SPI_W0, or the 32 from SPI_W8 with a high part, then
 * SPI_W15[31:24] until byte 256, and over again.
 */
static unsigned wbuf_master(unsigned n, bool high)
{
    unsigned size = high ? 32 : 64;

    n %= 256;
    return n < size ? (high ? 32 : 0) + n : 63;
}

/* The pads and lines */

static void spi_routes_changed(ESP32P4SpiState *s)
{
    s->src_valid = false;
    if (s->peer) {
        s->peer->src_valid = false;
    }
}

static void drive(ESP32P4SpiState *s, unsigned line, bool level, bool oe)
{
    if (sig_of(s, line) < 0) {
        return;
    }
    if (!s->lines_driven || s->out_level[line] != level) {
        s->out_level[line] = level;
        qemu_set_irq(s->sig_out[line], level);
    }
    if (!s->lines_driven || s->oe_level[line] != oe) {
        s->oe_level[line] = oe;
        qemu_set_irq(s->sig_oe[line], oe);
        spi_routes_changed(s);
    }
}

/*
 * The board's SSI devices: a device with "cs" n sees the level of a pad
 * that carries the controller's chip select n, or its inactive level
 * where none does.
 */
static void spi_update_devices(ESP32P4SpiState *s)
{
    BusChild *kid;
    bool any = false;

    QTAILQ_FOREACH(kid, &BUS(s->bus)->children, sibling) {
        SSIPeripheral *p = SSI_PERIPHERAL(kid->child);
        SSIPeripheralClass *pc = SSI_PERIPHERAL_GET_CLASS(p);
        bool active_high = pc->cs_polarity == SSI_CS_HIGH;
        bool level = !active_high, routed = false;

        if (pc->cs_polarity == SSI_CS_NONE) {
            any = true;
            continue;
        }
        if (p->cs_index < ncs(s)) {
            uint64_t pads = esp32p4_gpio_sig_out_pads(s->gpio,
                sig_of(s, ESP32P4_SPI_LINE_CS0 + p->cs_index));

            if (pads) {
                routed = esp32p4_gpio_pad_driven(s->gpio, ctz64(pads),
                                                 &level);
            }
        }
        if (!routed) {
            level = !active_high;
        }
        qemu_set_irq(qdev_get_gpio_in_named(DEVICE(p), SSI_GPIO_CS, 0),
                     level);
        any |= level == active_high;
    }
    s->ssi_selected = any;
}

static bool has_devices(ESP32P4SpiState *s)
{
    return !QTAILQ_EMPTY(&BUS(s->bus)->children);
}

static unsigned phase_lanes(bool oct, bool quad, bool dual)
{
    return oct ? 8 : quad ? 4 : dual ? 2 : 1;
}

/*
 * The lines of a master: the clock at its idle level, the chip selects
 * active from the start of PREP to the end of DONE (and on after a
 * transfer that ends with SPI_CS_KEEP_ACTIVE set, until the end of one
 * without it), the data lanes the current state sends on enabled
 * and those it receives on released. Between states D, WP and HD are held
 * at their idle levels, as a flash's write protect and hold need; Q, a
 * master's input in 1-bit modes, is released.
 */
static void master_drive_lines(ESP32P4SpiState *s)
{
    uint32_t misc = s->mreg[ESP32P4_SPI_M_MISC];
    uint32_t ctrl = s->mreg[ESP32P4_SPI_M_CTRL];
    bool cs = s->cs_on || s->cs_held;
    uint8_t oe = BIT(0) | BIT(2) | BIT(3);
    uint8_t lvl = (FIELD_EX32(ctrl, CTRL, D_POL) << 0) |
                  (FIELD_EX32(ctrl, CTRL, Q_POL) << 1) |
                  (FIELD_EX32(ctrl, CTRL, WP_POL) << 2) |
                  (FIELD_EX32(ctrl, CTRL, HOLD_POL) << 3);

    drive(s, ESP32P4_SPI_LINE_CK, FIELD_EX32(misc, MISC, CK_IDLE_EDGE),
          !FIELD_EX32(misc, MISC, CK_DIS));
    for (unsigned n = 0; n < ncs(s); n++) {
        bool on = cs && !(FIELD_EX32(misc, MISC, CS_DIS) & BIT(n));
        bool pol = FIELD_EX32(misc, MISC, MASTER_CS_POL) & BIT(n);

        drive(s, ESP32P4_SPI_LINE_CS0 + n, !on ^ pol, true);
    }
    if (s->busy && s->cs_on && s->cur < s->nph) {
        ESP32P4SpiPhase *ph = &s->ph[s->cur];
        uint32_t user = s->x[ESP32P4_SPI_M_USER];
        uint8_t m = MAKE_64BIT_MASK(0, ph->lanes);

        switch (ph->kind) {
        case PH_CMD:
        case PH_ADDR:
        case PH_DOUT:
            oe |= m;
            break;
        case PH_DUMMY:
            if (FIELD_EX32(s->x[ESP32P4_SPI_M_CTRL], CTRL, DUMMY_OUT)) {
                oe |= MAKE_64BIT_MASK(0, ph->ssi_lanes);
            } else {
                oe &= ~MAKE_64BIT_MASK(0, ph->ssi_lanes);
            }
            break;
        case PH_DIN:
            if (ph->lanes > 1 || FIELD_EX32(user, USER, SIO)) {
                oe &= ~m;
            }
            break;
        default:
            break;
        }
    }
    for (unsigned l = 0; l < nlanes(s); l++) {
        drive(s, l, (lvl >> l) & 1, (oe >> l) & 1);
    }
    drive(s, ESP32P4_SPI_LINE_DQS, FIELD_EX32(misc, MISC, DQS_IDLE_EDGE),
          false);
    if (has_devices(s)) {
        spi_update_devices(s);
    }
}

/* The lanes a slave sends data on in its current state. */
static uint8_t slave_out_lanes(ESP32P4SpiState *s)
{
    bool sio = s->user & R_USER_SIO_MASK;

    if (!s->s_cs) {
        return 0;
    }
    switch (s->s_state) {
    case S_FD:
        return BIT(1);
    case S_RD:
        if (s->s_data_lanes == 1) {
            return sio ? BIT(0) : BIT(1);
        }
        return MAKE_64BIT_MASK(0, s->s_data_lanes);
    default:
        return 0;
    }
}

/* The lines of a slave: inputs, but for the data it is sending. */
static void slave_drive_lines(ESP32P4SpiState *s)
{
    uint8_t oe = slave_out_lanes(s);
    uint8_t lvl = (FIELD_EX32(s->ctrl, CTRL, D_POL) << 0) |
                  (FIELD_EX32(s->ctrl, CTRL, Q_POL) << 1) |
                  (FIELD_EX32(s->ctrl, CTRL, WP_POL) << 2) |
                  (FIELD_EX32(s->ctrl, CTRL, HOLD_POL) << 3);

    drive(s, ESP32P4_SPI_LINE_CK, FIELD_EX32(s->misc, MISC, CK_IDLE_EDGE),
          false);
    for (unsigned n = 0; n < ncs(s); n++) {
        drive(s, ESP32P4_SPI_LINE_CS0 + n, true, false);
    }
    for (unsigned l = 0; l < nlanes(s); l++) {
        drive(s, l, (lvl >> l) & 1, (oe >> l) & 1);
    }
    drive(s, ESP32P4_SPI_LINE_DQS, false, false);
}

static void spi_drive_lines(ESP32P4SpiState *s)
{
    if (slave_mode(s)) {
        slave_drive_lines(s);
    } else {
        master_drive_lines(s);
    }
    s->lines_driven = true;
}

/*
 * The output signal of @s or @other that drives input signal @sig of @s:
 * through the pad the matrix or IO MUX takes it from, or through a wire
 * of the board's ("pad-wires") from another pad on that wire.
 */
static bool in_source(ESP32P4SpiState *s, ESP32P4SpiState *other,
                      unsigned sig, unsigned *out, bool *inv)
{
    ESP32P4SpiState *who[2] = { s, other };
    uint64_t wire = 0;
    int pad;

    if (esp32p4_gpio_sig_in_source(s->gpio, sig, out, inv)) {
        return true;
    }
    pad = esp32p4_gpio_sig_in_pad(s->gpio, sig);
    if (pad < 0 || !s->gpio->pad_wire[pad]) {
        return false;
    }
    for (unsigned n = 0; n < ESP32P4_GPIO_PADS; n++) {
        if (s->gpio->pad_wire[n] == s->gpio->pad_wire[pad]) {
            wire |= 1ULL << n;
        }
    }
    for (unsigned k = 0; k < 2; k++) {
        for (unsigned line = 0; who[k] && line < ESP32P4_SPI_LINES; line++) {
            int o = sig_of(who[k], line);

            if (o >= 0 && (esp32p4_gpio_sig_out_pads(s->gpio, o) & wire)) {
                *out = o;
                *inv = false;
                return true;
            }
        }
    }
    return false;
}

/*
 * [spec:nuos:req:emu.esp32p4.spi]
 * Where each data input of @s comes from, through the matrix or the IO
 * MUX (and the board's wires between pads): an output of its own, or of
 * @other (the other controller), a pad no signal of the chip drives (the
 * board's SSI devices), or neither (the level the matrix delivers); and
 * whether @other's clock input comes from @s's clock output.
 */
static void resolve(ESP32P4SpiState *s, ESP32P4SpiState *other)
{
    unsigned ck;
    bool ck_inv;

    s->clocks_peer = other &&
        in_source(other, s, sig_of(other, ESP32P4_SPI_LINE_CK), &ck,
                  &ck_inv) &&
        ck == (unsigned)sig_of(s, ESP32P4_SPI_LINE_CK);
    s->ssi_out_lanes = 0;
    for (unsigned l = 0; l < ESP32P4_SPI_LANES; l++) {
        ESP32P4SpiLaneSrc *src = &s->src[l];
        int sig = l < nlanes(s) ? sig_of(s, l) : -1;
        unsigned out;
        bool inv, level;
        int j, pad;

        src->kind = SRC_CONST;
        src->lane = 0;
        src->inv = false;
        if (sig < 0) {
            continue;
        }
        if (esp32p4_gpio_sig_out_pads(s->gpio, sig)) {
            s->ssi_out_lanes |= BIT(l);
        }
        if (in_source(s, other, sig, &out, &inv)) {
            j = lane_of_signal(s, out);
            if (j >= 0) {
                src->kind = SRC_SELF;
            } else {
                j = other ? lane_of_signal(other, out) : -1;
                if (j < 0) {
                    continue;
                }
                src->kind = SRC_PEER;
            }
            src->lane = j;
            src->inv = inv;
            continue;
        }
        pad = esp32p4_gpio_sig_in_pad(s->gpio, sig);
        if (pad >= 0 && !esp32p4_gpio_pad_driven(s->gpio, pad, &level)) {
            src->kind = SRC_EXT;
        }
    }
    s->src_valid = true;
}

/*
 * The levels on @s's data inputs in a cycle in which it drives @own,
 * the other controller @other, and the board's devices answer @ext.
 */
static uint8_t gather(ESP32P4SpiState *s, uint8_t own, uint8_t other,
                      uint8_t ext)
{
    uint8_t in = 0;

    for (unsigned l = 0; l < nlanes(s); l++) {
        ESP32P4SpiLaneSrc *src = &s->src[l];
        bool v;

        switch (src->kind) {
        case SRC_SELF:
            v = (own >> src->lane) & 1;
            break;
        case SRC_PEER:
            v = (other >> src->lane) & 1;
            break;
        case SRC_EXT:
            v = s->ssi_selected ? (ext >> l) & 1 : s->in_level[l];
            break;
        default:
            v = s->in_level[l];
            break;
        }
        in |= (v ^ src->inv) << l;
    }
    return in;
}

/* The other controller, if it is a slave this master clocks. */
static ESP32P4SpiState *clocked_peer(ESP32P4SpiState *s)
{
    ESP32P4SpiState *p = s->peer;

    if (!p || !s->clocks_peer || !slave_mode(p) || !sys_running(p)) {
        return NULL;
    }
    return p;
}

/* The slave */

static bool slave_seg(ESP32P4SpiState *s)
{
    return s->dma_conf & R_DMA_CONF_DMA_SLV_SEG_TRANS_EN_MASK;
}

static bool slave_tx_dma(ESP32P4SpiState *s)
{
    if (s->s_state == S_FD) {
        return s->dma_conf & R_DMA_CONF_DMA_TX_ENA_MASK;
    }
    return s->s_state == S_RD && s->s_kind == K_RDDMA;
}

static bool slave_rx_dma(ESP32P4SpiState *s)
{
    if (s->s_state == S_FD) {
        return s->dma_conf & R_DMA_CONF_DMA_RX_ENA_MASK;
    }
    return s->s_state == S_WR && s->s_kind == K_WRDMA;
}

/*
 * The buffer byte a slave's half-duplex transfer at address @a uses (TRM
 * 46.5.6.2): past byte 63 it overwrites SPI_W8 to SPI_W15, raising
 * SPI_SLV_BUF_ADDR_ERR_INT.
 */
static unsigned slave_buf_idx(ESP32P4SpiState *s, unsigned a)
{
    if (a < 64) {
        return a;
    }
    spi_raise(s, INT_SLV_BUF_ADDR_ERR);
    return 32 + (a - 64) % 32;
}

/* A full-duplex slave's @n'th byte: the buffer from SPI_W0, as a master's. */
static unsigned slave_fd_idx(ESP32P4SpiState *s, unsigned n)
{
    if (n == 64) {
        spi_raise(s, INT_SLV_BUF_ADDR_ERR);
    }
    return wbuf_master(n, false);
}

/* [spec:nuos:req:emu.esp32p4.spi] */
static void slave_begin(ESP32P4SpiState *s)
{
    s->s_bits = 0;
    s->s_data_bits = 0;
    s->s_n = 0;
    s->s_rx = 0;
    s->s_cmd = 0;
    s->s_addr = 0;
    s->s_kind = K_NONE;
    s->s_err = false;
    if (s->user & R_USER_DOUTDIN_MASK) {
        s->s_state = S_FD;
        s->s_lanes = 1;
    } else {
        s->s_state = S_CMD;
        s->s_lanes = (s->user & R_USER_QPI_MODE_MASK) ? 4 : 1;
    }
    slave_drive_lines(s);
}

/* The bits of the last, partial byte a slave received, as its strobe. */
static void slave_store_partial(ESP32P4SpiState *s)
{
    unsigned order = FIELD_EX32(s->ctrl, CTRL, RD_BIT_ORDER);
    unsigned lanes = s->s_lanes;
    unsigned k = (s->s_bits % 8) / lanes;
    uint8_t strb = k ? chunk_mask(lanes, k, order) : 0xff;

    s->slave = FIELD_DP32(s->slave, SLAVE, SLV_LAST_BYTE_STRB, strb);
}

static void slave_rx_byte(ESP32P4SpiState *s, uint8_t byte, unsigned bits)
{
    if (s->s_state == S_FD) {
        if (slave_rx_dma(s)) {
            if (!s->s_err && !s->rx_drop &&
                !rx_put(s, byte, bits, s->ms_dlen)) {
                s->s_err = true;
            }
        } else {
            wbuf_set(s, slave_fd_idx(s, s->s_n), byte);
        }
    } else if (s->s_kind == K_WRBUF) {
        wbuf_set(s, slave_buf_idx(s, s->s_addr + s->s_n), byte);
    } else if (s->s_kind == K_WRDMA) {
        if (s->s_err || s->rx_drop) {
            return;
        }
        if (!rx_put(s, byte, bits, s->ms_dlen)) {
            s->s_err = true;
            if (slave_seg(s) &&
                (s->dma_conf & R_DMA_CONF_SLV_RX_SEG_TRANS_CLR_EN_MASK)) {
                s->rx_drop = true;
            }
        }
    }
}

/*
 * A slave's next byte to send, at the start of a byte: the CPU buffer, or
 * the TX DMA, which sends zeros once it has nothing more to give.
 */
static uint8_t slave_tx_byte(ESP32P4SpiState *s)
{
    uint8_t byte;

    if (slave_tx_dma(s)) {
        if (txf_pop(s, &byte)) {
            return byte;
        }
        spi_raise(s, INT_DMA_OUTFIFO_EMPTY_ERR);
        return 0;
    }
    if (s->s_state == S_FD) {
        return wbuf_get(s, slave_fd_idx(s, s->s_n));
    }
    return wbuf_get(s, slave_buf_idx(s, s->s_addr + s->s_n));
}

/* Whether a slave can take its next cycle without waiting for the DMA. */
static bool slave_ready(ESP32P4SpiState *s)
{
    unsigned lanes = s->s_lanes;

    if (!s->s_cs || !slave_mode(s)) {
        return true;
    }
    if (slave_tx_dma(s) && s->s_bits % 8 == 0 && tx_waits(s)) {
        return false;
    }
    if (slave_rx_dma(s) && (s->s_bits + lanes) % 8 == 0 && !s->s_err &&
        rx_waits(s)) {
        return false;
    }
    return true;
}

/* A slave's levels on its data lanes in this cycle. */
static uint8_t slave_out(ESP32P4SpiState *s)
{
    unsigned lanes = s->s_lanes;
    unsigned order = FIELD_EX32(s->ctrl, CTRL, WR_BIT_ORDER);
    unsigned c;
    uint8_t v, out = slave_out_lanes(s);

    if (!out || !slave_mode(s)) {
        return 0;
    }
    c = (s->s_bits % 8) / lanes;
    if (c == 0) {
        s->s_tx = slave_tx_byte(s);
    }
    v = chunk_out(s->s_tx, lanes, c, order);
    if (lanes == 1) {
        return v ? out : 0;
    }
    return v;
}

/*
 * [spec:nuos:req:emu.esp32p4.spi]
 * A half-duplex slave's command (TRM 46.5.10.2): bits 3:0 the transfer,
 * bits 7:4 the lanes of the address and data, 1, 2 or 4.
 */
static void slave_decode(ESP32P4SpiState *s)
{
    bool qpi = s->user & R_USER_QPI_MODE_MASK;
    uint8_t cmd = s->s_cmd;
    unsigned mode = cmd >> 4;

    s->s_addr_lanes = 1;
    s->s_data_lanes = 1;
    if ((!qpi && cmd == 0x05) || (qpi && cmd == 0xa5)) {
        s->s_kind = K_SEGEND;
    } else if (!qpi && cmd == 0x06) {
        s->s_kind = K_ENQPI;
    } else if (qpi && cmd == 0xdd) {
        s->s_kind = K_EXQPI;
    } else {
        static const uint8_t kinds[16] = {
            [1] = K_WRBUF, [2] = K_RDBUF, [3] = K_WRDMA, [4] = K_RDDMA,
            [7] = K_CMD7, [8] = K_CMD8, [9] = K_CMD9, [0xa] = K_CMDA,
        };

        s->s_kind = kinds[cmd & 0xf];
        switch (mode) {
        case 0x0:
            break;
        case 0x1:
            s->s_data_lanes = 2;
            break;
        case 0x2:
            s->s_data_lanes = 4;
            break;
        case 0x5:
            s->s_addr_lanes = 2;
            s->s_data_lanes = 2;
            break;
        case 0xa:
            s->s_addr_lanes = 4;
            s->s_data_lanes = 4;
            break;
        default:
            s->s_kind = K_NONE;
            break;
        }
        if (qpi && mode != 0xa) {
            s->s_kind = K_NONE;
        }
        if (s->s_kind == K_NONE) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "esp32p4_spi: SPI%u slave: unsupported command "
                          "0x%02x\n", s->host, cmd);
            s->s_kind = K_ERR;
        }
    }
    s->s_bits = 0;
    if (s->s_kind == K_ERR || s->s_kind == K_SEGEND ||
        s->s_kind == K_ENQPI || s->s_kind == K_EXQPI) {
        s->s_state = S_IGNORE;
    } else {
        s->s_state = S_ADDR;
        s->s_lanes = s->s_addr_lanes;
    }
}

/* A slave takes the levels @in of its data lanes in a cycle. */
static void slave_in(ESP32P4SpiState *s, uint8_t in)
{
    unsigned order = FIELD_EX32(s->ctrl, CTRL, RD_BIT_ORDER);
    unsigned lanes = s->s_lanes;
    unsigned c;
    uint8_t v;

    if (!s->s_cs || !slave_mode(s)) {
        return;
    }
    if (lanes == 1) {
        v = in & 1;
    } else {
        v = in & MAKE_64BIT_MASK(0, lanes);
        if (lanes == 4 && (s->misc & R_MISC_QUAD_DIN_PIN_SWAP_MASK)) {
            v = ((v & 0x5) << 1) | ((v & 0xa) >> 1);
        }
    }
    c = (s->s_bits % 8) / lanes;
    switch (s->s_state) {
    case S_FD:
        chunk_in(&s->s_rx, v, 1, c, order);
        s->s_bits++;
        s->s_data_bits++;
        if (s->s_bits % 8 == 0) {
            slave_rx_byte(s, s->s_rx, 8);
            s->s_rx = 0;
            s->s_n++;
        }
        break;
    case S_CMD:
        chunk_in(&s->s_cmd, v, lanes, c, order);
        s->s_bits += lanes;
        if (s->s_bits == 8) {
            slave_decode(s);
        }
        break;
    case S_ADDR:
        chunk_in(&s->s_addr, v, lanes, c, order);
        s->s_bits += lanes;
        if (s->s_bits == 8) {
            s->s_bits = 0;
            s->s_state = S_DUMMY;
        }
        break;
    case S_DUMMY:
        if (++s->s_bits == 8) {
            s->s_bits = 0;
            s->s_lanes = s->s_data_lanes;
            switch (s->s_kind) {
            case K_WRBUF:
            case K_WRDMA:
                s->s_state = S_WR;
                break;
            case K_RDBUF:
            case K_RDDMA:
                s->s_state = S_RD;
                slave_drive_lines(s);
                break;
            default:
                s->s_state = S_IGNORE;
                break;
            }
        }
        break;
    case S_WR:
        chunk_in(&s->s_rx, v, lanes, c, order);
        s->s_bits += lanes;
        s->s_data_bits += lanes;
        if (s->s_bits % 8 == 0) {
            slave_rx_byte(s, s->s_rx, 8);
            s->s_rx = 0;
            s->s_n++;
        }
        break;
    case S_RD:
        s->s_bits += lanes;
        s->s_data_bits += lanes;
        if (s->s_bits % 8 == 0) {
            s->s_n++;
        }
        break;
    default:
        break;
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.spi]
 * CS has gone inactive: the end of a slave transfer.
 */
static void slave_end(ESP32P4SpiState *s)
{
    uint32_t ints = INT_TRANS_DONE;
    uint32_t bitlen_en = 0;
    bool eof = false;

    if (s->s_state == S_IDLE) {
        return;
    }
    if (s->s_state == S_FD || s->s_state == S_WR) {
        if (s->s_bits % 8) {
            slave_store_partial(s);
            slave_rx_byte(s, s->s_rx, s->s_bits % 8);
        } else if (s->s_data_bits) {
            s->slave = FIELD_DP32(s->slave, SLAVE, SLV_LAST_BYTE_STRB, 0xff);
        }
    }
    if (s->s_state == S_FD) {
        s->slave1 = FIELD_DP32(s->slave1, SLAVE1, SLV_DATA_BITLEN,
                               s->s_data_bits);
        eof = slave_rx_dma(s) && !slave_seg(s);
    } else if (s->s_kind != K_NONE) {
        s->slave1 = FIELD_DP32(s->slave1, SLAVE1, SLV_LAST_COMMAND, s->s_cmd);
        s->slave1 = FIELD_DP32(s->slave1, SLAVE1, SLV_LAST_ADDR, s->s_addr);
        switch (s->s_kind) {
        case K_WRBUF:
            ints |= INT_SLV_WR_BUF_DONE;
            bitlen_en = R_SLAVE_SLV_WRBUF_BITLEN_EN_MASK;
            break;
        case K_RDBUF:
            ints |= INT_SLV_RD_BUF_DONE;
            bitlen_en = R_SLAVE_SLV_RDBUF_BITLEN_EN_MASK;
            break;
        case K_WRDMA:
            ints |= INT_SLV_WR_DMA_DONE;
            bitlen_en = R_SLAVE_SLV_WRDMA_BITLEN_EN_MASK;
            eof = !slave_seg(s);
            break;
        case K_RDDMA:
            ints |= INT_SLV_RD_DMA_DONE;
            bitlen_en = R_SLAVE_SLV_RDDMA_BITLEN_EN_MASK;
            break;
        case K_CMD7:
            ints |= INT_SLV_CMD7;
            eof = true;
            break;
        case K_CMD8:
            ints |= INT_SLV_CMD8;
            break;
        case K_CMD9:
            ints |= INT_SLV_CMD9;
            break;
        case K_CMDA:
            ints |= INT_SLV_CMDA;
            break;
        case K_SEGEND:
            if (slave_seg(s)) {
                ints |= INT_DMA_SEG_TRANS_DONE;
                eof = true;
            }
            break;
        case K_ENQPI:
            s->user |= R_USER_QPI_MODE_MASK;
            ints |= INT_SLV_EN_QPI;
            break;
        case K_EXQPI:
            s->user &= ~R_USER_QPI_MODE_MASK;
            ints |= INT_SLV_EX_QPI;
            break;
        case K_ERR:
            ints |= INT_SLV_CMD_ERR;
            break;
        }
        if (s->slave & bitlen_en) {
            s->slave1 = FIELD_DP32(s->slave1, SLAVE1, SLV_DATA_BITLEN,
                                   s->s_data_bits);
        }
    }
    if (eof && (s->dma_conf & R_DMA_CONF_DMA_RX_ENA_MASK)) {
        rx_eof(s);
    }
    s->s_state = S_IDLE;
    s->cmd &= ~R_CMD_USR_MASK;
    rxf_drain(s);
    slave_drive_lines(s);
    spi_raise(s, ints);
}

/* The slave's CS0 input, after SPI_SLAVE_CS_POL. */
static void slave_cs(ESP32P4SpiState *s, bool active)
{
    if (active == s->s_cs) {
        return;
    }
    s->s_cs = active;
    if (!slave_mode(s) || !sys_running(s)) {
        s->s_state = S_IDLE;
        return;
    }
    if (active) {
        slave_begin(s);
    } else {
        slave_end(s);
    }
}

/* The master */

/*
 * The first whole ns by which half cycle @h of the transfer has ended:
 * edge h0 + h of its half cycles; capped so that a time stays in range.
 */
static int64_t master_half_time(ESP32P4SpiState *s, uint64_t h)
{
    if (h > INT64_MAX / 4 || s->h0 > INT64_MAX / 4) {
        return INT64_MAX / 2;
    }
    return MIN(esp32p4_clk_edge_ns(&s->half_e, s->h0 + h), INT64_MAX / 2);
}

/*
 * Count the transfer's half cycles afresh, at the current half period,
 * half cycle @h ending at clk_spi_mst's first edge at or after @t.
 */
static void master_restart(ESP32P4SpiState *s, int64_t t, uint64_t h)
{
    ESP32P4ClkEdges mst = esp32p4_clk_edges_or_at(s->mst_clk, t);
    int64_t k;
    bool at;

    s->half_e = esp32p4_clk_edges_from(s->half, &mst, t);
    if (!esp32p4_clk_edges_running(&s->half_e)) {
        s->h0 = 0;
        return;
    }
    k = esp32p4_clk_edge_index(&s->half_e, t, &at) + !at;
    s->h0 = k - (int64_t)h;
}

/*
 * Half an SPI_CLK cycle, exactly, from clk_spi_mst's exact period; stopped
 * while that is.
 */
static ESP32P4ClkPeriod master_half(ESP32P4SpiState *s)
{
    uint32_t clock = s->x[ESP32P4_SPI_M_CLOCK];
    uint64_t div = 1;

    if (!(clock & R_CLOCK_CLK_EQU_SYSCLK_MASK)) {
        div = (FIELD_EX32(clock, CLOCK, CLKDIV_PRE) + 1) *
              (FIELD_EX32(clock, CLOCK, CLKCNT_N) + 1);
    }
    return esp32p4_clk_period_scale(esp32p4_clk_exact(s->mst_clk), div, 2);
}

static bool master_clocks(ESP32P4SpiState *s)
{
    return clock_is_enabled(s->mst_clk) && sys_running(s);
}

static void add_phase(ESP32P4SpiState *s, unsigned kind, unsigned lanes,
                      unsigned ssi_lanes, uint32_t bits, uint32_t halves)
{
    ESP32P4SpiPhase *ph = &s->ph[s->nph++];

    assert(s->nph <= ESP32P4_SPI_PHASES);
    ph->kind = kind;
    ph->lanes = lanes;
    ph->ssi_lanes = ssi_lanes;
    ph->bits = bits;
    ph->beats = 0;
    ph->halves = halves;
    if (kind >= PH_CMD && kind <= PH_DATA) {
        ph->beats = kind == PH_DUMMY ? bits : DIV_ROUND_UP(bits, lanes);
        ph->halves = 2 * ph->beats;
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.spi]
 * The states of a master transaction (a segment of a configurable
 * segmented transfer), from its configuration (TRM 46.5.9.1, 46.5.9.2).
 */
static void master_plan(ESP32P4SpiState *s)
{
    uint32_t user = s->x[ESP32P4_SPI_M_USER];
    uint32_t user1 = s->x[ESP32P4_SPI_M_USER1];
    uint32_t user2 = s->x[ESP32P4_SPI_M_USER2];
    uint32_t ctrl = s->x[ESP32P4_SPI_M_CTRL];
    uint32_t bits = FIELD_EX32(s->x[ESP32P4_SPI_M_MS_DLEN], MS_DLEN,
                               MS_DATA_BITLEN) + 1;
    bool qpi = FIELD_EX32(user, USER, QPI_MODE);
    bool opi = spi2(s) && FIELD_EX32(user, USER, OPI_MODE);
    unsigned cmd_l, addr_l, out_l, in_l, dummy_l = 1;

    if (opi || qpi) {
        cmd_l = addr_l = out_l = in_l = opi ? 8 : 4;
    } else {
        cmd_l = phase_lanes(FIELD_EX32(ctrl, CTRL, FCMD_OCT),
                            FIELD_EX32(ctrl, CTRL, FCMD_QUAD),
                            FIELD_EX32(ctrl, CTRL, FCMD_DUAL));
        addr_l = phase_lanes(FIELD_EX32(ctrl, CTRL, FADDR_OCT),
                             FIELD_EX32(ctrl, CTRL, FADDR_QUAD),
                             FIELD_EX32(ctrl, CTRL, FADDR_DUAL));
        out_l = phase_lanes(FIELD_EX32(user, USER, FWRITE_OCT),
                            FIELD_EX32(user, USER, FWRITE_QUAD),
                            FIELD_EX32(user, USER, FWRITE_DUAL));
        in_l = phase_lanes(FIELD_EX32(ctrl, CTRL, FREAD_OCT),
                           FIELD_EX32(ctrl, CTRL, FREAD_QUAD),
                           FIELD_EX32(ctrl, CTRL, FREAD_DUAL));
    }
    s->nph = 0;
    add_phase(s, PH_PREP, 0, 0, 0,
              FIELD_EX32(user, USER, CS_SETUP) ?
              2 * FIELD_EX32(user1, USER1, CS_SETUP_TIME) + 3 : 1);
    if (FIELD_EX32(user, USER, USR_COMMAND)) {
        add_phase(s, PH_CMD, cmd_l, cmd_l,
                  FIELD_EX32(user2, USER2, USR_COMMAND_BITLEN) + 1, 0);
        dummy_l = cmd_l;
    }
    if (FIELD_EX32(user, USER, USR_ADDR)) {
        add_phase(s, PH_ADDR, addr_l, addr_l,
                  FIELD_EX32(user1, USER1, USR_ADDR_BITLEN) + 1, 0);
        dummy_l = addr_l;
    }
    if (FIELD_EX32(user, USER, USR_DUMMY)) {
        add_phase(s, PH_DUMMY, dummy_l, dummy_l,
                  FIELD_EX32(user1, USER1, USR_DUMMY_CYCLELEN) + 1, 0);
    }
    if (FIELD_EX32(user, USER, DOUTDIN)) {
        if (out_l != 1 || in_l != 1) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "esp32p4_spi: SPI%u: full duplex is 1-bit only; "
                          "the multi-line data bits are ignored\n", s->host);
        }
        if (FIELD_EX32(user, USER, USR_MOSI) ||
            FIELD_EX32(user, USER, USR_MISO)) {
            add_phase(s, PH_DATA, 1, 1, bits, 0);
        }
    } else {
        if (FIELD_EX32(user, USER, USR_MOSI)) {
            add_phase(s, PH_DOUT, out_l, out_l, bits, 0);
        }
        if (FIELD_EX32(user, USER, USR_MISO)) {
            add_phase(s, PH_DIN, in_l, in_l, bits, 0);
        }
    }
    add_phase(s, PH_DONE, 0, 0, 0,
              FIELD_EX32(user, USER, CS_HOLD) ?
              2 * FIELD_EX32(user1, USER1, CS_HOLD_TIME) + 3 : 1);
    s->cur = 0;
    s->cur_beat = 0;
    s->gen = 0;
    s->gen_beat = 0;
    s->gen_n = 0;
    s->ring_head = 0;
    s->ring_num = 0;
    s->ssi_gen_bits = 0;
    s->ssi_sent_bits = 0;
    s->ssi_acc = 0;
    s->din_byte = 0;
    s->din_n = 0;
}

/* The CONF state that starts each segment of a segmented transfer. */
static void master_plan_conf(ESP32P4SpiState *s)
{
    s->nph = 0;
    add_phase(s, PH_CONF, 0, 0, 0,
              2 * (s->x[ESP32P4_SPI_M_CONF_BITLEN] + 1));
    s->cur = 0;
    s->cur_beat = 0;
    s->gen = s->nph;
    s->ring_num = 0;
    s->conf_bitmap = 0;
    s->conf_word = 0;
    s->conf_byte = 0;
    s->conf_bit = 0;
    s->conf_val = 0;
}

static bool phase_has_dma_tx(ESP32P4SpiState *s, ESP32P4SpiPhase *ph)
{
    return (ph->kind == PH_DOUT ||
            (ph->kind == PH_DATA &&
             FIELD_EX32(s->x[ESP32P4_SPI_M_USER], USER, USR_MOSI))) &&
           (s->dma_conf & R_DMA_CONF_DMA_TX_ENA_MASK);
}

static bool phase_has_dma_rx(ESP32P4SpiState *s, ESP32P4SpiPhase *ph)
{
    return (ph->kind == PH_DIN ||
            (ph->kind == PH_DATA &&
             FIELD_EX32(s->x[ESP32P4_SPI_M_USER], USER, USR_MISO))) &&
           (s->dma_conf & R_DMA_CONF_DMA_RX_ENA_MASK);
}

/* End the transaction early, at its DONE state. */
static void master_abort(ESP32P4SpiState *s)
{
    s->cur = s->nph - 1;
    s->cur_beat = 0;
    s->gen = s->nph - 1;
    s->gen_beat = 0;
    s->ring_num = 0;
    master_drive_lines(s);
}

/*
 * The next byte a master sends: from the CPU buffer, or the TX DMA. -1
 * when the DMA has none yet but more is coming. Once the DMA has nothing
 * more to give the master sends its last byte again (TRM 46.5.7.2).
 */
static int master_tx_byte(ESP32P4SpiState *s, ESP32P4SpiPhase *ph)
{
    uint8_t byte;

    if (!phase_has_dma_tx(s, ph)) {
        return wbuf_get(s, wbuf_master(s->gen_n,
                        FIELD_EX32(s->x[ESP32P4_SPI_M_USER], USER,
                                   USR_MOSI_HIGHPART)));
    }
    if (txf_pop(s, &byte)) {
        s->tx_last = byte;
        return byte;
    }
    if (tx_waits(s)) {
        return -1;
    }
    if (!s->tx_err_raised) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32p4_spi: SPI%u: the TX DMA ran out of data\n",
                      s->host);
        s->tx_err_raised = true;
    }
    spi_raise(s, INT_DMA_OUTFIFO_EMPTY_ERR | INT_MST_TX_AFIFO_REMPTY_ERR);
    if (s->x[ESP32P4_SPI_M_USER2] & R_USER2_MST_REMPTY_ERR_END_EN_MASK) {
        s->abort = true;
    }
    return s->tx_last;
}

/* Place a chunk of @lanes lanes on the data lanes. */
static uint8_t lanes_out(uint8_t chunk, unsigned lanes)
{
    return lanes == 1 ? chunk & 1 : chunk;
}

/*
 * [spec:nuos:req:emu.esp32p4.spi]
 * Generate the master's next cycle: false if it must wait for the DMA.
 */
static bool gen_one(ESP32P4SpiState *s)
{
    ESP32P4SpiBeat *b;
    ESP32P4SpiPhase *ph;
    uint32_t user = s->x[ESP32P4_SPI_M_USER];
    uint32_t ctrl = s->x[ESP32P4_SPI_M_CTRL];
    unsigned order = FIELD_EX32(ctrl, CTRL, WR_BIT_ORDER);
    unsigned lanes, c, n;
    uint8_t out = 0, ssi_out = 0, byte;
    int t;

    while (s->gen < s->nph && s->gen_beat == s->ph[s->gen].beats) {
        s->gen++;
        s->gen_beat = 0;
    }
    assert(s->gen < s->nph);
    ph = &s->ph[s->gen];
    lanes = ph->lanes;
    n = 8 / MAX(lanes, 1);
    c = s->gen_beat % n;
    switch (ph->kind) {
    case PH_CMD:
        byte = s->x[ESP32P4_SPI_M_USER2] >> (8 * (s->gen_beat / n));
        out = lanes_out(chunk_out(byte, lanes, c, order), lanes);
        ssi_out = chunk_out(byte, lanes, c, order);
        break;
    case PH_ADDR:
        byte = s->x[ESP32P4_SPI_M_ADDR] >> (24 - 8 * (s->gen_beat / n));
        out = lanes_out(chunk_out(byte, lanes, c, order), lanes);
        ssi_out = chunk_out(byte, lanes, c, order);
        break;
    case PH_DUMMY:
        break;
    case PH_DOUT:
    case PH_DATA:
        if (ph->kind == PH_DATA && !FIELD_EX32(user, USER, USR_MOSI)) {
            out = FIELD_EX32(ctrl, CTRL, D_POL);
            break;
        }
        if (c == 0) {
            t = master_tx_byte(s, ph);
            if (t < 0) {
                return false;
            }
            s->gen_byte = t;
            s->gen_n++;
        }
        ssi_out = chunk_out(s->gen_byte, lanes, c, order);
        out = lanes_out(ssi_out, lanes);
        break;
    case PH_DIN:
        if (lanes == 1 && !FIELD_EX32(user, USER, SIO)) {
            out = FIELD_EX32(ctrl, CTRL, D_POL);
        }
        break;
    }
    assert(s->ring_num < ESP32P4_SPI_RING);
    b = &s->ring[(s->ring_head + s->ring_num++) % ESP32P4_SPI_RING];
    b->phase = s->gen;
    b->out = out;
    b->ssi_bits = 0;
    b->ssi_pos = s->ssi_gen_bits;
    if (has_devices(s)) {
        unsigned bits = ph->ssi_lanes;
        uint8_t routed = lanes == 1 ? s->ssi_out_lanes & 1
                                    : s->ssi_out_lanes;

        if (ph->kind != PH_DUMMY && ph->kind != PH_DIN) {
            ssi_out &= lanes == 1 ? routed : routed & 0xff;
        } else {
            ssi_out = 0;
        }
        b->ssi_bits = bits;
        for (int i = bits - 1; i >= 0; i--) {
            s->ssi_acc = (s->ssi_acc << 1) | ((ssi_out >> i) & 1);
            if (++s->ssi_gen_bits % 8 == 0) {
                s->ssi_resp[(s->ssi_gen_bits / 8 - 1) % 4] =
                    ssi_transfer(s->bus, s->ssi_acc & 0xff);
                s->ssi_sent_bits = s->ssi_gen_bits;
                s->ssi_acc = 0;
            }
        }
    }
    s->gen_beat++;
    return true;
}

/* Whether the generator has cycles left in this transaction. */
static bool gen_left(ESP32P4SpiState *s)
{
    for (unsigned i = s->gen; i < s->nph; i++) {
        if (s->ph[i].beats > (i == s->gen ? s->gen_beat : 0)) {
            return true;
        }
    }
    return false;
}

/*
 * A transaction's bits for the board's devices end mid-byte: send the
 * byte out with zeros after them, as QEMU's SSI devices take bytes.
 */
static void ssi_flush(ESP32P4SpiState *s)
{
    unsigned k = 8 - s->ssi_gen_bits % 8;

    qemu_log_mask(LOG_UNIMP,
                  "esp32p4_spi: SPI%u: %u bits are not whole bytes; QEMU's "
                  "SSI devices get %u bits more\n", s->host,
                  s->ssi_gen_bits, k);
    s->ssi_resp[(s->ssi_gen_bits / 8) % 4] =
        ssi_transfer(s->bus, (s->ssi_acc << k) & 0xff);
    s->ssi_acc = 0;
    s->ssi_gen_bits += k;
    s->ssi_sent_bits = s->ssi_gen_bits;
}

/* Make the next cycle ready, the devices' answer to it included. */
static bool master_head(ESP32P4SpiState *s)
{
    for (;;) {
        if (s->ring_num) {
            ESP32P4SpiBeat *b = &s->ring[s->ring_head];

            if (!b->ssi_bits || b->ssi_pos + b->ssi_bits <= s->ssi_sent_bits) {
                return true;
            }
        }
        if (!gen_left(s)) {
            assert(s->ring_num && s->ssi_gen_bits % 8);
            ssi_flush(s);
            continue;
        }
        if (!gen_one(s)) {
            return false;
        }
    }
}

/* The devices' answer to cycle @b: its bits, the highest lane's first. */
static uint8_t ssi_answer(ESP32P4SpiState *s, ESP32P4SpiBeat *b)
{
    uint8_t v = 0;

    for (unsigned i = 0; i < b->ssi_bits; i++) {
        uint32_t p = b->ssi_pos + i;
        uint8_t byte = s->ssi_resp[(p / 8) % 4];

        v = (v << 1) | ((byte >> (7 - p % 8)) & 1);
    }
    return v;
}

/* The lane a master receives a 1-bit state on. */
static unsigned master_in_lane(ESP32P4SpiState *s, ESP32P4SpiPhase *ph)
{
    return ph->kind == PH_DIN && FIELD_EX32(s->x[ESP32P4_SPI_M_USER], USER,
                                            SIO) ? 0 : 1;
}

/* Whether cycle @beat of @ph completes a received byte. */
static bool completes_byte(ESP32P4SpiPhase *ph, uint32_t beat)
{
    return (beat + 1) % (8 / ph->lanes) == 0 || beat + 1 == ph->beats;
}

static bool phase_receives(ESP32P4SpiState *s, ESP32P4SpiPhase *ph)
{
    return ph->kind == PH_DIN ||
           (ph->kind == PH_DATA &&
            FIELD_EX32(s->x[ESP32P4_SPI_M_USER], USER, USR_MISO));
}

/* The master takes the levels @in of its data lanes in a cycle. */
static void master_take(ESP32P4SpiState *s, ESP32P4SpiPhase *ph, uint8_t in)
{
    uint32_t user = s->x[ESP32P4_SPI_M_USER];
    unsigned order = FIELD_EX32(s->x[ESP32P4_SPI_M_CTRL], CTRL,
                                RD_BIT_ORDER);
    unsigned lanes = ph->lanes;
    unsigned c = s->cur_beat % (8 / lanes);
    uint8_t v;

    if (!phase_receives(s, ph)) {
        return;
    }
    if (lanes == 1) {
        v = (in >> master_in_lane(s, ph)) & 1;
    } else {
        v = in & MAKE_64BIT_MASK(0, lanes);
        if (lanes == 4 && (s->x[ESP32P4_SPI_M_MISC] &
                           R_MISC_QUAD_DIN_PIN_SWAP_MASK)) {
            v = ((v & 0x5) << 1) | ((v & 0xa) >> 1);
        }
    }
    chunk_in(&s->din_byte, v, lanes, c, order);
    if (!completes_byte(ph, s->cur_beat)) {
        return;
    }
    if (phase_has_dma_rx(s, ph)) {
        unsigned bits = MIN(8, ph->bits - 8 * s->din_n);

        s->rx_had_data = true;
        if (!rx_put(s, s->din_byte, bits, s->x[ESP32P4_SPI_M_MS_DLEN])) {
            spi_raise(s, INT_MST_RX_AFIFO_WFULL_ERR);
            if (s->x[ESP32P4_SPI_M_USER1] &
                R_USER1_MST_WFULL_ERR_END_EN_MASK) {
                s->abort = true;
            }
        }
    } else {
        wbuf_set(s, wbuf_master(s->din_n,
                                FIELD_EX32(user, USER, USR_MISO_HIGHPART)),
                 s->din_byte);
    }
    s->din_byte = 0;
    s->din_n++;
}

/*
 * [spec:nuos:req:emu.esp32p4.spi]
 * One SPI_CLK cycle of a data-carrying state: the master's levels, the
 * slave's it clocks and the board's devices' answer, each end taking what
 * the routing brings it. False if it must wait for the DMA.
 */
static bool master_beat(ESP32P4SpiState *s)
{
    ESP32P4SpiPhase *ph = &s->ph[s->cur];
    ESP32P4SpiState *p;
    ESP32P4SpiBeat *b;
    uint8_t p_out, m_in, p_in;

    if (!s->src_valid) {
        resolve(s, s->peer);
    }
    if (!master_head(s)) {
        return false;
    }
    if (phase_has_dma_rx(s, ph) && completes_byte(ph, s->cur_beat) &&
        rx_waits(s)) {
        return false;
    }
    if (!s->src_valid) {
        resolve(s, s->peer);
    }
    p = clocked_peer(s);
    if (p && !slave_ready(p)) {
        return false;
    }
    if (!s->src_valid) {
        resolve(s, s->peer);
    }
    if (p && !p->src_valid) {
        resolve(p, s);
    }
    b = &s->ring[s->ring_head];
    p_out = p ? slave_out(p) : 0;
    m_in = gather(s, b->out, p_out, 0);
    if (s->ssi_selected && b->ssi_bits) {
        uint8_t ans = ssi_answer(s, b);

        if (ph->lanes == 1) {
            ans = (ans & 1) << master_in_lane(s, ph);
        }
        m_in = gather(s, b->out, p_out, ans);
    }
    if (p) {
        p_in = gather(p, p_out, b->out, 0);
        slave_in(p, p_in);
    }
    master_take(s, ph, m_in);
    s->ring_head = (s->ring_head + 1) % ESP32P4_SPI_RING;
    s->ring_num--;
    return true;
}


static uint32_t mreg_mask(ESP32P4SpiState *s, unsigned i)
{
    switch (i) {
    case ESP32P4_SPI_M_ADDR:
        return UINT32_MAX;
    case ESP32P4_SPI_M_CTRL:
        return ctrl_mask(s);
    case ESP32P4_SPI_M_CLOCK:
        return clock_mask();
    case ESP32P4_SPI_M_USER:
        return user_mask(s);
    case ESP32P4_SPI_M_USER1:
        return USER1_MASK;
    case ESP32P4_SPI_M_USER2:
        return USER2_MASK;
    case ESP32P4_SPI_M_MS_DLEN:
        return R_MS_DLEN_MS_DATA_BITLEN_MASK;
    case ESP32P4_SPI_M_MISC:
        return misc_mask(s);
    case ESP32P4_SPI_M_DIN_MODE:
        return din_mode_mask(s);
    case ESP32P4_SPI_M_DIN_NUM:
        return din_num_mask(s);
    case ESP32P4_SPI_M_DOUT_MODE:
        return dout_mode_mask(s);
    default:
        g_assert_not_reached();
    }
}

static void spi_write_dma_conf(ESP32P4SpiState *s, uint32_t v);

/*
 * A register a CONF buffer names, bit @bit of its bitmap (TRM table
 * 46.5-13): written on both sides of the clock domains at once.
 */
static void conf_apply(ESP32P4SpiState *s, unsigned bit, uint32_t v)
{
    if (bit <= ESP32P4_SPI_M_DOUT_MODE) {
        uint32_t *reg = mreg_src(s, bit);

        *reg = v & mreg_mask(s, bit);
        s->mreg[bit] = *reg;
        s->x[bit] = *reg;
        return;
    }
    switch (bit) {
    case 11:
        spi_write_dma_conf(s, v);
        break;
    case 12:
        s->int_ena = v & int_mask(s);
        spi_update_irq(s);
        break;
    case 13:
        s->int_raw &= ~v;
        spi_update_irq(s);
        break;
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.spi]
 * The CONF state of a segmented transfer: take the bitmap and the
 * registers it names from the TX DMA, applying each as it comes. False if
 * it must wait for the DMA. Once the DMA has nothing more to give, the
 * registers not yet reached keep their values; a bitmap not had at all
 * fails the magic check.
 */
static bool master_conf(ESP32P4SpiState *s)
{
    for (;;) {
        uint8_t byte;

        if (s->conf_word) {
            if (CONF_MAGIC(s->conf_bitmap) !=
                FIELD_EX32(s->slave, SLAVE, DMA_SEG_MAGIC_VALUE)) {
                return true;
            }
            while (s->conf_bit < CONF_REGS &&
                   !(s->conf_bitmap & BIT(s->conf_bit))) {
                s->conf_bit++;
            }
            if (s->conf_bit == CONF_REGS) {
                return true;
            }
        }
        if (!txf_pop(s, &byte)) {
            if (tx_waits(s)) {
                return false;
            }
            qemu_log_mask(LOG_GUEST_ERROR,
                          "esp32p4_spi: SPI%u: the TX DMA ran out in a "
                          "CONF buffer\n", s->host);
            return true;
        }
        s->conf_val |= (uint32_t)byte << (8 * s->conf_byte);
        if (++s->conf_byte < 4) {
            continue;
        }
        if (s->conf_word++ == 0) {
            s->conf_bitmap = s->conf_val;
        } else {
            conf_apply(s, s->conf_bit++, s->conf_val);
        }
        s->conf_byte = 0;
        s->conf_val = 0;
    }
}

static bool conf_ok(ESP32P4SpiState *s)
{
    return s->conf_word &&
           CONF_MAGIC(s->conf_bitmap) ==
           FIELD_EX32(s->slave, SLAVE, DMA_SEG_MAGIC_VALUE);
}

/* The lines as a state of the master's begins. */
static void master_enter(ESP32P4SpiState *s)
{
    switch (s->ph[s->cur].kind) {
    case PH_PREP:
        s->cs_on = true;
        s->tx_err_raised = false;
        break;
    case PH_CONF:
        s->cs_on = false;
        break;
    default:
        break;
    }
    master_drive_lines(s);
    spi_routes_changed(s);
}

/*
 * [spec:nuos:req:emu.esp32p4.spi]
 * The end of a master transfer: SPI_USR clears; data the RX DMA took ends
 * its frame unless SPI_RX_EOF_EN ended it by length.
 */
static void master_finish(ESP32P4SpiState *s, uint32_t ints)
{
    if (s->rx_had_data && !(s->dma_conf & R_DMA_CONF_RX_EOF_EN_MASK)) {
        rx_eof(s);
    }
    s->rx_had_data = false;
    s->busy = false;
    s->waiting = false;
    s->stalled = false;
    s->sct = false;
    s->abort = false;
    s->cs_on = false;
    s->nph = 0;
    s->cmd &= ~R_CMD_USR_MASK;
    timer_del(s->timer);
    rxf_drain(s);
    master_drive_lines(s);
    spi_raise(s, ints);
}

/*
 * The end of the DONE state: CS goes inactive (unless kept active) and
 * SPI_TRANS_DONE_INT rises; a segmented transfer goes on to its next
 * CONF state or ends with SPI_DMA_SEG_TRANS_DONE_INT.
 */
static void master_segment_end(ESP32P4SpiState *s)
{
    s->cs_on = false;
    s->cs_held = FIELD_EX32(s->x[ESP32P4_SPI_M_MISC], MISC, CS_KEEP_ACTIVE);
    master_drive_lines(s);
    spi_raise(s, INT_TRANS_DONE);
    if (!s->sct) {
        master_finish(s, 0);
        return;
    }
    if (!s->abort &&
        FIELD_EX32(s->x[ESP32P4_SPI_M_USER], USER, USR_CONF_NXT)) {
        master_plan_conf(s);
        s->entered = false;
        return;
    }
    master_finish(s, INT_DMA_SEG_TRANS_DONE);
}

/* The length of the current step: a timed state, or a cycle. */
static uint32_t step_halves(ESP32P4SpiState *s)
{
    ESP32P4SpiPhase *ph = &s->ph[s->cur];

    return ph->beats ? 2 : ph->halves;
}

/*
 * [spec:nuos:req:emu.esp32p4.spi]
 * Run the master's transfer up to the present: every step whose time has
 * come, then a timer for the next batch of them.
 */
static void master_run(ESP32P4SpiState *s)
{
    if (s->running) {
        return;
    }
    s->running = true;
    while (s->busy && !s->waiting && !s->stalled && master_clocks(s)) {
        ESP32P4SpiPhase *ph = &s->ph[s->cur];
        int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        uint32_t step = step_halves(s);
        ESP32P4ClkPeriod half;
        int64_t end;

        if (!s->entered) {
            master_enter(s);
            s->entered = true;
        }
        end = master_half_time(s, s->h + step);
        if (end > now) {
            uint32_t ahead = ph->beats ? 2 * MIN(BATCH_BEATS,
                                                 ph->beats - s->cur_beat)
                                       : step;

            timer_mod(s->timer, master_half_time(s, s->h + ahead));
            break;
        }
        if (!ph->beats) {
            if (ph->kind == PH_CONF) {
                if (!master_conf(s)) {
                    s->stalled = true;
                    break;
                }
                if (!conf_ok(s)) {
                    master_finish(s, INT_SEG_MAGIC_ERR |
                                     INT_DMA_SEG_TRANS_DONE);
                    break;
                }
                /*
                 * The segment runs on the clock its CONF buffer set,
                 * from the CONF state's exact end if that is the clock
                 * the last ran on, and otherwise from the clock's edge
                 * at it (the first in the ns before it).
                 */
                half = master_half(s);
                if (esp32p4_clk_period_eq(half, s->half)) {
                    s->h += step;
                } else {
                    s->half = half;
                    s->h = 0;
                    master_restart(s, end - 1, 0);
                }
                master_plan(s);
                s->entered = false;
                continue;
            }
            s->h += step;
            if (ph->kind == PH_DONE) {
                master_segment_end(s);
                continue;
            }
            s->cur++;
            s->cur_beat = 0;
            s->entered = false;
            continue;
        }
        if (!master_beat(s)) {
            s->stalled = true;
            break;
        }
        s->h += 2;
        if (++s->cur_beat == ph->beats) {
            s->cur++;
            s->cur_beat = 0;
            s->entered = false;
        }
        if (s->abort && s->ph[s->cur].kind != PH_DONE) {
            master_abort(s);
            s->entered = false;
        }
    }
    s->running = false;
}

static void master_timer_cb(void *opaque)
{
    master_run(opaque);
}

/* Whether the transfer has a state that sends DMA data. */
static bool master_sends_dma(ESP32P4SpiState *s)
{
    for (unsigned i = 0; i < s->nph; i++) {
        if (phase_has_dma_tx(s, &s->ph[i])) {
            return true;
        }
    }
    return false;
}

/*
 * [spec:nuos:req:emu.esp32p4.spi]
 * Start the transfer SPI_USR asked for, once clk_spi_mst runs and, with
 * SPI_MST_FD_WAIT_DMA_TX_DATA, once the DMA has given data to send.
 */
static void master_try_start(ESP32P4SpiState *s)
{
    if (!s->busy || !s->waiting || !master_clocks(s)) {
        return;
    }
    memcpy(s->x, s->mreg, sizeof(s->x));
    if (s->sct) {
        master_plan_conf(s);
    } else {
        master_plan(s);
        if ((s->slave & R_SLAVE_MST_FD_WAIT_DMA_TX_DATA_MASK) &&
            master_sends_dma(s)) {
            txf_fill(s);
            if (!s->txf.num) {
                return;
            }
        }
    }
    s->half = master_half(s);
    s->waiting = false;
    s->stalled = false;
    s->abort = false;
    s->entered = false;
    s->rx_had_data = false;
    s->h = 0;
    master_restart(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL), 0);
    master_run(s);
}

/*
 * The DMA has data or room again: a transfer that waited for it goes on.
 * The wait delays the rest of the transfer by as long as it lasted.
 */
static void master_kick(ESP32P4SpiState *s)
{
    int64_t now, end;

    if (!s->busy) {
        return;
    }
    if (s->waiting) {
        master_try_start(s);
        return;
    }
    if (!s->stalled) {
        return;
    }
    s->stalled = false;
    now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    end = master_half_time(s, s->h + step_halves(s));
    if (now > end) {
        master_restart(s, now, s->h + step_halves(s));
    }
    master_run(s);
}

static void spi_tx_ready(void *opaque)
{
    ESP32P4SpiState *s = opaque;

    txf_fill(s);
    master_kick(s);
    if (s->peer) {
        master_kick(s->peer);
    }
}

static void spi_rx_ready(void *opaque)
{
    ESP32P4SpiState *s = opaque;

    rxf_drain(s);
    master_kick(s);
    if (s->peer) {
        master_kick(s->peer);
    }
}

static void spi_update_cb(void *opaque);

/*
 * [spec:nuos:req:emu.esp32p4.spi]
 * Take what the controller has due by now, earliest first: SPI_UPDATE's
 * copy, then the master's steps whose time has come (master_run()).
 * Without icount their timers can run well after, while software polls
 * CMD's USR and UPDATE, DMA_INT_RAW's TRANS_DONE, or reads W0-W15 and
 * GDMA's channels for what the transfer received: a register access, and
 * GDMA's, first take them. Only a timer that has come is run, so that the
 * timers' order among others due at the same time stays as it was.
 */
static void spi_catch_up(ESP32P4SpiState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int64_t upd = timer_expire_time_ns(s->update_timer);
    int64_t step = timer_expire_time_ns(s->timer);
    bool upd_due = timer_pending(s->update_timer) && upd <= now;
    bool step_due = timer_pending(s->timer) && step <= now;

    if (upd_due && (!step_due || upd <= step)) {
        timer_del(s->update_timer);
        esp32_line_run_at(upd, spi_update_cb, s);
    }
    if (step_due) {
        timer_del(s->timer);
        master_run(s);
    }
    if (upd_due && timer_pending(s->update_timer)) {
        timer_del(s->update_timer);
        esp32_line_run_at(upd, spi_update_cb, s);
    }
}

static void spi_dma_sync(void *opaque, int64_t when)
{
    spi_catch_up(opaque);
}

static const ESP32P4GdmaPeriphOps spi_dma_ops = {
    .tx_ready = spi_tx_ready,
    .rx_ready = spi_rx_ready,
    .sync = spi_dma_sync,
};

/* SPI_UPDATE */

static void spi_update_cb(void *opaque)
{
    ESP32P4SpiState *s = opaque;

    s->update_pending = false;
    for (unsigned i = 0; i < ESP32P4_SPI_M_CONF_BITLEN; i++) {
        s->mreg[i] = *mreg_src(s, i);
    }
    s->mreg[ESP32P4_SPI_M_CONF_BITLEN] = FIELD_EX32(s->cmd, CMD,
                                                    CONF_BITLEN);
    if (!slave_mode(s)) {
        master_drive_lines(s);
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.spi]
 * SPI_UPDATE copies the configuration into clk_spi_mst's domain two of
 * its cycles later, and reads 1 until then; with clk_spi_mst stopped it
 * waits for the clock.
 */
static void spi_schedule_update(ESP32P4SpiState *s)
{
    if (s->update_pending && clock_is_enabled(s->mst_clk) &&
        !timer_pending(s->update_timer)) {
        int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        ESP32P4ClkEdges e = esp32p4_clk_edges_or_at(s->mst_clk, now);

        timer_mod(s->update_timer, esp32p4_clk_edges_after(&e, now, 2));
    }
}

/* Clocks */

static void spi_mst_clk_update(void *opaque, ClockEvent event)
{
    ESP32P4SpiState *s = opaque;
    ESP32P4ClkPeriod half;

    if (event == ClockPreUpdate) {
        if (s->busy && !s->waiting) {
            master_run(s);
        }
        return;
    }
    if (!clock_is_enabled(s->mst_clk)) {
        timer_del(s->update_timer);
    }
    spi_schedule_update(s);
    if (!s->busy) {
        return;
    }
    if (s->waiting) {
        master_try_start(s);
        return;
    }
    half = master_half(s);
    if (esp32p4_clk_period_running(half)) {
        s->half = half;
        master_resume(s);
    }
}

/*
 * A transfer paused with the controller's clocks goes on from where it
 * was, as they start again.
 */
static void master_resume(ESP32P4SpiState *s)
{
    if (!s->busy) {
        return;
    }
    if (s->waiting) {
        master_try_start(s);
        return;
    }
    master_restart(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL), s->h);
    s->stalled = false;
    master_run(s);
}

static void spi_sys_clk_update(void *opaque, ClockEvent event)
{
    ESP32P4SpiState *s = opaque;

    if (!sys_running(s)) {
        return;
    }
    txf_fill(s);
    rxf_drain(s);
    master_resume(s);
    if (s->peer) {
        master_kick(s->peer);
    }
}

/* The matrix's input signals. */
static void spi_sig_in(void *opaque, int line, int level)
{
    ESP32P4SpiState *s = opaque;

    s->in_level[line] = level != 0;
    if (line == ESP32P4_SPI_LINE_CS0 && slave_mode(s)) {
        slave_cs(s, !level ^ FIELD_EX32(s->misc, MISC, SLAVE_CS_POL));
    }
}

static void spi_route_changed(Notifier *n, void *data)
{
    ESP32P4SpiState *s = container_of(n, ESP32P4SpiState, route_notifier);

    spi_routes_changed(s);
    if (has_devices(s) && !slave_mode(s)) {
        spi_update_devices(s);
    }
}

/* Registers */

/* The register at @addr as read, which has no side effects. */
static bool spi_read32(ESP32P4SpiState *s, hwaddr addr, uint32_t *v)
{
    switch (addr) {
    case A_CMD:
        *v = s->cmd | (s->update_pending ? R_CMD_UPDATE_MASK : 0);
        return true;
    case A_ADDR:
        *v = s->addr;
        return true;
    case A_CTRL:
        *v = s->ctrl;
        return true;
    case A_CLOCK:
        *v = s->clock;
        return true;
    case A_USER:
        *v = s->user;
        return true;
    case A_USER1:
        *v = s->user1;
        return true;
    case A_USER2:
        *v = s->user2;
        return true;
    case A_MS_DLEN:
        *v = s->ms_dlen;
        return true;
    case A_MISC:
        *v = s->misc;
        return true;
    case A_DIN_MODE:
        *v = s->din_mode;
        return true;
    case A_DIN_NUM:
        *v = s->din_num;
        return true;
    case A_DOUT_MODE:
        *v = s->dout_mode;
        return true;
    case A_DMA_CONF:
        /*
         * The FIFOs' readiness: the TX FIFO has no DMA data, the RX FIFO
         * takes none (it is full, or SPI_DMA_RX_ENA is clear).
         */
        *v = s->dma_conf |
             (s->txf.num ? 0 : R_DMA_CONF_DMA_OUTFIFO_EMPTY_MASK) |
             ((s->dma_conf & R_DMA_CONF_DMA_RX_ENA_MASK) &&
              !fifo_full(&s->rxf) ? 0 : R_DMA_CONF_DMA_INFIFO_FULL_MASK);
        return true;
    case A_DMA_INT_ENA:
        *v = s->int_ena;
        return true;
    case A_DMA_INT_CLR:
    case A_DMA_INT_SET:
        *v = 0;
        return true;
    case A_DMA_INT_RAW:
        *v = s->int_raw;
        return true;
    case A_DMA_INT_ST:
        *v = s->int_raw & s->int_ena;
        return true;
    case A_W0 ... A_W15:
        *v = s->w[(addr - A_W0) / 4];
        return true;
    case A_SLAVE:
        *v = s->slave;
        return true;
    case A_SLAVE1:
        *v = s->slave1;
        return true;
    case A_CLK_GATE:
        *v = s->clk_gate;
        return true;
    case A_DATE:
        *v = s->date;
        return true;
    default:
        *v = 0;
        return false;
    }
}

static uint64_t esp32p4_spi_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4SpiState *s = opaque;
    uint32_t v;

    spi_catch_up(s);
    if (!spi_read32(s, addr & ~3, &v)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32p4_spi: SPI%u: read of 0x%" HWADDR_PRIx
                      ", which has no register\n", s->host, addr);
    }
    return extract32(v, (addr & 3) * 8, size * 8);
}

/*
 * [spec:nuos:req:emu.esp32p4.spi]
 * SPI_DMA_CONF: the AFIFO resets act as written; the TX FIFO takes DMA
 * data while SPI_DMA_TX_ENA is set.
 */
static void spi_write_dma_conf(ESP32P4SpiState *s, uint32_t v)
{
    s->dma_conf = v & DMA_CONF_RW;
    if (v & R_DMA_CONF_RX_AFIFO_RST_MASK) {
        fifo_reset(&s->rxf);
        s->rx_bits = 0;
        s->rx_drop = false;
    }
    if (v & R_DMA_CONF_DMA_AFIFO_RST_MASK) {
        fifo_reset(&s->txf);
    }
    txf_fill(s);
    rxf_drain(s);
}

/*
 * [spec:nuos:req:emu.esp32p4.spi]
 * SPI_SLAVE: the mode switch, which turns the bus signals around, and the
 * soft reset of the slave's state.
 */
static void spi_write_slave(ESP32P4SpiState *s, uint32_t v)
{
    bool was = slave_mode(s);

    s->slave = (s->slave & R_SLAVE_SLV_LAST_BYTE_STRB_MASK) |
               (v & slave_mask(s));
    if (slave_mode(s) != was) {
        if (s->busy) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "esp32p4_spi: SPI%u: mode changed during a "
                          "master transfer\n", s->host);
        }
        s->s_cs = false;
        s->s_state = S_IDLE;
        spi_drive_lines(s);
        spi_routes_changed(s);
    }
    if (v & R_SLAVE_SOFT_RESET_MASK) {
        s->s_cs = false;
        s->s_state = S_IDLE;
        spi_drive_lines(s);
    }
    /* A master that waited for this slave's DMA no longer does. */
    if (s->peer) {
        master_kick(s->peer);
    }
}

static void spi_write32(ESP32P4SpiState *s, hwaddr addr, uint32_t v)
{
    switch (addr) {
    case A_CMD:
        s->cmd = (s->cmd & R_CMD_USR_MASK) |
                 (v & cmd_mask(s) & ~R_CMD_USR_MASK);
        if (v & R_CMD_UPDATE_MASK) {
            s->update_pending = true;
            spi_schedule_update(s);
        }
        if (v & R_CMD_USR_MASK) {
            if (slave_mode(s)) {
                s->cmd |= R_CMD_USR_MASK;
            } else if (s->busy) {
                qemu_log_mask(LOG_GUEST_ERROR,
                              "esp32p4_spi: SPI%u: SPI_USR set during a "
                              "transfer\n", s->host);
            } else {
                s->cmd |= R_CMD_USR_MASK;
                s->busy = true;
                s->waiting = true;
                s->sct = spi2(s) && (s->slave & R_SLAVE_USR_CONF_MASK);
                master_try_start(s);
            }
        } else if (s->cmd & R_CMD_USR_MASK) {
            if (slave_mode(s)) {
                s->cmd &= ~R_CMD_USR_MASK;
            } else {
                qemu_log_mask(LOG_GUEST_ERROR,
                              "esp32p4_spi: SPI%u: clearing SPI_USR does not "
                              "stop a transfer\n", s->host);
            }
        }
        break;
    case A_ADDR:
        s->addr = v;
        break;
    case A_CTRL:
        s->ctrl = v & ctrl_mask(s);
        if (slave_mode(s)) {
            slave_drive_lines(s);
        }
        break;
    case A_CLOCK:
        s->clock = v & clock_mask();
        break;
    case A_USER:
        s->user = v & user_mask(s);
        break;
    case A_USER1:
        s->user1 = v & USER1_MASK;
        break;
    case A_USER2:
        s->user2 = v & USER2_MASK;
        break;
    case A_MS_DLEN:
        s->ms_dlen = v & R_MS_DLEN_MS_DATA_BITLEN_MASK;
        break;
    case A_MISC:
        s->misc = v & misc_mask(s);
        if (slave_mode(s)) {
            slave_drive_lines(s);
        }
        break;
    case A_DIN_MODE:
        s->din_mode = v & din_mode_mask(s);
        break;
    case A_DIN_NUM:
        s->din_num = v & din_num_mask(s);
        break;
    case A_DOUT_MODE:
        s->dout_mode = v & dout_mode_mask(s);
        break;
    case A_DMA_CONF:
        spi_write_dma_conf(s, v);
        break;
    case A_DMA_INT_ENA:
        s->int_ena = v & int_mask(s);
        spi_update_irq(s);
        break;
    case A_DMA_INT_CLR:
        s->int_raw &= ~v;
        spi_update_irq(s);
        break;
    case A_DMA_INT_SET:
        spi_raise(s, v);
        break;
    case A_DMA_INT_RAW:
    case A_DMA_INT_ST:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32p4_spi: SPI%u: write to read-only register "
                      "0x%" HWADDR_PRIx "\n", s->host, addr);
        break;
    case A_W0 ... A_W15:
        s->w[(addr - A_W0) / 4] = v;
        break;
    case A_SLAVE:
        spi_write_slave(s, v);
        break;
    case A_SLAVE1:
        s->slave1 = v;
        break;
    case A_CLK_GATE:
        s->clk_gate = v & 0x7;
        break;
    case A_DATE:
        s->date = v & 0x0fffffff;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32p4_spi: SPI%u: write to 0x%" HWADDR_PRIx
                      ", which has no register\n", s->host, addr);
        break;
    }
}

/*
 * A narrower write merges into the register as read, the bits that act
 * when written as 1 (SPI_UPDATE, SPI_USR) taken as 0.
 */
static void esp32p4_spi_write(void *opaque, hwaddr addr, uint64_t value,
                              unsigned size)
{
    ESP32P4SpiState *s = opaque;
    hwaddr a = addr & ~3;
    uint32_t v = value;

    spi_catch_up(s);
    if (size < 4) {
        uint32_t cur;

        spi_read32(s, a, &cur);
        if (a == A_CMD) {
            cur &= ~(R_CMD_UPDATE_MASK | R_CMD_USR_MASK);
        }
        v = deposit32(cur, (addr & 3) * 8, size * 8, value);
    }
    spi_write32(s, a, v);
}

static const MemoryRegionOps esp32p4_spi_ops = {
    .read = esp32p4_spi_read,
    .write = esp32p4_spi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

/* Reset */

static void esp32p4_spi_reset_hold(Object *obj, ResetType type)
{
    ESP32P4SpiState *s = ESP32P4_SPI(obj);

    timer_del(s->timer);
    timer_del(s->update_timer);
    s->cmd = 0;
    s->addr = 0;
    s->ctrl = CTRL_RESET;
    s->clock = CLOCK_RESET;
    s->user = USER_RESET;
    s->user1 = USER1_RESET;
    s->user2 = USER2_RESET;
    s->ms_dlen = 0;
    s->misc = misc_reset(s);
    s->din_mode = 0;
    s->din_num = 0;
    s->dout_mode = 0;
    s->dma_conf = 0;
    s->int_ena = 0;
    s->int_raw = 0;
    memset(s->w, 0, sizeof(s->w));
    s->slave = spi2(s) ? SLAVE_MAGIC_RESET : 0;
    s->slave1 = 0;
    s->clk_gate = 0;
    s->date = DATE_RESET;
    for (unsigned i = 0; i < ESP32P4_SPI_M_CONF_BITLEN; i++) {
        s->mreg[i] = *mreg_src(s, i);
    }
    s->mreg[ESP32P4_SPI_M_CONF_BITLEN] = 0;
    s->update_pending = false;
    fifo_reset(&s->txf);
    fifo_reset(&s->rxf);
    s->tx_last = 0;
    s->rx_drop = false;
    s->rx_bits = 0;
    s->busy = false;
    s->waiting = false;
    s->stalled = false;
    s->sct = false;
    s->abort = false;
    s->running = false;
    s->entered = false;
    s->cs_on = false;
    s->cs_held = false;
    s->nph = 0;
    s->cur = 0;
    s->cur_beat = 0;
    s->ring_num = 0;
    s->rx_had_data = false;
    s->src_valid = false;
    s->s_cs = false;
    s->s_state = S_IDLE;
}

static void esp32p4_spi_reset_exit(Object *obj, ResetType type)
{
    ESP32P4SpiState *s = ESP32P4_SPI(obj);

    s->lines_driven = false;
    spi_drive_lines(s);
    spi_update_irq(s);
    if (s->peer) {
        master_kick(s->peer);
    }
}

static void esp32p4_spi_init(Object *obj)
{
    ESP32P4SpiState *s = ESP32P4_SPI(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    DeviceState *dev = DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32p4_spi_ops, s,
                          TYPE_ESP32P4_SPI, ESP32P4_SPI_REGS_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_out_named(dev, s->sig_out, ESP32P4_SPI_SIG_OUT,
                             ESP32P4_SPI_LINES);
    qdev_init_gpio_out_named(dev, s->sig_oe, ESP32P4_SPI_SIG_OE,
                             ESP32P4_SPI_LINES);
    qdev_init_gpio_in_named(dev, spi_sig_in, ESP32P4_SPI_SIG_IN,
                            ESP32P4_SPI_LINES);
    s->mst_clk = qdev_init_clock_in(dev, "mst", spi_mst_clk_update, s,
                                    ClockPreUpdate | ClockUpdate);
    s->sys_clk = qdev_init_clock_in(dev, "sys", spi_sys_clk_update, s,
                                    ClockUpdate);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, master_timer_cb, s);
    s->update_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, spi_update_cb, s);
}

static void esp32p4_spi_realize(DeviceState *dev, Error **errp)
{
    ESP32P4SpiState *s = ESP32P4_SPI(dev);
    g_autofree char *bus = g_strdup_printf("spi%u", s->host);

    if (s->host != 2 && s->host != 3) {
        error_setg(errp, "%s: host must be 2 or 3", TYPE_ESP32P4_SPI);
        return;
    }
    if (!s->gdma || !s->gpio) {
        error_setg(errp, "%s: the 'gdma' and 'gpio' links must be set",
                   TYPE_ESP32P4_SPI);
        return;
    }
    s->bus = ssi_create_bus(dev, bus);
    esp32p4_gdma_attach(s->gdma, periph(s), &spi_dma_ops, s);
    s->route_notifier.notify = spi_route_changed;
    esp32p4_gpio_add_route_notifier(s->gpio, &s->route_notifier);
}

static void esp32p4_spi_finalize(Object *obj)
{
    ESP32P4SpiState *s = ESP32P4_SPI(obj);

    timer_free(s->timer);
    timer_free(s->update_timer);
}

static int esp32p4_spi_post_load(void *opaque, int version_id)
{
    ESP32P4SpiState *s = opaque;

    s->src_valid = false;
    s->running = false;
    return 0;
}

static const VMStateDescription vmstate_esp32p4_spi_fifo = {
    .name = "esp32p4-spi-fifo",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8_ARRAY(data, ESP32P4SpiFifo, ESP32P4_SPI_AFIFO),
        VMSTATE_UINT8_ARRAY(flags, ESP32P4SpiFifo, ESP32P4_SPI_AFIFO),
        VMSTATE_UINT32(head, ESP32P4SpiFifo),
        VMSTATE_UINT32(num, ESP32P4SpiFifo),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_esp32p4_spi_phase = {
    .name = "esp32p4-spi-phase",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8(kind, ESP32P4SpiPhase),
        VMSTATE_UINT8(lanes, ESP32P4SpiPhase),
        VMSTATE_UINT8(ssi_lanes, ESP32P4SpiPhase),
        VMSTATE_UINT32(bits, ESP32P4SpiPhase),
        VMSTATE_UINT32(beats, ESP32P4SpiPhase),
        VMSTATE_UINT32(halves, ESP32P4SpiPhase),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_esp32p4_spi_beat = {
    .name = "esp32p4-spi-beat",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8(phase, ESP32P4SpiBeat),
        VMSTATE_UINT8(out, ESP32P4SpiBeat),
        VMSTATE_UINT8(ssi_bits, ESP32P4SpiBeat),
        VMSTATE_UINT32(ssi_pos, ESP32P4SpiBeat),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_esp32p4_spi = {
    .name = TYPE_ESP32P4_SPI,
    .version_id = 3,
    .minimum_version_id = 3,
    .post_load = esp32p4_spi_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(cmd, ESP32P4SpiState),
        VMSTATE_UINT32(addr, ESP32P4SpiState),
        VMSTATE_UINT32(ctrl, ESP32P4SpiState),
        VMSTATE_UINT32(clock, ESP32P4SpiState),
        VMSTATE_UINT32(user, ESP32P4SpiState),
        VMSTATE_UINT32(user1, ESP32P4SpiState),
        VMSTATE_UINT32(user2, ESP32P4SpiState),
        VMSTATE_UINT32(ms_dlen, ESP32P4SpiState),
        VMSTATE_UINT32(misc, ESP32P4SpiState),
        VMSTATE_UINT32(din_mode, ESP32P4SpiState),
        VMSTATE_UINT32(din_num, ESP32P4SpiState),
        VMSTATE_UINT32(dout_mode, ESP32P4SpiState),
        VMSTATE_UINT32(dma_conf, ESP32P4SpiState),
        VMSTATE_UINT32(int_ena, ESP32P4SpiState),
        VMSTATE_UINT32(int_raw, ESP32P4SpiState),
        VMSTATE_UINT32_ARRAY(w, ESP32P4SpiState, 16),
        VMSTATE_UINT32(slave, ESP32P4SpiState),
        VMSTATE_UINT32(slave1, ESP32P4SpiState),
        VMSTATE_UINT32(clk_gate, ESP32P4SpiState),
        VMSTATE_UINT32(date, ESP32P4SpiState),
        VMSTATE_UINT32_ARRAY(mreg, ESP32P4SpiState, ESP32P4_SPI_M_REGS),
        VMSTATE_BOOL(update_pending, ESP32P4SpiState),
        VMSTATE_UINT8_ARRAY(out_level, ESP32P4SpiState, ESP32P4_SPI_LINES),
        VMSTATE_UINT8_ARRAY(oe_level, ESP32P4SpiState, ESP32P4_SPI_LINES),
        VMSTATE_UINT8_ARRAY(in_level, ESP32P4SpiState, ESP32P4_SPI_LINES),
        VMSTATE_BOOL(lines_driven, ESP32P4SpiState),
        VMSTATE_STRUCT(txf, ESP32P4SpiState, 1, vmstate_esp32p4_spi_fifo,
                       ESP32P4SpiFifo),
        VMSTATE_STRUCT(rxf, ESP32P4SpiState, 1, vmstate_esp32p4_spi_fifo,
                       ESP32P4SpiFifo),
        VMSTATE_UINT8(tx_last, ESP32P4SpiState),
        VMSTATE_BOOL(rx_drop, ESP32P4SpiState),
        VMSTATE_UINT32(rx_bits, ESP32P4SpiState),
        VMSTATE_BOOL(busy, ESP32P4SpiState),
        VMSTATE_BOOL(waiting, ESP32P4SpiState),
        VMSTATE_BOOL(stalled, ESP32P4SpiState),
        VMSTATE_BOOL(sct, ESP32P4SpiState),
        VMSTATE_BOOL(abort, ESP32P4SpiState),
        VMSTATE_BOOL(entered, ESP32P4SpiState),
        VMSTATE_BOOL(cs_on, ESP32P4SpiState),
        VMSTATE_BOOL(cs_held, ESP32P4SpiState),
        VMSTATE_UINT32_ARRAY(x, ESP32P4SpiState, ESP32P4_SPI_M_REGS),
        VMSTATE_STRUCT_ARRAY(ph, ESP32P4SpiState, ESP32P4_SPI_PHASES, 1,
                             vmstate_esp32p4_spi_phase, ESP32P4SpiPhase),
        VMSTATE_UINT8(nph, ESP32P4SpiState),
        VMSTATE_UINT8(cur, ESP32P4SpiState),
        VMSTATE_UINT32(cur_beat, ESP32P4SpiState),
        VMSTATE_ESP32P4_CLK_PERIOD(half, ESP32P4SpiState),
        VMSTATE_ESP32P4_CLK_EDGES(half_e, ESP32P4SpiState),
        VMSTATE_INT64(h0, ESP32P4SpiState),
        VMSTATE_UINT64(h, ESP32P4SpiState),
        VMSTATE_UINT8(gen, ESP32P4SpiState),
        VMSTATE_UINT32(gen_beat, ESP32P4SpiState),
        VMSTATE_UINT8(gen_byte, ESP32P4SpiState),
        VMSTATE_UINT32(gen_n, ESP32P4SpiState),
        VMSTATE_STRUCT_ARRAY(ring, ESP32P4SpiState, ESP32P4_SPI_RING, 1,
                             vmstate_esp32p4_spi_beat, ESP32P4SpiBeat),
        VMSTATE_UINT32(ring_head, ESP32P4SpiState),
        VMSTATE_UINT32(ring_num, ESP32P4SpiState),
        VMSTATE_UINT32(ssi_gen_bits, ESP32P4SpiState),
        VMSTATE_UINT32(ssi_sent_bits, ESP32P4SpiState),
        VMSTATE_UINT32(ssi_acc, ESP32P4SpiState),
        VMSTATE_UINT8_ARRAY(ssi_resp, ESP32P4SpiState, 4),
        VMSTATE_UINT8(din_byte, ESP32P4SpiState),
        VMSTATE_UINT32(din_n, ESP32P4SpiState),
        VMSTATE_BOOL(rx_had_data, ESP32P4SpiState),
        VMSTATE_BOOL(tx_err_raised, ESP32P4SpiState),
        VMSTATE_BOOL(ssi_selected, ESP32P4SpiState),
        VMSTATE_UINT32(conf_bitmap, ESP32P4SpiState),
        VMSTATE_UINT8(conf_word, ESP32P4SpiState),
        VMSTATE_UINT8(conf_byte, ESP32P4SpiState),
        VMSTATE_UINT8(conf_bit, ESP32P4SpiState),
        VMSTATE_UINT32(conf_val, ESP32P4SpiState),
        VMSTATE_BOOL(s_cs, ESP32P4SpiState),
        VMSTATE_UINT8(s_state, ESP32P4SpiState),
        VMSTATE_UINT32(s_bits, ESP32P4SpiState),
        VMSTATE_UINT32(s_data_bits, ESP32P4SpiState),
        VMSTATE_UINT8(s_cmd, ESP32P4SpiState),
        VMSTATE_UINT8(s_addr, ESP32P4SpiState),
        VMSTATE_UINT8(s_rx, ESP32P4SpiState),
        VMSTATE_UINT8(s_tx, ESP32P4SpiState),
        VMSTATE_UINT32(s_n, ESP32P4SpiState),
        VMSTATE_UINT8(s_lanes, ESP32P4SpiState),
        VMSTATE_UINT8(s_addr_lanes, ESP32P4SpiState),
        VMSTATE_UINT8(s_data_lanes, ESP32P4SpiState),
        VMSTATE_UINT8(s_kind, ESP32P4SpiState),
        VMSTATE_BOOL(s_err, ESP32P4SpiState),
        VMSTATE_TIMER_PTR(timer, ESP32P4SpiState),
        VMSTATE_TIMER_PTR(update_timer, ESP32P4SpiState),
        VMSTATE_CLOCK(mst_clk, ESP32P4SpiState),
        VMSTATE_CLOCK(sys_clk, ESP32P4SpiState),
        VMSTATE_END_OF_LIST()
    },
};

static const Property esp32p4_spi_properties[] = {
    DEFINE_PROP_UINT8("host", ESP32P4SpiState, host, 0),
    DEFINE_PROP_LINK("gdma", ESP32P4SpiState, gdma, TYPE_ESP32P4_GDMA,
                     ESP32P4GdmaState *),
    DEFINE_PROP_LINK("gpio", ESP32P4SpiState, gpio, TYPE_ESP32P4_GPIO,
                     ESP32P4GpioState *),
    DEFINE_PROP_LINK("peer", ESP32P4SpiState, peer, TYPE_ESP32P4_SPI,
                     ESP32P4SpiState *),
};

static void esp32p4_spi_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = esp32p4_spi_realize;
    dc->vmsd = &vmstate_esp32p4_spi;
    device_class_set_props(dc, esp32p4_spi_properties);
    rc->phases.hold = esp32p4_spi_reset_hold;
    rc->phases.exit = esp32p4_spi_reset_exit;
}

/* [spec:nuos:req:emu.esp32p4.spi] */
static const TypeInfo esp32p4_spi_types[] = {
    {
        .name           = TYPE_ESP32P4_SPI,
        .parent         = TYPE_SYS_BUS_DEVICE,
        .instance_size  = sizeof(ESP32P4SpiState),
        .instance_init  = esp32p4_spi_init,
        .instance_finalize = esp32p4_spi_finalize,
        .class_init     = esp32p4_spi_class_init,
    },
};

DEFINE_TYPES(esp32p4_spi_types)
