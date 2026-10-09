#ifndef ESP32_I2C_H
#define ESP32_I2C_H

#include "hw/core/sysbus.h"
#include "qemu/fifo8.h"
#include "hw/i2c/i2c.h"
#include "hw/core/registerfields.h"

#define TYPE_ESP32_I2C "esp32.i2c"
#define Esp32_I2C(obj) OBJECT_CHECK(Esp32I2CState, (obj), TYPE_ESP32_I2C)


#define ESP32_I2C_MEM_SIZE 0x100
#define ESP32_I2C_FIFO_LENGTH 32
#define ESP32_I2C_CMD_COUNT 16

/*
 * The controller's open-drain lines, by the GPIO matrix's signals
 * (I2CEXTn_SCL/SDA, in and out): an output at 0 pulls the line low, at 1
 * releases it. Outputs are named gpio-outs, inputs named gpio-ins.
 */
#define ESP32_I2C_SCL_OUT "esp32-i2c-scl-out"
#define ESP32_I2C_SDA_OUT "esp32-i2c-sda-out"
#define ESP32_I2C_SCL_IN  "esp32-i2c-scl-in"
#define ESP32_I2C_SDA_IN  "esp32-i2c-sda-in"


typedef struct Esp32I2CState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    I2CBus *bus;
    Fifo8 rx_fifo;
    Fifo8 tx_fifo;
    bool trans_ongoing;

    uint32_t ctr_reg;
    uint32_t timeout_reg;
    uint32_t int_ena_reg;
    uint32_t int_raw_reg;
    uint32_t sda_hold_reg;
    uint32_t sda_sample_reg;
    uint32_t high_period_reg;
    uint32_t low_period_reg;
    uint32_t start_hold_reg;
    uint32_t rstart_setup_reg;
    uint32_t stop_hold_reg;
    uint32_t stop_setup_reg;
    uint32_t cmd_reg[ESP32_I2C_CMD_COUNT];

    qemu_irq scl_out;
    qemu_irq sda_out;
    /* The lines as the controller sees them, through the matrix */
    bool scl_in;
    bool sda_in;
    /*
     * The START condition found both lines following the controller's
     * outputs through the pads, so the QEMU bus's devices, which sit on
     * those lines, take part in the transaction.
     */
    bool connected;
} Esp32I2CState;


REG32(I2C_CTR, 0x04);
    FIELD(I2C_CTR, MS_MODE, 4, 1);
    FIELD(I2C_CTR, TRANS_START, 5, 1);

REG32(I2C_STATUS, 0x08);
    FIELD(I2C_STATUS, BUS_BUSY, 4, 1);
    FIELD(I2C_STATUS, RXFIFO_CNT, 8, 6);
    FIELD(I2C_STATUS, TXFIFO_CNT, 18, 6);

REG32(I2C_TIMEOUT, 0x0c);

REG32(I2C_FIFO_CONF, 0x18);
    FIELD(I2C_FIFO_CONF, NONFIFO_EN, 10, 1);
    FIELD(I2C_FIFO_CONF, RX_FIFO_RST, 12, 1);
    FIELD(I2C_FIFO_CONF, TX_FIFO_RST, 13, 1);

REG32(I2C_FIFO_DATA, 0x1c);

REG32(I2C_INT_RAW, 0x20);
    FIELD(I2C_INT_RAW, ACK_ERR, 10, 1);
    FIELD(I2C_INT_RAW, TIME_OUT, 8, 1);
    FIELD(I2C_INT_RAW, ARBITRATION_LOST, 5, 1);
    FIELD(I2C_INT_RAW, TRANS_COMPLETE, 7, 1);
    FIELD(I2C_INT_RAW, END_DETECT, 3, 1);

REG32(I2C_INT_CLR, 0x24);
    FIELD(I2C_INT_CLR, ACK_ERR, 10, 1);
    FIELD(I2C_INT_CLR, TRANS_COMPLETE, 7, 1);
    FIELD(I2C_INT_CLR, END_DETECT, 3, 1);

REG32(I2C_INT_ENA, 0x28);
    FIELD(I2C_INT_ENA, ACK_ERR, 10, 1);
    FIELD(I2C_INT_ENA, TRANS_COMPLETE, 7, 1);
    FIELD(I2C_INT_ENA, END_DETECT, 3, 1);

REG32(I2C_INT_ST, 0x2c);
    FIELD(I2C_INT_ST, ACK_ERR, 10, 1);
    FIELD(I2C_INT_ST, TRANS_COMPLETE, 7, 1);
    FIELD(I2C_INT_ST, END_DETECT, 3, 1);

REG32(I2C_SDA_HOLD, 0x30);
REG32(I2C_SDA_SAMPLE, 0x34);
REG32(I2C_HIGH_PERIOD, 0x38);
REG32(I2C_LOW_PERIOD, 0x00);  // 0x00 is not a typo
REG32(I2C_START_HOLD, 0x40);
REG32(I2C_RSTART_SETUP, 0x44);
REG32(I2C_STOP_HOLD, 0x48);
REG32(I2C_STOP_SETUP, 0x4c);

REG32(I2C_CMD, 0x58);
    FIELD(I2C_CMD, BYTE_NUM, 0, 8);
    FIELD(I2C_CMD, ACK_CHECK_EN, 8, 1);
    FIELD(I2C_CMD, ACK_EXP, 9, 1);
    FIELD(I2C_CMD, ACK_VAL, 10, 1);
    FIELD(I2C_CMD, OPCODE, 11, 3);
    FIELD(I2C_CMD, DONE, 31, 1);
/* 15 more command registers omitted */

/* I2C_CMD.OPCODE values */
typedef enum {
    I2C_OPCODE_RSTART = 0,
    I2C_OPCODE_WRITE  = 1,
    I2C_OPCODE_READ   = 2,
    I2C_OPCODE_STOP   = 3,
    I2C_OPCODE_END    = 4,
} i2c_opcode_t;

#endif /* ESP32_I2C_H */
