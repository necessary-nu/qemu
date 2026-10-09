#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/error-report.h"
#include "hw/i2c/esp32_i2c.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"

static void esp32_i2c_do_transaction(Esp32I2CState *s);
static void esp32_i2c_update_irq(Esp32I2CState *s);

static void esp32_i2c_reset_hold(Object *obj, ResetType type)
{
    Esp32I2CState * s = Esp32_I2C(obj);

    fifo8_reset(&s->rx_fifo);
    fifo8_reset(&s->tx_fifo);
    s->trans_ongoing = false;
    s->connected = false;
    s->ctr_reg = 0;
    s->timeout_reg = 0;
    s->int_ena_reg = 0;
    s->int_raw_reg = 0;
    s->sda_hold_reg = 0;
    s->sda_sample_reg = 0;
    s->high_period_reg = 0;
    s->low_period_reg = 0;
    s->start_hold_reg = 0;
    s->rstart_setup_reg = 0;
    s->stop_hold_reg = 0;
    s->stop_setup_reg = 0;
    memset(s->cmd_reg, 0, sizeof(s->cmd_reg));

    fifo8_reset(&s->tx_fifo);
    fifo8_reset(&s->rx_fifo);
}

static uint32_t esp32_i2c_get_status_reg(Esp32I2CState* s)
{
    uint32_t res = 0;
    res = FIELD_DP32(res, I2C_STATUS, BUS_BUSY, s->trans_ongoing);
    res = FIELD_DP32(res, I2C_STATUS, RXFIFO_CNT, fifo8_num_used(&s->rx_fifo));
    res = FIELD_DP32(res, I2C_STATUS, TXFIFO_CNT, fifo8_num_used(&s->tx_fifo));
    return res;
}

static void esp32_i2c_update_irq(Esp32I2CState * s)
{
    int irq_state = !!(s->int_raw_reg & s->int_ena_reg);
    qemu_set_irq(s->irq, irq_state);
}

static uint64_t esp32_i2c_read(void * opaque, hwaddr addr, unsigned int size)
{
    Esp32I2CState * s = Esp32_I2C(opaque);

    switch(addr) {
    case A_I2C_CTR:
        return s->ctr_reg;
    case A_I2C_STATUS:
        return esp32_i2c_get_status_reg(s);
    case A_I2C_FIFO_DATA: {
        if (fifo8_num_used(&s->rx_fifo) == 0) {
            error_report("esp32_i2c: read I2C FIFO while it is empty");
            return 0xee;
        }
        uint8_t res = fifo8_pop(&s->rx_fifo);
        return res;
    }
    case A_I2C_INT_RAW:
        return s->int_raw_reg;
    case A_I2C_INT_ENA:
        return s->int_ena_reg;
    case A_I2C_INT_ST:
        return s->int_raw_reg & s->int_ena_reg;
    case A_I2C_CMD ... (A_I2C_CMD + (ESP32_I2C_CMD_COUNT - 1) * 4):
        return s->cmd_reg[(addr - A_I2C_CMD) / 4];
    case A_I2C_TIMEOUT:
        return s->timeout_reg;
    case A_I2C_SDA_HOLD:
        return s->sda_hold_reg;
    case A_I2C_SDA_SAMPLE:
        return s->sda_sample_reg;
    case A_I2C_HIGH_PERIOD:
        return s->high_period_reg;
    case A_I2C_LOW_PERIOD:
        return s->low_period_reg;
    case A_I2C_START_HOLD:
        return s->start_hold_reg;
    case A_I2C_RSTART_SETUP:
        return s->rstart_setup_reg;
    case A_I2C_STOP_HOLD:
        return s->stop_hold_reg;
    case A_I2C_STOP_SETUP:
        return s->stop_setup_reg;
    default:
        return 0;
    }
}

static void esp32_i2c_write(void * opaque, hwaddr addr, uint64_t value, unsigned int size)
{
    Esp32I2CState * s = Esp32_I2C(opaque);

    switch(addr) {
    case A_I2C_CTR:
        if (FIELD_EX32(value, I2C_CTR, MS_MODE) != 1) {
            error_report("esp32_i2c: slave mode not implemented");
        }
        if (FIELD_EX32(value, I2C_CTR, TRANS_START)) {
            esp32_i2c_do_transaction(s);
            value &= ~ R_I2C_CTR_TRANS_START_MASK;
        }
        s->ctr_reg = value;
        break;
    case A_I2C_FIFO_CONF:
        if (FIELD_EX32(value, I2C_FIFO_CONF, NONFIFO_EN)) {
            error_report("esp32_i2c: APB mode not implemented");
        }
        if (FIELD_EX32(value, I2C_FIFO_CONF, RX_FIFO_RST)) {
            fifo8_reset(&s->rx_fifo);
        }
        if (FIELD_EX32(value, I2C_FIFO_CONF, TX_FIFO_RST)) {
            fifo8_reset(&s->tx_fifo);
        }
        break;
    case A_I2C_FIFO_DATA:
        if (fifo8_num_free(&s->tx_fifo) == 0) {
            error_report("esp32_i2c: write to I2C TX FIFO while it is full");
        } else {
            fifo8_push(&s->tx_fifo, value);
        }
        break;
    case A_I2C_INT_CLR:
        s->int_raw_reg &= ~value;
        esp32_i2c_update_irq(s);
        break;
    case A_I2C_INT_ENA:
        s->int_ena_reg = value;
        esp32_i2c_update_irq(s);
        break;
    case A_I2C_CMD ... (A_I2C_CMD + (ESP32_I2C_CMD_COUNT - 1) * 4):
        s->cmd_reg[(addr - A_I2C_CMD) / 4] = value;
        break;
    case A_I2C_TIMEOUT:
        s->timeout_reg = value;
        break;
    case A_I2C_SDA_HOLD:
        s->sda_hold_reg = value;
        break;
    case A_I2C_SDA_SAMPLE:
        s->sda_sample_reg = value;
        break;
    case A_I2C_HIGH_PERIOD:
        s->high_period_reg = value;
        break;
    case A_I2C_LOW_PERIOD:
        s->low_period_reg = value;
        break;
    case A_I2C_START_HOLD:
        s->start_hold_reg = value;
        break;
    case A_I2C_RSTART_SETUP:
        s->rstart_setup_reg = value;
        break;
    case A_I2C_STOP_HOLD:
        s->stop_hold_reg = value;
        break;
    case A_I2C_STOP_SETUP:
        s->stop_setup_reg = value;
        break;
    default:
        break;
    }
}

/* Set the controller's SCL and SDA outputs: 0 pulls low, 1 releases. */
static void esp32_i2c_drive(Esp32I2CState *s, bool scl, bool sda)
{
    qemu_set_irq(s->scl_out, scl);
    qemu_set_irq(s->sda_out, sda);
}

/*
 * [spec:nuos:req:emu.esp32.gpio]
 * Put a START (or repeated START) on the lines: release both, then pull
 * SDA low with SCL high, then SCL low. The devices on the QEMU bus sit on
 * the lines the controller is routed to, so they take part only if each
 * line, pulled low, reads back low through the matrix: the controller's
 * outputs reach pads whose inputs come back to it. Returns false, with
 * the interrupt raised, if the bus is not free: SCL held low times out,
 * SDA held low loses arbitration.
 */
static bool esp32_i2c_start(Esp32I2CState *s)
{
    esp32_i2c_drive(s, true, true);
    if (!s->scl_in) {
        s->int_raw_reg = FIELD_DP32(s->int_raw_reg, I2C_INT_RAW, TIME_OUT, 1);
        return false;
    }
    if (!s->sda_in) {
        s->int_raw_reg = FIELD_DP32(s->int_raw_reg, I2C_INT_RAW,
                                    ARBITRATION_LOST, 1);
        return false;
    }
    esp32_i2c_drive(s, true, false);
    s->connected = !s->sda_in;
    esp32_i2c_drive(s, false, false);
    s->connected &= !s->scl_in;
    return true;
}

/* Put a STOP on the lines: SDA low, release SCL, then release SDA. */
static void esp32_i2c_stop(Esp32I2CState *s)
{
    esp32_i2c_drive(s, false, false);
    esp32_i2c_drive(s, true, false);
    esp32_i2c_drive(s, true, true);
}

/*
 * A bit read with the controller off the QEMU bus: the level SDA reads
 * with the controller releasing it, which for an acknowledge is 0 for an
 * ACK.
 */
static bool esp32_i2c_unconnected_bit(Esp32I2CState *s)
{
    esp32_i2c_drive(s, false, true);
    return s->sda_in;
}

/*
 * [spec:nuos:req:emu.esp32.gpio]
 * Run the command list. Bytes move at once; the lines show the START and
 * STOP conditions around them and the bus's level in between, but not
 * each bit's clock (see esp32_i2c_start() for when the bus's devices take
 * part).
 */
static void esp32_i2c_do_transaction(Esp32I2CState *s)
{
    bool stop_or_end = false;

    for (int i_cmd = 0; i_cmd < ESP32_I2C_CMD_COUNT && !stop_or_end; ++i_cmd) {
        uint32_t cmd = s->cmd_reg[i_cmd];
        char opcode = FIELD_EX32(cmd, I2C_CMD, OPCODE);
        bool ack_check = FIELD_EX32(cmd, I2C_CMD, ACK_CHECK_EN);
        bool ack_exp = FIELD_EX32(cmd, I2C_CMD, ACK_EXP);

        switch (opcode) {
        case I2C_OPCODE_RSTART:
            if (s->trans_ongoing && s->connected) {
                i2c_end_transfer(s->bus);
            }
            s->trans_ongoing = false;
            if (!esp32_i2c_start(s)) {
                stop_or_end = true;
            }
            break;
        case I2C_OPCODE_WRITE: {
            size_t length = FIELD_EX32(cmd, I2C_CMD, BYTE_NUM);

            for (size_t nbytes = 0; nbytes < length; ++nbytes) {
                uint8_t data = fifo8_is_empty(&s->tx_fifo) ? 0 :
                               fifo8_pop(&s->tx_fifo);
                bool nack;

                if (!s->trans_ongoing) {
                    /* The address byte, after a START */
                    s->trans_ongoing = true;
                    if (s->connected) {
                        nack = i2c_start_transfer(s->bus, data >> 1,
                                                  data & 1) != 0;
                    } else {
                        nack = esp32_i2c_unconnected_bit(s);
                    }
                } else if (s->connected) {
                    nack = i2c_send(s->bus, data) != 0;
                } else {
                    nack = esp32_i2c_unconnected_bit(s);
                }
                if (ack_check && nack != ack_exp) {
                    /* The controller gives up the bus with a STOP */
                    s->int_raw_reg = FIELD_DP32(s->int_raw_reg, I2C_INT_RAW,
                                                ACK_ERR, 1);
                    if (s->connected) {
                        i2c_end_transfer(s->bus);
                    }
                    s->trans_ongoing = false;
                    esp32_i2c_stop(s);
                    stop_or_end = true;
                    break;
                }
            }
            break;
        }
        case I2C_OPCODE_READ: {
            size_t length = FIELD_EX32(cmd, I2C_CMD, BYTE_NUM);
            for (size_t nbytes = 0; nbytes < length; ++nbytes) {
                if (fifo8_num_free(&s->rx_fifo) == 0) {
                    error_report("esp32_i2c: RX FIFO overflow");
                } else {
                    uint8_t data;

                    if (s->connected) {
                        data = i2c_recv(s->bus);
                    } else {
                        data = esp32_i2c_unconnected_bit(s) ? 0xff : 0x00;
                    }
                    fifo8_push(&s->rx_fifo, data);
                }
            }
            break;
        }
        case I2C_OPCODE_STOP:
            if (s->trans_ongoing && s->connected) {
                i2c_end_transfer(s->bus);
            }
            s->trans_ongoing = false;
            esp32_i2c_stop(s);
            s->int_raw_reg = FIELD_DP32(s->int_raw_reg, I2C_INT_RAW,
                                        TRANS_COMPLETE, 1);
            stop_or_end = true;
            break;
        case I2C_OPCODE_END:
            s->int_raw_reg = FIELD_DP32(s->int_raw_reg, I2C_INT_RAW,
                                        END_DETECT, 1);
            stop_or_end = true;
            break;
        default:
            error_report("esp32_i2c: Invalid command %d opcode %d", i_cmd,
                         opcode);
            break;
        }
        s->cmd_reg[i_cmd] = FIELD_DP32(s->cmd_reg[i_cmd], I2C_CMD, DONE, 1);
    }
    esp32_i2c_update_irq(s);
}

static void esp32_i2c_scl_in(void *opaque, int n, int level)
{
    Esp32_I2C(opaque)->scl_in = level != 0;
}

static void esp32_i2c_sda_in(void *opaque, int n, int level)
{
    Esp32_I2C(opaque)->sda_in = level != 0;
}

static const MemoryRegionOps esp32_i2c_ops = {
    .read = esp32_i2c_read,
    .write = esp32_i2c_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32_i2c_init(Object * obj)
{
    Esp32I2CState *s = Esp32_I2C(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32_i2c_ops, s, TYPE_ESP32_I2C, ESP32_I2C_MEM_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);

    s->bus = i2c_init_bus(DEVICE(s), "i2c");

    qdev_init_gpio_out_named(DEVICE(s), &s->scl_out, ESP32_I2C_SCL_OUT, 1);
    qdev_init_gpio_out_named(DEVICE(s), &s->sda_out, ESP32_I2C_SDA_OUT, 1);
    qdev_init_gpio_in_named(DEVICE(s), esp32_i2c_scl_in, ESP32_I2C_SCL_IN, 1);
    qdev_init_gpio_in_named(DEVICE(s), esp32_i2c_sda_in, ESP32_I2C_SDA_IN, 1);
    /* The matrix's default for unrouted I2C inputs: high */
    s->scl_in = true;
    s->sda_in = true;

    fifo8_create(&s->tx_fifo, ESP32_I2C_FIFO_LENGTH);
    fifo8_create(&s->rx_fifo, ESP32_I2C_FIFO_LENGTH);
}

/* Both lines released: the bus idles high */
static void esp32_i2c_reset_exit(Object *obj, ResetType type)
{
    esp32_i2c_drive(Esp32_I2C(obj), true, true);
}

static const VMStateDescription vmstate_esp32_i2c = {
    .name = TYPE_ESP32_I2C,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_FIFO8(rx_fifo, Esp32I2CState),
        VMSTATE_FIFO8(tx_fifo, Esp32I2CState),
        VMSTATE_BOOL(trans_ongoing, Esp32I2CState),
        VMSTATE_UINT32(ctr_reg, Esp32I2CState),
        VMSTATE_UINT32(timeout_reg, Esp32I2CState),
        VMSTATE_UINT32(int_ena_reg, Esp32I2CState),
        VMSTATE_UINT32(int_raw_reg, Esp32I2CState),
        VMSTATE_UINT32(sda_hold_reg, Esp32I2CState),
        VMSTATE_UINT32(sda_sample_reg, Esp32I2CState),
        VMSTATE_UINT32(high_period_reg, Esp32I2CState),
        VMSTATE_UINT32(low_period_reg, Esp32I2CState),
        VMSTATE_UINT32(start_hold_reg, Esp32I2CState),
        VMSTATE_UINT32(rstart_setup_reg, Esp32I2CState),
        VMSTATE_UINT32(stop_hold_reg, Esp32I2CState),
        VMSTATE_UINT32(stop_setup_reg, Esp32I2CState),
        VMSTATE_UINT32_ARRAY(cmd_reg, Esp32I2CState, ESP32_I2C_CMD_COUNT),
        VMSTATE_BOOL(scl_in, Esp32I2CState),
        VMSTATE_BOOL(sda_in, Esp32I2CState),
        VMSTATE_BOOL(connected, Esp32I2CState),
        VMSTATE_END_OF_LIST()
    }
};

static void esp32_i2c_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32_i2c_reset_hold;
    rc->phases.exit = esp32_i2c_reset_exit;
    dc->vmsd = &vmstate_esp32_i2c;
}

/* [spec:nuos:req:emu.esp32.gpio] */
static const TypeInfo esp32_i2c_type_info = {
    .name = TYPE_ESP32_I2C,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32I2CState),
    .instance_init = esp32_i2c_init,
    .class_init = esp32_i2c_class_init,
};

static void esp32_i2c_register_types(void)
{
    type_register_static(&esp32_i2c_type_info);
}

type_init(esp32_i2c_register_types)
