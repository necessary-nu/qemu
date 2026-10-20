/*
 * The ESP32-P4 board's audio: the ES8311 codec on its I2S and I2C pads,
 * the NS4150B amplifier and the speaker and microphone
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The Waveshare ESP32-P4-WIFI6-POE-ETH's schematic, "Codec" and
 * "PA&SPEAKER&MIC":
 *
 *   - U10, an ES8311: CCLK on ESP_I2C_SCL (GPIO8) and CDATA on
 *     ESP_I2C_SDA (GPIO7), pulled up to ESP_3V3 by R48 and R50 (2.2 kOhm)
 *     and shared with the DSI and CSI connectors; CE pulled down by R53
 *     (10 kOhm), address 0x18; MCLK on GPIO13 (I2S_MCLK), SCLK on GPIO12,
 *     ASDOUT on GPIO11, LRCK on GPIO10, DSDIN on GPIO9; PVDD and DVDD on
 *     ESP_3V3, AVDD on A3V3.
 *   - MIC1, an analog microphone, its output through L5 and C100 into
 *     MIC1P (C72), MIC1N to AGND through C116 and C74: the codec's
 *     differential input, nothing on DMIC_SDA.
 *   - OUTP and OUTN through C90 and C92, 100 nF and 150 kOhm (R55, R66)
 *     into U8, an NS4150B, whose CTRL is PA_CTRL, GPIO53 through R71 (0R),
 *     pulled down by R74 (10 kOhm): the amplifier is off until software
 *     drives GPIO53 high. Its output goes to the speaker connector H4.
 *
 * The codec's pins see the pads as the GPIO matrix and IO MUX leave them:
 * the I2S controllers clock it through the pads (esp32p4_i2s_add_peer()),
 * other sources edge by edge through the pad levels; it drives ASDOUT on
 * GPIO11, and in master mode SCLK and LRCK on GPIO12 and GPIO10, as a
 * driver outside the chip; its own pull-ups on SCLK and LRCK (PULLUP_SE),
 * whose strength the datasheet does not give, are taken as 50 kOhm. Its
 * MCLK is the clock of I2S0, I2S1 or I2S2's I2Sn_MCLK_out where GPIO13
 * carries that signal (through the matrix, output enabled), and none
 * otherwise. The I2C bus is the wires on GPIO7 and GPIO8, which the codec
 * answers at the level of the wires (hw/i2c/i2c_wire_bus.c), whichever
 * controller drives them. The 100 nF and 150 kOhm high-pass at the
 * amplifier's input (about 11 Hz) is not modelled.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/riscv/esp32p4.h"

/* The codec's pins as their pads, for the I2S controllers. */
static void codec_begin(void *opaque, uint32_t ncycles, int64_t start_ns,
                        int64_t end_ns, bool sclk_inverted)
{
    es8311_sclk_run(opaque, ncycles, start_ns, end_ns, sclk_inverted);
}

static bool codec_fall(void *opaque, bool lrck, bool *driven)
{
    return es8311_sclk_fall(opaque, lrck, driven);
}

static void codec_rise(void *opaque, bool lrck, bool din)
{
    es8311_sclk_rise(opaque, lrck, din);
}

static void codec_end(void *opaque)
{
    es8311_sclk_end(opaque);
}

static const ESP32P4I2sPeerOps codec_peer_ops = {
    .begin = codec_begin,
    .fall = codec_fall,
    .rise = codec_rise,
    .end = codec_end,
};

/*
 * [spec:nuos:req:emu.esp32p4.i2s]
 * The clock on GPIO13, for the codec's MCLK: the I2Sn_MCLK_out of the
 * controller whose signal the pad carries.
 */
static void codec_mclk_update(ESP32P4MachineState *s)
{
    ESP32P4BoardAudio *a = &s->audio;
    bool inv;
    int sig = esp32p4_gpio_pad_out_sig(&s->gpio, ESP32P4_BOARD_CODEC_MCLK_PAD,
                                       &inv);
    uint64_t period = 0;

    for (unsigned n = 0; n < ESP32P4_I2S_HP_COUNT; n++) {
        if (sig >= 0 &&
            sig == esp32p4_i2s_out_signal(n, ESP32P4_I2S_OUT_MCLK)) {
            period = clock_get(a->i2s_mclk[n]);
        }
    }
    if (clock_get(a->codec_mclk) != period) {
        clock_set(a->codec_mclk, period);
        clock_propagate(a->codec_mclk);
    }
}

static void codec_mclk_source(void *opaque, ClockEvent event)
{
    codec_mclk_update(opaque);
}

static void codec_route_changed(Notifier *n, void *data)
{
    ESP32P4BoardAudio *a = container_of(n, ESP32P4BoardAudio, route_notifier);

    codec_mclk_update(a->machine);
}

/*
 * A pin the board drives from outside the chip, a level and an enable:
 * the pad driven at the level while enabled, released otherwise.
 */
static void board_pin_update(ESP32P4MachineState *s, unsigned pin)
{
    ESP32P4BoardAudio *a = &s->audio;
    unsigned pad = a->pin_pad[pin], d = a->pin_driver[pin];

    if (a->pin_oe[pin]) {
        qemu_set_irq(esp32p4_gpio_ext_in(&s->gpio, d, pad),
                     a->pin_level[pin]);
    } else {
        qemu_set_irq(esp32p4_gpio_ext_release(&s->gpio, d, pad), 1);
    }
}

static void board_pin_level(void *opaque, int n, int level)
{
    ESP32P4MachineState *s = opaque;
    ESP32P4BoardAudio *a = &s->audio;

    a->pin_level[n] = level;
    if (a->pin_oe[n]) {
        board_pin_update(s, n);
    }
}

static void board_pin_oe(void *opaque, int n, int level)
{
    ESP32P4MachineState *s = opaque;
    ESP32P4BoardAudio *a = &s->audio;

    a->pin_oe[n] = level;
    board_pin_update(s, n);
}

/* The I2C wires' SDA pull: open drain, low or released. */
static void board_sda_pull(void *opaque, int n, int level)
{
    ESP32P4MachineState *s = opaque;
    ESP32P4BoardAudio *a = &s->audio;

    a->pin_level[ESP32P4_BOARD_PIN_SDA] = false;
    a->pin_oe[ESP32P4_BOARD_PIN_SDA] = level;
    board_pin_update(s, ESP32P4_BOARD_PIN_SDA);
}

/* The codec's pull-ups on SCLK and LRCK */
static void board_codec_pull_up(void *opaque, int n, int level)
{
    ESP32P4MachineState *s = opaque;
    DeviceState *gpio = DEVICE(&s->gpio);

    qemu_set_irq(qdev_get_gpio_in_named(gpio, ESP32P4_GPIO_BOARD_PULL_UP,
                                        ESP32P4_BOARD_CODEC_SCLK_PAD),
                 level ? ESP32P4_BOARD_CODEC_PULL_UP : 0);
    qemu_set_irq(qdev_get_gpio_in_named(gpio, ESP32P4_GPIO_BOARD_PULL_UP,
                                        ESP32P4_BOARD_CODEC_LRCK_PAD),
                 level ? ESP32P4_BOARD_CODEC_PULL_UP : 0);
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s]
 * [spec:nuos:req:emu.esp32p4.i2c]
 * [spec:nuos:req:emu.esp32p4.gpio]
 * The board's audio, as the schematic wires it (see above). The
 * machine's audiodev is the board's speaker and microphone.
 */
void esp32p4_init_audio(ESP32P4MachineState *s)
{
    static const char *const mst[ESP32P4_I2S_HP_COUNT] = {
        "i2s0-mst", "i2s1-mst", "i2s2-mst",
    };
    static const struct {
        unsigned pin;
        unsigned pad;
        const char *out;
        const char *oe;
    } pins[] = {
        { ESP32P4_BOARD_PIN_SCLK, ESP32P4_BOARD_CODEC_SCLK_PAD,
          ES8311_SCLK_OUT, ES8311_SCLK_OE },
        { ESP32P4_BOARD_PIN_LRCK, ESP32P4_BOARD_CODEC_LRCK_PAD,
          ES8311_LRCK_OUT, ES8311_LRCK_OE },
        { ESP32P4_BOARD_PIN_ASDOUT, ESP32P4_BOARD_CODEC_ASDOUT_PAD,
          ES8311_ASDOUT, ES8311_ASDOUT_OE },
    };
    MachineState *ms = MACHINE(s);
    ESP32P4BoardAudio *a = &s->audio;
    DeviceState *gpio = DEVICE(&s->gpio), *clkrst = DEVICE(&s->clkrst);
    DeviceState *codec, *amp, *wires;
    unsigned codec_driver;
    ESP32P4I2sPeer peer = {
        .sclk_pad = ESP32P4_BOARD_CODEC_SCLK_PAD,
        .lrck_pad = ESP32P4_BOARD_CODEC_LRCK_PAD,
        .din_pad = ESP32P4_BOARD_CODEC_DSDIN_PAD,
        .dout_pad = ESP32P4_BOARD_CODEC_ASDOUT_PAD,
        .ops = &codec_peer_ops,
    };
    unsigned i;

    a->machine = s;
    a->pin_pad[ESP32P4_BOARD_PIN_SDA] = ESP32P4_BOARD_I2C_SDA_PAD;
    a->pin_driver[ESP32P4_BOARD_PIN_SDA] =
        esp32p4_gpio_ext_driver(&s->gpio, "the ESP_I2C wires' devices");
    codec_driver = esp32p4_gpio_ext_driver(&s->gpio, "the ES8311");

    /* The ESP_I2C wires and their pull-ups */
    object_initialize_child(OBJECT(s), "esp-i2c", &a->esp_i2c,
                            TYPE_I2C_WIRE_BUS);
    wires = DEVICE(&a->esp_i2c);
    qdev_prop_set_string(wires, "bus-name", "esp-i2c");
    sysbus_realize(SYS_BUS_DEVICE(wires), &error_fatal);
    qdev_connect_gpio_out_named(wires, I2C_WIRE_BUS_SDA_PULL, 0,
                                qemu_allocate_irq(board_sda_pull, s, 0));
    esp32p4_gpio_watch_pad(&s->gpio, ESP32P4_BOARD_I2C_SCL_PAD,
                    qdev_get_gpio_in_named(wires, I2C_WIRE_BUS_SCL, 0));
    esp32p4_gpio_watch_pad(&s->gpio, ESP32P4_BOARD_I2C_SDA_PAD,
                    qdev_get_gpio_in_named(wires, I2C_WIRE_BUS_SDA, 0));
    qemu_set_irq(qdev_get_gpio_in_named(gpio, ESP32P4_GPIO_BOARD_PULL_UP,
                                        ESP32P4_BOARD_I2C_SCL_PAD),
                 ESP32P4_BOARD_I2C_PULL_UP);
    qemu_set_irq(qdev_get_gpio_in_named(gpio, ESP32P4_GPIO_BOARD_PULL_UP,
                                        ESP32P4_BOARD_I2C_SDA_PAD),
                 ESP32P4_BOARD_I2C_PULL_UP);
    /*
     * [spec:nuos:req:emu.esp32p4.panel-control]
     * The wires go to the DSI connector too, for the display module
     * plugged into it (-device waveshare-5-dsi-touch-a).
     */
    esp32p4_dsi_host_set_connector_i2c(&s->dsi_host, a->esp_i2c.bus);

    /* The codec's MCLK, from whichever I2S drives GPIO13 */
    a->codec_mclk = clock_new(OBJECT(s), "codec-mclk");
    for (i = 0; i < ESP32P4_I2S_HP_COUNT; i++) {
        g_autofree char *name = g_strdup_printf("codec-%s", mst[i]);

        a->i2s_mclk[i] = clock_new(OBJECT(s), name);
        clock_set_callback(a->i2s_mclk[i], codec_mclk_source, s,
                           ClockUpdate);
        clock_set_source(a->i2s_mclk[i],
                         qdev_get_clock_out(clkrst, mst[i]));
    }
    a->route_notifier.notify = codec_route_changed;
    esp32p4_gpio_add_route_notifier(&s->gpio, &a->route_notifier);

    /* The codec */
    object_initialize_child(OBJECT(s), "es8311", &a->codec, TYPE_ES8311);
    codec = DEVICE(&a->codec);
    if (ms->audiodev) {
        qdev_prop_set_string(codec, "audiodev", ms->audiodev);
    }
    qdev_connect_clock_in(codec, "mclk", a->codec_mclk);
    qdev_realize(codec, BUS(a->esp_i2c.bus), &error_fatal);
    for (i = 0; i < ARRAY_SIZE(pins); i++) {
        a->pin_pad[pins[i].pin] = pins[i].pad;
        a->pin_driver[pins[i].pin] = codec_driver;
        qdev_connect_gpio_out_named(codec, pins[i].out, 0,
                                    qemu_allocate_irq(board_pin_level, s,
                                                      pins[i].pin));
        qdev_connect_gpio_out_named(codec, pins[i].oe, 0,
                                    qemu_allocate_irq(board_pin_oe, s,
                                                      pins[i].pin));
    }
    qdev_connect_gpio_out_named(codec, ES8311_PULL_UP, 0,
                                qemu_allocate_irq(board_codec_pull_up, s, 0));
    esp32p4_gpio_watch_pad(&s->gpio, ESP32P4_BOARD_CODEC_SCLK_PAD,
                    qdev_get_gpio_in_named(codec, ES8311_SCLK, 0));
    esp32p4_gpio_watch_pad(&s->gpio, ESP32P4_BOARD_CODEC_LRCK_PAD,
                    qdev_get_gpio_in_named(codec, ES8311_LRCK, 0));
    esp32p4_gpio_watch_pad(&s->gpio, ESP32P4_BOARD_CODEC_DSDIN_PAD,
                    qdev_get_gpio_in_named(codec, ES8311_DSDIN, 0));
    peer.opaque = &a->codec;
    for (i = 0; i < ESP32P4_I2S_COUNT; i++) {
        esp32p4_i2s_add_peer(&s->i2s[i], &peer);
    }

    /* The amplifier on PA_CTRL, and R74 holding it off */
    object_initialize_child(OBJECT(s), "pa", &a->amp, TYPE_NS4150);
    amp = DEVICE(&a->amp);
    sysbus_realize(SYS_BUS_DEVICE(amp), &error_fatal);
    qdev_connect_gpio_out_named(amp, NS4150_ON, 0,
                                qdev_get_gpio_in_named(codec,
                                                       ES8311_OUT_PATH, 0));
    esp32p4_gpio_watch_pad(&s->gpio, ESP32P4_BOARD_PA_CTRL_PAD,
                    qdev_get_gpio_in_named(amp, NS4150_CTRL, 0));
    qemu_set_irq(qdev_get_gpio_in_named(gpio, ESP32P4_GPIO_BOARD_PULL_DOWN,
                                        ESP32P4_BOARD_PA_CTRL_PAD),
                 ESP32P4_BOARD_PA_CTRL_PULL_DOWN);
}
