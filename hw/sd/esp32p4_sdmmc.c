/*
 * ESP32-P4 SD/MMC host controller (SDHOST)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapter
 * 57 (SD/MMC Host Controller); ESP-IDF's sdmmc_reg.h and sdmmc_struct.h
 * for v3, the authority for the registers and reset values, and its
 * esp_hal_sd sdmmc_ll.h and sdmmc_periph.c for the P4.
 *
 * The P4's SDHOST is the DesignWare controller (hw/sd/dwc_sdmmc.c) with
 * two card slots, each up to 8 bits wide, a FIFO of 512 words (TRM
 * 57.6), and, as the rev3 register map has it, no PWREN or
 * BACK_END_POWER, no boot mode bits in CMD, an 11-bit RX_WMARK and no
 * descriptor skip length in BMOD. Its descriptors are the controller's
 * four words; ESP-IDF pads them to 64 bytes for the cache and chains them
 * through DES3, which ring mode without a skip length cannot follow.
 *
 * The P4's own registers, from 0x800:
 *
 *   - CLK_EDGE_SEL (0x800), held as written: the clock phases and the
 *     divider it describes act in HP_SYS_CLKRST's SDIO_LS clocks on the
 *     P4, where ESP-IDF programs them; it never programs them here.
 *   - RAW_INTS (0x804), read-only, whose meaning the TRM does not give:
 *     it reads 0, logged as unimplemented.
 *   - DLL_CLK_CONF (0x808) and DLL_CONF (0x80C): the DLL that phases the
 *     clock in high-speed mode (HP_SYS_CLKRST's SDIO_HS_MODE, the SDIO
 *     PLL's clocks undivided): DLL_CCLK_IN_SAM_PHASE sets the sample
 *     clock's phase, below; the other phases and the clock enables are
 *     held as written; the calibration is not modelled, so DLL_CAL_END
 *     stays 0 (logged as unimplemented).
 *
 * The CIU's clock, cclk_in, is SDIO_LS_SLF_CLK; it reaches a card only
 * while SDIO_LS_DRV_CLK and SDIO_LS_SAM_CLK run too. The IDMAC's accesses
 * go through the permission managers as DMA master SDMMC, AHB master ID 6
 * (ESP32P4_ICM_ID_SDMMC): one the DMA APM refuses ends with a fatal bus
 * error, and the AXI ICM records it. One a memory controller's AXI port
 * holds (the flash's or the PSRAM's while it cannot take an access: its
 * clocks stopped, held in reset, AXI_REQ_EN clear; esp32p4_mspi_dma_wait())
 * waits, and the IDMAC with it, without an error, until the port takes
 * accesses again (the controller's dma_wait()).
 *
 * [spec:nuos:req:emu.esp32p4.sd-uhs]
 * Sampling (TRM 57.10). The host samples CMD and DAT on its sample clock,
 * whose phase is, in low-speed mode, HP_SYS_CLKRST's
 * SDIO_LS_SAM_CLK_EDGE_SEL in quarters of cclk_in's period (as ESP-IDF
 * reckons it: the source's period times the divider, over four) and, in
 * high-speed mode, DLL_CCLK_IN_SAM_PHASE in 64ths. A bit is good where
 * that sample point, after the card clock edge that follows the one
 * launching it (half a period after a Default Speed card's falling-edge
 * launch; on DAT in DDR, the next edge of either kind), falls in the
 * card's output window (hw/sd/sd.h's SDOutputTiming) delayed by the board
 * round trip: the clock out through a pad and the trace to the card, the
 * data back along its trace and in through a pad, "board-delay-ps",
 * 4.0 ns by default (the P4's pad delays are not published; 1.6 ns each
 * way through a pad and 0.4 ns each way along the board's traces). A
 * response sampled outside it fails its CRC check, a data block its CRC
 * (hw/sd/dwc_sdmmc.c). With the defaults, at SDR50 (100 MHz, phases of
 * 2.5 ns) phases 1 and 2 sample within the window and 0 and 3 do not;
 * at DDR50 (50 MHz, phases of 5 ns) only phase 1 does on both CMD and
 * DAT; Default Speed and High Speed work at phase 0 up to 25 and 50 MHz.
 *
 * [spec:nuos:req:emu.esp32p4.sdmmc]
 * [spec:nuos:req:emu.esp32p4.gpio]
 * Pads. Slot 0's lines are IO MUX functions only (SD1_CCLK, SD1_CCMD,
 * SD1_CDATA0 to 7, function 0 of GPIO43, GPIO44, GPIO39 to GPIO42 and
 * GPIO45 to GPIO48: TRM table 10.14-1, ESP-IDF's sdmmc_pins.h; ESP-IDF's
 * sdmmc_ll.h has SDMMC_LL_SLOT_SUPPORT_GPIO_MATRIX(0) 0); slot 1's are
 * GPIO matrix signals only (0 to 9, SD_CARD_CCLK_2 to SD_CARD_CDATA7_2:
 * table 10.12-1, sdmmc_periph.c). A line joins the host to its card as
 * the pads between them are set up (the GPIO model's routing,
 * hw/gpio/esp32p4_gpio.c), on the pads the board wires the card to:
 *
 *   - the host's output reaches the card's pin while the pad takes it
 *     when the host drives it: slot 0's pad on function 0 (MCU_SEL), its
 *     output enable then the host's own; slot 1's pad with
 *     GPIO_FUNCn_OUT_SEL the host's signal and the enable the host's
 *     (OEN_SEL clear) or, with OEN_SEL set, GPIO_ENABLE's, as inverted by
 *     OEN_INV_SEL; neither held, nor driven from outside;
 *   - the card's output reaches the host's input while the input takes
 *     the card's pad with its input enabled (FUN_IE): slot 0's pad on
 *     function 0, slot 1's through the matrix (SIG_IN_SEL, IN_SEL the
 *     pad, IN_INV_SEL clear), the chip not driving the pad while the host
 *     releases it;
 *   - an input that does not take the card's pad reads what the GPIO
 *     model gives it: 0 on slot 0's pad on its function with its input
 *     disabled, 1 (an I1 input) on a pad on another function; slot 1's
 *     signals their default, 1, bypassing the matrix, or the pad or
 *     constant the matrix selects; and a pin the host's output does not
 *     reach sees its pad's level: the board's pull-ups, the pad's own
 *     pulls, another output on it (hw/sd/dwc_sdmmc.c has what each makes
 *     of the bus).
 *
 * The microSD socket has slot 0's CLK, CMD and DAT0 to DAT3, so a card
 * there never sees DAT4 to DAT7 (GPIO45 is the socket's supply switch);
 * the board's 51 kOhm pull-ups on CMD and DAT0 to DAT3, to VO4, are the
 * machine's. Slot 1's card stands where the board's ESP32-C6 is, its CLK,
 * CMD and DAT0 to DAT3 on GPIO18, GPIO19 and GPIO14 to GPIO17 (the
 * board's schematic, as ESP-Hosted wires its SDIO), each with a 51 kOhm
 * pull-up to 3.3 V. ESP-IDF's sd_host_sdmmc.c sets slot 0's pins up with
 * configure_pin_iomux() (function 0, input enabled, pull-down off, drive
 * strength 3) and slot 1's with configure_pin_gpio_matrix().
 *
 * The pads' drive strength (FUN_DRV) is not modelled: the datasheet gives
 * only each setting's current (5 to 40 mA, table 10.14-1), not the edge
 * rates or delays it gives into the card's load, so nothing ties it to
 * the sample window above, whose 4.0 ns round trip stands for the pads at
 * ESP-IDF's strength 3.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/esp32p4_clk_period.h"
#include "hw/sd/esp32p4_sdmmc.h"
#include "hw/gpio/esp32_line.h"
#include "hw/misc/esp32p4_icm.h"
#include "migration/vmstate.h"

#define A_RAW_INTS          0x804
#define A_DLL_CLK_CONF      0x808
#define A_DLL_CONF          0x80c

#define DLL_CLK_CONF_WRITABLE   0x001fffffu
#define DLL_CAL_STOP            BIT(0)

/* [spec:nuos:req:emu.esp32p4.sdmmc] */
static bool esp32p4_sdmmc_vendor_read(DWCSDMMCState *d, hwaddr offset,
                                      uint32_t *value)
{
    ESP32P4SdmmcState *s = ESP32P4_SDMMC(d);

    switch (offset) {
    case A_RAW_INTS:
        qemu_log_mask(LOG_UNIMP, "%s: RAW_INTS is not modelled\n",
                      TYPE_ESP32P4_SDMMC);
        *value = 0;
        return true;
    case A_DLL_CLK_CONF:
        *value = s->dll_clk_conf;
        return true;
    case A_DLL_CONF:
        *value = s->dll_conf;
        return true;
    default:
        return false;
    }
}

/* Slot 0's IO MUX pads, by line */
static const uint8_t slot0_pads[DWC_SDMMC_LINES] = ESP32P4_SDMMC_SLOT0_PADS;

/* The board pad slot @slot's line @i is wired to on the card, or -1. */
static int esp32p4_sdmmc_card_pad(ESP32P4SdmmcState *s, unsigned slot,
                                  unsigned i)
{
    unsigned pad;

    if (slot == 0) {
        return i < 2 + s->slot0_dat_lines ? slot0_pads[i] : -1;
    }
    pad = s->slot1_pad[i];
    return pad < ESP32P4_GPIO_PADS ? pad : -1;
}

/*
 * [spec:nuos:req:emu.esp32p4.sdmmc]
 * [spec:nuos:req:emu.esp32p4.gpio]
 * Slot @slot's lines as the GPIO matrix and IO MUX join the host and its
 * card: see "Pads" above.
 */
static void esp32p4_sdmmc_lines(DWCSDMMCState *d, unsigned slot,
                                DWCSDMMCLines *l)
{
    ESP32P4SdmmcState *s = ESP32P4_SDMMC(d);
    ESP32P4GpioState *g = s->gpio;

    if (!g) {
        return;
    }
    l->to_card = 0;
    l->to_host = 0;
    l->host_level = 0;
    l->card_level = 0;
    for (unsigned i = 0; i < DWC_SDMMC_LINES; i++) {
        int pad = esp32p4_sdmmc_card_pad(s, slot, i);
        uint16_t bit = 1u << i;
        bool on = false, off = false, off_level, in, level;

        if (pad >= 0) {
            esp32p4_gpio_pad_out_path(g, pad, slot ? (int)i : -1,
                                      ESP32P4_SDMMC_SLOT0_FUNC, &on, &off,
                                      &off_level);
            l->to_card |= on ? bit : 0;
            l->card_level |= esp32p4_gpio_pad_level(g, pad) ? bit : 0;
        }
        if (bit == DWC_SDMMC_LINE_CLK) {
            continue;
        }
        if (slot == 0) {
            in = pad >= 0 &&
                 esp32p4_gpio_iomux_in_selected(g, pad,
                                                ESP32P4_SDMMC_SLOT0_FUNC);
            level = esp32p4_gpio_iomux_in_level(g, slot0_pads[i],
                                                ESP32P4_SDMMC_SLOT0_FUNC);
        } else {
            in = pad >= 0 && esp32p4_gpio_sig_in_pad(g, i) == pad &&
                 !esp32p4_gpio_sig_in_inverted(g, i);
            level = esp32p4_gpio_sig_in_level(g, i);
        }
        l->to_host |= in && !off ? bit : 0;
        l->host_level |= level ? bit : 0;
    }
}

/* [spec:nuos:req:emu.esp32p4.sdmmc] */
/* The routing changed: the controller follows the pads as they now are. */
static void esp32p4_sdmmc_route_changed(Notifier *n, void *data)
{
    ESP32P4SdmmcState *s = container_of(n, ESP32P4SdmmcState,
                                        route_notifier);

    dwc_sdmmc_lines_changed(DWC_SDMMC(s));
}

/*
 * The pads are about to change, or are sampled: the controller first
 * takes its events due by then, as the pads were.
 */
static void esp32p4_sdmmc_line_sync(Notifier *n, void *data)
{
    ESP32P4SdmmcState *s = container_of(n, ESP32P4SdmmcState, line_source);

    dwc_sdmmc_catch_up(DWC_SDMMC(s));
}

/* [spec:nuos:req:emu.esp32p4.sdmmc] */
static bool esp32p4_sdmmc_vendor_write(DWCSDMMCState *d, hwaddr offset,
                                       uint32_t value, uint32_t mask)
{
    ESP32P4SdmmcState *s = ESP32P4_SDMMC(d);

    switch (offset) {
    case A_RAW_INTS:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only RAW_INTS\n",
                      TYPE_ESP32P4_SDMMC);
        return true;
    case A_DLL_CLK_CONF:
        s->dll_clk_conf = ((s->dll_clk_conf & ~mask) | (value & mask)) &
                          DLL_CLK_CONF_WRITABLE;
        return true;
    case A_DLL_CONF:
        s->dll_conf = ((s->dll_conf & ~mask) | (value & mask)) &
                      DLL_CAL_STOP;
        qemu_log_mask(LOG_UNIMP, "%s: the DLL's calibration is not "
                      "modelled\n", TYPE_ESP32P4_SDMMC);
        return true;
    default:
        return false;
    }
}

/* [spec:nuos:req:emu.esp32p4.sd-uhs] */
static bool esp32p4_sdmmc_sample_ok(DWCSDMMCState *d, unsigned slot,
                                    bool dat)
{
    ESP32P4SdmmcState *s = ESP32P4_SDMMC(d);
    uint64_t cclk_ps = (clock_get(d->cclk) * 1000) >> 32;
    uint64_t div = dwc_sdmmc_card_div(d, slot);
    uint64_t t, period, phase, capture, delay, hold;
    SDOutputTiming out;
    bool ddr;

    if (!div || !s->clkrst ||
        !sdbus_get_output_timing(&d->sdbus[slot], &out)) {
        return true;
    }
    if (esp32p4_clkrst_sdio_hs_mode(s->clkrst)) {
        phase = cclk_ps * extract32(s->dll_clk_conf, 15, 6) / 64;
    } else {
        phase = cclk_ps * esp32p4_clkrst_sdio_sam_edge(s->clkrst) / 4;
    }
    t = cclk_ps * div;
    ddr = dat && out.ddr;
    period = ddr ? t / 2 : t;
    if (ddr) {
        phase %= period;
    }
    capture = (out.falling ? t / 2 : period) + phase;
    delay = (dat ? out.dat_delay_ps : out.cmd_delay_ps) + s->board_delay_ps;
    hold = (dat ? out.dat_hold_ps : out.cmd_hold_ps) + s->board_delay_ps;
    return capture >= delay && capture <= period + hold;
}

/* The CIU's clock, if the output and input clocks run too. */
static uint64_t esp32p4_sdmmc_cclk_period(DWCSDMMCState *d)
{
    ESP32P4SdmmcState *s = ESP32P4_SDMMC(d);

    if (!clock_get(s->drv_clk) || !clock_get(s->sam_clk)) {
        return 0;
    }
    return clock_get(d->cclk);
}

/* [spec:nuos:req:emu.esp32p4.sdmmc] */
static int64_t esp32p4_sdmmc_clock_after(DWCSDMMCState *d, Clock *clk,
                                         int64_t from, uint64_t cycles)
{
    ESP32P4ClkEdges e = esp32p4_clk_edges_or_at(clk, from);

    return MAX(esp32p4_clk_edges_after(&e, from, cycles), from + 1);
}

static void esp32p4_sdmmc_clock_update(void *opaque, ClockEvent event)
{
    if (event == ClockPreUpdate) {
        dwc_sdmmc_catch_up(DWC_SDMMC(opaque));
        return;
    }
    dwc_sdmmc_clock_changed(DWC_SDMMC(opaque));
}

/*
 * [spec:nuos:req:emu.esp32p4.sdmmc]
 * The IDMAC's access waits on a memory controller's held AXI port, until
 * esp32p4_sdmmc_dma_ready() lets it go on.
 */
static bool esp32p4_sdmmc_dma_wait(DWCSDMMCState *d, uint32_t addr,
                                   uint32_t len, bool write)
{
    ESP32P4SdmmcState *s = ESP32P4_SDMMC(d);

    return esp32p4_mspi_dma_wait_on(&s->dma_wait, d->dma_as, addr, len,
                                    d->dma_attrs, write);
}

static void esp32p4_sdmmc_dma_ready(void *opaque)
{
    dwc_sdmmc_dma_ready(DWC_SDMMC(opaque));
}

static void esp32p4_sdmmc_init(Object *obj)
{
    ESP32P4SdmmcState *s = ESP32P4_SDMMC(obj);
    DWCSDMMCState *d = DWC_SDMMC(obj);

    s->drv_clk = qdev_init_clock_in(DEVICE(obj), "drv",
                                    esp32p4_sdmmc_clock_update, s,
                                    ClockPreUpdate | ClockUpdate);
    s->sam_clk = qdev_init_clock_in(DEVICE(obj), "sam",
                                    esp32p4_sdmmc_clock_update, s,
                                    ClockPreUpdate | ClockUpdate);
    qdev_init_gpio_out_named(DEVICE(obj), s->pad_out, ESP32P4_SDMMC_PAD_OUT,
                             DWC_SDMMC_MAX_SLOTS * DWC_SDMMC_LINES);
    qdev_init_gpio_out_named(DEVICE(obj), s->pad_oe, ESP32P4_SDMMC_PAD_OE,
                             DWC_SDMMC_MAX_SLOTS * DWC_SDMMC_LINES);
    s->route_notifier.notify = esp32p4_sdmmc_route_changed;
    s->line_source.notify = esp32p4_sdmmc_line_sync;
    esp32p4_mspi_dma_wait_init(&s->dma_wait, esp32p4_sdmmc_dma_ready, s);
    d->num_slots = 2;
    d->fifo_depth = 512;
    d->cmd_mask = 0xb0ffffff;
    d->fifoth_mask = 0x77ff0fff;
    d->has_power_regs = false;
    d->has_dsl = false;
    d->clk_edge_mask = 0x01ffffff;
    d->clk_edge_reset = 0x00820200;
    d->clk_edge_div = false;
}

/* [spec:nuos:req:emu.esp32p4.sdmmc] */
static void esp32p4_sdmmc_realize(DeviceState *dev, Error **errp)
{
    ESP32P4SdmmcState *s = ESP32P4_SDMMC(dev);
    DWCSDMMCState *d = DWC_SDMMC(dev);

    if (!s->pms) {
        error_setg(errp, "%s: the pms link is not set", TYPE_ESP32P4_SDMMC);
        return;
    }
    if (s->slot0_dat_lines > 8) {
        error_setg(errp, "%s: slot0-dat-lines %u is more than 8",
                   TYPE_ESP32P4_SDMMC, s->slot0_dat_lines);
        return;
    }
    d->dma_as = esp32p4_pms_bus_as(s->pms);
    d->dma_attrs = esp32p4_dma_attrs(ESP32P4_DMA_MASTER_SDMMC,
                                     ESP32P4_ICM_ID_SDMMC);
    DWC_SDMMC_GET_CLASS(d)->parent_realize(dev, errp);
    if (s->gpio) {
        esp32p4_gpio_add_route_notifier(s->gpio, &s->route_notifier);
        esp32_line_add_source(&s->line_source);
    }
}

static void esp32p4_sdmmc_unrealize(DeviceState *dev)
{
    ESP32P4SdmmcState *s = ESP32P4_SDMMC(dev);

    if (s->gpio) {
        notifier_remove(&s->route_notifier);
        notifier_remove(&s->line_source);
    }
}

static void esp32p4_sdmmc_reset_hold(Object *obj, ResetType type)
{
    ESP32P4SdmmcState *s = ESP32P4_SDMMC(obj);
    ResettableClass *prc = RESETTABLE_CLASS(
        object_class_get_parent(object_get_class(obj)));

    if (prc->phases.hold) {
        prc->phases.hold(obj, type);
    }
    s->dll_clk_conf = 0;
    s->dll_conf = 0;
    esp32p4_mspi_dma_wait_cancel(&s->dma_wait);
}

/*
 * [spec:nuos:req:emu.esp32p4.sdmmc]
 * The lines between transfers, as the pads see them: CMD and DAT
 * released to the board's pull-ups, and SDCLK low, its enable off, as the
 * IO MUX's GPIO43 is high impedance from reset (table 10.14-1); slot 1's
 * SDCLK has no enable of its own (1'd1 in the matrix, table 10.12-1),
 * and drives its pad low wherever the matrix routes it. The transfers
 * themselves are above single pad transitions (hw/sd/dwc_sdmmc.c).
 */
static void esp32p4_sdmmc_reset_exit(Object *obj, ResetType type)
{
    ESP32P4SdmmcState *s = ESP32P4_SDMMC(obj);
    ResettableClass *prc = RESETTABLE_CLASS(
        object_class_get_parent(object_get_class(obj)));

    if (prc->phases.exit) {
        prc->phases.exit(obj, type);
    }
    for (unsigned slot = 0; slot < DWC_SDMMC_MAX_SLOTS; slot++) {
        for (unsigned i = 0; i < DWC_SDMMC_LINES; i++) {
            unsigned n = slot * DWC_SDMMC_LINES + i;
            bool clk = (1u << i) == DWC_SDMMC_LINE_CLK;

            qemu_set_irq(s->pad_out[n], !clk);
            qemu_set_irq(s->pad_oe[n], 0);
        }
    }
}

/* An IDMAC access a held port held: the IDMAC asks again. */
static int esp32p4_sdmmc_post_load(void *opaque, int version_id)
{
    ESP32P4SdmmcState *s = opaque;

    esp32p4_mspi_dma_wait_retry(&s->dma_wait);
    return 0;
}

static const VMStateDescription vmstate_esp32p4_sdmmc = {
    .name = TYPE_ESP32P4_SDMMC,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = esp32p4_sdmmc_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT(parent_obj, ESP32P4SdmmcState, 2, vmstate_dwc_sdmmc,
                       DWCSDMMCState),
        VMSTATE_CLOCK(drv_clk, ESP32P4SdmmcState),
        VMSTATE_CLOCK(sam_clk, ESP32P4SdmmcState),
        VMSTATE_UINT32(dll_clk_conf, ESP32P4SdmmcState),
        VMSTATE_UINT32(dll_conf, ESP32P4SdmmcState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property esp32p4_sdmmc_properties[] = {
    DEFINE_PROP_LINK("pms", ESP32P4SdmmcState, pms, TYPE_ESP32P4_PMS,
                     ESP32P4PmsState *),
    DEFINE_PROP_LINK("clkrst", ESP32P4SdmmcState, clkrst,
                     TYPE_ESP32P4_CLKRST, ESP32P4ClkRstState *),
    DEFINE_PROP_LINK("gpio", ESP32P4SdmmcState, gpio, TYPE_ESP32P4_GPIO,
                     ESP32P4GpioState *),
    DEFINE_PROP_UINT32("board-delay-ps", ESP32P4SdmmcState, board_delay_ps,
                       4000),
    DEFINE_PROP_UINT8("slot0-dat-lines", ESP32P4SdmmcState, slot0_dat_lines,
                      4),
    DEFINE_PROP_UINT8("slot1-pad-clk", ESP32P4SdmmcState, slot1_pad[0], 18),
    DEFINE_PROP_UINT8("slot1-pad-cmd", ESP32P4SdmmcState, slot1_pad[1], 19),
    DEFINE_PROP_UINT8("slot1-pad-d0", ESP32P4SdmmcState, slot1_pad[2], 14),
    DEFINE_PROP_UINT8("slot1-pad-d1", ESP32P4SdmmcState, slot1_pad[3], 15),
    DEFINE_PROP_UINT8("slot1-pad-d2", ESP32P4SdmmcState, slot1_pad[4], 16),
    DEFINE_PROP_UINT8("slot1-pad-d3", ESP32P4SdmmcState, slot1_pad[5], 17),
    DEFINE_PROP_UINT8("slot1-pad-d4", ESP32P4SdmmcState, slot1_pad[6],
                      ESP32P4_SDMMC_NO_PAD),
    DEFINE_PROP_UINT8("slot1-pad-d5", ESP32P4SdmmcState, slot1_pad[7],
                      ESP32P4_SDMMC_NO_PAD),
    DEFINE_PROP_UINT8("slot1-pad-d6", ESP32P4SdmmcState, slot1_pad[8],
                      ESP32P4_SDMMC_NO_PAD),
    DEFINE_PROP_UINT8("slot1-pad-d7", ESP32P4SdmmcState, slot1_pad[9],
                      ESP32P4_SDMMC_NO_PAD),
};

static void esp32p4_sdmmc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    DWCSDMMCClass *sc = DWC_SDMMC_CLASS(klass);

    device_class_set_parent_realize(dc, esp32p4_sdmmc_realize,
                                    &sc->parent_realize);
    dc->unrealize = esp32p4_sdmmc_unrealize;
    dc->vmsd = &vmstate_esp32p4_sdmmc;
    device_class_set_props(dc, esp32p4_sdmmc_properties);
    rc->phases.hold = esp32p4_sdmmc_reset_hold;
    rc->phases.exit = esp32p4_sdmmc_reset_exit;
    sc->vendor_read = esp32p4_sdmmc_vendor_read;
    sc->vendor_write = esp32p4_sdmmc_vendor_write;
    sc->cclk_period = esp32p4_sdmmc_cclk_period;
    sc->clock_after = esp32p4_sdmmc_clock_after;
    sc->sample_ok = esp32p4_sdmmc_sample_ok;
    sc->lines = esp32p4_sdmmc_lines;
    sc->dma_wait = esp32p4_sdmmc_dma_wait;
}

/* [spec:nuos:req:emu.esp32p4.sdmmc] */
static const TypeInfo esp32p4_sdmmc_types[] = {
    {
        .name          = TYPE_ESP32P4_SDMMC,
        .parent        = TYPE_DWC_SDMMC,
        .instance_size = sizeof(ESP32P4SdmmcState),
        .instance_init = esp32p4_sdmmc_init,
        .class_init    = esp32p4_sdmmc_class_init,
    },
};

DEFINE_TYPES(esp32p4_sdmmc_types)
