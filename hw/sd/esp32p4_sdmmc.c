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
 * error, and the AXI ICM records it.
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
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/esp32p4_clk_period.h"
#include "hw/sd/esp32p4_sdmmc.h"
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
    d->dma_as = esp32p4_pms_bus_as(s->pms);
    d->dma_attrs = esp32p4_dma_attrs(ESP32P4_DMA_MASTER_SDMMC,
                                     ESP32P4_ICM_ID_SDMMC);
    DWC_SDMMC_GET_CLASS(d)->parent_realize(dev, errp);
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
}

static const VMStateDescription vmstate_esp32p4_sdmmc = {
    .name = TYPE_ESP32P4_SDMMC,
    .version_id = 1,
    .minimum_version_id = 1,
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
    DEFINE_PROP_UINT32("board-delay-ps", ESP32P4SdmmcState, board_delay_ps,
                       4000),
};

static void esp32p4_sdmmc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    DWCSDMMCClass *sc = DWC_SDMMC_CLASS(klass);

    device_class_set_parent_realize(dc, esp32p4_sdmmc_realize,
                                    &sc->parent_realize);
    dc->vmsd = &vmstate_esp32p4_sdmmc;
    device_class_set_props(dc, esp32p4_sdmmc_properties);
    rc->phases.hold = esp32p4_sdmmc_reset_hold;
    sc->vendor_read = esp32p4_sdmmc_vendor_read;
    sc->vendor_write = esp32p4_sdmmc_vendor_write;
    sc->cclk_period = esp32p4_sdmmc_cclk_period;
    sc->clock_after = esp32p4_sdmmc_clock_after;
    sc->sample_ok = esp32p4_sdmmc_sample_ok;
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
