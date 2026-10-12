/*
 * SD Memory Card emulation as defined in the "SD Memory Card Physical
 * layer specification, Version 2.00."
 *
 * eMMC emulation defined in "JEDEC Standard No. 84-A43"
 *
 * Copyright (c) 2006 Andrzej Zaborowski  <balrog@zabor.org>
 * Copyright (c) 2007 CodeSourcery
 * Copyright (c) 2018 Philippe Mathieu-Daudé
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/cutils.h"
#include "qemu/bswap.h"
#include "hw/core/irq.h"
#include "hw/core/line-time.h"
#include "hw/core/registerfields.h"
#include "system/block-backend.h"
#include "hw/sd/sd.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qemu/bitmap.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "qemu/error-report.h"
#include "qemu/timer.h"
#include "qemu/log.h"
#include "qemu/guest-random.h"
#include "qemu/module.h"
#include "sdmmc-internal.h"
#include "trace.h"
#include "crypto/hmac.h"

//#define DEBUG_SD 1

#define SDSC_MAX_CAPACITY   (2 * GiB)

#define INVALID_ADDRESS     UINT32_MAX

typedef enum {
    sd_r0 = 0,    /* no response */
    sd_r1,        /* normal response command */
    spi_r2,       /* STATUS */
    sd_r2_i,      /* CID register */
    sd_r2_s,      /* CSD register */
    sd_r3,        /* OCR register */
    sd_r6 = 6,    /* Published RCA response */
    sd_r7,        /* Operating voltage */
    sd_r1b = -1,
    sd_illegal = -2,
} sd_rsp_type_t;

typedef enum {
    sd_spi,
    sd_bc,     /* broadcast -- no response */
    sd_bcr,    /* broadcast with response */
    sd_ac,     /* addressed -- no data transfer */
    sd_adtc,   /* addressed with data transfer */
} sd_cmd_type_t;

enum SDCardModes {
    sd_inactive,
    sd_card_identification_mode,
    sd_data_transfer_mode,
};

enum SDCardStates {
    sd_waitirq_state        = -2, /* emmc */
    sd_inactive_state       = -1,

    sd_idle_state           = 0,
    sd_ready_state          = 1,
    sd_identification_state = 2,
    sd_standby_state        = 3,
    sd_transfer_state       = 4,
    sd_sendingdata_state    = 5,
    sd_receivingdata_state  = 6,
    sd_programming_state    = 7,
    sd_disconnect_state     = 8,
    sd_bus_test_state       = 9,  /* emmc */
    sd_sleep_state          = 10, /* emmc */
    sd_io_state             = 15  /* sd */
};

#define SDMMC_CMD_MAX 64

typedef sd_rsp_type_t (*sd_cmd_handler)(SDState *sd, SDRequest req);

typedef struct SDProto {
    const char *name;
    struct {
        const unsigned class;
        const sd_cmd_type_t type;
        const char *name;
        sd_cmd_handler handler;
    } cmd[SDMMC_CMD_MAX], acmd[SDMMC_CMD_MAX];
} SDProto;

#define RPMB_STUFF_LEN      196
#define RPMB_KEY_MAC_LEN    32
#define RPMB_DATA_LEN       256     /* one RPMB block is half a sector */
#define RPMB_NONCE_LEN      16
#define RPMB_HASH_LEN       284

typedef struct QEMU_PACKED {
    uint8_t stuff_bytes[RPMB_STUFF_LEN];
    uint8_t key_mac[RPMB_KEY_MAC_LEN];
    uint8_t data[RPMB_DATA_LEN];
    uint8_t nonce[RPMB_NONCE_LEN];
    uint32_t write_counter;
    uint16_t address;
    uint16_t block_count;
    uint16_t result;
    uint16_t req_resp;
} RPMBDataFrame;

QEMU_BUILD_BUG_MSG(sizeof(RPMBDataFrame) != 512,
                   "invalid RPMBDataFrame size");

struct SDState {
    DeviceState parent_obj;

    /* SD Memory Card Registers */
    uint32_t ocr;
    uint8_t scr[8];
    uint8_t cid[16];
    uint8_t csd[16];
    uint16_t rca;
    uint32_t card_status;
    uint8_t sd_status[64];
    union {
        uint8_t ext_csd[512];
        struct {
            uint8_t ext_csd_rw[192]; /* Modes segment */
            uint8_t ext_csd_ro[320]; /* Properties segment */
        };
    };

    /* Static properties */

    uint8_t spec_version;
    uint64_t boot_part_size;
    uint64_t rpmb_part_size;
    BlockBackend *blk;
    uint8_t boot_config;

    const SDProto *proto;

    /* Runtime changeables */

    int32_t state;    /* current card state, one of SDCardStates */
    uint32_t vhs;
    bool wp_switch;
    unsigned long *wp_group_bmap;
    int32_t wp_group_bits;
    uint64_t size;
    uint32_t blk_len;
    uint32_t multi_blk_cnt;
    uint32_t erase_start;
    uint32_t erase_end;
    uint8_t pwd[16];
    uint32_t pwd_len;
    uint8_t function_group[6];
    uint8_t current_cmd;
    const char *last_cmd_name;
    /* True if we will handle the next command as an ACMD. Note that this does
     * *not* track the APP_CMD status bit!
     */
    bool expecting_acmd;
    uint32_t blk_written;

    uint64_t data_start;
    uint32_t data_offset;
    size_t data_size;
    uint8_t data[512];
    struct {
        uint32_t write_counter;
        uint8_t key[RPMB_KEY_MAC_LEN];
        uint8_t key_set;
        RPMBDataFrame result;
    } rpmb;
    QEMUTimer *ocr_power_timer;
    uint8_t dat_lines;
    bool cmd_line;
    char *preset_auth_key;

    /* UHS-I (see "UHS-I" below) */
    uint8_t uhs_mode;
    /* S18R of the first ACMD41 since the card left idle; whether seen */
    bool s18r;
    bool s18r_latched;
    /* The I/O's signalling voltage, mV, and the voltage switch under way */
    uint16_t signal_mv;
    uint8_t vsw_state;
    int64_t vsw_stop_ns;
    int64_t vsw_due_ns;
    QEMUTimer *vsw_timer;
    /* SDCLK reaches the card. */
    bool sdclk;
    /* The functions a CMD6 switch selects, taken at its status data's end */
    uint8_t function_group_new[6];
    bool function_switch_pending;
};

static void sd_realize(DeviceState *dev, Error **errp);

static const SDProto sd_proto_spi;
static const SDProto sd_proto_emmc;

static bool sd_is_spi(SDState *sd)
{
    return sd->proto == &sd_proto_spi;
}

static bool sd_is_emmc(SDState *sd)
{
    return sd->proto == &sd_proto_emmc;
}

static const char *sd_version_str(enum SDPhySpecificationVersion version)
{
    static const char *sdphy_version[] = {
        [SD_PHY_SPECv2_00_VERS]     = "v2.00",
        [SD_PHY_SPECv3_01_VERS]     = "v3.01",
    };
    if (version >= ARRAY_SIZE(sdphy_version)) {
        return "unsupported version";
    }
    return sdphy_version[version];
}

static const char *sd_mode_name(enum SDCardModes mode)
{
    static const char *mode_name[] = {
        [sd_inactive]                   = "inactive",
        [sd_card_identification_mode]   = "identification",
        [sd_data_transfer_mode]         = "transfer",
    };
    assert(mode < ARRAY_SIZE(mode_name));
    return mode_name[mode];
}

static const char *sd_state_name(enum SDCardStates state)
{
    static const char *state_name[] = {
        [sd_idle_state]             = "idle",
        [sd_ready_state]            = "ready",
        [sd_identification_state]   = "identification",
        [sd_standby_state]          = "standby",
        [sd_transfer_state]         = "transfer",
        [sd_sendingdata_state]      = "sendingdata",
        [sd_bus_test_state]         = "bus-test",
        [sd_receivingdata_state]    = "receivingdata",
        [sd_programming_state]      = "programming",
        [sd_disconnect_state]       = "disconnect",
        [sd_sleep_state]            = "sleep",
        [sd_io_state]               = "i/o"
    };
    if (state == sd_inactive_state) {
        return "inactive";
    }
    if (state == sd_waitirq_state) {
        return "wait-irq";
    }
    assert(state < ARRAY_SIZE(state_name));
    return state_name[state];
}

static const char *sd_response_name(sd_rsp_type_t rsp)
{
    static const char *response_name[] = {
        [sd_r0]     = "RESP#0 (no response)",
        [sd_r1]     = "RESP#1 (normal cmd)",
        [spi_r2]    = "RESP#2 (STATUS reg)",
        [sd_r2_i]   = "RESP#2 (CID reg)",
        [sd_r2_s]   = "RESP#2 (CSD reg)",
        [sd_r3]     = "RESP#3 (OCR reg)",
        [sd_r6]     = "RESP#6 (RCA)",
        [sd_r7]     = "RESP#7 (operating voltage)",
    };
    if (rsp == sd_illegal) {
        return "ILLEGAL RESP";
    }
    if (rsp == sd_r1b) {
        rsp = sd_r1;
    }
    assert(rsp < ARRAY_SIZE(response_name));
    return response_name[rsp];
}

static const char *sd_cmd_name(SDState *sd, uint8_t cmd)
{
    static const char *cmd_abbrev[SDMMC_CMD_MAX] = {
        [18]    = "READ_MULTIPLE_BLOCK",
                                            [25]    = "WRITE_MULTIPLE_BLOCK",
    };
    const SDProto *sdp = sd->proto;

    if (sdp->cmd[cmd].handler) {
        assert(!cmd_abbrev[cmd]);
        return sdp->cmd[cmd].name;
    }
    return cmd_abbrev[cmd] ? cmd_abbrev[cmd] : "UNKNOWN_CMD";
}

static const char *sd_acmd_name(SDState *sd, uint8_t cmd)
{
    const SDProto *sdp = sd->proto;

    if (sdp->acmd[cmd].handler) {
        return sdp->acmd[cmd].name;
    }

    return "UNKNOWN_ACMD";
}

static uint8_t sd_get_dat_lines(SDState *sd)
{
    return sd->dat_lines;
}

static bool sd_get_cmd_line(SDState *sd)
{
    return sd->cmd_line;
}

static void sd_set_voltage(SDState *sd, uint16_t millivolts)
{
    trace_sdcard_set_voltage(millivolts);

    switch (millivolts) {
    case 3001 ... 3600: /* SD_VOLTAGE_3_3V */
    case 2001 ... 3000: /* SD_VOLTAGE_3_0V */
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "SD card voltage not supported: %.3fV",
                      millivolts / 1000.f);
    }
}

static enum SDCardModes sd_mode(SDState *sd)
{
    switch (sd->state) {
    case sd_inactive_state:
        return sd_inactive;
    case sd_idle_state:
    case sd_ready_state:
    case sd_identification_state:
        return sd_card_identification_mode;
    case sd_standby_state:
    case sd_transfer_state:
    case sd_sendingdata_state:
    case sd_receivingdata_state:
    case sd_programming_state:
    case sd_disconnect_state:
        return sd_data_transfer_mode;
    default:
        g_assert_not_reached();
    }
}

static uint8_t sd_crc7(const void *message, size_t width)
{
    int i, bit;
    uint8_t shift_reg = 0x00;
    const uint8_t *msg = (const uint8_t *)message;

    for (i = 0; i < width; i ++, msg ++)
        for (bit = 7; bit >= 0; bit --) {
            shift_reg <<= 1;
            if ((shift_reg >> 7) ^ ((*msg >> bit) & 1))
                shift_reg ^= 0x89;
        }

    return shift_reg;
}

/* Operation Conditions register */

#define OCR_POWER_DELAY_NS      500000 /* 0.5ms */

FIELD(OCR, VDD_VOLTAGE_WINDOW,          0, 24)
FIELD(OCR, VDD_VOLTAGE_WIN_LO,          0,  8)
FIELD(OCR, DUAL_VOLTAGE_CARD,           7,  1)
FIELD(OCR, VDD_VOLTAGE_WIN_HI,          8, 16)
FIELD(OCR, ACCEPT_SWITCH_1V8,          24,  1) /* Only UHS-I */
FIELD(OCR, UHS_II_CARD,                29,  1) /* Only UHS-II */
FIELD(OCR, CARD_CAPACITY,              30,  1) /* 0:SDSC, 1:SDHC/SDXC */
FIELD(OCR, CARD_POWER_UP,              31,  1)

#define ACMD41_ENQUIRY_MASK     0x00ffffff
#define ACMD41_R3_MASK          (R_OCR_VDD_VOLTAGE_WIN_HI_MASK \
                               | R_OCR_ACCEPT_SWITCH_1V8_MASK \
                               | R_OCR_UHS_II_CARD_MASK \
                               | R_OCR_CARD_CAPACITY_MASK \
                               | R_OCR_CARD_POWER_UP_MASK)

static void sd_ocr_powerup(void *opaque)
{
    SDState *sd = opaque;

    trace_sdcard_powerup();
    assert(!FIELD_EX32(sd->ocr, OCR, CARD_POWER_UP));

    /* card power-up OK */
    sd->ocr = FIELD_DP32(sd->ocr, OCR, CARD_POWER_UP, 1);

    if (sd->size > SDSC_MAX_CAPACITY) {
        sd->ocr = FIELD_DP32(sd->ocr, OCR, CARD_CAPACITY, 1);
    }
}

static void sd_set_ocr(SDState *sd)
{
    /* All voltages OK */
    sd->ocr = R_OCR_VDD_VOLTAGE_WIN_HI_MASK;

    if (sd_is_spi(sd)) {
        /*
         * We don't need to emulate power up sequence in SPI-mode.
         * Thus, the card's power up status bit should be set to 1 when reset.
         * The card's capacity status bit should also be set if SD card size
         * is larger than 2GB for SDHC support.
         */
        sd_ocr_powerup(sd);
    }
}

/*
 * UHS-I
 *
 * [spec:nuos:req:emu.esp32p4.sd-uhs]
 * A card whose "uhs" property is 1 (UHS-I; a board sets it for its slot,
 * and 0, the default, keeps the card as other machines know it) is a
 * UHS104 microSD card: it is of high capacity, as UHS-I cards are, and
 * Physical Layer 3.01 or later (Simplified Specification 9.10, 3.9.2).
 *
 * Signalling voltage. The card signals at 3.3 V from power-up. The first
 * ACMD41 with a voltage window since the card entered idle asks, with its
 * S18R, for 1.8 V signalling (the arguments of those after it repeat it,
 * 4.2.3.1); once the card is ready, the response's S18A says it accepts:
 * a UHS-I card of high capacity, asked with S18R while it signals at
 * 3.3 V (4.2.3.1, 4.2.4.1). CMD11 then starts the switch, and is illegal
 * (no response) in the cases 4.2.4.4 lists: a card that does not switch,
 * an ACMD41 with S18R 0, a card not in the ready state, or one already at
 * 1.8 V. The card stays at 1.8 V through CMD0; only a power cycle (its
 * reset) brings it back to 3.3 V (3.9.4).
 *
 * The switch. The simplified specification leaves out its timing (4.2.4.2
 * and 4.2.4.3, figures 4-7 and 4-8); the card follows the sequence the SD
 * Host Controller Simplified Specification's signal voltage switch
 * procedure and Linux's mmc_set_uhs_voltage() keep to:
 *   - with CMD11's R1 the card drives CMD and DAT[3:0] low;
 *   - once SDCLK stops reaching it, its I/O moves to 1.8 V, which takes
 *     5 ms;
 *   - SDCLK back at least 5 ms after it stopped: the card drives CMD high
 *     and keeps DAT[3:0] low for 1 ms more, then releases them, which the
 *     bus's pull-ups take high: the switch is complete (4.2.4.1: "checked
 *     by high level of DAT[3:0]") and the card works in SDR12;
 *   - SDCLK back sooner: the switch fails, and the card holds DAT[3:0]
 *     low until a power cycle, as the host's failure check finds it.
 * The card answers no command while the switch is under way or failed.
 *
 * Bus speed modes (CMD6 function group 1, 4.3.10.3, Table 4-11): Default
 * Speed and High Speed at 3.3 V, SDR12, SDR25, SDR50, SDR104 and DDR50 at
 * 1.8 V (DDR50 is mandatory for microSD, 3.9.2). Groups 3 (driver strength
 * types B, A, C, D) and 4 (power limit 0.72 W to 2.88 W, not 1.80 W, as a
 * 3.0x card's) offer more than their default at 1.8 V only.
 *
 * Output timing. The simplified specification leaves out the bus timing
 * (6.6.6, 6.7.3, 6.7.4): the card's is that of a card just within the full
 * specification's limits, its output valid from tODLY after the edge that
 * launches a bit until tOH after the one that launches the next: Default
 * Speed, from the falling edge, 14 ns and 0; High Speed 14 ns and 2.5 ns;
 * SDR12 and SDR25 14 ns and 1.5 ns; SDR50 7.5 ns and 1.5 ns; SDR104, whose
 * output phase is the card's own, a window of 0.6 UI from 2.5 ns, held
 * 0.5 ns; DDR50 CMD 13.7 ns and 1.5 ns, DAT on both edges 7.0 ns and
 * 1.5 ns.
 */
enum {
    SD_VSW_NONE,
    /* CMD11 answered: CMD and DAT[3:0] low, until SDCLK stops */
    SD_VSW_LOW,
    /* SDCLK stopped at vsw_stop_ns: the I/O moves to 1.8 V */
    SD_VSW_STOPPED,
    /* SDCLK back: DAT[3:0] released at vsw_due_ns */
    SD_VSW_RELEASE,
    /* SDCLK back too soon: DAT[3:0] held low until a power cycle */
    SD_VSW_FAILED,
};

#define SD_VSW_STOP_NS          (5 * SCALE_MS)
#define SD_VSW_RELEASE_NS       (1 * SCALE_MS)

enum {
    SD_MODE_DS,
    SD_MODE_HS,
    SD_MODE_SDR12,
    SD_MODE_SDR25,
    SD_MODE_SDR50,
    SD_MODE_SDR104,
    SD_MODE_DDR50,
};

static SDBus *sd_get_bus(SDState *sd)
{
    BusState *bus = qdev_get_parent_bus(DEVICE(sd));

    return bus ? SD_BUS(bus) : NULL;
}

/* A UHS-I card, as it can be: SD mode, high capacity. */
static bool sd_is_uhs(SDState *sd)
{
    return sd->uhs_mode == UHS_I && !sd_is_spi(sd) && !sd_is_emmc(sd) &&
           sd->spec_version >= SD_PHY_SPECv3_01_VERS &&
           sd->size > SDSC_MAX_CAPACITY;
}

/* [spec:nuos:req:emu.esp32p4.sd-uhs] */
/* S18A: a UHS-I card asked by S18R, signalling at 3.3 V. */
static bool sd_accepts_1v8(SDState *sd)
{
    return sd_is_uhs(sd) && sd->s18r && sd->signal_mv == SD_VOLTAGE_3_3V;
}

static uint16_t sd_get_signal_voltage(SDState *sd)
{
    return sd->signal_mv;
}

/* The bus speed mode CMD6 group 1 has the card in. */
static unsigned sd_bus_mode(SDState *sd)
{
    if (sd->signal_mv == SD_VOLTAGE_1_8V) {
        return SD_MODE_SDR12 + sd->function_group[0];
    }
    return sd->function_group[0] ? SD_MODE_HS : SD_MODE_DS;
}

/* [spec:nuos:req:emu.esp32p4.sd-uhs] */
static void sd_get_output_timing(SDState *sd, SDOutputTiming *t)
{
    static const SDOutputTiming timing[] = {
        [SD_MODE_DS] = { 14000, 0, 14000, 0, true, false },
        [SD_MODE_HS] = { 14000, 2500, 14000, 2500, false, false },
        [SD_MODE_SDR12] = { 14000, 1500, 14000, 1500, false, false },
        [SD_MODE_SDR25] = { 14000, 1500, 14000, 1500, false, false },
        [SD_MODE_SDR50] = { 7500, 1500, 7500, 1500, false, false },
        [SD_MODE_SDR104] = { 2500, 500, 2500, 500, false, false },
        [SD_MODE_DDR50] = { 13700, 1500, 7000, 1500, false, true },
    };

    *t = timing[sd_bus_mode(sd)];
}

/* The switch completes: the card releases DAT[3:0], at vsw_due_ns. */
static void sd_vsw_release(void *opaque)
{
    SDState *sd = opaque;
    SDBus *bus = sd_get_bus(sd);

    sd->dat_lines = 0xf;
    sd->vsw_state = SD_VSW_NONE;
    if (bus) {
        sdbus_lines_changed(bus);
    }
}

static void sd_vsw_timer(void *opaque)
{
    SDState *sd = opaque;

    line_run_at(sd->vsw_due_ns, sd_vsw_release, sd);
}

/*
 * [spec:nuos:req:emu.esp32p4.sd-uhs]
 * SDCLK starts or stops reaching the card, at line_time(): the steps of
 * the voltage switch.
 */
static void sd_set_clock(SDState *sd, bool running)
{
    int64_t now = line_time();

    if (running == sd->sdclk) {
        return;
    }
    sd->sdclk = running;
    switch (sd->vsw_state) {
    case SD_VSW_LOW:
        if (!running) {
            sd->vsw_state = SD_VSW_STOPPED;
            sd->vsw_stop_ns = now;
            sd->signal_mv = SD_VOLTAGE_1_8V;
        }
        break;
    case SD_VSW_STOPPED:
        if (!running) {
            break;
        }
        if (now - sd->vsw_stop_ns < SD_VSW_STOP_NS) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: SDCLK back %" PRId64 " ns "
                          "after it stopped for the voltage switch, which "
                          "needs it stopped 5 ms: the switch fails, DAT[3:0] "
                          "held low until a power cycle\n", sd->proto->name,
                          now - sd->vsw_stop_ns);
            sd->vsw_state = SD_VSW_FAILED;
            break;
        }
        sd->cmd_line = true;
        sd->vsw_state = SD_VSW_RELEASE;
        sd->vsw_due_ns = now + SD_VSW_RELEASE_NS;
        timer_mod_ns(sd->vsw_timer, sd->vsw_due_ns);
        break;
    default:
        break;
    }
}

/* The card's UHS-I state at power-up. */
static void sd_uhs_power_up(SDState *sd)
{
    timer_del(sd->vsw_timer);
    sd->vsw_state = SD_VSW_NONE;
    sd->signal_mv = SD_VOLTAGE_3_3V;
}

/* SD Configuration register */

static void sd_set_scr(SDState *sd)
{
    sd->scr[0] = 0 << 4;        /* SCR structure version 1.0 */
    sd->scr[0] |= 2;            /* Spec Version 2.00 or Version 3.0X */
    sd->scr[1] = (2 << 4)       /* SDSC Card (Security Version 1.01) */
                 | 0b0101;      /* 1-bit or 4-bit width bus modes */
    sd->scr[2] = 0x00;          /* Extended Security is not supported. */
    if (sd->spec_version >= SD_PHY_SPECv3_01_VERS) {
        sd->scr[2] |= 1 << 7;   /* Spec Version 3.0X */
    }
    sd->scr[3] = 0x00;
    /* reserved for manufacturer usage */
    sd->scr[4] = 0x00;
    sd->scr[5] = 0x00;
    sd->scr[6] = 0x00;
    sd->scr[7] = 0x00;
}

/* Card IDentification register */

#define MID     0xaa
#define OID     "XY"
#define PNM     "QEMU!"
#define PRV     0x01
#define MDT_YR  2006
#define MDT_MON 2

static void sd_set_cid(SDState *sd)
{
    sd->cid[0] = MID;       /* Fake card manufacturer ID (MID) */
    sd->cid[1] = OID[0];    /* OEM/Application ID (OID) */
    sd->cid[2] = OID[1];
    sd->cid[3] = PNM[0];    /* Fake product name (PNM) */
    sd->cid[4] = PNM[1];
    sd->cid[5] = PNM[2];
    sd->cid[6] = PNM[3];
    sd->cid[7] = PNM[4];
    sd->cid[8] = PRV;       /* Fake product revision (PRV) */
    stl_be_p(&sd->cid[9], 0xdeadbeef); /* Fake serial number (PSN) */
    sd->cid[13] = 0x00 |    /* Manufacture date (MDT) */
        ((MDT_YR - 2000) / 10);
    sd->cid[14] = ((MDT_YR % 10) << 4) | MDT_MON;
    sd->cid[15] = (sd_crc7(sd->cid, 15) << 1) | 1;
}

static void emmc_set_cid(SDState *sd)
{
    sd->cid[0] = MID;       /* Fake card manufacturer ID (MID) */
    sd->cid[1] = 0b01;      /* CBX: soldered BGA */
    sd->cid[2] = OID[0];    /* OEM/Application ID (OID) */
    sd->cid[3] = PNM[0];    /* Fake product name (PNM) */
    sd->cid[4] = PNM[1];
    sd->cid[5] = PNM[2];
    sd->cid[6] = PNM[3];
    sd->cid[7] = PNM[4];
    sd->cid[8] = PNM[4];
    sd->cid[9] = PRV;       /* Fake product revision (PRV) */
    stl_be_p(&sd->cid[10], 0xdeadbeef); /* Fake serial number (PSN) */
    sd->cid[14] = (MDT_MON << 4) | (MDT_YR - 1997); /* Manufacture date (MDT) */
    sd->cid[15] = (sd_crc7(sd->cid, 15) << 1) | 1;
}

/* Card-Specific Data register */

#define HWBLOCK_SHIFT   9        /* 512 bytes */
#define SECTOR_SHIFT    5        /* 16 kilobytes */
#define WPGROUP_SHIFT   7        /* 2 megs */
#define CMULT_SHIFT     9        /* 512 times HWBLOCK_SIZE */
#define WPGROUP_SIZE    (1 << (HWBLOCK_SHIFT + SECTOR_SHIFT + WPGROUP_SHIFT))

static const uint8_t sd_csd_rw_mask[16] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xfe,
};

static void emmc_set_ext_csd(SDState *sd, uint64_t size)
{
    uint32_t sectcount = size >> HWBLOCK_SHIFT;

    memset(sd->ext_csd, 0, sizeof(sd->ext_csd)); /* FIXME only RW at reset */

    /* Properties segment (RO) */
    sd->ext_csd[EXT_CSD_S_CMD_SET] = 0b1; /* supported command sets */
    sd->ext_csd[EXT_CSD_BOOT_INFO] = 0x0; /* Boot information */
                                     /* Boot partition size. 128KB unit */
    sd->ext_csd[EXT_CSD_BOOT_MULT] = sd->boot_part_size / (128 * KiB);
    sd->ext_csd[EXT_CSD_ACC_SIZE] = 0x1; /* Access size */
    sd->ext_csd[EXT_CSD_HC_ERASE_GRP_SIZE] = 0x01; /* HC Erase unit size */
    sd->ext_csd[EXT_CSD_ERASE_TIMEOUT_MULT] = 0x01; /* HC erase timeout */
    sd->ext_csd[EXT_CSD_REL_WR_SEC_C] = 0x1; /* Reliable write sector count */
    sd->ext_csd[EXT_CSD_HC_WP_GRP_SIZE] = 0x01; /* HC write protect group size */
    sd->ext_csd[EXT_CSD_S_C_VCC] = 0x01; /* Sleep current VCC  */
    sd->ext_csd[EXT_CSD_S_C_VCCQ] = 0x01; /* Sleep current VCCQ */
    sd->ext_csd[EXT_CSD_S_A_TIMEOUT] = 0x01; /* Sleep/Awake timeout */
    stl_le_p(&sd->ext_csd[EXT_CSD_SEC_CNT], sectcount); /* Sector count */
    sd->ext_csd[210] = 0x46; /* Min write perf for 8bit@52Mhz */
    sd->ext_csd[209] = 0x46; /* Min read perf for 8bit@52Mhz  */
    sd->ext_csd[208] = 0x46; /* Min write perf for 4bit@52Mhz */
    sd->ext_csd[207] = 0x46; /* Min read perf for 4bit@52Mhz */
    sd->ext_csd[206] = 0x46; /* Min write perf for 4bit@26Mhz */
    sd->ext_csd[205] = 0x46; /* Min read perf for 4bit@26Mhz */
    sd->ext_csd[EXT_CSD_CARD_TYPE] = 0b11;
    sd->ext_csd[EXT_CSD_STRUCTURE] = 2;
    sd->ext_csd[EXT_CSD_REV] = 5;
    sd->ext_csd[EXT_CSD_RPMB_MULT] = sd->rpmb_part_size / (128 * KiB);
    sd->ext_csd[EXT_CSD_PARTITION_SUPPORT] = 0b111;

    /* Mode segment (RW) */
    sd->ext_csd[EXT_CSD_PART_CONFIG] = sd->boot_config;
}

static void emmc_set_csd(SDState *sd, uint64_t size)
{
    int hwblock_shift = HWBLOCK_SHIFT;
    uint32_t sectsize = (1 << (SECTOR_SHIFT + 1)) - 1;
    uint32_t wpsize = (1 << (WPGROUP_SHIFT + 1)) - 1;

    sd->csd[0] = (3 << 6) | (4 << 2); /* Spec v4.3 with EXT_CSD */
    sd->csd[1] = (1 << 3) | 6; /* Asynchronous data access time: 1ms */
    sd->csd[2] = 0x00;
    sd->csd[3] = (1 << 3) | 3;; /* Maximum bus clock frequency: 100MHz */
    sd->csd[4] = 0x0f;
    if (size <= 2 * GiB) {
        /* use 1k blocks */
        uint32_t csize1k = (size >> (CMULT_SHIFT + 10)) - 1;
        sd->csd[5] = 0x5a;
        sd->csd[6] = 0x80 | ((csize1k >> 10) & 0xf);
        sd->csd[7] = (csize1k >> 2) & 0xff;
    } else { /* >= 2GB : size stored in ext CSD, block addressing */
        sd->csd[5] = 0x59;
        sd->csd[6] = 0x8f;
        sd->csd[7] = 0xff;
        sd->ocr = FIELD_DP32(sd->ocr, OCR, CARD_CAPACITY, 1);
    }
    sd->csd[8] = 0xff;
    sd->csd[9] = 0xfc |     /* Max. write current */
        ((CMULT_SHIFT - 2) >> 1);
    sd->csd[10] = 0x40 |    /* Erase sector size */
        (((CMULT_SHIFT - 2) << 7) & 0x80) | (sectsize >> 1);
    sd->csd[11] = 0x00 |    /* Write protect group size */
        ((sectsize << 7) & 0x80) | wpsize;
    sd->csd[12] = 0x90 |    /* Write speed factor */
        (hwblock_shift >> 2);
    sd->csd[13] = 0x20 |    /* Max. write data block length */
        ((hwblock_shift << 6) & 0xc0);
    sd->csd[14] = 0x00;
    sd->csd[15] = (sd_crc7(sd->csd, 15) << 1) | 1;
    emmc_set_ext_csd(sd, size);
}

static void sd_set_csd(SDState *sd, uint64_t size)
{
    int hwblock_shift = HWBLOCK_SHIFT;
    uint32_t csize;
    uint32_t sectsize = (1 << (SECTOR_SHIFT + 1)) - 1;
    uint32_t wpsize = (1 << (WPGROUP_SHIFT + 1)) - 1;

    /* To indicate 2 GiB card, BLOCK_LEN shall be 1024 bytes */
    if (size == SDSC_MAX_CAPACITY) {
        hwblock_shift += 1;
    }
    csize = (size >> (CMULT_SHIFT + hwblock_shift)) - 1;

    if (size <= SDSC_MAX_CAPACITY) { /* Standard Capacity SD */
        sd->csd[0] = 0x00;      /* CSD structure */
        sd->csd[1] = 0x26;      /* Data read access-time-1 */
        sd->csd[2] = 0x00;      /* Data read access-time-2 */
        sd->csd[3] = 0x32;      /* Max. data transfer rate: 25 MHz */
        sd->csd[4] = 0x5f;      /* Card Command Classes */
        sd->csd[5] = 0x50 |     /* Max. read data block length */
            hwblock_shift;
        sd->csd[6] = 0xe0 |     /* Partial block for read allowed */
            ((csize >> 10) & 0x03);
        sd->csd[7] = 0x00 |     /* Device size */
            ((csize >> 2) & 0xff);
        sd->csd[8] = 0x3f |     /* Max. read current */
            ((csize << 6) & 0xc0);
        sd->csd[9] = 0xfc |     /* Max. write current */
            ((CMULT_SHIFT - 2) >> 1);
        sd->csd[10] = 0x40 |    /* Erase sector size */
            (((CMULT_SHIFT - 2) << 7) & 0x80) | (sectsize >> 1);
        sd->csd[11] = 0x00 |    /* Write protect group size */
            ((sectsize << 7) & 0x80) | wpsize;
        sd->csd[12] = 0x90 |    /* Write speed factor */
            (hwblock_shift >> 2);
        sd->csd[13] = 0x20 |    /* Max. write data block length */
            ((hwblock_shift << 6) & 0xc0);
        sd->csd[14] = 0x00;     /* File format group */
    } else {                    /* SDHC */
        size /= 512 * KiB;
        size -= 1;
        sd->csd[0] = 0x40;
        sd->csd[1] = 0x0e;
        sd->csd[2] = 0x00;
        sd->csd[3] = 0x32;
        sd->csd[4] = 0x5b;
        sd->csd[5] = 0x59;
        sd->csd[6] = 0x00;
        st24_be_p(&sd->csd[7], size);
        sd->csd[10] = 0x7f;
        sd->csd[11] = 0x80;
        sd->csd[12] = 0x0a;
        sd->csd[13] = 0x40;
        sd->csd[14] = 0x00;
    }
    sd->csd[15] = (sd_crc7(sd->csd, 15) << 1) | 1;
}

/* Relative Card Address register */

static void sd_set_rca(SDState *sd, uint16_t value)
{
    trace_sdcard_set_rca(value);
    sd->rca = value;
}

static uint16_t sd_req_get_rca(SDState *s, SDRequest req)
{
    switch (s->proto->cmd[req.cmd].type) {
    case sd_ac:
    case sd_adtc:
        return req.arg >> 16;
    case sd_spi:
    default:
        g_assert_not_reached();
    }
}

static bool sd_req_rca_same(SDState *s, SDRequest req)
{
    return sd_req_get_rca(s, req) == s->rca;
}

/* Card Status register */

FIELD(CSR, AKE_SEQ_ERROR,               3,  1)
FIELD(CSR, APP_CMD,                     5,  1)
FIELD(CSR, FX_EVENT,                    6,  1)
FIELD(CSR, SWITCH_ERROR,                7,  1)
FIELD(CSR, READY_FOR_DATA,              8,  1)
FIELD(CSR, CURRENT_STATE,               9,  4)
FIELD(CSR, ERASE_RESET,                13,  1)
FIELD(CSR, CARD_ECC_DISABLED,          14,  1)
FIELD(CSR, WP_ERASE_SKIP,              15,  1)
FIELD(CSR, CSD_OVERWRITE,              16,  1)
FIELD(CSR, DEFERRED_RESPONSE,          17,  1)
FIELD(CSR, ERROR,                      19,  1)
FIELD(CSR, CC_ERROR,                   20,  1)
FIELD(CSR, CARD_ECC_FAILED,            21,  1)
FIELD(CSR, ILLEGAL_COMMAND,            22,  1)
FIELD(CSR, COM_CRC_ERROR,              23,  1)
FIELD(CSR, LOCK_UNLOCK_FAILED,         24,  1)
FIELD(CSR, CARD_IS_LOCKED,             25,  1)
FIELD(CSR, WP_VIOLATION,               26,  1)
FIELD(CSR, ERASE_PARAM,                27,  1)
FIELD(CSR, ERASE_SEQ_ERROR,            28,  1)
FIELD(CSR, BLOCK_LEN_ERROR,            29,  1)
FIELD(CSR, ADDRESS_ERROR,              30,  1)
FIELD(CSR, OUT_OF_RANGE,               31,  1)

/* Card status bits, split by clear condition:
 * A : According to the card current state
 * B : Always related to the previous command
 * C : Cleared by read
 */
#define CARD_STATUS_A           (R_CSR_READY_FOR_DATA_MASK \
                               | R_CSR_CARD_ECC_DISABLED_MASK \
                               | R_CSR_CARD_IS_LOCKED_MASK)
#define CARD_STATUS_B           (R_CSR_CURRENT_STATE_MASK \
                               | R_CSR_ILLEGAL_COMMAND_MASK \
                               | R_CSR_COM_CRC_ERROR_MASK)
#define CARD_STATUS_C           (R_CSR_AKE_SEQ_ERROR_MASK \
                               | R_CSR_APP_CMD_MASK \
                               | R_CSR_ERASE_RESET_MASK \
                               | R_CSR_WP_ERASE_SKIP_MASK \
                               | R_CSR_CSD_OVERWRITE_MASK \
                               | R_CSR_ERROR_MASK \
                               | R_CSR_CC_ERROR_MASK \
                               | R_CSR_CARD_ECC_FAILED_MASK \
                               | R_CSR_LOCK_UNLOCK_FAILED_MASK \
                               | R_CSR_WP_VIOLATION_MASK \
                               | R_CSR_ERASE_PARAM_MASK \
                               | R_CSR_ERASE_SEQ_ERROR_MASK \
                               | R_CSR_BLOCK_LEN_ERROR_MASK \
                               | R_CSR_ADDRESS_ERROR_MASK \
                               | R_CSR_OUT_OF_RANGE_MASK)

static void sd_set_cardstatus(SDState *sd)
{
    sd->card_status = READY_FOR_DATA;
}

static void sd_set_sdstatus(SDState *sd)
{
    memset(sd->sd_status, 0, 64);
}

static const uint8_t sd_tuning_block_pattern4[64] = {
    /* Physical Layer Simplified Specification 9.10, Table 4-3 */
    0xff, 0x0f, 0xff, 0x00,     0xff, 0xcc, 0xc3, 0xcc,
    0xc3, 0x3c, 0xcc, 0xff,     0xfe, 0xff, 0xfe, 0xef,
    0xff, 0xdf, 0xff, 0xdd,     0xff, 0xfb, 0xff, 0xfb,
    0xbf, 0xff, 0x7f, 0xff,     0x77, 0xf7, 0xbd, 0xef,
    0xff, 0xf0, 0xff, 0xf0,     0x0f, 0xfc, 0xcc, 0x3c,
    0xcc, 0x33, 0xcc, 0xcf,     0xff, 0xef, 0xff, 0xee,
    0xff, 0xfd, 0xff, 0xfd,     0xdf, 0xff, 0xbf, 0xff,
    0xbb, 0xff, 0xf7, 0xff,     0xf7, 0x7f, 0x7b, 0xde
};

static int sd_req_crc_validate(SDRequest *req)
{
    uint8_t buffer[5];
    buffer[0] = 0x40 | req->cmd;
    stl_be_p(&buffer[1], req->arg);
    return 0;
    return sd_crc7(buffer, 5) != req->crc;  /* TODO */
}

static size_t sd_response_size(SDState *sd, sd_rsp_type_t rtype)
{
    switch (rtype) {
    case sd_r1:
    case sd_r1b:
        return sd_is_spi(sd) ? 1 : 4;

    case spi_r2:
        assert(sd_is_spi(sd));
        return 2;

    case sd_r2_i:
    case sd_r2_s:
        assert(!sd_is_spi(sd));
        return 16;

    case sd_r3:
    case sd_r7:
        return sd_is_spi(sd) ? 5 : 4;

    case sd_r6:
        assert(!sd_is_spi(sd));
        return 4;

    case sd_r0:
    case sd_illegal:
        return sd_is_spi(sd) ? 1 : 0;

    default:
        g_assert_not_reached();
    }
}

static void sd_response_r1_make(SDState *sd, uint8_t *response)
{
    if (sd_is_spi(sd)) {
        response[0] = sd->state == sd_idle_state;
        response[0] |= FIELD_EX32(sd->card_status, CSR, ERASE_RESET) << 1;
        response[0] |= FIELD_EX32(sd->card_status, CSR, ILLEGAL_COMMAND) << 2;
        response[0] |= FIELD_EX32(sd->card_status, CSR, COM_CRC_ERROR) << 3;
        response[0] |= FIELD_EX32(sd->card_status, CSR, ERASE_SEQ_ERROR) << 4;
        response[0] |= FIELD_EX32(sd->card_status, CSR, ADDRESS_ERROR) << 5;
        response[0] |= FIELD_EX32(sd->card_status, CSR, BLOCK_LEN_ERROR) << 6;
        response[0] |= 0 << 7;
    } else {
        stl_be_p(response, sd->card_status);
    }

    /* Clear the "clear on read" status bits */
    sd->card_status &= ~CARD_STATUS_C;
}

static void spi_response_r2_make(SDState *sd, uint8_t *resp)
{
    /* Prepend R1 */
    sd_response_r1_make(sd, resp);

    resp[1]  = FIELD_EX32(sd->card_status, CSR, CARD_IS_LOCKED) << 0;
    resp[1] |= (FIELD_EX32(sd->card_status, CSR, LOCK_UNLOCK_FAILED)
                || FIELD_EX32(sd->card_status, CSR, WP_ERASE_SKIP)) << 1;
    resp[1] |= FIELD_EX32(sd->card_status, CSR, ERROR) << 2;
    resp[1] |= FIELD_EX32(sd->card_status, CSR, CC_ERROR) << 3;
    resp[1] |= FIELD_EX32(sd->card_status, CSR, CARD_ECC_FAILED) << 4;
    resp[1] |= FIELD_EX32(sd->card_status, CSR, WP_VIOLATION) << 5;
    resp[1] |= FIELD_EX32(sd->card_status, CSR, ERASE_PARAM) << 6;
    resp[1] |= FIELD_EX32(sd->card_status, CSR, OUT_OF_RANGE) << 7;
}

static void sd_response_r3_make(SDState *sd, uint8_t *response)
{
    if (sd_is_spi(sd)) {
        /* Prepend R1 */
        sd_response_r1_make(sd, response);
        response++;
    }
    stl_be_p(response, sd->ocr & ACMD41_R3_MASK);
}

static void sd_response_r6_make(SDState *sd, uint8_t *response)
{
    uint16_t status;

    status = ((sd->card_status >> 8) & 0xc000) |
             ((sd->card_status >> 6) & 0x2000) |
              (sd->card_status & 0x1fff);
    sd->card_status &= ~(CARD_STATUS_C & 0xc81fff);
    stw_be_p(response + 0, sd->rca);
    stw_be_p(response + 2, status);
}

static void sd_response_r7_make(SDState *sd, uint8_t *response)
{
    if (sd_is_spi(sd)) {
        /* Prepend R1 */
        sd_response_r1_make(sd, response);
        response++;
    }
    stl_be_p(response, sd->vhs);
}

static uint32_t sd_blk_len(SDState *sd)
{
    if (FIELD_EX32(sd->ocr, OCR, CARD_CAPACITY)) {
        return 1 << HWBLOCK_SHIFT;
    }
    return sd->blk_len;
}

/*
 * This requires a disk image that has two boot partitions inserted at the
 * beginning of it, followed by an RPMB partition. The size of the boot
 * partitions is the "boot-partition-size" property, the one of the RPMB
 * partition is 'rpmb-partition-size'.
 */
static uint32_t sd_part_offset(SDState *sd)
{
    unsigned partition_access;

    if (!sd_is_emmc(sd)) {
        return 0;
    }

    partition_access = sd->ext_csd[EXT_CSD_PART_CONFIG]
                                 & EXT_CSD_PART_CONFIG_ACC_MASK;
    switch (partition_access) {
    case EXT_CSD_PART_CONFIG_ACC_DEFAULT:
        return sd->boot_part_size * 2 + sd->rpmb_part_size;
    case EXT_CSD_PART_CONFIG_ACC_BOOT1:
        return 0;
    case EXT_CSD_PART_CONFIG_ACC_BOOT2:
        return sd->boot_part_size * 1;
    case EXT_CSD_PART_CONFIG_ACC_RPMB:
        return sd->boot_part_size * 2;
    default:
         g_assert_not_reached();
    }
}

static uint64_t sd_req_get_address(SDState *sd, SDRequest req)
{
    uint64_t addr;

    if (FIELD_EX32(sd->ocr, OCR, CARD_CAPACITY)) {
        addr = (uint64_t) req.arg << HWBLOCK_SHIFT;
    } else {
        addr = req.arg;
    }
    trace_sdcard_req_addr(req.arg, addr);
    return addr;
}

static inline uint64_t sd_addr_to_wpnum(uint64_t addr)
{
    return addr >> (HWBLOCK_SHIFT + SECTOR_SHIFT + WPGROUP_SHIFT);
}

static void sd_reset(DeviceState *dev)
{
    SDState *sd = SDMMC_COMMON(dev);
    SDCardClass *sc = SDMMC_COMMON_GET_CLASS(sd);
    uint64_t size;
    uint64_t sect;

    trace_sdcard_reset();
    if (sd->blk) {
        blk_get_geometry(sd->blk, &sect);
    } else {
        sect = 0;
    }
    size = sect << HWBLOCK_SHIFT;
    if (sd_is_emmc(sd)) {
        size -= sd->boot_part_size * 2 + sd->rpmb_part_size;
    }

    sect = sd_addr_to_wpnum(size) + 1;

    sd->state = sd_idle_state;

    /* card registers */
    sd->rca = sd_is_emmc(sd) ? 0x0001 : 0x0000;
    sd->size = size;
    sd_set_ocr(sd);
    sd_set_scr(sd);
    sc->set_cid(sd);
    sc->set_csd(sd, size);
    sd_set_cardstatus(sd);
    sd_set_sdstatus(sd);

    g_free(sd->wp_group_bmap);
    sd->wp_switch = sd->blk ? !blk_is_writable(sd->blk) : false;
    sd->wp_group_bits = sect;
    sd->wp_group_bmap = bitmap_new(sd->wp_group_bits);
    memset(sd->function_group, 0, sizeof(sd->function_group));
    sd->function_switch_pending = false;
    sd->s18r = false;
    sd->s18r_latched = false;
    sd_uhs_power_up(sd);
    sd->erase_start = INVALID_ADDRESS;
    sd->erase_end = INVALID_ADDRESS;
    sd->blk_len = 0x200;
    sd->pwd_len = 0;
    sd->expecting_acmd = false;
    sd->dat_lines = 0xf;
    sd->cmd_line = true;
    sd->multi_blk_cnt = 0;
}

static bool sd_get_inserted(SDState *sd)
{
    return sd->blk && blk_is_inserted(sd->blk);
}

static bool sd_get_readonly(SDState *sd)
{
    return sd->wp_switch;
}

static void sd_cardchange(void *opaque, bool load, Error **errp)
{
    SDState *sd = opaque;
    DeviceState *dev = DEVICE(sd);
    SDBus *sdbus;
    bool inserted = sd_get_inserted(sd);
    bool readonly = sd_get_readonly(sd);

    if (inserted) {
        trace_sdcard_inserted(readonly);
        sd_reset(dev);
    } else {
        trace_sdcard_ejected();
    }

    sdbus = SD_BUS(qdev_get_parent_bus(dev));
    sdbus_set_inserted(sdbus, inserted);
    if (inserted) {
        sdbus_set_readonly(sdbus, readonly);
    }
}

static const BlockDevOps sd_block_ops = {
    .change_media_cb = sd_cardchange,
};

static bool sd_ocr_vmstate_needed(void *opaque)
{
    SDState *sd = opaque;

    /* Include the OCR state (and timer) if it is not yet powered up */
    return !FIELD_EX32(sd->ocr, OCR, CARD_POWER_UP);
}

static const VMStateDescription sd_ocr_vmstate = {
    .name = "sd-card/ocr-state",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = sd_ocr_vmstate_needed,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(ocr, SDState),
        VMSTATE_TIMER_PTR(ocr_power_timer, SDState),
        VMSTATE_END_OF_LIST()
    },
};

static bool vmstate_needed_for_rpmb(void *opaque)
{
    SDState *sd = opaque;

    return sd->rpmb_part_size > 0;
}

static const VMStateDescription emmc_rpmb_vmstate = {
    .name = "sd-card/rpmb-state",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = vmstate_needed_for_rpmb,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8_ARRAY(rpmb.result.key_mac, SDState, RPMB_KEY_MAC_LEN),
        VMSTATE_UINT8_ARRAY(rpmb.result.data, SDState, RPMB_DATA_LEN),
        VMSTATE_UINT8_ARRAY(rpmb.result.nonce, SDState, RPMB_NONCE_LEN),
        VMSTATE_UINT32(rpmb.result.write_counter, SDState),
        VMSTATE_UINT16(rpmb.result.address, SDState),
        VMSTATE_UINT16(rpmb.result.block_count, SDState),
        VMSTATE_UINT16(rpmb.result.result, SDState),
        VMSTATE_UINT16(rpmb.result.req_resp, SDState),
        VMSTATE_UINT32(rpmb.write_counter, SDState),
        VMSTATE_UINT8_ARRAY(rpmb.key, SDState, 32),
        VMSTATE_UINT8(rpmb.key_set, SDState),
        VMSTATE_END_OF_LIST()
    },
};

static bool vmstate_needed_for_emmc(void *opaque)
{
    SDState *sd = opaque;

    return sd_is_emmc(sd);
}

static const VMStateDescription emmc_extcsd_vmstate = {
    .name = "sd-card/ext_csd_modes-state",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = vmstate_needed_for_emmc,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8_ARRAY(ext_csd_rw, SDState, 192),
        VMSTATE_END_OF_LIST()
    },
};

static bool sd_uhs_vmstate_needed(void *opaque)
{
    SDState *sd = opaque;

    return sd->uhs_mode || sd->function_switch_pending;
}

static const VMStateDescription sd_uhs_vmstate = {
    .name = "sd-card/uhs-state",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = sd_uhs_vmstate_needed,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(ocr, SDState),
        VMSTATE_BOOL(s18r, SDState),
        VMSTATE_BOOL(s18r_latched, SDState),
        VMSTATE_UINT16(signal_mv, SDState),
        VMSTATE_UINT8(vsw_state, SDState),
        VMSTATE_INT64(vsw_stop_ns, SDState),
        VMSTATE_INT64(vsw_due_ns, SDState),
        VMSTATE_TIMER_PTR(vsw_timer, SDState),
        VMSTATE_BOOL(sdclk, SDState),
        VMSTATE_UINT8(dat_lines, SDState),
        VMSTATE_BOOL(cmd_line, SDState),
        VMSTATE_UINT8_ARRAY(function_group_new, SDState, 6),
        VMSTATE_BOOL(function_switch_pending, SDState),
        VMSTATE_END_OF_LIST()
    },
};

static int sd_vmstate_pre_load(void *opaque)
{
    SDState *sd = opaque;

    /* If the OCR state is not included (prior versions, or not
     * needed), then the OCR must be set as powered up. If the OCR state
     * is included, this will be replaced by the state restore.
     */
    sd_ocr_powerup(sd);

    return 0;
}

static const VMStateDescription sd_vmstate = {
    .name = "sd-card",
    .version_id = 2,
    .minimum_version_id = 2,
    .pre_load = sd_vmstate_pre_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UNUSED(4),
        VMSTATE_INT32(state, SDState),
        VMSTATE_UINT8_ARRAY(cid, SDState, 16),
        VMSTATE_UINT8_ARRAY(csd, SDState, 16),
        VMSTATE_UINT16(rca, SDState),
        VMSTATE_UINT32(card_status, SDState),
        VMSTATE_PARTIAL_BUFFER(sd_status, SDState, 1),
        VMSTATE_UINT32(vhs, SDState),
        VMSTATE_BITMAP(wp_group_bmap, SDState, 0, wp_group_bits),
        VMSTATE_UINT32(blk_len, SDState),
        VMSTATE_UINT32(multi_blk_cnt, SDState),
        VMSTATE_UINT32(erase_start, SDState),
        VMSTATE_UINT32(erase_end, SDState),
        VMSTATE_UINT8_ARRAY(pwd, SDState, 16),
        VMSTATE_UINT32(pwd_len, SDState),
        VMSTATE_UINT8_ARRAY(function_group, SDState, 6),
        VMSTATE_UINT8(current_cmd, SDState),
        VMSTATE_BOOL(expecting_acmd, SDState),
        VMSTATE_UINT32(blk_written, SDState),
        VMSTATE_UINT64(data_start, SDState),
        VMSTATE_UINT32(data_offset, SDState),
        VMSTATE_UINT8_ARRAY(data, SDState, 512),
        VMSTATE_UNUSED_V(1, 512),
        VMSTATE_UNUSED(1),
        VMSTATE_END_OF_LIST()
    },
    .subsections = (const VMStateDescription * const []) {
        &sd_ocr_vmstate,
        &sd_uhs_vmstate,
        &emmc_extcsd_vmstate,
        &emmc_rpmb_vmstate,
        NULL
    },
};

static void sd_blk_read(SDState *sd, uint64_t addr, uint32_t len)
{
    trace_sdcard_read_block(addr, len);
    addr += sd_part_offset(sd);
    if (!sd->blk || blk_pread(sd->blk, addr, len, sd->data, 0) < 0) {
        fprintf(stderr, "sd_blk_read: read error on host side\n");
    }
}

static void sd_blk_write(SDState *sd, uint64_t addr, uint32_t len)
{
    trace_sdcard_write_block(addr, len);
    addr += sd_part_offset(sd);
    if (!sd->blk || blk_pwrite(sd->blk, addr, len, sd->data, 0) < 0) {
        fprintf(stderr, "sd_blk_write: write error on host side\n");
    }
}

static bool rpmb_calc_hmac(SDState *sd, const RPMBDataFrame *frame,
                           unsigned int num_blocks, uint8_t *mac)
{
    g_autoptr(QCryptoHmac) hmac = NULL;
    size_t mac_len = RPMB_KEY_MAC_LEN;
    bool success = true;
    Error *err = NULL;
    uint64_t offset;

    hmac = qcrypto_hmac_new(QCRYPTO_HASH_ALGO_SHA256, sd->rpmb.key,
                            RPMB_KEY_MAC_LEN, &err);
    if (!hmac) {
        error_report_err(err);
        return false;
    }

    /*
     * This implies a read request because we only support single-block write
     * requests so far.
     */
    if (num_blocks > 1) {
        /*
         * Unfortunately, the underlying crypto libraries do not allow us to
         * migrate an active QCryptoHmac state. Therefore, we have to calculate
         * the HMAC in one run. To avoid buffering a complete read sequence in
         * SDState, reconstruct all frames except for the last one.
         */
        void *buf = sd->data;

        assert(RPMB_HASH_LEN <= sizeof(sd->data));

        /*
         * We will hash everything from data field to the end of RPMBDataFrame.
         */
        memcpy((uint8_t *)buf + RPMB_DATA_LEN,
               (uint8_t *)frame + offsetof(RPMBDataFrame, nonce),
               RPMB_HASH_LEN - RPMB_DATA_LEN);

        offset = lduw_be_p(&frame->address) * RPMB_DATA_LEN + sd_part_offset(sd);
        do {
            if (blk_pread(sd->blk, offset, RPMB_DATA_LEN, buf, 0) < 0) {
                error_report("sd_blk_read: read error on host side");
                success = false;
                break;
            }
            if (qcrypto_hmac_bytes(hmac, buf, RPMB_HASH_LEN, NULL, NULL,
                                   &err) < 0) {
                error_report_err(err);
                success = false;
                break;
            }
            offset += RPMB_DATA_LEN;
        } while (--num_blocks > 1);
    }

    if (success &&
        qcrypto_hmac_bytes(hmac, frame->data, RPMB_HASH_LEN, &mac,
                           &mac_len, &err) < 0) {
        error_report_err(err);
        success = false;
    }
    assert(!success || mac_len == RPMB_KEY_MAC_LEN);

    return success;
}

static void emmc_rpmb_blk_read(SDState *sd, uint64_t addr, uint32_t len)
{
    uint16_t resp = lduw_be_p(&sd->rpmb.result.req_resp);
    uint16_t result = lduw_be_p(&sd->rpmb.result.result);
    unsigned int curr_block = 0;

    if ((result & ~RPMB_RESULT_COUTER_EXPIRED) == RPMB_RESULT_OK &&
        resp == RPMB_RESP(RPMB_REQ_AUTH_DATA_READ)) {
        curr_block = lduw_be_p(&sd->rpmb.result.address);
        if (sd->rpmb.result.block_count == 0) {
            stw_be_p(&sd->rpmb.result.block_count, sd->multi_blk_cnt);
        } else {
            curr_block += lduw_be_p(&sd->rpmb.result.block_count);
            curr_block -= sd->multi_blk_cnt;
        }
        addr = curr_block * RPMB_DATA_LEN + sd_part_offset(sd);
        if (blk_pread(sd->blk, addr, RPMB_DATA_LEN,
                      sd->rpmb.result.data, 0) < 0) {
            error_report("sd_blk_read: read error on host side");
            memset(sd->rpmb.result.data, 0, sizeof(sd->rpmb.result.data));
            stw_be_p(&sd->rpmb.result.result,
                     RPMB_RESULT_READ_FAILURE
                     | (result & RPMB_RESULT_COUTER_EXPIRED));
        }
        if (sd->multi_blk_cnt == 1 &&
            !rpmb_calc_hmac(sd, &sd->rpmb.result,
                            lduw_be_p(&sd->rpmb.result.block_count),
                            sd->rpmb.result.key_mac)) {
            memset(sd->rpmb.result.data, 0, sizeof(sd->rpmb.result.data));
            stw_be_p(&sd->rpmb.result.result, RPMB_RESULT_AUTH_FAILURE);
        }
    } else if (!rpmb_calc_hmac(sd, &sd->rpmb.result, 1,
                               sd->rpmb.result.key_mac)) {
        memset(sd->rpmb.result.data, 0, sizeof(sd->rpmb.result.data));
        stw_be_p(&sd->rpmb.result.result, RPMB_RESULT_AUTH_FAILURE);
    }
    memcpy(sd->data, &sd->rpmb.result, sizeof(sd->rpmb.result));

    trace_sdcard_rpmb_read_block(resp, curr_block,
                                 lduw_be_p(&sd->rpmb.result.result));
}

static void emmc_rpmb_blk_write(SDState *sd, uint64_t addr, uint32_t len)
{
    RPMBDataFrame *frame = (RPMBDataFrame *)sd->data;
    uint16_t req = lduw_be_p(&frame->req_resp);
    uint8_t mac[RPMB_KEY_MAC_LEN];

    if (req == RPMB_REQ_READ_RESULT) {
        /* just return the current result register */
        goto exit;
    }
    memset(&sd->rpmb.result, 0, sizeof(sd->rpmb.result));
    memcpy(sd->rpmb.result.nonce, frame->nonce, sizeof(sd->rpmb.result.nonce));
    stw_be_p(&sd->rpmb.result.result, RPMB_RESULT_OK);
    stw_be_p(&sd->rpmb.result.req_resp, RPMB_RESP(req));

    if (!sd->rpmb.key_set && req != RPMB_REQ_PROGRAM_AUTH_KEY) {
        stw_be_p(&sd->rpmb.result.result, RPMB_RESULT_NO_AUTH_KEY);
        goto exit;
    }

    switch (req) {
    case RPMB_REQ_PROGRAM_AUTH_KEY:
        if (sd->rpmb.key_set) {
            stw_be_p(&sd->rpmb.result.result, RPMB_RESULT_WRITE_FAILURE);
            break;
        }
        memcpy(sd->rpmb.key, frame->key_mac, sizeof(sd->rpmb.key));
        sd->rpmb.key_set = 1;
        break;
    case RPMB_REQ_READ_WRITE_COUNTER:
        stl_be_p(&sd->rpmb.result.write_counter, sd->rpmb.write_counter);
        break;
    case RPMB_REQ_AUTH_DATA_WRITE:
        /* We only support single-block writes so far */
        if (sd->multi_blk_cnt != 1) {
            stw_be_p(&sd->rpmb.result.result, RPMB_RESULT_GENERAL_FAILURE);
            break;
        }
        if (sd->rpmb.write_counter == 0xffffffff) {
            stw_be_p(&sd->rpmb.result.result, RPMB_RESULT_WRITE_FAILURE);
            break;
        }
        if (!rpmb_calc_hmac(sd, frame, 1, mac) ||
            memcmp(frame->key_mac, mac, RPMB_KEY_MAC_LEN) != 0) {
            stw_be_p(&sd->rpmb.result.result, RPMB_RESULT_AUTH_FAILURE);
            break;
        }
        if (ldl_be_p(&frame->write_counter) != sd->rpmb.write_counter) {
            stw_be_p(&sd->rpmb.result.result, RPMB_RESULT_COUNTER_FAILURE);
            break;
        }
        sd->rpmb.result.address = frame->address;
        addr = lduw_be_p(&frame->address) * RPMB_DATA_LEN + sd_part_offset(sd);
        if (blk_pwrite(sd->blk, addr, RPMB_DATA_LEN, frame->data, 0) < 0) {
            error_report("sd_blk_write: write error on host side");
            stw_be_p(&sd->rpmb.result.result, RPMB_RESULT_WRITE_FAILURE);
        } else {
            sd->rpmb.write_counter++;
        }
        stl_be_p(&sd->rpmb.result.write_counter, sd->rpmb.write_counter);
        break;
    case RPMB_REQ_AUTH_DATA_READ:
        sd->rpmb.result.address = frame->address;
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "RPMB request %d not implemented\n", req);
        stw_be_p(&sd->rpmb.result.result, RPMB_RESULT_GENERAL_FAILURE);
        break;
    }
exit:
    if (sd->rpmb.write_counter == 0xffffffff) {
        stw_be_p(&sd->rpmb.result.result,
                 lduw_be_p(&sd->rpmb.result.result) | RPMB_RESULT_COUTER_EXPIRED);
    }
    trace_sdcard_rpmb_write_block(req, lduw_be_p(&sd->rpmb.result.result));
}

static void sd_erase(SDState *sd)
{
    uint64_t erase_start = sd->erase_start;
    uint64_t erase_end = sd->erase_end;
    bool sdsc = true;
    uint64_t wpnum;
    uint64_t erase_addr;
    int erase_len = 1 << HWBLOCK_SHIFT;

    trace_sdcard_erase(sd->erase_start, sd->erase_end);
    if (sd->erase_start == INVALID_ADDRESS
            || sd->erase_end == INVALID_ADDRESS) {
        sd->card_status |= ERASE_SEQ_ERROR;
        sd->erase_start = INVALID_ADDRESS;
        sd->erase_end = INVALID_ADDRESS;
        return;
    }

    if (FIELD_EX32(sd->ocr, OCR, CARD_CAPACITY)) {
        /* High capacity memory card: erase units are 512 byte blocks */
        erase_start <<= HWBLOCK_SHIFT;
        erase_end <<= HWBLOCK_SHIFT;
        sdsc = false;
    }

    if (erase_start > sd->size || erase_end > sd->size) {
        sd->card_status |= OUT_OF_RANGE;
        sd->erase_start = INVALID_ADDRESS;
        sd->erase_end = INVALID_ADDRESS;
        return;
    }

    sd->erase_start = INVALID_ADDRESS;
    sd->erase_end = INVALID_ADDRESS;
    sd->csd[14] |= 0x40;

    memset(sd->data, 0xff, erase_len);
    for (erase_addr = erase_start; erase_addr <= erase_end;
         erase_addr += erase_len) {
        if (sdsc) {
            /* Only SDSC cards support write protect groups */
            wpnum = sd_addr_to_wpnum(erase_addr);
            assert(wpnum < sd->wp_group_bits);
            if (test_bit(wpnum, sd->wp_group_bmap)) {
                sd->card_status |= WP_ERASE_SKIP;
                continue;
            }
        }
        sd_blk_write(sd, erase_addr, erase_len);
    }
}

static uint32_t sd_wpbits(SDState *sd, uint64_t addr)
{
    uint32_t i, wpnum;
    uint32_t ret = 0;

    wpnum = sd_addr_to_wpnum(addr);

    for (i = 0; i < 32; i++, wpnum++, addr += WPGROUP_SIZE) {
        if (addr >= sd->size) {
            /*
             * If the addresses of the last groups are outside the valid range,
             * then the corresponding write protection bits shall be set to 0.
             */
            continue;
        }
        assert(wpnum < sd->wp_group_bits);
        if (test_bit(wpnum, sd->wp_group_bmap)) {
            ret |= (1 << i);
        }
    }

    return ret;
}

enum ExtCsdAccessMode {
    EXT_CSD_ACCESS_MODE_COMMAND_SET = 0,
    EXT_CSD_ACCESS_MODE_SET_BITS    = 1,
    EXT_CSD_ACCESS_MODE_CLEAR_BITS  = 2,
    EXT_CSD_ACCESS_MODE_WRITE_BYTE  = 3
};

static void emmc_function_switch(SDState *sd, uint32_t arg)
{
    uint8_t access = extract32(arg, 24, 2);
    uint8_t index = extract32(arg, 16, 8);
    uint8_t value = extract32(arg, 8, 8);
    uint8_t b = sd->ext_csd[index];

    trace_sdcard_switch(access, index, value, extract32(arg, 0, 2));

    if (index >= 192) {
        qemu_log_mask(LOG_GUEST_ERROR, "MMC switching illegal offset\n");
        sd->card_status |= R_CSR_SWITCH_ERROR_MASK;
        return;
    }

    switch (access) {
    case EXT_CSD_ACCESS_MODE_COMMAND_SET:
        qemu_log_mask(LOG_UNIMP, "MMC Command set switching not supported\n");
        return;
    case EXT_CSD_ACCESS_MODE_SET_BITS:
        b |= value;
        break;
    case EXT_CSD_ACCESS_MODE_CLEAR_BITS:
        b &= ~value;
        break;
    case EXT_CSD_ACCESS_MODE_WRITE_BYTE:
        b = value;
        break;
    }

    if (index == EXT_CSD_PART_CONFIG) {
        uint8_t part = b & EXT_CSD_PART_CONFIG_ACC_MASK;

        if (((part == EXT_CSD_PART_CONFIG_ACC_BOOT1 ||
              part == EXT_CSD_PART_CONFIG_ACC_BOOT2) && !sd->boot_part_size) ||
            (part == EXT_CSD_PART_CONFIG_ACC_RPMB && !sd->rpmb_part_size)) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "MMC switching to illegal partition\n");
            sd->card_status |= R_CSR_SWITCH_ERROR_MASK;
            return;
        }
    }

    trace_sdcard_ext_csd_update(index, sd->ext_csd[index], b);
    sd->ext_csd[index] = b;
}

/*
 * The functions CMD6 offers in function group @g + 1 (Table 4-11): bit i
 * for function i; 0 and 15 always. Group 1's UHS-I modes and groups 3's
 * and 4's other functions only at 1.8 V (4.3.10.3).
 */
static uint16_t sd_function_support(SDState *sd, unsigned g)
{
    bool v18 = sd->signal_mv == SD_VOLTAGE_1_8V;

    switch (g) {
    case 0:
        return v18 ? 0x801f : 0x8003;
    case 1:
        return 0x8043;
    case 2:
    case 3:
        return v18 ? 0x800f : 0x8001;
    default:
        return 0x8001;
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.sd-uhs]
 * A UHS-I card's maximum current, mA at 3.6 V, under functions @f: its
 * bus speed mode's, in mode 1 within the power limit's current (4.3.10.4,
 * Table 4-12).
 */
static uint16_t sd_function_current(SDState *sd, const uint8_t *f, bool set)
{
    static const uint16_t mode_ma[] = { 100, 200, 400, 400, 400 };
    uint16_t ma;

    if (sd->signal_mv == SD_VOLTAGE_1_8V) {
        ma = mode_ma[f[0]];
    } else {
        ma = f[0] ? 200 : 100;
    }
    if (set) {
        ma = MIN(ma, 200 * (f[3] + 1));
    }
    return ma;
}

/*
 * [spec:nuos:req:emu.esp32p4.sd-uhs]
 * CMD6's status (Table 4-13). Mode 0 checks the functions the argument
 * selects, mode 1 switches to them: a function the card does not offer
 * reads 0xF and, in mode 1, voids the whole switch, its other groups
 * reading their current functions (4.3.10.2, 4.3.10.3). The switch takes
 * effect at the end of the status data, within the 8 clocks 4.3.10.1
 * allows, so that block goes in the old bus mode.
 */
static void sd_function_switch(SDState *sd, uint32_t arg)
{
    bool set = arg & 0x80000000;
    uint8_t want[6], status[6];
    bool valid = true;
    uint16_t ma;
    int i;

    /* A switch whose status data was cut short never took effect. */
    sd->function_switch_pending = false;
    for (i = 0; i < 6; i++) {
        uint8_t f = extract32(arg, 4 * i, 4);

        if (f == 0xf) {
            want[i] = sd->function_group[i];
        } else if (sd_function_support(sd, i) & BIT(f)) {
            want[i] = f;
        } else {
            want[i] = 0xf;
            valid = false;
        }
    }
    for (i = 0; i < 6; i++) {
        status[i] = want[i];
        if (set && !valid && want[i] != 0xf) {
            status[i] = sd->function_group[i];
        }
    }
    if (set && valid) {
        memcpy(sd->function_group_new, want, sizeof(want));
        sd->function_switch_pending = true;
    }

    memset(sd->data, 0, 64);
    if (sd->uhs_mode) {
        ma = valid ? sd_function_current(sd, want, set) : 0;
    } else {
        /* What QEMU's cards have always given */
        ma = 1;
    }
    stw_be_p(&sd->data[0], ma);
    for (i = 0; i < 6; i++) {
        stw_be_p(&sd->data[12 - 2 * i], sd_function_support(sd, i));
        sd->data[16 - (i >> 1)] |= status[i] << ((i % 2) * 4);
    }
    /* Version 1 has the busy status (all ready); 0 does not. */
    sd->data[17] = sd->uhs_mode ? 1 : 0;
}

/* A CMD6 switch's status data has gone: its functions take effect. */
static void sd_function_switch_done(SDState *sd)
{
    if (sd->function_switch_pending) {
        memcpy(sd->function_group, sd->function_group_new,
               sizeof(sd->function_group));
        sd->function_switch_pending = false;
    }
}

static inline bool sd_wp_addr(SDState *sd, uint64_t addr)
{
    return test_bit(sd_addr_to_wpnum(addr), sd->wp_group_bmap);
}

static void sd_lock_command(SDState *sd)
{
    int erase, lock, clr_pwd, set_pwd, pwd_len;
    erase = !!(sd->data[0] & 0x08);
    lock = sd->data[0] & 0x04;
    clr_pwd = sd->data[0] & 0x02;
    set_pwd = sd->data[0] & 0x01;

    if (sd->blk_len > 1)
        pwd_len = sd->data[1];
    else
        pwd_len = 0;

    if (lock) {
        trace_sdcard_lock();
    } else {
        trace_sdcard_unlock();
    }
    if (erase) {
        if (!(sd->card_status & CARD_IS_LOCKED) || sd->blk_len > 1 ||
                        set_pwd || clr_pwd || lock || sd->wp_switch ||
                        (sd->csd[14] & 0x20)) {
            sd->card_status |= LOCK_UNLOCK_FAILED;
            return;
        }
        bitmap_zero(sd->wp_group_bmap, sd->wp_group_bits);
        sd->csd[14] &= ~0x10;
        sd->card_status &= ~CARD_IS_LOCKED;
        sd->pwd_len = 0;
        /* Erasing the entire card here! */
        fprintf(stderr, "SD: Card force-erased by CMD42\n");
        return;
    }

    if (sd->blk_len < 2 + pwd_len ||
                    pwd_len <= sd->pwd_len ||
                    pwd_len > sd->pwd_len + 16) {
        sd->card_status |= LOCK_UNLOCK_FAILED;
        return;
    }

    if (sd->pwd_len && memcmp(sd->pwd, sd->data + 2, sd->pwd_len)) {
        sd->card_status |= LOCK_UNLOCK_FAILED;
        return;
    }

    pwd_len -= sd->pwd_len;
    if ((pwd_len && !set_pwd) ||
                    (clr_pwd && (set_pwd || lock)) ||
                    (lock && !sd->pwd_len && !set_pwd) ||
                    (!set_pwd && !clr_pwd &&
                     (((sd->card_status & CARD_IS_LOCKED) && lock) ||
                      (!(sd->card_status & CARD_IS_LOCKED) && !lock)))) {
        sd->card_status |= LOCK_UNLOCK_FAILED;
        return;
    }

    if (set_pwd) {
        memcpy(sd->pwd, sd->data + 2 + sd->pwd_len, pwd_len);
        sd->pwd_len = pwd_len;
    }

    if (clr_pwd) {
        sd->pwd_len = 0;
    }

    if (lock)
        sd->card_status |= CARD_IS_LOCKED;
    else
        sd->card_status &= ~CARD_IS_LOCKED;
}

static bool address_in_range(SDState *sd, const char *desc,
                             uint64_t addr, uint32_t length)
{
    if (addr + length > sd->size) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s offset %"PRIu64" > card %"PRIu64" [%%%u]\n",
                      desc, addr, sd->size, length);
        sd->card_status |= ADDRESS_ERROR;
        return false;
    }
    return true;
}

static sd_rsp_type_t sd_invalid_state_for_cmd(SDState *sd, SDRequest req)
{
    qemu_log_mask(LOG_GUEST_ERROR, "%s: CMD%i in a wrong state: %s (spec %s)\n",
                  sd->proto->name, req.cmd, sd_state_name(sd->state),
                  sd_version_str(sd->spec_version));

    return sd_illegal;
}

static sd_rsp_type_t sd_invalid_mode_for_cmd(SDState *sd, SDRequest req)
{
    qemu_log_mask(LOG_GUEST_ERROR, "%s: CMD%i in a wrong mode: %s (spec %s)\n",
                  sd->proto->name, req.cmd, sd_mode_name(sd_mode(sd)),
                  sd_version_str(sd->spec_version));

    return sd_illegal;
}

static sd_rsp_type_t sd_cmd_illegal(SDState *sd, SDRequest req)
{
    qemu_log_mask(LOG_GUEST_ERROR, "%s: Unknown CMD%i for spec %s\n",
                  sd->proto->name, req.cmd,
                  sd_version_str(sd->spec_version));

    return sd_illegal;
}

/* Commands that are recognised but not yet implemented. */
static sd_rsp_type_t sd_cmd_unimplemented(SDState *sd, SDRequest req)
{
    qemu_log_mask(LOG_UNIMP, "%s: CMD%i not implemented\n",
                  sd->proto->name, req.cmd);

    return sd_illegal;
}

static sd_rsp_type_t sd_cmd_optional(SDState *sd, SDRequest req)
{
    qemu_log_mask(LOG_UNIMP, "%s: Optional CMD%i not implemented\n",
                  sd->proto->name, req.cmd);

    return sd_illegal;
}

/* Configure fields for following sd_generic_write_data() calls */
static sd_rsp_type_t sd_cmd_to_receivingdata(SDState *sd, SDRequest req,
                                             uint64_t start, size_t size)
{
    if (sd->state != sd_transfer_state) {
        return sd_invalid_state_for_cmd(sd, req);
    }
    sd->state = sd_receivingdata_state;
    sd->data_start = start;
    sd->data_offset = 0;
    /* sd->data[] used as receive buffer */
    sd->data_size = size ?: sizeof(sd->data);
    return sd_r1;
}

/* Configure fields for following sd_generic_read_data() calls */
static sd_rsp_type_t sd_cmd_to_sendingdata(SDState *sd, SDRequest req,
                                           uint64_t start,
                                           const void *data, size_t size)
{
    if (sd->state != sd_transfer_state) {
        return sd_invalid_state_for_cmd(sd, req);
    }

    sd->state = sd_sendingdata_state;
    sd->data_start = start;
    sd->data_offset = 0;
    if (data) {
        assert(size > 0 && size <= sizeof(sd->data));
        memcpy(sd->data, data, size);
    }
    if (size) {
        sd->data_size = size;
    }
    return sd_r1;
}

/* CMD0 */
static sd_rsp_type_t sd_cmd_GO_IDLE_STATE(SDState *sd, SDRequest req)
{
    if (sd->state == sd_sleep_state) {
        switch (req.arg) {
        case 0x00000000:
        case 0xf0f0f0f0:
            break;
        default:
            return sd_r0;
        }
    }
    if (sd->state != sd_inactive_state) {
        /* [spec:nuos:req:emu.esp32p4.sd-uhs] */
        /* The card keeps signalling at 1.8 V, in SDR12 (3.9.4). */
        uint16_t signal_mv = sd->signal_mv;

        sd->state = sd_idle_state;
        sd_reset(DEVICE(sd));
        sd->signal_mv = signal_mv;
    }

    return sd_is_spi(sd) ? sd_r1 : sd_r0;
}

/* CMD2 */
static sd_rsp_type_t sd_cmd_ALL_SEND_CID(SDState *sd, SDRequest req)
{
    switch (sd->state) {
    case sd_ready_state:
        sd->state = sd_identification_state;
        return sd_r2_i;
    default:
        return sd_invalid_state_for_cmd(sd, req);
    }
}

/* CMD3 */
static sd_rsp_type_t sd_cmd_SEND_RELATIVE_ADDR(SDState *sd, SDRequest req)
{
    uint16_t random_rca;

    switch (sd->state) {
    case sd_identification_state:
    case sd_standby_state:
        sd->state = sd_standby_state;
        qemu_guest_getrandom_nofail(&random_rca, sizeof(random_rca));
        sd_set_rca(sd, random_rca);
        return sd_r6;

    default:
        return sd_invalid_state_for_cmd(sd, req);
    }
}

static sd_rsp_type_t emmc_cmd_SET_RELATIVE_ADDR(SDState *sd, SDRequest req)
{
    switch (sd->state) {
    case sd_identification_state:
    case sd_standby_state:
        sd->state = sd_standby_state;
        sd_set_rca(sd, req.arg >> 16);
        return sd_r1;

    default:
        return sd_invalid_state_for_cmd(sd, req);
    }
}

/* CMD5 */
static sd_rsp_type_t emmc_cmd_sleep_awake(SDState *sd, SDRequest req)
{
    bool do_sleep = extract32(req.arg, 15, 1);

    switch (sd->state) {
    case sd_sleep_state:
        if (!do_sleep) {
            /* Awake */
            sd->state = sd_standby_state;
        }
        return sd_r1b;

    case sd_standby_state:
        if (do_sleep) {
            sd->state = sd_sleep_state;
        }
        return sd_r1b;

    default:
        return sd_invalid_state_for_cmd(sd, req);
    }
}

/* CMD6 */
static sd_rsp_type_t sd_cmd_SWITCH_FUNCTION(SDState *sd, SDRequest req)
{
    if (sd_mode(sd) != sd_data_transfer_mode) {
        return sd_invalid_mode_for_cmd(sd, req);
    }
    if (sd_is_spi(sd)) {
        if (sd->state == sd_idle_state) {
            return sd_invalid_state_for_cmd(sd, req);
        }
    } else {
        if (sd->state != sd_transfer_state) {
            return sd_invalid_state_for_cmd(sd, req);
        }
    }

    sd_function_switch(sd, req.arg);
    return sd_cmd_to_sendingdata(sd, req, 0, NULL, 64);
}

static sd_rsp_type_t emmc_cmd_SWITCH(SDState *sd, SDRequest req)
{
    switch (sd->state) {
    case sd_transfer_state:
        sd->state = sd_programming_state;
        emmc_function_switch(sd, req.arg);
        sd->state = sd_transfer_state;
        return sd_r1b;
    default:
        return sd_invalid_state_for_cmd(sd, req);
    }
}

/* CMD7 */
static sd_rsp_type_t sd_cmd_DE_SELECT_CARD(SDState *sd, SDRequest req)
{
    bool same_rca = sd_req_rca_same(sd, req);

    switch (sd->state) {
    case sd_standby_state:
        if (!same_rca) {
            return sd_r0;
        }
        sd->state = sd_transfer_state;
        return sd_r1b;

    case sd_transfer_state:
    case sd_sendingdata_state:
        if (same_rca) {
            break;
        }
        sd->state = sd_standby_state;
        return sd_r1b;

    case sd_disconnect_state:
        if (!same_rca) {
            return sd_r0;
        }
        sd->state = sd_programming_state;
        return sd_r1b;

    case sd_programming_state:
        if (same_rca) {
            break;
        }
        sd->state = sd_disconnect_state;
        return sd_r1b;

    default:
        break;
    }
    return sd_invalid_state_for_cmd(sd, req);
}

/* CMD8 */
static sd_rsp_type_t sd_cmd_SEND_IF_COND(SDState *sd, SDRequest req)
{
    if (sd->state != sd_idle_state) {
        return sd_invalid_state_for_cmd(sd, req);
    }
    sd->vhs = 0;

    /* No response if not exactly one VHS bit is set.  */
    if (!(req.arg >> 8) || (req.arg >> (ctz32(req.arg & ~0xff) + 1))) {
        return sd_is_spi(sd) ? sd_r7 : sd_r0;
    }

    /* Accept.  */
    sd->vhs = req.arg;
    return sd_r7;
}

/* CMD8 */
static sd_rsp_type_t emmc_cmd_SEND_EXT_CSD(SDState *sd, SDRequest req)
{
    if (sd->state != sd_transfer_state) {
        return sd_invalid_state_for_cmd(sd, req);
    }

    return sd_cmd_to_sendingdata(sd, req, sd_req_get_address(sd, req),
                                 sd->ext_csd, sizeof(sd->ext_csd));
}

static sd_rsp_type_t spi_cmd_SEND_CxD(SDState *sd, SDRequest req,
                                      const void *data, size_t size)
{
    /*
     * XXX as of v10.1.0-rc1 command is reached in sd_idle_state,
     * so disable this check.
    if (sd->state != sd_standby_state) {
        return sd_invalid_state_for_cmd(sd, req);
    }
    */

    /*
     * Since SPI returns CSD and CID on the DAT lines,
     * switch to sd_transfer_state.
     */
    sd->state = sd_transfer_state;

    return sd_cmd_to_sendingdata(sd, req, 0, data, size);
}

/* CMD9 */
static sd_rsp_type_t spi_cmd_SEND_CSD(SDState *sd, SDRequest req)
{
    return spi_cmd_SEND_CxD(sd, req, sd->csd, sizeof(sd->csd));
}

static sd_rsp_type_t sd_cmd_SEND_CSD(SDState *sd, SDRequest req)
{
    if (sd->state != sd_standby_state) {
        return sd_invalid_state_for_cmd(sd, req);
    }

    return sd_req_rca_same(sd, req) ? sd_r2_s : sd_r0;
}

/* CMD10 */
static sd_rsp_type_t spi_cmd_SEND_CID(SDState *sd, SDRequest req)
{
    return spi_cmd_SEND_CxD(sd, req, sd->cid, sizeof(sd->cid));
}

static sd_rsp_type_t sd_cmd_SEND_CID(SDState *sd, SDRequest req)
{
    if (sd->state != sd_standby_state) {
        return sd_invalid_state_for_cmd(sd, req);
    }

    return sd_req_rca_same(sd, req) ? sd_r2_i : sd_r0;
}

/*
 * [spec:nuos:req:emu.esp32p4.sd-uhs]
 * CMD11: R1 starts the voltage switch, the card driving CMD and DAT[3:0]
 * low; in the cases 4.2.4.4 lists it is illegal, with no response.
 */
static sd_rsp_type_t sd_cmd_VOLTAGE_SWITCH(SDState *sd, SDRequest req)
{
    const char *why = NULL;

    if (!sd_is_uhs(sd)) {
        why = "the card does not switch";
    } else if (!sd->s18r) {
        why = "ACMD41 did not ask for it (S18R 0)";
    } else if (sd->state != sd_ready_state) {
        why = "the card is not in the ready state";
    } else if (sd->signal_mv != SD_VOLTAGE_3_3V) {
        why = "the card signals at 1.8 V already";
    }
    if (why) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: CMD11 is illegal: %s\n",
                      sd->proto->name, why);
        return sd_illegal;
    }
    sd->vsw_state = SD_VSW_LOW;
    sd->cmd_line = false;
    sd->dat_lines = 0;
    return sd_r1;
}

/* CMD12 */
static sd_rsp_type_t sd_cmd_STOP_TRANSMISSION(SDState *sd, SDRequest req)
{
    switch (sd->state) {
    case sd_sendingdata_state:
        sd->state = sd_transfer_state;
        return sd_r1b;
    case sd_receivingdata_state:
        sd->state = sd_programming_state;
        /* Bzzzzzzztt .... Operation complete.  */
        sd->state = sd_transfer_state;
        return sd_r1;
    default:
        return sd_invalid_state_for_cmd(sd, req);
    }
}

/* CMD13 */
static sd_rsp_type_t sd_cmd_SEND_STATUS(SDState *sd, SDRequest req)
{
    if (sd_mode(sd) != sd_data_transfer_mode) {
        return sd_invalid_mode_for_cmd(sd, req);
    }

    switch (sd->state) {
    case sd_standby_state:
    case sd_transfer_state:
    case sd_sendingdata_state:
    case sd_receivingdata_state:
    case sd_programming_state:
    case sd_disconnect_state:
        break;
    default:
        return sd_invalid_state_for_cmd(sd, req);
    }

    if (sd_is_spi(sd)) {
        return spi_r2;
    }

    return sd_req_rca_same(sd, req) ? sd_r1 : sd_r0;
}

/* CMD15 */
static sd_rsp_type_t sd_cmd_GO_INACTIVE_STATE(SDState *sd, SDRequest req)
{
    if (sd_mode(sd) != sd_data_transfer_mode) {
        return sd_invalid_mode_for_cmd(sd, req);
    }
    switch (sd->state) {
    case sd_standby_state:
    case sd_transfer_state:
    case sd_sendingdata_state:
    case sd_receivingdata_state:
    case sd_programming_state:
    case sd_disconnect_state:
        break;
    default:
        return sd_invalid_state_for_cmd(sd, req);
    }
    if (sd_req_rca_same(sd, req)) {
        sd->state = sd_inactive_state;
    }

    return sd_r0;
}

/* CMD16 */
static sd_rsp_type_t sd_cmd_SET_BLOCKLEN(SDState *sd, SDRequest req)
{
    if (sd->state != sd_transfer_state) {
        return sd_invalid_state_for_cmd(sd, req);
    }
    if (req.arg > (1 << HWBLOCK_SHIFT)) {
        sd->card_status |= BLOCK_LEN_ERROR;
    } else {
        trace_sdcard_set_blocklen(req.arg);
        sd->blk_len = req.arg;
    }

    return sd_r1;
}

/* CMD17 */
static sd_rsp_type_t sd_cmd_READ_SINGLE_BLOCK(SDState *sd, SDRequest req)
{
    uint64_t addr;

    if (sd->state != sd_transfer_state) {
        return sd_invalid_state_for_cmd(sd, req);
    }

    addr = sd_req_get_address(sd, req);
    if (!address_in_range(sd, "READ_SINGLE_BLOCK", addr, sd->blk_len)) {
        return sd_r1;
    }

    sd_blk_read(sd, addr, sd->blk_len);
    return sd_cmd_to_sendingdata(sd, req, addr, NULL, sd->blk_len);
}

/*
 * [spec:nuos:req:emu.esp32p4.sd-uhs]
 * CMD19: the tuning block, in the transfer state at 1.8 V (an unlocked
 * card's: a locked one takes no class 2 command); illegal otherwise
 * (4.2.4.5).
 */
static sd_rsp_type_t sd_cmd_SEND_TUNING_BLOCK(SDState *sd, SDRequest req)
{
    if (sd->spec_version < SD_PHY_SPECv3_01_VERS) {
        return sd_cmd_illegal(sd, req);
    }
    if (sd->signal_mv != SD_VOLTAGE_1_8V) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: CMD19 at 3.3 V signalling is "
                      "illegal\n", sd->proto->name);
        return sd_illegal;
    }

    return sd_cmd_to_sendingdata(sd, req, 0,
                                 sd_tuning_block_pattern4,
                                 sizeof(sd_tuning_block_pattern4));
}

/* CMD23 */
static sd_rsp_type_t sd_cmd_SET_BLOCK_COUNT(SDState *sd, SDRequest req)
{
    if (sd->spec_version < SD_PHY_SPECv3_01_VERS) {
        return sd_cmd_illegal(sd, req);
    }

    if (sd->state != sd_transfer_state) {
        return sd_invalid_state_for_cmd(sd, req);
    }

    sd->multi_blk_cnt = req.arg;
    if (sd_is_emmc(sd)) {
        sd->multi_blk_cnt &= 0xffff;
    }
    trace_sdcard_set_block_count(sd->multi_blk_cnt);

    return sd_r1;
}

/* CMD24 */
static sd_rsp_type_t sd_cmd_WRITE_SINGLE_BLOCK(SDState *sd, SDRequest req)
{
    uint64_t addr;

    if (sd->state != sd_transfer_state) {
        return sd_invalid_state_for_cmd(sd, req);
    }

    addr = sd_req_get_address(sd, req);
    if (!address_in_range(sd, "WRITE_SINGLE_BLOCK", addr, sd->blk_len)) {
        return sd_r1;
    }

    if (sd->size <= SDSC_MAX_CAPACITY) {
        if (sd_wp_addr(sd, addr)) {
            sd->card_status |= WP_VIOLATION;
        }
    }
    if (sd->csd[14] & 0x30) {
        sd->card_status |= WP_VIOLATION;
    }

    sd->blk_written = 0;
    return sd_cmd_to_receivingdata(sd, req, addr, sd->blk_len);
}

/* CMD26 */
static sd_rsp_type_t emmc_cmd_PROGRAM_CID(SDState *sd, SDRequest req)
{
    return sd_cmd_to_receivingdata(sd, req, 0, sizeof(sd->cid));
}

/* CMD27 */
static sd_rsp_type_t sd_cmd_PROGRAM_CSD(SDState *sd, SDRequest req)
{
    return sd_cmd_to_receivingdata(sd, req, 0, sizeof(sd->csd));
}

static sd_rsp_type_t sd_cmd_SET_CLR_WRITE_PROT(SDState *sd, SDRequest req,
                                               bool is_write)
{
    uint64_t addr;

    if (sd->size > SDSC_MAX_CAPACITY) {
        return sd_illegal;
    }

    if (sd->state != sd_transfer_state) {
        return sd_invalid_state_for_cmd(sd, req);
    }

    addr = sd_req_get_address(sd, req);
    if (!address_in_range(sd, is_write ? "SET_WRITE_PROT" : "CLR_WRITE_PROT",
                          addr, 1)) {
        return sd_r1b;
    }

    sd->state = sd_programming_state;
    if (is_write) {
        set_bit(sd_addr_to_wpnum(addr), sd->wp_group_bmap);
    } else {
        clear_bit(sd_addr_to_wpnum(addr), sd->wp_group_bmap);
    }
    /* Bzzzzzzztt .... Operation complete.  */
    sd->state = sd_transfer_state;
    return sd_r1;
}

/* CMD28 */
static sd_rsp_type_t sd_cmd_SET_WRITE_PROT(SDState *sd, SDRequest req)
{
    return sd_cmd_SET_CLR_WRITE_PROT(sd, req, true);
}

/* CMD29 */
static sd_rsp_type_t sd_cmd_CLR_WRITE_PROT(SDState *sd, SDRequest req)
{
    return sd_cmd_SET_CLR_WRITE_PROT(sd, req, false);
}

/* CMD30 */
static sd_rsp_type_t sd_cmd_SEND_WRITE_PROT(SDState *sd, SDRequest req)
{
    uint64_t addr;
    uint32_t data;

    if (sd->size > SDSC_MAX_CAPACITY) {
        return sd_illegal;
    }

    if (sd->state != sd_transfer_state) {
        return sd_invalid_state_for_cmd(sd, req);
    }

    addr = sd_req_get_address(sd, req);
    if (!address_in_range(sd, "SEND_WRITE_PROT", addr, sd->blk_len)) {
        return sd_r1;
    }

    data = sd_wpbits(sd, req.arg);
    return sd_cmd_to_sendingdata(sd, req, addr, &data, sizeof(data));
}

/* CMD32 */
static sd_rsp_type_t sd_cmd_ERASE_WR_BLK_START(SDState *sd, SDRequest req)
{
    if (sd->state != sd_transfer_state) {
        return sd_invalid_state_for_cmd(sd, req);
    }
    sd->erase_start = req.arg;
    return sd_r1;
}

/* CMD33 */
static sd_rsp_type_t sd_cmd_ERASE_WR_BLK_END(SDState *sd, SDRequest req)
{
    if (sd->state != sd_transfer_state) {
        return sd_invalid_state_for_cmd(sd, req);
    }
    sd->erase_end = req.arg;
    return sd_r1;
}

/* CMD38 */
static sd_rsp_type_t sd_cmd_ERASE(SDState *sd, SDRequest req)
{
    if (sd->state != sd_transfer_state) {
        return sd_invalid_state_for_cmd(sd, req);
    }
    if (sd->csd[14] & 0x30) {
        sd->card_status |= WP_VIOLATION;
        return sd_r1b;
    }

    sd->state = sd_programming_state;
    sd_erase(sd);
    /* Bzzzzzzztt .... Operation complete.  */
    sd->state = sd_transfer_state;
    return sd_r1b;
}

/* CMD42 */
static sd_rsp_type_t sd_cmd_LOCK_UNLOCK(SDState *sd, SDRequest req)
{
    return sd_cmd_to_receivingdata(sd, req, 0, 0);
}

/* CMD55 */
static sd_rsp_type_t sd_cmd_APP_CMD(SDState *sd, SDRequest req)
{
    switch (sd->state) {
    case sd_ready_state:
    case sd_identification_state:
    case sd_inactive_state:
    case sd_sleep_state:
        return sd_invalid_state_for_cmd(sd, req);
    case sd_idle_state:
        if (!sd_is_spi(sd) && sd_req_get_rca(sd, req) != 0x0000) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "SD: illegal RCA 0x%04x for APP_CMD\n", req.cmd);
        }
        /* fall-through */
    default:
        break;
    }
    if (!sd_is_spi(sd) && !sd_req_rca_same(sd, req)) {
        return sd_r0;
    }
    sd->expecting_acmd = true;
    sd->card_status |= APP_CMD;

    return sd_r1;
}

/* CMD56 */
static sd_rsp_type_t sd_cmd_GEN_CMD(SDState *sd, SDRequest req)
{
    if (sd->state != sd_transfer_state) {
        return sd_invalid_state_for_cmd(sd, req);
    }

    /* Vendor specific command: our model is RAZ/WI */
    if (req.arg & 1) {
        memset(sd->data, 0, sizeof(sd->data));
        return sd_cmd_to_sendingdata(sd, req, 0, NULL, 0);
    } else {
        return sd_cmd_to_receivingdata(sd, req, 0, 0);
    }
}

/* CMD58 */
static sd_rsp_type_t spi_cmd_READ_OCR(SDState *sd, SDRequest req)
{
    return sd_r3;
}

/* CMD59 */
static sd_rsp_type_t spi_cmd_CRC_ON_OFF(SDState *sd, SDRequest req)
{
    return sd_r1;
}

/* ACMD6 */
static sd_rsp_type_t sd_acmd_SET_BUS_WIDTH(SDState *sd, SDRequest req)
{
    if (sd->state != sd_transfer_state) {
        return sd_invalid_state_for_cmd(sd, req);
    }

    sd->sd_status[0] &= 0x3f;
    sd->sd_status[0] |= (req.arg & 0x03) << 6;
    return sd_r1;
}

/* ACMD13 */
static sd_rsp_type_t sd_acmd_SD_STATUS(SDState *sd, SDRequest req)
{
    sd_rsp_type_t rsp;

    rsp = sd_cmd_to_sendingdata(sd, req, 0,
                                sd->sd_status, sizeof(sd->sd_status));
    if (sd_is_spi(sd) && rsp != sd_illegal) {
        return spi_r2;
    }
    return rsp;
}

/* ACMD22 */
static sd_rsp_type_t sd_acmd_SEND_NUM_WR_BLOCKS(SDState *sd, SDRequest req)
{
    return sd_cmd_to_sendingdata(sd, req, 0,
                                 &sd->blk_written, sizeof(sd->blk_written));
}

/* ACMD23 */
static sd_rsp_type_t sd_acmd_SET_WR_BLK_ERASE_COUNT(SDState *sd, SDRequest req)
{
    if (sd->state != sd_transfer_state) {
        return sd_invalid_state_for_cmd(sd, req);
    }
    return sd_r1;
}

/* ACMD41 */
static sd_rsp_type_t sd_cmd_SEND_OP_COND(SDState *sd, SDRequest req)
{
    if (sd->state != sd_idle_state) {
        return sd_invalid_state_for_cmd(sd, req);
    }

    /* [spec:nuos:req:emu.esp32p4.sd-uhs] */
    /* The first ACMD41 with a voltage window gives S18R (4.2.3.1). */
    if (!sd->s18r_latched && (req.arg & ACMD41_ENQUIRY_MASK)) {
        sd->s18r_latched = true;
        sd->s18r = FIELD_EX32(req.arg, OCR, ACCEPT_SWITCH_1V8);
    }

    /*
     * If it's the first ACMD41 since reset, we need to decide
     * whether to power up. If this is not an enquiry ACMD41,
     * we immediately report power on and proceed below to the
     * ready state, but if it is, we set a timer to model a
     * delay for power up. This works around a bug in EDK2
     * UEFI, which sends an initial enquiry ACMD41, but
     * assumes that the card is in ready state as soon as it
     * sees the power up bit set.
     */
    if (!FIELD_EX32(sd->ocr, OCR, CARD_POWER_UP)) {
        if ((req.arg & ACMD41_ENQUIRY_MASK) != 0) {
            timer_del(sd->ocr_power_timer);
            sd_ocr_powerup(sd);
        } else {
            trace_sdcard_inquiry_cmd41();
            if (!timer_pending(sd->ocr_power_timer)) {
                timer_mod_ns(sd->ocr_power_timer,
                             (qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)
                              + OCR_POWER_DELAY_NS));
            }
        }
    }

    /* S18A, valid with the busy bit set */
    sd->ocr = FIELD_DP32(sd->ocr, OCR, ACCEPT_SWITCH_1V8,
                         FIELD_EX32(sd->ocr, OCR, CARD_POWER_UP) &&
                         sd_accepts_1v8(sd));

    if (sd_is_spi(sd)) {
        sd->state = sd_ready_state;
        return sd_r1;
    } else {
        if (FIELD_EX32(sd->ocr & req.arg, OCR, VDD_VOLTAGE_WINDOW)) {
            /*
             * We accept any voltage.  10000 V is nothing.
             *
             * Once we're powered up, we advance straight to ready state
             * unless it's an enquiry ACMD41 (bits 23:0 == 0).
             */
            sd->state = sd_ready_state;
        }
        return sd_r3;
    }
}

/* ACMD42 */
static sd_rsp_type_t sd_acmd_SET_CLR_CARD_DETECT(SDState *sd, SDRequest req)
{
    if (sd->state != sd_transfer_state) {
        return sd_invalid_state_for_cmd(sd, req);
    }

    /* Bringing in the 50KOhm pull-up resistor... Done.  */
    return sd_r1;
}

/* ACMD51 */
static sd_rsp_type_t sd_acmd_SEND_SCR(SDState *sd, SDRequest req)
{
    return sd_cmd_to_sendingdata(sd, req, 0, sd->scr, sizeof(sd->scr));
}

static sd_rsp_type_t sd_normal_command(SDState *sd, SDRequest req)
{
    uint64_t addr;

    sd->last_cmd_name = sd_cmd_name(sd, req.cmd);
    /* CMD55 precedes an ACMD, so we are not interested in tracing it.
     * However there is no ACMD55, so we want to trace this particular case.
     */
    if (req.cmd != 55 || sd->expecting_acmd) {
        trace_sdcard_normal_command(sd->proto->name,
                                    sd->last_cmd_name, req.cmd,
                                    req.arg,
                                    sd_mode_name(sd_mode(sd)),
                                    sd_state_name(sd->state));
    }

    /* Not interpreting this as an app command */
    sd->card_status &= ~APP_CMD;

    /* CMD23 (set block count) must be immediately followed by CMD18 or CMD25
     * if not, its effects are cancelled */
    if (sd->multi_blk_cnt != 0 && !(req.cmd == 18 || req.cmd == 25)) {
        sd->multi_blk_cnt = 0;
    }

    if (sd->proto->cmd[req.cmd].class == 6 && FIELD_EX32(sd->ocr, OCR,
                                                         CARD_CAPACITY)) {
        /* Only Standard Capacity cards support class 6 commands */
        return sd_illegal;
    }

    if (sd->proto->cmd[req.cmd].handler) {
        return sd->proto->cmd[req.cmd].handler(sd, req);
    }

    switch (req.cmd) {
    /* Block read commands (Class 2) */
    case 18:  /* CMD18:  READ_MULTIPLE_BLOCK */
        addr = sd_req_get_address(sd, req);
        switch (sd->state) {
        case sd_transfer_state:

            if (!address_in_range(sd, "READ_BLOCK", addr, sd->blk_len)) {
                return sd_r1;
            }

            sd->state = sd_sendingdata_state;
            sd->data_start = addr;
            sd->data_offset = 0;
            return sd_r1;

        default:
            break;
        }
        break;

    /* Block write commands (Class 4) */
    case 25:  /* CMD25:  WRITE_MULTIPLE_BLOCK */
        addr = sd_req_get_address(sd, req);
        switch (sd->state) {
        case sd_transfer_state:

            if (!address_in_range(sd, "WRITE_BLOCK", addr, sd->blk_len)) {
                return sd_r1;
            }

            sd->state = sd_receivingdata_state;
            sd->data_start = addr;
            sd->data_offset = 0;
            sd->blk_written = 0;

            if (sd->size <= SDSC_MAX_CAPACITY) {
                if (sd_wp_addr(sd, sd->data_start)) {
                    sd->card_status |= WP_VIOLATION;
                }
            }
            if (sd->csd[14] & 0x30) {
                sd->card_status |= WP_VIOLATION;
            }
            return sd_r1;

        default:
            break;
        }
        break;

    default:
        qemu_log_mask(LOG_GUEST_ERROR, "SD: Unknown CMD%i\n", req.cmd);
        return sd_illegal;
    }

    return sd_invalid_state_for_cmd(sd, req);
}

static sd_rsp_type_t sd_app_command(SDState *sd,
                                    SDRequest req)
{
    sd->last_cmd_name = sd_acmd_name(sd, req.cmd);
    trace_sdcard_app_command(sd->proto->name, sd->last_cmd_name,
                             req.cmd, req.arg,
                             sd_mode_name(sd_mode(sd)),
                             sd_state_name(sd->state));
    sd->card_status |= APP_CMD;

    if (sd->proto->acmd[req.cmd].handler) {
        return sd->proto->acmd[req.cmd].handler(sd, req);
    }

    switch (req.cmd) {
    case 18:    /* Reserved for SD security applications */
    case 25:
    case 26:
    case 38:
    case 43 ... 49:
        /* Refer to the "SD Specifications Part3 Security Specification" for
         * information about the SD Security Features.
         */
        qemu_log_mask(LOG_UNIMP, "SD: CMD%i Security not implemented\n",
                      req.cmd);
        return sd_illegal;

    default:
        /* Fall back to standard commands.  */
        return sd_normal_command(sd, req);
    }

    qemu_log_mask(LOG_GUEST_ERROR, "SD: ACMD%i in a wrong state\n", req.cmd);
    return sd_illegal;
}

static bool cmd_valid_while_locked(SDState *sd, unsigned cmd)
{
    unsigned cmd_class;

    /* Valid commands in locked state:
     * basic class (0)
     * lock card class (7)
     * CMD16
     * implicitly, the ACMD prefix CMD55
     * ACMD41 and ACMD42
     * Anything else provokes an "illegal command" response.
     */
    if (sd->expecting_acmd) {
        return cmd == 41 || cmd == 42;
    }
    if (cmd == 16 || cmd == 55) {
        return true;
    }
    if (!sd->proto->cmd[cmd].handler) {
        return false;
    }
    cmd_class = sd->proto->cmd[cmd].class;

    return cmd_class == 0 || cmd_class == 7;
}

static size_t sd_do_command(SDState *sd, SDRequest *req,
                            uint8_t *response, size_t respsz)
{
    int last_state;
    sd_rsp_type_t rtype;
    int rsplen;

    if (!sd->blk || !blk_is_inserted(sd->blk)) {
        return 0;
    }

    if (sd->state == sd_inactive_state) {
        rtype = sd_illegal;
        goto send_response;
    }

    /* [spec:nuos:req:emu.esp32p4.sd-uhs] */
    if (sd->vsw_state != SD_VSW_NONE) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: CMD%u %s: the card holds its "
                      "lines and does not answer\n", sd->proto->name,
                      req->cmd, sd->vsw_state == SD_VSW_FAILED ?
                      "after a failed voltage switch" :
                      "during the voltage switch");
        return 0;
    }

    if (sd_req_crc_validate(req)) {
        sd->card_status |= COM_CRC_ERROR;
        rtype = sd_illegal;
        goto send_response;
    }

    if (req->cmd >= SDMMC_CMD_MAX) {
        qemu_log_mask(LOG_GUEST_ERROR, "SD: incorrect command 0x%02x\n",
                      req->cmd);
        req->cmd &= 0x3f;
    }

    if (sd->state == sd_sleep_state && req->cmd) {
        qemu_log_mask(LOG_GUEST_ERROR, "SD: Card is sleeping\n");
        rtype = sd_r0;
        goto send_response;
    }

    if (sd->card_status & CARD_IS_LOCKED) {
        if (!cmd_valid_while_locked(sd, req->cmd)) {
            sd->card_status |= ILLEGAL_COMMAND;
            sd->expecting_acmd = false;
            qemu_log_mask(LOG_GUEST_ERROR, "SD: Card is locked\n");
            rtype = sd_illegal;
            goto send_response;
        }
    }

    last_state = sd->state;

    if (sd->expecting_acmd) {
        sd->expecting_acmd = false;
        rtype = sd_app_command(sd, *req);
    } else {
        rtype = sd_normal_command(sd, *req);
    }

    if (rtype == sd_illegal) {
        sd->card_status |= ILLEGAL_COMMAND;
    } else {
        /* Valid command, we can update the 'state before command' bits.
         * (Do this now so they appear in r1 responses.)
         */
        sd->card_status = FIELD_DP32(sd->card_status, CSR,
                                     CURRENT_STATE, last_state);
    }

send_response:
    rsplen = sd_response_size(sd, rtype);
    assert(rsplen <= respsz);

    switch (rtype) {
    case sd_r1:
    case sd_r1b:
        sd_response_r1_make(sd, response);
        break;

    case spi_r2:
        spi_response_r2_make(sd, response);
        break;

    case sd_r2_i:
        memcpy(response, sd->cid, sizeof(sd->cid));
        break;

    case sd_r2_s:
        memcpy(response, sd->csd, sizeof(sd->csd));
        break;

    case sd_r3:
        sd_response_r3_make(sd, response);
        break;

    case sd_r6:
        sd_response_r6_make(sd, response);
        break;

    case sd_r7:
        sd_response_r7_make(sd, response);
        break;

    case sd_r0:
        /*
         * Invalid state transition, reset implementation
         * fields to avoid OOB abuse.
         */
        sd->data_start = 0;
        sd->data_offset = 0;
        /* fall-through */
    case sd_illegal:
        break;
    default:
        g_assert_not_reached();
    }
    trace_sdcard_response(sd_response_name(rtype), rsplen);

    if (rtype != sd_illegal) {
        /* Clear the "clear on valid command" status bits now we've
         * sent any response
         */
        sd->card_status &= ~CARD_STATUS_B;
    }

#ifdef DEBUG_SD
    qemu_hexdump(stderr, "Response", response, rsplen);
#endif

    sd->current_cmd = rtype == sd_illegal ? 0 : req->cmd;

    return rsplen;
}

/* Return true if buffer is consumed. Configured by sd_cmd_to_receivingdata() */
static bool sd_generic_write_data(SDState *sd, const void *buf, size_t *len)
{
    size_t to_write = MIN(sd->data_size - sd->data_offset, *len);

    memcpy(&sd->data[sd->data_offset], buf, to_write);
    sd->data_offset += to_write;
    *len = to_write;

    if (sd->data_offset >= sd->data_size) {
        sd->state = sd_transfer_state;
        return true;
    }
    return false;
}

/* Return true when buffer is consumed. Configured by sd_cmd_to_sendingdata() */
static bool sd_generic_read_data(SDState *sd, void *buf, size_t *len)
{
    size_t to_read = MIN(sd->data_size - sd->data_offset, *len);

    memcpy(buf, &sd->data[sd->data_offset], to_read);
    sd->data_offset += to_read;
    *len = to_read;

    if (sd->data_offset >= sd->data_size) {
        sd->state = sd_transfer_state;
        return true;
    }

    return false;
}

static void sdcard_write_data_dump(const char *proto, const char *cmd_desc,
                                   uint8_t cmd, uint32_t offset,
                                   const void *buf, size_t len)
{
    g_autoptr(GString) str = NULL;

    if (trace_event_get_state_backends(TRACE_SDCARD_WRITE_DATA)) {
        str = qemu_hexdump_line(NULL, buf, len, 8, 0);
        trace_sdcard_write_data(proto, cmd_desc, cmd, offset, str->str);
    }
}

static size_t sd_write_data(SDState *sd, const void *buf, size_t length)
{
    unsigned int partition_access;
    int i;
    const uint8_t *value = buf;

    if (!sd->blk || !blk_is_inserted(sd->blk)) {
        return length;
    }

    if (sd->state != sd_receivingdata_state) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: not in Receiving-Data state\n", __func__);
        return length;
    }

    if (sd->card_status & (ADDRESS_ERROR | WP_VIOLATION))
        return length;

    sdcard_write_data_dump(sd->proto->name,
                           sd->last_cmd_name,
                           sd->current_cmd, sd->data_offset, buf, length);
    switch (sd->current_cmd) {
    case 24:  /* CMD24:  WRITE_SINGLE_BLOCK */
        if (sd_generic_write_data(sd, buf, &length)) {
            /* TODO: Check CRC before committing */
            sd->state = sd_programming_state;
            sd_blk_write(sd, sd->data_start, sd->data_offset);
            sd->blk_written ++;
            sd->csd[14] |= 0x40;
            /* Bzzzzzzztt .... Operation complete.  */
            sd->state = sd_transfer_state;
        }
        break;

    case 25:  /* CMD25:  WRITE_MULTIPLE_BLOCK */
        /*
         * Only read one byte at a time. We will be called again with the
         * remaining.
         */
        length = 1;

        if (sd->data_offset == 0) {
            /* Start of the block - let's check the address is valid */
            if (!address_in_range(sd, "WRITE_MULTIPLE_BLOCK",
                                  sd->data_start, sd->blk_len)) {
                break;
            }
            if (sd->size <= SDSC_MAX_CAPACITY) {
                if (sd_wp_addr(sd, sd->data_start)) {
                    sd->card_status |= WP_VIOLATION;
                    break;
                }
            }
        }
        sd->data[sd->data_offset++] = value[0];
        if (sd->data_offset >= sd->blk_len) {
            /* TODO: Check CRC before committing */
            sd->state = sd_programming_state;
            partition_access = sd->ext_csd[EXT_CSD_PART_CONFIG]
                    & EXT_CSD_PART_CONFIG_ACC_MASK;
            if (partition_access == EXT_CSD_PART_CONFIG_ACC_RPMB) {
                emmc_rpmb_blk_write(sd, sd->data_start, sd->data_offset);
            } else {
                sd_blk_write(sd, sd->data_start, sd->data_offset);
            }
            sd->blk_written++;
            sd->data_start += sd->blk_len;
            sd->data_offset = 0;
            sd->csd[14] |= 0x40;

            /* Bzzzzzzztt .... Operation complete.  */
            if (sd->multi_blk_cnt != 0) {
                if (--sd->multi_blk_cnt == 0) {
                    /* Stop! */
                    sd->state = sd_transfer_state;
                    break;
                }
            }

            sd->state = sd_receivingdata_state;
        }
        break;

    case 26:  /* CMD26:  PROGRAM_CID */
        if (sd_generic_write_data(sd, buf, &length)) {
            /* TODO: Check CRC before committing */
            sd->state = sd_programming_state;
            for (i = 0; i < sizeof(sd->cid); i ++)
                if ((sd->cid[i] | 0x00) != sd->data[i])
                    sd->card_status |= CID_CSD_OVERWRITE;

            if (!(sd->card_status & CID_CSD_OVERWRITE))
                for (i = 0; i < sizeof(sd->cid); i ++) {
                    sd->cid[i] |= 0x00;
                    sd->cid[i] &= sd->data[i];
                }
            /* Bzzzzzzztt .... Operation complete.  */
            sd->state = sd_transfer_state;
        }
        break;

    case 27:  /* CMD27:  PROGRAM_CSD */
        if (sd_generic_write_data(sd, buf, &length)) {
            /* TODO: Check CRC before committing */
            sd->state = sd_programming_state;
            for (i = 0; i < sizeof(sd->csd); i ++)
                if ((sd->csd[i] | sd_csd_rw_mask[i]) !=
                    (sd->data[i] | sd_csd_rw_mask[i]))
                    sd->card_status |= CID_CSD_OVERWRITE;

            /* Copy flag (OTP) & Permanent write protect */
            if (sd->csd[14] & ~sd->data[14] & 0x60)
                sd->card_status |= CID_CSD_OVERWRITE;

            if (!(sd->card_status & CID_CSD_OVERWRITE))
                for (i = 0; i < sizeof(sd->csd); i ++) {
                    sd->csd[i] |= sd_csd_rw_mask[i];
                    sd->csd[i] &= sd->data[i];
                }
            /* Bzzzzzzztt .... Operation complete.  */
            sd->state = sd_transfer_state;
        }
        break;

    case 42:  /* CMD42:  LOCK_UNLOCK */
        if (sd_generic_write_data(sd, buf, &length)) {
            /* TODO: Check CRC before committing */
            sd->state = sd_programming_state;
            sd_lock_command(sd);
            /* Bzzzzzzztt .... Operation complete.  */
            sd->state = sd_transfer_state;
        }
        break;

    case 56:  /* CMD56:  GEN_CMD */
        sd_generic_write_data(sd, buf, &length);
        break;

    default:
        g_assert_not_reached();
    }

    return length;
}

static size_t sd_read_data(SDState *sd, void *buf, size_t length)
{
    /* TODO: Append CRCs */
    const uint8_t dummy_byte = 0x00;
    unsigned int partition_access;
    uint32_t io_len;
    uint8_t *value = buf;

    if (!sd->blk || !blk_is_inserted(sd->blk)) {
        memset(buf, dummy_byte, length);
        return length;
    }

    if (sd->state != sd_sendingdata_state) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: not in Sending-Data state\n", __func__);
        memset(buf, dummy_byte, length);
        return length;
    }

    if (sd->card_status & (ADDRESS_ERROR | WP_VIOLATION)) {
        memset(buf, dummy_byte, length);
        return length;
    }

    io_len = sd_blk_len(sd);

    trace_sdcard_read_data(sd->proto->name,
                           sd->last_cmd_name, sd->current_cmd,
                           sd->data_offset, sd->data_size, io_len);
    switch (sd->current_cmd) {
    case 6:  /* CMD6:   SWITCH_FUNCTION */
        if (sd_generic_read_data(sd, buf, &length)) {
            sd_function_switch_done(sd);
        }
        break;

    case 8:  /* CMD8:   SEND_EXT_CSD */
    case 9:  /* CMD9:   SEND_CSD */
    case 10: /* CMD10:  SEND_CID */
    case 13: /* ACMD13: SD_STATUS */
    case 17: /* CMD17:  READ_SINGLE_BLOCK */
    case 19: /* CMD19:  SEND_TUNING_BLOCK (SD) */
    case 22: /* ACMD22: SEND_NUM_WR_BLOCKS */
    case 30: /* CMD30:  SEND_WRITE_PROT */
    case 51: /* ACMD51: SEND_SCR */
    case 56: /* CMD56:  GEN_CMD */
        sd_generic_read_data(sd, buf, &length);
        break;

    case 18:  /* CMD18:  READ_MULTIPLE_BLOCK */
        /*
         * We will only read one byte at a time. We will be called again with
         * the remaining buffer.
         */
        length = 1;

        if (sd->data_offset == 0) {
            if (!address_in_range(sd, "READ_MULTIPLE_BLOCK",
                                  sd->data_start, io_len)) {
                *value = dummy_byte;
                return length;
            }
            partition_access = sd->ext_csd[EXT_CSD_PART_CONFIG]
                    & EXT_CSD_PART_CONFIG_ACC_MASK;
            if (partition_access == EXT_CSD_PART_CONFIG_ACC_RPMB) {
                emmc_rpmb_blk_read(sd, sd->data_start, io_len);
            } else {
                sd_blk_read(sd, sd->data_start, io_len);
            }
        }
        *value = sd->data[sd->data_offset++];

        if (sd->data_offset >= io_len) {
            sd->data_start += io_len;
            sd->data_offset = 0;

            if (sd->multi_blk_cnt != 0) {
                if (--sd->multi_blk_cnt == 0) {
                    /* Stop! */
                    sd->state = sd_transfer_state;
                    break;
                }
            }
        }
        break;

    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: DAT read illegal for command %s\n",
                                       __func__, sd->last_cmd_name);
        memset(buf, dummy_byte, length);
    }

    return length;
}

static bool sd_receive_ready(SDState *sd)
{
    return sd->state == sd_receivingdata_state;
}

static bool sd_data_ready(SDState *sd)
{
    return sd->state == sd_sendingdata_state;
}

static const SDProto sd_proto_spi = {
    .name = "SPI",
    .cmd = {
        [0]  = {0,  sd_spi, "GO_IDLE_STATE", sd_cmd_GO_IDLE_STATE},
        [1]  = {0,  sd_spi, "SEND_OP_COND", sd_cmd_SEND_OP_COND},
        [5]  = {9,  sd_spi, "IO_SEND_OP_COND", sd_cmd_optional},
        [6]  = {10, sd_spi, "SWITCH_FUNCTION", sd_cmd_SWITCH_FUNCTION},
        [8]  = {0,  sd_spi, "SEND_IF_COND", sd_cmd_SEND_IF_COND},
        [9]  = {0,  sd_spi, "SEND_CSD", spi_cmd_SEND_CSD},
        [10] = {0,  sd_spi, "SEND_CID", spi_cmd_SEND_CID},
        [12] = {0,  sd_spi, "STOP_TRANSMISSION", sd_cmd_STOP_TRANSMISSION},
        [13] = {0,  sd_spi, "SEND_STATUS", sd_cmd_SEND_STATUS},
        [16] = {2,  sd_spi, "SET_BLOCKLEN", sd_cmd_SET_BLOCKLEN},
        [17] = {2,  sd_spi, "READ_SINGLE_BLOCK", sd_cmd_READ_SINGLE_BLOCK},
        [24] = {4,  sd_spi, "WRITE_SINGLE_BLOCK", sd_cmd_WRITE_SINGLE_BLOCK},
        [27] = {4,  sd_spi, "PROGRAM_CSD", sd_cmd_PROGRAM_CSD},
        [28] = {6,  sd_spi, "SET_WRITE_PROT", sd_cmd_SET_WRITE_PROT},
        [29] = {6,  sd_spi, "CLR_WRITE_PROT", sd_cmd_CLR_WRITE_PROT},
        [30] = {6,  sd_spi, "SEND_WRITE_PROT", sd_cmd_SEND_WRITE_PROT},
        [32] = {5,  sd_spi, "ERASE_WR_BLK_START", sd_cmd_ERASE_WR_BLK_START},
        [33] = {5,  sd_spi, "ERASE_WR_BLK_END", sd_cmd_ERASE_WR_BLK_END},
        [34] = {10, sd_spi, "READ_SEC_CMD", sd_cmd_optional},
        [35] = {10, sd_spi, "WRITE_SEC_CMD", sd_cmd_optional},
        [36] = {10, sd_spi, "SEND_PSI", sd_cmd_optional},
        [37] = {10, sd_spi, "CONTROL_ASSD_SYSTEM", sd_cmd_optional},
        [38] = {5,  sd_spi, "ERASE", sd_cmd_ERASE},
        [42] = {7,  sd_spi, "LOCK_UNLOCK", sd_cmd_LOCK_UNLOCK},
        [50] = {10, sd_spi, "DIRECT_SECURE_READ", sd_cmd_optional},
        [52] = {9,  sd_spi, "IO_RW_DIRECT", sd_cmd_optional},
        [53] = {9,  sd_spi, "IO_RW_EXTENDED", sd_cmd_optional},
        [55] = {8,  sd_spi, "APP_CMD", sd_cmd_APP_CMD},
        [56] = {8,  sd_spi, "GEN_CMD", sd_cmd_GEN_CMD},
        [57] = {10, sd_spi, "DIRECT_SECURE_WRITE", sd_cmd_optional},
        [58] = {0,  sd_spi, "READ_OCR", spi_cmd_READ_OCR},
        [59] = {0,  sd_spi, "CRC_ON_OFF", spi_cmd_CRC_ON_OFF},
    },
    .acmd = {
        [13] = {8,  sd_spi, "SD_STATUS", sd_acmd_SD_STATUS},
        [22] = {8,  sd_spi, "SEND_NUM_WR_BLOCKS", sd_acmd_SEND_NUM_WR_BLOCKS},
        [23] = {8,  sd_spi, "SET_WR_BLK_ERASE_COUNT", sd_acmd_SET_WR_BLK_ERASE_COUNT},
        [41] = {8,  sd_spi, "SEND_OP_COND", sd_cmd_SEND_OP_COND},
        [42] = {8,  sd_spi, "SET_CLR_CARD_DETECT", sd_acmd_SET_CLR_CARD_DETECT},
        [51] = {8,  sd_spi, "SEND_SCR", sd_acmd_SEND_SCR},
    },
};

static const SDProto sd_proto_sd = {
    .name = "SD",
    .cmd = {
        [0]  = {0,  sd_bc,   "GO_IDLE_STATE", sd_cmd_GO_IDLE_STATE},
        [2]  = {0,  sd_bcr,  "ALL_SEND_CID", sd_cmd_ALL_SEND_CID},
        [3]  = {0,  sd_bcr,  "SEND_RELATIVE_ADDR", sd_cmd_SEND_RELATIVE_ADDR},
        [4]  = {0,  sd_bc,   "SEND_DSR", sd_cmd_unimplemented},
        [5]  = {9,  sd_bc,   "IO_SEND_OP_COND", sd_cmd_optional},
        [6]  = {10, sd_adtc, "SWITCH_FUNCTION", sd_cmd_SWITCH_FUNCTION},
        [7]  = {0,  sd_ac,   "(DE)SELECT_CARD", sd_cmd_DE_SELECT_CARD},
        [8]  = {0,  sd_bcr,  "SEND_IF_COND", sd_cmd_SEND_IF_COND},
        [9]  = {0,  sd_ac,   "SEND_CSD", sd_cmd_SEND_CSD},
        [10] = {0,  sd_ac,   "SEND_CID", sd_cmd_SEND_CID},
        [11] = {0,  sd_ac,   "VOLTAGE_SWITCH", sd_cmd_VOLTAGE_SWITCH},
        [12] = {0,  sd_ac,   "STOP_TRANSMISSION", sd_cmd_STOP_TRANSMISSION},
        [13] = {0,  sd_ac,   "SEND_STATUS", sd_cmd_SEND_STATUS},
        [15] = {0,  sd_ac,   "GO_INACTIVE_STATE", sd_cmd_GO_INACTIVE_STATE},
        [16] = {2,  sd_ac,   "SET_BLOCKLEN", sd_cmd_SET_BLOCKLEN},
        [17] = {2,  sd_adtc, "READ_SINGLE_BLOCK", sd_cmd_READ_SINGLE_BLOCK},
        [19] = {2,  sd_adtc, "SEND_TUNING_BLOCK", sd_cmd_SEND_TUNING_BLOCK},
        [20] = {2,  sd_ac,   "SPEED_CLASS_CONTROL", sd_cmd_optional},
        [23] = {2,  sd_ac,   "SET_BLOCK_COUNT", sd_cmd_SET_BLOCK_COUNT},
        [24] = {4,  sd_adtc, "WRITE_SINGLE_BLOCK", sd_cmd_WRITE_SINGLE_BLOCK},
        [27] = {4,  sd_adtc, "PROGRAM_CSD", sd_cmd_PROGRAM_CSD},
        [28] = {6,  sd_ac,   "SET_WRITE_PROT", sd_cmd_SET_WRITE_PROT},
        [29] = {6,  sd_ac,   "CLR_WRITE_PROT", sd_cmd_CLR_WRITE_PROT},
        [30] = {6,  sd_adtc, "SEND_WRITE_PROT", sd_cmd_SEND_WRITE_PROT},
        [32] = {5,  sd_ac,   "ERASE_WR_BLK_START", sd_cmd_ERASE_WR_BLK_START},
        [33] = {5,  sd_ac,   "ERASE_WR_BLK_END", sd_cmd_ERASE_WR_BLK_END},
        [34] = {10, sd_adtc, "READ_SEC_CMD", sd_cmd_optional},
        [35] = {10, sd_adtc, "WRITE_SEC_CMD", sd_cmd_optional},
        [36] = {10, sd_adtc, "SEND_PSI", sd_cmd_optional},
        [37] = {10, sd_ac,   "CONTROL_ASSD_SYSTEM", sd_cmd_optional},
        [38] = {5,  sd_ac,   "ERASE", sd_cmd_ERASE},
        [42] = {7,  sd_adtc, "LOCK_UNLOCK", sd_cmd_LOCK_UNLOCK},
        [43] = {1,  sd_ac,   "Q_MANAGEMENT", sd_cmd_optional},
        [44] = {1,  sd_ac,   "Q_TASK_INFO_A", sd_cmd_optional},
        [45] = {1,  sd_ac,   "Q_TASK_INFO_B", sd_cmd_optional},
        [46] = {1,  sd_adtc, "Q_RD_TASK", sd_cmd_optional},
        [47] = {1,  sd_adtc, "Q_WR_TASK", sd_cmd_optional},
        [48] = {1,  sd_adtc, "READ_EXTR_SINGLE", sd_cmd_optional},
        [49] = {1,  sd_adtc, "WRITE_EXTR_SINGLE", sd_cmd_optional},
        [50] = {10, sd_adtc, "DIRECT_SECURE_READ", sd_cmd_optional},
        [52] = {9,  sd_bc,   "IO_RW_DIRECT", sd_cmd_optional},
        [53] = {9,  sd_bc,   "IO_RW_EXTENDED", sd_cmd_optional},
        [55] = {8,  sd_ac,   "APP_CMD", sd_cmd_APP_CMD},
        [56] = {8,  sd_adtc, "GEN_CMD", sd_cmd_GEN_CMD},
        [57] = {10, sd_adtc, "DIRECT_SECURE_WRITE", sd_cmd_optional},
        [58] = {11, sd_adtc, "READ_EXTR_MULTI", sd_cmd_optional},
        [59] = {11, sd_adtc, "WRITE_EXTR_MULTI", sd_cmd_optional},
    },
    .acmd = {
        [6]  = {8,  sd_ac,   "SET_BUS_WIDTH", sd_acmd_SET_BUS_WIDTH},
        [13] = {8,  sd_adtc, "SD_STATUS", sd_acmd_SD_STATUS},
        [22] = {8,  sd_adtc, "SEND_NUM_WR_BLOCKS", sd_acmd_SEND_NUM_WR_BLOCKS},
        [23] = {8,  sd_ac,   "SET_WR_BLK_ERASE_COUNT", sd_acmd_SET_WR_BLK_ERASE_COUNT},
        [41] = {8,  sd_bcr,  "SEND_OP_COND", sd_cmd_SEND_OP_COND},
        [42] = {8,  sd_ac,   "SET_CLR_CARD_DETECT", sd_acmd_SET_CLR_CARD_DETECT},
        [51] = {8,  sd_adtc, "SEND_SCR", sd_acmd_SEND_SCR},
    },
};

static const SDProto sd_proto_emmc = {
    /* Only v4.3 is supported */
    .name = "eMMC",
    .cmd = {
        [0]  = {0,  sd_bc,   "GO_IDLE_STATE", sd_cmd_GO_IDLE_STATE},
        [1]  = {0,  sd_bcr,  "SEND_OP_COND", sd_cmd_SEND_OP_COND},
        [2]  = {0,  sd_bcr,  "ALL_SEND_CID", sd_cmd_ALL_SEND_CID},
        [3]  = {0,  sd_ac,   "SET_RELATIVE_ADDR", emmc_cmd_SET_RELATIVE_ADDR},
        [4]  = {0,  sd_bc,   "SEND_DSR", sd_cmd_unimplemented},
        [5]  = {0,  sd_ac,   "SLEEP/AWAKE", emmc_cmd_sleep_awake},
        [6]  = {10, sd_adtc, "SWITCH", emmc_cmd_SWITCH},
        [7]  = {0,  sd_ac,   "(DE)SELECT_CARD", sd_cmd_DE_SELECT_CARD},
        [8]  = {0,  sd_adtc, "SEND_EXT_CSD", emmc_cmd_SEND_EXT_CSD},
        [9]  = {0,  sd_ac,   "SEND_CSD", sd_cmd_SEND_CSD},
        [10] = {0,  sd_ac,   "SEND_CID", sd_cmd_SEND_CID},
        [11] = {1,  sd_adtc, "READ_DAT_UNTIL_STOP", sd_cmd_unimplemented},
        [12] = {0,  sd_ac,   "STOP_TRANSMISSION", sd_cmd_STOP_TRANSMISSION},
        [13] = {0,  sd_ac,   "SEND_STATUS", sd_cmd_SEND_STATUS},
        [14] = {0,  sd_adtc, "BUSTEST_R", sd_cmd_unimplemented},
        [15] = {0,  sd_ac,   "GO_INACTIVE_STATE", sd_cmd_GO_INACTIVE_STATE},
        [16] = {2,  sd_ac,   "SET_BLOCKLEN", sd_cmd_SET_BLOCKLEN},
        [17] = {2,  sd_adtc, "READ_SINGLE_BLOCK", sd_cmd_READ_SINGLE_BLOCK},
        [19] = {0,  sd_adtc, "BUSTEST_W", sd_cmd_unimplemented},
        [20] = {3,  sd_adtc, "WRITE_DAT_UNTIL_STOP", sd_cmd_unimplemented},
        [23] = {2,  sd_ac,   "SET_BLOCK_COUNT", sd_cmd_SET_BLOCK_COUNT},
        [24] = {4,  sd_adtc, "WRITE_SINGLE_BLOCK", sd_cmd_WRITE_SINGLE_BLOCK},
        [26] = {4,  sd_adtc, "PROGRAM_CID", emmc_cmd_PROGRAM_CID},
        [27] = {4,  sd_adtc, "PROGRAM_CSD", sd_cmd_PROGRAM_CSD},
        [28] = {6,  sd_ac,   "SET_WRITE_PROT", sd_cmd_SET_WRITE_PROT},
        [29] = {6,  sd_ac,   "CLR_WRITE_PROT", sd_cmd_CLR_WRITE_PROT},
        [30] = {6,  sd_adtc, "SEND_WRITE_PROT", sd_cmd_SEND_WRITE_PROT},
        [31] = {6,  sd_adtc, "SEND_WRITE_PROT_TYPE", sd_cmd_unimplemented},
        [35] = {5,  sd_ac,   "ERASE_WR_BLK_START", sd_cmd_ERASE_WR_BLK_START},
        [36] = {5,  sd_ac,   "ERASE_WR_BLK_END", sd_cmd_ERASE_WR_BLK_END},
        [38] = {5,  sd_ac,   "ERASE", sd_cmd_ERASE},
        [39] = {9,  sd_ac,   "FAST_IO", sd_cmd_unimplemented},
        [40] = {9,  sd_bcr,  "GO_IRQ_STATE", sd_cmd_unimplemented},
        [42] = {7,  sd_adtc, "LOCK_UNLOCK", sd_cmd_LOCK_UNLOCK},
        [49] = {0,  sd_adtc, "SET_TIME", sd_cmd_unimplemented},
        [55] = {8,  sd_ac,   "APP_CMD", sd_cmd_APP_CMD},
        [56] = {8,  sd_adtc, "GEN_CMD", sd_cmd_GEN_CMD},
    },
};

static void sd_instance_init(Object *obj)
{
    SDState *sd = SDMMC_COMMON(obj);
    SDCardClass *sc = SDMMC_COMMON_GET_CLASS(sd);

    sd->proto = sc->proto;
    sd->last_cmd_name = "UNSET";
    sd->ocr_power_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, sd_ocr_powerup, sd);
    sd->vsw_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, sd_vsw_timer, sd);
    sd->signal_mv = SD_VOLTAGE_3_3V;
}

static void sd_instance_finalize(Object *obj)
{
    SDState *sd = SDMMC_COMMON(obj);

    timer_free(sd->ocr_power_timer);
    timer_free(sd->vsw_timer);
}

static void sd_blk_size_error(SDState *sd, int64_t blk_size,
                              int64_t blk_size_aligned, const char *rule,
                              Error **errp)
{
    const char *dev_type = sd_is_emmc(sd) ? "eMMC" : "SD card";
    char *blk_size_str;

    blk_size_str = size_to_str(blk_size);
    error_setg(errp, "Invalid %s size: %s", dev_type, blk_size_str);
    g_free(blk_size_str);

    blk_size_str = size_to_str(blk_size_aligned);
    error_append_hint(errp,
                      "%s size has to be %s, e.g. %s.\n"
                      "You can resize disk images with"
                      " 'qemu-img resize <imagefile> <new-size>'\n"
                      "(note that this will lose data if you make the"
                      " image smaller than it currently is).\n",
                      dev_type, rule, blk_size_str);
    g_free(blk_size_str);
}

static void sd_realize(DeviceState *dev, Error **errp)
{
    SDState *sd = SDMMC_COMMON(dev);
    int ret;

    switch (sd->spec_version) {
    case SD_PHY_SPECv2_00_VERS
     ... SD_PHY_SPECv3_01_VERS:
        break;
    default:
        error_setg(errp, "Invalid SD card Spec version: %u", sd->spec_version);
        return;
    }
    if (sd->uhs_mode != UHS_NOT_SUPPORTED && sd->uhs_mode != UHS_I) {
        error_setg(errp, "uhs must be 0 (none) or 1 (UHS-I)");
        return;
    }

    if (sd->blk) {
        if (!blk_supports_write_perm(sd->blk)) {
            error_setg(errp, "Cannot use read-only drive as SD card");
            return;
        }

        int64_t blk_size = blk_getlength(sd->blk);
        if (blk_size >= 0) {
            blk_size -= sd->boot_part_size * 2 + sd->rpmb_part_size;
            if (blk_size > SDSC_MAX_CAPACITY) {
                if (sd_is_emmc(sd) &&
                    !QEMU_IS_ALIGNED(blk_size, 1 << HWBLOCK_SHIFT)) {
                    int64_t blk_size_aligned =
                        ((blk_size >> HWBLOCK_SHIFT) + 1) << HWBLOCK_SHIFT;
                    sd_blk_size_error(sd, blk_size, blk_size_aligned,
                                      "multiples of 512", errp);
                    return;
                } else if (!sd_is_emmc(sd) &&
                    !QEMU_IS_ALIGNED(blk_size, 512 * KiB)) {
                    int64_t blk_size_aligned = ((blk_size >> 19) + 1) << 19;
                    sd_blk_size_error(sd, blk_size, blk_size_aligned,
                                      "multiples of 512K", errp);
                    return;
                }
            } else if (blk_size > 0 && !is_power_of_2(blk_size)) {
                sd_blk_size_error(sd, blk_size, pow2ceil(blk_size),
                                  "a power of 2", errp);
                return;
            } else if (blk_size < 0) {
                error_setg(errp, "eMMC image smaller than boot partitions");
                return;
            }
        }

        ret = blk_set_perm(sd->blk, BLK_PERM_CONSISTENT_READ | BLK_PERM_WRITE,
                           BLK_PERM_ALL, errp);
        if (ret < 0) {
            return;
        }
        blk_set_dev_ops(sd->blk, &sd_block_ops, sd);
    }
    if (!QEMU_IS_ALIGNED(sd->boot_part_size, 128 * KiB) ||
        sd->boot_part_size > 255 * 128 * KiB) {
        g_autofree char *size_str = size_to_str(sd->boot_part_size);

        error_setg(errp, "Invalid boot partition size: %s", size_str);
        error_append_hint(errp,
                          "The boot partition size must be multiples of 128K"
                          "and not larger than 32640K.\n");
    }
    if (!QEMU_IS_ALIGNED(sd->rpmb_part_size, 128 * KiB) ||
        sd->rpmb_part_size > 128 * 128 * KiB) {
        char *size_str = size_to_str(sd->boot_part_size);

        error_setg(errp, "Invalid RPMB partition size: %s", size_str);
        g_free(size_str);
        error_append_hint(errp,
                          "The RPMB partition size must be multiples of 128K"
                          "and not larger than 16384K.\n");
    }
    if (sd_is_emmc(sd) && sd->preset_auth_key) {
        if (strlen(sd->preset_auth_key) != 64) {
            error_setg(errp,
                       "Authentication key must be 32 bytes long, "
                       "encoded hexadecimally");
            return;
        }

        char *pos = sd->preset_auth_key;
        unsigned int n;
        for (n = 0; n < RPMB_KEY_MAC_LEN; n++, pos += 2) {
            int chrs;
            if (sscanf(pos, "%02hhx%n", &sd->rpmb.key[n], &chrs) != 1 ||
                chrs != 2) {
                error_setg(errp,
                           "Authentication key contains invalid characters");
                return;
            }
        }
        sd->rpmb.key_set = 1;
    }
}

static void emmc_realize(DeviceState *dev, Error **errp)
{
    SDState *sd = SDMMC_COMMON(dev);

    sd->spec_version = SD_PHY_SPECv3_01_VERS; /* Actually v4.5 */

    sd_realize(dev, errp);
}

static const Property sdmmc_common_properties[] = {
    DEFINE_PROP_DRIVE("drive", SDState, blk),
};

static const Property sd_properties[] = {
    DEFINE_PROP_UINT8("spec_version", SDState,
                      spec_version, SD_PHY_SPECv3_01_VERS),
    /* [spec:nuos:req:emu.esp32p4.sd-uhs] */
    DEFINE_PROP_UINT8("uhs", SDState, uhs_mode, UHS_NOT_SUPPORTED),
};

static const Property emmc_properties[] = {
    DEFINE_PROP_UINT64("boot-partition-size", SDState, boot_part_size, 0),
    DEFINE_PROP_UINT8("boot-config", SDState, boot_config, 0x0),
    DEFINE_PROP_UINT64("rpmb-partition-size", SDState, rpmb_part_size, 0),
    DEFINE_PROP_STRING("auth-key", SDState, preset_auth_key),
};

static void sdmmc_common_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SDCardClass *sc = SDMMC_COMMON_CLASS(klass);

    device_class_set_props(dc, sdmmc_common_properties);
    dc->vmsd = &sd_vmstate;
    device_class_set_legacy_reset(dc, sd_reset);
    dc->bus_type = TYPE_SD_BUS;
    set_bit(DEVICE_CATEGORY_STORAGE, dc->categories);

    sc->set_voltage = sd_set_voltage;
    sc->get_dat_lines = sd_get_dat_lines;
    sc->get_cmd_line = sd_get_cmd_line;
    sc->get_signal_voltage = sd_get_signal_voltage;
    sc->set_clock = sd_set_clock;
    sc->get_output_timing = sd_get_output_timing;
    sc->do_command = sd_do_command;
    sc->write_data = sd_write_data;
    sc->read_data = sd_read_data;
    sc->receive_ready = sd_receive_ready;
    sc->data_ready = sd_data_ready;
    sc->get_inserted = sd_get_inserted;
    sc->get_readonly = sd_get_readonly;
}

static void sd_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SDCardClass *sc = SDMMC_COMMON_CLASS(klass);

    dc->realize = sd_realize;
    device_class_set_props(dc, sd_properties);

    sc->set_cid = sd_set_cid;
    sc->set_csd = sd_set_csd;
    sc->proto = &sd_proto_sd;
}

/*
 * We do not model the chip select pin, so allow the board to select
 * whether card should be in SSI or MMC/SD mode.  It is also up to the
 * board to ensure that ssi transfers only occur when the chip select
 * is asserted.
 */
static void sd_spi_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SDCardClass *sc = SDMMC_COMMON_CLASS(klass);

    dc->desc = "SD SPI";
    sc->proto = &sd_proto_spi;
}

static void emmc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SDCardClass *sc = SDMMC_COMMON_CLASS(klass);

    assert(qcrypto_hmac_supports(QCRYPTO_HASH_ALGO_SHA256));

    dc->desc = "eMMC";
    dc->realize = emmc_realize;
    device_class_set_props(dc, emmc_properties);

    sc->proto = &sd_proto_emmc;

    sc->set_cid = emmc_set_cid;
    sc->set_csd = emmc_set_csd;
}

static const TypeInfo sd_types[] = {
    {
        .name           = TYPE_SDMMC_COMMON,
        .parent         = TYPE_DEVICE,
        .abstract       = true,
        .instance_size  = sizeof(SDState),
        .class_size     = sizeof(SDCardClass),
        .class_init     = sdmmc_common_class_init,
        .instance_init  = sd_instance_init,
        .instance_finalize = sd_instance_finalize,
    },
    {
        .name           = TYPE_SD_CARD,
        .parent         = TYPE_SDMMC_COMMON,
        .class_init     = sd_class_init,
    },
    {
        .name           = TYPE_SD_CARD_SPI,
        .parent         = TYPE_SD_CARD,
        .class_init     = sd_spi_class_init,
    },
    {
        .name           = TYPE_EMMC,
        .parent         = TYPE_SDMMC_COMMON,
        .class_init     = emmc_class_init,
    },
};

DEFINE_TYPES(sd_types)
