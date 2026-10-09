/*
 * RP2350 Cortex-M33 processor debug and trace components
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: RP2350 Datasheet, "Cortex-M33 processor" (configuration,
 * PPB map, DWT, CTI), pico-sdk hardware/regs/m33.h, the Armv8-M
 * Architecture Reference Manual (ITM, DWT, FPB, debug ROM tables) and
 * the Arm ETMv4 and CoreSight CTI architecture.
 *
 * Each RP2350 core is a Cortex-M33 r1p0 with the full debug set: ITM,
 * a four-comparator DWT, an eight-comparator FPB (FPBv2), an ETM and a
 * CTI, and no MTB. The processor-internal part of the PPB
 * (0xe0000000-0xe0043fff) and the PPB ROM table at 0xe00ff000 that a
 * debugger finds from the AHB-AP's BASE register are core-local; this
 * device is one core's set:
 *
 *   0xe0000000  ITM          0xe0041000  ETM
 *   0xe0001000  DWT          0xe0042000  CTI
 *   0xe0002000  FPB          0xe00ff000  PPB ROM table
 *   0xe000efbc  SCS identification block (and its NS alias at 0xe002efbc)
 *
 * The ROM table lists the SCS, DWT, FPB (the "BPU" entry), ITM, TPIU,
 * ETM, CTI and MTB slots in the Cortex-M33's fixed order; the TPIU and
 * MTB entries are marked not present, as the RP2350's TPIU is the SoC
 * one in the CoreSight window and the processor has no MTB.
 *
 * Identification registers and reset values follow m33.h. Where an m33.h
 * reset value contradicts the configuration the datasheet documents, the
 * configuration wins: DWT_CTRL reports four comparators and the cycle
 * and profiling counters the DWT has (m33.h: NUMCOMP 7, NOCYCCNT and
 * NOPRFCNT set), FP_CTRL reports FPB revision 2 with eight instruction
 * comparators and no literal comparators (m33.h: REV 6, NUM_CODE 0x58,
 * NUM_LIT 5), FP_COMPn takes the comparator address (m33.h lists only
 * BE), and DWT_FUNCTIONn.MATCH resets to 0 so that no comparator is
 * active out of reset (m33.h gives FUNCTION1 MATCH 8).
 *
 * These are state, not behaviour: no ATB trace is produced (ITM stimulus
 * writes, DWT and ETM packets go nowhere), DWT comparators and FPB
 * breakpoints do not match, the DWT profiling counters and PC sampler do
 * not count, and CTI trigger outputs (halt, restart, interrupts, ETM
 * events) have no effect. Each of these logs LOG_UNIMP when software
 * turns it on. DWT_CYCCNT counts the core clock while CYCCNTENA is set.
 * DEMCR is not modelled by the SCS, so TRCENA does not gate the DWT.
 *
 * The PPB is privileged: an unprivileged access is a BusFault, except to
 * ITM stimulus ports whose ITM_TPR group is unmasked. These components
 * sit in the debug domain and are reset only with the chip, not by a
 * core's warm reset.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/core/qdev-clock.h"
#include "hw/misc/rp2350_m33_debug.h"
#include "migration/vmstate.h"

#define CMP_SIZE        0x1000

/* Registers at the top of every component. */
#define A_DEVARCH       0xfbc
#define A_DEVID         0xfc8
#define A_DEVTYPE       0xfcc
#define A_PIDR4         0xfd0
#define A_PIDR5         0xfd4
#define A_PIDR7         0xfdc
#define A_PIDR0         0xfe0
#define A_PIDR3         0xfec
#define A_CIDR0         0xff0
#define A_CIDR3         0xffc

/* ITM */
#define A_ITM_STIM0     0x000
#define A_ITM_STIM31    0x07c
#define A_ITM_TER0      0xe00
#define A_ITM_TPR       0xe40
#define A_ITM_TCR       0xe80
#define ITM_TCR_ITMENA  (1u << 0)
#define ITM_STIM_FIFOREADY (1u << 0)
#define ITM_STIM_DISABLED  (1u << 1)

/* DWT */
#define A_DWT_CTRL      0x000
#define A_DWT_CYCCNT    0x004
#define A_DWT_PCSR      0x01c
#define A_DWT_COMP0     0x020
#define A_DWT_FUNCTION0 0x028
#define DWT_CTRL_CYCCNTENA (1u << 0)
#define DWT_CTRL_PCSAMPLENA (1u << 12)
#define DWT_CTRL_EXTTRCENA (1u << 16)
#define DWT_CTRL_CNTEVTENA 0x003e0000u  /* CPI, EXC, SLEEP, LSU, FOLD */
#define DWT_CTRL_CYCEVTENA (1u << 22)
#define DWT_FUNCTION_MATCHED (1u << 24)
#define DWT_FUNCTION_MATCH 0xfu
/*
 * NUMCOMP 4, every feature present; the remaining fields' reset values
 * are UNKNOWN architecturally and follow m33.h.
 */
#define DWT_CTRL_RESET  0x40741824u

/* FPB */
#define A_FP_CTRL       0x000
#define A_FP_COMP0      0x008
#define FP_CTRL_ENABLE  (1u << 0)
#define FP_CTRL_KEY     (1u << 1)
/* REV 1 (FPBv2), NUM_CODE 8, NUM_LIT 0. */
#define FP_CTRL_RESET   0x10000080u

/* ETM, at 0xe0041000 */
#define A_TRCPRGCTLR    0x004
#define A_TRCSTATR      0x00c
#define A_TRCPDSR       0x314
#define A_TRCCLAIMSET   0xfa0
#define A_TRCCLAIMCLR   0xfa4
#define TRCPRGCTLR_EN   (1u << 0)
#define TRCSTATR_IDLE   (1u << 0)
#define TRCSTATR_PMSTABLE (1u << 1)
#define TRCPDSR_STICKYPD (1u << 1)
#define ETM_CLAIM_BITS  0xfu

/* CTI, at 0xe0042000 */
#define A_CTICONTROL    0x000
#define A_CTIINTACK     0x010
#define A_CTIAPPSET     0x014
#define A_CTIAPPCLEAR   0x018
#define A_CTIAPPPULSE   0x01c
#define A_CTIOUTEN0     0x0a0
#define A_CTITRIGOUTSTATUS 0x134
#define A_ITCHOUT       0xee4
#define A_ITTRIGOUT     0xee8
#define A_CTI_ITCTRL    0xf00
#define CTICONTROL_GLBEN (1u << 0)
#define CTI_NUM_TRIG    8
#define CTI_CHANNELS    0xfu

/*
 * The SCS identification block, overlaid on the SCS from DDEVARCH to the
 * last CIDR. Its register offsets are those of a component's last 4 KiB.
 */
#define SCS_ID_BIAS     A_DEVARCH
#define SCS_ID_SIZE     (CMP_SIZE - A_DEVARCH)

/* PPB ROM table entries: offset from 0xe00ff000, present when bit 0 set. */
static const uint32_t rom_entries[] = {
    0xfff0f003, /* SCS  0xe000e000 */
    0xfff02003, /* DWT  0xe0001000 */
    0xfff03003, /* FPB  0xe0002000 */
    0xfff01003, /* ITM  0xe0000000 */
    0xfff41002, /* TPIU 0xe0040000, not present */
    0xfff42003, /* ETM  0xe0041000 */
    0xfff43003, /* CTI  0xe0042000 */
    0xfff44002, /* MTB  0xe0043000, not present */
    0x00000000, /* end of table */
};

const hwaddr rp2350_m33_debug_base[RP2350_M33_DEBUG_NUM_REGIONS] = {
    [RP2350_M33_DEBUG_ITM] = 0xe0000000,
    [RP2350_M33_DEBUG_DWT] = 0xe0001000,
    [RP2350_M33_DEBUG_FPB] = 0xe0002000,
    [RP2350_M33_DEBUG_SCS_ID] = 0xe000e000 + SCS_ID_BIAS,
    [RP2350_M33_DEBUG_SCS_ID_NS] = 0xe002e000 + SCS_ID_BIAS,
    [RP2350_M33_DEBUG_ETM] = 0xe0041000,
    [RP2350_M33_DEBUG_CTI] = 0xe0042000,
    [RP2350_M33_DEBUG_ROM] = 0xe00ff000,
};

typedef struct M33Reg {
    uint16_t offset;
    uint32_t reset;
    /* Bits a write changes; RO registers have none. */
    uint32_t wmask;
} M33Reg;

typedef struct RP2350M33DebugInfo {
    const char *name;
    hwaddr size;
    /* Added to a region offset to give the register offset. */
    hwaddr bias;
    const M33Reg *regs;
    unsigned num_regs;
    uint32_t devarch;
    uint32_t devid;
    /* DEVTYPE, or MEMTYPE for a ROM table. */
    uint32_t devtype;
    uint8_t pidr[5];
    uint8_t cidr1;
} RP2350M33DebugInfo;

/* Peripheral IDs every Cortex-M33 r1p0 component except the ETM reports. */
#define M33_PIDR(rev) { 0x21, 0xbd, 0x0b | (rev) << 4, 0x00, 0x04 }
/* CoreSight component class 0x9 */
#define CIDR1_CORESIGHT 0x90
/* Class 0x1 ROM table */
#define CIDR1_ROM       0x10

static const M33Reg itm_regs[] = {
    { A_ITM_TER0, 0, 0xffffffff },
    { A_ITM_TPR, 0, 0x0000000f },
    /* BUSY reads 0: there is never trace in flight. */
    { A_ITM_TCR, 0, 0x007f0f3f },
    { 0xef0, 0, 0 },            /* INT_ATREADY */
    { 0xef8, 0, 0x00000003 },   /* INT_ATVALID */
    { 0xf00, 0, 0x00000001 },   /* ITM_ITCTRL */
};

static const M33Reg dwt_regs[] = {
    { A_DWT_CTRL, DWT_CTRL_RESET, 0x00ff1fff },
    { 0x008, 0, 0xff },         /* DWT_CPICNT */
    { 0x00c, 0, 0xff },         /* DWT_EXCCNT */
    { 0x010, 0, 0xff },         /* DWT_SLEEPCNT */
    { 0x014, 0, 0xff },         /* DWT_LSUCNT */
    { 0x018, 0, 0xff },         /* DWT_FOLDCNT */
    { A_DWT_COMP0, 0, 0xffffffff },
    { A_DWT_FUNCTION0, 0x58000000, 0x00000c3f },
    { A_DWT_COMP0 + 0x10, 0, 0xffffffff },
    { A_DWT_FUNCTION0 + 0x10, 0x89000820, 0x00000c3f },
    { A_DWT_COMP0 + 0x20, 0, 0xffffffff },
    { A_DWT_FUNCTION0 + 0x20, 0x50000000, 0x00000c3f },
    { A_DWT_COMP0 + 0x30, 0, 0xffffffff },
    { A_DWT_FUNCTION0 + 0x30, 0x20000800, 0x00000c3f },
};

static const M33Reg fpb_regs[] = {
    { A_FP_CTRL, FP_CTRL_RESET, FP_CTRL_ENABLE },
    { 0x004, 0, 0 },            /* FP_REMAP: no remap support */
    { A_FP_COMP0 + 0x00, 0, 0xffffffff },
    { A_FP_COMP0 + 0x04, 0, 0xffffffff },
    { A_FP_COMP0 + 0x08, 0, 0xffffffff },
    { A_FP_COMP0 + 0x0c, 0, 0xffffffff },
    { A_FP_COMP0 + 0x10, 0, 0xffffffff },
    { A_FP_COMP0 + 0x14, 0, 0xffffffff },
    { A_FP_COMP0 + 0x18, 0, 0xffffffff },
    { A_FP_COMP0 + 0x1c, 0, 0xffffffff },
};

static const M33Reg etm_regs[] = {
    { A_TRCPRGCTLR, 0, 0x00000001 },
    { 0x010, 0, 0x00001ff8 },   /* TRCCONFIGR */
    { 0x020, 0, 0x00008787 },   /* TRCEVENTCTL0R */
    { 0x024, 0, 0x00001803 },   /* TRCEVENTCTL1R */
    { 0x02c, 0, 0x0000010c },   /* TRCSTALLCTLR */
    { 0x030, 0, 0x00000083 },   /* TRCTSCTLR */
    { 0x034, 0x0000000a, 0 },   /* TRCSYNCPR */
    { 0x038, 0, 0x00000fff },   /* TRCCCCTLR */
    { 0x080, 0, 0x00090e83 },   /* TRCVICTLR */
    { 0x140, 0, 0x0000ffff },   /* TRCCNTRLDVR0 */
    { 0x180, 0x00000000, 0 },   /* TRCIDR8 */
    { 0x184, 0x00000000, 0 },   /* TRCIDR9 */
    { 0x188, 0x00000000, 0 },   /* TRCIDR10 */
    { 0x18c, 0x00000000, 0 },   /* TRCIDR11 */
    { 0x190, 0x00000001, 0 },   /* TRCIDR12 */
    { 0x194, 0x00000000, 0 },   /* TRCIDR13 */
    { 0x1c0, 0x00000000, 0 },   /* TRCIMSPEC */
    { 0x1e0, 0x280006e1, 0 },   /* TRCIDR0 */
    { 0x1e4, 0x4100f421, 0 },   /* TRCIDR1 */
    { 0x1e8, 0x00000004, 0 },   /* TRCIDR2 */
    { 0x1ec, 0x0f090004, 0 },   /* TRCIDR3 */
    { 0x1f0, 0x00114000, 0 },   /* TRCIDR4 */
    { 0x1f4, 0x90c70004, 0 },   /* TRCIDR5 */
    { 0x1f8, 0x00000000, 0 },   /* TRCIDR6 */
    { 0x1fc, 0x00000000, 0 },   /* TRCIDR7 */
    { 0x208, 0, 0x003700ff },   /* TRCRSCTLR2 */
    { 0x20c, 0, 0x003700ff },   /* TRCRSCTLR3 */
    { 0x2a0, 0, 0x80000000 },   /* TRCSSCSR */
    { 0x2c0, 0, 0x0000000f },   /* TRCSSPCICR */
    { 0x310, 0, 0x00000008 },   /* TRCPDCR */
    /* POWER and STICKYPD: powered, and powered up since last read. */
    { A_TRCPDSR, 0x00000003, 0 },
    { 0xee4, 0, 0x0000007f },   /* TRCITATBIDR */
    { 0xef4, 0, 0x00000003 },   /* TRCITIATBINR */
    { 0xefc, 0, 0x00000003 },   /* TRCITIATBOUTR */
    /* The claim tags; TRCCLAIMSET reads which are implemented. */
    { A_TRCCLAIMCLR, 0, ETM_CLAIM_BITS },
    { 0xfb8, 0, 0 },            /* TRCAUTHSTATUS */
};

static const M33Reg cti_regs[] = {
    { A_CTICONTROL, 0, CTICONTROL_GLBEN },
    /* The application trigger channels CTIAPPSET reads. */
    { A_CTIAPPSET, 0, CTI_CHANNELS },
    { 0x020, 0, CTI_CHANNELS }, /* CTIINEN0..7 */
    { 0x024, 0, CTI_CHANNELS },
    { 0x028, 0, CTI_CHANNELS },
    { 0x02c, 0, CTI_CHANNELS },
    { 0x030, 0, CTI_CHANNELS },
    { 0x034, 0, CTI_CHANNELS },
    { 0x038, 0, CTI_CHANNELS },
    { 0x03c, 0, CTI_CHANNELS },
    { A_CTIOUTEN0 + 0x00, 0, CTI_CHANNELS },
    { A_CTIOUTEN0 + 0x04, 0, CTI_CHANNELS },
    { A_CTIOUTEN0 + 0x08, 0, CTI_CHANNELS },
    { A_CTIOUTEN0 + 0x0c, 0, CTI_CHANNELS },
    { A_CTIOUTEN0 + 0x10, 0, CTI_CHANNELS },
    { A_CTIOUTEN0 + 0x14, 0, CTI_CHANNELS },
    { A_CTIOUTEN0 + 0x18, 0, CTI_CHANNELS },
    { A_CTIOUTEN0 + 0x1c, 0, CTI_CHANNELS },
    { 0x130, 0, 0 },            /* CTITRIGINSTATUS */
    { 0x138, 0, 0 },            /* CTICHINSTATUS */
    { 0x140, CTI_CHANNELS, CTI_CHANNELS }, /* CTIGATE */
    { 0x144, 0, 0 },            /* ASICCTL */
    { 0xef4, 0, 0 },            /* ITCHIN */
    { A_CTI_ITCTRL, 0, 0x00000001 },
};

static const RP2350M33DebugInfo m33_debug_info[RP2350_M33_DEBUG_NUM_REGIONS] = {
    [RP2350_M33_DEBUG_ITM] = {
        .name = "rp2350-m33-itm", .size = CMP_SIZE,
        .regs = itm_regs, .num_regs = ARRAY_SIZE(itm_regs),
        .devarch = 0x47701a01, .devtype = 0x43,
        .pidr = M33_PIDR(0), .cidr1 = CIDR1_CORESIGHT,
    },
    [RP2350_M33_DEBUG_DWT] = {
        .name = "rp2350-m33-dwt", .size = CMP_SIZE,
        .regs = dwt_regs, .num_regs = ARRAY_SIZE(dwt_regs),
        .devarch = 0x47701a02, .devtype = 0x00,
        .pidr = M33_PIDR(0), .cidr1 = CIDR1_CORESIGHT,
    },
    [RP2350_M33_DEBUG_FPB] = {
        .name = "rp2350-m33-fpb", .size = CMP_SIZE,
        .regs = fpb_regs, .num_regs = ARRAY_SIZE(fpb_regs),
        .devarch = 0x47701a03, .devtype = 0x00,
        .pidr = M33_PIDR(0), .cidr1 = CIDR1_CORESIGHT,
    },
    [RP2350_M33_DEBUG_SCS_ID] = {
        .name = "rp2350-m33-scs-id", .size = SCS_ID_SIZE,
        .bias = SCS_ID_BIAS,
        .devarch = 0x47702a04, .devtype = 0x00,
        .pidr = M33_PIDR(0), .cidr1 = CIDR1_CORESIGHT,
    },
    [RP2350_M33_DEBUG_SCS_ID_NS] = {
        .name = "rp2350-m33-scs-id-ns", .size = SCS_ID_SIZE,
        .bias = SCS_ID_BIAS,
        .devarch = 0x47702a04, .devtype = 0x00,
        .pidr = M33_PIDR(0), .cidr1 = CIDR1_CORESIGHT,
    },
    [RP2350_M33_DEBUG_ETM] = {
        .name = "rp2350-m33-etm", .size = CMP_SIZE,
        .regs = etm_regs, .num_regs = ARRAY_SIZE(etm_regs),
        .devarch = 0x47724a13, .devtype = 0x13,
        .pidr = M33_PIDR(2), .cidr1 = CIDR1_CORESIGHT,
    },
    [RP2350_M33_DEBUG_CTI] = {
        .name = "rp2350-m33-cti", .size = CMP_SIZE,
        .regs = cti_regs, .num_regs = ARRAY_SIZE(cti_regs),
        /* Four channels, eight triggers. */
        .devarch = 0x47701a14, .devid = 0x00040800, .devtype = 0x14,
        .pidr = M33_PIDR(0), .cidr1 = CIDR1_CORESIGHT,
    },
    [RP2350_M33_DEBUG_ROM] = {
        .name = "rp2350-m33-rom-table", .size = CMP_SIZE,
        /* MEMTYPE.SYSMEM: system memory is also on this bus. */
        .devtype = 0x01,
        /* Arm, part 0x4c9: the Cortex-M33 PPB ROM table. */
        .pidr = { 0xc9, 0xb4, 0x0b, 0x00, 0x04 }, .cidr1 = CIDR1_ROM,
    },
};

static uint32_t *comp_reg(RP2350M33DebugComponent *c, hwaddr offset)
{
    unsigned i;

    for (i = 0; i < c->info->num_regs; i++) {
        if (c->info->regs[i].offset == offset) {
            return &c->regs[i];
        }
    }
    return NULL;
}

static uint32_t *s_reg(RP2350M33DebugState *s, RP2350M33DebugRegion r,
                       hwaddr offset)
{
    uint32_t *reg = comp_reg(&s->comp[r], offset);

    assert(reg);
    return reg;
}

/* The identification block: DEVARCH, DEVID, DEVTYPE, PIDRn and CIDRn. */
static bool comp_id_read(const RP2350M33DebugInfo *info, hwaddr offset,
                         uint32_t *v)
{
    static const uint8_t cidr[4] = { 0x0d, 0x00, 0x05, 0xb1 };

    switch (offset) {
    case A_DEVARCH:
        *v = info->devarch;
        return true;
    case A_DEVID:
        *v = info->devid;
        return true;
    case A_DEVTYPE:
        *v = info->devtype;
        return true;
    case A_PIDR4:
        *v = info->pidr[4];
        return true;
    case A_PIDR5 ... A_PIDR7:
        *v = 0;
        return true;
    case A_PIDR0 ... A_PIDR3:
        *v = info->pidr[(offset - A_PIDR0) / 4];
        return true;
    case A_CIDR0 ... A_CIDR3:
        *v = offset == A_CIDR0 + 4 ? info->cidr1 : cidr[(offset - A_CIDR0) / 4];
        return true;
    default:
        return false;
    }
}

/* DWT */

static int64_t now_ns(void)
{
    return qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

static uint32_t dwt_cyccnt(RP2350M33DebugState *s)
{
    uint32_t ctrl = *s_reg(s, RP2350_M33_DEBUG_DWT, A_DWT_CTRL);

    if (!(ctrl & DWT_CTRL_CYCCNTENA)) {
        return s->dwt_cyccnt;
    }
    return s->dwt_cyccnt +
           (uint32_t)clock_ns_to_ticks(s->cpuclk, now_ns() - s->dwt_cyccnt_ns);
}

/* Fold the count so far into dwt_cyccnt before its rate changes. */
static void dwt_cyccnt_latch(RP2350M33DebugState *s)
{
    s->dwt_cyccnt = dwt_cyccnt(s);
    s->dwt_cyccnt_ns = now_ns();
}

static void rp2350_m33_debug_cpuclk_update(void *opaque, ClockEvent event)
{
    dwt_cyccnt_latch(RP2350_M33_DEBUG(opaque));
}

static bool dwt_read(RP2350M33DebugState *s, hwaddr offset, uint32_t *v)
{
    uint32_t *reg;

    switch (offset) {
    case A_DWT_CYCCNT:
        *v = dwt_cyccnt(s);
        return true;
    case A_DWT_PCSR:
        qemu_log_mask(LOG_UNIMP, "rp2350-m33-dwt: PC sampling is not "
                      "modelled; DWT_PCSR reads as not sampled\n");
        *v = 0xffffffff;
        return true;
    case A_DWT_FUNCTION0:
    case A_DWT_FUNCTION0 + 0x10:
    case A_DWT_FUNCTION0 + 0x20:
    case A_DWT_FUNCTION0 + 0x30:
        /* MATCHED clears when read. */
        reg = s_reg(s, RP2350_M33_DEBUG_DWT, offset);
        *v = *reg;
        *reg &= ~DWT_FUNCTION_MATCHED;
        return true;
    default:
        return false;
    }
}

static bool dwt_write(RP2350M33DebugState *s, hwaddr offset, uint32_t v)
{
    uint32_t *ctrl = s_reg(s, RP2350_M33_DEBUG_DWT, A_DWT_CTRL);
    uint32_t set;

    switch (offset) {
    case A_DWT_CTRL:
        dwt_cyccnt_latch(s);
        set = v & ~*ctrl;
        *ctrl = (*ctrl & ~0x00ff1fffu) | (v & 0x00ff1fffu);
        if (set & DWT_CTRL_CNTEVTENA) {
            qemu_log_mask(LOG_UNIMP, "rp2350-m33-dwt: the CPI, exception, "
                          "sleep, LSU and folded-instruction counters do "
                          "not count\n");
        }
        if (set & (DWT_CTRL_CYCEVTENA | DWT_CTRL_PCSAMPLENA |
                   DWT_CTRL_EXTTRCENA)) {
            qemu_log_mask(LOG_UNIMP, "rp2350-m33-dwt: trace packets are "
                          "not generated\n");
        }
        return true;
    case A_DWT_CYCCNT:
        s->dwt_cyccnt = v;
        s->dwt_cyccnt_ns = now_ns();
        return true;
    case A_DWT_FUNCTION0:
    case A_DWT_FUNCTION0 + 0x10:
    case A_DWT_FUNCTION0 + 0x20:
    case A_DWT_FUNCTION0 + 0x30:
        if (v & DWT_FUNCTION_MATCH) {
            qemu_log_mask(LOG_UNIMP, "rp2350-m33-dwt: comparator %d never "
                          "matches\n", (int)(offset - A_DWT_FUNCTION0) / 0x10);
        }
        return false;
    default:
        return false;
    }
}

/* FPB */

static bool fpb_write(RP2350M33DebugState *s, hwaddr offset, uint32_t v)
{
    uint32_t *ctrl;

    if (offset != A_FP_CTRL) {
        return false;
    }
    /* A write without KEY set is ignored. */
    if (v & FP_CTRL_KEY) {
        ctrl = s_reg(s, RP2350_M33_DEBUG_FPB, A_FP_CTRL);
        *ctrl = (*ctrl & ~FP_CTRL_ENABLE) | (v & FP_CTRL_ENABLE);
        if (v & FP_CTRL_ENABLE) {
            qemu_log_mask(LOG_UNIMP, "rp2350-m33-fpb: breakpoints are not "
                          "taken\n");
        }
    }
    return true;
}

/* ETM */

static bool etm_read(RP2350M33DebugState *s, hwaddr offset, uint32_t *v)
{
    uint32_t *reg;

    switch (offset) {
    case A_TRCSTATR:
        /* Disabled, the trace unit is idle and its registers stable. */
        *v = (*s_reg(s, RP2350_M33_DEBUG_ETM, A_TRCPRGCTLR) & TRCPRGCTLR_EN)
             ? 0 : TRCSTATR_IDLE | TRCSTATR_PMSTABLE;
        return true;
    case A_TRCPDSR:
        reg = s_reg(s, RP2350_M33_DEBUG_ETM, A_TRCPDSR);
        *v = *reg;
        *reg &= ~TRCPDSR_STICKYPD;
        return true;
    case A_TRCCLAIMSET:
        *v = ETM_CLAIM_BITS;
        return true;
    default:
        return false;
    }
}

static bool etm_write(RP2350M33DebugState *s, hwaddr offset, uint32_t v)
{
    uint32_t *claim = s_reg(s, RP2350_M33_DEBUG_ETM, A_TRCCLAIMCLR);

    switch (offset) {
    case A_TRCPRGCTLR:
        if (v & TRCPRGCTLR_EN) {
            qemu_log_mask(LOG_UNIMP, "rp2350-m33-etm: no instruction trace "
                          "is generated\n");
        }
        return false;
    case A_TRCCLAIMSET:
        *claim |= v & ETM_CLAIM_BITS;
        return true;
    case A_TRCCLAIMCLR:
        *claim &= ~v;
        return true;
    default:
        return false;
    }
}

/* CTI */

/*
 * The trigger outputs the application channels drive. No trigger input
 * fires and no other CTI drives the channels, so these are the only
 * active channels.
 */
static uint32_t cti_trigouts(RP2350M33DebugState *s, uint32_t channels)
{
    uint32_t out = 0;
    int n;

    if (!(*s_reg(s, RP2350_M33_DEBUG_CTI, A_CTICONTROL) & CTICONTROL_GLBEN)) {
        return 0;
    }
    for (n = 0; n < CTI_NUM_TRIG; n++) {
        if (*s_reg(s, RP2350_M33_DEBUG_CTI, A_CTIOUTEN0 + 4 * n) & channels) {
            out |= 1u << n;
        }
    }
    return out;
}

static void cti_report(RP2350M33DebugState *s, uint32_t out)
{
    uint32_t new = out & ~s->cti_trigout_logged;

    if (new) {
        qemu_log_mask(LOG_UNIMP, "rp2350-m33-cti: trigger outputs 0x%02x "
                      "(halt, restart, interrupts, ETM events) have no "
                      "effect\n", new);
    }
    s->cti_trigout_logged = out;
}

static void cti_update(RP2350M33DebugState *s)
{
    cti_report(s, cti_trigouts(s, *s_reg(s, RP2350_M33_DEBUG_CTI,
                                          A_CTIAPPSET)));
}

static bool cti_read(RP2350M33DebugState *s, hwaddr offset, uint32_t *v)
{
    switch (offset) {
    case A_CTITRIGOUTSTATUS:
        *v = cti_trigouts(s, *s_reg(s, RP2350_M33_DEBUG_CTI, A_CTIAPPSET));
        return true;
    case A_CTIINTACK:
    case A_CTIAPPCLEAR:
    case A_CTIAPPPULSE:
    case A_ITCHOUT:
    case A_ITTRIGOUT:
        /* Write-only. */
        *v = 0;
        return true;
    default:
        return false;
    }
}

static bool cti_write(RP2350M33DebugState *s, hwaddr offset, uint32_t v)
{
    uint32_t *app = s_reg(s, RP2350_M33_DEBUG_CTI, A_CTIAPPSET);

    switch (offset) {
    case A_CTIAPPSET:
        *app |= v & CTI_CHANNELS;
        break;
    case A_CTIAPPCLEAR:
        *app &= ~v;
        break;
    case A_CTIAPPPULSE:
        cti_report(s, cti_trigouts(s, (*app | v) & CTI_CHANNELS));
        break;
    case A_CTIINTACK:
        /* Trigger outputs follow the channels; there is no latch. */
        return true;
    case A_ITCHOUT:
    case A_ITTRIGOUT:
        if (*s_reg(s, RP2350_M33_DEBUG_CTI, A_CTI_ITCTRL) & 1) {
            qemu_log_mask(LOG_UNIMP, "rp2350-m33-cti: integration outputs "
                          "drive nothing\n");
        }
        return true;
    default:
        return false;
    }
    cti_update(s);
    return true;
}

/* Register access */

static uint32_t comp_read(RP2350M33DebugComponent *c, hwaddr offset)
{
    RP2350M33DebugState *s = c->s;
    RP2350M33DebugRegion r = c - s->comp;
    uint32_t v = 0;
    uint32_t *reg;
    bool done = false;

    switch (r) {
    case RP2350_M33_DEBUG_DWT:
        done = dwt_read(s, offset, &v);
        break;
    case RP2350_M33_DEBUG_ETM:
        done = etm_read(s, offset, &v);
        break;
    case RP2350_M33_DEBUG_CTI:
        done = cti_read(s, offset, &v);
        break;
    case RP2350_M33_DEBUG_ROM:
        if (offset < sizeof(rom_entries)) {
            v = rom_entries[offset / 4];
            done = true;
        }
        break;
    default:
        break;
    }
    if (done) {
        return v;
    }
    reg = comp_reg(c, offset);
    if (reg) {
        return *reg;
    }
    if (comp_id_read(c->info, offset, &v)) {
        return v;
    }
    /* Reserved: RAZ. */
    return 0;
}

static void comp_write(RP2350M33DebugComponent *c, hwaddr offset, uint32_t v)
{
    RP2350M33DebugState *s = c->s;
    RP2350M33DebugRegion r = c - s->comp;
    const M33Reg *def;
    uint32_t *reg;
    bool done = false;

    switch (r) {
    case RP2350_M33_DEBUG_DWT:
        done = dwt_write(s, offset, v);
        break;
    case RP2350_M33_DEBUG_FPB:
        done = fpb_write(s, offset, v);
        break;
    case RP2350_M33_DEBUG_ETM:
        done = etm_write(s, offset, v);
        break;
    case RP2350_M33_DEBUG_CTI:
        done = cti_write(s, offset, v);
        break;
    default:
        break;
    }
    if (done) {
        return;
    }
    reg = comp_reg(c, offset);
    if (!reg) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only or reserved "
                      "offset 0x%03" HWADDR_PRIx "\n", c->info->name, offset);
        return;
    }
    def = &c->info->regs[reg - c->regs];
    if (!def->wmask) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only offset "
                      "0x%03" HWADDR_PRIx "\n", c->info->name, offset);
        return;
    }
    *reg = (*reg & ~def->wmask) | (v & def->wmask);
    if (r == RP2350_M33_DEBUG_CTI) {
        cti_update(s);
    }
}

static bool itm_stim_port(RP2350M33DebugComponent *c, hwaddr offset)
{
    return c - c->s->comp == RP2350_M33_DEBUG_ITM &&
           offset >= A_ITM_STIM0 && offset <= A_ITM_STIM31 + 3;
}

/*
 * An unprivileged stimulus port access is permitted unless ITM_TPR masks
 * the port's group of eight, in which case it is ignored.
 */
static bool itm_stim_masked(RP2350M33DebugState *s, unsigned port,
                            MemTxAttrs attrs)
{
    uint32_t tpr = *s_reg(s, RP2350_M33_DEBUG_ITM, A_ITM_TPR);

    return attrs.user && (tpr & (1u << (port / 8)));
}

static bool itm_stim_enabled(RP2350M33DebugState *s, unsigned port)
{
    return (*s_reg(s, RP2350_M33_DEBUG_ITM, A_ITM_TCR) & ITM_TCR_ITMENA) &&
           (*s_reg(s, RP2350_M33_DEBUG_ITM, A_ITM_TER0) & (1u << port));
}

/* [spec:nuos:req:emu.coresight] */
static MemTxResult rp2350_m33_debug_read(void *opaque, hwaddr addr,
                                         uint64_t *data, unsigned size,
                                         MemTxAttrs attrs)
{
    RP2350M33DebugComponent *c = opaque;
    RP2350M33DebugState *s = c->s;
    hwaddr offset = addr + c->info->bias;
    uint32_t v;

    if (itm_stim_port(c, offset)) {
        unsigned port = offset / 4;

        if (itm_stim_masked(s, port, attrs)) {
            *data = 0;
            return MEMTX_OK;
        }
        /* The FIFO always has room: nothing drains it more slowly. */
        v = itm_stim_enabled(s, port) ? ITM_STIM_FIFOREADY
                                      : ITM_STIM_DISABLED;
        *data = extract32(v, (offset & 3) * 8, size * 8);
        return MEMTX_OK;
    }
    if (attrs.user) {
        return MEMTX_ERROR;
    }
    /* The NS alias of the SCS is RAZ/WI to Non-secure accesses. */
    if (c - s->comp == RP2350_M33_DEBUG_SCS_ID_NS && !attrs.secure) {
        *data = 0;
        return MEMTX_OK;
    }
    v = comp_read(c, offset & ~3);
    *data = extract32(v, (offset & 3) * 8, size * 8);
    return MEMTX_OK;
}

/* [spec:nuos:req:emu.coresight] */
static MemTxResult rp2350_m33_debug_write(void *opaque, hwaddr addr,
                                          uint64_t value, unsigned size,
                                          MemTxAttrs attrs)
{
    RP2350M33DebugComponent *c = opaque;
    RP2350M33DebugState *s = c->s;
    hwaddr offset = addr + c->info->bias;

    if (itm_stim_port(c, offset)) {
        unsigned port = offset / 4;

        if (!itm_stim_masked(s, port, attrs) && itm_stim_enabled(s, port)) {
            qemu_log_mask(LOG_UNIMP, "rp2350-m33-itm: stimulus port %u "
                          "write produces no trace\n", port);
        }
        return MEMTX_OK;
    }
    if (attrs.user) {
        return MEMTX_ERROR;
    }
    if (c - s->comp == RP2350_M33_DEBUG_SCS_ID_NS && !attrs.secure) {
        return MEMTX_OK;
    }
    if (size != 4) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: %u-byte write to offset 0x%03"
                      HWADDR_PRIx " ignored: registers are word access only\n",
                      c->info->name, size, offset);
        return MEMTX_OK;
    }
    comp_write(c, offset, value);
    return MEMTX_OK;
}

static const MemoryRegionOps rp2350_m33_debug_ops = {
    .read_with_attrs = rp2350_m33_debug_read,
    .write_with_attrs = rp2350_m33_debug_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .valid.unaligned = false,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

static void rp2350_m33_debug_hold_reset(Object *obj, ResetType type)
{
    RP2350M33DebugState *s = RP2350_M33_DEBUG(obj);
    int r;
    unsigned i;

    for (r = 0; r < RP2350_M33_DEBUG_NUM_REGIONS; r++) {
        RP2350M33DebugComponent *c = &s->comp[r];

        memset(c->regs, 0, sizeof(c->regs));
        for (i = 0; i < c->info->num_regs; i++) {
            c->regs[i] = c->info->regs[i].reset;
        }
    }
    s->dwt_cyccnt = 0;
    s->dwt_cyccnt_ns = now_ns();
    s->cti_trigout_logged = 0;
}

static void rp2350_m33_debug_init(Object *obj)
{
    RP2350M33DebugState *s = RP2350_M33_DEBUG(obj);
    int r;

    for (r = 0; r < RP2350_M33_DEBUG_NUM_REGIONS; r++) {
        RP2350M33DebugComponent *c = &s->comp[r];

        c->s = s;
        c->info = &m33_debug_info[r];
        assert(c->info->num_regs <= RP2350_M33_DEBUG_MAX_REGS);
        memory_region_init_io(&c->mr, obj, &rp2350_m33_debug_ops, c,
                              c->info->name, c->info->size);
        sysbus_init_mmio(SYS_BUS_DEVICE(obj), &c->mr);
    }
    s->cpuclk = qdev_init_clock_in(DEVICE(obj), "cpuclk",
                                   rp2350_m33_debug_cpuclk_update, s,
                                   ClockPreUpdate);
}

static const VMStateDescription vmstate_rp2350_m33_debug_comp = {
    .name = TYPE_RP2350_M33_DEBUG "-component",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, RP2350M33DebugComponent,
                             RP2350_M33_DEBUG_MAX_REGS),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_rp2350_m33_debug = {
    .name = TYPE_RP2350_M33_DEBUG,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(comp, RP2350M33DebugState,
                             RP2350_M33_DEBUG_NUM_REGIONS, 1,
                             vmstate_rp2350_m33_debug_comp,
                             RP2350M33DebugComponent),
        VMSTATE_CLOCK(cpuclk, RP2350M33DebugState),
        VMSTATE_UINT32(dwt_cyccnt, RP2350M33DebugState),
        VMSTATE_INT64(dwt_cyccnt_ns, RP2350M33DebugState),
        VMSTATE_UINT32(cti_trigout_logged, RP2350M33DebugState),
        VMSTATE_END_OF_LIST()
    },
};

static void rp2350_m33_debug_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = rp2350_m33_debug_hold_reset;
    dc->vmsd = &vmstate_rp2350_m33_debug;
}

/* [spec:nuos:req:emu.coresight] */
static const TypeInfo rp2350_m33_debug_info = {
    .name          = TYPE_RP2350_M33_DEBUG,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RP2350M33DebugState),
    .instance_init = rp2350_m33_debug_init,
    .class_init    = rp2350_m33_debug_class_init,
};

static void rp2350_m33_debug_register_types(void)
{
    type_register_static(&rp2350_m33_debug_info);
}
type_init(rp2350_m33_debug_register_types)
