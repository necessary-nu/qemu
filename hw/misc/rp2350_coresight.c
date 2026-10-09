/*
 * RP2350 CoreSight debug and trace components (self-hosted debug window)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: RP2350 Datasheet, "Debug" (CORESIGHT_PERIPH_BASE), and the
 * Arm CoreSight SoC-600 TRM r4p1 for the components it is built from.
 *
 * The window at 0x40140000 holds, at 4 KiB granules:
 *
 *   0x0000  root ROM table (Raspberry Pi, part 0x004)
 *   0x2000  AHB5-AP for core 0 (its second logical AP at 0x3000)
 *   0x4000  AHB5-AP for core 1 (0x5000)
 *   0x6000  timestamp generator, control frame (css600_tsgen)
 *   0x7000  ATB funnel (css600_atbfunnel, 4 ports)
 *   0x8000  TPIU (css600_tpiu)
 *   0x9000  CTI (css600_cti, 5 triggers, 4 channels)
 *   0xa000  APB-AP onto the RISC-V Debug Module (0xb000)
 *
 * Every component reports the identification registers a hardware probe
 * reads from a real RP2350 (SoC-600 r4p0/r4p1 revisions: the AHB-AP IDR
 * is 0x34770008), and keeps its configuration registers with their reset
 * values and access types. SoC-600 has no software lock, so LAR is
 * ignored and LSR reads as zero (lock not implemented).
 *
 * The AHB-APs perform real memory accesses on their core's bus. No trace
 * source produces ATB data in this model, so the funnel, TPIU and capture
 * path carry nothing; configuring them to emit trace logs LOG_UNIMP.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/rp2350_coresight.h"
#include "migration/vmstate.h"
#include "system/memory.h"

/* Components, by 4 KiB granule within the window. */
#define SLOT_ROM        0x0
#define SLOT_AHBAP0     0x2
#define SLOT_AHBAP1     0x4
#define SLOT_TSGEN      0x6
#define SLOT_FUNNEL     0x7
#define SLOT_TPIU       0x8
#define SLOT_CTI        0x9
#define SLOT_APBAP      0xa

#define WINDOW_BASE     0x40140000

/* Registers every CoreSight component has at the top of its 4 KiB. */
#define A_ITCTRL        0xf00
#define A_CLAIMSET      0xfa0
#define A_CLAIMCLR      0xfa4
#define A_DEVAFF0       0xfa8
#define A_DEVAFF1       0xfac
#define A_LAR           0xfb0
#define A_LSR           0xfb4
#define A_AUTHSTATUS    0xfb8
#define A_DEVARCH       0xfbc
#define A_DEVID         0xfc8
#define A_DEVTYPE       0xfcc
#define A_PIDR4         0xfd0
#define A_PIDR5         0xfd4
#define A_PIDR6         0xfd8
#define A_PIDR7         0xfdc
#define A_PIDR0         0xfe0
#define A_PIDR1         0xfe4
#define A_PIDR2         0xfe8
#define A_PIDR3         0xfec
#define A_CIDR0         0xff0
#define A_CIDR1         0xff4
#define A_CIDR2         0xff8
#define A_CIDR3         0xffc

/* MEM-AP (ADIv6 APv2) registers. */
#define A_DAR_END       0x400
#define A_CSW           0xd00
#define A_TAR           0xd04
#define A_DRW           0xd0c
#define A_BD0           0xd10
#define A_BD3           0xd1c
#define A_TRR           0xd24
#define A_CFG           0xdf4
#define A_BASE          0xdf8
#define A_IDR           0xdfc
#define A_ITSTATUS      0xefc

#define CSW_SIZE        0x00000007u
#define CSW_ADDRINC     0x00000030u
#define CSW_DEVICEEN    (1u << 6)
#define CSW_HPROT6      (1u << 15)
#define CSW_ERRNPASS    (1u << 16)
#define CSW_ERRSTOP     (1u << 17)
#define CSW_SDEVICEEN   (1u << 23)
#define CSW_HPROT       0x1f000000u
#define CSW_HPROT_PRIV  (1u << 25)
#define CSW_HNONSEC     (1u << 30)
#define CSW_APB_PROT    0x70000000u

#define TRR_ERR         1u

/* Both MEM-APs: 10-bit TAR incrementer, error handling v1, 1 KiB DAR. */
#define MEMAP_CFG       0x000101a0u
#define AHBAP_IDR       0x34770008u
#define APBAP_IDR       0x24770006u
/* Each core's AHB-AP points the debugger at the Cortex-M33 ROM table. */
#define AHBAP_BASE      0xe00ff003u
/* The RISC-V Debug Module is not a CoreSight component: no entry. */
#define APBAP_BASE      0x00000002u
#define AHBAP_CSW_WMASK (CSW_HNONSEC | CSW_HPROT | CSW_ERRSTOP | \
                         CSW_ERRNPASS | CSW_HPROT6 | CSW_ADDRINC | CSW_SIZE)
#define APBAP_CSW_WMASK (CSW_APB_PROT | CSW_ERRSTOP | CSW_ERRNPASS | \
                         CSW_ADDRINC)
/*
 * RP2350 resets HNONSEC to 0, so the AHB-APs start out making Secure
 * accesses; the rest is the css600_ahbap reset (privileged data).
 */
#define AHBAP_CSW_RESET 0x03000002u
#define APBAP_CSW_RESET 0x30000002u

/* Timestamp generator, control frame. */
#define A_CNTCR         0x000
#define A_CNTSR         0x004
#define A_CNTCVL        0x008
#define A_CNTCVU        0x00c
#define A_CNTFID0       0x020
#define A_ITSTAT        0xef8
#define CNTCR_EN        (1u << 0)
#define CNTCR_HDBG      (1u << 1)

/* ATB funnel. */
#define A_FUNNELCONTROL     0x000
#define A_PRIORITYCONTROL   0x004
#define A_FUNNEL_IT_FIRST   0xeec
#define A_FUNNEL_IT_LAST    0xefc
#define FUNNEL_PORTS        4
#define FUNNEL_ENS          0x0000000fu
#define FUNNEL_CTRL_WMASK   0x00001f0fu
#define FUNNEL_CTRL_RESET   0x00000300u
#define FUNNEL_PRIO_WMASK   0x00000fffu

/* TPIU. */
#define A_SSPSR         0x000
#define A_CSPSR         0x004
#define A_STMR          0x100
#define A_TCVR          0x104
#define A_TCMR          0x108
#define A_STPMR         0x200
#define A_CTPMR         0x204
#define A_TPRCR         0x208
#define A_FFSR          0x300
#define A_FFCR          0x304
#define A_FSCR          0x308
#define A_EXTCTLIN      0x400
#define A_EXTCTLOUT     0x404
#define A_TPIU_IT_FIRST 0xee8
#define A_TPIU_IT_LAST  0xefc
/*
 * The TPIU is built 32 bits wide: four of the bits reach the trace pins,
 * and the on-chip capture FIFO takes all 32 (CSPSR = 0x80000000).
 */
#define TPIU_SSPSR      0xffffffffu
#define TPIU_STMR       0x0000011fu
#define TPIU_STPMR      0x0003000fu
#define TPIU_CTPMR_WMASK 0x0003000fu
#define TPIU_FFCR_WMASK 0x0000b773u
#define TPIU_FFCR_RESET 0x00001000u
#define TPIU_FFCR_FONMAN (1u << 6)
#define TPIU_FSCR_RESET 0x00000040u
/* Formatter stopped, no TRACECTL pin (RP2350 has none), no flush active. */
#define TPIU_FFSR       0x00000002u

/* CTI. */
#define A_CTICONTROL        0x000
#define A_CTIINTACK         0x010
#define A_CTIAPPSET         0x014
#define A_CTIAPPCLEAR       0x018
#define A_CTIAPPPULSE       0x01c
#define A_CTIINEN0          0x020
#define A_CTIINEN31         0x09c
#define A_CTIOUTEN0         0x0a0
#define A_CTIOUTEN31        0x11c
#define A_CTITRIGINSTATUS   0x130
#define A_CTITRIGOUTSTATUS  0x134
#define A_CTICHINSTATUS     0x138
#define A_CTICHOUTSTATUS    0x13c
#define A_CTIGATE           0x140
#define A_ASICCTRL          0x144
#define A_ITCHOUT           0xee4
#define A_ITTRIGOUT         0xee8
#define A_ITCHIN            0xef4
#define A_ITTRIGIN          0xef8
#define CTI_CHANNELS_MASK   ((1u << RP2350_CTI_CHANNELS) - 1)

/* JEP106 designer codes: continuation count and identity code. */
#define JEP_ARM_CONT    0x4
#define JEP_ARM_ID      0x3b
#define JEP_RPI_CONT    0x9
#define JEP_RPI_ID      0x13

/* Component classes, as CIDR1 reports them. */
#define CIDR1_CORESIGHT 0x90
#define CIDR1_GENERIC   0xf0

typedef struct CSIdent {
    uint16_t part;
    uint8_t jep_cont;
    uint8_t jep_id;
    uint8_t revision;
    uint8_t cidr1;
    uint32_t devarch;
    uint32_t devid;
    uint32_t devtype;
} CSIdent;

static const CSIdent rom_ident = {
    .part = 0x004, .jep_cont = JEP_RPI_CONT, .jep_id = JEP_RPI_ID,
    .cidr1 = CIDR1_CORESIGHT, .devarch = 0x47700af7,
};

static const CSIdent ahbap_ident = {
    .part = 0x9e3, .jep_cont = JEP_ARM_CONT, .jep_id = JEP_ARM_ID,
    .revision = 3, .cidr1 = CIDR1_CORESIGHT, .devarch = 0x47700a17,
};

static const CSIdent apbap_ident = {
    .part = 0x9e2, .jep_cont = JEP_ARM_CONT, .jep_id = JEP_ARM_ID,
    .revision = 2, .cidr1 = CIDR1_CORESIGHT, .devarch = 0x47700a17,
};

/* The timestamp generator is a generic IP component, not CoreSight. */
static const CSIdent tsgen_ident = {
    .part = 0x193, .jep_cont = JEP_ARM_CONT, .jep_id = JEP_ARM_ID,
    .revision = 0, .cidr1 = CIDR1_GENERIC,
};

static const CSIdent funnel_ident = {
    .part = 0x9eb, .jep_cont = JEP_ARM_CONT, .jep_id = JEP_ARM_ID,
    .revision = 2, .cidr1 = CIDR1_CORESIGHT,
    .devid = 0x34, .devtype = 0x12,
};

static const CSIdent tpiu_ident = {
    .part = 0x9e7, .jep_cont = JEP_ARM_CONT, .jep_id = JEP_ARM_ID,
    .revision = 2, .cidr1 = CIDR1_CORESIGHT,
    .devid = 0x20, .devtype = 0x11,
};

static const CSIdent cti_ident = {
    .part = 0x9ed, .jep_cont = JEP_ARM_CONT, .jep_id = JEP_ARM_ID,
    .revision = 3, .cidr1 = CIDR1_CORESIGHT, .devarch = 0x47701a14,
    /* One trigger interface each way, 4 channels, 5 triggers, no mux. */
    .devid = 0x01040500, .devtype = 0x14,
};

/*
 * The root ROM table lists six components. The APB-AP onto the RISC-V
 * Debug Module is not listed, though it answers at 0xa000.
 */
static const uint32_t rom_entries[] = {
    (SLOT_AHBAP0 << 12) | 3,
    (SLOT_AHBAP1 << 12) | 3,
    (SLOT_TSGEN << 12) | 3,
    (SLOT_FUNNEL << 12) | 3,
    (SLOT_TPIU << 12) | 3,
    (SLOT_CTI << 12) | 3,
};

static bool cs_ident_read(const CSIdent *id, hwaddr reg, uint32_t *v)
{
    switch (reg) {
    case A_DEVARCH:
        *v = id->devarch;
        return true;
    case A_DEVID:
        *v = id->devid;
        return true;
    case A_DEVTYPE:
        *v = id->devtype;
        return true;
    case A_PIDR4:
        /* SIZE 0: the component's extent is not stated. */
        *v = id->jep_cont;
        return true;
    case A_PIDR5:
    case A_PIDR6:
    case A_PIDR7:
    case A_PIDR3:
        *v = 0;
        return true;
    case A_PIDR0:
        *v = id->part & 0xff;
        return true;
    case A_PIDR1:
        *v = ((id->part >> 8) & 0xf) | ((id->jep_id & 0xf) << 4);
        return true;
    case A_PIDR2:
        *v = ((id->jep_id >> 4) & 0x7) | (1u << 3) | (id->revision << 4);
        return true;
    case A_CIDR0:
        *v = 0x0d;
        return true;
    case A_CIDR1:
        *v = id->cidr1;
        return true;
    case A_CIDR2:
        *v = 0x05;
        return true;
    case A_CIDR3:
        *v = 0xb1;
        return true;
    default:
        return false;
    }
}

/*
 * ITCTRL, the claim tags (`nclaim` of them) and the absent software lock,
 * which every component here shares.
 */
static bool cs_common_read(uint32_t itctrl, uint32_t claim, unsigned nclaim,
                           hwaddr reg, uint32_t *v)
{
    switch (reg) {
    case A_ITCTRL:
        *v = itctrl;
        return true;
    case A_CLAIMSET:
        *v = MAKE_64BIT_MASK(0, nclaim);
        return true;
    case A_CLAIMCLR:
        *v = claim;
        return true;
    case A_LAR:
    case A_LSR:
        *v = 0;
        return true;
    default:
        return false;
    }
}

static bool cs_common_write(uint32_t *itctrl, uint32_t *claim,
                            unsigned nclaim, hwaddr reg, uint32_t v)
{
    switch (reg) {
    case A_ITCTRL:
        *itctrl = v & 1;
        return true;
    case A_CLAIMSET:
        *claim |= v & MAKE_64BIT_MASK(0, nclaim);
        return true;
    case A_CLAIMCLR:
        *claim &= ~v;
        return true;
    case A_LAR:
        return true;
    default:
        return false;
    }
}

static bool dbgen(RP2350CoreSightState *s)
{
    return !s->debug_disable;
}

static bool spiden(RP2350CoreSightState *s)
{
    return !s->debug_disable && !s->secure_debug_disable;
}

/* One AUTHSTATUS field: 0b11 enabled, 0b10 disabled. */
static uint32_t auth_field(bool enabled)
{
    return enabled ? 3 : 2;
}

/* Secure and Non-secure, invasive and non-invasive. */
static uint32_t authstatus(RP2350CoreSightState *s)
{
    return auth_field(spiden(s)) << 6 | auth_field(spiden(s)) << 4 |
           auth_field(dbgen(s)) << 2 | auth_field(dbgen(s));
}

static void bad_write(const char *what, hwaddr addr)
{
    qemu_log_mask(LOG_GUEST_ERROR, "rp2350-coresight: %s: write to read-only "
                  "or reserved offset 0x%" HWADDR_PRIx "\n", what, addr);
}

static void bad_read(const char *what, hwaddr addr)
{
    qemu_log_mask(LOG_GUEST_ERROR, "rp2350-coresight: %s: read of reserved "
                  "offset 0x%" HWADDR_PRIx "\n", what, addr);
}

/* ROM table. */

static uint32_t rom_read(RP2350CoreSightState *s, hwaddr reg)
{
    uint32_t v;

    if (reg < 0x800) {
        /* Entries past the last read as zero, the end marker. */
        return reg / 4 < ARRAY_SIZE(rom_entries) ? rom_entries[reg / 4] : 0;
    }
    if (reg == A_AUTHSTATUS) {
        return authstatus(s);
    }
    if (cs_ident_read(&rom_ident, reg, &v)) {
        return v;
    }
    bad_read("ROM table", reg);
    return 0;
}

/* MEM-APs. */

typedef struct MemAP {
    const CSIdent *ident;
    RP2350MemAPRegs *regs;
    /* The core this AHB-AP debugs, or -1 for the APB-AP. */
    int core;
} MemAP;

static uint32_t memap_csw(RP2350CoreSightState *s, MemAP *ap)
{
    uint32_t csw = ap->regs->csw;

    if (dbgen(s)) {
        csw |= CSW_DEVICEEN;
    }
    /*
     * The secure debug disable gates only the AHB-APs' Secure enable; the
     * APB-AP's is tied high.
     */
    if (ap->core >= 0 ? spiden(s) : dbgen(s)) {
        csw |= CSW_SDEVICEEN;
    }
    return csw;
}

/* Log an error response; returns what the upstream interface answers. */
static MemTxResult memap_error(MemAP *ap)
{
    ap->regs->trr |= TRR_ERR;
    return ap->regs->csw & CSW_ERRNPASS ? MEMTX_OK : MEMTX_ERROR;
}

/*
 * The AHB-AP slots of the window, which an AHB-AP may not reach: such an
 * access is refused at once instead of deadlocking.
 */
static bool is_ahbap_slot(hwaddr addr)
{
    hwaddr slot = (addr - WINDOW_BASE) >> 12;

    return addr >= WINDOW_BASE && addr < WINDOW_BASE + RP2350_CORESIGHT_SIZE &&
           slot >= SLOT_AHBAP0 && slot < SLOT_AHBAP1 + 2;
}

/*
 * One memory transfer through the AP at `addr`, with the data on the
 * byte lanes of `*data` that the address selects. `*resp` is the answer
 * on the AP's own (upstream) interface. Returns whether the downstream
 * transfer completed without an error response, which is what lets TAR
 * auto-increment.
 */
static bool memap_transfer(RP2350CoreSightState *s, MemAP *ap, uint32_t addr,
                           uint32_t *data, bool write, MemTxResult *resp)
{
    RP2350MemAPRegs *r = ap->regs;
    uint32_t csw = memap_csw(s, ap);
    unsigned size = csw & CSW_SIZE;
    unsigned lane = (addr & 3) & ~((1u << size) - 1);
    AddressSpace *as;
    MemTxAttrs attrs = MEMTXATTRS_UNSPECIFIED;
    MemTxResult res = MEMTX_OK;
    uint32_t v = 0;

    *resp = MEMTX_OK;
    if (!write) {
        *data = 0;
    }
    if ((csw & CSW_ERRSTOP) && (r->trr & TRR_ERR)) {
        /* A logged error blocks further transfers until TRR is cleared. */
        *resp = csw & CSW_ERRNPASS ? MEMTX_OK : MEMTX_ERROR;
        return false;
    }
    if (size > 2) {
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-coresight: MEM-AP transfer "
                      "with reserved CSW.Size %u\n", size);
        *resp = memap_error(ap);
        return false;
    }
    if (!(csw & CSW_DEVICEEN)) {
        *resp = memap_error(ap);
        return false;
    }
    if (ap->core < 0) {
        qemu_log_mask(LOG_UNIMP, "rp2350-coresight: APB-AP %s at 0x%08x: the "
                      "RISC-V Debug Module is not modelled\n",
                      write ? "write" : "read", addr);
        return true;
    }
    if (!(csw & CSW_HNONSEC) && !(csw & CSW_SDEVICEEN)) {
        /* Secure transfers need the Secure AP enable. */
        *resp = memap_error(ap);
        return false;
    }
    if (is_ahbap_slot(addr)) {
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-coresight: AHB-AP access to "
                      "an AHB-AP at 0x%08x through the debug window\n", addr);
        *resp = memap_error(ap);
        return false;
    }

    attrs.unspecified = 0;
    attrs.secure = !(csw & CSW_HNONSEC);
    attrs.user = !(csw & CSW_HPROT_PRIV);
    as = &s->ap_as[ap->core];
    addr &= ~((1u << size) - 1);

    if (write) {
        v = *data >> (lane * 8);
        switch (size) {
        case 0:
            address_space_stb(as, addr, v, attrs, &res);
            break;
        case 1:
            address_space_stw_le(as, addr, v, attrs, &res);
            break;
        default:
            address_space_stl_le(as, addr, v, attrs, &res);
            break;
        }
    } else {
        switch (size) {
        case 0:
            v = address_space_ldub(as, addr, attrs, &res);
            break;
        case 1:
            v = address_space_lduw_le(as, addr, attrs, &res);
            break;
        default:
            v = address_space_ldl_le(as, addr, attrs, &res);
            break;
        }
    }
    if (res != MEMTX_OK) {
        *resp = memap_error(ap);
        return false;
    }

    /*
     * Every access into this window holds the system APB bridge, and an
     * AP transfer to an APB address needs that same bridge: the hardware
     * deadlocks until the bridge's 65535-cycle timeout abandons the
     * upstream access with a bus fault. The downstream transfer then
     * completes, but its read data is lost.
     */
    if ((addr >> 28) == 0x4) {
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-coresight: self-hosted AHB-AP "
                      "access to APB address 0x%08x deadlocks the APB "
                      "bridge until its timeout\n", addr);
        *resp = MEMTX_ERROR;
        return true;
    }
    if (!write) {
        *data = v << (lane * 8);
    }
    return true;
}

static MemTxResult memap_drw(RP2350CoreSightState *s, MemAP *ap,
                             uint32_t *data, bool write)
{
    RP2350MemAPRegs *r = ap->regs;
    unsigned size = r->csw & CSW_SIZE;
    unsigned inc = extract32(r->csw, 4, 2);
    MemTxResult res;

    if (!memap_transfer(s, ap, r->tar, data, write, &res)) {
        return res;
    }
    if (inc == 1) {
        /* The incrementer is 10 bits wide: TAR wraps within 1 KiB. */
        r->tar = (r->tar & ~0x3ffu) | ((r->tar + (1u << size)) & 0x3ff);
    } else if (inc) {
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-coresight: reserved "
                      "CSW.AddrInc %u\n", inc);
    }
    return res;
}

/* A transfer at a fixed address: DAR<n> or BD<n>, which never move TAR. */
static MemTxResult memap_direct(RP2350CoreSightState *s, MemAP *ap,
                                uint32_t addr, uint32_t *data, bool write)
{
    MemTxResult res;

    memap_transfer(s, ap, addr, data, write, &res);
    return res;
}

static MemTxResult memap_read(RP2350CoreSightState *s, MemAP *ap,
                              hwaddr reg, uint32_t *v)
{
    RP2350MemAPRegs *r = ap->regs;

    if (reg < A_DAR_END) {
        return memap_direct(s, ap, (r->tar & ~0x3ffu) | reg, v, false);
    }
    if (reg >= A_BD0 && reg <= A_BD3) {
        return memap_direct(s, ap, (r->tar & ~0xfu) | (reg - A_BD0), v,
                            false);
    }
    switch (reg) {
    case A_CSW:
        *v = memap_csw(s, ap);
        break;
    case A_TAR:
        *v = r->tar;
        break;
    case A_DRW:
        return memap_drw(s, ap, v, false);
    case A_TRR:
        *v = r->trr;
        break;
    case A_CFG:
        *v = MEMAP_CFG;
        break;
    case A_BASE:
        *v = ap->core >= 0 ? AHBAP_BASE : APBAP_BASE;
        break;
    case A_IDR:
        *v = ap->core >= 0 ? AHBAP_IDR : APBAP_IDR;
        break;
    case A_ITSTATUS:
        /* No DP abort reaches the AP from the self-hosted side. */
        *v = 0;
        break;
    case A_AUTHSTATUS:
        if (ap->core >= 0) {
            *v = authstatus(s);
        } else {
            *v = auth_field(dbgen(s)) * 0x55;
        }
        break;
    default:
        if (cs_common_read(r->itctrl, r->claim, 2, reg, v) ||
            cs_ident_read(ap->ident, reg, v)) {
            break;
        }
        bad_read("MEM-AP", reg);
        *v = 0;
        break;
    }
    return MEMTX_OK;
}

static MemTxResult memap_write(RP2350CoreSightState *s, MemAP *ap,
                               hwaddr reg, uint32_t v)
{
    RP2350MemAPRegs *r = ap->regs;

    if (reg < A_DAR_END) {
        return memap_direct(s, ap, (r->tar & ~0x3ffu) | reg, &v, true);
    }
    if (reg >= A_BD0 && reg <= A_BD3) {
        return memap_direct(s, ap, (r->tar & ~0xfu) | (reg - A_BD0), &v,
                            true);
    }
    switch (reg) {
    case A_CSW:
        if (ap->core >= 0) {
            r->csw = v & AHBAP_CSW_WMASK;
        } else {
            /* The APB-AP only makes word transfers. */
            r->csw = (v & APBAP_CSW_WMASK) | 2;
        }
        break;
    case A_TAR:
        r->tar = v;
        break;
    case A_DRW:
        return memap_drw(s, ap, &v, true);
    case A_TRR:
        r->trr &= ~(v & TRR_ERR);
        break;
    case A_ITSTATUS:
        break;
    default:
        if (!cs_common_write(&r->itctrl, &r->claim, 2, reg, v)) {
            bad_write("MEM-AP", reg);
        }
        break;
    }
    return MEMTX_OK;
}

/* Timestamp generator. */

static uint64_t tsgen_now(RP2350CoreSightState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (!(s->tsgen_cntcr & CNTCR_EN)) {
        return s->tsgen_count;
    }
    return s->tsgen_count + clock_ns_to_ticks(s->clk, now - s->tsgen_epoch_ns);
}

static void tsgen_sync(RP2350CoreSightState *s)
{
    s->tsgen_count = tsgen_now(s);
    s->tsgen_epoch_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

static void tsgen_clk_update(void *opaque, ClockEvent event)
{
    tsgen_sync(opaque);
}

static uint32_t tsgen_read(RP2350CoreSightState *s, hwaddr reg)
{
    uint32_t v;

    switch (reg) {
    case A_CNTCR:
        return s->tsgen_cntcr;
    case A_CNTSR:
        /* No core enters halting debug here, so halt_req never fires. */
        return 0;
    case A_CNTCVL:
        return tsgen_now(s);
    case A_CNTCVU:
        return tsgen_now(s) >> 32;
    case A_CNTFID0:
        return s->tsgen_cntfid0;
    case A_ITSTAT:
        return 0;
    case A_ITCTRL:
        return s->tsgen_itctrl;
    default:
        /* Not a CoreSight component: DEVARCH, DEVID, DEVTYPE read 0. */
        if (cs_ident_read(&tsgen_ident, reg, &v)) {
            return v;
        }
        bad_read("TSGEN", reg);
        return 0;
    }
}

static void tsgen_write(RP2350CoreSightState *s, hwaddr reg, uint32_t v)
{
    switch (reg) {
    case A_CNTCR:
        tsgen_sync(s);
        s->tsgen_cntcr = v & (CNTCR_EN | CNTCR_HDBG);
        break;
    case A_CNTCVL:
        /* Held until CNTCVU completes the 64-bit write. */
        s->tsgen_cntcvl = v;
        break;
    case A_CNTCVU:
        if (s->tsgen_cntcr & CNTCR_EN) {
            qemu_log_mask(LOG_GUEST_ERROR, "rp2350-coresight: TSGEN: CNTCVU "
                          "written while the counter runs\n");
        }
        s->tsgen_count = deposit64(s->tsgen_cntcvl, 32, 32, v);
        s->tsgen_epoch_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        break;
    case A_CNTFID0:
        s->tsgen_cntfid0 = v;
        break;
    case A_ITCTRL:
        s->tsgen_itctrl = v & 1;
        break;
    case A_LAR:
        break;
    default:
        bad_write("TSGEN", reg);
        break;
    }
}

/* ATB funnel. */

static uint32_t funnel_read(RP2350CoreSightState *s, hwaddr reg)
{
    uint32_t v;

    switch (reg) {
    case A_FUNNELCONTROL:
        return s->funnel_ctrl;
    case A_PRIORITYCONTROL:
        return s->funnel_prio;
    case A_FUNNEL_IT_FIRST ... A_FUNNEL_IT_LAST:
        /* The ATB integration signals are not modelled; nothing drives them. */
        if (s->funnel_itctrl) {
            qemu_log_mask(LOG_UNIMP, "rp2350-coresight: funnel integration "
                          "register 0x%" HWADDR_PRIx "\n", reg);
        }
        return 0;
    case A_AUTHSTATUS:
        return 0;
    default:
        if (cs_common_read(s->funnel_itctrl, s->funnel_claim, 4, reg, &v) ||
            cs_ident_read(&funnel_ident, reg, &v)) {
            return v;
        }
        bad_read("funnel", reg);
        return 0;
    }
}

static void funnel_write(RP2350CoreSightState *s, hwaddr reg, uint32_t v)
{
    switch (reg) {
    case A_FUNNELCONTROL:
        if (v & FUNNEL_ENS & ~s->funnel_ctrl) {
            /* Port 0/1 are core 0's ITM/ETM, 2/3 core 1's. */
            qemu_log_mask(LOG_UNIMP, "rp2350-coresight: funnel ports 0x%x "
                          "enabled: ITM and ETM produce no trace data\n",
                          v & FUNNEL_ENS);
        }
        s->funnel_ctrl = v & FUNNEL_CTRL_WMASK;
        break;
    case A_PRIORITYCONTROL:
        /* Writes with any port enabled are silently rejected. */
        if (!(s->funnel_ctrl & FUNNEL_ENS)) {
            s->funnel_prio = v & FUNNEL_PRIO_WMASK;
        }
        break;
    case A_FUNNEL_IT_FIRST ... A_FUNNEL_IT_LAST:
        if (s->funnel_itctrl) {
            qemu_log_mask(LOG_UNIMP, "rp2350-coresight: funnel integration "
                          "register 0x%" HWADDR_PRIx "\n", reg);
        }
        break;
    default:
        if (!cs_common_write(&s->funnel_itctrl, &s->funnel_claim, 4, reg, v)) {
            bad_write("funnel", reg);
        }
        break;
    }
}

/* TPIU. */

static uint32_t tpiu_read(RP2350CoreSightState *s, hwaddr reg)
{
    uint32_t v;

    switch (reg) {
    case A_SSPSR:
        return TPIU_SSPSR;
    case A_CSPSR:
        return s->tpiu_cspsr;
    case A_STMR:
        return TPIU_STMR;
    case A_TCVR:
        return s->tpiu_tcvr;
    case A_TCMR:
        return s->tpiu_tcmr;
    case A_STPMR:
        return TPIU_STPMR;
    case A_CTPMR:
        return s->tpiu_ctpmr;
    case A_TPRCR:
        return s->tpiu_tprcr;
    case A_FFSR:
        return TPIU_FFSR;
    case A_FFCR:
        return s->tpiu_ffcr;
    case A_FSCR:
        return s->tpiu_fscr;
    case A_EXTCTLIN:
        return 0;
    case A_EXTCTLOUT:
        return s->tpiu_extctlout;
    case A_TPIU_IT_FIRST ... A_TPIU_IT_LAST:
        if (s->tpiu_itctrl) {
            qemu_log_mask(LOG_UNIMP, "rp2350-coresight: TPIU integration "
                          "register 0x%" HWADDR_PRIx "\n", reg);
        }
        return 0;
    case A_AUTHSTATUS:
        return 0;
    default:
        if (cs_common_read(s->tpiu_itctrl, s->tpiu_claim, 4, reg, &v) ||
            cs_ident_read(&tpiu_ident, reg, &v)) {
            return v;
        }
        bad_read("TPIU", reg);
        return 0;
    }
}

static void tpiu_write(RP2350CoreSightState *s, hwaddr reg, uint32_t v)
{
    switch (reg) {
    case A_CSPSR:
        if (ctpop32(v) != 1) {
            qemu_log_mask(LOG_GUEST_ERROR, "rp2350-coresight: TPIU: CSPSR "
                          "0x%08x selects other than one port size\n", v);
        }
        s->tpiu_cspsr = v & TPIU_SSPSR;
        break;
    case A_TCVR:
        s->tpiu_tcvr = v & 0xff;
        break;
    case A_TCMR:
        s->tpiu_tcmr = v & 0x1f;
        break;
    case A_CTPMR:
        s->tpiu_ctpmr = v & TPIU_CTPMR_WMASK;
        if (s->tpiu_ctpmr & 0xf) {
            qemu_log_mask(LOG_UNIMP, "rp2350-coresight: TPIU test patterns "
                          "are not generated\n");
        }
        break;
    case A_TPRCR:
        s->tpiu_tprcr = v & 0xff;
        break;
    case A_FFCR:
        /*
         * A manual flush completes at once: no trace data is ever held in
         * the formatter, so FOnMan never reads back set.
         */
        s->tpiu_ffcr = v & TPIU_FFCR_WMASK & ~TPIU_FFCR_FONMAN;
        break;
    case A_FSCR:
        s->tpiu_fscr = v & 0xfff;
        break;
    case A_EXTCTLOUT:
        s->tpiu_extctlout = v & 0xff;
        break;
    case A_TPIU_IT_FIRST ... A_TPIU_IT_LAST:
        if (s->tpiu_itctrl) {
            qemu_log_mask(LOG_UNIMP, "rp2350-coresight: TPIU integration "
                          "register 0x%" HWADDR_PRIx "\n", reg);
        }
        break;
    default:
        if (!cs_common_write(&s->tpiu_itctrl, &s->tpiu_claim, 4, reg, v)) {
            bad_write("TPIU", reg);
        }
        break;
    }
}

/* CTI. */

/* The channels as the trigger outputs and the channel outputs see them. */
static uint32_t cti_channels(RP2350CoreSightState *s)
{
    return s->cti_ctrl & 1 ? s->cti_app : 0;
}

static uint32_t cti_trigouts(RP2350CoreSightState *s, uint32_t channels)
{
    uint32_t out = 0;
    int n;

    for (n = 0; n < RP2350_CTI_TRIGGERS; n++) {
        if (channels & s->cti_outen[n]) {
            out |= 1u << n;
        }
    }
    return out;
}

static void cti_log_events(uint32_t triggers)
{
    if (triggers) {
        qemu_log_mask(LOG_UNIMP, "rp2350-coresight: CTI trigger outputs 0x%x "
                      "asserted: their destinations are not modelled\n",
                      triggers);
    }
}

static void cti_update(RP2350CoreSightState *s)
{
    uint32_t out = cti_trigouts(s, cti_channels(s));

    cti_log_events(out & ~s->cti_trigout);
    s->cti_trigout = out;
}

static uint32_t cti_read(RP2350CoreSightState *s, hwaddr reg)
{
    uint32_t v;

    switch (reg) {
    case A_CTICONTROL:
        return s->cti_ctrl;
    case A_CTIAPPSET:
        return s->cti_app;
    case A_CTIINEN0 ... A_CTIINEN31:
        v = (reg - A_CTIINEN0) / 4;
        return v < RP2350_CTI_TRIGGERS ? s->cti_inen[v] : 0;
    case A_CTIOUTEN0 ... A_CTIOUTEN31:
        v = (reg - A_CTIOUTEN0) / 4;
        return v < RP2350_CTI_TRIGGERS ? s->cti_outen[v] : 0;
    case A_CTITRIGINSTATUS:
    case A_CTICHINSTATUS:
        /* No trigger source or cross-trigger matrix drives the inputs. */
        return 0;
    case A_CTITRIGOUTSTATUS:
        return s->cti_trigout;
    case A_CTICHOUTSTATUS:
        return cti_channels(s) & s->cti_gate;
    case A_CTIGATE:
        return s->cti_gate;
    case A_ASICCTRL:
        /* No external multiplexer (DEVID.EXTMUXNUM is 0). */
        return 0;
    case A_ITCHIN:
    case A_ITTRIGIN:
    case A_ITCHOUT:
    case A_ITTRIGOUT:
        return 0;
    case A_DEVAFF0:
    case A_DEVAFF1:
        return 0;
    case A_AUTHSTATUS:
        /* DBGEN and NIDEN, which the debug disable flag drops. */
        return auth_field(dbgen(s)) << 2 | auth_field(dbgen(s));
    case A_CTIINTACK:
    case A_CTIAPPCLEAR:
    case A_CTIAPPPULSE:
        return 0;
    default:
        if (cs_common_read(s->cti_itctrl, s->cti_claim, 4, reg, &v) ||
            cs_ident_read(&cti_ident, reg, &v)) {
            return v;
        }
        bad_read("CTI", reg);
        return 0;
    }
}

static void cti_write(RP2350CoreSightState *s, hwaddr reg, uint32_t v)
{
    unsigned n;

    switch (reg) {
    case A_CTICONTROL:
        s->cti_ctrl = v & 1;
        break;
    case A_CTIINTACK:
        /* No trigger output here latches for a software handshake. */
        break;
    case A_CTIAPPSET:
        s->cti_app |= v & CTI_CHANNELS_MASK;
        break;
    case A_CTIAPPCLEAR:
        s->cti_app &= ~v;
        break;
    case A_CTIAPPPULSE:
        if (s->cti_ctrl & 1) {
            cti_log_events(cti_trigouts(s, v & CTI_CHANNELS_MASK) &
                           ~s->cti_trigout);
        }
        break;
    case A_CTIINEN0 ... A_CTIINEN31:
        n = (reg - A_CTIINEN0) / 4;
        if (n < RP2350_CTI_TRIGGERS) {
            s->cti_inen[n] = v & CTI_CHANNELS_MASK;
        }
        break;
    case A_CTIOUTEN0 ... A_CTIOUTEN31:
        n = (reg - A_CTIOUTEN0) / 4;
        if (n < RP2350_CTI_TRIGGERS) {
            s->cti_outen[n] = v & CTI_CHANNELS_MASK;
        }
        break;
    case A_CTIGATE:
        s->cti_gate = v & CTI_CHANNELS_MASK;
        break;
    case A_ASICCTRL:
        break;
    case A_ITCHOUT:
    case A_ITTRIGOUT:
        if (s->cti_itctrl) {
            qemu_log_mask(LOG_UNIMP, "rp2350-coresight: CTI integration "
                          "outputs are not connected\n");
        }
        break;
    default:
        if (!cs_common_write(&s->cti_itctrl, &s->cti_claim, 4, reg, v)) {
            bad_write("CTI", reg);
        }
        break;
    }
    cti_update(s);
}

/* The window. */

/* Which core's AHB-AP is at `slot`, or -1 if none is. */
static int ahbap_core(hwaddr slot)
{
    if (slot >= SLOT_AHBAP0 && slot < SLOT_AHBAP0 + 2) {
        return 0;
    }
    if (slot >= SLOT_AHBAP1 && slot < SLOT_AHBAP1 + 2) {
        return 1;
    }
    return -1;
}

static bool window_memap(RP2350CoreSightState *s, hwaddr slot, MemAP *ap)
{
    int core = ahbap_core(slot);

    if (core >= 0) {
        ap->ident = &ahbap_ident;
        ap->regs = &s->ahbap[core][slot & 1];
        ap->core = core;
        return true;
    }
    if (slot == SLOT_APBAP || slot == SLOT_APBAP + 1) {
        ap->ident = &apbap_ident;
        ap->regs = &s->apbap[slot - SLOT_APBAP];
        ap->core = -1;
        return true;
    }
    return false;
}

/* A core reaching its own AHB-AP gets a bus fault, not a deadlock. */
static bool own_ap(int requester, hwaddr slot)
{
    int core = ahbap_core(slot);

    return core >= 0 && requester == RP2350_CORESIGHT_CORE0 + core;
}

static MemTxResult window_read32(RP2350CoreSightView *view, hwaddr addr,
                                 uint32_t *v)
{
    RP2350CoreSightState *s = view->cs;
    hwaddr slot = addr >> 12;
    hwaddr reg = addr & 0xfff;
    MemAP ap;

    if (own_ap(view->requester, slot)) {
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-coresight: core %d read of "
                      "its own AHB-AP\n",
                      view->requester - RP2350_CORESIGHT_CORE0);
        return MEMTX_ERROR;
    }
    if (window_memap(s, slot, &ap)) {
        return memap_read(s, &ap, reg, v);
    }
    switch (slot) {
    case SLOT_ROM:
        *v = rom_read(s, reg);
        break;
    case SLOT_TSGEN:
        *v = tsgen_read(s, reg);
        break;
    case SLOT_FUNNEL:
        *v = funnel_read(s, reg);
        break;
    case SLOT_TPIU:
        *v = tpiu_read(s, reg);
        break;
    case SLOT_CTI:
        *v = cti_read(s, reg);
        break;
    default:
        bad_read("window", addr);
        *v = 0;
        break;
    }
    return MEMTX_OK;
}

static MemTxResult window_write32(RP2350CoreSightView *view, hwaddr addr,
                                  uint32_t v)
{
    RP2350CoreSightState *s = view->cs;
    hwaddr slot = addr >> 12;
    hwaddr reg = addr & 0xfff;
    MemAP ap;

    if (own_ap(view->requester, slot)) {
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-coresight: core %d write to "
                      "its own AHB-AP\n",
                      view->requester - RP2350_CORESIGHT_CORE0);
        return MEMTX_ERROR;
    }
    if (window_memap(s, slot, &ap)) {
        return memap_write(s, &ap, reg, v);
    }
    switch (slot) {
    case SLOT_ROM:
        bad_write("ROM table", reg);
        break;
    case SLOT_TSGEN:
        tsgen_write(s, reg, v);
        break;
    case SLOT_FUNNEL:
        funnel_write(s, reg, v);
        break;
    case SLOT_TPIU:
        tpiu_write(s, reg, v);
        break;
    case SLOT_CTI:
        cti_write(s, reg, v);
        break;
    default:
        bad_write("window", addr);
        break;
    }
    return MEMTX_OK;
}

/*
 * The components are 32-bit registers. A narrow write is replicated
 * across the bus by the processor and writes the whole register; a
 * narrow read returns its byte lanes.
 */
/* [spec:nuos:req:emu.coresight] */
static MemTxResult rp2350_coresight_read(void *opaque, hwaddr addr,
                                         uint64_t *data, unsigned size,
                                         MemTxAttrs attrs)
{
    uint32_t v = 0;
    MemTxResult r = window_read32(opaque, addr & ~3, &v);

    *data = extract32(v, (addr & 3) * 8, size * 8);
    return r;
}

/* [spec:nuos:req:emu.coresight] */
static MemTxResult rp2350_coresight_write(void *opaque, hwaddr addr,
                                          uint64_t value, unsigned size,
                                          MemTxAttrs attrs)
{
    uint32_t v = value;

    if (size == 1) {
        v = (v & 0xff) * 0x01010101u;
    } else if (size == 2) {
        v = (v & 0xffff) * 0x00010001u;
    }
    return window_write32(opaque, addr & ~3, v);
}

static const MemoryRegionOps rp2350_coresight_ops = {
    .read_with_attrs = rp2350_coresight_read,
    .write_with_attrs = rp2350_coresight_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

MemoryRegion *rp2350_coresight_view(RP2350CoreSightState *s, int requester)
{
    return &s->view[requester];
}

static void rp2350_coresight_reset_hold(Object *obj, ResetType type)
{
    RP2350CoreSightState *s = RP2350_CORESIGHT(obj);
    int i, l;

    for (l = 0; l < 2; l++) {
        for (i = 0; i < RP2350_CORESIGHT_CORES; i++) {
            s->ahbap[i][l] = (RP2350MemAPRegs) { .csw = AHBAP_CSW_RESET };
        }
        s->apbap[l] = (RP2350MemAPRegs) { .csw = APBAP_CSW_RESET };
    }

    s->tsgen_cntcr = 0;
    s->tsgen_cntcvl = 0;
    s->tsgen_cntfid0 = 0;
    s->tsgen_itctrl = 0;
    s->tsgen_count = 0;
    s->tsgen_epoch_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    s->funnel_ctrl = FUNNEL_CTRL_RESET;
    s->funnel_prio = 0;
    s->funnel_itctrl = 0;
    s->funnel_claim = 0;

    s->tpiu_cspsr = 1;
    s->tpiu_tcvr = 0;
    s->tpiu_tcmr = 0;
    s->tpiu_ctpmr = 0;
    s->tpiu_tprcr = 0;
    s->tpiu_ffcr = TPIU_FFCR_RESET;
    s->tpiu_fscr = TPIU_FSCR_RESET;
    s->tpiu_extctlout = 0;
    s->tpiu_itctrl = 0;
    s->tpiu_claim = 0;

    s->cti_ctrl = 0;
    s->cti_app = 0;
    memset(s->cti_inen, 0, sizeof(s->cti_inen));
    memset(s->cti_outen, 0, sizeof(s->cti_outen));
    s->cti_gate = CTI_CHANNELS_MASK;
    s->cti_itctrl = 0;
    s->cti_claim = 0;
    s->cti_trigout = 0;
}

/* The debug disable signals from OTP. */
static void rp2350_coresight_set_debug_disable(void *opaque, int n, int level)
{
    RP2350CoreSightState *s = opaque;

    s->debug_disable = level;
}

static void rp2350_coresight_set_secure_debug_disable(void *opaque, int n,
                                                      int level)
{
    RP2350CoreSightState *s = opaque;

    s->secure_debug_disable = level;
}

static void rp2350_coresight_init(Object *obj)
{
    RP2350CoreSightState *s = RP2350_CORESIGHT(obj);
    static const char *const names[RP2350_CORESIGHT_VIEWS] = {
        "rp2350-coresight", "rp2350-coresight.core0",
        "rp2350-coresight.core1",
    };
    int i;

    for (i = 0; i < RP2350_CORESIGHT_VIEWS; i++) {
        s->view_opaque[i].cs = s;
        s->view_opaque[i].requester = i;
        memory_region_init_io(&s->view[i], obj, &rp2350_coresight_ops,
                              &s->view_opaque[i], names[i],
                              RP2350_CORESIGHT_SIZE);
        /*
         * An AP transfer can land back in this window through the bus,
         * which hardware allows (and then times out); no state is held
         * across the nested access.
         */
        s->view[i].disable_reentrancy_guard = true;
    }
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->view[RP2350_CORESIGHT_SYSTEM]);
    s->clk = qdev_init_clock_in(DEVICE(obj), "clk", tsgen_clk_update, s,
                                ClockPreUpdate);
    qdev_init_gpio_in_named(DEVICE(obj), rp2350_coresight_set_debug_disable,
                            "debug-disable", 1);
    qdev_init_gpio_in_named(DEVICE(obj),
                            rp2350_coresight_set_secure_debug_disable,
                            "secure-debug-disable", 1);
}

static void rp2350_coresight_realize(DeviceState *dev, Error **errp)
{
    RP2350CoreSightState *s = RP2350_CORESIGHT(dev);
    int i;

    for (i = 0; i < RP2350_CORESIGHT_CORES; i++) {
        if (!s->core_memory[i]) {
            error_setg(errp, "core%d-memory property was not set", i);
            return;
        }
        address_space_init(&s->ap_as[i], s->core_memory[i],
                           i ? "rp2350-coresight.ahbap1"
                             : "rp2350-coresight.ahbap0");
    }
}

static const VMStateDescription vmstate_rp2350_memap = {
    .name = "rp2350-coresight-memap",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(csw, RP2350MemAPRegs),
        VMSTATE_UINT32(tar, RP2350MemAPRegs),
        VMSTATE_UINT32(trr, RP2350MemAPRegs),
        VMSTATE_UINT32(itctrl, RP2350MemAPRegs),
        VMSTATE_UINT32(claim, RP2350MemAPRegs),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_rp2350_coresight = {
    .name = TYPE_RP2350_CORESIGHT,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_BOOL(debug_disable, RP2350CoreSightState),
        VMSTATE_BOOL(secure_debug_disable, RP2350CoreSightState),
        VMSTATE_STRUCT_2DARRAY(ahbap, RP2350CoreSightState,
                               RP2350_CORESIGHT_CORES, 2, 1,
                               vmstate_rp2350_memap, RP2350MemAPRegs),
        VMSTATE_STRUCT_ARRAY(apbap, RP2350CoreSightState, 2, 1,
                             vmstate_rp2350_memap, RP2350MemAPRegs),
        VMSTATE_UINT32(tsgen_cntcr, RP2350CoreSightState),
        VMSTATE_UINT32(tsgen_cntcvl, RP2350CoreSightState),
        VMSTATE_UINT32(tsgen_cntfid0, RP2350CoreSightState),
        VMSTATE_UINT32(tsgen_itctrl, RP2350CoreSightState),
        VMSTATE_UINT64(tsgen_count, RP2350CoreSightState),
        VMSTATE_INT64(tsgen_epoch_ns, RP2350CoreSightState),
        VMSTATE_UINT32(funnel_ctrl, RP2350CoreSightState),
        VMSTATE_UINT32(funnel_prio, RP2350CoreSightState),
        VMSTATE_UINT32(funnel_itctrl, RP2350CoreSightState),
        VMSTATE_UINT32(funnel_claim, RP2350CoreSightState),
        VMSTATE_UINT32(tpiu_cspsr, RP2350CoreSightState),
        VMSTATE_UINT32(tpiu_tcvr, RP2350CoreSightState),
        VMSTATE_UINT32(tpiu_tcmr, RP2350CoreSightState),
        VMSTATE_UINT32(tpiu_ctpmr, RP2350CoreSightState),
        VMSTATE_UINT32(tpiu_tprcr, RP2350CoreSightState),
        VMSTATE_UINT32(tpiu_ffcr, RP2350CoreSightState),
        VMSTATE_UINT32(tpiu_fscr, RP2350CoreSightState),
        VMSTATE_UINT32(tpiu_extctlout, RP2350CoreSightState),
        VMSTATE_UINT32(tpiu_itctrl, RP2350CoreSightState),
        VMSTATE_UINT32(tpiu_claim, RP2350CoreSightState),
        VMSTATE_UINT32(cti_ctrl, RP2350CoreSightState),
        VMSTATE_UINT32(cti_app, RP2350CoreSightState),
        VMSTATE_UINT32_ARRAY(cti_inen, RP2350CoreSightState,
                             RP2350_CTI_TRIGGERS),
        VMSTATE_UINT32_ARRAY(cti_outen, RP2350CoreSightState,
                             RP2350_CTI_TRIGGERS),
        VMSTATE_UINT32(cti_gate, RP2350CoreSightState),
        VMSTATE_UINT32(cti_itctrl, RP2350CoreSightState),
        VMSTATE_UINT32(cti_claim, RP2350CoreSightState),
        VMSTATE_UINT32(cti_trigout, RP2350CoreSightState),
        VMSTATE_CLOCK(clk, RP2350CoreSightState),
        VMSTATE_END_OF_LIST()
    },
};

static const Property rp2350_coresight_properties[] = {
    DEFINE_PROP_LINK("core0-memory", RP2350CoreSightState, core_memory[0],
                     TYPE_MEMORY_REGION, MemoryRegion *),
    DEFINE_PROP_LINK("core1-memory", RP2350CoreSightState, core_memory[1],
                     TYPE_MEMORY_REGION, MemoryRegion *),
};

static void rp2350_coresight_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = rp2350_coresight_realize;
    dc->vmsd = &vmstate_rp2350_coresight;
    device_class_set_props(dc, rp2350_coresight_properties);
    rc->phases.hold = rp2350_coresight_reset_hold;
}

/* [spec:nuos:req:emu.coresight] */
static const TypeInfo rp2350_coresight_info = {
    .name          = TYPE_RP2350_CORESIGHT,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RP2350CoreSightState),
    .instance_init = rp2350_coresight_init,
    .class_init    = rp2350_coresight_class_init,
};

static void rp2350_coresight_register_types(void)
{
    type_register_static(&rp2350_coresight_info);
}
type_init(rp2350_coresight_register_types)
