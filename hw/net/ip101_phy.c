/*
 * IC Plus IP101GR 10/100 Ethernet PHY, managed over MDIO
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The PHY of Espressif's ESP32-Ethernet-Kit and the one ESP-IDF's esp_eth
 * drives by default. Registers 0-15 are the IEEE 802.3 clause 22 set;
 * registers 16-31 are paged by the Page Control Register (20), and page
 * 16, selected at reset, holds the interrupt status (17) and the MDI/MDIX
 * control and specific status (30) that report the resolved link.
 *
 * The medium is QEMU's network backend, a link partner that advertises
 * every 10/100 mode and symmetric pause. Auto-negotiation with it, and
 * reset, complete at once. The PHY's interrupt pin is not modelled.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/qdev-properties.h"
#include "hw/net/ip101_phy.h"
#include "hw/net/mii.h"
#include "migration/vmstate.h"

#define IP101_PHYID1        0x0243
#define IP101_PHYID2        0x0c54

#define IP101_BMCR_RESET    (MII_BMCR_SPEED100 | MII_BMCR_AUTOEN | MII_BMCR_FD)
#define IP101_BMCR_RW       (MII_BMCR_LOOPBACK | MII_BMCR_SPEED100 | \
                             MII_BMCR_AUTOEN | MII_BMCR_PDOWN | \
                             MII_BMCR_ISOLATE | MII_BMCR_FD | MII_BMCR_CTST)
#define IP101_BMSR_BASE     (MII_BMSR_100TX_FD | MII_BMSR_100TX_HD | \
                             MII_BMSR_10T_FD | MII_BMSR_10T_HD | \
                             MII_BMSR_MFPS | MII_BMSR_AUTONEG | \
                             MII_BMSR_EXTCAP)
#define IP101_ANAR_RESET    (MII_ANAR_PAUSE | MII_ANAR_TXFD | MII_ANAR_TX | \
                             MII_ANAR_10FD | MII_ANAR_10 | MII_ANAR_CSMACD)
#define IP101_ANAR_RW       (MII_ANAR_RFAULT | MII_ANAR_PAUSE_ASYM | \
                             MII_ANAR_PAUSE | MII_ANAR_TXFD | MII_ANAR_TX | \
                             MII_ANAR_10FD | MII_ANAR_10)
#define IP101_ANAR_ABILITY  (MII_ANAR_TXFD | MII_ANAR_TX | MII_ANAR_10FD | \
                             MII_ANAR_10)
/* What the backend, as link partner, advertises */
#define IP101_ANLPAR_LINKED (MII_ANLPAR_ACK | MII_ANLPAR_PAUSE | \
                             MII_ANLPAR_TXFD | MII_ANLPAR_TX | \
                             MII_ANLPAR_10FD | MII_ANLPAR_10 | \
                             MII_ANLPAR_CSMACD)

#define IP101_REG_PSCR      16
#define IP101_REG_ISR       17
#define IP101_REG_PCR       20
#define IP101_REG_CSSR      30

#define IP101_PCR_PAGE      0x1f
#define IP101_PCR_RESET     16

/* ISR, page 16: status bits clear on read, the rest are R/W */
#define IP101_ISR_LINK      (1u << 0)
#define IP101_ISR_DUPLEX    (1u << 1)
#define IP101_ISR_SPEED     (1u << 2)
#define IP101_ISR_INTR      (1u << 3)
#define IP101_ISR_STATUS    0x000f
#define IP101_ISR_RW        0x8f00

/* CSSR, page 16 */
#define IP101_CSSR_FORCE_MDIX (1u << 3)
#define IP101_CSSR_LINK_UP  (1u << 8)

static void ip101_update_link(IP101PhyState *s)
{
    bool link = s->medium_up && !(s->bmcr & MII_BMCR_PDOWN);
    bool speed_100 = s->speed_100;
    bool full_duplex = s->full_duplex;
    uint16_t isr = 0;

    if (s->bmcr & MII_BMCR_AUTOEN) {
        uint16_t common = s->anar & IP101_ANAR_ABILITY;

        /* The partner advertises every mode; take the best we offer. */
        link = link && common;
        s->an_complete = link;
        if (link) {
            s->page_received = true;
            speed_100 = common & (MII_ANAR_TXFD | MII_ANAR_TX);
            full_duplex = speed_100 ? (common & MII_ANAR_TXFD) :
                                      (common & MII_ANAR_10FD);
        }
    } else {
        s->an_complete = false;
        speed_100 = s->bmcr & MII_BMCR_SPEED100;
        full_duplex = s->bmcr & MII_BMCR_FD;
    }

    if (link != s->link_up) {
        isr |= IP101_ISR_LINK;
    }
    if (link && speed_100 != s->speed_100) {
        isr |= IP101_ISR_SPEED;
    }
    if (link && full_duplex != s->full_duplex) {
        isr |= IP101_ISR_DUPLEX;
    }
    if (isr) {
        s->isr |= isr | IP101_ISR_INTR;
    }
    if (!link) {
        s->bmsr_link = false;
    }
    s->link_up = link;
    s->speed_100 = speed_100;
    s->full_duplex = full_duplex;
}

static void ip101_reset_regs(IP101PhyState *s)
{
    s->bmcr = IP101_BMCR_RESET;
    s->anar = IP101_ANAR_RESET;
    s->bmsr_link = false;
    s->an_complete = false;
    s->page_received = false;
    s->link_up = false;
    s->speed_100 = true;
    s->full_duplex = true;
    s->pcr = IP101_PCR_RESET;
    s->isr = 0;
    s->pscr16 = 0;
    s->cssr_ctrl = 0;
    ip101_update_link(s);
    /* Coming out of reset is not a link change. */
    s->isr = 0;
}

void ip101_phy_set_medium(IP101PhyState *s, bool up)
{
    s->medium_up = up;
    ip101_update_link(s);
}

bool ip101_phy_link_up(IP101PhyState *s)
{
    return s->link_up && !(s->bmcr & (MII_BMCR_ISOLATE | MII_BMCR_LOOPBACK));
}

bool ip101_phy_loopback(IP101PhyState *s)
{
    return (s->bmcr & MII_BMCR_LOOPBACK) &&
           !(s->bmcr & (MII_BMCR_ISOLATE | MII_BMCR_PDOWN));
}

bool ip101_phy_isolated(IP101PhyState *s)
{
    return s->bmcr & MII_BMCR_ISOLATE;
}

/* The resolved mode, as CSSR's operation mode field encodes it */
static uint16_t ip101_op_mode(IP101PhyState *s)
{
    if (!s->link_up) {
        return 0;
    }
    return (s->full_duplex ? 4 : 0) | (s->speed_100 ? 2 : 1);
}

/* [spec:nuos:req:emu.esp32.emac] */
uint16_t ip101_phy_mdio_read(IP101PhyState *s, unsigned reg)
{
    uint16_t v;

    switch (reg) {
    case MII_BMCR:
        return s->bmcr;
    case MII_BMSR:
        v = IP101_BMSR_BASE | (s->bmsr_link ? MII_BMSR_LINK_ST : 0) |
            (s->an_complete ? MII_BMSR_AN_COMP : 0);
        /* Reading releases the latched-low link status. */
        s->bmsr_link = s->link_up;
        return v;
    case MII_PHYID1:
        return IP101_PHYID1;
    case MII_PHYID2:
        return IP101_PHYID2;
    case MII_ANAR:
        return s->anar;
    case MII_ANLPAR:
        return s->an_complete ? IP101_ANLPAR_LINKED : 0;
    case MII_ANER:
        v = (s->an_complete ? MII_ANER_NWAY : 0) |
            (s->page_received ? (1u << 1) : 0);
        s->page_received = false;
        return v;
    case MII_ANNP ... MII_EXTSTAT:
        /* No next pages, no 1000BASE-T, no MMDs: these read as zero. */
        return 0;
    case IP101_REG_PCR:
        return s->pcr;
    }

    if ((s->pcr & IP101_PCR_PAGE) != 16) {
        qemu_log_mask(LOG_UNIMP, "ip101: read of register %u on page %u\n",
                      reg, s->pcr & IP101_PCR_PAGE);
        return 0;
    }
    switch (reg) {
    case IP101_REG_PSCR:
        return s->pscr16;
    case IP101_REG_ISR:
        v = s->isr;
        s->isr &= ~IP101_ISR_STATUS;
        return v;
    case IP101_REG_CSSR:
        return ip101_op_mode(s) | s->cssr_ctrl |
               (s->link_up ? IP101_CSSR_LINK_UP : 0);
    default:
        qemu_log_mask(LOG_UNIMP, "ip101: read of page 16 register %u\n",
                      reg);
        return 0;
    }
}

/* [spec:nuos:req:emu.esp32.emac] */
void ip101_phy_mdio_write(IP101PhyState *s, unsigned reg, uint16_t val)
{
    switch (reg) {
    case MII_BMCR:
        if (val & MII_BMCR_RESET) {
            ip101_reset_regs(s);
            return;
        }
        s->bmcr = val & IP101_BMCR_RW;
        if ((val & MII_BMCR_ANRESTART) && (s->bmcr & MII_BMCR_AUTOEN)) {
            /* Renegotiation takes the link down before it comes back. */
            s->an_complete = false;
            s->bmsr_link = false;
        }
        ip101_update_link(s);
        return;
    case MII_ANAR:
        s->anar = (val & IP101_ANAR_RW) | MII_ANAR_CSMACD;
        return;
    case MII_BMSR:
    case MII_PHYID1:
    case MII_PHYID2:
    case MII_ANLPAR:
    case MII_ANER:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "ip101: write to read-only register %u\n", reg);
        return;
    case MII_ANNP ... MII_EXTSTAT:
        qemu_log_mask(LOG_UNIMP, "ip101: write to register %u\n", reg);
        return;
    case IP101_REG_PCR:
        s->pcr = val & IP101_PCR_PAGE;
        return;
    }

    if ((s->pcr & IP101_PCR_PAGE) != 16) {
        qemu_log_mask(LOG_UNIMP, "ip101: write to register %u on page %u\n",
                      reg, s->pcr & IP101_PCR_PAGE);
        return;
    }
    switch (reg) {
    case IP101_REG_PSCR:
        s->pscr16 = val;
        return;
    case IP101_REG_ISR:
        s->isr = (s->isr & IP101_ISR_STATUS) | (val & IP101_ISR_RW);
        return;
    case IP101_REG_CSSR:
        s->cssr_ctrl = val & IP101_CSSR_FORCE_MDIX;
        return;
    default:
        qemu_log_mask(LOG_UNIMP, "ip101: write to page 16 register %u\n",
                      reg);
        return;
    }
}

static void ip101_phy_reset_hold(Object *obj, ResetType type)
{
    ip101_reset_regs(IP101_PHY(obj));
}

static const VMStateDescription vmstate_ip101_phy = {
    .name = TYPE_IP101_PHY,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_BOOL(medium_up, IP101PhyState),
        VMSTATE_UINT16(bmcr, IP101PhyState),
        VMSTATE_UINT16(anar, IP101PhyState),
        VMSTATE_BOOL(bmsr_link, IP101PhyState),
        VMSTATE_BOOL(an_complete, IP101PhyState),
        VMSTATE_BOOL(page_received, IP101PhyState),
        VMSTATE_BOOL(link_up, IP101PhyState),
        VMSTATE_BOOL(speed_100, IP101PhyState),
        VMSTATE_BOOL(full_duplex, IP101PhyState),
        VMSTATE_UINT16(pcr, IP101PhyState),
        VMSTATE_UINT16(isr, IP101PhyState),
        VMSTATE_UINT16(pscr16, IP101PhyState),
        VMSTATE_UINT16(cssr_ctrl, IP101PhyState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property ip101_phy_properties[] = {
    DEFINE_PROP_UINT8("addr", IP101PhyState, addr, 1),
};

static void ip101_phy_realize(DeviceState *dev, Error **errp)
{
    IP101PhyState *s = IP101_PHY(dev);

    if (s->addr > 31) {
        error_setg(errp, "ip101-phy: MDIO address %u out of range", s->addr);
    }
}

static void ip101_phy_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = ip101_phy_reset_hold;
    dc->realize = ip101_phy_realize;
    dc->vmsd = &vmstate_ip101_phy;
    device_class_set_props(dc, ip101_phy_properties);
}

/* [spec:nuos:req:emu.esp32.emac] */
static const TypeInfo ip101_phy_info = {
    .name = TYPE_IP101_PHY,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(IP101PhyState),
    .class_init = ip101_phy_class_init,
};

static void ip101_phy_register_types(void)
{
    type_register_static(&ip101_phy_info);
}

type_init(ip101_phy_register_types)
