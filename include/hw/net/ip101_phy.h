/*
 * IC Plus IP101GR 10/100 Ethernet PHY, managed over MDIO
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_NET_IP101_PHY_H
#define HW_NET_IP101_PHY_H

#include "hw/core/qdev.h"

#define TYPE_IP101_PHY "ip101-phy"
OBJECT_DECLARE_SIMPLE_TYPE(IP101PhyState, IP101_PHY)

struct IP101PhyState {
    DeviceState parent_obj;

    /* MDIO address, set by the PHY's strap pins */
    uint8_t addr;

    /* A link partner is on the medium */
    bool medium_up;

    uint16_t bmcr;
    uint16_t anar;
    /* BMSR link status, latched low until BMSR is read */
    bool bmsr_link;
    bool an_complete;
    /* ANER page received, latched high until ANER is read */
    bool page_received;
    /* The link as the PHY last resolved it */
    bool link_up;
    bool speed_100;
    bool full_duplex;
    /* Page 16 registers */
    uint16_t pcr;
    uint16_t isr;
    uint16_t pscr16;
    uint16_t cssr_ctrl;
};

/* Whether a link partner is present on the medium. */
void ip101_phy_set_medium(IP101PhyState *s, bool up);

uint16_t ip101_phy_mdio_read(IP101PhyState *s, unsigned reg);
void ip101_phy_mdio_write(IP101PhyState *s, unsigned reg, uint16_t val);

/* The PHY passes frames between the MAC and the medium. */
bool ip101_phy_link_up(IP101PhyState *s);
/* The PHY returns the MAC's transmissions to its receive side. */
bool ip101_phy_loopback(IP101PhyState *s);
/* The PHY's MII is electrically isolated from the MAC. */
bool ip101_phy_isolated(IP101PhyState *s);

#endif
