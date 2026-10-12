/*
 * M25P80 datasheet chips: SPI NOR parts modelled as their datasheets give
 * them, with their status registers, command sets, busy times, suspend
 * and resume
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_BLOCK_M25P80_CHIP_H
#define HW_BLOCK_M25P80_CHIP_H

#include "qemu/notify.h"
#include "qemu/timer.h"
#include "system/memory.h"

typedef struct M25ChipInfo M25ChipInfo;

extern const M25ChipInfo m25chip_w25q16jv;
extern const M25ChipInfo m25chip_w25q32rv;
extern const M25ChipInfo m25chip_gd25q256e;

#define M25CHIP_PAGE_SIZE       256
#define M25CHIP_SECREGS         3
#define M25CHIP_SECREG_MAX      2048
#define M25CHIP_UID_MAX         16
#define M25CHIP_LOCKS           64

/* An internal program, erase or status register write cycle. */
typedef struct M25ChipOp {
    uint8_t kind;
    /* Where it acts: an array or security register address range. */
    uint32_t addr;
    uint32_t len;
    /* Its whole duration, the busy time spent on it, its current run. */
    int64_t total;
    int64_t done;
    int64_t started;
    /* A program's page buffer: 0xff where nothing was sent. */
    uint8_t data[M25CHIP_PAGE_SIZE];
    /* A status register write's new values and the registers it writes. */
    uint8_t sr[3];
    uint8_t sr_mask;
} M25ChipOp;

typedef struct M25Chip {
    /* Set by the owner before m25chip_realize(). */
    const M25ChipInfo *info;
    DeviceState *dev;
    uint8_t *storage;
    MemoryRegion *mem;
    /* Raised while `mem` is in ROMD mode, lowered while it is not. */
    qemu_irq mapped;
    BlockBackend *blk;
    bool max_timing;
    char *uid_prop;
    bool wp_level;

    QEMUTimer *op_timer;
    QEMUTimer *deaf_timer;
    /* Cells changed while the mapping was out of ROMD mode. */
    uint32_t dirty_lo;
    uint32_t dirty_hi;
    uint8_t uid[M25CHIP_UID_MAX];

    /* Status registers: as they read (volatile), and their stored bits. */
    uint8_t sr[3];
    uint8_t sr_nv[3];
    bool wel;
    bool vsr_we;
    bool reset_enable;
    bool qpi;
    bool four_byte;
    uint8_t ear;
    uint8_t read_params;
    uint8_t wrap;
    uint8_t cont_cmd;
    bool powered_down;
    int64_t deaf_until;
    uint8_t locks[M25CHIP_LOCKS];
    uint8_t secreg[M25CHIP_SECREGS][M25CHIP_SECREG_MAX];

    /* The cycle running, and one suspended. */
    M25ChipOp op;
    M25ChipOp susp;
    bool suspending;
    int64_t sus_at;
    int64_t resumed_at;

    /* The transaction in progress. */
    bool selected;
    uint8_t phase;
    uint8_t cmd;
    uint8_t addr_len;
    uint8_t addr_got;
    uint32_t addr;
    uint8_t mode;
    uint8_t skip;
    uint32_t count;
    uint8_t out[M25CHIP_UID_MAX];
    uint8_t out_len;
    uint8_t out_pos;
    uint8_t in[3];
    uint8_t pp[M25CHIP_PAGE_SIZE];
    uint8_t pp_set[M25CHIP_PAGE_SIZE / 8];
} M25Chip;

extern const VMStateDescription vmstate_m25chip;

bool m25chip_realize(M25Chip *c, Error **errp);
void m25chip_unrealize(M25Chip *c);
void m25chip_reset(M25Chip *c);
void m25chip_set_cs(M25Chip *c, bool high);
uint8_t m25chip_transfer(M25Chip *c, uint8_t tx);
int64_t m25chip_busy_until(M25Chip *c);
uint32_t m25chip_size(const M25ChipInfo *info);

#endif
