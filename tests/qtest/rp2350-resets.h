/*
 * QTest helper for RP2350 machines: taking subsystems out of reset
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef TESTS_QTEST_RP2350_RESETS_H
#define TESTS_QTEST_RP2350_RESETS_H

#include "libqtest.h"

#define RP2350_RESETS_RESET_CLR  0x40023000
#define RP2350_RESETS_RESET_DONE 0x40020008
#define RP2350_RESETS_ALL        0x1fffffff
/* From deasserting a reset to RESET_DONE. */
#define RP2350_RESETS_RELEASE_NS 250

/*
 * Take `blocks` (RESETS bits) out of reset and wait for RESET_DONE, as
 * pico-sdk's unreset_block_mask_wait_blocking() does. Every subsystem
 * starts in reset, and its registers read as zero and ignore writes until
 * it leaves reset.
 */
static inline void rp2350_unreset(QTestState *qts, uint32_t blocks)
{
    qtest_writel(qts, RP2350_RESETS_RESET_CLR, blocks);
    qtest_clock_step(qts, RP2350_RESETS_RELEASE_NS);
    g_assert_cmphex(qtest_readl(qts, RP2350_RESETS_RESET_DONE) & blocks, ==,
                    blocks);
}

#endif
