/*
 * QTest testcase: a shutdown a guest requests stops every vCPU at once
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Two harts of the RISC-V virt machine run under TCG with icount: hart 1
 * counts in a word of RAM as fast as it can; hart 0 waits until it has
 * counted 1000, records the count it sees and, in the same instruction
 * block, writes PASS to the SiFive test finisher, which requests a
 * shutdown. With -action shutdown=pause the VM stops; hart 1 must not
 * have counted past what hart 0 saw, as the request stops every vCPU at
 * once, not only the one making it. Under icount the harts take turns on
 * one thread, so that this holds exactly.
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qobject/qdict.h"

#define PROGRAM     0x80000000u
#define COUNT       0x80001000u
#define SEEN        0x80001004u

/*
 *     csrr  t0, mhartid
 *     lui   t1, 0x80001        # COUNT
 *     bnez  t0, hart1
 *     lui   t2, 0x100          # the test finisher, 0x100000
 *     lui   t3, 0x5
 *     addi  t3, t3, 0x555      # FINISHER_PASS
 *     li    t6, 1000
 * wait:
 *     lw    t5, 0(t1)
 *     blt   t5, t6, wait
 *     lw    t5, 0(t1)
 *     sw    t5, 4(t1)          # SEEN
 *     sw    t3, 0(t2)          # the shutdown
 * halt:
 *     j     halt
 * hart1:
 *     lw    t5, 0(t1)
 *     addi  t5, t5, 1
 *     sw    t5, 0(t1)
 *     j     hart1
 */
static const uint32_t program[] = {
    0xf14022f3, 0x80001337, 0x02029663, 0x001003b7,
    0x00005e37, 0x555e0e13, 0x3e800f93, 0x00032f03,
    0xffff4ee3, 0x00032f03, 0x01e32223, 0x01c3a023,
    0x0000006f, 0x00032f03, 0x001f0f13, 0x01e32023,
    0xff5ff06f,
};

static void test_shutdown_pause(void)
{
    QTestState *qts;
    uint32_t count, seen;

    if (!qtest_has_accel("tcg")) {
        g_test_skip("TCG is not available");
        return;
    }
    qts = qtest_init("-machine virt -smp 2 -bios none -accel tcg "
                     "-icount shift=0,sleep=off -action shutdown=pause -S");
    for (int i = 0; i < ARRAY_SIZE(program); i++) {
        qtest_writel(qts, PROGRAM + 4 * i, program[i]);
    }
    qtest_qmp_assert_success(qts, "{ 'execute': 'cont' }");
    qtest_qmp_eventwait(qts, "SHUTDOWN");
    qtest_qmp_eventwait(qts, "STOP");

    count = qtest_readl(qts, COUNT);
    seen = qtest_readl(qts, SEEN);
    g_assert_cmpuint(seen, >=, 1000);
    g_assert_cmpuint(count, ==, seen);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/shutdown-stop/pause", test_shutdown_pause);
    return g_test_run();
}
