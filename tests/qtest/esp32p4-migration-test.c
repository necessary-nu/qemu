/*
 * QTest testcase: migrating a running ESP32-P4 with DMA and timers in flight
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * HP core 0 runs, under TCG with icount, rounds of a GDMA-AHB memory-to-
 * memory copy of 2 KiB in L2MEM: it fills the source with the round's
 * pattern, clears the destination, starts the copy and waits for it,
 * checks every word, and counts the round, and the SYSTIMER's alarms,
 * every 100 us, that it finds. The copy runs on the GDMA's bus rounds and
 * the alarms on the SYSTIMER's comparator, both timers, while the core
 * polls. The test migrates the running machine to a file many times over,
 * at random moments, each destination becoming the next source, and
 * checks each time:
 *   - the migration completes;
 *   - the destination loads the memory the source has once stopped, and
 *     the source, stopped, changes nothing afterwards: no timer of the
 *     source's runs once it has stopped for the migration;
 *   - the destination, resumed, goes on with its rounds without a word
 *     copied wrong, and with its alarms.
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "libqtest.h"
#include "qobject/qdict.h"
#include "elf.h"

#define LOAD_ADDR       0x4ff40000u
#define SRC             0x8ff60000u
#define DST             0x8ff61000u
#define WORDS           0x8ff62000u
#define LEN             0x800u
#define ROUNDS          (WORDS + 0)
#define ERROR_ROUND     (WORDS + 4)
#define ERROR_AT        (WORDS + 8)
#define ALARMS          (WORDS + 12)
#define MIGRATIONS      12

/*
 * Loaded at LOAD_ADDR, run from L2MEM through the caches; its data is in
 * L2MEM at the uncached alias (0x40000000 above L2MEM's address, which is
 * the DMA's): the source and destination of the copy, its progress (the
 * last round checked, the round and offset of a word copied wrong, the
 * alarms counted), and its two descriptors.
 *
 * _start:
 *     li    t0, 0x500c2064        # TIMG0_WDTWPROTECT
 *     li    t1, 0x50d83aa1
 *     sw    t1, 0(t0)
 *     sw    zero, -0x1c(t0)       # TIMG0_WDTCONFIG0: off, flash boot too
 *     li    t3, 0x400000          # WDT_CONF_UPDATE_EN
 *     sw    t3, -0x1c(t0)
 *     sw    zero, 0(t0)
 *     li    t0, 0x50116018        # LP_WDT_WPROTECT
 *     sw    t1, 0(t0)
 *     sw    zero, -0x18(t0)       # LP_WDT_CONFIG0: off, flash boot too
 *     sw    zero, 0(t0)
 *
 *     li    s4, 0x500e2000        # SYSTIMER
 *     li    t1, 0x40000640        # TARGET0_CONF: period mode, 100 us
 *     sw    t1, 0x34(s4)
 *     li    t1, 1
 *     sw    t1, 0x50(s4)          # COMP0_LOAD
 *     lw    t1, 0(s4)
 *     li    t2, 0x1000000         # CONF: TARGET0_WORK_EN
 *     or    t1, t1, t2
 *     sw    t1, 0(s4)
 *
 *     li    s1, 0x8ff60000        # src, through the uncached alias
 *     li    s2, 0x8ff61000        # dst
 *     li    s3, 0x8ff62000        # progress words; tx, rx
 *     li    s5, 0x40000000        # uncached alias - DMA address
 *     li    s0, 0
 * round:
 *     addi  s0, s0, 1
 *     slli  t1, s0, 16            # word i of round r: r << 16 | i
 *     li    t0, 0
 * fill:
 *     add   t2, s1, t0
 *     or    t3, t1, t0
 *     sw    t3, 0(t2)
 *     add   t2, s2, t0
 *     sw    zero, 0(t2)
 *     addi  t0, t0, 4
 *     li    t4, 2048
 *     blt   t0, t4, fill
 *
 *     li    t0, 0xc0800fff        # OWNER | SUC_EOF | length 2048 | size 4095
 *     sw    t0, 16(s3)
 *     sub   t2, s1, s5
 *     sw    t2, 20(s3)
 *     sw    zero, 24(s3)
 *     li    t0, 0x80000fff        # OWNER | size 4095
 *     sw    t0, 32(s3)
 *     sub   t2, s2, s5
 *     sw    t2, 36(s3)
 *     sw    zero, 40(s3)
 *
 *     li    t0, 0x50085000
 *     li    t1, 0xff
 *     sw    t1, 0x1c(t0)          # IN_INT_CLR(1)
 *     sw    t1, 0x4c(t0)          # OUT_INT_CLR(1)
 *     li    t1, 1
 *     sw    t1, 0x130(t0)         # IN_CONF0(1): reset
 *     sw    zero, 0x130(t0)
 *     sw    t1, 0x190(t0)         # OUT_CONF0(1): reset
 *     sw    zero, 0x190(t0)
 *     li    t1, 6
 *     sw    t1, 0x160(t0)         # IN_PERI_SEL(1): channel 1's dummy, 6
 *     sw    t1, 0x1c0(t0)         # OUT_PERI_SEL(1)
 *     li    t1, 0xd8              # MEM_TRANS_EN, bursts of 64 bytes
 *     sw    t1, 0x130(t0)
 *     li    t1, 0x32c             # EOF_MODE, AUTO_WRBACK, bursts of 64 bytes
 *     sw    t1, 0x190(t0)
 *     addi  t2, s3, 32
 *     sub   t2, t2, s5
 *     sw    t2, 0x3b0(t0)         # IN_LINK_ADDR(1): rx
 *     addi  t2, s3, 16
 *     sub   t2, t2, s5
 *     sw    t2, 0x3bc(t0)         # OUT_LINK_ADDR(1): tx
 *     li    t1, 5
 *     sw    t1, 0x140(t0)         # IN_LINK(1): start
 *     li    t1, 2
 *     sw    t1, 0x1a0(t0)         # OUT_LINK(1): start
 * wait:
 *     lw    t1, 0x10(t0)          # IN_INT_RAW(1)
 *     andi  t1, t1, 2             # IN_SUC_EOF
 *     beqz  t1, wait
 *
 *     slli  t1, s0, 16
 *     li    t0, 0
 * check:
 *     add   t2, s2, t0
 *     lw    t3, 0(t2)
 *     or    t5, t1, t0
 *     bne   t3, t5, bad
 *     addi  t0, t0, 4
 *     li    t4, 2048
 *     blt   t0, t4, check
 *     sw    s0, 0(s3)
 *     lw    t1, 0x68(s4)          # INT_RAW: TARGET0
 *     andi  t1, t1, 1
 *     beqz  t1, round
 *     sw    t1, 0x6c(s4)          # INT_CLR
 *     lw    t2, 12(s3)            # count the alarm
 *     addi  t2, t2, 1
 *     sw    t2, 12(s3)
 *     j     round
 * bad:
 *     sw    s0, 4(s3)
 *     sw    t0, 8(s3)
 * hang:
 *     j     hang
 */
static const uint32_t program[] = {
    0x500c22b7, 0x06428293, 0x50d84337, 0xaa130313,
    0x0062a023, 0xfe02a223, 0x00400e37, 0xffc2a223,
    0x0002a023, 0x501162b7, 0x01828293, 0x0062a023,
    0xfe02a423, 0x0002a023, 0x500e2a37, 0x40000337,
    0x64030313, 0x026a2a23, 0x00100313, 0x046a2823,
    0x000a2303, 0x010003b7, 0x00736333, 0x006a2023,
    0x8ff604b7, 0x8ff61937, 0x8ff629b7, 0x40000ab7,
    0x00000413, 0x00140413, 0x01041313, 0x00000293,
    0x005483b3, 0x00536e33, 0x01c3a023, 0x005903b3,
    0x0003a023, 0x00428293, 0x00001eb7, 0x800e8e93,
    0xffd2c0e3, 0xc08012b7, 0xfff28293, 0x0059a823,
    0x415483b3, 0x0079aa23, 0x0009ac23, 0x800012b7,
    0xfff28293, 0x0259a023, 0x415903b3, 0x0279a223,
    0x0209a423, 0x500852b7, 0x0ff00313, 0x0062ae23,
    0x0462a623, 0x00100313, 0x1262a823, 0x1202a823,
    0x1862a823, 0x1802a823, 0x00600313, 0x1662a023,
    0x1c62a023, 0x0d800313, 0x1262a823, 0x32c00313,
    0x1862a823, 0x02098393, 0x415383b3, 0x3a72a823,
    0x01098393, 0x415383b3, 0x3a72ae23, 0x00500313,
    0x1462a023, 0x00200313, 0x1a62a023, 0x0102a303,
    0x00237313, 0xfe030ce3, 0x01041313, 0x00000293,
    0x005903b3, 0x0003ae03, 0x00536f33, 0x03ee1c63,
    0x00428293, 0x00001eb7, 0x800e8e93, 0xffd2c2e3,
    0x0089a023, 0x068a2303, 0x00137313, 0xee030ce3,
    0x066a2623, 0x00c9a383, 0x00138393, 0x0079a623,
    0xee5ff06f, 0x0089a223, 0x0059a423, 0x0000006f,
};

/* An ELF of @program for -kernel, in a temporary file. */
static char *write_kernel(void)
{
    uint8_t elf[84 + sizeof(program)] = { 0 };
    GError *err = NULL;
    char *path = NULL;
    int fd;

    memcpy(elf, ELFMAG, SELFMAG);
    elf[EI_CLASS] = ELFCLASS32;
    elf[EI_DATA] = ELFDATA2LSB;
    elf[EI_VERSION] = EV_CURRENT;
    stw_le_p(elf + 16, ET_EXEC);
    stw_le_p(elf + 18, EM_RISCV);
    stl_le_p(elf + 20, EV_CURRENT);
    stl_le_p(elf + 24, LOAD_ADDR);      /* e_entry */
    stl_le_p(elf + 28, 52);             /* e_phoff */
    stw_le_p(elf + 40, 52);             /* e_ehsize */
    stw_le_p(elf + 42, 32);             /* e_phentsize */
    stw_le_p(elf + 44, 1);              /* e_phnum */
    stl_le_p(elf + 52, PT_LOAD);
    stl_le_p(elf + 56, 84);             /* p_offset */
    stl_le_p(elf + 60, LOAD_ADDR);      /* p_vaddr */
    stl_le_p(elf + 64, LOAD_ADDR);      /* p_paddr */
    stl_le_p(elf + 68, sizeof(program));
    stl_le_p(elf + 72, sizeof(program));
    stl_le_p(elf + 76, PF_R | PF_X);
    stl_le_p(elf + 80, 4);              /* p_align */
    for (size_t i = 0; i < ARRAY_SIZE(program); i++) {
        stl_le_p(elf + 84 + 4 * i, program[i]);
    }
    fd = g_file_open_tmp("esp32p4-migration-XXXXXX.elf", &path, &err);
    g_assert_no_error(err);
    g_assert(write(fd, elf, sizeof(elf)) == sizeof(elf));
    close(fd);
    return path;
}

/* The guest's memory: the copy's source and destination and its words. */
typedef struct Snapshot {
    uint8_t src[LEN];
    uint8_t dst[LEN];
    uint8_t words[0x30];
} Snapshot;

static void snapshot(QTestState *qts, Snapshot *s)
{
    qtest_memread(qts, SRC, s->src, LEN);
    qtest_memread(qts, DST, s->dst, LEN);
    qtest_memread(qts, WORDS, s->words, sizeof(s->words));
}

static const char *migration_status(QTestState *qts)
{
    QDict *rsp = qtest_qmp_assert_success_ref(qts,
                                    "{ 'execute': 'query-migrate' }");
    const char *st = g_intern_string(qdict_get_try_str(rsp, "status") ?:
                                     "none");

    qobject_unref(rsp);
    return st;
}

static void wait_migration(QTestState *qts)
{
    const char *st;

    while (true) {
        st = migration_status(qts);
        if (!strcmp(st, "completed")) {
            return;
        }
        g_assert_cmpstr(st, !=, "failed");
        g_usleep(10 * 1000);
    }
}

static QTestState *start(const char *kernel, bool incoming)
{
    return qtest_initf("-M esp32p4,flash-size=4M -accel tcg "
                       "-icount shift=0,sleep=off -kernel %s %s",
                       kernel, incoming ? "-incoming defer -S" : "");
}

static void expect_no_error(QTestState *qts)
{
    uint32_t round = qtest_readl(qts, ERROR_ROUND);

    if (round) {
        g_test_message("round %u copied the word at 0x%x wrong", round,
                       qtest_readl(qts, ERROR_AT));
    }
    g_assert_cmpuint(round, ==, 0);
}

/*
 * The guest runs on from the rounds and alarms it had counted in @from:
 * it counts more of each, and copies no word wrong.
 */
static void expect_progress(QTestState *qts, const uint8_t *from)
{
    uint32_t rounds = ldl_le_p(from), alarms = ldl_le_p(from + 12);

    for (int i = 0; qtest_readl(qts, ROUNDS) <= rounds + 10 ||
                    qtest_readl(qts, ALARMS) <= alarms + 10; i++) {
        expect_no_error(qts);
        g_assert_cmpint(i, <, 2000);
        g_usleep(5 * 1000);
    }
    expect_no_error(qts);
}

static void test_migrate_running(void)
{
    g_autofree char *kernel = NULL;
    g_autofree char *dir = NULL;
    g_autofree char *file = NULL;
    g_autofree char *uri = NULL;
    g_autofree Snapshot *stopped = NULL;
    g_autofree Snapshot *later = NULL;
    g_autofree Snapshot *loaded = NULL;
    QTestState *src, *dst;
    GError *err = NULL;

    if (!qtest_has_accel("tcg")) {
        g_test_skip("TCG is not available");
        return;
    }
    stopped = g_new(Snapshot, 1);
    later = g_new(Snapshot, 1);
    loaded = g_new0(Snapshot, 1);
    kernel = write_kernel();
    dir = g_dir_make_tmp("esp32p4-migration-XXXXXX", &err);
    g_assert_no_error(err);
    file = g_build_filename(dir, "state", NULL);
    uri = g_strdup_printf("file:%s", file);

    src = start(kernel, false);
    expect_progress(src, loaded->words);
    for (int i = 0; i < MIGRATIONS; i++) {
        g_usleep(g_test_rand_int_range(10, 150) * 1000);
        qtest_qmp_assert_success(src, "{ 'execute': 'migrate', "
                                 "'arguments': { 'uri': %s } }", uri);
        wait_migration(src);
        snapshot(src, stopped);

        dst = start(kernel, true);
        qtest_qmp_assert_success(dst, "{ 'execute': 'migrate-incoming', "
                                 "'arguments': { 'uri': %s } }", uri);
        wait_migration(dst);
        snapshot(dst, loaded);
        snapshot(src, later);
        g_assert(!memcmp(loaded, stopped, sizeof(*loaded)));
        g_assert(!memcmp(later, stopped, sizeof(*later)));
        qtest_quit(src);

        qtest_qmp_assert_success(dst, "{ 'execute': 'cont' }");
        expect_progress(dst, loaded->words);
        src = dst;
    }
    qtest_quit(src);

    unlink(file);
    rmdir(dir);
    unlink(kernel);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/esp32p4/migration/running", test_migrate_running);
    return g_test_run();
}
