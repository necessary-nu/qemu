/*
 * QTest testcase for the ESP32-P4 machine's lp-rom property: a user's LP
 * ROM image, refused unless it is the whole LP ROM, read by the LP core
 * alone, starting it at its entry, and kept through resets
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The images are stand-ins made here, not Espressif's LP ROM: a pattern
 * of words a test can recognise, with code at the entry, 0x80 in. The
 * qtest accelerator executes nothing, so what the LP core runs is for the
 * guest tests; here the image is read through the masters' views, as a
 * debugger attached to each sees it (HMP's xp on that CPU).
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qemu/cutils.h"
#include "qobject/qdict.h"
#include "libqtest.h"

#define LP_CPU                  "/machine/lp-cluster/cpu"
/* The CPUs' indices: HP core 0, HP core 1, the LP core. */
#define HP_CPU0_INDEX           0
#define LP_CPU_INDEX            2

#define LP_ROM                  0x50100000u
#define LP_ROM_SIZE             0x4000u
#define LP_ROM_ENTRY            (LP_ROM + 0x80)
#define LP_SRAM                 0x50108000u

#define LP_SYS                  0x50110000u
#define LP_CORE_BOOT_ADDR       (LP_SYS + 0x018)
#define BOOT_ADDR_HP_LP         (LP_SYS + 0x028)
#define LP_CPU_DBG_PC           (LP_SYS + 0x190)

#define PMU                     0x50115000u
#define PMU_LP_INT_CLR          (PMU + 0x180)
#define PMU_LP_CPU_PWR0         (PMU + 0x184)
#define PMU_LP_CPU_PWR1         (PMU + 0x188)
#define PMU_LP_CPU_PWR2         (PMU + 0x18c)
#define PMU_HP_LP_CPU_COMM      (PMU + 0x19c)
#define LP_CPU_SLP_STALL_EN     (1u << 29)
#define LP_CPU_SLP_RESET_EN     (1u << 30)
#define SLEEP_REQ               (1u << 31)
#define HP_TRIGGER_LP           (1u << 31)
#define LP_INT_HP_SW_TRIGGER    (1u << 31)
#define WAKE_LP_CPU             22

#define LPPERI_RESET_EN         0x50120008u
#define RST_EN_LP_CORE          (1u << 31)

/* The bus's answer to an address nothing decodes for the master. */
#define MEMTX_OK                0
#define MEMTX_DECODE_ERROR      2
/* The APMs' requester IDs: HP core 0, GDMA channel 0 as ICM ID 16. */
#define REQUESTER_HP_CORE0      0x8000u
#define REQUESTER_GDMA_CH0      (0x8000u | 0x4000u | (0u << 8) | 16u)

#define US                      1000ll

static uint32_t image_word(uint32_t off)
{
    return 0x1a000000u | off;
}

/*
 * A stand-in LP ROM image of @size bytes in a temporary file: each word
 * its offset, tagged, but for a few instructions at the entry, which
 * qtest does not execute but a reader of the image can tell.
 */
static char *make_image(size_t size)
{
    static const uint32_t entry[] = {
        0x501082b7, /* lui  t0, 0x50108 */
        0x5a500313, /* li   t1, 0x5a5 */
        0x1062a023, /* sw   t1, 0x100(t0) */
        0x0000006f, /* j    . */
    };
    g_autofree uint8_t *data = g_malloc0(size);
    GError *err = NULL;
    char *path = NULL;
    int fd;

    for (uint32_t off = 0; off + 4 <= size; off += 4) {
        stl_le_p(data + off, image_word(off));
    }
    for (int i = 0; i < ARRAY_SIZE(entry) && 0x80 + 4 * i + 4 <= size; i++) {
        stl_le_p(data + 0x80 + 4 * i, entry[i]);
    }
    fd = g_file_open_tmp("esp32p4-lp-rom-XXXXXX.bin", &path, &err);
    g_assert_no_error(err);
    close(fd);
    g_assert(g_file_set_contents(path, (char *)data, size, &err));
    g_assert_no_error(err);
    return path;
}

/*
 * Run QEMU outside qtest and return its exit status, for configurations
 * the machine must refuse before it starts.
 */
static int run_qemu(const char *args, char **err)
{
    g_autofree char *cmd = g_strdup_printf("%s -display none -serial none "
                                           "-monitor none %s",
                                           qtest_qemu_binary(NULL), args);
    g_auto(GStrv) argv = NULL;
    GError *gerr = NULL;
    int status;

    g_assert(g_shell_parse_argv(cmd, NULL, &argv, &gerr));
    g_assert(g_spawn_sync(NULL, argv, NULL, G_SPAWN_STDOUT_TO_DEV_NULL, NULL,
                          NULL, NULL, err, &status, &gerr));
    g_assert_no_error(gerr);
    return status;
}

static void assert_refused(const char *args, const char *message)
{
    g_autofree char *err = NULL;

    g_assert_cmpint(run_qemu(args, &err), !=, 0);
    if (!strstr(err, message)) {
        g_test_message("'%s' not in: %s", message, err);
        g_test_fail();
    }
}

/* An HMP command run with CPU @cpu as the monitor's current CPU. */
static char *hmp_on(QTestState *qts, int cpu, const char *cmd)
{
    QDict *rsp = qtest_qmp(qts, "{'execute': 'human-monitor-command', "
                           "'arguments': {'command-line': %s, "
                           "'cpu-index': %d}}", cmd, cpu);
    char *out;

    g_assert(qdict_haskey(rsp, "return"));
    out = g_strdup(qdict_get_str(rsp, "return"));
    qobject_unref(rsp);
    return out;
}

/*
 * Read @count units of @unit ('b', 'h' or 'w') at @addr as CPU @cpu sees
 * them, into @out; false if the CPU cannot reach the address.
 */
static bool cpu_read(QTestState *qts, int cpu, uint32_t addr, char unit,
                     int count, uint32_t *out)
{
    g_autofree char *cmd = g_strdup_printf("xp /%d%cx 0x%x", count, unit,
                                           addr);
    g_autofree char *text = hmp_on(qts, cpu, cmd);
    const char *p = text;
    int n = 0;

    if (strstr(text, "Cannot access memory")) {
        return false;
    }
    while (n < count) {
        const char *colon = strchr(p, ':');

        g_assert(colon);
        p = colon + 1;
        while (n < count) {
            unsigned long v;

            while (*p == ' ') {
                p++;
            }
            if (*p != '0') {
                break;
            }
            g_assert_cmpint(qemu_strtoul(p, &p, 16, &v), ==, 0);
            out[n++] = v;
        }
    }
    return true;
}

/*
 * An access by master @rid (an APM requester ID) through the bus, by the
 * permission managers' "probe" hook: its MemTxResult.
 */
static unsigned probe(QTestState *qts, uint32_t rid, char dir, uint32_t addr)
{
    g_autofree char *cmd = g_strdup_printf("%u 0 %c 0x%x 0x12345678", rid,
                                           dir, addr);
    unsigned result;
    QDict *rsp;

    rsp = qtest_qmp(qts, "{'execute': 'qom-set', 'arguments': "
                    "{'path': '/machine/pms', 'property': 'probe', "
                    "'value': %s}}", cmd);
    g_assert(qdict_haskey(rsp, "return"));
    qobject_unref(rsp);
    rsp = qtest_qmp(qts, "{'execute': 'qom-get', 'arguments': "
                    "{'path': '/machine/pms', 'property': 'probe'}}");
    g_assert(qdict_haskey(rsp, "return"));
    g_assert_cmpint(sscanf(qdict_get_str(rsp, "return"), "%u", &result), ==,
                    1);
    qobject_unref(rsp);
    return result;
}

/* A register of CPU @cpu, from "info registers". */
static uint32_t cpu_reg(QTestState *qts, int cpu, const char *name)
{
    g_autofree char *regs = hmp_on(qts, cpu, "info registers");
    g_autofree char *key = g_strdup_printf(" %s ", name);
    const char *p = strstr(regs, key);
    const char *end;
    unsigned long v;

    g_assert(p);
    g_assert_cmpint(qemu_strtoul(p + strlen(key), &end, 16, &v), ==, 0);
    return v;
}

/* The whole LP ROM as the LP core sees it equals the image @path. */
static void assert_lp_view(QTestState *qts, const char *path)
{
    g_autofree char *want = NULL;
    g_autofree uint32_t *got = g_new(uint32_t, LP_ROM_SIZE / 4);
    gsize len;

    g_assert(g_file_get_contents(path, &want, &len, NULL));
    g_assert_cmpuint(len, ==, LP_ROM_SIZE);
    g_assert(cpu_read(qts, LP_CPU_INDEX, LP_ROM, 'w', LP_ROM_SIZE / 4, got));
    for (uint32_t off = 0; off < LP_ROM_SIZE; off += 4) {
        g_assert_cmphex(got[off / 4], ==, ldl_le_p(want + off));
    }
}

static uint32_t lp_dbg_pc(QTestState *qts)
{
    return qtest_readl(qts, LP_CPU_DBG_PC);
}

/* Start the LP core through the PMU, as ESP-IDF's ulp_lp_core_run(). */
static void lp_wake(QTestState *qts)
{
    qtest_writel(qts, PMU_LP_CPU_PWR2, 1u << WAKE_LP_CPU);
    qtest_writel(qts, PMU_HP_LP_CPU_COMM, HP_TRIGGER_LP);
    g_assert(!qtest_qom_get_bool(qts, LP_CPU, "in-reset"));
}

/* Its sleep, stalled and reset, and the trigger's wakeup cleared. */
static void lp_sleep(QTestState *qts)
{
    qtest_writel(qts, PMU_LP_INT_CLR, LP_INT_HP_SW_TRIGGER);
    qtest_writel(qts, PMU_LP_CPU_PWR0, qtest_readl(qts, PMU_LP_CPU_PWR0) |
                 LP_CPU_SLP_STALL_EN | LP_CPU_SLP_RESET_EN);
    qtest_writel(qts, PMU_LP_CPU_PWR1, SLEEP_REQ);
    qtest_clock_step(qts, 10 * US);
    g_assert(qtest_qom_get_bool(qts, LP_CPU, "in-reset"));
}

/*
 * Anything but the whole LP ROM is refused before the machine starts: a
 * file of another size, empty or a byte short or over, half or twice the
 * LP ROM, and a file that is not there.
 */
/* [spec:nuos:req:emu.esp32p4.lp-rom/test] */
static void test_refused(void)
{
    static const size_t sizes[] = {
        0, LP_ROM_SIZE - 1, LP_ROM_SIZE + 1, LP_ROM_SIZE / 2,
        LP_ROM_SIZE * 2,
    };

    for (int i = 0; i < ARRAY_SIZE(sizes); i++) {
        g_autofree char *path = make_image(sizes[i]);
        g_autofree char *args = g_strdup_printf("-M esp32p4,lp-rom=%s",
                                                path);
        g_autofree char *message = g_strdup_printf("is %zu bytes; it must be "
                                                   "the whole LP ROM, the "
                                                   "16384 bytes from "
                                                   "0x50100000", sizes[i]);

        assert_refused(args, message);
        unlink(path);
    }
    assert_refused("-M esp32p4,lp-rom=/nonexistent/lp-rom.bin",
                   "cannot read the LP ROM image");
}

/*
 * The image as each master reaches it: the LP core reads all of it, in
 * bytes, halfwords and words; HP core 0 and a DMA master, which cannot
 * reach the LP ROM, get a bus error, the access logged, and a write from
 * them leaves the image as it is.
 */
/* [spec:nuos:req:emu.esp32p4.lp-rom/test] */
static void test_reads(void)
{
    g_autofree char *path = make_image(LP_ROM_SIZE);
    g_autofree char *log = NULL;
    g_autofree char *text = NULL;
    GError *err = NULL;
    QTestState *qts;
    uint32_t v[2];
    int fd = g_file_open_tmp("esp32p4-lp-rom-XXXXXX.log", &log, &err);
    QDict *r;

    g_assert_no_error(err);
    close(fd);
    qts = qtest_initf("-M esp32p4,lp-rom=%s -d guest_errors,unimp -D %s",
                      path, log);
    r = qtest_qmp(qts, "{'execute': 'qom-get', 'arguments': "
                  "{'path': '/machine', 'property': 'lp-rom'}}");
    g_assert_cmpstr(qdict_get_str(r, "return"), ==, path);
    qobject_unref(r);

    /* The LP core: all of it, and its narrower reads. */
    assert_lp_view(qts, path);
    g_assert(cpu_read(qts, LP_CPU_INDEX, LP_ROM + 0x3ffc, 'w', 1, v));
    g_assert_cmphex(v[0], ==, image_word(0x3ffc));
    g_assert(cpu_read(qts, LP_CPU_INDEX, LP_ROM + 0x1237, 'b', 1, v));
    g_assert_cmphex(v[0], ==, image_word(0x1234) >> 24);
    g_assert(cpu_read(qts, LP_CPU_INDEX, LP_ROM + 0x1232, 'h', 1, v));
    g_assert_cmphex(v[0], ==, image_word(0x1230) >> 16);
    g_assert(cpu_read(qts, LP_CPU_INDEX, LP_ROM_ENTRY, 'w', 2, v));
    g_assert_cmphex(v[0], ==, 0x501082b7);
    g_assert_cmphex(v[1], ==, 0x5a500313);

    /* HP core 0, and the bus every master but the LP core is on. */
    g_assert(!cpu_read(qts, HP_CPU0_INDEX, LP_ROM, 'w', 1, v));
    g_assert(!cpu_read(qts, HP_CPU0_INDEX, LP_ROM + 0x3ffc, 'w', 1, v));
    g_assert_cmpuint(probe(qts, REQUESTER_HP_CORE0, 'r', LP_ROM), ==,
                     MEMTX_DECODE_ERROR);
    g_assert_cmpuint(probe(qts, REQUESTER_GDMA_CH0, 'r', LP_ROM + 0x100), ==,
                     MEMTX_DECODE_ERROR);
    g_assert_cmpuint(probe(qts, REQUESTER_GDMA_CH0, 'w', LP_ROM + 0x100), ==,
                     MEMTX_DECODE_ERROR);
    qtest_writel(qts, LP_ROM + 0x200, 0x12345678);
    assert_lp_view(qts, path);
    qtest_quit(qts);

    g_assert(g_file_get_contents(log, &text, NULL, NULL));
    g_assert(strstr(text, "esp32p4.lp-rom: read of 4 bytes at 0x3ffc: only "
                    "the LP core reaches the LP ROM"));
    g_assert(strstr(text, "esp32p4.lp-rom: write of 4 bytes at 0x200: only "
                    "the LP core reaches the LP ROM"));
    g_assert(!strstr(text, "the LP ROM is unavailable"));
    g_assert(!strstr(text, "LP ROM's contents are not published"));
    unlink(log);
    unlink(path);
}

/*
 * The LP core's start with an image: at the LP ROM's entry, mtvec at its
 * vector table, whatever application boot address software has set; from
 * an LPPERI reset of the core too; and at a boot address elsewhere, past
 * the LP ROM as skip_lp_rom_boot has it.
 */
/* [spec:nuos:req:emu.esp32p4.lp-rom/test] */
static void test_boot(void)
{
    g_autofree char *path = make_image(LP_ROM_SIZE);
    QTestState *qts = qtest_initf("-M esp32p4,lp-rom=%s", path);

    g_assert(qtest_qom_get_bool(qts, LP_CPU, "in-reset"));
    g_assert_cmphex(qtest_readl(qts, LP_CORE_BOOT_ADDR), ==, LP_ROM);
    qtest_writel(qts, BOOT_ADDR_HP_LP, LP_SRAM + 0x80);
    lp_wake(qts);
    g_assert_cmphex(lp_dbg_pc(qts), ==, LP_ROM_ENTRY);
    g_assert_cmphex(cpu_reg(qts, LP_CPU_INDEX, "mtvec"), ==, LP_ROM | 1);

    /* LPPERI's reset of the core starts it at the LP ROM's entry again. */
    qtest_writel(qts, LPPERI_RESET_EN, RST_EN_LP_CORE);
    g_assert_cmphex(lp_dbg_pc(qts), ==, LP_ROM_ENTRY);
    lp_sleep(qts);

    /* skip_lp_rom_boot: the boot address the application's entry. */
    qtest_writel(qts, LP_CORE_BOOT_ADDR, LP_SRAM + 0x80);
    lp_wake(qts);
    g_assert_cmphex(lp_dbg_pc(qts), ==, LP_SRAM + 0x80);
    lp_sleep(qts);
    qtest_quit(qts);
    unlink(path);
}

/*
 * The image through a system reset, which also brings the boot address
 * back to the LP ROM: the LP core starts at its entry again and reads the
 * same image.
 */
/* [spec:nuos:req:emu.esp32p4.lp-rom/test] */
static void test_reset(void)
{
    g_autofree char *path = make_image(LP_ROM_SIZE);
    QTestState *qts = qtest_initf("-M esp32p4,lp-rom=%s", path);

    qtest_writel(qts, LP_CORE_BOOT_ADDR, LP_SRAM + 0x80);
    lp_wake(qts);
    lp_sleep(qts);
    qtest_system_reset(qts);
    g_assert_cmphex(qtest_readl(qts, LP_CORE_BOOT_ADDR), ==, LP_ROM);
    assert_lp_view(qts, path);
    lp_wake(qts);
    g_assert_cmphex(lp_dbg_pc(qts), ==, LP_ROM_ENTRY);
    qtest_quit(qts);
    unlink(path);
}

/*
 * Without the property the LP ROM is absent: the LP core reads 0 there and
 * starts at the application's boot address; HP core 0 and a DMA master
 * still cannot reach it, and get a bus error.
 */
/* [spec:nuos:req:emu.esp32p4.lp-rom/test] */
static void test_absent(void)
{
    QTestState *qts = qtest_init("-M esp32p4");
    uint32_t v[1];
    QDict *r;

    r = qtest_qmp(qts, "{'execute': 'qom-get', 'arguments': "
                  "{'path': '/machine', 'property': 'lp-rom'}}");
    g_assert_cmpstr(qdict_get_str(r, "return"), ==, "");
    qobject_unref(r);
    g_assert(cpu_read(qts, LP_CPU_INDEX, LP_ROM_ENTRY, 'w', 1, v));
    g_assert_cmphex(v[0], ==, 0);
    g_assert(!cpu_read(qts, HP_CPU0_INDEX, LP_ROM_ENTRY, 'w', 1, v));
    g_assert_cmpuint(probe(qts, REQUESTER_HP_CORE0, 'r', LP_ROM_ENTRY), ==,
                     MEMTX_DECODE_ERROR);
    g_assert_cmpuint(probe(qts, REQUESTER_GDMA_CH0, 'r', LP_ROM_ENTRY), ==,
                     MEMTX_DECODE_ERROR);
    g_assert_cmpuint(probe(qts, REQUESTER_GDMA_CH0, 'w', LP_ROM_ENTRY), ==,
                     MEMTX_DECODE_ERROR);
    qtest_writel(qts, BOOT_ADDR_HP_LP, LP_SRAM + 0x80);
    lp_wake(qts);
    g_assert_cmphex(lp_dbg_pc(qts), ==, LP_SRAM + 0x80);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("esp32p4/lp-rom/refused", test_refused);
    qtest_add_func("esp32p4/lp-rom/reads", test_reads);
    qtest_add_func("esp32p4/lp-rom/boot", test_boot);
    qtest_add_func("esp32p4/lp-rom/reset", test_reset);
    qtest_add_func("esp32p4/lp-rom/absent", test_absent);
    return g_test_run();
}
