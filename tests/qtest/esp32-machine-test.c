/*
 * QTest testcase for what the ESP32 machine logs as it is built
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "libqtest.h"

#define IRAM_BASE   0x40080000
#define EM_XTENSA   94
#define PT_LOAD     1

/* An ELF whose one instruction, at @addr, branches to itself. */
static char *write_elf(uint32_t addr)
{
    uint8_t elf[52 + 32 + 4] = { 0 };
    GError *err = NULL;
    char *path = NULL;
    int fd = g_file_open_tmp("esp32-test-XXXXXX.elf", &path, &err);

    memcpy(elf, "\x7f" "ELF", 4);
    elf[4] = 1;                         /* ELFCLASS32 */
    elf[5] = 1;                         /* ELFDATA2LSB */
    elf[6] = 1;                         /* EV_CURRENT */
    stw_le_p(elf + 16, 2);              /* ET_EXEC */
    stw_le_p(elf + 18, EM_XTENSA);
    stl_le_p(elf + 20, 1);              /* e_version */
    stl_le_p(elf + 24, addr);           /* e_entry */
    stl_le_p(elf + 28, 52);             /* e_phoff */
    stw_le_p(elf + 40, 52);             /* e_ehsize */
    stw_le_p(elf + 42, 32);             /* e_phentsize */
    stw_le_p(elf + 44, 1);              /* e_phnum */
    stw_le_p(elf + 46, 40);             /* e_shentsize */
    stl_le_p(elf + 52, PT_LOAD);
    stl_le_p(elf + 56, 84);             /* p_offset */
    stl_le_p(elf + 60, addr);           /* p_vaddr */
    stl_le_p(elf + 64, addr);           /* p_paddr */
    stl_le_p(elf + 68, 4);              /* p_filesz */
    stl_le_p(elf + 72, 4);              /* p_memsz */
    stl_le_p(elf + 76, 5);              /* PF_R | PF_X */
    stl_le_p(elf + 80, 4);              /* p_align */
    stl_le_p(elf + 84, 0x00ffff06);     /* j . */

    g_assert_no_error(err);
    g_assert_cmpint(write(fd, elf, sizeof(elf)), ==, sizeof(elf));
    close(fd);
    return path;
}

/*
 * What QEMU, started stopped (-S) with @args, writes to stderr as it
 * builds the machine: the monitor, on stdin, quits it once it has.
 */
static char *startup_stderr(const char *args)
{
    g_autofree char *cmd = g_strdup_printf("%s -S -display none -serial none "
                                           "-monitor stdio %s",
                                           qtest_qemu_binary(NULL), args);
    g_auto(GStrv) argv = NULL;
    GString *err = g_string_new(NULL);
    GError *gerr = NULL;
    int in_fd, err_fd, status;
    char buf[256];
    ssize_t n;
    GPid pid;

    g_assert(g_shell_parse_argv(cmd, NULL, &argv, &gerr));
    g_assert(g_spawn_async_with_pipes(NULL, argv, NULL,
                                      G_SPAWN_DO_NOT_REAP_CHILD |
                                      G_SPAWN_STDOUT_TO_DEV_NULL,
                                      NULL, NULL, &pid, &in_fd, NULL,
                                      &err_fd, &gerr));
    g_assert_no_error(gerr);
    g_assert_cmpint(write(in_fd, "quit\n", 5), ==, 5);
    close(in_fd);
    while ((n = read(err_fd, buf, sizeof(buf))) > 0) {
        g_string_append_len(err, buf, n);
    }
    close(err_fd);
    g_assert_cmpint(waitpid(pid, &status, 0), ==, pid);
    g_spawn_close_pid(pid);
    return g_string_free(err, false);
}

/*
 * Building the machine logs nothing as a guest error or an unimplemented
 * access, so that -d's log starts with the guest's own; nor does an empty
 * guest, a branch to itself, run on TCG and reset.
 */
static void test_quiet(void)
{
    g_autofree char *elf = write_elf(IRAM_BASE);

    for (int tcg = 0; tcg < 2; tcg++) {
        g_autofree char *log = NULL;
        g_autofree char *text = NULL;
        QTestState *qts;
        int fd;

        fd = g_file_open_tmp("esp32-test-XXXXXX.log", &log, NULL);
        g_assert_cmpint(fd, >=, 0);
        close(fd);
        qts = qtest_initf("-M esp32 -nic none -kernel %s %s "
                          "-d guest_errors,unimp -D %s", elf,
                          tcg ? "-accel tcg" : "", log);
        if (tcg) {
            g_usleep(100 * 1000);
        }
        qtest_system_reset(qts);
        if (tcg) {
            g_usleep(100 * 1000);
        }
        qtest_quit(qts);

        g_assert(g_file_get_contents(log, &text, NULL, NULL));
        if (*text) {
            g_test_message("%s logged: %s", tcg ? "TCG" : "qtest", text);
            g_test_fail();
        }
        unlink(log);
    }
    unlink(elf);
}

/*
 * A user booting the ROM without flash (-drive if=mtd) is warned that it
 * has nothing to boot from; not one whose -kernel ELF replaces it.
 */
static void test_no_flash(void)
{
    g_autofree char *elf = write_elf(IRAM_BASE);
    g_autofree char *with_kernel = g_strdup_printf("-M esp32 -nic none "
                                                   "-kernel %s", elf);
    g_autofree char *err = startup_stderr("-M esp32 -nic none");
    g_autofree char *err_kernel = startup_stderr(with_kernel);

    g_assert(strstr(err, "warning: esp32: no flash is configured"));
    g_assert(!strstr(err_kernel, "no flash"));
    unlink(elf);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/esp32/machine/quiet", test_quiet);
    qtest_add_func("/esp32/machine/no-flash", test_no_flash);

    return g_test_run();
}
