/*
 * QTest testcase for the ESP32-P4 boot ROM's checks of the image it boots:
 * the digest appended to it and Secure Boot's signature
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "libqtest.h"
#include "esp32p4-boot-utils.h"

#define CACHE_L2_BYPASS_CACHE_CONF      0x3ff10274
#define BYPASS_L2_CACHE_EN              (1u << 5)
#define CACHE_L2_CACHE_CACHESIZE_CONF   0x3ff10278

/*
 * The digest appended to an image: the ROM hashes the image in the SHA
 * accelerator block by block, writing each block while the last is
 * hashed, and compares. An image of 1 KiB passes; with its digest
 * altered the ROM reports the mismatch and, Secure Boot being off, boots
 * it all the same.
 */
/* [spec:nuos:req:emu.esp32p4.boot/test] */
static void test_image_digest(void)
{
    g_autofree char *good = write_flash_filled(prog_magic,
                                               ARRAY_SIZE(prog_magic), 256,
                                               false);
    g_autofree char *bad = write_flash_filled(prog_magic,
                                              ARRAY_SIZE(prog_magic), 256,
                                              true);
    Boot b = boot(0x08, good, NULL, NULL);

    await_magic(&b);
    await_output(&b, "load:0x4ff80000,len:0x418");
    await_output(&b, "entry 0x4ff80000");
    {
        g_autofree char *text = output(&b);

        g_assert_null(strstr(text, "SHA-256 comparison failed"));
    }
    boot_end(&b);

    b = boot(0x08, bad, NULL, NULL);
    await_output(&b, "SHA-256 comparison failed");
    await_output(&b, "Attempting to boot anyway");
    await_magic(&b);
    boot_end(&b);
    unlink(good);
    unlink(bad);
}

/*
 * A Secure Boot v2 signature block for the image put_image() makes of
 * prog_magic (no filler), from espsecure (esptool v5.5.0, "sign-data
 * --version 2", a throwaway ECDSA P-256 key): the curve point and the
 * signature, the block's CRC-32, and the key's digest
 * ("digest-sbv2-public-key"). The image is padded with 0xFF to 4 KiB, its
 * SHA-256 digest is in the block, and the block starts the 4 KiB sector
 * after it.
 */
static const uint8_t sb_ecdsa_point[64] = {
    0x33, 0x31, 0x5e, 0x7f, 0xb3, 0xf6, 0xed, 0x1a, 0x74, 0xc6, 0x34, 0xa8,
    0x37, 0x89, 0x9f, 0x91, 0xb6, 0xb6, 0x1f, 0x74, 0x09, 0x4c, 0xf0, 0xfe,
    0x2e, 0xca, 0x9b, 0xd3, 0x3b, 0xa8, 0x48, 0x7e, 0x44, 0x72, 0x74, 0xf7,
    0xe9, 0x9b, 0x2d, 0x40, 0x1b, 0xfb, 0x7f, 0x03, 0xd9, 0x58, 0xf9, 0xe2,
    0x53, 0xdf, 0x37, 0x93, 0x94, 0xbb, 0xbd, 0xfe, 0x32, 0x3f, 0xa6, 0xc9,
    0x1b, 0xd1, 0x4f, 0xb1,
};

static const uint8_t sb_ecdsa_signature[64] = {
    0xa9, 0xdf, 0x4a, 0xfb, 0x24, 0xcb, 0xf9, 0xb8, 0xbf, 0x7d, 0x0a, 0x9f,
    0x28, 0x5b, 0x65, 0x08, 0xfa, 0xd5, 0x18, 0x47, 0x8d, 0xa2, 0x39, 0x00,
    0x3f, 0xd4, 0xd8, 0xd4, 0xa7, 0x9b, 0x8a, 0xc2, 0xa3, 0x72, 0x07, 0x68,
    0x30, 0x61, 0xe1, 0xbf, 0x54, 0x81, 0xbb, 0xda, 0x4b, 0xa9, 0x81, 0xb3,
    0x8b, 0xa3, 0xc3, 0x8a, 0xb0, 0xe1, 0xd5, 0x6c, 0x51, 0x76, 0xb3, 0x0c,
    0xe8, 0x80, 0xd3, 0xa2,
};

static const uint8_t sb_ecdsa_key_digest[32] = {
    0x22, 0x95, 0x48, 0xe2, 0x78, 0x3e, 0xec, 0x95, 0x23, 0xb9, 0x80, 0xbf,
    0x1e, 0x6e, 0x75, 0xc7, 0xac, 0xcc, 0x82, 0x30, 0xfe, 0x3f, 0x59, 0xa2,
    0xc3, 0x0a, 0x3d, 0xc4, 0xda, 0x03, 0x70, 0x05,
};
#define SB_ECDSA_BLOCK_CRC 0xf3df122du

/* Put the signature sector after put_image()'s image at @offset. */
static void put_signature(uint8_t *flash, uint32_t offset)
{
    uint8_t *blk = flash + offset + 0x1000;
    gsize digest_len = 32;
    GChecksum *sha = g_checksum_new(G_CHECKSUM_SHA256);

    g_checksum_update(sha, flash + offset, 0x1000);
    memset(blk, 0, 1216);
    blk[0] = 0xe7;                      /* magic */
    blk[1] = 3;                         /* version: ECDSA */
    g_checksum_get_digest(sha, blk + 4, &digest_len);
    g_checksum_free(sha);
    blk[36] = 2;                        /* curve: P-256 */
    memcpy(blk + 37, sb_ecdsa_point, sizeof(sb_ecdsa_point));
    memcpy(blk + 101, sb_ecdsa_signature, sizeof(sb_ecdsa_signature));
    stl_le_p(blk + 1196, SB_ECDSA_BLOCK_CRC);
}

/*
 * Secure Boot (EFUSE_SECURE_BOOT_EN, a public key digest in BLOCK_KEY0 for
 * SECURE_BOOT_DIGEST0): the ROM passes its anti-glitch checks of the
 * random number generator, which reset the chip when they fail with
 * Secure Boot on, lists the key digest, and refuses the image, unsigned
 * or signed with the trusted key. The machine runs v3.0's ROM (ESP-ROM
 * esp32p4-eco5-20250430), whose eFuses say v3.0, and it has erratum
 * ROM-764: it stages the image it verifies at 0x4FF00000, inside the
 * 128 KiB of L2MEM the L2 cache takes from reset, which the CPU cannot
 * write, so the image's digest taken from the buffer does not match.
 * v3.1's ROM (ECO6), which has it fixed, is not published. The signature
 * verification itself, on images in RAM, is sb-signature-test's
 * (emu/tests/esp32p4-guest).
 */
/* [spec:nuos:req:emu.esp32p4.boot/test] */
static void test_secure_boot(void)
{
    uint32_t fuses[EFUSE_FILE_SIZE / 4] = { 0 };
    g_autofree uint8_t *image = g_malloc(FLASH_SIZE);
    g_autofree char *unsigned_flash = write_flash_filled(
        prog_magic, ARRAY_SIZE(prog_magic), 256, false);
    g_autofree char *signed_flash = NULL;
    g_autofree char *efuse = NULL;
    const char *flashes[2];
    Boot b;

    memset(image, 0xff, FLASH_SIZE);
    put_image(image, BOOTLOADER_OFFSET, prog_magic, ARRAY_SIZE(prog_magic),
              0, false);
    put_signature(image, BOOTLOADER_OFFSET);
    signed_flash = write_tmp("esp32p4-boot-XXXXXX.img", image, FLASH_SIZE);

    fuses[EFUSE_DATA(1)] = KEY_PURPOSE_0(KEY_PURPOSE_SECURE_BOOT_DIGEST0);
    fuses[EFUSE_DATA(2)] = SECURE_BOOT_EN;
    for (int i = 0; i < 8; i++) {
        fuses[EFUSE_KEY0(i)] = ldl_le_p(sb_ecdsa_key_digest + 4 * i);
    }
    efuse = write_efuse_words(fuses);
    flashes[0] = unsigned_flash;
    flashes[1] = signed_flash;
    for (int i = 0; i < 2; i++) {
        b = boot(0x08, flashes[i], efuse, NULL);
        await_output(&b, "rst:0x1 (POWERON),boot:0x9 (SPI_FAST_FLASH_BOOT)");
        await_output(&b, "Valid secure boot key blocks: 0");
        await_output(&b, "SHA-256 simple hash comparison failed.");
        await_output(&b, "secure boot verification failed");
        await_output(&b, "ets_main.c 372");
        expect_no_magic(&b);
        boot_end(&b);
    }
    unlink(unsigned_flash);
    unlink(signed_flash);
    unlink(efuse);
}

/*
 * The rest of the ROM's Secure Boot path, end to end: with the L2 cache
 * bypassed and its size cleared before the ROM runs (as a debugger can set
 * them while core 0 is halted at reset), L2MEM is all RAM and the ROM's
 * staging buffer at 0x4FF00000 holds what it copies, so the ROM checks the
 * key digest, verifies the ECDSA signature of the image signed with the
 * trusted key and boots it. A core reset (MWDT0's flash boot protection)
 * restores the L2 cache, and with it erratum ROM-764.
 */
/* [spec:nuos:req:emu.esp32p4.boot/test] */
static void test_secure_boot_l2_bypassed(void)
{
    uint32_t fuses[EFUSE_FILE_SIZE / 4] = { 0 };
    g_autofree uint8_t *image = g_malloc(FLASH_SIZE);
    g_autofree char *flash = NULL;
    g_autofree char *efuse = NULL;
    g_autofree char *drive = NULL;
    g_autofree char *fuse_args = NULL;
    Boot b;
    int fd;

    memset(image, 0xff, FLASH_SIZE);
    put_image(image, BOOTLOADER_OFFSET, prog_magic, ARRAY_SIZE(prog_magic),
              0, false);
    put_signature(image, BOOTLOADER_OFFSET);
    flash = write_tmp("esp32p4-boot-XXXXXX.img", image, FLASH_SIZE);
    fuses[EFUSE_DATA(1)] = KEY_PURPOSE_0(KEY_PURPOSE_SECURE_BOOT_DIGEST0);
    fuses[EFUSE_DATA(2)] = SECURE_BOOT_EN;
    for (int i = 0; i < 8; i++) {
        fuses[EFUSE_KEY0(i)] = ldl_le_p(sb_ecdsa_key_digest + 4 * i);
    }
    efuse = write_efuse_words(fuses);

    fd = g_file_open_tmp("esp32p4-boot-XXXXXX.log", &b.log, NULL);
    g_assert_cmpint(fd, >=, 0);
    close(fd);
    b.qts = qtest_initf("-M esp32p4,flash-size=4M,strap-mode=0x08 -S "
                        "-accel tcg -icount shift=0,sleep=off "
                        "-drive file=%s,if=mtd,format=raw,snapshot=on "
                        "-drive file=%s,if=none,format=raw,id=efuse,"
                        "snapshot=on -global driver=nvram.esp32p4.efuse,"
                        "property=drive,value=efuse "
                        "-chardev file,id=s0,path=%s -serial chardev:s0",
                        flash, efuse, b.log);
    qtest_writel(b.qts, CACHE_L2_BYPASS_CACHE_CONF, BYPASS_L2_CACHE_EN);
    qtest_writel(b.qts, CACHE_L2_CACHE_CACHESIZE_CONF, 0);
    qtest_qmp_assert_success(b.qts, "{'execute': 'cont'}");
    await_output(&b, "Valid secure boot key blocks: 0");
    await_output(&b, "ECDSA secure boot verification succeeded");
    await_output(&b, "load:0x4ff80000,len:0x18");
    await_magic(&b);
    await_output(&b, "rst:0x7 (HP_SYS_HP_WDT_RESET)");
    await_output(&b, "SHA-256 simple hash comparison failed.");
    boot_end(&b);
    unlink(flash);
    unlink(efuse);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/esp32p4/boot/image-digest", test_image_digest);
    qtest_add_func("/esp32p4/boot/secure-boot", test_secure_boot);
    qtest_add_func("/esp32p4/boot/secure-boot-l2-bypassed",
                   test_secure_boot_l2_bypassed);
    return g_test_run();
}
