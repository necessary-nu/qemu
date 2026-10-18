#ifndef HW_FLASH_H
#define HW_FLASH_H

/* NOR flash devices */

#include "exec/hwaddr.h"
#include "qom/object.h"

/* pflash_cfi01.c */

#define TYPE_PFLASH_CFI01 "cfi.pflash01"
OBJECT_DECLARE_SIMPLE_TYPE(PFlashCFI01, PFLASH_CFI01)


PFlashCFI01 *pflash_cfi01_register(hwaddr base,
                                   const char *name,
                                   hwaddr size,
                                   BlockBackend *blk,
                                   uint32_t sector_len,
                                   int width,
                                   uint16_t id0, uint16_t id1,
                                   uint16_t id2, uint16_t id3,
                                   int be);
BlockBackend *pflash_cfi01_get_blk(PFlashCFI01 *fl);
MemoryRegion *pflash_cfi01_get_memory(PFlashCFI01 *fl);
void pflash_cfi01_legacy_drive(PFlashCFI01 *dev, DriveInfo *dinfo);

/* pflash_cfi02.c */

#define TYPE_PFLASH_CFI02 "cfi.pflash02"
OBJECT_DECLARE_SIMPLE_TYPE(PFlashCFI02, PFLASH_CFI02)


PFlashCFI02 *pflash_cfi02_register(hwaddr base,
                                   const char *name,
                                   hwaddr size,
                                   BlockBackend *blk,
                                   uint32_t sector_len,
                                   int nb_mappings,
                                   int width,
                                   uint16_t id0, uint16_t id1,
                                   uint16_t id2, uint16_t id3,
                                   uint16_t unlock_addr0,
                                   uint16_t unlock_addr1,
                                   int be);

/* m25p80.c */

#define TYPE_M25P80 "m25p80-generic"

BlockBackend *m25p80_get_blk(DeviceState *dev);

/*
 * When a datasheet part's BUSY bit will clear by itself (its cycle ends,
 * or a suspend takes effect); 0 when it is clear or the part has no busy
 * time.
 */
int64_t m25p80_busy_until(DeviceState *dev);

/*
 * Whether the part's IO3 pin is its HOLD# input, which pauses it while
 * low: with its quad enable clear.
 */
bool m25p80_hold_enabled(DeviceState *dev);

/*
 * For a board that clocks the part's lines itself: whether the part
 * drives its data outputs for the next byte the bus clocks (it is
 * selected, has taken a command that reads, and answers it), and the
 * read command a continuous read mode (M5-4 = 10b) keeps for its next
 * transaction, which then starts with the address (0: none).
 */
bool m25p80_drives_output(DeviceState *dev);
uint8_t m25p80_continuous_cmd(DeviceState *dev);

/*
 * For a board that clocks the part's lines itself: how many data lines
 * (1, 2 or 4) the part takes, or drives, its next byte on, as its command
 * and the phase of it the transaction has reached say. On one line the
 * part takes its input on IO0 (SI) and drives its output on IO1 (SO); on
 * two or four, IO0 to IO1 or IO3, the highest-numbered line carrying the
 * highest bit of each clock's.
 */
unsigned m25p80_io_lines(DeviceState *dev);

/*
 * The part's supply leaves its operating range: whatever program or erase
 * it was running is cut short. It comes back: the part starts again as at
 * power-up, a datasheet part also taking no command for its tVSL.
 */
void m25p80_power_off(DeviceState *dev);
void m25p80_power_on(DeviceState *dev);

#endif
