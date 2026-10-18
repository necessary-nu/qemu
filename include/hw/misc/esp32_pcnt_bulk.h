/*
 * Espressif pulse counters taking a periodic output's edges in bulk
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ESP32_PCNT_BULK_H
#define HW_MISC_ESP32_PCNT_BULK_H

#include "hw/gpio/esp32_line.h"

/*
 * A pulse counter's signal input fed by an output's wave (esp32_line.h)
 * rather than its line: the edges the output makes after `pos`, the moment
 * the input has been brought to, as its wave gives them, inverted on the
 * way with `inv`. raw, raw_since and filt are the input's state at pos as
 * the counter keeps it edge by edge: the level at the input, the moment
 * from which a change waiting in the filter counts (when the input last
 * changed, unless the filter's count ran on from earlier), and the level
 * past the glitch filter, which a change at e reaches at due(e), e itself
 * without the filter. dmax bounds due(e) - e.
 *
 * Taken in bulk, the filtered edges come at due(e) of each edge e of the
 * wave while every pulse outlasts the filter (esp32_pcnt_feed_passes()),
 * once a change waiting in the filter at pos is due before the next edge
 * (esp32_pcnt_feed_aligned()): the counter then works out how many it has
 * had by a moment, and when the m-th comes, without taking them one by
 * one. A counter whose filter could run a count on through the next edge
 * (a dip its samples have not seen) takes that edge one by one first.
 */
typedef struct Esp32PcntFeed {
    Esp32LineWave *w;
    bool inv;
    int64_t pos;
    bool raw;
    bool filt;
    int64_t raw_since;
    bool filter;
    int64_t dmax;
    int64_t (*due)(void *opaque, int64_t e);
    void *opaque;
} Esp32PcntFeed;

int64_t esp32_pcnt_feed_due(const Esp32PcntFeed *f, int64_t e);
int64_t esp32_pcnt_feed_next_raw(const Esp32PcntFeed *f);
bool esp32_pcnt_feed_passes(const Esp32PcntFeed *f);
bool esp32_pcnt_feed_aligned(const Esp32PcntFeed *f);
uint64_t esp32_pcnt_feed_count(const Esp32PcntFeed *f, int64_t x);
int64_t esp32_pcnt_feed_time(const Esp32PcntFeed *f, uint64_t m);
void esp32_pcnt_feed_advance(Esp32PcntFeed *f, int64_t x);

/*
 * A unit's counter stepped by the filtered edges: by dn on the next, da on
 * the one after, alternately (each -1, 0 or 1), its 16-bit count wrapping.
 * Its watch points, as they match the count after a step: equal to a
 * value, at least one (the high limit) or at most one (the low limit).
 */
enum {
    ESP32_PCNT_CMP_EQ,
    ESP32_PCNT_CMP_GE,
    ESP32_PCNT_CMP_LE,
};

typedef struct Esp32PcntCmp {
    uint8_t kind;
    int16_t v;
} Esp32PcntCmp;

#define ESP32_PCNT_CMPS     8

uint64_t esp32_pcnt_first_match(int16_t c, int dn, int da,
                                const Esp32PcntCmp *cmp, unsigned n);
bool esp32_pcnt_count_edges(int16_t c, int dn, int da, uint64_t k,
                            int16_t *last, int16_t *before);

#endif
