/*
 * QEMU WAV audio driver
 *
 * Copyright (c) 2004-2005 Vassili Karpov (malc)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/error-report.h"
#include "qemu/audio.h"
#include "qom/object.h"

#include "audio_int.h"

#define TYPE_AUDIO_WAV "audio-wav"
OBJECT_DECLARE_SIMPLE_TYPE(AudioWav, AUDIO_WAV)

struct AudioWav {
    AudioMixengBackend parent_obj;
};

typedef struct WAVVoiceOut {
    HWVoiceOut hw;
    FILE *f;
    RateCtl rate;
    int total_samples;
} WAVVoiceOut;

/*
 * The capture stream: the samples of the file in-path names, at its own
 * rate, channels and format and paced by the virtual clock as the
 * playback stream is, then silence; silence throughout without one.
 */
typedef struct WAVVoiceIn {
    HWVoiceIn hw;
    FILE *f;
    RateCtl rate;
    /* Bytes left in the file's data chunk */
    uint32_t left;
} WAVVoiceIn;

static size_t wav_write_out(HWVoiceOut *hw, void *buf, size_t len)
{
    WAVVoiceOut *wav = (WAVVoiceOut *) hw;
    int64_t bytes = audio_rate_get_bytes(&wav->rate, &hw->info, len);
    assert(bytes % hw->info.bytes_per_frame == 0);

    if (bytes && fwrite(buf, bytes, 1, wav->f) != 1) {
        error_report("wav: fwrite of %" PRId64 " bytes failed: %s",
                     bytes, strerror(errno));
    }

    wav->total_samples += bytes / hw->info.bytes_per_frame;
    return bytes;
}

/* VICE code: Store number as little endian. */
static void le_store (uint8_t *buf, uint32_t val, int len)
{
    int i;
    for (i = 0; i < len; i++) {
        buf[i] = (uint8_t) (val & 0xff);
        val >>= 8;
    }
}

static int wav_init_out(HWVoiceOut *hw, struct audsettings *as)
{
    WAVVoiceOut *wav = (WAVVoiceOut *) hw;
    int bits16 = 0, stereo = 0;
    uint8_t hdr[] = {
        0x52, 0x49, 0x46, 0x46, 0x00, 0x00, 0x00, 0x00, 0x57, 0x41, 0x56,
        0x45, 0x66, 0x6d, 0x74, 0x20, 0x10, 0x00, 0x00, 0x00, 0x01, 0x00,
        0x02, 0x00, 0x44, 0xac, 0x00, 0x00, 0x10, 0xb1, 0x02, 0x00, 0x04,
        0x00, 0x10, 0x00, 0x64, 0x61, 0x74, 0x61, 0x00, 0x00, 0x00, 0x00
    };
    Audiodev *dev = hw->s->dev;
    AudiodevWavOptions *wopts = &dev->u.wav;
    struct audsettings wav_as = audiodev_to_audsettings(dev->u.wav.out);
    const char *wav_path = wopts->path ?: "qemu.wav";

    stereo = wav_as.nchannels == 2;
    switch (wav_as.fmt) {
    case AUDIO_FORMAT_S8:
    case AUDIO_FORMAT_U8:
        bits16 = 0;
        break;

    case AUDIO_FORMAT_S16:
    case AUDIO_FORMAT_U16:
        bits16 = 1;
        break;

    case AUDIO_FORMAT_S32:
    case AUDIO_FORMAT_U32:
        error_report("wav: WAVE files cannot handle 32-bit formats");
        return -1;

    case AUDIO_FORMAT_F32:
        error_report("wav: WAVE files cannot handle float formats");
        return -1;

    default:
        abort();
    }

    hdr[34] = bits16 ? 0x10 : 0x08;

    wav_as.big_endian = false;
    audio_pcm_init_info (&hw->info, &wav_as);

    hw->samples = 1024;
    le_store (hdr + 22, hw->info.nchannels, 2);
    le_store (hdr + 24, hw->info.freq, 4);
    le_store (hdr + 28, hw->info.freq << (bits16 + stereo), 4);
    le_store (hdr + 32, 1 << (bits16 + stereo), 2);

    wav->f = fopen(wav_path, "wb");
    if (!wav->f) {
        error_report("wav: failed to open wave file '%s': %s",
                     wav_path, strerror(errno));
        return -1;
    }

    if (fwrite (hdr, sizeof (hdr), 1, wav->f) != 1) {
        error_report("wav: failed to write header: %s", strerror(errno));
        return -1;
    }

    audio_rate_start(&wav->rate);
    return 0;
}

static void wav_fini_out (HWVoiceOut *hw)
{
    WAVVoiceOut *wav = (WAVVoiceOut *) hw;
    uint8_t rlen[4];
    uint8_t dlen[4];
    uint32_t datalen = wav->total_samples * hw->info.bytes_per_frame;
    uint32_t rifflen = datalen + 36;

    if (!wav->f) {
        return;
    }

    le_store (rlen, rifflen, 4);
    le_store (dlen, datalen, 4);

    if (fseek (wav->f, 4, SEEK_SET)) {
        error_report("wav: fseek to rlen failed: %s", strerror(errno));
        goto doclose;
    }
    if (fwrite (rlen, 4, 1, wav->f) != 1) {
        error_report("wav: failed to write rlen: %s", strerror(errno));
        goto doclose;
    }
    if (fseek (wav->f, 32, SEEK_CUR)) {
        error_report("wav: fseek to dlen failed: %s", strerror(errno));
        goto doclose;
    }
    if (fwrite (dlen, 4, 1, wav->f) != 1) {
        error_report("wav: failed to write dlen: %s", strerror(errno));
        goto doclose;
    }

 doclose:
    if (fclose (wav->f))  {
        error_report("wav: fclose failed: %s", strerror(errno));
    }
    wav->f = NULL;
}

static void wav_enable_out(HWVoiceOut *hw, bool enable)
{
    WAVVoiceOut *wav = (WAVVoiceOut *) hw;

    if (enable) {
        audio_rate_start(&wav->rate);
    }
}

static uint32_t le_load(const uint8_t *buf, int len)
{
    uint32_t v = 0;

    for (int i = len - 1; i >= 0; i--) {
        v = (v << 8) | buf[i];
    }
    return v;
}

/*
 * Read a RIFF WAVE file's header up to its data: PCM, 8-bit unsigned or
 * 16-bit signed samples, mono or stereo. Returns the data's length, or
 * -1 for a file it cannot take.
 */
static int64_t wav_read_header(FILE *f, const char *path,
                               struct audsettings *as)
{
    uint8_t hdr[12], chunk[8], fmt[16];
    bool have_fmt = false;

    if (fread(hdr, sizeof(hdr), 1, f) != 1 || memcmp(hdr, "RIFF", 4) ||
        memcmp(hdr + 8, "WAVE", 4)) {
        error_report("wav: '%s' is not a RIFF WAVE file", path);
        return -1;
    }
    while (fread(chunk, sizeof(chunk), 1, f) == 1) {
        uint32_t len = le_load(chunk + 4, 4);

        if (!memcmp(chunk, "fmt ", 4) && len >= sizeof(fmt)) {
            unsigned bits;

            if (fread(fmt, sizeof(fmt), 1, f) != 1) {
                break;
            }
            bits = le_load(fmt + 14, 2);
            if (le_load(fmt, 2) != 1 || (bits != 8 && bits != 16) ||
                le_load(fmt + 2, 2) < 1 || le_load(fmt + 2, 2) > 2) {
                error_report("wav: '%s' is not 8- or 16-bit PCM, mono or "
                             "stereo", path);
                return -1;
            }
            as->nchannels = le_load(fmt + 2, 2);
            as->freq = le_load(fmt + 4, 4);
            as->fmt = bits == 8 ? AUDIO_FORMAT_U8 : AUDIO_FORMAT_S16;
            as->big_endian = false;
            have_fmt = true;
            len -= sizeof(fmt);
        } else if (!memcmp(chunk, "data", 4)) {
            if (!have_fmt) {
                break;
            }
            return len;
        }
        /* Chunks are padded to an even length. */
        if (fseek(f, len + (len & 1), SEEK_CUR)) {
            break;
        }
    }
    error_report("wav: '%s' has no format and data chunks", path);
    return -1;
}

static int wav_init_in(HWVoiceIn *hw, struct audsettings *as)
{
    WAVVoiceIn *wav = (WAVVoiceIn *) hw;
    Audiodev *dev = hw->s->dev;
    AudiodevWavOptions *wopts = &dev->u.wav;
    struct audsettings wav_as = audiodev_to_audsettings(dev->u.wav.in);

    wav->f = NULL;
    wav->left = 0;
    if (wopts->in_path) {
        int64_t len;

        wav->f = fopen(wopts->in_path, "rb");
        if (!wav->f) {
            error_report("wav: failed to open wave file '%s': %s",
                         wopts->in_path, strerror(errno));
            return -1;
        }
        len = wav_read_header(wav->f, wopts->in_path, &wav_as);
        if (len < 0) {
            fclose(wav->f);
            wav->f = NULL;
            return -1;
        }
        wav->left = len;
    }
    audio_pcm_init_info(&hw->info, &wav_as);
    hw->samples = 1024;
    audio_rate_start(&wav->rate);
    return 0;
}

static void wav_fini_in(HWVoiceIn *hw)
{
    WAVVoiceIn *wav = (WAVVoiceIn *) hw;

    if (wav->f) {
        fclose(wav->f);
        wav->f = NULL;
    }
}

static size_t wav_read(HWVoiceIn *hw, void *buf, size_t size)
{
    WAVVoiceIn *wav = (WAVVoiceIn *) hw;
    size_t bytes = audio_rate_get_bytes(&wav->rate, &hw->info, size);
    size_t want = MIN(bytes, wav->left), got = 0;

    want -= want % hw->info.bytes_per_frame;
    if (wav->f && want) {
        got = fread(buf, 1, want, wav->f);
        got -= got % hw->info.bytes_per_frame;
        wav->left = got < want ? 0 : wav->left - got;
    }
    if (got < bytes) {
        audio_pcm_info_clear_buf(&hw->info, (uint8_t *)buf + got,
                                 (bytes - got) / hw->info.bytes_per_frame);
    }
    return bytes;
}

static void wav_enable_in(HWVoiceIn *hw, bool enable)
{
    WAVVoiceIn *wav = (WAVVoiceIn *) hw;

    if (enable) {
        audio_rate_start(&wav->rate);
    }
}

static void audio_wav_class_init(ObjectClass *klass, const void *data)
{
    AudioMixengBackendClass *k = AUDIO_MIXENG_BACKEND_CLASS(klass);

    k->max_voices_out = 1;
    k->max_voices_in = 1;
    k->voice_size_out = sizeof(WAVVoiceOut);
    k->voice_size_in = sizeof(WAVVoiceIn);

    k->init_out = wav_init_out;
    k->fini_out = wav_fini_out;
    k->write = wav_write_out;
    k->buffer_get_free = audio_generic_buffer_get_free;
    k->run_buffer_out = audio_generic_run_buffer_out;
    k->enable_out = wav_enable_out;

    k->init_in = wav_init_in;
    k->fini_in = wav_fini_in;
    k->read = wav_read;
    k->run_buffer_in = audio_generic_run_buffer_in;
    k->enable_in = wav_enable_in;
}

static const TypeInfo audio_types[] = {
    {
        .name = TYPE_AUDIO_WAV,
        .parent = TYPE_AUDIO_MIXENG_BACKEND,
        .instance_size = sizeof(AudioWav),
        .class_init = audio_wav_class_init,
    }
};

DEFINE_TYPES(audio_types)
module_obj(TYPE_AUDIO_WAV);
