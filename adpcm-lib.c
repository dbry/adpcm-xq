////////////////////////////////////////////////////////////////////////////
//                           **** ADPCM-XQ ****                           //
//                  Xtreme Quality ADPCM Encoder/Decoder                  //
//                    Copyright (c) 2024 David Bryant.                    //
//                          All Rights Reserved.                          //
//      Distributed under the BSD Software License (see license.txt)      //
////////////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <math.h>

#include "adpcm-lib.h"

/* This module encodes and decodes 4-bit ADPCM (DVI/IMA varient). ADPCM data is divided
 * into independently decodable blocks that can be relatively small. The most common
 * configuration is to store 505 samples into a 256 byte block, although other sizes are
 * permitted as long as the number of samples is one greater than a multiple of 8. When
 * multiple channels are present, they are interleaved in the data with an 8-sample
 * interval. 
 */

/********************************* 4-bit ADPCM encoder ********************************/

typedef uint64_t rms_error_t;     // best if "double" or "uint64_t", "float" okay in a pinch
#define MAX_RMS_ERROR UINT64_MAX
// typedef double rms_error_t;     // best if "double" or "uint64_t", "float" okay in a pinch
// #define MAX_RMS_ERROR DBL_MAX

#define CLIP(data, min, max) \
if ((data) > (max)) data = max; \
else if ((data) < (min)) data = min;

#define NIBBLE_TO_DELTA(n) ((n) < 8 ? (n) + 1 : 7 - (n))
#define DELTA_TO_NIBBLE(d) ((d) < 0 ? 7 - (d) : (d) - 1)

/* step table */
static const uint16_t step_table[89] = {
    7, 8, 9, 10, 11, 12, 13, 14,
    16, 17, 19, 21, 23, 25, 28, 31,
    34, 37, 41, 45, 50, 55, 60, 66,
    73, 80, 88, 97, 107, 118, 130, 143,
    157, 173, 190, 209, 230, 253, 279, 307,
    337, 371, 408, 449, 494, 544, 598, 658,
    724, 796, 876, 963, 1060, 1166, 1282, 1411,
    1552, 1707, 1878, 2066, 2272, 2499, 2749, 3024,
    3327, 3660, 4026, 4428, 4871, 5358, 5894, 6484,
    7132, 7845, 8630, 9493, 10442, 11487, 12635, 13899,
    15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794,
    32767
};

/* step index tables */
static const int index_table[] = {
    /* adpcm data size is 4 */
    -1, -1, -1, -1, 2, 4, 6, 8
};

static const int index_table_3bit[] = {
    /* adpcm data size is 3 */
    -1, -1, 1, 2
};

static const int index_table_5bit[] = {
    /* adpcm data size is 5 */
    -1, -1, -1, -1, -1, -1, -1, -1, 1, 2, 4, 6, 8, 10, 13, 16
};

struct adpcm_channel {
    const struct adpcm_context *cxt;        // read-only pointer back to context
    int32_t pcmdata;                        // current PCM value
    int32_t error, weight, history [2];     // for noise shaping
    int32_t shaping_weight;                 // for noise shaping
    int8_t index;                           // current index into step size table
};

struct adpcm_context {
    struct adpcm_channel channels [2];
    int num_channels, lookahead, noise_shaping;
    int static_shaping_weight;
};

/* Create ADPCM encoder context with given number of channels.
 * The returned pointer is used for subsequent calls. Note that
 * even though an ADPCM encoder could be set up to encode frames
 * independently, we use a context so that we can use previous
 * data to improve quality (this encoder might not be optimal
 * for encoding independent frames).
 */

void *adpcm_create_context (int num_channels, int lookahead, int noise_shaping)
{
    struct adpcm_context *pcnxt = malloc (sizeof (struct adpcm_context));
    int ch, i;

    memset (pcnxt, 0, sizeof (struct adpcm_context));
    pcnxt->channels [0].cxt = pcnxt->channels [1].cxt = pcnxt;
    pcnxt->noise_shaping = noise_shaping;
    pcnxt->static_shaping_weight = 1024;
    pcnxt->num_channels = num_channels;
    pcnxt->lookahead = lookahead;

    // we set the indicies to invalid values so that we always recalculate them
    // on at least the first frame (and every frame if the depth is sufficient)

    for (ch = 0; ch < num_channels; ++ch)
        pcnxt->channels [ch].index = -1;

    return pcnxt;
}

/* Set the shaping weight in range: -1.0 > weight >= 1.0.
 * Note that previously this was fixed to pure first-order (i.e., 1.0).
 * Also, values very close to -1.0 are not recommended because
 * of the high DC gain.
 */

void adpcm_set_shaping_weight (void *p, double shaping_weight)
{
    struct adpcm_context *pcnxt = (struct adpcm_context *) p;

    pcnxt->static_shaping_weight = (int) floor (shaping_weight * 1024.0 + 0.5);

    if (pcnxt->static_shaping_weight > 1024) pcnxt->static_shaping_weight = 1024;
    if (pcnxt->static_shaping_weight < -1023) pcnxt->static_shaping_weight = -1023;
}

/* Free the ADPCM encoder context.
 */

void adpcm_free_context (void *p)
{
    struct adpcm_context *pcnxt = (struct adpcm_context *) p;

    free (pcnxt);
}

// Apply noise-shaping to the supplied sample value using the shaping_weight
// and accumulated error term stored in the adpcm_channel structure. Note that
// the error term in the structure is updated, but won't be "correct" until the
// final re-quantized sample value is added to it (and of course we don't know
// that value yet).

static inline int32_t noise_shape (struct adpcm_channel *pchan, int32_t sample)
{
    int32_t temp = -((pchan->shaping_weight * pchan->error + 512) >> 10);

    if (pchan->shaping_weight < 0 && temp) {
        if (temp == pchan->error)
            temp = (temp < 0) ? temp + 1 : temp - 1;

        pchan->error = -sample;
        sample += temp;
    }
    else
        pchan->error = -(sample += temp);

    return sample;
}

static rms_error_t minimum_error (const struct adpcm_channel *pchan, int nch, int32_t csample, const int16_t *sample, int depth, int *best_nibble, rms_error_t max_error)
{
    int32_t delta = csample - pchan->pcmdata, csample2;
    struct adpcm_channel chan = *pchan;
    uint16_t step = step_table[chan.index];
    uint16_t trial_delta = (step >> 3);
    int nibble, nibble2;
    rms_error_t min_error;

    // this odd-looking code always generates the nibble value with the least error,
    // regardless of step size (which was not true previously)

    if (delta < 0) {
        int mag = ((-delta << 2) + (step & 3) + ((step & 1) << 1)) / step;
        nibble = 0x8 | (mag > 7 ? 7 : mag);
    }
    else {
        int mag = ((delta << 2) + (step & 3) + ((step & 1) << 1)) / step;
        nibble = mag > 7 ? 7 : mag;
    }

    if (nibble & 1) trial_delta += (step >> 2);
    if (nibble & 2) trial_delta += (step >> 1);
    if (nibble & 4) trial_delta += step;

    if (nibble & 8)
        chan.pcmdata -= trial_delta;
    else
        chan.pcmdata += trial_delta;

    CLIP(chan.pcmdata, -32768, 32767);
    if (best_nibble) *best_nibble = nibble;
    min_error = (rms_error_t) (chan.pcmdata - csample) * (chan.pcmdata - csample);

    // if we're at a leaf, or we're not at a leaf but have already exceeded the error limit, return
    if (!depth || min_error >= max_error)
        return min_error;

    // otherwise we execute that naively closest nibble and search deeper for improvement

    chan.index += index_table[nibble & 0x07];
    CLIP(chan.index, 0, 88);
    csample2 = sample [nch];

    if (chan.cxt->noise_shaping) {
        chan.error += chan.pcmdata;
        csample2 = noise_shape (&chan, csample2);
    }

    min_error += minimum_error (&chan, nch, csample2, sample + nch, depth - 1, NULL, max_error - min_error);

    // min_error is the error (from here to the leaf) for the naively closest nibble.
    // We may be able to improve on that by doing an alternative (not closest) nibble.

    for (nibble2 = 0; nibble2 <= 0xF; ++nibble2) {
        rms_error_t error, threshold;

        if (nibble2 == nibble)  // don't do the same value again
            continue;

        // we skip this trial if:
        // 1. we're not doing exhaustive search, and
        // 2. the trial value is not either 0x7 or 0x15 (i.e., delta +/- 8) and,
        // 3. the trial delta is greater than two away from the initial estimate...

        if (!(chan.cxt->lookahead & LOOKAHEAD_EXHAUSTIVE) && (~nibble2 & 0x7) &&
            abs (NIBBLE_TO_DELTA (nibble) - NIBBLE_TO_DELTA (nibble2)) > 2)
                continue;

        chan = *pchan;
        trial_delta = (step >> 3);

        if (nibble2 & 1) trial_delta += (step >> 2);
        if (nibble2 & 2) trial_delta += (step >> 1);
        if (nibble2 & 4) trial_delta += step;

        if (nibble2 & 8)
            chan.pcmdata -= trial_delta;
        else
            chan.pcmdata += trial_delta;

        CLIP(chan.pcmdata, -32768, 32767);

        error = (rms_error_t) (chan.pcmdata - csample) * (chan.pcmdata - csample);
        threshold = max_error < min_error ? max_error : min_error;

        if (error < threshold) {
            chan.index += index_table[nibble2 & 0x07];
            CLIP(chan.index, 0, 88);
            csample2 = sample [nch];

            if (chan.cxt->noise_shaping) {
                chan.error += chan.pcmdata;
                csample2 = noise_shape (&chan, csample2);
            }

            error += minimum_error (&chan, nch, csample2, sample + nch, depth - 1, NULL, threshold - error);

            if (error < min_error) {
                if (best_nibble) *best_nibble = nibble2;
                min_error = error;
            }
        }
    }

    return min_error;
}

static uint8_t encode_sample (struct adpcm_context *pcnxt, int ch, const int16_t *sample, int num_samples)
{
    struct adpcm_channel *pchan = pcnxt->channels + ch;
    int32_t csample = *sample;
    int depth = num_samples - 1, nibble;
    uint16_t step = step_table[pchan->index];
    uint16_t trial_delta = (step >> 3);

    if (pcnxt->noise_shaping == NOISE_SHAPING_DYNAMIC) {
        int32_t sam = (3 * pchan->history [0] - pchan->history [1]) >> 1;
        int32_t temp = csample - (((pchan->weight * sam) + 512) >> 10);

        if (sam && temp) pchan->weight -= (((sam ^ temp) >> 29) & 4) - 2;
        pchan->history [1] = pchan->history [0];
        pchan->history [0] = csample;

        pchan->shaping_weight = (pchan->weight < 256) ? 1024 : 1536 - (pchan->weight * 2);
    }
    else if (pcnxt->noise_shaping == NOISE_SHAPING_STATIC)
        pchan->shaping_weight = pcnxt->static_shaping_weight;

    if (pcnxt->noise_shaping)
        csample = noise_shape (pchan, csample); 

    if (depth > (pcnxt->lookahead & LOOKAHEAD_DEPTH))
        depth = (pcnxt->lookahead & LOOKAHEAD_DEPTH);

    minimum_error (pchan, pcnxt->num_channels, csample, sample, depth, &nibble, MAX_RMS_ERROR);

    if (nibble & 1) trial_delta += (step >> 2);
    if (nibble & 2) trial_delta += (step >> 1);
    if (nibble & 4) trial_delta += step;

    if (nibble & 8)
        pchan->pcmdata -= trial_delta;
    else
        pchan->pcmdata += trial_delta;

    pchan->index += index_table[nibble & 0x07];
    CLIP(pchan->index, 0, 88);
    CLIP(pchan->pcmdata, -32768, 32767);

    if (pcnxt->noise_shaping)
        pchan->error += pchan->pcmdata;

    return nibble;
}

static void encode_chunks (struct adpcm_context *pcnxt, uint8_t **outbuf, size_t *outbufsize, const int16_t **inbuf, int inbufcount)
{
    const int16_t *pcmbuf;
    int chunks, ch, i;

    chunks = (inbufcount - 1) / 8;
    *outbufsize += (chunks * 4) * pcnxt->num_channels;

    while (chunks--)
    {
        for (ch = 0; ch < pcnxt->num_channels; ch++)
        {
            pcmbuf = *inbuf + ch;

            for (i = 0; i < 4; i++) {
                **outbuf = encode_sample (pcnxt, ch, pcmbuf, chunks * 8 + (3 - i) * 2 + 2);
                pcmbuf += pcnxt->num_channels;
                **outbuf |= encode_sample (pcnxt, ch, pcmbuf, chunks * 8 + (3 - i) * 2 + 1) << 4;
                pcmbuf += pcnxt->num_channels;
                (*outbuf)++;
            }
        }

        *inbuf += 8 * pcnxt->num_channels;
    }
}

/* Encode a block of 16-bit PCM data into 4-bit ADPCM.
 *
 * Parameters:
 *  p               the context returned by adpcm_begin()
 *  outbuf          destination buffer
 *  outbufsize      pointer to variable where the number of bytes written
 *                   will be stored
 *  inbuf           source PCM samples
 *  inbufcount      number of composite PCM samples provided (note: this is
 *                   the total number of 16-bit samples divided by the number
 *                   of channels)
 *
 * Returns 1 (for success as there is no error checking)
 */

int adpcm_encode_block (void *p, uint8_t *outbuf, size_t *outbufsize, const int16_t *inbuf, int inbufcount)
{
    struct adpcm_context *pcnxt = (struct adpcm_context *) p;
    int depth = inbufcount - 1;
    int ch;

    *outbufsize = 0;

    if (!inbufcount)
        return 1;

    if (depth > (pcnxt->lookahead & LOOKAHEAD_DEPTH))
        depth = (pcnxt->lookahead & LOOKAHEAD_DEPTH);

    for (ch = 0; ch < pcnxt->num_channels; ch++)
        pcnxt->channels[ch].pcmdata = *inbuf++;

    // Use minimum_error() to find the optimum initial index if this is the first frame or
    // the lookahead depth is at least 3. Below that just using the value leftover from
    // the previous frame is better, and of course faster.

    if (pcnxt->channels [0].index < 0 || depth >= 3)
        for (ch = 0; ch < pcnxt->num_channels; ch++) {
            rms_error_t min_error = MAX_RMS_ERROR;
            rms_error_t error_per_index [89];
            int best_index;

            if (depth < 3)      // don't use a lower depth than 3 for this
                depth = 3;

            for (int tindex = 0; tindex <= 88; tindex++) {
                struct adpcm_channel chan = pcnxt->channels [ch];

                chan.index = tindex;
                chan.shaping_weight = 0;
                error_per_index [tindex] = minimum_error (&chan, pcnxt->num_channels, inbuf [ch], inbuf + ch, depth, NULL, MAX_RMS_ERROR);
            }

            // we use a 3-wide average window because the minimum_error() results can be noisy

            for (int tindex = 0; tindex <= 87; tindex++) {
                rms_error_t terror = error_per_index [tindex];

                if (tindex)
                    terror = (error_per_index [tindex - 1] + terror + error_per_index [tindex + 1]) / 3;

                if (terror < min_error) {
                    best_index = tindex;
                    min_error = terror;
                }
            }

            pcnxt->channels [ch].index = best_index;
        }

    for (ch = 0; ch < pcnxt->num_channels; ch++) {
        outbuf[0] = pcnxt->channels[ch].pcmdata;
        outbuf[1] = pcnxt->channels[ch].pcmdata >> 8;
        outbuf[2] = pcnxt->channels[ch].index;
        outbuf[3] = 0;

        outbuf += 4;
        *outbufsize += 4;
    }

    encode_chunks (pcnxt, &outbuf, outbufsize, &inbuf, inbufcount);

    return 1;
}

/********************************* 4-bit ADPCM decoder ********************************/

/* Decode the block of 4-bit ADPCM data into PCM. This requires no context because ADPCM
 * blocks are independently decodable. This assumes that a single entire block is always
 * decoded; it must be called multiple times for multiple blocks and cannot resume in the
 * middle of a block. Note that for all other bit depths, use adpcm_decode_block_ex().
 *
 * Parameters:
 *  outbuf          destination for interleaved PCM samples
 *  inbuf           source ADPCM block
 *  inbufsize       size of source ADPCM block
 *  channels        number of channels in block (must be determined from other context)
 *
 * Returns number of converted composite samples (total samples divided by number of channels)
 */ 

int adpcm_decode_block (int16_t *outbuf, const uint8_t *inbuf, size_t inbufsize, int channels)
{
    int ch, samples = 1, chunks;
    int32_t pcmdata[2];
    int8_t index[2];

    if (inbufsize < (uint32_t) channels * 4)
        return 0;

    for (ch = 0; ch < channels; ch++) {
        *outbuf++ = pcmdata[ch] = (int16_t) (inbuf [0] | (inbuf [1] << 8));
        index[ch] = inbuf [2];

        if (index [ch] < 0 || index [ch] > 88 || inbuf [3])     // sanitize the input a little...
            return 0;

        inbufsize -= 4;
        inbuf += 4;
    }

    chunks = inbufsize / (channels * 4);
    samples += chunks * 8;

    while (chunks--) {
        int ch, i;

        for (ch = 0; ch < channels; ++ch) {

            for (i = 0; i < 4; ++i) {
                uint16_t step = step_table [index [ch]], delta = step >> 3;

                if (*inbuf & 1) delta += (step >> 2);
                if (*inbuf & 2) delta += (step >> 1);
                if (*inbuf & 4) delta += step;

                if (*inbuf & 8)
                    pcmdata[ch] -= delta;
                else
                    pcmdata[ch] += delta;

                index[ch] += index_table [*inbuf & 0x7];
                CLIP(index[ch], 0, 88);
                CLIP(pcmdata[ch], -32768, 32767);
                outbuf [i * 2 * channels] = pcmdata[ch];

                step = step_table [index [ch]]; delta = step >> 3;

                if (*inbuf & 0x10) delta += (step >> 2);
                if (*inbuf & 0x20) delta += (step >> 1);
                if (*inbuf & 0x40) delta += step;

                if (*inbuf & 0x80)
                    pcmdata[ch] -= delta;
                else
                    pcmdata[ch] += delta;
                
                index[ch] += index_table [(*inbuf >> 4) & 0x7];
                CLIP(index[ch], 0, 88);
                CLIP(pcmdata[ch], -32768, 32767);
                outbuf [(i * 2 + 1) * channels] = pcmdata[ch];

                inbuf++;
            }

            outbuf++;
        }

        outbuf += channels * 7;
    }

    return samples;
}

/********************************* 4-bit ADPCM decoder ********************************/

/* Decode the block of ADPCM data, with from 2 to 5 bits per sample, into 16-bit PCM.
 * This requires no context because ADPCM blocks are independently decodable. This assumes
 * that a single entire block is always decoded; it must be called multiple times for
 * multiple blocks and cannot resume in the middle of a block.
 *
 * Parameters:
 *  outbuf          destination for interleaved PCM samples
 *  inbuf           source ADPCM block
 *  inbufsize       size of source ADPCM block
 *  channels        number of channels in block (must be determined from other context)
 *  bps             bits per ADPCM sample (2-5, must be determined from other context)
 *
 * Returns number of converted composite samples (total samples divided by number of channels)
 */ 

int adpcm_decode_block_ex (int16_t *outbuf, const uint8_t *inbuf, size_t inbufsize, int channels, int bps)
{
    int samples = 1, ch;
    int32_t pcmdata[2];
    int8_t index[2];

    if (bps == 4)
        return adpcm_decode_block (outbuf, inbuf, inbufsize, channels);

    if (bps < 2 || bps > 5 || inbufsize < (uint32_t) channels * 4)
        return 0;

    for (ch = 0; ch < channels; ch++) {
        *outbuf++ = pcmdata[ch] = (int16_t) (inbuf [0] | (inbuf [1] << 8));
        index[ch] = inbuf [2];

        if (index [ch] < 0 || index [ch] > 88 || inbuf [3])     // sanitize the input a little...
            return 0;

        inbufsize -= 4;
        inbuf += 4;
    }

    if (!inbufsize || (inbufsize % (channels * 4)))             // extra clean
        return samples;

    samples += inbufsize / channels * 8 / bps;

    switch (bps) {
        case 2:
            for (ch = 0; ch < channels; ++ch) {
                int shiftbits = 0, numbits = 0, i, j;

                for (j = i = 0; i < samples - 1; ++i) {
                    uint16_t step = step_table [index [ch]];

                    if (numbits < bps) {
                        shiftbits |= inbuf [(j & ~3) * channels + (ch * 4) + (j & 3)] << numbits;
                        numbits += 8;
                        j++;
                    }

                    if (shiftbits & 2)
                        pcmdata[ch] -= step * (shiftbits & 1) + (step >> 1);
                    else
                        pcmdata[ch] += step * (shiftbits & 1) + (step >> 1);

                    index[ch] += (shiftbits & 1) * 3 - 1;
                    shiftbits >>= bps;
                    numbits -= bps;

                    CLIP(index[ch], 0, 88);
                    CLIP(pcmdata[ch], -32768, 32767);
                    outbuf [i * channels + ch] = pcmdata[ch];
                }
            }

            break;

        case 3:
            for (ch = 0; ch < channels; ++ch) {
                int shiftbits = 0, numbits = 0, i, j;

                for (j = i = 0; i < samples - 1; ++i) {
                    uint16_t step = step_table [index [ch]], delta = step >> 2;

                    if (numbits < bps) {
                        shiftbits |= inbuf [(j & ~3) * channels + (ch * 4) + (j & 3)] << numbits;
                        numbits += 8;
                        j++;
                    }

                    if (shiftbits & 1) delta += (step >> 1);
                    if (shiftbits & 2) delta += step;

                    if (shiftbits & 4)
                        pcmdata[ch] -= delta;
                    else
                        pcmdata[ch] += delta;

                    index[ch] += index_table_3bit [shiftbits & 0x3];
                    shiftbits >>= bps;
                    numbits -= bps;

                    CLIP(index[ch], 0, 88);
                    CLIP(pcmdata[ch], -32768, 32767);
                    outbuf [i * channels + ch] = pcmdata[ch];
                }
            }

            break;

        case 5:
            for (ch = 0; ch < channels; ++ch) {
                int shiftbits = 0, numbits = 0, i, j;

                for (j = i = 0; i < samples - 1; ++i) {
                    uint16_t step = step_table [index [ch]], delta = step >> 4;

                    if (numbits < bps) {
                        shiftbits |= inbuf [(j & ~3) * channels + (ch * 4) + (j & 3)] << numbits;
                        numbits += 8;
                        j++;
                    }

                    if (shiftbits & 1) delta += (step >> 3);
                    if (shiftbits & 2) delta += (step >> 2);
                    if (shiftbits & 4) delta += (step >> 1);
                    if (shiftbits & 8) delta += step;

                    if (shiftbits & 0x10)
                        pcmdata[ch] -= delta;
                    else
                        pcmdata[ch] += delta;

                    index[ch] += index_table_5bit [shiftbits & 0xf];
                    shiftbits >>= bps;
                    numbits -= bps;

                    CLIP(index[ch], 0, 88);
                    CLIP(pcmdata[ch], -32768, 32767);
                    outbuf [i * channels + ch] = pcmdata[ch];
                }
            }

            break;

        default:
            return 0;
    }

    return samples;
}
