#ifndef ARDUINO

////////////////////////////////////////////////////////////////////////////
//                           **** ADPCM-XQ ****                           //
//                  Xtreme Quality ADPCM Encoder/Decoder                  //
//                    Copyright (c) 2024 David Bryant.                    //
//                          All Rights Reserved.                          //
//      Distributed under the BSD Software License (see license.txt)      //
////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>

#include "adpcm-lib.h"

// This runtime macro is not strictly needed because the code is endian-safe,
// but including it improves performance on little-endian systems because we
// can avoid a couple loops through the audio.
#define IS_BIG_ENDIAN (*(uint16_t *)"\0\xff" < 0x0100)

static const char *sign_on = "\n"
" ADPCM-XQ   Xtreme Quality IMA-ADPCM WAV Encoder / Decoder   Version 0.5\n"
" Copyright (c) 2024 David Bryant. All Rights Reserved.\n\n";

static const char *usage =
" Usage:     ADPCM-XQ [-options] infile.wav outfile.wav\n\n"
" Operation: conversion is performed based on the type of the infile\n"
"          (either encode 16-bit PCM to 4-bit IMA-ADPCM or decode back)\n\n"
" Options:  -[0-16]= encode lookahead samples (default = 3, max = 16)\n"
"           -b<n>  = override auto block size, 2^n bytes (n = 8-15)\n"
"           -d     = decode only (fail on WAV file already PCM)\n"
"           -e     = encode only (fail on WAV file already ADPCM)\n"
"           -f     = encode flat noise (no noise shaping, aka -s0.0)\n"
"           -h     = display this help message\n"
"           -n     = measure and report quantization noise\n"
"           -q     = quiet mode (display errors only)\n"
"           -r     = raw output (little-endian, no WAV header written)\n"
"           -s<n>  = override default noise shaping, (-1.0 < n <= 1.0)\n"
"           -v     = verbose (display lots of info)\n"
"           -w<n>  = override default 4-bit ADPCM width (2 <= n <= 5)\n"
"           -x     = exhaustive search (old behavior, very slow at depth)\n"
"           -y     = overwrite outfile if it exists\n\n"
" Web:       Visit www.github.com/dbry/adpcm-xq for latest version and info\n\n";

#define ADPCM_FLAG_NOISE_SHAPING    0x1
#define ADPCM_FLAG_RAW_OUTPUT       0x2
#define ADPCM_FLAG_MEASURE_NOISE    0x4

static double strtod_hexfree (const char *nptr, char **endptr);
static int adpcm_converter (char *infilename, char *outfilename);
static int verbosity = 0, decode_only = 0, encode_only = 0, flags = ADPCM_FLAG_NOISE_SHAPING;
static int lookahead = 3, blocksize_pow2 = 0, encode_width_bits = 4;
static double static_shaping_weight = 0.0;

int main (int argc, char **argv)
{
    int overwrite = 0, asked_help = 0;
    char *infilename = NULL, *outfilename = NULL;
    FILE *outfile;

    // if the name of the executable ends in "encoder" or "decoder", just do that function
    encode_only = argc && strstr (argv [0], "encoder") && strlen (strstr (argv [0], "encoder")) == strlen ("encoder");
    decode_only = argc && strstr (argv [0], "decoder") && strlen (strstr (argv [0], "decoder")) == strlen ("decoder");

    // loop through command-line arguments

    while (--argc) {
#if defined (_WIN32)
        if ((**++argv == '-' || **argv == '/') && (*argv)[1])
#else
        if ((**++argv == '-') && (*argv)[1])
#endif
            while (*++*argv)
                switch (**argv) {

                    case '0': case '1': case '2':
                    case '3': case '4': case '5':
                    case '6': case '7': case '8':
                    case '9':
                        lookahead = (strtol (*argv, argv, 10) & LOOKAHEAD_DEPTH) | (lookahead & LOOKAHEAD_EXHAUSTIVE);

                        if ((lookahead & LOOKAHEAD_DEPTH) > 16) {
                            fprintf (stderr, "\nlookahead must be 0 to 16!\n");
                            return -1;
                        }

                        --*argv;
                        break;

                    case 'B': case 'b':
                        blocksize_pow2 = strtol (++*argv, argv, 10);

                        if (blocksize_pow2 < 8 || blocksize_pow2 > 15) {
                            fprintf (stderr, "\nblock size power must be 8 to 15!\n");
                            return -1;
                        }

                        --*argv;
                        break;

                    case 'D': case 'd':
                        decode_only = 1;
                        break;

                    case 'E': case 'e':
                        encode_only = 1;
                        break;

                    case 'F': case 'f':
                        flags &= ~ADPCM_FLAG_NOISE_SHAPING;
                        static_shaping_weight = 0.0;
                        break;

                    case 'H': case 'h':
                        asked_help = 0;
                        break;

                    case 'N': case 'n':
                        flags |= ADPCM_FLAG_MEASURE_NOISE;
                        break;

                    case 'Q': case 'q':
                        verbosity = -1;
                        break;

                    case 'R': case 'r':
                        flags |= ADPCM_FLAG_RAW_OUTPUT;
                        break;

                    case 'S': case 's':
                        static_shaping_weight = (float) strtod_hexfree (++*argv, argv);

                        if (static_shaping_weight <= -1.0 || static_shaping_weight > 1.0) {
                            fprintf (stderr, "\ninvalid noise shaping value!");
                            return -1;
                        }

                        if (static_shaping_weight == 0.0)
                            flags &= ~ADPCM_FLAG_NOISE_SHAPING;

                        --*argv;
                        break;

                    case 'V': case 'v':
                        verbosity = 1;
                        break;

                    case 'W': case 'w':
                        encode_width_bits = strtol (++*argv, argv, 10);

                        if (encode_width_bits < 2 || encode_width_bits > 5) {
                            fprintf (stderr, "\nencoding width in bits must be 2 to 5!\n");
                            return -1;
                        }

                        --*argv;
                        break;

                    case 'X': case 'x':
                        lookahead |= LOOKAHEAD_EXHAUSTIVE;
                        break;

                    case 'Y': case 'y':
                        overwrite = 1;
                        break;

                    default:
                        fprintf (stderr, "\nillegal option: %c !\n", **argv);
                        return 1;
                }
        else if (!infilename) {
            infilename = malloc (strlen (*argv) + 10);
            strcpy (infilename, *argv);
        }
        else if (!outfilename) {
            outfilename = malloc (strlen (*argv) + 10);
            strcpy (outfilename, *argv);
        }
        else {
            fprintf (stderr, "\nextra unknown argument: %s !\n", *argv);
            return 1;
        }
    }

    if (verbosity >= 0)
        fprintf (stderr, "%s", sign_on);

    if (!outfilename || asked_help) {
        printf ("%s", usage);
        return 0;
    }

    if (!strcmp (infilename, outfilename)) {
        fprintf (stderr, "can't overwrite input file (specify different/new output file name)\n");
        return -1;
    }

    if (!overwrite && (outfile = fopen (outfilename, "r"))) {
        fclose (outfile);
        fprintf (stderr, "output file \"%s\" exists (use -y to overwrite)\n", outfilename);
        return -1;
    }

    return adpcm_converter (infilename, outfilename);
}

// The C-standard function strtod() also handles hex numbers prefixed
// with [+-]0[xX]. Unfortunately this causes problems for us in rare
// cases where a value of zero is specified for one option followed
// by the 'x' option (e.g., -s0xe). This version of strtod() does not
// allow hex specification, but otherwise should be identical.

static double strtod_hexfree (const char *nptr, char **endptr)
{
    const char *sptr = nptr;

    // skip past any leading whitespace and possibly a sign
    while (isspace (*sptr)) sptr++;
    if (*sptr == '+' || *sptr == '-') sptr++;

    // if hex detected ("0x" or "0X"), return 0.0 and end at the X
    if (*sptr == '0' && tolower (sptr [1]) == 'x') {
        if (endptr) *endptr = (char *) sptr + 1;
        return 0.0;
    }

    // otherwise unmodified strtod() result
    return strtod (nptr, endptr);
}

typedef struct {
    char ckID [4];
    uint32_t ckSize;
    char formType [4];
} RiffChunkHeader;

typedef struct {
    char ckID [4];
    uint32_t ckSize;
} ChunkHeader;

#define ChunkHeaderFormat "4L"

typedef struct {
    uint16_t FormatTag, NumChannels;
    uint32_t SampleRate, BytesPerSecond;
    uint16_t BlockAlign, BitsPerSample;
    uint16_t cbSize;
    union {
        uint16_t ValidBitsPerSample;
        uint16_t SamplesPerBlock;
        uint16_t Reserved;
    } Samples;
    int32_t ChannelMask;
    uint16_t SubFormat;
    char GUID [14];
} WaveHeader;

#define WaveHeaderFormat "SSLLSSSSLS"

typedef struct {
    char ckID [4];
    uint32_t ckSize;
    uint32_t TotalSamples;
} FactHeader;

#define FactHeaderFormat "4LL"

#define WAVE_FORMAT_PCM         0x1
#define WAVE_FORMAT_IMA_ADPCM   0x11
#define WAVE_FORMAT_EXTENSIBLE  0xfffe

#define SNAP_NEAREST_POW2(v) { int d = 0; while ((v) & ((v) - 1)) (v) += (d = (~d >> 31 | 1) - d); }

static int write_pcm_wav_header (FILE *outfile, int num_channels, uint32_t num_samples, uint32_t sample_rate);
static int write_adpcm_wav_header (FILE *outfile, int num_channels, int bps, uint32_t num_samples, uint32_t sample_rate, int samples_per_block);
static int adpcm_decode_data (FILE *infile, FILE *outfile, int num_channels, int bps, uint32_t num_samples, int block_size);
static int adpcm_encode_data (FILE *infile, FILE *outfile, int num_channels, int bps, uint32_t num_samples, int samples_per_block, int sample_rate);
static void little_endian_to_native (void *data, char *format);
static void native_to_little_endian (void *data, char *format);

static int adpcm_converter (char *infilename, char *outfilename)
{
    int format = 0, res = 0, bits_per_sample = 0, num_channels = 0;
    uint32_t fact_samples = 0, num_samples = 0, sample_rate = 0;
    FILE *infile, *outfile;
    RiffChunkHeader riff_chunk_header;
    ChunkHeader chunk_header;
    WaveHeader WaveHeader;

    if (!(infile = fopen (infilename, "rb"))) {
        fprintf (stderr, "can't open file \"%s\" for reading!\n", infilename);
        return -1;
    }

    // read initial RIFF form header

    if (!fread (&riff_chunk_header, sizeof (RiffChunkHeader), 1, infile) ||
        strncmp (riff_chunk_header.ckID, "RIFF", 4) ||
        strncmp (riff_chunk_header.formType, "WAVE", 4)) {
            fprintf (stderr, "\"%s\" is not a valid .WAV file!\n", infilename);
            return -1;
    }

    // loop through all elements of the RIFF wav header (until the data chuck)

    while (1) {

        if (!fread (&chunk_header, sizeof (ChunkHeader), 1, infile)) {
            fprintf (stderr, "\"%s\" is not a valid .WAV file!\n", infilename);
            return -1;
        }

        little_endian_to_native (&chunk_header, ChunkHeaderFormat);

        // if it's the format chunk, we want to get some info out of there and
        // make sure it's a .wav file we can handle

        if (!strncmp (chunk_header.ckID, "fmt ", 4)) {
            int supported = 1;

            if (chunk_header.ckSize < 16 || chunk_header.ckSize > sizeof (WaveHeader) ||
                !fread (&WaveHeader, chunk_header.ckSize, 1, infile)) {
                    fprintf (stderr, "\"%s\" is not a valid .WAV file!\n", infilename);
                    return -1;
            }

            little_endian_to_native (&WaveHeader, WaveHeaderFormat);

            format = (WaveHeader.FormatTag == WAVE_FORMAT_EXTENSIBLE && chunk_header.ckSize == 40) ?
                WaveHeader.SubFormat : WaveHeader.FormatTag;

            bits_per_sample = (chunk_header.ckSize == 40 && WaveHeader.Samples.ValidBitsPerSample) ?
                WaveHeader.Samples.ValidBitsPerSample : WaveHeader.BitsPerSample;

            if (WaveHeader.NumChannels < 1 || WaveHeader.NumChannels > 2)
                supported = 0;
            else if (format == WAVE_FORMAT_PCM) {
                if (decode_only) {
                    fprintf (stderr, "\"%s\" is PCM .WAV file, invalid in decode-only mode!\n", infilename);
                    return -1;
                }

                if (bits_per_sample < 9 || bits_per_sample > 16)
                    supported = 0;

                if (WaveHeader.BlockAlign != WaveHeader.NumChannels * 2)
                    supported = 0;
            }
            else if (format == WAVE_FORMAT_IMA_ADPCM) {
                if (encode_only) {
                    fprintf (stderr, "\"%s\" is ADPCM .WAV file, invalid in encode-only mode!\n", infilename);
                    return -1;
                }

                if (verbosity > 0)
                    fprintf (stderr, "%d-bit ADPCM, %d channels, %d samples/block, %d-byte block alignment\n",
                        bits_per_sample, WaveHeader.NumChannels, WaveHeader.Samples.SamplesPerBlock, WaveHeader.BlockAlign);

                if (bits_per_sample < 2 || bits_per_sample > 5)
                    supported = 0;

                if (WaveHeader.Samples.SamplesPerBlock >
                    adpcm_block_size_to_sample_count (WaveHeader.BlockAlign, WaveHeader.NumChannels, bits_per_sample)) {
                        fprintf (stderr, "\"%s\" is not a valid .WAV file!\n", infilename);
                        return -1;
                }
            }
            else
                supported = 0;

            if (!supported) {
                fprintf (stderr, "\"%s\" is an unsupported .WAV format!\n", infilename);
                return -1;
            }

            if (verbosity > 0) {
                fprintf (stderr, "format tag size = %d\n", chunk_header.ckSize);
                fprintf (stderr, "FormatTag = 0x%x, NumChannels = %u, BitsPerSample = %u\n",
                    WaveHeader.FormatTag, WaveHeader.NumChannels, WaveHeader.BitsPerSample);
                fprintf (stderr, "BlockAlign = %u, SampleRate = %lu, BytesPerSecond = %lu\n",
                    WaveHeader.BlockAlign, (unsigned long) WaveHeader.SampleRate, (unsigned long) WaveHeader.BytesPerSecond);

                if (chunk_header.ckSize > 16) {
                    if (format == WAVE_FORMAT_PCM)
                        fprintf (stderr, "cbSize = %d, ValidBitsPerSample = %d\n", WaveHeader.cbSize,
                            WaveHeader.Samples.ValidBitsPerSample);
                    else if (format == WAVE_FORMAT_IMA_ADPCM)
                        fprintf (stderr, "cbSize = %d, SamplesPerBlock = %d\n", WaveHeader.cbSize,
                            WaveHeader.Samples.SamplesPerBlock);
                }

                if (chunk_header.ckSize > 20)
                    fprintf (stderr, "ChannelMask = %x, SubFormat = %d\n",
                        WaveHeader.ChannelMask, WaveHeader.SubFormat);
            }
        }
        else if (!strncmp (chunk_header.ckID, "fact", 4)) {

            if (chunk_header.ckSize < 4 || !fread (&fact_samples, sizeof (fact_samples), 1, infile)) {
                fprintf (stderr, "\"%s\" is not a valid .WAV file!\n", infilename);
                return -1;
            }

            little_endian_to_native (&fact_samples, "L");

            if (chunk_header.ckSize > 4) {
                int bytes_to_skip = chunk_header.ckSize - 4;
                char dummy;

                while (bytes_to_skip--)
                    if (!fread (&dummy, 1, 1, infile)) {
                        fprintf (stderr, "\"%s\" is not a valid .WAV file!\n", infilename);
                        return -1;
                    }
            }
        }
        else if (!strncmp (chunk_header.ckID, "data", 4)) {

            // on the data chunk, get size and exit parsing loop

            if (!WaveHeader.NumChannels) {      // make sure we saw a "fmt" chunk...
                fprintf (stderr, "\"%s\" is not a valid .WAV file!\n", infilename);
                return -1;
            }

            if (!chunk_header.ckSize) {
                fprintf (stderr, "this .WAV file has no audio samples, probably is corrupt!\n");
                return -1;
            }

            if (format == WAVE_FORMAT_PCM) {
                if (chunk_header.ckSize % WaveHeader.BlockAlign) {
                    fprintf (stderr, "\"%s\" is not a valid .WAV file!\n", infilename);
                    return -1;
                }

                num_samples = chunk_header.ckSize / WaveHeader.BlockAlign;
            }
            else {
                uint32_t complete_blocks = chunk_header.ckSize / WaveHeader.BlockAlign;
                int leftover_bytes = chunk_header.ckSize % WaveHeader.BlockAlign;
                int samples_last_block;

                num_samples = complete_blocks * WaveHeader.Samples.SamplesPerBlock;

                if (leftover_bytes) {
                    if (leftover_bytes % (WaveHeader.NumChannels * 4)) {
                        fprintf (stderr, "\"%s\" is not a valid .WAV file!\n", infilename);
                        return -1;
                    }

                    if (verbosity > 0) fprintf (stderr, "data chunk has %d bytes left over for final ADPCM block\n", leftover_bytes);
                    samples_last_block = ((leftover_bytes - (WaveHeader.NumChannels * 4)) * 8) / (bits_per_sample * WaveHeader.NumChannels) + 1;
                    num_samples += samples_last_block;
                }
                else
                    samples_last_block = WaveHeader.Samples.SamplesPerBlock;

                if (fact_samples) {
                    if (fact_samples < num_samples && fact_samples > num_samples - samples_last_block) {
                        if (verbosity > 0) fprintf (stderr, "total samples reduced %lu by FACT chunk\n", (unsigned long) (num_samples - fact_samples));
                        num_samples = fact_samples;
                    }
                    else if (WaveHeader.NumChannels == 2 && (fact_samples >>= 1) < num_samples && fact_samples > num_samples - samples_last_block) {
                        if (verbosity > 0) fprintf (stderr, "num samples reduced %lu by [incorrect] FACT chunk\n", (unsigned long) (num_samples - fact_samples));
                        num_samples = fact_samples;
                    }
                }
            }

            if (!num_samples) {
                fprintf (stderr, "this .WAV file has no audio samples, probably is corrupt!\n");
                return -1;
            }

            if (verbosity > 0)
                fprintf (stderr, "num samples = %lu\n", (unsigned long) num_samples);

            num_channels = WaveHeader.NumChannels;
            sample_rate = WaveHeader.SampleRate;
            break;
        }
        else {          // just ignore unknown chunks
            int bytes_to_eat = (chunk_header.ckSize + 1) & ~1L;
            char dummy;

            if (verbosity > 0)
                fprintf (stderr, "extra unknown chunk \"%c%c%c%c\" of %d bytes\n",
                    chunk_header.ckID [0], chunk_header.ckID [1], chunk_header.ckID [2],
                    chunk_header.ckID [3], chunk_header.ckSize);

            while (bytes_to_eat--)
                if (!fread (&dummy, 1, 1, infile)) {
                    fprintf (stderr, "\"%s\" is not a valid .WAV file!\n", infilename);
                    return -1;
                }
        }
    }

    if (!(outfile = fopen (outfilename, "wb"))) {
        fprintf (stderr, "can't open file \"%s\" for writing!\n", outfilename);
        return -1;
    }

    if (format == WAVE_FORMAT_PCM) {
        int block_size, samples_per_block;

        if (blocksize_pow2)
            block_size = 1 << blocksize_pow2;
        else
            block_size = 256 * num_channels * (sample_rate < 11000 ? 1 : sample_rate / 11000);

        SNAP_NEAREST_POW2 (block_size);     // for "middling" sample rates, snap to nearest power of two
        block_size = adpcm_align_block_size (block_size, num_channels, encode_width_bits, 0);
        samples_per_block = adpcm_block_size_to_sample_count (block_size, num_channels, encode_width_bits);

        if (verbosity > 0)
            fprintf (stderr, "each %d byte ADPCM block will contain %d samples * %d channels\n",
                block_size, samples_per_block, num_channels);

        if (!(flags & ADPCM_FLAG_RAW_OUTPUT) && !write_adpcm_wav_header (outfile, num_channels, encode_width_bits, num_samples, sample_rate, samples_per_block)) {
            fprintf (stderr, "can't write header to file \"%s\" !\n", outfilename);
            return -1;
        }

        if (verbosity >= 0) fprintf (stderr, "encoding PCM file \"%s\" to%sADPCM file \"%s\"...\n",
            infilename, (flags & ADPCM_FLAG_RAW_OUTPUT) ? " raw " : " ", outfilename);

        res = adpcm_encode_data (infile, outfile, num_channels, encode_width_bits, num_samples, samples_per_block, sample_rate);
    }
    else if (format == WAVE_FORMAT_IMA_ADPCM) {
        if (!(flags & ADPCM_FLAG_RAW_OUTPUT) && !write_pcm_wav_header (outfile, num_channels, num_samples, sample_rate)) {
            fprintf (stderr, "can't write header to file \"%s\" !\n", outfilename);
            return -1;
        }

        if (verbosity >= 0) fprintf (stderr, "decoding ADPCM file \"%s\" to%sPCM file \"%s\"...\n",
            infilename, (flags & ADPCM_FLAG_RAW_OUTPUT) ? " raw " : " ", outfilename);

        res = adpcm_decode_data (infile, outfile, num_channels, bits_per_sample, num_samples, WaveHeader.BlockAlign);
    }

    fclose (outfile);
    fclose (infile);
    return res;
}

static int write_pcm_wav_header (FILE *outfile, int num_channels, uint32_t num_samples, uint32_t sample_rate)
{
    RiffChunkHeader riffhdr;
    ChunkHeader datahdr, fmthdr;
    WaveHeader wavhdr;

    int wavhdrsize = 16;
    int bytes_per_sample = 2;
    uint32_t total_data_bytes = num_samples * bytes_per_sample * num_channels;

    memset (&wavhdr, 0, sizeof (wavhdr));

    wavhdr.FormatTag = WAVE_FORMAT_PCM;
    wavhdr.NumChannels = num_channels;
    wavhdr.SampleRate = sample_rate;
    wavhdr.BytesPerSecond = sample_rate * num_channels * bytes_per_sample;
    wavhdr.BlockAlign = bytes_per_sample * num_channels;
    wavhdr.BitsPerSample = 16;

    memcpy (riffhdr.ckID, "RIFF", sizeof (riffhdr.ckID));
    memcpy (riffhdr.formType, "WAVE", sizeof (riffhdr.formType));
    riffhdr.ckSize = sizeof (riffhdr) + wavhdrsize + sizeof (datahdr) + total_data_bytes;
    memcpy (fmthdr.ckID, "fmt ", sizeof (fmthdr.ckID));
    fmthdr.ckSize = wavhdrsize;

    memcpy (datahdr.ckID, "data", sizeof (datahdr.ckID));
    datahdr.ckSize = total_data_bytes;

    // write the RIFF chunks up to just before the data starts

    native_to_little_endian (&riffhdr, ChunkHeaderFormat);
    native_to_little_endian (&fmthdr, ChunkHeaderFormat);
    native_to_little_endian (&wavhdr, WaveHeaderFormat);
    native_to_little_endian (&datahdr, ChunkHeaderFormat);

    return fwrite (&riffhdr, sizeof (riffhdr), 1, outfile) &&
        fwrite (&fmthdr, sizeof (fmthdr), 1, outfile) &&
        fwrite (&wavhdr, wavhdrsize, 1, outfile) &&
        fwrite (&datahdr, sizeof (datahdr), 1, outfile);
}

static int write_adpcm_wav_header (FILE *outfile, int num_channels, int bps, uint32_t num_samples, uint32_t sample_rate, int samples_per_block)
{
    RiffChunkHeader riffhdr;
    ChunkHeader datahdr, fmthdr;
    WaveHeader wavhdr;
    FactHeader facthdr;

    int wavhdrsize = 20;
    int block_size = adpcm_sample_count_to_block_size (samples_per_block, num_channels, bps);
    uint32_t num_blocks = num_samples / samples_per_block;
    int leftover_samples = num_samples % samples_per_block;
    uint32_t total_data_bytes = num_blocks * block_size;

    if (leftover_samples)
        total_data_bytes += adpcm_align_block_size (adpcm_sample_count_to_block_size (leftover_samples, num_channels, bps), num_channels, bps, 1);

    memset (&wavhdr, 0, sizeof (wavhdr));

    wavhdr.FormatTag = WAVE_FORMAT_IMA_ADPCM;
    wavhdr.NumChannels = num_channels;
    wavhdr.SampleRate = sample_rate;
    wavhdr.BytesPerSecond = sample_rate * block_size / samples_per_block;
    wavhdr.BlockAlign = block_size;
    wavhdr.BitsPerSample = bps;
    wavhdr.cbSize = 2;
    wavhdr.Samples.SamplesPerBlock = samples_per_block;

    memcpy (riffhdr.ckID, "RIFF", sizeof (riffhdr.ckID));
    memcpy (riffhdr.formType, "WAVE", sizeof (riffhdr.formType));
    riffhdr.ckSize = sizeof (riffhdr) + wavhdrsize + sizeof (facthdr) + sizeof (datahdr) + total_data_bytes;
    memcpy (fmthdr.ckID, "fmt ", sizeof (fmthdr.ckID));
    fmthdr.ckSize = wavhdrsize;
    memcpy (facthdr.ckID, "fact", sizeof (facthdr.ckID));
    facthdr.TotalSamples = num_samples;
    facthdr.ckSize = 4;

    memcpy (datahdr.ckID, "data", sizeof (datahdr.ckID));
    datahdr.ckSize = total_data_bytes;

    // write the RIFF chunks up to just before the data starts

    native_to_little_endian (&riffhdr, ChunkHeaderFormat);
    native_to_little_endian (&fmthdr, ChunkHeaderFormat);
    native_to_little_endian (&wavhdr, WaveHeaderFormat);
    native_to_little_endian (&facthdr, FactHeaderFormat);
    native_to_little_endian (&datahdr, ChunkHeaderFormat);

    return fwrite (&riffhdr, sizeof (riffhdr), 1, outfile) &&
        fwrite (&fmthdr, sizeof (fmthdr), 1, outfile) &&
        fwrite (&wavhdr, wavhdrsize, 1, outfile) &&
        fwrite (&facthdr, sizeof (facthdr), 1, outfile) &&
        fwrite (&datahdr, sizeof (datahdr), 1, outfile);
}

static int adpcm_decode_data (FILE *infile, FILE *outfile, int num_channels, int bits_per_sample, uint32_t num_samples, int block_size)
{
    int samples_per_block = adpcm_block_size_to_sample_count (block_size, num_channels, bits_per_sample), percent;
    void *pcm_block = malloc (samples_per_block * num_channels * 2);
    void *adpcm_block = malloc (block_size);
    uint32_t progress_divider = 0;

    if (!pcm_block || !adpcm_block) {
        fprintf (stderr, "could not allocate memory for buffers!\n");
        return -1;
    }

    if (verbosity >= 0 && num_samples > 1000) {
        progress_divider = (num_samples + 50) / 100;
        fprintf (stderr, "\rprogress: %d%% ", percent = 0);
        fflush (stderr);
    }

    while (num_samples) {
        int this_block_adpcm_samples = samples_per_block;
        int this_block_pcm_samples = samples_per_block;

        if (this_block_adpcm_samples > (int) num_samples) {
            block_size = adpcm_sample_count_to_block_size (num_samples, num_channels, bits_per_sample);
            this_block_adpcm_samples = adpcm_block_size_to_sample_count (block_size, num_channels, bits_per_sample);
            this_block_pcm_samples = num_samples;

            if (verbosity > 0)
                fprintf (stderr, "\rfinal block decodes %d (of %d) samples in %d-byte block\n",
                    num_samples, this_block_adpcm_samples, block_size);
        }

        if (!fread (adpcm_block, block_size, 1, infile)) {
            fprintf (stderr, "could not read all audio data from input file!\n");
            return -1;
        }

        if (adpcm_decode_block_ex (pcm_block, adpcm_block, block_size, num_channels, bits_per_sample) != this_block_adpcm_samples) {
            fprintf (stderr, "adpcm_decode_block_ex() did not return expected value!\n");
            return -1;
        }

        if (IS_BIG_ENDIAN) {
            int scount = this_block_pcm_samples * num_channels;
            unsigned char *cp = (unsigned char *) pcm_block;

            while (scount--) {
                int16_t temp = * (int16_t *) cp;
                *cp++ = (unsigned char) temp;
                *cp++ = (unsigned char) (temp >> 8);
            }
        }

        if (!fwrite (pcm_block, this_block_pcm_samples * num_channels * 2, 1, outfile)) {
            fprintf (stderr, "could not write all audio data to output file!\n");
            return -1;
        }

        num_samples -= this_block_pcm_samples;

        if (progress_divider) {
            int new_percent = 100 - num_samples / progress_divider;

            if (new_percent != percent) {
                fprintf (stderr, "\rprogress: %d%% ", percent = new_percent);
                fflush (stderr);
            }
        }
    }

    if (verbosity >= 0)
        fprintf (stderr, "\r...completed successfully\n");

    free (adpcm_block);
    free (pcm_block);
    return 0;
}

static int adpcm_encode_data (FILE *infile, FILE *outfile, int num_channels, int bps, uint32_t num_samples, int samples_per_block, int sample_rate)
{
    int block_size = adpcm_sample_count_to_block_size (samples_per_block, num_channels, bps), percent, noise_shaping;
    int16_t *pcm_block = malloc (samples_per_block * num_channels * 2);
    void *adpcm_block = malloc (block_size);
    uint32_t progress_divider = 0;
    void *adpcm_cnxt = NULL;

    double rms_noise_total [2] = { 0.0, 0.0 };
    double rms_noise_peak [2] = { 0.0, 0.0 };
    int32_t max_error [2] = { 0, 0 };
    uint32_t noise_samples = 0;

    if (!pcm_block || !adpcm_block) {
        fprintf (stderr, "could not allocate memory for buffers!\n");
        return -1;
    }

    if (verbosity >= 0 && num_samples > 1000) {
        progress_divider = (num_samples + 50) / 100;
        fprintf (stderr, "\rprogress: %d%% ", percent = 0);
        fflush (stderr);
    }

    if (flags & ADPCM_FLAG_NOISE_SHAPING) {
        if (static_shaping_weight != 0.0)
            noise_shaping = NOISE_SHAPING_STATIC;
        else if (sample_rate > 64000) {
            noise_shaping = NOISE_SHAPING_STATIC;
            static_shaping_weight = 1.0;
        }
        else
            noise_shaping = NOISE_SHAPING_DYNAMIC;
    }
    else
        noise_shaping = NOISE_SHAPING_OFF;

    adpcm_cnxt = adpcm_create_context (num_channels, sample_rate, lookahead, noise_shaping);

    if (!adpcm_cnxt) {
        fprintf (stderr, "could not create ADPCM context!\n");
        return -1;
    }

    if (noise_shaping == NOISE_SHAPING_STATIC)
        adpcm_set_shaping_weight (adpcm_cnxt, static_shaping_weight);

    while (num_samples) {
        int this_block_adpcm_samples = samples_per_block;
        int this_block_pcm_samples = samples_per_block;
        size_t num_bytes;

        if (this_block_pcm_samples > (int) num_samples) {
            block_size = adpcm_align_block_size (adpcm_sample_count_to_block_size (num_samples, num_channels, bps), num_channels, bps, 1);
            this_block_adpcm_samples = adpcm_block_size_to_sample_count (block_size, num_channels, bps);
            this_block_pcm_samples = num_samples;

            if (verbosity > 0)
                fprintf (stderr, "\rfinal block encodes %d (of %d) samples in %d-byte block\n",
                    num_samples, this_block_adpcm_samples, block_size);
        }

        if (!fread (pcm_block, this_block_pcm_samples * num_channels * 2, 1, infile)) {
            fprintf (stderr, "\rcould not read all audio data from input file!\n");
            return -1;
        }

        if (IS_BIG_ENDIAN) {
            int scount = this_block_pcm_samples * num_channels;
            unsigned char *cp = (unsigned char *) pcm_block;

            while (scount--) {
                int16_t temp = cp [0] + (cp [1] << 8);
                * (int16_t *) cp = temp;
                cp += 2;
            }
        }

        // if this is the last block and it's not full, duplicate the last sample(s) so we don't
        // create problems for the lookahead

        if (this_block_adpcm_samples > this_block_pcm_samples) {
            int16_t *dst = pcm_block + this_block_pcm_samples * num_channels, *src = dst - num_channels;
            int dups = (this_block_adpcm_samples - this_block_pcm_samples) * num_channels;

            while (dups--)
                *dst++ = *src++;
        }

        adpcm_encode_block_ex (adpcm_cnxt, adpcm_block, &num_bytes, pcm_block, this_block_adpcm_samples, bps);

        if ((int) num_bytes != block_size) {
            fprintf (stderr, "\radpcm_encode_block_ex() did not return expected value (expected %d, got %d)!\n", block_size, (int) num_bytes);
            return -1;
        }

        if (flags & ADPCM_FLAG_MEASURE_NOISE) {
            int16_t *pcm_decoded = malloc (samples_per_block * num_channels * 2);
            double rms_noise [2] = { 0.0, 0.0 };
            int i;

            if (adpcm_decode_block_ex (pcm_decoded, adpcm_block, block_size, num_channels, bps) != this_block_adpcm_samples) {
                fprintf (stderr, "\radpcm_decode_block_ex() did not return expected value!\n");
                return -1;
            }

            for (i = 0; i < this_block_pcm_samples * num_channels; i += num_channels) {
                int32_t error = abs (pcm_block [i] - pcm_decoded [i]);

                if (error > max_error [0])
                    max_error [0] = error;

                rms_noise [0] += (double) error * error;

                if (num_channels == 2) {
                    error = abs (pcm_block [i+1] - pcm_decoded [i+1]);

                    if (error > max_error [1])
                        max_error [1] = error;

                    rms_noise [1] += (double) error * error;
                }
            }

            noise_samples += this_block_pcm_samples;
            rms_noise_total [0] += rms_noise [0];
            rms_noise_total [1] += rms_noise [1];

            if (rms_noise [0] / this_block_pcm_samples > rms_noise_peak [0])
                rms_noise_peak [0] = rms_noise [0] / this_block_pcm_samples;

            if (rms_noise [1] / this_block_pcm_samples > rms_noise_peak [1])
                rms_noise_peak [1] = rms_noise [1] / this_block_pcm_samples;

            free (pcm_decoded);
        }

        if (!fwrite (adpcm_block, block_size, 1, outfile)) {
            fprintf (stderr, "\rcould not write all audio data to output file!\n");
            return -1;
        }

        num_samples -= this_block_pcm_samples;

        if (progress_divider) {
            int new_percent = 100 - num_samples / progress_divider;

            if (new_percent != percent) {
                fprintf (stderr, "\rprogress: %d%% ", percent = new_percent);
                fflush (stderr);
            }
        }
    }

    if (verbosity >= 0)
        fprintf (stderr, "\r...completed successfully\n");

    if (flags & ADPCM_FLAG_MEASURE_NOISE) {
        double full_scale_rms = 32768.0 * 32767.0 * 0.5;

        if (num_channels == 2) {
            rms_noise_total [0] /= noise_samples * full_scale_rms;
            rms_noise_total [1] /= noise_samples * full_scale_rms;
            rms_noise_peak [0] /= full_scale_rms;
            rms_noise_peak [1] /= full_scale_rms;

            fprintf (stderr, "\n         Channel:    left      right \n");
            fprintf (stderr, "---------------------------------------\n");
            fprintf (stderr, "Max Sample Error:  %6ld     %6ld\n", (long) max_error [0], (long) max_error [1]);
            fprintf (stderr, " RMS Total Noise:  %6.2f dB  %6.2f dB\n", log10 (rms_noise_total [0]) * 10.0, log10 (rms_noise_total [1]) * 10.0);
            fprintf (stderr, "  RMS Peak Noise:  %6.2f dB  %6.2f dB\n\n", log10 (rms_noise_peak [0]) * 10.0, log10 (rms_noise_peak [1]) * 10.0);
        }
        else {
            rms_noise_total [0] /= noise_samples * full_scale_rms;
            rms_noise_peak [0] /= full_scale_rms;

            fprintf (stderr, "\nMax Sample Error:  %6ld\n", (long) max_error [0]);
            fprintf (stderr, " RMS Total Noise:  %6.2f dB\n", log10 (rms_noise_total [0]) * 10.0);
            fprintf (stderr, "  RMS Peak Noise:  %6.2f dB\n\n", log10 (rms_noise_peak [0]) * 10.0);
        }
    }

    adpcm_free_context (adpcm_cnxt);
    free (adpcm_block);
    free (pcm_block);
    return 0;
}

static void little_endian_to_native (void *data, char *format)
{
    unsigned char *cp = (unsigned char *) data;
    int32_t temp;

    while (*format) {
        switch (*format) {
            case 'L':
                temp = cp [0] + ((int32_t) cp [1] << 8) + ((int32_t) cp [2] << 16) + ((int32_t) cp [3] << 24);
                * (int32_t *) cp = temp;
                cp += 4;
                break;

            case 'S':
                temp = cp [0] + (cp [1] << 8);
                * (short *) cp = (short) temp;
                cp += 2;
                break;

            default:
                if (isdigit ((unsigned char) *format))
                    cp += *format - '0';

                break;
        }

        format++;
    }
}

static void native_to_little_endian (void *data, char *format)
{
    unsigned char *cp = (unsigned char *) data;
    int32_t temp;

    while (*format) {
        switch (*format) {
            case 'L':
                temp = * (int32_t *) cp;
                *cp++ = (unsigned char) temp;
                *cp++ = (unsigned char) (temp >> 8);
                *cp++ = (unsigned char) (temp >> 16);
                *cp++ = (unsigned char) (temp >> 24);
                break;

            case 'S':
                temp = * (short *) cp;
                *cp++ = (unsigned char) temp;
                *cp++ = (unsigned char) (temp >> 8);
                break;

            default:
                if (isdigit ((unsigned char) *format))
                    cp += *format - '0';

                break;
        }

        format++;
    }
}
#endif

