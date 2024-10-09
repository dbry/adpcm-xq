## ADPCM-XQ

Xtreme Quality ADPCM Encoder/Decoder

Copyright (c) 2024 David Bryant.

All Rights Reserved.

Distributed under the [BSD Software License](https://github.com/dbry/adpcm-xq/blob/master/license.txt).

## What is this?

While very popular at the end of the last century, ADPCM is no longer a
common audio encoding format, and is certainly not recommended as a general
purpose encoder. However, it requires minimal CPU resources for decoding,
and so still is ideally suited for certain embedded games and applications
that contain canned audio samples.

This encoder combines two different techniques to achieve higher quality
than existing ADPCM encoders while remaining fully compatible with standard
decoders. The first is dynamic noise shaping, which shifts the quantization
noise up or down in frequency based on the spectrum of the source signal.
This technique is identical to the algorithm used in WavPack's lossy mode
and can make any audible quantization noise much less annoying (or, in some
cases, inaudible).

The other technique is "lookahead" in which the encoder exhaustively
searches ahead to find the optimum coding sequence based on future samples.
This process can reduce the quantization noise from about 1 to 10 dB (depending
on the source) and also reduces or eliminates the harmonic content in the
noise that sometimes plagues ADPCM. Unfortunately, at its maximum settings
this can be very slow, but this should be relatively irrelevant if the
encoder is being used to generate canned samples.

**Adpcm-xq** consists of three standard C files and a header file ([adpcm-lib.h](adpcm-lib.h)).
It can be used as a stand-alone command-line program implemented in [adpcm-xq.c](adpcm-xq.c),
or the library, which consists of [adpcm-lib.c](adpcm-lib.c) and [adpcm-dns.c](adpcm-dns.c),
can be built into and utilized by another application. The library portion has
been designed with maximum portability in mind and should work correctly even
on 16-bit and big-endian architectures.

## What's New?

The latest version of **adpcm-xq** has many enhancements including greatly
improved performance of the conversion and the ability to calculate and display
the quantization noise introduced in the operation.
[See all the details here.](https://github.com/dbry/adpcm-xq/releases/tag/v0.5)

## Variations

There are several forms and variations of IMA ADPCM encoding. The one handled
by **adpcm-xq** is the canonical one used in Microsoft WAV files. The audio is
divided into fixed-sized blocks that include a 4-byte header (or 8-byte for
stereo) that includes the first sample. The size of the blocks is stored in the
WAV header and the audio nibbles are ordered least-significant temporally first.

The latest version of adpcm-xq also includes 2-bit, 3-bit and 5-bit ADPCM. These
are **not well-supported at all** (and some support is buggy) but they might be
useful if this library is decoding them in situations where more compression
or higher quality is desired. BTW, [Rockbox](https://www.rockbox.org/) provides
excellent support for them!

Some applications like games and consoles that decode IMA ADPCM in hardware or
microcode use modified versions of IMA ADPCM that do not use headers and do not
divide the audio into blocks or frames of any kind. The decoding parameters are
simply initialized to zero and the audio nibbles continue uninterrupted to the
end of the clip. I have created an experimental version that will generate two
variations of this data. One is standard nibble order and the other is reversed
nibble order (sometimes called **Intel/DVI4** or **ADP4** and is the format used in
AIFF files). These formats are only writable as "raw" by the **adpcm-xq** command-line
program because they are not representable in WAV files, and they cannot
be decoded by the program either (for the same reason), however the library
itself handles them. [The experimental branch is here.](https://github.com/dbry/adpcm-xq/commits/new-formats/)

## Building

To build the command-line tool (**ADPCM-XQ**) on Linux:

> $ gcc -O3 *.c -lm -o adpcm-xq

on Darwin/Mac:

> $ cmake -DCMAKE_OSX_ARCHITECTURES="arm64;x86_64" . ; make

on MS Visual Studio:

> C:\cl -O3 adpcm-xq.c adpcm-lib.c adpcm-dns.c

## Help

```
 ADPCM-XQ   Xtreme Quality IMA-ADPCM WAV Encoder / Decoder   Version 0.5
 Copyright (c) 2024 David Bryant. All Rights Reserved.

 Usage:     ADPCM-XQ [-options] infile.wav outfile.wav

 Operation: conversion is performed based on the type of the infile
          (either encode 16-bit PCM to 4-bit IMA-ADPCM or decode back)

 Options:  -[0-16]= encode lookahead samples (default = 3, max = 16)
           -b<n>  = override auto block size, 2^n bytes (n = 8-15)
           -d     = decode only (fail on WAV file already PCM)
           -e     = encode only (fail on WAV file already ADPCM)
           -f     = encode flat noise (no noise shaping, aka -s0.0)
           -h     = display this help message
           -n     = measure and report quantization noise
           -q     = quiet mode (display errors only)
           -r     = raw output (little-endian, no WAV header written)
           -s<n>  = override default noise shaping, (-1.0 < n <= 1.0)
           -v     = verbose (display lots of info)
           -w<n>  = override default 4-bit ADPCM width (2 <= n <= 5)
           -x     = exhaustive search (old behavior, very slow at depth)
           -y     = overwrite outfile if it exists

 Web:       Visit www.github.com/dbry/adpcm-xq for latest version and info

```

## Caveats

- Unknown RIFF chunk types are correctly parsed on input files, but are not
passed to the output file.

- In some situations, at high lookahead levels, the operation can get very slow
or even seem to be stuck, however this will happen at much higher lookahead depths
than before. The default level 3 should always be fine and then the user can
simply try increasing levels until the time becomes untenable. The new quantization
noise option (**-n**) can be used to determine if higher levels are providing
improvement (lower numbers are better). Note that the flat noise option (**-f**)
will provide the lowest *measured* noise, but the default dynamic noise shaping
may provide *less audible* noise.

- Pipes are not yet supported.
