#ifndef _AUDIO_EFFECT_MACROS_H_
#define _AUDIO_EFFECT_MACROS_H_

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "config.h"

#define MAX_INPUT_NUM           16              // 16 channel
#define MAX_OUTPUT_NUM          16              // 16 channel
#define MAX_EFFECTS_NUM         40
#define MAX_CHAIN_NUM           1
#define MAX_DELAY_NUM           16
#define INVALID_ENDPOINT        (-1)

#define PCM_SAMPLE_SIZE         (PCM_FORMAT_BITS/8) //bytes

#if PCM_FORMAT_BITS == 16
    #define PCM_SAMPLE_VAL_MAX  32767       // (2^16 / 2) - 1
#elif PCM_FORMAT_BITS == 24
    #define PCM_SAMPLE_VAL_MAX  8388607     // (2^24 / 2) - 1
#elif PCM_FORMAT_BITS == 32
    #define PCM_SAMPLE_VAL_MAX  2147483647  // (2^32 / 2) - 1
#endif

#define PCM_SAMPLE_RATE_8       8000
#define PCM_SAMPLE_RATE_16      16000
#define PCM_SAMPLE_RATE_24      24000
#define PCM_SAMPLE_RATE_441     44100
#define PCM_SAMPLE_RATE_48      48000

// #define PERIOD_FRAME            (PCM_PERIOD_TIME * PCM_FORMAT_RATE / 1000)
#define PERIOD_FRAME            512 //frame
#define PERIOD_TIME_MS          (PERIOD_FRAME * 1000 / PCM_FORMAT_RATE)
#define PERIOD_SIZE             (PERIOD_FRAME * PCM_SAMPLE_SIZE)

#define AUDIO_CH_LEFT           0
#define AUDIO_CH_RIGHT          1
#define AUDIO_CH_MONO           AUDIO_CH_LEFT

#define EFFECT_FIR_COEFF_TAPS   101


void print_data(uint8_t *buffer, int size);

#endif
