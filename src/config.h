#ifndef _AUDIO_EFFECT_CONFIG_H_
#define _AUDIO_EFFECT_CONFIG_H_

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>



#define MAX_INPUT_NUM                   16      // 16 channel
#define MAX_OUTPUT_NUM                  16      // 16 channel
#define MAX_EFFECTS_NUM                 80
#define MAX_CHAIN_NUM                   1
#define MAX_DELAY_NUM                   16
#define MAX_EFFECT_INPUT                8
#define MAX_EFFECT_OUTPUT               MAX_EFFECT_INPUT
#define MAX_BAND_NUM                    20
#define MAX_COCURRENT_OUTPUT            2


#define INVALID_ENDPOINT                (-1)

#define PCM_FORMAT_BITS                 16      // bits
#define PCM_FORMAT_RATE                 44100
#define PCM_PERIOD_TIME                 10      // ms 44100 => 20ms; 48000, 10ms
#define PCM_DELAY_BUFF_SIZE             1024    // frame 1024 * 1000 /44100,  23.22ms

#define HW_INPUT_PORTS                  1
#define HW_OUTPUT_PORTS                 1
#define HW_INPUT_CH                     2
#define HW_OUTPUT_CH                    2

#define MAX_COEFF_NUM                   65536

#define PCM_SAMPLE_SIZE                 (PCM_FORMAT_BITS/8) //bytes

#if (PCM_FORMAT_BITS == 16)
#define PCM_SAMPLE_VAL_MAX              32767       // (2^16 / 2) - 1
#elif (PCM_FORMAT_BITS == 24)
#define PCM_SAMPLE_VAL_MAX              8388607     // (2^24 / 2) - 1
#elif (PCM_FORMAT_BITS == 32)
#define PCM_SAMPLE_VAL_MAX              2147483647  // (2^32 / 2) - 1
#endif

#define PCM_SAMPLE_RATE_8               8000
#define PCM_SAMPLE_RATE_16              16000
#define PCM_SAMPLE_RATE_24              24000
#define PCM_SAMPLE_RATE_441             44100
#define PCM_SAMPLE_RATE_48              48000

// #define PERIOD_FRAME                 (PCM_PERIOD_TIME * PCM_FORMAT_RATE / 1000)
#define PERIOD_FRAME                    512 //frame
#define PERIOD_TIME_MS                  (PERIOD_FRAME * 1000 / PCM_FORMAT_RATE)
#define PERIOD_SIZE                     (PERIOD_FRAME * PCM_SAMPLE_SIZE)

#define AUDIO_CH_MASK_LEFT              ((uint32_t 1) << 0)
#define AUDIO_CH_MASK_RIGHT             ((uint32_t 1) << 1)
#define AUDIO_CH_MASK_2                 ((uint32_t 1) << 2)
#define AUDIO_CH_MASK_3                 ((uint32_t 1) << 3)
#define AUDIO_CH_MASK_4                 ((uint32_t 1) << 4)
#define AUDIO_CH_MASK_5                 ((uint32_t 1) << 5)
#define AUDIO_CH_MASK_6                 ((uint32_t 1) << 6)
#define AUDIO_CH_MASK_7                 ((uint32_t 1) << 7)
#define AUDIO_CH_MASK_8                 ((uint32_t 1) << 8)
#define AUDIO_CH_MASK_9                 ((uint32_t 1) << 9)

#define AUDIO_CH_ID_LEFT                0
#define AUDIO_CH_ID_RIGHT               1
#define AUDIO_CH_ID_MONO                AUDIO_CH_ID_LEFT

#define AUDIO_CH_MONO                   1
#define AUDIO_CH_STEREO                 2

#define EFFECT_FIR_COEFF_TAPS           101


void print_data(uint8_t *buffer, int size);

#endif
