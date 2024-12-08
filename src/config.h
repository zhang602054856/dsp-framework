#ifndef _AUDIO_EFFECT_CONFIG_H_
#define _AUDIO_EFFECT_CONFIG_H_


#define PCM_FORMAT_BITS         16      //bits
#define PCM_FORMAT_RATE         44100
#define PCM_PERIOD_TIME         10      // ms 44100 => 20ms; 48000, 10ms
#define PCM_DELAY_BUFF_SIZE     1024    // frame 1024 * 1000 /44100 ~= 23.22ms

#define HW_INPUT_PORTS          1
#define HW_OUTPUT_PORTS         1
#define HW_INPUT_CH             2
#define HW_OUTPUT_CH            2

#define MAX_COEFF_NUM           2048
#define MAX_BAND_NUM            20


#endif
