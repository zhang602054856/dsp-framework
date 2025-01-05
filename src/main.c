
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>

#include "audio_effect_core.h"
#include "audio_effect_buffers.h"
#include "audio_basic_effects.h"



static gain_config_t gain_0_default = {
    .ramp_type = GAIN_RAMP_EXPONENTIAL,
    .gain = -6.02, //db
};


static module_config_t scaler_0 = {
    .name = "audio,scaler_0",
    .type = MODULE_EFFECT_GAIN,
    .input_ch = 1,
    .output_ch = 1,
    .default_conf = &gain_0_default,
};

static module_config_t scaler_1 = {
    .name = "audio,scaler_1",
    .type = MODULE_EFFECT_GAIN,
    .input_ch = 1,
    .output_ch = 1,
    .default_conf = &gain_0_default,
};

static module_config_t scaler_2 = {
    .name = "audio,scaler_2",
    .type = MODULE_EFFECT_GAIN,
    .input_ch = 1,
    .output_ch = 1,
    .default_conf = &gain_0_default,
};

static module_config_t scaler_3 = {
    .name = "audio,scaler_3",
    .type = MODULE_EFFECT_GAIN,
    .input_ch = 1,
    .output_ch = 1,
    .default_conf = &gain_0_default,
};


static gain_config_t volume_0_default = {
    .ramp_type = GAIN_RAMP_EXPONENTIAL,
    .gain = 0,
    //exp: ramp = 50ms   linear: ramp = 90.0 dB/s
    .attack = 50,
};

static module_config_t volume_0 = {
    .name = "audio,volume_0",
    .type = MODULE_EFFECT_VOL,
    .input_ch = 1,
    .output_ch = 1,
    .default_conf = &volume_0_default,
};

static gain_config_t volume_1_default = {
    .ramp_type = GAIN_RAMP_LINEAR,
    .gain = -36,
    //exp: ramp = 50ms   linear: ramp = 90.0 dB/s
    .attack = 90,
};

static module_config_t volume_1 = {
    .name = "audio,volume_1",
    .type = MODULE_EFFECT_VOL,
    .input_ch = 1,
    .output_ch = 1,
    .default_conf = &volume_1_default,
};

static module_config_t volume_2 = {
    .name = "audio,volume_2",
    .type = MODULE_EFFECT_VOL,
    .input_ch = 1,
    .output_ch = 1,
    .default_conf = &volume_0_default,
};

static module_config_t volume_3 = {
    .name = "audio,volume_3",
    .type = MODULE_EFFECT_VOL,
    .input_ch = 1,
    .output_ch = 1,
    .default_conf = &volume_0_default,
};


static mixer_config_t mixer_0_default = {
    .ramp_type = GAIN_RAMP_LINEAR,
    .input_gain = {-6.03, 0},   // 2ch
    .ramp = 90, //90db/s
};

static module_config_t mixer_0 = {
    .name = "audio,mixer_0",
    .type = MODULE_EFFECT_MIXER,
    .input_ch = 2,
    .output_ch = 1,
    .default_conf = &mixer_0_default,
};

static module_config_t mixer_1 = {
    .name = "audio,mixer_1",
    .type = MODULE_EFFECT_MIXER,
    .input_ch = 2,
    .output_ch = 1,
    .default_conf = &mixer_0_default,
};

#if 0
static delay_config_t delay_0_default = {
    .time_ms = 10,  //delay 10ms
};

static module_config_t delay_0 = {
    .name = "audio,delay",
    .type = MODULE_EFFECT_DELAY,
    // .order = 5,
    .input_ch = 1,
    .output_ch = 1,
    .default_conf = &delay_0_default,
};


static delay_config_t echo_0_default = {
    //delay 5ms 0.7, 0.5
    .time_ms = 5,
    .feedback = 0.7,
    .mix_gain = 0.5,
};

static module_config_t echo_0 = {
    .name = "audio,echo",
    .type = MODULE_EFFECT_ECHO,
    // .order = 4,
    .input_ch = 1,
    .output_ch = 1,
    .default_conf = &echo_0_default,
};
#endif

static biquad_config_t equalizer_0_default = {
    .bands[0] = {
        .filter_type = BIQUAD_TYPE_LOW_SHELF,
        .gain = 3.02,
        .q_factor = 10.1,
        .center_freq = 400,
    },
    .bands[1] = {
        .filter_type = BIQUAD_TYPE_PEAKING,
        .gain = -6.02,
        .q_factor = 5.1,
        .center_freq = 800,
    },
    .bands[2] = {
        .filter_type = BIQUAD_TYPE_PEAKING,
        .gain = 3.02,
        .q_factor = 5.1,
        .center_freq = 1200,
    },
    .bands[3] = {
        .filter_type = BIQUAD_TYPE_HIGH_SHELF,
        .gain = 6.02,
        .q_factor = 12.1,
        .center_freq = 2000,
    },
    .band_num = 4,
};

static module_config_t equalizer_0 = {
    .name = "audio,eq_0",
    .type = MODULE_EFFECT_EQUALIZER,
    .input_ch = 2,
    .output_ch = 2,
    .default_conf = &equalizer_0_default,
};

static module_config_t equalizer_1 = {
    .name = "audio,eq_1",
    .type = MODULE_EFFECT_EQUALIZER,
    .input_ch = 1,
    .output_ch = 1,
    .default_conf = &equalizer_0_default,
};

static module_config_t equalizer_2 = {
    .name = "audio,eq_2",
    .type = MODULE_EFFECT_EQUALIZER,
    .input_ch = 1,
    .output_ch = 1,
    .default_conf = &equalizer_0_default,
};

static module_config_t equalizer_3 = {
    .name = "audio,eq_3",
    .type = MODULE_EFFECT_EQUALIZER,
    .input_ch = 1,
    .output_ch = 1,
    .default_conf = &equalizer_0_default,
};

static module_config_t equalizer_4 = {
    .name = "audio,eq_4",
    .type = MODULE_EFFECT_EQUALIZER,
    .input_ch = 1,
    .output_ch = 1,
    .default_conf = &equalizer_0_default,
};

static module_config_t equalizer_5 = {
    .name = "audio,eq_5",
    .type = MODULE_EFFECT_EQUALIZER,
    .input_ch = 1,
    .output_ch = 1,
    .default_conf = &equalizer_0_default,
};

static module_config_t equalizer_6 = {
    .name = "audio,eq_6",
    .type = MODULE_EFFECT_EQUALIZER,
    .input_ch = 1,
    .output_ch = 1,
    .default_conf = &equalizer_0_default,
};

static mute_config_t mute_0_default = {
    .ramp_type = GAIN_RAMP_LINEAR,
    .mute = false,
    .attack = 90,
    .decay = 90,
};

static mute_config_t mute_1_default = {
    .ramp_type = GAIN_RAMP_LINEAR,
    .mute = true,
    .attack = 90,
    .decay = 90,
};

static module_config_t mute_0 = {
    .name = "audio,mute_0",
    .type = MODULE_EFFECT_MUTE,
    .input_ch = 1,
    .output_ch = 1,
    .default_conf = &mute_0_default,
};

static module_config_t mute_1 = {
    .name = "audio,mute_1",
    .type = MODULE_EFFECT_MUTE,
    .input_ch = 1,
    .output_ch = 1,
    .default_conf = &mute_1_default,
};

static module_config_t mute_2 = {
    .name = "audio,mute_2",
    .type = MODULE_EFFECT_MUTE,
    .input_ch = 1,
    .output_ch = 1,
    .default_conf = &mute_0_default,
};

static module_config_t mute_3 = {
    .name = "audio,mute_3",
    .type = MODULE_EFFECT_MUTE,
    .input_ch = 1,
    .output_ch = 1,
    .default_conf = &mute_0_default,
};

static gain_config_t limiter_0_default = {
    .gain = -6.03,      //db
    .attack = 10,       //ms
    .decay = 80,        //ms
};

static module_config_t limiter_0 = {
    .name = "audio,limiter",
    .type = MODULE_EFFECT_LIMITER,
    .input_ch = 4,
    .output_ch = 4,
    .default_conf = &limiter_0_default,
};

static module_config_t wire_0 = {
    .name = "audio,wire_0",
    .type = MODULE_EFFECT_WIRE,
    .input_ch = 1,
    .output_ch = 2,
};

static module_config_t wire_1 = {
    .name = "audio,wire_1",
    .type = MODULE_EFFECT_WIRE,
    .input_ch = 1,
    .output_ch = 2,
};


const uint8_t hw_slots_map_default[] = {
    0, 1,  2,  3,  4,  5,  6,  7,
    8, 9, 10, 11, 12, 13, 14, 15
};

static module_config_t inputs = {
    .name = "audio,input_endpoint",
    .type = MODULE_ENDPOINT_INPUT,
    .order = ORDER_INPUT_END_POINT,
    .default_conf = (void *)hw_slots_map_default,
};

static module_config_t outputs = {
    .name = "audio,output_endpoint",
    .type = MODULE_ENDPOINT_OUTPUT,
    .order = ORDER_OUTPUT_END_POINT,
    .default_conf = (void *)hw_slots_map_default,
};

static pipeline_config_t pipeline_list[] = {
    {   .source = &inputs,
        .source_index = 0,
        .sink = &scaler_0,
        .sink_index = 0,
    },
    {   .source = &inputs,
        .source_index = 1,
        .sink = &scaler_1,
        .sink_index = 0,
    },
    {   .source = &inputs,
        .source_index = 2,
        .sink = &scaler_2,
        .sink_index = 0,
    },
    {   .source = &inputs,
        .source_index = 3,
        .sink = &scaler_3,
        .sink_index = 0,
    },

    {   .source = &scaler_0,
        .source_index = 0,
        .sink = &volume_0,
        .sink_index = 0,
    },
    {   .source = &volume_0,
        .source_index = 0,
        .sink = &mixer_0,
        .sink_index = 0,
    },

    {   .source = &scaler_1,
        .source_index = 0,
        .sink = &volume_1,
        .sink_index = 0,
    },
    {   .source = &volume_1,
        .source_index = 0,
        .sink = &equalizer_0,
        .sink_index = 0,
    },

    {   .source = &scaler_2,
        .source_index = 0,
        .sink = &volume_2,
        .sink_index = 0,
    },
    {   .source = &volume_2,
        .source_index = 0,
        .sink = &equalizer_0,
        .sink_index = 1,
    },

    {   .source = &mixer_0,
        .source_index = 0,
        // .sink = &wire_0,
        .sink = &equalizer_2,
        .sink_index = 0,
    },

    // {   .source = &wire_0,
    //     .source_index = 0,
    //     .sink = &equalizer_2,
    //     .sink_index = 0,
    // },

    {   .source = &mixer_0,
        .source_index = 0,
        .sink = &equalizer_3,
        .sink_index = 0,
    },
    {   .source = &equalizer_0,
        .source_index = 0,
        .sink = &mixer_0,
        .sink_index = 1,
    },
    {   .source = &equalizer_0,
        .source_index = 1,
        .sink = &wire_1,
        .sink_index = 0,
    },

    {   .source = &wire_1,
        .source_index = 0,
        .sink = &equalizer_4,
        .sink_index = 0,
    },

    {   .source = &wire_1,
        .source_index = 1,
        .sink = &mixer_1,
        .sink_index = 0,
    },

    {   .source = &scaler_3,
        .source_index = 0,
        .sink = &equalizer_1,
        .sink_index = 0,
    },

    {   .source = &equalizer_1,
        .source_index = 0,
        .sink = &mixer_1,
        .sink_index = 1,
    },

    {   .source = &mixer_1,
        .source_index = 0,
        .sink = &equalizer_5,
        .sink_index = 0,
    },

    {   .source = &equalizer_2,
        .source_index = 0,
        .sink = &limiter_0,
        .sink_index = 0,
    },
    {   .source = &equalizer_3,
        .source_index = 0,
        .sink = &limiter_0,
        .sink_index = 1,
    },
    {   .source = &equalizer_4,
        .source_index = 0,
        .sink = &limiter_0,
        .sink_index = 2,
    },
    {   .source = &equalizer_5,
        .source_index = 0,
        .sink = &limiter_0,
        .sink_index = 3,
    },

    {   .source = &limiter_0,
        .source_index = 0,
        .sink = &mute_0,
        .sink_index = 0,
    },
    {   .source = &limiter_0,
        .source_index = 1,
        .sink = &mute_1,
        .sink_index = 0,
    },
    {   .source = &limiter_0,
        .source_index = 2,
        .sink = &mute_2,
        .sink_index = 0,
    },
    {   .source = &limiter_0,
        .source_index = 3,
        .sink = &mute_3,
        .sink_index = 0,
    },

    {   .source = &mute_0,
        .source_index = 0,
        .sink = &outputs,
        .sink_index = 0,
    },
    {   .source = &mute_1,
        .source_index = 0,
        .sink = &outputs,
        .sink_index = 1,
    },
    {   .source = &mute_2,
        .source_index = 0,
        .sink = &outputs,
        .sink_index = 2,
    },
    {   .source = &mute_3,
        .source_index = 0,
        .sink = &outputs,
        .sink_index = 3,
    },
};

static int pipeline_num = sizeof(pipeline_list)/sizeof(pipeline_config_t);
static bool ping_pang_tick = false;


#define ID_RIFF 0x46464952
#define ID_WAVE 0x45564157
#define ID_FMT  0x20746d66
#define ID_DATA 0x61746164

struct wav_header {
    uint32_t riff_id;
    uint32_t riff_sz;
    uint32_t riff_fmt;
    uint32_t fmt_id;
    uint32_t fmt_sz;
    uint16_t audio_format;
    uint16_t num_channels;
    uint32_t sample_rate;
    uint32_t byte_rate;
    uint16_t block_align;
    uint16_t bits_per_sample;
    uint32_t data_id;
    uint32_t data_sz;
};

static int load_pcm_file(const char *file, void* buffer)
{
    int fd = open(file, O_RDONLY, 0644);

    if(fd < 0) {
        printf("open file fail! \n");
        return -1;
    }
    else if(read(fd, buffer, PERIOD_SIZE) == -1) {
        printf("file read failed \n");
        return -2;
    }

    return 0;
}

static bool wavfile_opened = false;
static int read_wav_fd = -1;
static size_t size = 0;

static int load_wav_file(const char *file, void* buffer)
{
    struct wav_header wav_file_header;

    if (false == wavfile_opened) {
        read_wav_fd = open(file, O_RDONLY, 0644);

        if(read_wav_fd < 0) {
            printf("open file fail! \n");
            return -1;
        }
        else if(read(read_wav_fd, &wav_file_header, sizeof(struct wav_header)) == -1) {
            printf("file read failed \n");
            return -2;
        }
        else if (wav_file_header.riff_id != ID_RIFF ||
            wav_file_header.riff_fmt != ID_WAVE) {
            printf("it's not a wav file! \n");
            return -3;
        }
        wavfile_opened = true;
        int rate = wav_file_header.sample_rate;
        int channel = wav_file_header.num_channels;
        printf("fd=%d, channels=%d, sample_rate=%d\n", read_wav_fd, channel, rate);
    }

    int readByte = read(read_wav_fd, buffer, PERIOD_SIZE);
    if (readByte <= 0) {
        printf("wav file total read [%zu] bytes \n", size);
        close(read_wav_fd);
        wavfile_opened = false;
        return -4;
    }
    size += readByte;

    return readByte;
}

static int pcm_write_fd = -1;
static bool write_opened = false;
static int write_pcm_file(const char *file, void* buffer)
{
    if (!write_opened) {
        write_opened = true;
        pcm_write_fd = open(file, O_CREAT|O_RDWR, 0644);
        if(pcm_write_fd < 0) {
            printf("create file fail! \n");
            return -1;
        }
    }

    if(write(pcm_write_fd, buffer, PERIOD_SIZE) == -1) {
        printf("file write failed \n");
        close(pcm_write_fd);
        pcm_write_fd = -1;
        return -1;
    }
    return 0;
}


static int stero_pcm_fd = -1;
static int write_stero_pcm_file(const char *file, uint16_t* left, uint16_t* righ)
{
    if (stero_pcm_fd == -1) {
        stero_pcm_fd = open(file, O_CREAT|O_RDWR, 0644);
        if(stero_pcm_fd < 0) {
            printf("create %s fail! \n", file);
            return -1;
        }
    }
    uint16_t buffer[2 * PERIOD_FRAME] = {0};
    for (int i = 0; i < PERIOD_FRAME; i++) {
        buffer[i * 2] = left[i];
        buffer[i * 2 + 1] = righ[i];
    }

    if(write(stero_pcm_fd, buffer, 2* PERIOD_SIZE) == -1) {
        printf("file write failed \n");
        close(stero_pcm_fd);
        stero_pcm_fd = -1;
        return -1;
    }
    return 0;
}


int main(int argc, char **argv)
{
    init_effect_buffers();

    audio_chain_t* chain = get_audio_chain(0);

    init_audio_chain(chain);

    pipeline_connect(chain, pipeline_list, pipeline_num);

    uint16_t *buffer_0 = (uint16_t *)malloc(PERIOD_SIZE);
    // uint16_t *buffer_1 = (uint16_t *)malloc(PERIOD_SIZE);
    uint16_t *output = (uint16_t *)malloc(PERIOD_SIZE);
    uint16_t *output_2 =  (uint16_t *)malloc(PERIOD_SIZE);

    // float new_gain = -6.1f;
    // dsp_effects_modify(3, &new_gain, 1);

    while(true) {

        memset(buffer_0, 0, PERIOD_SIZE);
        // memset(output_2, 0, PERIOD_SIZE);
        int ret = load_wav_file("../test_pcm/sweep-44.1KHz-mono.wav", buffer_0);
        if (ret < 0) {
            audio_effect_module_t* module = get_module_from_order(get_audio_chain(), 3);
            printf("%s: excuted %u times\n", module->name, module->execute_count);
            if(stero_pcm_fd > 0) {
                close(stero_pcm_fd);
                stero_pcm_fd = -1;
            }
            break;
        }
        copy_to_pp_buffer(AUDIO_CH_ID_LEFT, buffer_0);
        copy_to_pp_buffer(AUDIO_CH_ID_RIGHT, buffer_0);

        dsp_effects_process();

        copy_from_pp_buffer(AUDIO_CH_ID_LEFT, output);
        copy_from_pp_buffer(AUDIO_CH_ID_RIGHT, output_2);

        write_stero_pcm_file("../test_pcm/stereo-eq-processed.pcm", output, output_2);
        // write_pcm_file("../test_pcm/mixer-processed-mono-1.pcm", output);
        // write_pcm_file("../test_pcm/mixer-processed-mono-1.pcm", output_2);

    }

    free(buffer_0);
    // free(buffer_1);
    free(output);
    free(output_2);
}