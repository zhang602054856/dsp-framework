
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>

#include "audio_effect_core.h"
#include "audio_effect_buffers.h"
#include "effects.h"



static module_config_t gain_0 = {
    .name = "audio,gain_left",
    .type = MODULE_EFFECT_GAIN,
    .order = 2,
    .input_ch = 1,
    .output_ch = 1,
    .param_size = 1,
};


static module_config_t volume_0 = {
    .name = "audio,volume_left",
    .type = MODULE_EFFECT_VOL,
    .order = 2,
    .input_ch = 1,
    .output_ch = 1,
    .gain_ramp_type = GAIN_RAMP_EXPONENTIAL,
    .param_size = 2,
    .rt_param_size = 1,
};

static module_config_t volume_1 = {
    .name = "audio,volume_right",
    .type = MODULE_EFFECT_VOL,
    .order = 3,
    .input_ch = 1,
    .output_ch = 1,
    .gain_ramp_type = GAIN_RAMP_EXPONENTIAL,
    .param_size = 2,
    .rt_param_size = 1,
};

static module_config_t delay_0 = {
    .name = "audio,delay",
    .type = MODULE_EFFECT_DELAY,
    .order = 4,
    .input_ch = 1,
    .output_ch = 1,
    .param_size = 1,
    .rt_param_size = 1,
};

static module_config_t echo_0 = {
    .name = "audio,echo",
    .type = MODULE_EFFECT_ECHO,
    .order = 4,
    .input_ch = 1,
    .output_ch = 1,
    .param_size = 1,
    .rt_param_size = 1,
};

static module_config_t equalizer_0 = {
    .name = "audio,eq",
    .type = MODULE_EFFECT_EQUALIZER,
    .order = 4,
    .input_ch = 1,
    .output_ch = 1,
    .param_size = IIR_FILTER_COEFF_SIZE * IIR_FILTER_BAND,
    .rt_param_size = IIR_FILTER_RT_COEFF_SIZE * IIR_FILTER_BAND,
};


static module_config_t mixer_0 = {
    .name = "audio,mixer",
    .type = MODULE_EFFECT_MIXER,
    .order = 5,
    .input_ch = 2,
    .output_ch = 1,
    .param_size = 5,
    .rt_param_size = 2,
};

static module_config_t mute_0 = {
    .name = "audio,mute",
    .type = MODULE_EFFECT_MUTE,
    .order = 6,
    .input_ch = 1,
    .output_ch = 1,
    .gain_ramp_type = GAIN_RAMP_LINEAR,
    .param_size = 3,
    .rt_param_size = 1,
};

static module_config_t limiter_0 = {
    .name = "audio,limiter",
    .type = MODULE_EFFECT_LIMITER,
    .order = 7,
    .input_ch = 1,
    .output_ch = 1,
    .param_size = 4,
    .rt_param_size = 1,
};

static module_config_t inputs = {
    .name = "audio,input_endpoint",
    .type = MODULE_ENDPOINT_INPUT,
    .order = 0,
};

static module_config_t outputs = {
    .name = "audio,output_endpoint",
    .type = MODULE_ENDPOINT_OUTPUT,
    .order = 1,
};

static pipeline_config_t pipeline_list[] = {
    {   .source = &inputs,
        .source_index = 0,
        .sink = &gain_0,
        .sink_index = 0,
    },
    {   .source = &inputs,
        .source_index = 1,
        .sink = &volume_1,
        .sink_index = 0,
    },
    {   .source = &gain_0,
        .source_index = 0,
        .sink = &equalizer_0,
        .sink_index = 0,
    },
    // {   .source = &delay_0,
    //     .source_index = 0,
    //     .sink = &mixer_0,
    //     .sink_index = 0,
    // },
    // {   .source = &volume_1,
    //     .source_index = 0,
    //     .sink = &mixer_0,
    //     .sink_index = 1,
    // },
    {   .source = &equalizer_0,
        .source_index = 0,
        .sink = &outputs,
        .sink_index = 0,
    },
    {   .source = &volume_1,
        .source_index = 0,
        .sink = &outputs,
        .sink_index = 1,
    },
    // {   .source = &mixer_0,
    //     .source_index = 0,
    //     .sink = &mute_0,
    //     .sink_index = 0,
    // },
    // {   .source = &mute_0,
    //     .source_index = 0,
    //     .sink = &limiter_0,
    //     .sink_index = 0,
    // },
    // {   .source = &limiter_0,
    //     .source_index = 0,
    //     .sink = &outputs,
    //     .sink_index = 0,
    // },
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
    while(true) {

        memset(buffer_0, 0, PERIOD_SIZE);
        // memset(output_2, 0, PERIOD_SIZE);
        int ret = load_wav_file("../test_pcm/sweep-44.1KHz-mono.wav", buffer_0);
        if (ret < 0) {
            audio_effect_module_t* module = get_module_from_chain(get_audio_chain(), 3);
            printf("%s: excuted %u times\n", module->name, module->execute_count);
            if(stero_pcm_fd > 0) {
                close(stero_pcm_fd);
                stero_pcm_fd = -1;
            }
            exit(ret);
        }
        copy_to_pp_buffer(AUDIO_CH_LEFT, buffer_0);
        copy_to_pp_buffer(AUDIO_CH_RIGHT, buffer_0);

        dsp_effects_process();

        copy_from_pp_buffer(AUDIO_CH_LEFT, output);
        copy_from_pp_buffer(AUDIO_CH_RIGHT, output_2);

        write_stero_pcm_file("../test_pcm/stereo-eq-processed.pcm", output, output_2);
        // write_pcm_file("../test_pcm/mixer-processed-mono-1.pcm", output);
        // write_pcm_file("../test_pcm/mixer-processed-mono-1.pcm", output_2);

    }

    free(buffer_0);
    // free(buffer_1);
    free(output);
    free(output_2);
}