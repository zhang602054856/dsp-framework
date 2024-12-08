#ifndef _AUDIO_EFFECT_MODULE_H_
#define _AUDIO_EFFECT_MODULE_H_

#include "macros.h"

typedef int(*module_process_func_t)(void *data);

typedef struct audio_effect_module {
    // module name, 10 bytes length limit.
    const char *name;
    // id identified the processing sequence in the audio chain.
    uint8_t id;
    // identify the effect types, such as volume/mute/equalizer etc.
    uint8_t type;

    // buffer use to store the pcm data which processed by effect.
    float *buffer[MAX_OUTPUT_NUM];
    // input buffer point to previous module's output
    float *input_buffer[MAX_INPUT_NUM];

    // point to the next effect module which connected in an audio chain;
    // struct audio_effect_module *next[MAX_OUTPUT_NUM];
    uint8_t output_num;

    // point to the previous effect module which connected in an audio chain;
    // struct audio_effect_module *prev[MAX_INPUT_NUM];
    uint8_t input_num;

    uint8_t gain_ramp_type;

    float *params;
    int param_size;
    module_process_func_t process;

    // runtime opearation coefficient, cannot setup.
    float *rt_params;
    int rt_param_size;

    uint32_t execute_count;

    //identify the module state, it will be change to false after setup
    bool standby;
    // int8_t endpoint_input;
    // int8_t endpoint_output;

} audio_effect_module_t;

typedef struct audio_chain {
    // id identified the chain in whole DSP framework.
    uint8_t id;

    // effect modules which in audio chain;
    audio_effect_module_t *module_list[MAX_EFFECTS_NUM];
    uint8_t effects_num;

    // entry effect modules, max 16 ch;
    audio_effect_module_t *entry_modules[MAX_INPUT_NUM];
    uint8_t entry_num;

    float *input_buffer[MAX_INPUT_NUM];
    float *output_buffer[MAX_OUTPUT_NUM];
    uint8_t input_ch;
    uint8_t output_ch;

    uint8_t hw_input_map[MAX_INPUT_NUM];
    uint8_t hw_output_map[MAX_OUTPUT_NUM];

} audio_chain_t;

/* struct define for effect module configurations.
*/
typedef struct module_config {
    const char* name;
    uint8_t type;
    uint8_t order;
    uint8_t input_ch;
    uint8_t output_ch;
    uint8_t gain_ramp_type;
    // uint8_t endpoint_index;

    int param_size;
    int rt_param_size;
    float *coefficients;

} module_config_t;


/* struct define for equalizer effect configurations.
*/
typedef struct equalizer_config {
    // Inherit from module_config
    module_config_t module;
    // uint8_t band_num;

    // // dedecated config for equalizer
    // struct eq_filter {
    //     float frequence;
    //     float gain;
    //     float q_factor;
    //     uint8_t filter_type;
    // } eq_filter_params[MAX_BAND_NUM];

} equalizer_config_t;

/* struct define for pipeline connection configurations.
*/
typedef struct pipeline_config {
    module_config_t *source;
    module_config_t *sink;
    uint8_t source_index;
    uint8_t sink_index;
} pipeline_config_t;


audio_chain_t* get_audio_chain();

void init_audio_chain(audio_chain_t *chain);
void pipeline_connect(audio_chain_t *chain,
        pipeline_config_t *config, int pipe_num);
void dsp_effects_process();

audio_effect_module_t* add_module(audio_chain_t *chain, module_config_t *conf);
audio_effect_module_t* get_module_from_chain(audio_chain_t *chain, uint8_t id);

module_process_func_t get_module_process(uint8_t type);

#endif