#ifndef _AUDIO_EFFECT_MODULE_CORE_H_
#define _AUDIO_EFFECT_MODULE_CORE_H_


#include "config.h"
#include "audio_effect_config.h"

/* effect return definition */
#define EFFECT_NO_ERR                   0
#define EFFECT_NOT_INIT                 -1
#define EFFECT_GENERAL_ERROR            -2
#define EFFECT_INVAILD_ARG              -3
#define EFFECT_PIPELINE_ERR             -4


/* reserved order value for input/output endpoint */

#define ORDER_INPUT_END_POINT           0
#define ORDER_OUTPUT_END_POINT          1
#define ORDER_OFFSET                    2 // effect's order start of offset

/* effect module process function type */
typedef int (*module_process_func_t)(void *module);

/* effect module setup function type */
typedef int (*module_modify_func_t)(void *module, float *params, int size);

/* effect module init function type */
typedef int (*module_init_func_t)(void *module, void *config);


/* common structure of effect module instance in dsp framework */
typedef struct audio_effect_module {
    /* module name, 10 bytes limit */
    const char *name;

    /* identified the processing order in the audio chain */
    uint8_t order;

    /* identify the effect types, such as volume/mute/equalizer etc */
    DSP_MODULE_TYPE type;

    /* buffer use to store the processed pcm data for each channel. */
    float *buffer[MAX_OUTPUT_NUM];
    /* total output channels of buffer[] */
    uint8_t output_num;

    /* the input_buffer point to the previous module's output buffer,
     * it's not a real continue address buffer */
    float *input_buffer[MAX_INPUT_NUM];
    /* total input channels of input_buffer[] */
    uint8_t input_num;

    // point to the next effect module which connected in an audio chain;
    // struct audio_effect_module *next[MAX_OUTPUT_NUM];

    // point to the previous effect module which connected in an audio chain;
    // struct audio_effect_module *prev[MAX_INPUT_NUM];

    /* the point of coefficents buffer */
    void *params;

    /* the coefficents bytes size */
    uint32_t param_size;

    /* effect process function */
    module_process_func_t process;
    /* effect setup coefficient function */
    // module_modify_func_t setup;

    /* identify the module state, default is true, and change to false after init */
    bool standby;

    /* bypass */
    bool is_bypass;

    /* debug only */
    uint32_t execute_count;

} audio_effect_module_t;


typedef struct audio_chain {
    // id identified the chain in whole DSP framework.
    uint8_t id;

    // effect modules which in audio chain;
    audio_effect_module_t *module_list[MAX_EFFECTS_NUM];

    // the effect numbers in the chain
    uint8_t effects_num;

    // entry effect modules, max 16 ch; unused
    module_config_t *entry_modules[MAX_INPUT_NUM];

    uint8_t entry_num;  //unused

    /* the point of input_endpoint buffer, it's the 1st buffer of the chain.
     * as the input buffer of the next module.
     */
    float *input_buffer[MAX_INPUT_NUM];

    /* the point of output_endpoint buffer,
     * point to the previous module's output buffer.
     */
    float *output_buffer[MAX_OUTPUT_NUM];

    /* identify the input channels of the chain */
    uint8_t input_num;

    /* identify the output channels of the chain */
    uint8_t output_num;

    /* hardware input slot id mapping to index of input_buffer[] */
    uint8_t hw_input_map[MAX_INPUT_NUM];
    /* hardware output slot id mapping to index of output_buffer[] */
    uint8_t hw_output_map[MAX_OUTPUT_NUM];

} audio_chain_t;



/* external functions */

audio_chain_t* get_audio_chain();
void init_audio_chain(audio_chain_t *chain);
void pipeline_connect(audio_chain_t *chain, pipeline_config_t *config, int pipe_num);
void dsp_effects_process();
void dsp_effects_bypass(audio_effect_module_t *module, bool is_bypass);
int dsp_effects_modify(int order, float *new_config, int size);


audio_effect_module_t* get_module_from_order(audio_chain_t *chain, uint8_t order);
audio_effect_module_t* get_module_from_name(audio_chain_t *chain, const char *name);

void *module_get_coeff(audio_effect_module_t *module);

int get_module_coeff_size(DSP_MODULE_TYPE type, int input_ch, int bands);


#endif