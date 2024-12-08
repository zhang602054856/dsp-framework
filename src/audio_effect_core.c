#include "audio_effect_core.h"
#include "audio_effect_buffers.h"
#include "effects.h"

static audio_chain_t audio_chain;
static audio_effect_module_t effect_module_list[MODULE_TYPE_MAX];
static uint8_t module_num;

const uint8_t hw_input_map[] = {
    0, 1,  2,  3,  4,  5,  6,  7,
    8, 9, 10, 11, 12, 13, 14, 15
};
const uint8_t hw_output_map[] = {
    0, 1,  2,  3,  4,  5,  6,  7,
    8, 9, 10, 11, 12, 13, 14, 15
};

/**
 * @brief printf the pcm data as HEX, each line has 16 bytes
 *
 * @param buffer will be print
 * @param size indicate the buffer size
*/
void print_data(uint8_t *buffer, int size)
{
    int index = 0;
    for(int i = 0; i < size; i++) {
        printf("%02X ", buffer[index++]);
        if(index % 16 == 0)
            printf("\n");
    }
    printf("\n");
}

/**
 * @brief dump pipeline info
 *
 * @param null
*/
static void dump_pipelines()
{
    audio_chain_t *chain = get_audio_chain();
    printf("\n***** dump pipelines *****\n");

    printf("chain_input_buffer: \t");
    for(int i = 0; i < chain->input_ch; i++) {
        printf(" [%d] = %p", i, chain->input_buffer[i]);
    }

    printf("\nchain_output_buffer: \t");
    for(int i = 0; i < chain->output_ch; i++) {
        printf(" [%d] = %p", i, chain->output_buffer[i]);
    }

    for(int i = 0; i < chain->effects_num; i++ ) {
        audio_effect_module_t *module = chain->module_list[i];
        printf("\nmodule: %s\n", module->name);

        if (module->input_num > 0)
            printf("\tinput:\t");
        for(int cnt = 0; cnt < module->input_num; cnt++ ) {
            printf(" [%d] = %p", cnt, module->input_buffer[cnt]);
        }

        if (module->output_num)
            printf("\n\toutput:\t");
        for(int cnt = 0; cnt < module->output_num; cnt++ ) {
            printf(" [%d] = %p", cnt, module->buffer[cnt]);
        }
    }
    printf("\n***** end dump *****\n");
    printf("\n");
}

/**
 * @brief get an available module from static module list, then initialize it according
 *          to module_config
 *
 * @param null
 * @return effect module address if successful. NULL if failed.
*/
static audio_effect_module_t* get_available_module()
{
    if (module_num >= MODULE_TYPE_MAX) {
        printf("there is no more available effect modules, [%d] exceed max num\n", module_num);
        return NULL;
    }
    audio_effect_module_t* module = &effect_module_list[module_num];
    module_num++;
    return module;
}

/**
 * @brief add effect module into the audio chain with the module configurations
 *
 * @param chain indicate audio chain
 * @param config indicate effect module configuration
 * @return effect module address if successful. NULL if failed.
*/
audio_effect_module_t* add_module(audio_chain_t *chain, module_config_t *config)
{
    if (config->order > MAX_EFFECTS_NUM)
        return NULL;

    if (NULL != chain->module_list[config->order]) {
        return chain->module_list[config->order];
    }

    audio_effect_module_t *module = get_available_module();
    if (module == NULL)
        return NULL;

    // save module in chain
    chain->module_list[config->order] = module;
    chain->effects_num++;

    module->id = config->order;
    module->type = config->type;
    module->name = config->name;

    if (config->type == MODULE_ENDPOINT_INPUT ||
        config->type == MODULE_ENDPOINT_OUTPUT) {
        printf("%s|%s, id=%d\n", __func__, config->name, config->order);
        return module;
    }

    module->input_num = config->input_ch;
    module->output_num = config->output_ch;
    module->gain_ramp_type = config->gain_ramp_type;

    module->process = get_module_process(config->type);

    if(config->param_size > 0) {
        module->param_size = config->param_size;
        module->params = get_available_coefficent(config->param_size);
    }

    if(config->rt_param_size > 0) {
        module->rt_params = get_available_runtime_coefficent(config->rt_param_size);
    }

    for (int i = 0; i < config->output_ch; i++) {
        module->buffer[i] = get_available_module_buffer();
    }
    module->standby = true;

    printf("%s|%s, id=%d, channels: in[%d], out[%d], param_size[%d]\n", __func__,
            config->name, config->order, config->input_ch, config->output_ch, config->param_size);
    return module;
}

/**
 * @brief get the effect module from audio chain according id (operation order)
 *
 * @param chain indicate audio chain
 * @param id indicate operation order. it's equal to module->id
 * @return effect module address if successful. NULL if failed.
*/
audio_effect_module_t* get_module_from_chain(audio_chain_t *chain, uint8_t id)
{
    if (id > chain->effects_num)
        return NULL;

    return chain->module_list[id];
}

/**
 * @brief attach effect modules
 *
 * @param chain indicate audio chain
 * @param source indicate source effect module
 * @param src_index indicate the operation order of source effect module
 * @param sink indicate sink effect module
 * @param sink_index indicate the operation order of sink effect module
*/
static void effect_module_attach(audio_chain_t *chain,
                    audio_effect_module_t *source, uint8_t src_index,
                    audio_effect_module_t *sink, uint8_t  sink_index)
{
    /* check the connection of the endpoint */

    /* input_endpoint [index] <==> effect_module */
    if (source->type == MODULE_ENDPOINT_INPUT) {
        chain->input_buffer[src_index] = get_available_input_buffer(src_index);
        sink->input_buffer[sink_index] = chain->input_buffer[src_index];
        chain->input_ch++;
    }
    /* effect_module <==> output_endpoint [index] */
    else if (sink->type == MODULE_ENDPOINT_OUTPUT) {
        chain->output_buffer[sink_index] = source->buffer[src_index];
        chain->output_ch++;
    }
    /* module <==> module */
    else {
        sink->input_buffer[sink_index] = source->buffer[src_index];
    }
    printf("\tattached: %20s.[%d] => %s.[%d]\n", source->name, src_index,
                                                 sink->name, sink_index);
}

/**
 * @brief building pipelines according to configuration
 *
 * @param chain indicate audio chain
 * @param config indicate pipeline connection configurations
 * @param pipe_num indicate pipeline connection number
*/
void pipeline_connect(audio_chain_t *chain, pipeline_config_t *config, int pipe_num)
{
    for(int i = 0; i < pipe_num; i++) {
        // add effect module which used in pipeline
        audio_effect_module_t *source = add_module(chain, config[i].source);
        audio_effect_module_t *sink = add_module(chain, config[i].sink);

        if (source == NULL || sink == NULL) {
            printf("%s | source/sink unvaliable, pipline index[%d]\n", __func__, i);
            return;
        }
        effect_module_attach(chain,
                            source, config[i].source_index,
                            sink,   config[i].sink_index);
    }

#if 1
    dump_pipelines();
#endif
}


/**
 * @brief get the input buffer of effect module
 *
 * @param module indicate audio effect module
 * @param ch indicate the input buffer index
 * @return input buffer address if successful. NULL if failed.
*/
void *module_get_input_buffer(audio_effect_module_t *module, uint8_t ch)
{
    return module->input_buffer[ch];
}


/**
 * @brief get the output buffer of effect module
 *
 * @param module indicate audio effect module
 * @param ch indicate the output buffer index
 * @return output buffer address if successful. NULL if failed.
*/
void *module_get_output_buffer(audio_effect_module_t *module, uint8_t ch)
{
    return module->buffer[ch];
}


/**
 * @brief audio effect module processing
 *
 * @param chain indicate audio chain
*/
static void audio_chain_process(audio_chain_t *chain)
{
    for (int i = 0; i < chain->effects_num; i++) {
        struct audio_effect_module *module = chain->module_list[i];
        if(module->type == MODULE_ENDPOINT_INPUT)
            continue;
        if(module->type == MODULE_ENDPOINT_OUTPUT)
            continue;
        if(module->process == NULL) {
            printf("%s doesn't has process function\n", module->name);
            continue;
        }
        // printf("%s processing\n", module->name);
        module->process((void *)module);
    }
}

/**
 * @brief get the audio chain
 *
 * @param null
 * @return audio chain point
*/
audio_chain_t* get_audio_chain()
{
    return &audio_chain;
}


/**
 * @brief initialize audio chain
 *
 * @param module indicate audio chain
*/
void init_audio_chain(audio_chain_t *chain)
{
    chain->effects_num = 0;
    chain->input_ch = 0;
    chain->output_ch = 0;
    memset(chain->module_list, 0, sizeof(chain->module_list));
}

/**
 * @brief copy pcm frame from float to int16_t
 *
 * @param dest destination buffer
 * @param src source buffer
*/
static inline void copy_to_int16(int16_t *dest, float *src )
{
    for(int i = 0; i < PERIOD_FRAME; i++) {
        *(dest + i) = *(src + i);
    }
}

/**
 * @brief copy pcm frame from int16_t to float
 *
 * @param dest destination buffer
 * @param src source buffer
*/
static inline void copy_to_float(float *dest, int16_t *src)
{
    for(int i = 0; i < PERIOD_FRAME; i++) {
        *(dest + i) = *(src + i);
    }
}

/**
 * @brief dsp framework processing function
 *      Copy the pcm from the input pp buffer first,
 *      Then opearation effect processing in the audio chain.
 *      Finally, copy the pcm to the output pp buffer.
 *
 * @param null
*/
void dsp_effects_process()
{
    audio_chain_t *chain = get_audio_chain();
    // prepare the input buffer
    for(int i = 0; i < chain->input_ch; i++) {
        copy_to_float(chain->input_buffer[i], get_input_pp_buffer(hw_input_map[i]));
    }
    // chain processing
    audio_chain_process(chain);

    // prepare the output buffer after chan processing
    for(int i = 0; i < chain->output_ch; i++) {
        copy_to_int16(get_output_pp_buffer(hw_output_map[i]), chain->output_buffer[i]);
    }
}
