

#include "audio_effect_core.h"
#include "audio_effect_buffers.h"
#include "audio_basic_effects.h"

static audio_chain_t audio_chain;
static audio_effect_module_t effect_module_list[MAX_EFFECTS_NUM] = {0};
static uint8_t module_num = 0;

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
    for(int i = 0; i < chain->input_num; i++) {
        printf(" [%d] = %p", i, chain->input_buffer[i]);
    }

    printf("\nchain_output_buffer: \t");
    for(int i = 0; i < chain->output_num; i++) {
        printf(" [%d] = %p", i, chain->output_buffer[i]);
    }

    for(int i = 0; i < chain->effects_num; i++ ) {
        audio_effect_module_t *module = chain->module_list[i];
        printf("\nmodule: order[%d] %s\n", module->order, module->name);

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
    if (module_num >= MAX_EFFECTS_NUM) {
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
    if (config->order > MAX_EFFECTS_NUM) {
        printf("order [%d] exceed max num\n", config->order);
        return NULL;
    }

    /* dupilcate order, just return the module instance */
    if (NULL != chain->module_list[config->order]) {
        return chain->module_list[config->order];
    }

    /* allocate effect module instance from the global module list */
    audio_effect_module_t *module = get_available_module();
    if (module == NULL) {
        printf("fail to allocate module %s instance\n", config->name);
        return NULL;
    }
    /* save module instance into the chain */
    chain->module_list[config->order] = module;
    // chain->effects_num++;

    module->order = config->order;
    module->type = config->type;
    module->name = config->name;

    /* the endpoint type modules are specially, that there is no real effect
     * process function. just setup the slot mapping.
     */
    if (config->type == MODULE_ENDPOINT_INPUT) {
        memcpy(chain->hw_input_map, config->default_conf, MAX_INPUT_NUM);
        printf("%s| order=%02d %s\n", __func__, config->order, config->name);
        return module;
    }
    else if (config->type == MODULE_ENDPOINT_OUTPUT) {
        memcpy(chain->hw_output_map, (uint8_t *)config->default_conf, MAX_OUTPUT_NUM);
        printf("%s| order=%02d %s\n", __func__, config->order, config->name);
        return module;
    }

    module->input_num = config->input_ch;
    module->output_num = config->output_ch;
    module->process = get_module_process(config->type);

    /* calculate coefficient size */
    int bands = 0;
    if (config->type == MODULE_EFFECT_EQUALIZER) {
        biquad_config_t *biquad = config->default_conf;
        bands = biquad->band_num;
    }

    int param_size = get_module_coeff_size(config->type, config->input_ch, bands);
    if (param_size > 0) {
        module->param_size = param_size;
        module->params = get_available_coefficent(param_size);
    }

    /* allocate the effect module buffer that use to store the processed pcm data */
    for (int i = 0; i < config->output_ch; i++) {
        module->buffer[i] = get_available_module_buffer();
    }

    module->standby = true;

    /* get the effect module init function and invoke init. */
    module_init_func_t effect_init = get_module_init(config->type);
    if (effect_init != NULL && config->default_conf != NULL) {
        effect_init(module, config->default_conf);
    }
    printf("%s| order=%02d %s\n", __func__, config->order, config->name);

    return module;
}

/**
 * @brief get the effect module from audio chain according to operation order
 *
 * @param chain indicate audio chain
 * @param order indicate operation order.
 * @return effect module address if successful. NULL if failed.
 */
audio_effect_module_t* get_module_from_order(audio_chain_t *chain, uint8_t order)
{
    if (order > chain->effects_num)
        return NULL;

    return chain->module_list[order];
}

/**
 * @brief get the effect module from audio chain according to name
 *
 * @param chain indicate audio chain
 * @param name indicate module name.
 * @return effect module address if successful. NULL if failed.
 */
audio_effect_module_t* get_module_from_name(audio_chain_t *chain, const char *name)
{
    for (int i = ORDER_OFFSET; i < chain->effects_num; i++) {
        audio_effect_module_t* module = chain->module_list[i];
        if (strcmp(name, module->name) == 0) {
            return module;
        }
    }
    return NULL;
}

/**
 * @brief attach effect modules
 *  input_endpoint: [0]        [1]       [2]       [3]
 *                   |          |         |         |
 *                   |          |         |         |
 *      effects:  [module1] [module2] [module3] [module4]
 *                   |          |         |_________|
 *                   |          |              |
 *  output_endpoint [1]        [3]            [0]
 *
 * @param chain the point of audio_chain_t
 * @param source the point of source audio_effect_module_t
 * @param src_index the index of buffer in source audio_effect_module_t
 * @param sink the point of sink audio_effect_module_t
 * @param sink_index the index of sink effect module
 */
static void effect_module_attach(audio_chain_t *chain,
                    audio_effect_module_t *source, uint8_t src_index,
                    audio_effect_module_t *sink, uint8_t  sink_index)
{
    /* check the connection of the endpoint */
    /* - for input endpoint:
    *    src_index : identify the slot id of the input_buffer in audio chain;
    *    sink_index: identify the channel id of input_buffer in sink audio_effect_module_t,
    *                that use to receive the pcm from the input endpoint;
    *  - for output endpoint:
    *    src_index : identify the channel id of the buffer in source audio_effect_module_t;
    *    sink_index: identify the slot id of the output_buffer in audio chain, that use to output
    *                the pcm from the audio_effect_module_t;
    *  - for effect modules:
    *    src_index : identify the channel id of the buffer in source audio_effect_module_t;
    *    sink_index: identify the channel id of the input_buffer in sink audio_effect_module_t;
    */
    /* input_endpoint [src_index] ==> effect_module[sink_index] */
    if (source->type == MODULE_ENDPOINT_INPUT) {
        /* while the source type is input endpoint, allocate the input_buffer according the
         * src_index, the audio chain should has the input_buffer[] to store the input pcm.
         */
        chain->input_buffer[src_index] = get_available_input_buffer(src_index);

        /* the input_buffer of audio_effect_module_t points to the input buffer of audio chan */
        sink->input_buffer[sink_index] = chain->input_buffer[src_index];
        chain->input_num++;
    }
    /* effect_module[src_index] ==> output_endpoint [sink_index] */
    else if (sink->type == MODULE_ENDPOINT_OUTPUT) {
        chain->output_buffer[sink_index] = source->buffer[src_index];
        chain->output_num++;
    }
    /* module[src_index] ==> module[sink_index] */
    else {
        sink->input_buffer[sink_index] = source->buffer[src_index];
    }

    printf("\tattached: %20s[%d] => %s[%d]\n", source->name, src_index,
                                                 sink->name, sink_index);
}

/**
 * @brief check the module is belongs to input source or pcm generator
 *
 * @param type input module type
 * @return true
 * @return false
 */
static bool inline is_source_module(DSP_MODULE_TYPE type)
{
    if (type == MODULE_ENDPOINT_INPUT ||
        type == MODULE_GENERATOR_TONE ||
        type == MODULE_GENERATOR_DTMF ||
        type == MODULE_GENERATOR_WAVPLAYER) {
        return true;
    }
    return false;
}

/**
 * @brief bind all effect modules according to pipeline config
 *
 * @param chain the point of audio_chain_t instance
 * @param pipelines the point of pipeline_config_t instance
 * @param pipe_num identify the pipeline number
 * @return int EFFECT_NO_ERR or EFFECT_PIPELINE_ERR
 */
static int inline bind_module_connections(audio_chain_t *chain,
                            pipeline_config_t *pipelines, int pipe_num)
{
    for (int i = 0; i < pipe_num; i++) {
        module_config_t *source = pipelines[i].source;
        module_config_t *sink = pipelines[i].sink;

        if (true == is_source_module(source->type)) {
            chain->entry_modules[pipelines[i].source_index] = sink;
            chain->entry_num++;
            printf("entry module[%d] = %s\n", pipelines[i].source_index, sink->name);
        }
        // printf("link module|[%s] => %s\n", pipelines[i].source->name, pipelines[i].sink->name);
        uint8_t src_index = pipelines[i].source_index;
        if (source->output_cnt[src_index] < MAX_COCURRENT_OUTPUT) {
            source->next[src_index][source->output_cnt[src_index]++] = sink;
            sink->last[pipelines[i].sink_index] = source;
        }
        else {
            printf("%d over the max cocurrent output of %s\n",
                        source->output_cnt[src_index], source->name);
            return EFFECT_PIPELINE_ERR;
        }
    }
    printf("\n");
    return EFFECT_NO_ERR;
}



/**
 * @brief check all input modules has assigned order or not
 *
 * @param module point of module_config_t
 * @return true, all input modules has ordered
 * @return false
 */
static bool check_all_inputs_has_ordered(module_config_t *module)
{
    for (int i = 0; i < module->input_ch; i++) {
        module_config_t *prev = module->last[i];

        if (prev != NULL && prev->order == 0 &&
                            prev->type != MODULE_ENDPOINT_INPUT) {

            printf("current [%s].last[%d] = module[%s] has not assign order\n",
                    module->name, i, prev->name);
            return false;
        }
    }
    return true;
}

/**
 * @brief check and allocate order
 *
 * @param chain point of audio_chain_t
 * @param module point of module_config_t
 * @return int EFFECT_PIPELINE_ERR indicate error occurs, exit dsp-fw procedure
 *   EFFECT_NO_ERR indicate the order allocate successfully or continue performing.
 */
static int check_and_allocate_order(audio_chain_t *chain, module_config_t *module)
{
    /* for input/output endpoint, should not in the module lists */
    if(module->type == MODULE_ENDPOINT_INPUT) {
        module->order = ORDER_INPUT_END_POINT;
        printf("abort order assignment due to %s is input endpoint\n", module->name);
        return EFFECT_PIPELINE_ERR;
    }
    /* the last module of pipeline must be end of output endpoint
     * if the module is, transfer to next input point
     */
    else if (module->type == MODULE_ENDPOINT_OUTPUT) {
        module->order = ORDER_OUTPUT_END_POINT;
        return EFFECT_NO_ERR;
    }
    /* there are inputs have not assign order yet,
     * stop and transfer to the next input module */
    if (check_all_inputs_has_ordered(module) == false) {
        return EFFECT_NO_ERR;
    }

    /* assign order to effect module */
    module->order = chain->effects_num++;
    // printf("assigned order | %s\t= [%d]\n", module->name, module->order);

    uint8_t output_ch = 0; //module->output_cnt
    do {
        for (uint8_t outputs = 0; outputs < module->output_cnt[output_ch]; outputs++) {
            module_config_t *next = module->next[output_ch][outputs];
            if (next == NULL || check_and_allocate_order(chain, next) != 0) {
                printf("abort order assignment due to %s.next[%d] is null\n",
                                module->name, output_ch);
                return EFFECT_PIPELINE_ERR;
            }
        }
    } while (++output_ch < module->output_ch);

    return EFFECT_NO_ERR;
}

/**
 * @brief build pipelines according to configuration
 *
 * @param chain indicate audio chain
 * @param pipelines indicate pipeline connection configurations
 * @param pipe_num indicate pipeline connection number
 * @return EFFECT_NO_ERR or EFFECT_PIPELINE_ERR
 */
void pipeline_connect(audio_chain_t *chain, pipeline_config_t *pipelines, int pipe_num)
{
    /* bind module_config_t connections according to pipeline config */
    if (EFFECT_NO_ERR != bind_module_connections(chain, pipelines, pipe_num)) {
        printf("Abort order allocation while checking connections\n");
        return;
    }

    /* assigment the perform order according to previous binded connections */
    for (int i = 0; i < chain->entry_num; i++) {
        if (EFFECT_NO_ERR != check_and_allocate_order(chain, chain->entry_modules[i])) {
            printf("Abort order allocation due to pipeline config error!!!\n");
            return;
        }
    }
    printf("\n");

    /* construct the audio_effect_module_t and attach the buffers */
    for(int i = 0; i < pipe_num; i++) {
        audio_effect_module_t *source = add_module(chain, pipelines[i].source);
        audio_effect_module_t *sink   = add_module(chain, pipelines[i].sink);

        if (source == NULL || sink == NULL) {
            printf("%s | source/sink unvaliable, pipline index[%d]\n", __func__, i);
            return;
        }
        effect_module_attach(chain, source, pipelines[i].source_index,
                                    sink,   pipelines[i].sink_index);
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
 * @brief
 *
 * @param module indicate audio effect module
 * @param ch indicate the output buffer index
 * @return output buffer address if successful. NULL if failed.
 */
void *module_get_coeff(audio_effect_module_t *module)
{
    return module->params;
}

/**
 * @brief set the effect module bypass
 *
 * @param module indicate audio effect module
 * @param is_bypass bypass state
 * @return int
 */
void dsp_effects_bypass(audio_effect_module_t *module, bool is_bypass)
{
    module->is_bypass = is_bypass;
}

/**
 * @brief genernal bypass processing with same input and output number
 *
 * @param module indicate audio effect module
 * @param num AUDIO_CH_MONO or AUDIO_CH_STEREO
 */
static void effect_bypass(audio_effect_module_t *module, int num)
{
    for (int ch = 0; ch < num; ch++) {

        float *in = module_get_input_buffer(module, ch);
        float *out = module_get_output_buffer(module, ch);

        for(int i = 0; i < PERIOD_FRAME; i++) {
            out[i] = in[i];
        }
    }
}

/**
 * @brief audio effect module processing
 *
 * @param chain indicate audio chain
 */
static void audio_chain_process(audio_chain_t *chain)
{
    /* for input/outpu endpoint, there is no process function to perform */
    for (int i = ORDER_OFFSET; i < chain->effects_num; i++) {

        struct audio_effect_module *module = chain->module_list[i];

        if(module->process == NULL || module->is_bypass) {
            printf("%s bypassed\n", module->name);
            effect_bypass(module, module->output_num);
        }
        else {
            module->process((void *)module);
        }
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
    chain->effects_num = ORDER_OFFSET;
    chain->input_num = 0;
    chain->output_num = 0;
    memset(chain->module_list, 0, sizeof(chain->module_list));

    memset(effect_module_list, 0, sizeof(effect_module_list));
    module_num = 0;
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
 *   - Firstly, copy the pcm data from the input pp buffer
 *   - Then, perform all the effect process functions which attached in the audio chain.
 *   - Finally, copy the processed pcm to the output pp buffer.
 *
 * @return void
 */
void dsp_effects_process()
{
    audio_chain_t *chain = get_audio_chain();
    /* prepare the input buffer, copy from the input pp buffer to chain input buffers.
     * e.g: chain->input_buffer: [0]  [1]  [2]  [3]
     *                            |    |    |    |
     *                            |    |    |    |
     *    input_pingpang_buffer: [3]  [1]  [2]  [0]
     */
    for(int i = 0; i < chain->input_num; i++) {
        copy_to_float(chain->input_buffer[i],
                        get_input_pp_buffer(chain->hw_input_map[i]));
    }

    /* audio chain processing */
    audio_chain_process(chain);

    /* copy the output buffer after chain processed from the chain
     * output buffer to ouput pp buffers.
     * e.g: chain->output_buffer: [0]  [1]  [2]  [3]
     *                             |    |    |    |
     *                             |    |    |    |
     *    output_pingpang_buffer: [3]  [1]  [2]  [0]
     */
    for(int i = 0; i < chain->output_num; i++) {
        copy_to_int16(get_output_pp_buffer(chain->hw_output_map[i]),
                        chain->output_buffer[i]);
    }
}

/**
 * @brief update the specific coeffcients of effect module
 *
 * @param order the perform sequence order of effect module in the audio chain
 * @param new_config point to new config buffer
 * @param size config parameter size
 * @return int
 */
int dsp_effects_modify(int order, float *new_config, int size)
{
    audio_chain_t *chain = get_audio_chain();

    if (order == ORDER_INPUT_END_POINT || order == ORDER_OUTPUT_END_POINT) {
        return EFFECT_GENERAL_ERROR;
    }

    if(chain->effects_num <= order) {
        return EFFECT_INVAILD_ARG;
    }

    struct audio_effect_module *module = chain->module_list[order];

    module_modify_func_t module_modify = get_module_modify(module->type);

    if (module_modify != NULL ) {
        return module_modify(module, new_config, size);
    }

    return EFFECT_NO_ERR;
}
