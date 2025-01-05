
#include "audio_effect_buffers.h"

// PCM buffer:ports * ch * period_size
// ****L**** | ****R****
// ***CH0*** | ***CH1***
// **L8*H8** | **L8*H8**

// declared pingpang buffers;
static int8_t input_ping_buffer[HW_INPUT_PORTS * HW_INPUT_CH][PERIOD_SIZE];
static int8_t input_pang_buffer[HW_INPUT_PORTS * HW_INPUT_CH][PERIOD_SIZE];

static int8_t output_ping_buffer[HW_OUTPUT_PORTS * HW_OUTPUT_CH][PERIOD_SIZE];
static int8_t output_pang_buffer[HW_OUTPUT_PORTS * HW_OUTPUT_CH][PERIOD_SIZE];

//declared module buffers;
static float module_buffers[MAX_EFFECTS_NUM][PERIOD_FRAME];
static int module_buffer_num;

//declared input buffer of chain inputs.
static float input_pcm_buffers[MAX_INPUT_NUM][PERIOD_FRAME];

//declared coefficient data buffer
static int8_t coefficient_buffers[MAX_COEFF_NUM];
static uint32_t coeff_offset;

//declared buffer of delay/echo effects
static float delay_buffers[MAX_OUTPUT_NUM][PCM_DELAY_BUFF_SIZE];
static int delay_num;

// declared ping pang buffer tick
static bool is_ping_now;

/**
 * @brief initialize pcm buffers of dsp framework
 *
 * @param null
*/
void init_effect_buffers()
{
    coeff_offset = 0;
    module_buffer_num = 0;
    // rt_coeff_offset = 0;
    delay_num = 0;

    is_ping_now = false;

    /* cleanup all buffers at init */
    memset(input_ping_buffer, 0x0, sizeof(input_ping_buffer));
    memset(input_pang_buffer, 0x0, sizeof(input_pang_buffer));

    memset(output_ping_buffer, 0x0, sizeof(output_ping_buffer));
    memset(output_pang_buffer, 0x0, sizeof(output_pang_buffer));

    memset(module_buffers, 0x0, sizeof(module_buffers));
    memset(input_pcm_buffers, 0x0, sizeof(input_pcm_buffers));

    memset(coefficient_buffers, 0x0, sizeof(coefficient_buffers));

    memset(delay_buffers, 0x0, sizeof(delay_buffers));

    printf("PERIOD_SIZE=%d, PERIOD_FRAME=%d\n", PERIOD_SIZE, PERIOD_FRAME);
}

/**
 * @brief get input pingpang buffer according index
 *
 * @param index indicate the channel of input buffers
 * @return input pingpang buffer (mono)
*/
void* get_input_pp_buffer(uint8_t index)
{
    if (is_ping_now) {
        return (void*)&input_pang_buffer[index][0];
    }
    else {
        return (void*)&input_ping_buffer[index][0];
    }
}
/**
 * @brief copy pcm data (mono) to input pingpang buffer
 *
 * @param index indicate the channel slot of input buffer
 * @param source source pcm data
*/
void copy_to_pp_buffer(uint8_t index, void *source)
{
    if (is_ping_now) {
        memcpy(&input_pang_buffer[index][0], source, PERIOD_SIZE);
    }
    else {
        memcpy(&input_ping_buffer[index][0], source, PERIOD_SIZE);
    }
}

/**
 * @brief get output pingpang buffer according index
 *
 * @param index indicate the channel of output buffers
 * @return output pingpang buffer (mono)
*/
void* get_output_pp_buffer(uint8_t index)
{
    if (is_ping_now) {
        return (void*)&output_pang_buffer[index][0];
    }
    else {
        return (void*)&output_ping_buffer[index][0];
    }
}

/**
 * @brief copy pcm data (mono) from output pingpang buffer
 *
 * @param index indicate the channel slot of output buffer
 * @param dest destination pcm buffer
*/
void copy_from_pp_buffer(uint8_t index, void *dest)
{
    if (is_ping_now) {
        memcpy(dest, &output_pang_buffer[index][0], PERIOD_SIZE);
    }
    else {
        memcpy(dest, &output_ping_buffer[index][0], PERIOD_SIZE);
    }
}


/**
 * @brief Get the pcm buffer from the static module buffers, use it to store the processed
 * pcm data in effect module.
 * The output_num determined how many buffers needed to be allocated.
 *
 * @param null
 * @return pcm buffer (mono)
*/
float* get_available_module_buffer()
{
    if (module_buffer_num >= MAX_EFFECTS_NUM)
        return NULL;

    float *buf = &module_buffers[module_buffer_num][0];
    module_buffer_num++;
    return buf;
}

/**
 * @brief Get the coefficient buffer from static coefficient buffers
 * The coefficient buffer stores the algorithem operations coeff parameters. the size may
 * difference for different modules.
 *
 * @param size byte size of coefficient buffer that will be allocated
 * @return void the point address of coefficient buffer
*/
void* get_available_coefficent(uint32_t size)
{
    if (size == 0 || coeff_offset >= MAX_COEFF_NUM)
        return NULL;

    void* buf = &coefficient_buffers[coeff_offset];
    coeff_offset += size;
    return buf;
}

/**
 * @brief Get the input buffer of the chain according to index, it will be connected to
 * effect modules.
 *
 * @param index indicate the channel slot of input buffer
 * @return input buffer address (mono)
*/
float* get_available_input_buffer(uint8_t index)
{
    if (index > MAX_INPUT_NUM)
        return NULL;

    return &input_pcm_buffers[index][0];
}


/**
 * @brief Get the delay buffer
 * The delay buffer static allocate when compiling.
 *
 * @param size indicate the runtime coefficient number will be allocated
 * @return runtime coefficient buffer address
*/
float* get_available_delay_buffer()
{
    if (delay_num >= MAX_DELAY_NUM)
        return NULL;

    float* buf = &delay_buffers[delay_num][0];
    printf("delay buffer allocated %p\n", buf);
    delay_num++;
    return buf;
}
