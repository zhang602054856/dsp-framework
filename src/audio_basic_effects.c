#include <stdlib.h>
#include <unistd.h>
#include <math.h>

#include "platform.h"
#include "audio_effect_core.h"
#include "audio_basic_effects.h"

/*********************************************************************************/
/*                          volume / gain / mute                                 */
/*********************************************************************************/

//   dB  => coefficient
//   0db => 10^(0/20)   => 10^0      = 1
// -90db => 10^(-90/20) => 10^(-4.5) = 3.16227e-5
/**
 * @brief wire effect is one intput, and slpit to multiple outputs effect.
 *
 * @param output_num identified the output copy number
 * @return result, 0 successful.
 */
static int wire(struct audio_effect_module *module)
{
    float *in = module_get_input_buffer(module, AUDIO_CH_ID_MONO);
    for(int ch = 0; ch < module->output_num; ch++) {
        float *out = module_get_output_buffer(module, ch);
        for(int i = 0; i < PERIOD_FRAME; i++) {
            out[i] = in[i];
        }
    }
    return EFFECT_NO_ERR;
}

/* calculate gain coefficient according db value */
static int gain_setup(struct audio_effect_module *module, float gain)
{
    gain_params_t *coeff = module_get_coeff(module);
    coeff->target_gain = pow(10, gain/20);
    return EFFECT_NO_ERR;
}

static void gain_init(struct audio_effect_module *module, const gain_config_t *config)
{
    if(module->standby) {
        module->standby = false;

        gain_setup(module, config->gain);
    }
}

/**
 * @brief gain scaler effect algorithm, gain process only, without ramping
 *
 * formula: db = 20 * log10(Vout/Vin); -6.02db ==> 1/2 decay;
 *          Vout = 10^(db/20) * Vin
 *
 * @param vol coeff: 10^(db/20)
 * @return process result, 0 successful.
 */
static int gain(struct audio_effect_module *module)
{
    if(module->standby) {
        /* module doestn't initialized */
        return EFFECT_NOT_INIT;
    }

    gain_params_t *coeff = module_get_coeff(module);
    float coeff_gain = coeff->target_gain;

    // get the gain & ramp coefficient from coeff-ram.
    float *out = module_get_output_buffer(module, AUDIO_CH_ID_MONO);
    float *in = module_get_input_buffer(module, AUDIO_CH_ID_MONO);

    for(int i = 0; i < PERIOD_FRAME; i++) {
        out[i] = in[i] * coeff_gain;
    }

    return EFFECT_NO_ERR;
}

/**
 * @brief calculate the gain with exponential fade out/in.
 *
 * principle : control_db += (target_db - control_db) * ramp_time_constant
 *         ramp_time_constant = 1 - exp(-1 / (sample_rate * attack_time_s))
 *         48KHz:  10ms => 0.01s, ramp coeff = 0.002081
 *                 50ms => 0.05s, ramp coeff = 0.000417
 *                100ms => 0.1s,  ramp coeff = 0.000208
 * @param target, change current gain to target
 * @param current, current gain will be change to target
 * @param ramp_attack: ramp_time_constant for attack
 * @param ramp_decay: ramp_time_constant for decay
 * @return caculated gain db
 */
static inline float calculate_exponential_gain(float target,
                                        float current,
                                        float ramp_attack,
                                        float ramp_decay)
{
    float ramp = (target > current) ? ramp_attack : ramp_decay;
    current += (target - current) * ramp;
    return current;
}

/**
 * @brief calculate the gain with linear fade out/in.
 *
 * principle : control_db_coeff += ramp * (attack ? 1 : -1)
 *            ramp coeff: 0.1db/s => 0.1db/44100 = 2.26757e-6db per frame
 *            ramp_coeff = pow(10,  ramp_rate / (PCM_FORMAT_RATE * 20)) - 1;
 * @param target, change current gain coeff to target
 * @param current, current gain coeff will be change to target
 * @param ramp_attack: coefficient for attack rate: 1db/s;
 * @param ramp_decay: coefficient for decay rate: 1db/s
 * @return caculated gain db coefficient
 */
static inline float calculate_linear_gain(float target,
                                        float current,
                                        float ramp_attack,
                                        float ramp_decay)
{
    // decay default
    float factor = -1.0f;
    float ramp_rate = ramp_decay;

    // attack
    if (target > current) {
        factor = 1.0f;
        ramp_rate = ramp_attack;
    }

    if (fabs(target - current) > GAIN_COEFF_MIN) {
        current += ramp_rate * factor;
    }
    return current;
}

static inline float calcauted_gain_coeff(DSP_GAIN_RAMP_TYPE ramp_type,
                                        float target_gain,
                                        float cur_gain,
                                        float attack,
                                        float decay)
{
    if (ramp_type == GAIN_RAMP_LINEAR) {
        return calculate_linear_gain(target_gain, cur_gain, attack, decay);
    }
    else {
        return calculate_exponential_gain(target_gain, cur_gain, attack, decay);
    }
}

/**
 * @brief calculate the gain ramp coefficient, support linear and expontial type.
 *  for linear type: ramp_coeff = pow(10,  ramp_rate / (PCM_FORMAT_RATE * 20)) - 1;
 *  for expontial type: ramp_coeff = 1 - exp(-1000.f / (PCM_FORMAT_RATE * ramp));
 * @param ramp_type, support linear and exp, refer to DSP_GAIN_RAMP_TYPE
 * @param ramp: ramp rate; e.g: linear: 90.0 dB/s; exp: 50ms
 * @return caculated ramp coefficient
 */
static inline float calculate_gain_ramp_coeff(DSP_GAIN_RAMP_TYPE ramp_type, float ramp)
{
    if (ramp_type == GAIN_RAMP_NONE) {
        return 0.0f;
    }
    else if (ramp_type == GAIN_RAMP_LINEAR) {
        return pow(10,  ramp / (PCM_FORMAT_RATE * 20)) - 1;
    }
    // default RC slew/exponential
    else {
        return (1 - exp(-1000.f / (PCM_FORMAT_RATE * ramp)));
    }
}

/**
 * @brief calculate volume coeffecient
 * @param module the point of audio_effect_module instance
 * @param type gain ramp type, supports linear and exponential.
 * @param gain [-90 ~ 15]db, target gain => coeff: 10^(db/20)
 * @param ramp ramp parameters, determinded by ramp type:
 *             linear:      [1 ~ 255]db/s;
 *             exponential: [10 ~ 1000]ms.
 *             e.g: exp = 50ms   linear = 90.0 dB/s
 * @return int 0: setup successful, negative failure
 */
static int volume_setup(struct audio_effect_module *module,
                        DSP_GAIN_RAMP_TYPE type, float gain, float ramp)
{
    volume_params_t *coeff = module_get_coeff(module);

    coeff->target_gain = pow(10, gain/20); // coeff: 10^(db/20)
    coeff->ramp_type = type;
    coeff->attack_ramp = calculate_gain_ramp_coeff(type, ramp);

    return EFFECT_NO_ERR;
}


void volume_init(struct audio_effect_module *module, gain_config_t *config)
{
    if(module->standby) {
        module->standby = false;

        volume_params_t *coeff = module_get_coeff(module);
        // default is max volume, otherwise all pcm are muted
        // coeff->current_gain = GAIN_COEFF_MAX;
        memset(coeff->current_gain, GAIN_COEFF_MAX, module->input_num);
        volume_setup(module, config->ramp_type, config->gain, config->attack);
    }
}

int volume_modify_gain(struct audio_effect_module *module, float *params, int size)
{
    if(size > VOLUME_PARAM_SIZE) return EFFECT_INVAILD_ARG;

    float gain = params[0];
    volume_params_t *coeff = module_get_coeff(module);

    coeff->target_gain = pow(10, gain/20);

    printf("%s | set gain db %f => %f\n", __func__, gain, coeff->target_gain);
    return EFFECT_NO_ERR;
}

/**
 * @brief volume effect algorithm, audio signal change with fade out/in.
 *
 * @param module point to struct audio_effect_module instance
 * @return process result, 0 successful.
 */
static int volume(struct audio_effect_module *module)
{
    //get the gain & ramp coefficient
    volume_params_t *vol = module_get_coeff(module);
    //get coefficients
    float _gain = vol->target_gain;
    float _ramp = vol->attack_ramp;
    float _type = vol->ramp_type;

    //get the input & output buffer
    float *in[AUDIO_CH_STEREO] = {NULL};
    float *out[AUDIO_CH_STEREO] = {NULL};

    for (int ch = 0; ch < module->input_num; ch++) {
        in[ch] = module_get_input_buffer(module, ch);
        out[ch] = module_get_output_buffer(module, ch);

        for(int i = 0; i < PERIOD_FRAME; i++) {
            vol->current_gain[ch] = calcauted_gain_coeff(_type,
                                                    _gain,
                                                    vol->current_gain[ch],
                                                    _ramp,
                                                    _ramp);
            out[ch][i] = in[ch][i] * vol->current_gain[ch];
        }
    }

    return EFFECT_NO_ERR;
}
/**
 * @brief calculate mute coeffecient
 * @param mute the point of volume_params_t instance
 * @param type gain ramp type, supports linear and exponential.
 * @param mute_state true means mute the audio, vice versa
 * @param attack ramp parameter for attack
 * @param decay ramp parameter for decay
 *       both attach and decay are support two ramp type，and determined by type.
 *       - linear [1 ~ 255]db/s;
 *       - exponential [10 ~ 1000]ms.
 * @return int 0: setup successful, negative failure
 */
static int mute_setup(struct audio_effect_module *module,
                      DSP_GAIN_RAMP_TYPE type,
                      bool mute_state,
                      float attack,
                      float decay)
{
    volume_params_t *mute = module_get_coeff(module);
    // mute gain coeff
    mute->ramp_type = type;
    // transfer mute state to target_gain coefficient
    mute->target_gain = (mute_state == true) ? MUTE_COEFF : UNMUTE_COEFF;
    // attack_ramp_coeff
    mute->attack_ramp = calculate_gain_ramp_coeff(type, attack);
    // decay_ramp_coeff
    mute->decay_ramp = calculate_gain_ramp_coeff(type, decay);

    return EFFECT_NO_ERR;
}

void mute_init(struct audio_effect_module *module, mute_config_t *config)
{
    if(module->standby) {
        module->standby = false;

        volume_params_t *mute = module_get_coeff(module);
        memset(mute->current_gain, UNMUTE_COEFF, module->input_num);
        mute_setup(module, config->ramp_type, config->mute, config->attack, config->decay);
    }
}

int mute_modify_state(struct audio_effect_module *module, float *params, int size)
{
    if(size > MUTE_PARAM_SIZE) return EFFECT_INVAILD_ARG;

    volume_params_t *mute = module_get_coeff(module);

    mute->target_gain = (params[0] != 0) ? MUTE_COEFF : UNMUTE_COEFF;
    return EFFECT_NO_ERR;
}

/**
 * @brief mute effect algorithm, mute with ramp down, unmute with ramp out
 *
 * principle : ramp_time_constant = 1 - exp(-1 / (sample_rate * attack_time))
 *         48KHz:  10ms => 0.01s, ramp coeff = 0.002081
 *                 50ms => 0.05s, ramp coeff = 0.000417
 *                100ms => 0.1s,  ramp coeff = 0.000208
 *   attack/decay (target_db > control_db):
 *                 control_db += (target_db - control_db) * ramp_time_constant
 *
 * @param target gain coeff: 10^(db/20)
 * @param current gain coeff: 10^(db/20)
 * @param attack ramp up coeff: attack ramp_time_constant
 * @param decay ramp down coeff: decay ramp_time_constant
 * @return process result, 0 successful.
 */
static int mute(struct audio_effect_module *module)
{
    // get the gain & ramp coefficient
    volume_params_t *mute = module_get_coeff(module);
    //get coefficients
    float _mute_gain = mute->target_gain;
    float _attack = mute->attack_ramp;
    float _decay = mute->decay_ramp;
    float _type = mute->ramp_type;

    //get the input & output buffer
    float *in[AUDIO_CH_STEREO] = {NULL};
    float *out[AUDIO_CH_STEREO] = {NULL};

    for (int ch = 0; ch < module->input_num; ch++) {
        in[ch] = module_get_input_buffer(module, ch);
        out[ch] = module_get_output_buffer(module, ch);

        for(int i = 0; i < PERIOD_FRAME; i++) {
            mute->current_gain[ch] = calcauted_gain_coeff(_type,
                                                        _mute_gain,
                                                        mute->current_gain[ch],
                                                        _attack,
                                                        _decay);
            out[ch][i] = in[ch][i] * mute->current_gain[ch];
        }
    }

    return EFFECT_NO_ERR;
}

/*********************************************************************************/
/*                                    adder                                      */
/*********************************************************************************/

/**
 * @brief audio signal adder, the input without ramping.
 *
 * @param none
 * @return process result, 0 successful.
 */
static int adder(struct audio_effect_module *module)
{
    float *input[MAX_INPUT_NUM] = {0};
    float *out = module_get_output_buffer(module, AUDIO_CH_ID_MONO);

    for (int ch = 0; ch < module->input_num; ch++) {
        input[ch] = module_get_input_buffer(module, ch);
    }

    for (int i = 0; i < PERIOD_FRAME; i++) {
        float _mixed = 0.0f;
        for (int ch = 0; ch < module->input_num; ch++) {
            _mixed += input[ch][i];
        }
        out[i] = constrite(_mixed, PCM_SAMPLE_VAL_MAX);
    }

    return EFFECT_NO_ERR;
}

/*********************************************************************************/
/*                                    mixer                                      */
/*********************************************************************************/
/**
 * @brief calculate mixer coeffecient
 * @param module the point of audio_effect_module instance
 * @param target_db the point of gain arrary for all inputs
 * @param ramp ramp parameter for mixer, fixed to support Linear ramp type only.[1 ~ 255]db/s;
 * @return int 0: setup successful, negative failure
 */
static int mixer_setup(struct audio_effect_module *module,
                         const float *target_db, float ramp)
{
    // check if the input_num > max input num of mixer
    if (module->input_num > MAX_EFFECT_INPUT) {
        printf("input ch %d exceed MAX_EFFECT_INPUT\n", module->input_num);
        return EFFECT_GENERAL_ERROR;
    }

    mixer_params_t *mixer = module_get_coeff(module);

    for (int ch = 0; ch < module->input_num; ch++) {
        // target_gain_coeff for each input
        mixer->target_input_gain[ch] = pow(10, target_db[ch]/20);
    }

    mixer->ramp = calculate_gain_ramp_coeff(GAIN_RAMP_LINEAR, ramp);

    return EFFECT_NO_ERR;
}

void mixer_init(struct audio_effect_module *module, mixer_config_t *config)
{
    if(module->standby) {
        module->standby = false;

        mixer_params_t *coeff = module_get_coeff(module);
        memset(coeff->current_gain, GAIN_COEFF_MAX, MAX_EFFECT_INPUT); // 0db => 1.0f(coef)

        mixer_setup(module, config->input_gain, config->ramp);
    }
}

int mixer_modify_input_gain(struct audio_effect_module *module, float *params, int size)
{
    if(size > MIXER_PARAM_SIZE)
        return EFFECT_INVAILD_ARG;

    int input_ch = params[0];
    float gain = params[1];

    if (input_ch >= module->input_num)
        return EFFECT_GENERAL_ERROR;

    mixer_params_t *mixer = module_get_coeff(module);

    mixer->target_input_gain[input_ch] = pow(10, gain/20);
    return EFFECT_NO_ERR;
}

/**
 * @brief audio signal adder with ramp control.
 *
 * principle: input0 *= input0_gain;
 *            input1 *= input1_gain;
 *            output = input0 + input1;
 * @param input_gain_coeff[input_num], the target gain coeff for each input
 * @param input_curr_coeff[input_num], the current gain coeff for each input
 * @param input_ramp_coeff[2*input_num], ramping the current gain to target
 * @return process result, 0 successful.
 */
static int mixer(struct audio_effect_module *module)
{
    mixer_params_t *mixer = module_get_coeff(module);

    // get input & output buffers
    float *input[MAX_INPUT_NUM] = {0};
    float *out = module_get_output_buffer(module, AUDIO_CH_ID_MONO);

    float target_gain[MAX_EFFECT_INPUT];
    float *cur_gain[MAX_EFFECT_INPUT];

    for (int ch = 0; ch < module->input_num; ch++) {
        // get input buffer address
        input[ch] = module_get_input_buffer(module, ch);
        // input_target_gain_coeff
        target_gain[ch] = mixer->target_input_gain[ch];
        // input_current_gain_coeff
        cur_gain[ch] = &mixer->current_gain[ch];
    }
    // float ramp_coeff = module->params[module->input_num];

    for(int i = 0; i < PERIOD_FRAME; i++) {
        float _mixed = 0.0f;
        for (int ch = 0; ch < module->input_num; ch++) {
            *(cur_gain[ch]) = calcauted_gain_coeff(GAIN_RAMP_LINEAR,
                        target_gain[ch], *(cur_gain[ch]), mixer->ramp, mixer->ramp);
            _mixed += input[ch][i] * (*cur_gain[ch]);
        }
        out[i] = _mixed;
        // out[i] = constrite(_mixed, PCM_SAMPLE_VAL_MAX);
    }

    return EFFECT_NO_ERR;
}


/*********************************************************************************/
/*                                  limiter                                      */
/*********************************************************************************/

static float get_pcm_abs_max(float *pcm_buffer, int size)
{
    float maximum = 0;
    for (int i = 0; i < size; i++) {
        float _val = fabs(pcm_buffer[i]);
        if (_val > maximum) {
            maximum = _val;
        }
    }
    maximum = constrite(maximum, PCM_SAMPLE_VAL_MAX);

    return maximum;
}

static int limiter_setup(struct audio_effect_module *module,
                        float threshold, float attack, float decay)
{
    limiter_params_t *limiter = module_get_coeff(module);
    // threshold_gain_coeff
    limiter->target_gain = pow(10, threshold / 20);
    // attack_ramp_coeff
    limiter->attack_ramp = calculate_gain_ramp_coeff(GAIN_RAMP_RC_SLEW, attack);
    // decay_ramp_coeff
    limiter->decay_ramp = calculate_gain_ramp_coeff(GAIN_RAMP_RC_SLEW, decay);

    return EFFECT_NO_ERR;
}

void limiter_init(struct audio_effect_module *module, gain_config_t *config)
{
    if(module->standby) {
        module->standby = false;
        limiter_params_t *limiter = module_get_coeff(module);
        // current_gain_coeff default value is the max.
        // limiter->current_gain = GAIN_COEFF_MAX;
        memset(limiter->current_gain, GAIN_COEFF_MAX, module->input_num);

        limiter_setup(module, config->gain, config->attack, config->decay);
    }
}

int limiter_modify_threshold(struct audio_effect_module *module, float *params, int size)
{
    if(size > LIMITER_PARAM_SIZE) return EFFECT_INVAILD_ARG;

    float threshold_gain = params[0];

    limiter_params_t *limiter = module_get_coeff(module);

    limiter->target_gain = pow(10, threshold_gain/20);
    return EFFECT_NO_ERR;
}

/**
 * @brief audio signal limiter, avoid the audio signal exceed the threshold.
 *
 * @param module the point of audio_effect_module instance
 * @return process result, 0 successful.
 */
static int limiter(struct audio_effect_module *module)
{
    limiter_params_t *limiter = module_get_coeff(module);
    // get coefficients
    float _gain = limiter->target_gain;
    float _ramp = limiter->decay_ramp;

    // get runtime coefficient
    // float *cur_gain[MAX_EFFECT_INPUT] = limiter->current_gain

    // get input and output buffer
    float *in[MAX_EFFECT_INPUT] = {NULL};
    float *out[MAX_EFFECT_OUTPUT] = {NULL};

    for (int ch = 0; ch < module->input_num; ch++) {
        in[ch] = module_get_input_buffer(module, ch);
        out[ch] = module_get_output_buffer(module, ch);
    }
    // float *out = module_get_output_buffer(module, AUDIO_CH_ID_MONO);
    // float *in = module_get_input_buffer(module, AUDIO_CH_ID_MONO);

    for (int ch = 0; ch < module->input_num; ch++) {
        // get the peak gain from current input buffer
        float peak = get_pcm_abs_max(in[ch], PERIOD_FRAME);
        float peak_gain_coeff = peak / PCM_SAMPLE_VAL_MAX;

        for(int i = 0; i < PERIOD_FRAME; i++) {
            if (peak_gain_coeff > _gain) {
                // (*cur_gain) += (threshold_coeff - (*cur_gain)) * limiter->attack_ramp;
                limiter->current_gain[ch] = GAIN_COEFF_MAX;
            }
            else {
                // (*cur_gain)
                limiter->current_gain[ch] += (1.0f - limiter->current_gain[ch]/*(*cur_gain)*/) * _ramp;
            }
            out[ch][i] = in[ch][i] * limiter->current_gain[ch]/*(*cur_gain)*/;
            out[ch][i] = constrite(out[ch][i], PCM_SAMPLE_VAL_MAX);
        }
    }
    return EFFECT_NO_ERR;
}

/*********************************************************************************/
/*                                    delay                                      */
/*********************************************************************************/

/**
 * @brief Calculates delay coefficent
 *
 * @param delay_time_ms  Time constant in milliseconds
 * @param delay point to echo_params_t
 *
 * @return process result, 0 successful.
 */
static int delay_setup(struct audio_effect_module *module, float delay_time_ms)
{
    echo_params_t *delay = module_get_coeff(module);
    // calculate delay samples according time
    delay->delay_sample = PCM_FORMAT_RATE * delay_time_ms / 1000;

    printf("%s | delay times[%.2f]=>[%.2d]\n", __func__, delay_time_ms, delay->delay_sample);

    return EFFECT_NO_ERR;
}

void delay_init(struct audio_effect_module *module, delay_config_t *config)
{
    if(module->standby) {
        module->standby = false;

        echo_params_t *delay = module_get_coeff(module);
        // initial runtime coefficients
        delay->write_index = 0;
        delay->delay_buffer = get_available_delay_buffer();
        // initial default coefficients
        delay_setup(module, config->time_ms);
    }
}

/**
 * @brief audio signal delay (mono).
 *       PCM_DELAY_BUFF_SIZE must be equal to 2^N;
 * @param delay_samples, samples = delay_time (s) * sample_rate
 * @return process result, 0 successful.
 */
static int delay(struct audio_effect_module *module)
{
    // get delay coefficient
    echo_params_t *delay = module_get_coeff(module);
    int delay_sample = delay->delay_sample;

    float *out = module_get_output_buffer(module, AUDIO_CH_ID_MONO);
    float *in = module_get_input_buffer(module, AUDIO_CH_ID_MONO);

    for (int i = 0; i < PERIOD_FRAME; i++) {
        // calculate the read index for the delayed sample
        int read_index = \
            (delay->write_index + PCM_DELAY_BUFF_SIZE - delay_sample) & \
            (PCM_DELAY_BUFF_SIZE - 1);
        // get the output sample from the delay buffers
        out[i] = delay->delay_buffer[read_index];

        // slide the delay buffer
        delay->delay_buffer[delay->write_index] = in[i];

        // update write index for circular buffer
        delay->write_index = (delay->write_index + 1) & (PCM_DELAY_BUFF_SIZE - 1);
    }

    return EFFECT_NO_ERR;
}

/**
 * @brief audio signal delay with echo (mono).
 *       PCM_DELAY_BUFF_SIZE must be equal 2^N;
 * @param echo the point of echo_params_t instance, include the coefficients
 * @param delay_samples, samples = delay_time (s) * sample_rate
 * @param feedback, the coefficient of feedback, [0 ~ 1]
 * @param mix_gain, the coefficient of mix, [0 ~ 1]
 * @return int 0: setup successful, negative failure
 */
static int echo_setup(struct audio_effect_module *module,
                float delay_time_ms,
                float feedback,
                float mix_gain)
{
    echo_params_t *echo = module_get_coeff(module);
    // calculate delay samples according time
    echo->delay_sample = PCM_FORMAT_RATE * delay_time_ms / 1000;
    // feedback coefficient
    echo->feedback = feedback;
    // mix coefficient
    echo->mix_gain = mix_gain;

    printf("%s | times[%.2f]=>[%.2d]\n", __func__, delay_time_ms, echo->delay_sample);

    return EFFECT_NO_ERR;
}

void echo_init(struct audio_effect_module *module, delay_config_t *config)
{
    if(module->standby) {
        module->standby = false;

        echo_params_t *echo = module_get_coeff(module);
        // initial runtime coefficients
        echo->write_index = 0;
        echo->delay_buffer = get_available_delay_buffer();
        // initial default coefficients
        echo_setup(module, config->time_ms, config->feedback, config->mix_gain);
    }
}

int echo_modify_times(struct audio_effect_module *module, float *params, int size)
{
    if(size > DELAY_PARAM_SIZE) return EFFECT_INVAILD_ARG;

    echo_params_t *echo = module_get_coeff(module);

    float time_ms = params[0];
    // calculate delay samples according time
    echo->delay_sample = PCM_FORMAT_RATE * time_ms / 1000;

    printf("%s | new delayed times[%.2f]=>[%.2d]\n", __func__, time_ms, echo->delay_sample);
    return EFFECT_NO_ERR;
}

/**
 * @brief audio signal delay with echo (mono).
 *
 * @param module, point to effect module instance]
 * @return process result, 0 successful.
 */
static int echo(struct audio_effect_module *module)
{
    echo_params_t *echo = module_get_coeff(module);
    int delay_sample = echo->delay_sample;
    float mix_gain = echo->mix_gain;
    float feedback = echo->feedback;

    float *out = module_get_output_buffer(module, AUDIO_CH_ID_MONO);
    float *in = module_get_input_buffer(module, AUDIO_CH_ID_MONO);

    for (int i = 0; i < PERIOD_FRAME; i++) {
        // calculate the read index for the delayed sample
        int read_index = \
            (echo->write_index + PCM_DELAY_BUFF_SIZE - delay_sample) & \
            (PCM_DELAY_BUFF_SIZE - 1);

        /* get the delayed sample pcm according read index */
        float delayed_pcm = echo->delay_buffer[read_index];

        /* echo effect impl */
        out[i] = in[i] * (1.0f - mix_gain) + delayed_pcm * mix_gain;
        echo->delay_buffer[echo->write_index] = in[i] + delayed_pcm * feedback;

        // update write index for circular buffer
        echo->write_index = (echo->write_index + 1) & (PCM_DELAY_BUFF_SIZE - 1);
    }

    return EFFECT_NO_ERR;
}

/*********************************************************************************/
/*                              IIR equliazer fliter                             */
/*********************************************************************************/

/**
 * @brief calculate biquad filter coefficients
 *
 * @param coeff point of the the coefficients use for iir processing later
 * @param filter_type Filter type
 * @param gain  [-15 ~ 15] db, Filter Gain
 * @param q 	Q Factor: 0 to 16
 * @param freq  [0 ~ 96KHz] The cut off frequency of the filter
 */
static void iir_filter_generate_coeffs(iir_params_t *coeff,
                                BIQUAD_FILTER_TYPE filter_type,
                                float gain,
                                float q,
                                float freq)
{
    float omega = 2 * PI * freq / PCM_FORMAT_RATE;
    float A = pow(10, gain/40);   // Linear gain

    float sinw = sin(omega);
    float cosw = cos(omega);
    float alpha = sinw / (2 * q);
    // float sqrt_a_2 = 2.0 * sqrt(A);

    float norm = 0; // Normalize the coefficients

    switch (filter_type) {
        case BIQUAD_TYPE_LPF:
            coeff->a[0] = norm = 1.0 + alpha;
            coeff->a[1] = (-2 * cosw) / norm;
            coeff->a[2] = (1.0 - alpha) / norm;
            coeff->b[0] = (1.0 - cosw) * 0.5 / norm;
            coeff->b[1] = (1.0 - cosw) / norm;
            coeff->b[2] = coeff->b[0];
            break;

        case BIQUAD_TYPE_HPF:
            coeff->a[0] = norm = 1.0 + alpha;
            coeff->a[1] = (-2 * cosw) / norm;
            coeff->a[2] = (1.0 - alpha) / norm;
            coeff->b[0] = (1.0 + cosw) * 0.5 / norm;
            coeff->b[1] = -(1.0 + cosw) / norm;
            coeff->b[2] = coeff->b[0];
            break;

        case BIQUAD_TYPE_BPF:
            coeff->a[0] = norm = 1.0 + alpha;
            coeff->a[1] = (-2 * cosw) / norm;
            coeff->a[2] = (1.0 - alpha) / norm;
            coeff->b[0] = alpha / norm;
            coeff->b[1] = 0;
            coeff->b[2] = -1.0 * coeff->b[0];
            break;

        case BIQUAD_TYPE_NOTCH:
            coeff->a[0] = norm = 1.0 + alpha;
            coeff->a[1] = (-2 * cosw) / norm;
            coeff->a[2] = (1.0 - alpha) / norm;
            coeff->b[0] = 1.0 / norm;
            coeff->b[1] = -2.0 * cosw / norm;
            coeff->b[2] = 1.0 / norm;
            break;

        case BIQUAD_TYPE_LOW_SHELF: {
            float alpha = sinw / 2 * sqrt((A + 1/A) * (1 / 0.707 - 1) + 2);
            float beta = 2 * sqrt(A) * alpha;

            coeff->a[0] = norm = (A + 1) + (A - 1) * cosw + beta;
            coeff->a[1] = (-2 * ((A - 1) + (A + 1) * cosw)) / norm;
            coeff->a[2] = ((A + 1) + (A - 1) * cosw - beta) / norm;
            coeff->b[0] = (A * ((A + 1) - (A - 1) * cosw + beta)) / norm;
            coeff->b[1] = 2 * A * ((A - 1) - (A + 1) * cosw) / norm;
            coeff->b[2] = (A * ((A + 1) - (A - 1) * cosw - beta)) / norm;
            break;
        }
        case BIQUAD_TYPE_HIGH_SHELF: {
            float alpha = sinw / 2 * sqrt((A + 1/A) * (1 / 0.707 - 1) + 2);
            float beta = 2 * sqrt(A) * alpha;

            coeff->a[0] = norm = (A + 1) - (A - 1) * cosw + beta;
            coeff->a[1] = (2 * ((A - 1) - (A + 1) * cosw)) / norm;
            coeff->a[2] = ((A + 1) - (A - 1) * cosw - beta) / norm;
            coeff->b[0] = (A * ((A + 1) + (A - 1) * cosw + beta)) / norm;
            coeff->b[1] = -2 * A * ((A - 1) + (A + 1) * cosw) / norm;
            coeff->b[2] = (A * ((A + 1) + (A - 1) * cosw - beta)) / norm;
            break;
        }
        case BIQUAD_TYPE_PEAKING:
            coeff->a[0] = norm = 1 + alpha / A;
            coeff->a[1] = -2 * cosw / norm;
            coeff->a[2] = (1 - alpha / A) / norm;
            coeff->b[0] = (1 + alpha * A) / norm;
            coeff->b[1] = -2 * cosw / norm;
            coeff->b[2] = (1 - alpha * A) / norm;
            break;
        default:
            break;
    }

    /* backup the orignal params for next time calculation coeff */
    coeff->filter = filter_type;
    coeff->gain = gain;
    coeff->q = q;
    coeff->freq = freq;
}

static inline iir_params_t* get_biquard_coeff_buffer(struct audio_effect_module *module, int band)
{
    iir_params_t *coeff;
    coeff = module_get_coeff(module) + band * sizeof(iir_params_t);
    return coeff;
}

/**
 * @brief caculate one band iir filter coefficents according to input biquad_config_t
 *
 *   - filter_type iir filter type, refer to BIQUAD_FILTER_TYPE
 *   - gain [-15 ~ 15] db, Filter Gain
 *   - q_factor [0 ~ 16], Q Factor of the filter
 *   - center_freq [0 ~ 96KHz] The cut off frequency of the filter
 *
 * @param coeff the point of iir_params_t instance
 * @param band the band id identify the config use to setup in current biquard filter
 * @param config the point of biquad_config_t instance
 * @return int 0: setup successful, negative failure
 */
static int iir_filter_setup(struct audio_effect_module *module,
                            int band, struct band_config *config)
{
    // for (int i = 0; i < config->band_num; i++) {
        // float gain = config->bands[i].gain;
        // float q_factor = config->bands[i].q_factor;
        // float center_freq = config->bands[i].center_freq;
        // BIQUAD_FILTER_TYPE type = config->bands[i].filter_type;

        // iir_params_t *filter[config->band_num];
        // filter[i] = iir + i * sizeof(iir_params_t);

        // iir_filter_generate_coeffs(type, gain, q_factor, center_freq, filter[i]);
    // }

    iir_params_t *coeff = get_biquard_coeff_buffer(module, band);

    iir_filter_generate_coeffs( coeff,
                                config->filter_type,
                                config->gain,
                                config->q_factor,
                                config->center_freq);
    return EFFECT_NO_ERR;
}

static void iir_init(struct audio_effect_module *module, biquad_config_t *config)
{
    if(module->standby) {
        module->standby = false;

        // clear all the coefficient of module
        iir_params_t *iir = module_get_coeff(module);
        memset(iir, 0, config->band_num * sizeof(iir_params_t));

        // initial the default coefficients for each band
        for (int i = 0; i < config->band_num; i++) {
            struct band_config *band_conf = &config->bands[i];
            iir_filter_setup(module, i, band_conf);
        }
    }
}

int iir_modify_gain(struct audio_effect_module *module, float *params, int size)
{
    if(size > IIR_PARAM_SIZE) return EFFECT_INVAILD_ARG;

    int band = params[0];
    float gain = params[1];

    iir_params_t *coeff = get_biquard_coeff_buffer(module, band);

    iir_filter_generate_coeffs( coeff,
                                coeff->filter,
                                gain,
                                coeff->q,
                                coeff->freq);
    return EFFECT_NO_ERR;
}

/**
 * @brief Basic 2 order IIR fliter processing of mono/stereo pcm
 *        the input and ouput channels up to stereo.
 *
 * @param module the point of IIR module instance
 * @param out output buffer (mono or stereo)
 * @param in input buffer (mono or stereo)
 * @return void
 */
static void iir_biquad(struct audio_effect_module *module)
{
    float *in[AUDIO_CH_STEREO] = {NULL};
    float *out[AUDIO_CH_STEREO] = {NULL};

    for (int ch = 0; ch < module->input_num; ch++) {
        in[ch] = module_get_input_buffer(module, ch);
        out[ch] = module_get_output_buffer(module, ch);
    }

    int band_num = module->param_size / sizeof(iir_params_t) / module->input_num;
    iir_params_t *iir[band_num];

    for (int band = 0; band < band_num; band++) {
        iir[band] = module_get_coeff(module) + band * sizeof(iir_params_t);
        float b0 = iir[band]->b[0];
        float b1 = iir[band]->b[1];
        float b2 = iir[band]->b[2];
        float a1 = iir[band]->a[1];
        float a2 = iir[band]->a[2];

        for (int ch = 0; ch < module->input_num; ch++) {
            // iir filter process
            for (int i = 0; i < PERIOD_FRAME; i++) {
                float input = (band == 0 ? in[ch][i] : out[ch][i]);
                float output = (b0 * input +
                                b1 * iir[band]->x[ch][0] +
                                b2 * iir[band]->x[ch][1]) -
                               (a1 * iir[band]->y[ch][0] +
                                a2 * iir[band]->y[ch][1]);

                // slide the input
                iir[band]->x[ch][1] = iir[band]->x[ch][0];
                iir[band]->x[ch][0] = input;
                // slide the output
                iir[band]->y[ch][1] = iir[band]->y[ch][0];
                iir[band]->y[ch][0]= output;
                out[ch][i] = output;
            }
        }
    }
}

/**
 * @brief Get the module coefficient size
 *
 * @param type refer to DSP_MODULE_TYPE
 * @param input_ch for mixer effect, the coefficient size determined by the input number.
 *                 for others, using mono channel input.
 * @param bands for equalizer only.
 * @return int the coefficient size of the type.
 */
int get_module_coeff_size(DSP_MODULE_TYPE type, int input_ch, int bands)
{
    int coeff_size = 0;

    switch (type) {
        case MODULE_EFFECT_GAIN:
            coeff_size = sizeof(gain_params_t) * input_ch;
            break;

        case MODULE_EFFECT_VOL:
        case MODULE_EFFECT_MUTE:
        case MODULE_EFFECT_LIMITER:
            coeff_size = sizeof(volume_params_t) * input_ch;
            break;

        case MODULE_EFFECT_MIXER:
            coeff_size = sizeof(mixer_params_t) * input_ch;
            break;

        case MODULE_EFFECT_EQUALIZER:
            coeff_size = sizeof(iir_params_t) * input_ch * bands;
            break;

        case MODULE_EFFECT_DELAY:
        case MODULE_EFFECT_ECHO:
            coeff_size = sizeof(echo_params_t) * input_ch;
            break;

        default:
            coeff_size = ZERO_COEFF_SIZE;
            break;
    }

    return coeff_size;
}

/*********************************************************************************/
/*                              audio effects table                              */
/*********************************************************************************/

static void* module_process_list[MODULE_TYPE_MAX][3] = {
/*                           init             process             modify  */
/* MODULE_ENDPOINT_INPUT */  { NULL,          NULL,         NULL                      },
/* MODULE_ENDPOINT_OUTPUT */ { NULL,          NULL,         NULL                      },
/* MODULE_EFFECT_WIRE */     { NULL,          wire,         NULL                      },
/* MODULE_EFFECT_GAIN */     { gain_init,     gain,         volume_modify_gain        },
/* MODULE_EFFECT_VOL */      { volume_init,   volume,       volume_modify_gain        },
/* MODULE_EFFECT_MUTE */     { mute_init,     mute,         mute_modify_state         },
/* MODULE_EFFECT_ADDER */    { NULL,          adder,        NULL                      },
/* MODULE_EFFECT_MIXER */    { mixer_init,    mixer,        mixer_modify_input_gain   },
/* MODULE_EFFECT_EQUALIZER */{ iir_init,      iir_biquad,   iir_modify_gain           },
/* MODULE_EFFECT_LIMITER */  { limiter_init,  limiter,      limiter_modify_threshold  },
/* MODULE_EFFECT_DELAY */    { delay_init,    delay,        echo_modify_times         },
/* MODULE_EFFECT_ECHO */     { echo_init,     echo,         echo_modify_times         },
};

/**
 * @brief get the effect's process function according to type
 *
 * @param type refer to definition of DSP_MODULE_TYPE
 * @return function point address
 */
module_process_func_t get_module_process(DSP_MODULE_TYPE type)
{
    return (module_process_func_t)module_process_list[type][1];
}

module_init_func_t get_module_init(DSP_MODULE_TYPE type)
{
    return (module_init_func_t)module_process_list[type][0];
}

module_modify_func_t get_module_modify(DSP_MODULE_TYPE type)
{
    return (module_modify_func_t)module_process_list[type][2];
}
