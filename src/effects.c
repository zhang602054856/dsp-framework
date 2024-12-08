#include <stdlib.h>
#include <unistd.h>
#include <math.h>

#include "audio_effect_core.h"
#include "effects.h"

#define constrite(val, limit)  ((val) > (limit)) ? (limit) : ((val) < (-limit) ? (-limit) : (val))

#define GAIN_RAMP_TIME_DEFAULT      50 //ms
#define GAIN_RAMP_TIME_MIN          10 //ms
#define GAIN_DB_DEFAULT             0.f  //db
#define GAIN_DB_MIN                 -90.f//db
#define GAIN_COEFF_MIN              (3.1622e-5)  //-90db
#define PI                          3.14159265358979323846

static bool debug = false;

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
    float *in = module_get_input_buffer(module, AUDIO_CH_MONO);
    for(int ch = 0; ch < module->output_num; ch++) {
        float *out = module_get_output_buffer(module, ch);
        for(int i = 0; i < PERIOD_FRAME; i++) {
            out[i] = in[i];
        }
    }
    return 0;
}

/* calculate gain coefficient according db value */
static void gain_setup(struct audio_effect_module *module, float target_db)
{
    module->params[0] = pow(10, target_db/20);
}

/**
 * @brief gain scaler effect algorithm, gain process only, without ramping
 *
 * formula: db = 20 * log10(Vout/Vin); -6.02db ==> 1/2 decay;
 *             Vout = 10^(db/20) * Vin
 *
 * @param vol coeff: 10^(db/20)
 * @return process result, 0 successful.
 */
static int gain(struct audio_effect_module *module)
{
    if(module->standby) {
        module->standby = false;
        float db = -12.02;
        gain_setup(module, db);
    }

    float gain_coeff = module->params[0];

    // get the gain & ramp coefficient from coeff-ram.
    float *out = module_get_output_buffer(module, AUDIO_CH_MONO);
    float *in = module_get_input_buffer(module, AUDIO_CH_MONO);

    for(int i = 0; i < PERIOD_FRAME; i++) {
        out[i] = in[i] * gain_coeff;
    }

    // printf("dump volume data at %p:\n", out);
    // print_data((uint8_t *)out, PERIOD_SIZE);
    return 0;
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
 * @param ramp_type, support linear and exp
 * @param ramp: ramp rate: linear: 90.0 dB/s; exp: 50ms
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
 * @param target_db [-90 ~ 15]db, target gain
 * @param ramp ramp parameters, decided by ramp type:
 *             linear [1 ~ 255]db/s;
 *             exponential [10 ~ 1000]ms.
 * @return void
 */
void volume_setup(struct audio_effect_module *module,
                        float target_db, /*float current_db,*/ float ramp)
{
    uint8_t count = 0;
    module->params[count++] = pow(10, target_db/20);        // target_gain_coeff
    module->params[count++] = calculate_gain_ramp_coeff(module->gain_ramp_type, ramp);

    printf("volume setup | ramp[%f] target[%f]\n", module->params[1], module->params[0]);
}

/**
 * @brief volume effect algorithm, audio signal change with fade out/in.
 *
 * @param gain coeff: 10^(db/20)
 * @param ramp coeff: ramp_time_constant
 * @return process result, 0 successful.
 */
static int volume(struct audio_effect_module *module)
{
    if(module->standby) {
        module->standby = false;
        module->rt_params[0] = 1.0f;// default max volume

        //exp: ramp = 50ms   linear: ramp = 90.0 dB/s
        float ramp = (module->gain_ramp_type == GAIN_RAMP_LINEAR) ? 90.0f : 50.0f;
        volume_setup(module, 0.0f, ramp);
    }

    //get the gain & ramp coefficient from coefficient ram.
    float target_gain =  module->params[0];
    float ramp_coeff  =  module->params[1];

    //get runtime gain coefficient
    float *cur_gain   =  &module->rt_params[0];

    float *out = module_get_output_buffer(module, AUDIO_CH_MONO);
    float *in = module_get_input_buffer(module, AUDIO_CH_MONO);

    // printf("volume out = %p, in = %p\n", out, in);
    for(int i = 0; i < PERIOD_FRAME; i++) {
        *cur_gain = calcauted_gain_coeff(module->gain_ramp_type,
                                            target_gain, *cur_gain,
                                            ramp_coeff, ramp_coeff);
        out[i] = in[i] * (*cur_gain);
    }

    // debug only
    if (module->execute_count++ == 1000) {
    //    volume_setup(module, -40.f, 50.0);
    //    debug = true;
        // printf("%d\n", module->execute_count);
    }
    return 0;
}
/**
 * @brief calculate mute coeffecient
 * @param module the point of audio_effect_module instance
 * @param mute true means mute the audio, vice versa
 * @param attack ramp parameter for attack
 * @param decay ramp parameter for decay
 * both attach and decay are support two ramp type:
 *       - linear [1 ~ 255]db/s;
 *       - exponential [10 ~ 1000]ms.
 * @return void
 */
void mute_setup(struct audio_effect_module *module, bool mute, float attack, float decay)
{
    uint8_t count = 0;
    // mute gain coeff
    float target_db = (mute == true) ? GAIN_DB_MIN : GAIN_DB_DEFAULT;
    // target_gain_coeff
    module->params[count++] = pow(10, target_db/20);
    // attack_ramp_coeff
    module->params[count++] = calculate_gain_ramp_coeff(module->gain_ramp_type, attack);
    // decay_ramp_coeff
    module->params[count++] = calculate_gain_ramp_coeff(module->gain_ramp_type, decay);
}

/**
 * @brief mute effect algorithm, mute signal with fade out, unmute signal with fade in.
 *
 * principle : ramp_time_constant = 1 - exp(-1 / (sample_rate * attack_time))
 *         48KHz:  10ms => 0.01s, ramp coeff = 0.002081
 *                 50ms => 0.05s, ramp coeff = 0.000417
 *                100ms => 0.1s,  ramp coeff = 0.000208
 *         attack (target_db > control_db):
 *                 control_db += (target_db - control_db) * ramp_time_constant
 *         decay  (target_db > control_db):
 *                 control_db += (target_db - control_db) * ramp_time_constant
 * @param target gain coeff: 10^(db/20)
 * @param current gain coeff: 10^(db/20)
 * @param attack ramp coeff: attack ramp_time_constant
 * @param decay ramp coeff: decay ramp_time_constant
 * @return process result, 0 successful.
 */
static int mute(struct audio_effect_module *module)
{
    if(module->standby) {
        module->standby = false;
        module->rt_params[0] = 1.0f;       // current_gain_coeff

        mute_setup(module, false, 50, 50);
    }

    // get the gain & ramp coefficient from coeff-ram.
    float target_gain =  module->params[0];
    float attack_coeff = module->params[1];
    float decay_coeff  = module->params[2];

    float *curre_gain = &module->rt_params[0];

    float *out = module_get_output_buffer(module, AUDIO_CH_MONO);
    float *in = module_get_input_buffer(module, AUDIO_CH_MONO);

    // printf("volume out = %p, in = %p\n", out, in);
    for(int i = 0; i < PERIOD_FRAME; i++) {
        *curre_gain = calcauted_gain_coeff(module->gain_ramp_type,
                                            target_gain, (*curre_gain),
                                            attack_coeff, decay_coeff);
        out[i] = in[i] * (*curre_gain);
    }

    return 0;
}


/*********************************************************************************/
/*                                    mixer                                      */
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
    float *out = module_get_output_buffer(module, AUDIO_CH_MONO);

    for(int ch = 0; ch < module->input_num; ch++) {
        input[ch] = module_get_input_buffer(module, ch);
    }

    for(int i = 0; i < PERIOD_FRAME; i++) {
        float _mixed = 0.0f;
        for (int ch = 0; ch < module->input_num; ch++) {
            _mixed += input[ch][i];
        }
        out[i] = _mixed;
        // out[i] = constrite(_val, PCM_SAMPLE_VAL_MAX);
    }

    // printf("dump mixer data at %p:\n", out);
    // print_data((int8_t *)out, PERIOD_SIZE/8);
    return 0;
}

void mixer_setup(struct audio_effect_module *module, const float *target_db)
{
    for (int ch = 0; ch < module->input_num; ch++) {
        // input_target_gain_coeff
        module->params[ch] = pow(10, target_db[ch]/20);
    }

    module->params[module->input_num] =
        calculate_gain_ramp_coeff(module->gain_ramp_type, GAIN_RAMP_TIME_MIN);

    printf("mixer setup | ramp[%f] target[%f] cur[%f]\n",
                module->params[2], module->params[0], module->params[1]);
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
    if(module->standby) {
        module->standby = false;
        // input_current_gain_coeff
        memset(module->rt_params, 1.0f, module->rt_param_size);

        float default_db[] = {0.0f, 0.0f};
        mixer_setup(module, default_db);
    }

    float target_gain[module->input_num];
    float *cur_gain[module->input_num];

    // get input & output buffers
    float *input[MAX_INPUT_NUM] = {0};
    float *out = module_get_output_buffer(module, AUDIO_CH_MONO);

    for(int ch = 0; ch < module->input_num; ch++) {
        // get input buffer address
        input[ch] = module_get_input_buffer(module, ch);
        // input_target_gain_coeff
        target_gain[ch] = module->params[ch];
        // input_current_gain_coeff
        cur_gain[ch] = &module->rt_params[ch];
    }

    float ramp_coeff = module->params[module->input_num];

    // printf("mixer out = %p, in = %p, %p\n", out, input[0], input[1]);
    for(int i = 0; i < PERIOD_FRAME; i++) {
        float _mixed = 0.0f;
        for (int ch = 0; ch < module->input_num; ch++) {
            *(cur_gain[ch]) = calcauted_gain_coeff(module->gain_ramp_type,
                        target_gain[ch], *(cur_gain[ch]), ramp_coeff, ramp_coeff);
            _mixed += input[ch][i] * (*cur_gain[ch]);
        }
        out[i] = _mixed;
        // out[i] = constrite(_mixed, PCM_SAMPLE_VAL_MAX);
    }

    // printf("dump mixer data at %p:\n", out);
    // print_data((int8_t *)out, PERIOD_SIZE/8);
    return 0;
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

void limiter_setup(struct audio_effect_module *module,
                        float threshold, float attack, float decay)
{
    uint8_t count = 0;
    // threshold_gain_coeff
    module->params[count++] = pow(10, threshold / 20);
    // attack_ramp_coeff
    module->params[count++] = calculate_gain_ramp_coeff(GAIN_RAMP_RC_SLEW, attack);
    // decay_ramp_coeff
    module->params[count++] = calculate_gain_ramp_coeff(GAIN_RAMP_RC_SLEW, decay);

    printf("%s | threshold[%f]=>coeff[%f] [%f,%f]\n", __func__, threshold,
                module->params[0],module->params[1],module->params[2]);
}

/**
 * @brief audio signal limiter, avoid the audio signal exceed the threshold.
 *
 * @param threshold_coeff
 * @param attack_coeff
 * @param decay_coeff
 * @return process result, 0 successful.
 */
static int limiter(struct audio_effect_module *module)
{
    if(module->standby) {
        module->standby = false;
        // current_gain_coeff
        module->rt_params[0] = 1.0f;

        limiter_setup(module, -6.03f, 10, 80); //attack 10ms, decay 80ms
    }

    float threshold_coeff = module->params[0];
    float attack_coeff = module->params[1];
    float decay_coeff = module->params[2];

    float *cur_gain = &module->rt_params[0];

    float *out = module_get_output_buffer(module, AUDIO_CH_MONO);
    float *in = module_get_input_buffer(module, AUDIO_CH_MONO);

    // current_gain_coeff
    float peak = get_pcm_abs_max(in, PERIOD_FRAME);
    float peak_gain_coeff = peak / PCM_SAMPLE_VAL_MAX;

    for(int i = 0; i < PERIOD_FRAME; i++) {
        if (peak_gain_coeff > threshold_coeff) {
            // (*cur_gain) += (threshold_coeff - (*cur_gain)) * attack_coeff;
            (*cur_gain) = threshold_coeff;
        }
        else {
            (*cur_gain) += (1.0f - (*cur_gain)) * decay_coeff;
        }
        out[i] = in[i] * (*cur_gain);
        out[i] = constrite(out[i], PCM_SAMPLE_VAL_MAX);
    }

    return 0;
}

/*********************************************************************************/
/*                                    delay                                      */
/*********************************************************************************/

void delay_setup(struct audio_effect_module *module, float delay_time_ms)
{
    uint8_t count = 0;
    // calculate delay samples according time
    module->params[count++] = PCM_FORMAT_RATE * delay_time_ms / 1000;

    printf("%s | delay times[%.2f]=>[%.2f]\n", __func__, delay_time_ms, module->params[0]);
}

/**
 * @brief audio signal delay (mono).
 *       PCM_DELAY_BUFF_SIZE must be equal 2^N;
 * @param delay_samples, samples = delay_time (s) * sample_rate
 * @return process result, 0 successful.
 */
static int delay(struct audio_effect_module *module)
{
    static float *buffer = NULL;

    if(module->standby) {
        module->standby = false;

        module->rt_params[0] = 0;   // write index
        buffer = get_available_delay_buffer();
        delay_setup(module, 10); //delay 10ms
    }
    if (buffer == NULL)
        return -1;

    int delay_samples = module->params[0];
    int write_index = module->rt_params[0];

    float *out = module_get_output_buffer(module, AUDIO_CH_MONO);
    float *in = module_get_input_buffer(module, AUDIO_CH_MONO);

    for (int i = 0; i < PERIOD_FRAME; i++) {
        // Calculate the read index for the delayed sample
        int read_index = (write_index + PCM_DELAY_BUFF_SIZE - delay_samples) &
                            (PCM_DELAY_BUFF_SIZE - 1);
        out[i] = buffer[read_index];
        buffer[write_index] = in[i];
        // Update write index for circular buffer
        write_index = (write_index + 1) & (PCM_DELAY_BUFF_SIZE - 1);
    }
    // update write index into rt coefficient
    module->rt_params[0] = write_index;

    return 0;
}

void echo_setup(struct audio_effect_module *module,
                float delay_time_ms,
                float feedback,
                float mix_gain)
{
    uint8_t count = 0;
    // calculate delay samples according time
    module->params[count++] = PCM_FORMAT_RATE * delay_time_ms / 1000;
    // feedback coefficient
    module->params[count++] = feedback;
    // mix coefficient
    module->params[count++] = mix_gain;

    printf("%s | times[%.2f]=>[%.2f]\n", __func__, delay_time_ms, module->params[0]);
}

/**
 * @brief audio signal delay with echo (mono).
 *       PCM_DELAY_BUFF_SIZE must be equal 2^N;
 * @param delay_samples, samples = delay_time (s) * sample_rate
 * @param feedback, the coefficient of feedback, [0 ~ 1]
 * @param mix_gain, the coefficient of mix, [0 ~ 1]
 * @return process result, 0 successful.
 */
static int echo(struct audio_effect_module *module)
{
    static float *buffer = NULL;

    if(module->standby) {
        module->standby = false;

        module->rt_params[0] = 0;   // write index
        buffer = get_available_delay_buffer();

        //another parameters: delay 10ms 0.5, 0.1
        echo_setup(module, 5, 0.7, 0.5); //delay 5ms 0.7, 0.5
    }

    if (buffer == NULL)
        return -1;

    int   delay_samples = module->params[0];
    float feedback = module->params[1];
    float mix = module->params[2];

    int write_index = module->rt_params[0];

    float *out = module_get_output_buffer(module, AUDIO_CH_MONO);
    float *in = module_get_input_buffer(module, AUDIO_CH_MONO);

    for (int i = 0; i < PERIOD_FRAME; i++) {
        // Calculate the read index for the delayed sample
        int read_index = (write_index + PCM_DELAY_BUFF_SIZE - delay_samples) &
                            (PCM_DELAY_BUFF_SIZE - 1);
        float delayed_sample = buffer[read_index];

        out[i] = in[i] * (1.0f - mix) + delayed_sample * mix;
        buffer[write_index] = in[i] + delayed_sample * feedback;

        // Update write index for circular buffer
        write_index = (write_index + 1) & (PCM_DELAY_BUFF_SIZE - 1);
    }
    // update write index into rt coefficient
    module->rt_params[0] = write_index;

    return 0;
}


/*********************************************************************************/
/*                              FIR equliazer fliter                             */
/*********************************************************************************/

void fir_shelving_setup(struct audio_effect_module *module,
                        bool low_shelving, float cutoff_freq, float gain)
{
    float k = (low_shelving == true) ? 1.0 : -1.0;
    float *coefficients = module->params;
    int M = module->param_size - 1; // order size
    float fc = cutoff_freq / PCM_FORMAT_RATE;  // Normalized cutoff frequency
    float A = powf(10.0f, gain / 40.0f);  // Linear gain

    for (int i = 0; i <= M; i++) {
        if (i == M / 2) {
            coefficients[i] = 1 + (A - 1) * (0.5 + 0.5 * k * cos(2 * PI * fc));
        } else {
            coefficients[i] = sin(2 * PI * fc * (i - M / 2)) / (PI * (i - M / 2));
            coefficients[i] *= (0.54 - 0.46 * cos(2 * PI * i / M));  // Apply Hamming window
            coefficients[i] *= A;
        }

        printf("%s | coefficent[%d] = %f\n", __func__, i, coefficients[i]);
    }
}

void fir_low_pass_setup(struct audio_effect_module *module, float cutoff_freq)
{
    float *coefficients = module->params;
    int M = module->param_size; // order size

    float fc = cutoff_freq / PCM_FORMAT_RATE;  // Normalized cutoff frequency

    for (int i = 0; i <= M; i++) {
        if (i == M / 2) {
            coefficients[i] = 2 * fc;
        } else {
            coefficients[i] = sin(2 * PI * fc * (i - M / 2)) / (PI * (i - M / 2));
        }
        coefficients[i] *= (0.54 - 0.46 * cos(2 * PI * i / M));  // Apply Hamming window
    }
}


void fir_high_pass_setup(struct audio_effect_module *module, float cutoff_freq)
{
    float *coefficients = module->params;
    int M = module->param_size; // order size

    float fc = cutoff_freq / PCM_FORMAT_RATE;  // Normalized cutoff frequency

    for (int i = 0; i <= M; i++) {
        if (i == M / 2) {
            coefficients[i] = 1 - 2 * fc;
        } else {
            coefficients[i] = -sin(2 * PI * fc * (i - M / 2)) / (PI * (i - M / 2));
        }
        coefficients[i] *= (0.54 - 0.46 * cos(2 * PI * i / M));  // Apply Hamming window
    }
}
/**
 * @brief For an all-pass filter, the coefficients are designed to ensure that the magnitude
 * response is flat (i.e., unity gain) across all frequencies.
 *
 */
void fir_all_pass_setup(struct audio_effect_module *module)
{
   // These coefficients should be designed based on the desired phase response
    float *coefficients = module->params;
    int M = module->param_size; // order size
    for (int i = 0; i <= M; i++) {
        coefficients[i] = (i == M / 2) ? 1.0 : 0.0;
    }
}

void fir_band_pass_setup(struct audio_effect_module *module, float lower_cutoff, float upper_cutoff)
{
    float *coefficients = module->params;
    int M = module->param_size; // order size

    float lower = lower_cutoff / PCM_FORMAT_RATE;  // Normalized lower cutoff frequency
    float upper = upper_cutoff / PCM_FORMAT_RATE;  // Normalized upper cutoff frequency

    for (int i = 0; i <= M; i++) {
        if (i == M / 2) {
            coefficients[i] = 2 * (upper - lower);
        } else {
            float factor = (i - M / 2);
            coefficients[i] = (sin(2 * PI * upper * factor) -
                                sin(2 * PI * lower * factor)) / (PI * factor);
        }
        // Apply Hamming window
        coefficients[i] *= 0.54 - 0.46 * cos(2 * PI * i / M);
    }
}

void fir_band_stop_setup(struct audio_effect_module *module, float lower_cutoff, float upper_cutoff)
{
    float *coefficients = module->params;
    int M = module->param_size; // order size

    float lower = lower_cutoff / PCM_FORMAT_RATE;  // Normalized lower cutoff frequency
    float upper = upper_cutoff / PCM_FORMAT_RATE;  // Normalized upper cutoff frequency

    for (int i = 0; i <= M; i++) {
        if (i == M / 2) {
            coefficients[i] = 1 - 2 * (upper - lower);
        } else {
            float factor = (i - M / 2);
            coefficients[i] = (sin(2 * PI * upper * factor) -
                            sin(2 * PI * lower * factor)) / (PI * factor);
        }
        // Apply Hamming window
        coefficients[i] *= 0.54 - 0.46 * cos(2 * PI * i / M);
    }
}


void fir_peak_eq_setup(struct audio_effect_module *module,
                     float center_freq, float band_width, float gain)
{
    float *coefficients = module->params;
    int N = module->param_size;
    // int M = (module->param_size - 1) / 2; // order size

    // float omega_c = 2 * PI * center_freq / PCM_FORMAT_RATE;  // Normalized center frequency
    // float bw = 2 * PI * band_width / PCM_FORMAT_RATE;   // Normalized bandwidth
    // float A = powf(10.0f, gain / 20.0f);       // Linear gain
    // float sum = 0.0f;


    int M = (N - 1) / 2;
    float A = pow(10, gain / 40); // Convert gain from dB to linear scale
    float omega0 = 2 * PI * center_freq / PCM_FORMAT_RATE;
    float alpha = sin(omega0) / (2 * band_width);
    float sum = 0.0;

    for (int n = 0; n < N; n++) {
        if (n == M) {
            coefficients[n] = 1 + alpha * (A - 1);
        } else {
            coefficients[n] = sin(omega0 * (n - M)) / (PI * (n - M)) * (1 + alpha * (A - 1));
        }
        // Apply a Hamming window
        coefficients[n] *= (0.54 - 0.46 * cos(2 * PI * n / (N - 1)));
        sum += coefficients[n];
    }

    // Normalize the coefficients
    for (int n = 0; n < N; n++) {
        coefficients[n] /= sum;
    }
}


/**
 * @brief Basic FIR fliter processing
 *
 * 1. Audio Processing:
 *  o Low-Pass and High-Pass Filters: Typically, 50 to 200 taps are used.
 *    For high-quality audio applications, you might go up to 300 taps.
 *  ○ Band-Pass and Band-Stop Filters: Usually, 100 to 300 taps are sufficient.
 *    The exact number depends on the desired sharpness and attenuation.
 *  ○ Equalizers (EQ): For parametric EQs, 50 to 150 taps are common.
 *    Shelving filters might use 30 to 100 taps.
 * 2. Communications:
 *  ○ Channel Equalization: 50 to 200 taps,
 *    depending on the channel characteristics and required precision.
 *  ○ Pulse Shaping: 100 to 200 taps are typical for shaping pulses in digital communication systems.
 * @param output output buffer (mono)
 * @param input input buffer (mono)
 * @param coefficient coefficient buffer, size is filter order
 * @param filter_order The filter order determines the number of coefficients (or taps) in the FIR
 * filter, which directly affects the filter’s performance, including its frequency response,
 * transition bandwidth, and computational complexity.
 */
static void inline fir_filter(struct audio_effect_module *module)
{
    if(module->standby) {
        module->standby = false;

        fir_peak_eq_setup(module, 2000, 100, 6.02);
        // fir_shelving_setup(module, true, 400, -6.02);
        // fir_low_shelving_setup(module, 200, 0.02);
    }

    float *coefficient = module->params;
    static float state[FIR_FILTER_ORDER + 1] = {0};

    float *out = module_get_output_buffer(module, AUDIO_CH_MONO);
    float *in = module_get_input_buffer(module, AUDIO_CH_MONO);

    // fir filter process
    for (int i = 0; i < PERIOD_FRAME; i++) {
        for (int j = FIR_FILTER_ORDER; j > 0; j--) {
            state[j] = state[j-1];
        }
        state[0] = in[i];
        float _out = 0.f;
        for (int n = 0; n < module->param_size; n++) {
            _out += coefficient[n] * state[n];
        }
        out[i] = _out;
    }
}



/*********************************************************************************/
/*                              IIR equliazer fliter                             */
/*********************************************************************************/

/**
 * @brief calculate biquad filter coefficients
 *
 * @param gain  Filter Gain
 * @param q 	Q Factor: 0 to 16
 * @param freq  The cut off frequency of the filter
 * @param coeff The coefficients will be stored in the buffer
 */
static void iir_filter_generate_coeffs(BIQUARD_FILTER_TYPE filter_type,
                                float gain,
                                float q,
                                float freq,
                                void *coeff)
{
    // int count = 0;
    iir_filter_params_t *iir = (iir_filter_params_t *)coeff;

    float omega = 2 * PI * freq / PCM_FORMAT_RATE;
    float A = pow(10, gain/40);   // Linear gain

    float sinw = sin(omega);
    float cosw = cos(omega);
    float alpha = sinw / (2 * q);
    // float sqrt_a_2 = 2.0 * sqrt(A);

    float norm = 0; // Normalize the coefficients

    switch (filter_type) {
        case BIQUAD_TYPE_LPF:
            iir->a[0] = norm = 1.0 + alpha;
            iir->a[1] = (-2 * cosw) / norm;
            iir->a[2] = (1.0 - alpha) / norm;
            iir->b[0] = (1.0 - cosw) * 0.5 / norm;
            iir->b[1] = (1.0 - cosw) / norm;
            iir->b[2] = iir->b[0];
            break;

        case BIQUAD_TYPE_HPF:
            iir->a[0] = norm = 1.0 + alpha;
            iir->a[1] = (-2 * cosw) / norm;
            iir->a[2] = (1.0 - alpha) / norm;
            iir->b[0] = (1.0 + cosw) * 0.5 / norm;
            iir->b[1] = -(1.0 + cosw) / norm;
            iir->b[2] = iir->b[0];
            break;

        case BIQUAD_TYPE_BPF:
            iir->a[0] = norm = 1.0 + alpha;
            iir->a[1] = (-2 * cosw) / norm;
            iir->a[2] = (1.0 - alpha) / norm;
            iir->b[0] = alpha / norm;
            iir->b[1] = 0;
            iir->b[2] = -1.0 * iir->b[0];
            break;

        case BIQUAD_TYPE_NOTCH:
            iir->a[0] = norm = 1.0 + alpha;
            iir->a[1] = (-2 * cosw) / norm;
            iir->a[2] = (1.0 - alpha) / norm;
            iir->b[0] = 1.0 / norm;
            iir->b[1] = -2.0 * cosw / norm;
            iir->b[2] = 1.0 / norm;
            break;

        case BIQUAD_TYPE_LOW_SHELF: {
            float alpha = sinw / 2 * sqrt((A + 1/A) * (1 / 0.707 - 1) + 2);
            float beta = 2 * sqrt(A) * alpha;

            iir->a[0] = norm = (A + 1) + (A - 1) * cosw + beta;
            iir->a[1] = (-2 * ((A - 1) + (A + 1) * cosw)) / norm;
            iir->a[2] = ((A + 1) + (A - 1) * cosw - beta) / norm;
            iir->b[0] = (A * ((A + 1) - (A - 1) * cosw + beta)) / norm;
            iir->b[1] = 2 * A * ((A - 1) - (A + 1) * cosw) / norm;
            iir->b[2] = (A * ((A + 1) - (A - 1) * cosw - beta)) / norm;
            break;
        }
        case BIQUAD_TYPE_HIGH_SHELF: {
            float alpha = sinw / 2 * sqrt((A + 1/A) * (1 / 0.707 - 1) + 2);
            float beta = 2 * sqrt(A) * alpha;

            iir->a[0] = norm = (A + 1) - (A - 1) * cosw + beta;
            iir->a[1] = (2 * ((A - 1) - (A + 1) * cosw)) / norm;
            iir->a[2] = ((A + 1) - (A - 1) * cosw - beta) / norm;
            iir->b[0] = (A * ((A + 1) + (A - 1) * cosw + beta)) / norm;
            iir->b[1] = -2 * A * ((A - 1) + (A + 1) * cosw) / norm;
            iir->b[2] = (A * ((A + 1) + (A - 1) * cosw - beta)) / norm;
            break;
        }
        case BIQUAD_TYPE_PEAKING:
            iir->a[0] = norm = 1 + alpha / A;
            iir->a[1] = -2 * cosw / norm;
            iir->a[2] = (1 - alpha / A) / norm;
            iir->b[0] = (1 + alpha * A) / norm;
            iir->b[1] = -2 * cosw / norm;
            iir->b[2] = (1 - alpha * A) / norm;
            break;
        default:
            break;
    }
}

/**
 * @brief caculate the iir filter coefficents according to input biquard_params_t
 *      band_num identify the band number will be cacualate
 *      filter_type iir filter type, refer to BIQUARD_FILTER_TYPE
 *      gain [-15 ~ 15] db, Filter Gain
 *      q_factor [0 ~ 16], Q Factor of the filter
 *      center_freq [0 ~ 96KHz] The cut off frequency of the filter
 * @param module the instance point of audio_effect_module
 * @param param the instance point of biquard_params_t
 */
void iir_filter_setup(struct audio_effect_module *module, biquard_params_t *param)
{
    for (int i = 0; i < param->band_num; i++) {
        BIQUARD_FILTER_TYPE filter_type = param->bands[i].filter_type;
        float gain = param->bands[i].gain;
        float q_factor = param->bands[i].q_factor;
        float center_freq = param->bands[i].center_freq;
        float *coeff_addr = module->params + IIR_FILTER_COEFF_SIZE * i;

        iir_filter_generate_coeffs(filter_type, gain, q_factor, center_freq, coeff_addr);
    }
}

/**
 * @brief Basic 2 order IIR fliter processing
 *
 * @param output output buffer (mono)
 * @param input input buffer (mono)
 * @param iir IIR coefficient buffer, size is sizeof(iir_filter_params_t)
 * @param filter_order The filter order determines the number of coefficients (or taps) in the FIR
 * filter, which directly affects the filter’s performance, including its frequency response,
 * transition bandwidth, and computational complexity.
 */
static void inline iir_biquad_filter(struct audio_effect_module *module)
{
    if(module->standby) {
        module->standby = false;

        biquard_params_t equliazer = {
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
        iir_filter_setup(module, &equliazer);
        memset(module->rt_params, 0, module->rt_param_size);
    }

    float *out = module_get_output_buffer(module, AUDIO_CH_MONO);
    float *in = module_get_input_buffer(module, AUDIO_CH_MONO);

    int band_num = module->param_size / IIR_FILTER_COEFF_SIZE;

    for (int band = 0; band < band_num; band++) {

        iir_filter_params_t *iir =
                (iir_filter_params_t *)(module->params + band * IIR_FILTER_COEFF_SIZE);

        float *rt_params = module->rt_params + band * IIR_FILTER_RT_COEFF_SIZE;

        float y0 = rt_params[0];
        float y1 = rt_params[1];
        float x0 = rt_params[2];
        float x1 = rt_params[3];

        // iir filter process
        for (int i = 0; i < PERIOD_FRAME; i++) {
            float input = (band == 0 ? in[i] : out[i]);
            float output = (iir->b[0] * input + iir->b[1] * x0 + iir->b[2] * x1) -
                            (iir->a[1] * y0 + iir->a[2] * y1);
            x1 = x0; x0 = input;
            y1 = y0; y0= output;
            out[i] = output;
        }

        rt_params[0] = y0;
        rt_params[1] = y1;
        rt_params[2] = x0;
        rt_params[3] = x1;
    }
}


/*********************************************************************************/
/*                              audio effects table                              */
/*********************************************************************************/

static module_process_func_t module_process_list[MODULE_TYPE_MAX] = {
    NULL,               // MODULE_ENDPOINT_INPUT
    NULL,               // MODULE_ENDPOINT_OUTPUT
    (void *)gain,       // MODULE_EFFECT_GAIN
    (void *)volume,     // MODULE_EFFECT_VOL
    (void *)mute,       // MODULE_EFFECT_MUTE
    (void *)adder,      // MODULE_EFFECT_ADDER
    (void *)adder,      // MODULE_EFFECT_MIXER
    (void *)iir_biquad_filter,  // MODULE_EFFECT_EQUALIZER
    // (void *)fir_filter, // MODULE_EFFECT_EQUALIZER
    (void *)limiter,    // MODULE_EFFECT_LIMITER
    (void *)delay,      // MODULE_EFFECT_DELAY
    (void *)echo,       // MODULE_EFFECT_ECHO
};

/**
 * @brief get the effect's process function according to type
 *
 * @param type refer to definition of DSP_MODULE_TYPE
 * @return function point address
 */
module_process_func_t get_module_process(uint8_t type)
{
    return module_process_list[type];
}


