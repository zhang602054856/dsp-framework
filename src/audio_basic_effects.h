
#ifndef _AUDIO_EFFECTS_H_
#define _AUDIO_EFFECTS_H_

#define constrite(val, limit)           ((val) > (limit)) ? (limit) : \
                                            ((val) < (-limit) ? (-limit) : (val))
#define PI                              3.14159265358979323846

/* coefficent size = 0 means the effect it can be running with out any setting,
 * for example:
 * effect <gain>, just changing the db to target without
 * any ramping, so there is no runtime coeff buffer to store
 * the current gain coeff and ramp coeffs.
 */
#define ZERO_COEFF_SIZE                 0   /* zero param size */

/* IIR Biquad filter param size of per band */
#define IIR_FILTER_COEFF_SIZE           3
#define IIR_FILTER_RT_COEFF_SIZE        2

// #define FIR_FILTER_ORDER                100

#define GAIN_RAMP_TIME_DEFAULT          50  //ms
#define GAIN_RAMP_TIME_MIN              10  //ms
#define GAIN_DB_DEFAULT                 0.f //db
#define GAIN_DB_MIN                     -90.f//db
#define GAIN_COEFF_MIN                  (3.1622e-5)  //-90db
#define GAIN_COEFF_MAX                  (1.0f)       //0db
#define MUTE_COEFF                       0
#define UNMUTE_COEFF                     1

/* coeffiecient structure for gain */
typedef struct {
    /* coefficient for gain */
    float target_gain;
    /* backup coefficient for gain */
    float backup_target_gain;
    /* identify the backup coeff is avaliable, TRUE means coefficients */
    bool updated;
    /* there is no runtime coeff for gain */
} gain_params_t;

/* coeffiecient structure for volume/mute */
typedef struct {
    DSP_GAIN_RAMP_TYPE ramp_type;
    /* coefficient for gain */
    float target_gain;
    /* coefficient for volume/mute attack ramp */
    float attack_ramp;
    /* coefficient for mute decay ramp only */
    float decay_ramp;
    /* runtime gain coefficient */
    float current_gain[MAX_EFFECT_INPUT];
} volume_params_t;

/* coeffiecient structure for mixer */
typedef struct {
    /* mix gain coefficient for each input */
    float target_input_gain[MAX_EFFECT_INPUT];
    /* coefficient for volume attack & decay ramp */
    float ramp;
    /* runtime gain coefficient */
    float current_gain[MAX_EFFECT_INPUT];
} mixer_params_t;

/* coeffiecient structure for delay/echo */
typedef struct {
    /* delay samples, calculated by delay time (ms)
     * delay_sample = sample_rate * delay_time_ms / 1000 */
    int delay_sample;
    /* feedback factor [0 ~ 1.0] */
    float feedback;
    /* mix factor [0 ~ 1.0] */
    float mix_gain;

    /* runtime write index in delay sample buffer */
    int write_index;
    /* point to runtime delay sample buffer */
    float *delay_buffer;
} echo_params_t;

/* coeffiecient structure for limiter */
typedef struct {
    /* coefficient for gain */
    float target_gain;
    /* coefficient for volume/mute attack ramp */
    float attack_ramp;
    /* coefficient for mute decay ramp only */
    float decay_ramp;
    /* runtime gain coefficient */
    float current_gain[MAX_EFFECT_INPUT];
} limiter_params_t;

/* mono channel coeffiecient structure for IIR biquad filter */
typedef struct {
    /* coefficients for iir filter output */
    float a[IIR_FILTER_COEFF_SIZE];
    /* coefficients for iir filter input */
    float b[IIR_FILTER_COEFF_SIZE];

    /* runtime coeff of input value */
    float x[AUDIO_CH_STEREO][IIR_FILTER_RT_COEFF_SIZE];
    /* runtime coeff of output value */
    float y[AUDIO_CH_STEREO][IIR_FILTER_RT_COEFF_SIZE];
    /* runtime coeff use to store the orignal params that use to calculate a/b coeffs */
    float filter;
    float gain;
    float q;
    float freq;
} iir_params_t;


void *module_get_input_buffer(audio_effect_module_t *module, uint8_t ch_index);
void *module_get_output_buffer(audio_effect_module_t *module, uint8_t ch_index);
float* get_available_delay_buffer();

module_process_func_t get_module_process(DSP_MODULE_TYPE type);
module_init_func_t get_module_init(DSP_MODULE_TYPE type);
module_modify_func_t get_module_modify(DSP_MODULE_TYPE type);

#endif
