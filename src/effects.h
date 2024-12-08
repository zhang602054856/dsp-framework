
#ifndef _AUDIO_EFFECTS_H_
#define _AUDIO_EFFECTS_H_


typedef enum {
    MODULE_ENDPOINT_INPUT,
    MODULE_ENDPOINT_OUTPUT,
    MODULE_EFFECT_GAIN,
    MODULE_EFFECT_VOL,
    MODULE_EFFECT_MUTE,
    MODULE_EFFECT_ADDER,
    MODULE_EFFECT_MIXER,
    MODULE_EFFECT_EQUALIZER,
    MODULE_EFFECT_LIMITER,
    MODULE_EFFECT_DELAY,
    MODULE_EFFECT_ECHO,

    /* add new effect type at here */

    MODULE_TYPE_MAX
} DSP_MODULE_TYPE;

typedef enum {
    GAIN_RAMP_NONE,
    GAIN_RAMP_EXPONENTIAL,
    GAIN_RAMP_RC_SLEW = GAIN_RAMP_EXPONENTIAL,
    GAIN_RAMP_LINEAR,
    GAIN_RAMP_TYPE_NUM

} DSP_GAIN_RAMP_TYPE;



/* biquard fliter supported type */
typedef enum {
    BIQUAD_TYPE_LPF,
    BIQUAD_TYPE_HPF,
    BIQUAD_TYPE_BPF,
    BIQUAD_TYPE_NOTCH,
    BIQUAD_TYPE_LOW_SHELF,
    BIQUAD_TYPE_HIGH_SHELF,
    BIQUAD_TYPE_PEAKING,
    BIQUAD_TYPE_NUM
} BIQUARD_FILTER_TYPE;

typedef struct {

    float a[3]; // coefficients for output
    float b[3]; // coefficients for input

} iir_filter_params_t;

typedef struct {
    uint8_t band_num;
    struct {
        BIQUARD_FILTER_TYPE filter_type;
        float gain;
        float q_factor;
        float center_freq;
    } bands[MAX_BAND_NUM];

} biquard_params_t;


// params size of per band
#define IIR_FILTER_COEFF_SIZE           6
#define IIR_FILTER_RT_COEFF_SIZE        4
#define IIR_FILTER_BAND                 4

#define FIR_FILTER_ORDER                100

void *module_get_input_buffer(audio_effect_module_t *module, uint8_t ch_index);
void *module_get_output_buffer(audio_effect_module_t *module, uint8_t ch_index);
float* get_available_delay_buffer();

#endif
