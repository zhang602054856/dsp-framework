#ifndef _AUDIO_EFFECT_MODULE_H_
#define _AUDIO_EFFECT_MODULE_H_


/* biquad fliter supported type */
typedef enum {
    BIQUAD_TYPE_LPF,
    BIQUAD_TYPE_HPF,
    BIQUAD_TYPE_BPF,
    BIQUAD_TYPE_NOTCH,
    BIQUAD_TYPE_LOW_SHELF,
    BIQUAD_TYPE_HIGH_SHELF,
    BIQUAD_TYPE_PEAKING,
    BIQUAD_TYPE_NUM
} BIQUAD_FILTER_TYPE;

typedef enum {
    GAIN_RAMP_NONE,
    GAIN_RAMP_EXPONENTIAL,
    GAIN_RAMP_RC_SLEW = GAIN_RAMP_EXPONENTIAL,
    GAIN_RAMP_LINEAR,
    GAIN_RAMP_TYPE_NUM
} DSP_GAIN_RAMP_TYPE;


typedef enum {
    MODULE_ENDPOINT_INPUT,  // whole pipelines has only on input and ouput endpoint.
    MODULE_ENDPOINT_OUTPUT, // input/output endpoint support multiple channels.

    /* start of effect module type */
    MODULE_EFFECT_WIRE,
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

    /* special modules that treat as input endpoint */
    MODULE_GENERATOR_TONE,
    MODULE_GENERATOR_DTMF,
    MODULE_GENERATOR_WAVPLAYER,

    MODULE_TYPE_MAX
} DSP_MODULE_TYPE;


/*  effect params setable size */
#define VOLUME_PARAM_SIZE               1
#define MUTE_PARAM_SIZE                 1
#define MIXER_PARAM_SIZE                2
#define LIMITER_PARAM_SIZE              1
#define DELAY_PARAM_SIZE                1
#define IIR_PARAM_SIZE                  2


/* base structure for effect module configurations */
typedef struct module_config {
    const char* name;       // module name
    DSP_MODULE_TYPE type;   // type
    uint8_t order;          // operation order
    uint8_t input_ch;       // configured support input channels
    uint8_t output_ch;      // configured support output channels
    void *default_conf;     // point to default config

    struct module_config *last[MAX_EFFECT_INPUT];
    struct module_config *next[MAX_EFFECT_OUTPUT][MAX_COCURRENT_OUTPUT];
    uint8_t output_cnt[MAX_EFFECT_OUTPUT];

} module_config_t;


/* struct for pipeline connection configurations.
*/
typedef struct pipeline_config {
    /* point to source module_config_t */
    module_config_t *source;

    /* the output index of source module_config_t */
    uint8_t source_index;

    /* point to sink module_config_t */
    module_config_t *sink;

    /* the input index of sink module_config_t */
    uint8_t sink_index;
} pipeline_config_t;


/* specific struct for volume/gain/limiter effect configurations */
typedef struct gain_config {
    /* gain ramp type */
    DSP_GAIN_RAMP_TYPE ramp_type;
    /* gain coefficient */
    float gain;
    /* gain attack ramp coefficient */
    float attack;
    /* gain decay ramp coefficient */
    float decay;
} gain_config_t;

/* specific struct for mute effect configurations */
typedef struct mute_config {
    /* mute ramp type */
    DSP_GAIN_RAMP_TYPE ramp_type;
    /* mute state  */
    bool mute;
    /* gain attack ramp coefficient */
    float attack;
    /* gain decay ramp coefficient */
    float decay;
} mute_config_t;


/* specific struct for mixer effect configurations */
typedef struct mixer_config {
    /* ramp type */
    DSP_GAIN_RAMP_TYPE ramp_type;
    /* gain coefficient for each inputs */
    float input_gain[MAX_EFFECT_INPUT];
    /* gain attack & decay ramp coefficient */
    float ramp;
} mixer_config_t;


/* specific struct for delay/echo effect configurations */
typedef struct delay_config {
    /* use to calculate deley samples */
    float time_ms;
    /* feedback factor for echo */
    float feedback;
    /* mix factor for echo */
    float mix_gain;
} delay_config_t;


/* specific struct for biquad iir effect configurations */
typedef struct biquad_config {
    /* actually band numbers of biquad filter */
    uint8_t band_num;
    /* biquad filter band config, maximum 20 bands */
    struct band_config {
        /*filter type, support peaking, low shelving, high shelving*/
        BIQUAD_FILTER_TYPE filter_type;
        float gain;         /*filter gain db*/
        float q_factor;     /*filter q factor*/
        float center_freq;  /*cut off frequence*/
    } bands[MAX_BAND_NUM];

} biquad_config_t;



#endif