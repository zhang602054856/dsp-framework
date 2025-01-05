

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

    float *out = module_get_output_buffer(module, AUDIO_CH_ID_MONO);
    float *in = module_get_input_buffer(module, AUDIO_CH_ID_MONO);

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
