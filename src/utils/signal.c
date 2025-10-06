#include "signal.h"
#include "l1c_seq.h"
#include "l1c_mod.h"
#include "nav_scheduler.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define L1C_CHIP_RATE 1.023e6  // L1C chip rate (Hz)
#define CODE_LENGTH 10230      // L1C code length (chips)
#define DATA_TO_PILOT_POWER_RATIO 0.25  // Data channel gets 25% power

// Generate both data and pilot components for L1C
int generate_satellite_signal(
    int prn, double *i_samples, double *q_samples,
    int num_samples, double sample_rate,
    double code_phase, double carrier_phase,
    double doppler, double power_dbw)
{
    // Validate inputs
    if (!i_samples || !q_samples || num_samples <= 0 || sample_rate <= 0) {
        return -1;
    }

    // Generate L1C sequences
    uint8_t l1cd[CODE_LENGTH], l1cp[CODE_LENGTH];
    if (l1c_generate_seq(prn, l1cd, l1cp) != 0) {
        return -1;
    }

    // Get navigation data bits
    uint8_t nav_bits[1024];  // Buffer for nav bits
    int nav_len = nav_scheduler_get_bits(prn, nav_bits, 100);  // 100ms worth
    if (nav_len <= 0) return -1;

    // Generate BOC waveforms
    int *data_boc = malloc(CODE_LENGTH * 2 * sizeof(int));  // BOC(1,1)
    int *pilot_boc = malloc(CODE_LENGTH * 12 * sizeof(int)); // TMBOC
    if (!data_boc || !pilot_boc) {
        free(data_boc);
        free(pilot_boc);
        return -1;
    }

    // Apply BOC modulation
    int data_len = l1c_mod_boc11(l1cd, data_boc);
    int pilot_len = l1c_mod_tmboc(l1cp, pilot_boc);

    // Calculate power scaling
    double total_power = pow(10.0, power_dbw/10.0);
    double data_ampl = sqrt(total_power * DATA_TO_PILOT_POWER_RATIO);
    double pilot_ampl = sqrt(total_power * (1.0 - DATA_TO_PILOT_POWER_RATIO));

    // Sample generation loop
    double dt = 1.0/sample_rate;
    double code_phase_step = L1C_CHIP_RATE * dt;
    double carr_phase_step = (L1C_CARRIER_HZ + doppler) * dt;

    for (int i = 0; i < num_samples; i++) {
        // Compute code indices
        double code_time = code_phase + i * code_phase_step;
        int data_idx = ((int)code_time * 2) % data_len;
        int pilot_idx = ((int)code_time * 12) % pilot_len;

        // Get nav bit (10ms symbols)
        int nav_idx = (int)(i * dt * 100) % nav_len;
        int nav_bit = nav_bits[nav_idx] ? 1 : -1;

        // Combine components
        double code = data_ampl * data_boc[data_idx] * nav_bit +
                     pilot_ampl * pilot_boc[pilot_idx];

        // Mix to carrier
        double phase = carrier_phase + i * carr_phase_step;
        i_samples[i] = code * cos(2.0 * M_PI * phase);
        q_samples[i] = code * sin(2.0 * M_PI * phase);
    }

    free(data_boc);
    free(pilot_boc);
    return num_samples;
}

// Quantize I/Q samples to specified bit depth
int quantize_iq_samples(
    const double *samples, void *output,
    int num_samples, int bits)
{
    if (!samples || !output || bits < 1 || bits > 16) {
        return -1;
    }

    // Find maximum amplitude for scaling
    double max_amp = 0.0;
    for (int i = 0; i < num_samples * 2; i++) {
        double abs_val = fabs(samples[i]);
        if (abs_val > max_amp) max_amp = abs_val;
    }

    // Compute scaling factor
    double scale;
    int max_int;
    if (bits <= 8) {
        max_int = (1 << 7) - 1;  // 127 for 8-bit
        scale = max_int / max_amp;
        int8_t *out = (int8_t*)output;
        
        for (int i = 0; i < num_samples * 2; i++) {
            double scaled = samples[i] * scale;
            out[i] = (int8_t)fmax(fmin(scaled, max_int), -max_int);
        }
    } else {
        max_int = (1 << 15) - 1;  // 32767 for 16-bit
        scale = max_int / max_amp;
        int16_t *out = (int16_t*)output;
        
        for (int i = 0; i < num_samples * 2; i++) {
            double scaled = samples[i] * scale;
            out[i] = (int16_t)fmax(fmin(scaled, max_int), -max_int);
        }
    }

    return 0;
}