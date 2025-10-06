#include "channel.h"
#include <math.h>
#include <stdlib.h>

/* Box-Muller transform for Gaussian noise */
static void generate_gaussian(double *i, double *q) {
    double u1 = (double)rand()/RAND_MAX;
    double u2 = (double)rand()/RAND_MAX;
    
    double mag = sqrt(-2.0 * log(u1));
    *i = mag * cos(2.0*M_PI*u2);
    *q = mag * sin(2.0*M_PI*u2);
}

void apply_channel_effects(double *i_samples, double *q_samples, int num_samples,
                         double sample_rate, const channel_params_t *params) {
    // Compute noise power from C/N0
    double n0 = pow(10.0, -params->cn0_db/10.0);
    double sigma = sqrt(n0 * sample_rate / 2.0);
    
    // Apply multipath and noise
    for (int i = 0; i < num_samples; i++) {
        double i_direct = i_samples[i];
        double q_direct = q_samples[i];
        
        // Add multipath components
        for (int p = 1; p < params->num_paths; p++) {
            int delay_samples = (int)(params->multipath_delays[p] * sample_rate);
            if (i + delay_samples < num_samples) {
                double amp = pow(10.0, params->multipath_powers[p]/20.0);
                double phase = params->multipath_phases[p];
                
                double i_multi = amp * (i_direct*cos(phase) - q_direct*sin(phase));
                double q_multi = amp * (i_direct*sin(phase) + q_direct*cos(phase));
                
                i_samples[i + delay_samples] += i_multi;
                q_samples[i + delay_samples] += q_multi;
            }
        }
        
        // Add thermal noise
        double ni, nq;
        generate_gaussian(&ni, &nq);
        i_samples[i] += ni * sigma;
        q_samples[i] += nq * sigma;
    }
}
