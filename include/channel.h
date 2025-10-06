#ifndef CHANNEL_H
#define CHANNEL_H

typedef struct {
    double power_dbw;     // Direct path power
    double multipath_delays[3];  // Up to 3 multipath delays (s)
    double multipath_powers[3];  // Relative powers (dB)
    double multipath_phases[3];  // Initial phases (rad)
    int num_paths;        // Number of multipath components
    double cn0_db;        // Carrier-to-noise density ratio
} channel_params_t;

/* Apply channel effects to complex baseband samples */
void apply_channel_effects(double *i_samples, double *q_samples, int num_samples,
                         double sample_rate, const channel_params_t *params);

#endif /* CHANNEL_H */
