#ifndef SIGNAL_H
#define SIGNAL_H

#include <stdint.h>

// L1C signal constants
#define L1C_CARRIER_HZ    1575.42e6  // L1C carrier frequency
#define L1C_CHIP_RATE_HZ  1.023e6    // L1C chip rate
#define L1C_CODE_LENGTH   10230      // chips per code period

/*
 * Generate L1C baseband signal for one satellite.
 * Returns number of complex samples generated.
 */
int generate_satellite_signal(
    int prn,                    // PRN number (1..63)
    double *i_samples,          // I channel output buffer
    double *q_samples,          // Q channel output buffer
    int num_samples,            // Number of samples to generate
    double sample_rate,         // Sampling rate (Hz)
    double code_phase,          // Initial code phase (chips)
    double carrier_phase,       // Initial carrier phase (radians)
    double doppler,             // Doppler shift (Hz)
    double power_dbw           // Signal power (dBW)
);

/*
 * Quantize floating-point I/Q samples to integers.
 * samples: Interleaved I/Q samples
 * Returns 0 on success
 */
int quantize_iq_samples(
    const double *samples,      // Input samples (I0,Q0,I1,Q1,...)
    void *output,              // Output buffer
    int num_samples,           // Number of complex samples
    int bits                   // Bits per I/Q sample
);

#endif /* SIGNAL_H */
