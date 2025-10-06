#ifndef NAV_SCHEDULER_H
#define NAV_SCHEDULER_H

#include <stdint.h>

/*
 * nav_scheduler: simple per-PRN scheduler that produces a continuous stream
 * of CNAV frames (300 bits each). Uses nav_cnav_generate() to create each
 * new frame when needed.
 *
 * API:
 *   void nav_scheduler_init(void);
 *
 *   // Fill 'dest' with nbits nav bits for 'prn' (0/1). Returns nbits written.
 *   int nav_scheduler_get_bits(int prn, uint8_t *dest, int nbits);
 *
 *   // Convenience: apply nav bits directly to an existing ±1 waveform in-place.
 *   //   boc_wave: waveform samples (±1)
 *   //   boc_len: number of waveform samples
 *   //   samples_per_bit: number of waveform samples per nav bit
 *   //   prn: PRN whose nav bitstream to use
 *   // Returns 0 on success, -1 on error.
 *   int nav_scheduler_apply_to_waveform(int prn, int *boc_wave, int boc_len, int samples_per_bit);
 *
 */

void nav_scheduler_init(void);
int nav_scheduler_get_bits(int prn, uint8_t *dest, int nbits);
int nav_scheduler_apply_to_waveform(int prn, int *boc_wave, int boc_len, int samples_per_bit);

#endif /* NAV_SCHEDULER_H */
