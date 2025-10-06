#ifndef NAV_CNAV_H
#define NAV_CNAV_H

#include <stdint.h>

/*
 * Simplified CNAV interface (safe drop-in).
 *
 * nav_cnav_init:
 *   Optional initializer. Kept for API compatibility; does not require ephemeris
 *   pointers for this minimal implementation.
 *
 * nav_cnav_generate:
 *   Build a 300-bit CNAV-like frame for PRN (1..63).
 *   frame300: pointer to uint8_t buffer where bits (0/1) will be written.
 *   max_bits: must be >= 300.
 *   Returns 300 on success, -1 on error.
 *
 * nav_cnav_next_bit_for_prn:
 *   Stateful stream helper that cycles 300-bit frames per PRN.
 */
void nav_cnav_init(void *eph_ptr, int ieph_count);
int nav_cnav_generate(int prn, uint8_t *frame300, int max_bits);
uint8_t nav_cnav_next_bit_for_prn(int prn);

#endif /* NAV_CNAV_H */
