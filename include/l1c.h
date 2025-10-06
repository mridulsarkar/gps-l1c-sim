#ifndef L1C_H
#define L1C_H

#include <stdint.h>

/*
 * Generate L1C PRN sequences.
 *  - prn: 1..63
 *  - ranging_out: 10230 chips (0/1), the L1CP ranging code
 *  - overlay_out: 2047 chips (0/1), the full L1CO overlay code
 *
 * Return 0 on success, -1 on error.
 */
int l1c_generate_prn(int prn, uint8_t *ranging_out, uint8_t *overlay_out);

/*
 * Generate the 2047-chip L1C overlay sequence (S1).
 *  - poly_octal : polynomial taps (octal from ICD)
 *  - init_octal : initial register (octal from ICD)
 *  - out        : output buffer (length chips)
 *  - length     : number of chips (normally 2047)
 *  - final_reg  : final register state (returned, integer)
 */
void generate_s1_overlay(int poly_octal, int init_octal,
                         uint8_t out[], int length, int *final_reg);

#endif /* L1C_H */

