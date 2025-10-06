#ifndef L1C_SEQ_H
#define L1C_SEQ_H

#include <stdint.h>

/*
 * Generate the full L1C sequences (per PRN).
 *
 * prn          : 1..63
 * l1cd_out     : 10230 chips (L1CD data channel, ranging only)
 * l1cp_out     : 10230 chips (L1CP pilot channel, ranging × overlay secondary code)
 *
 * Returns 0 on success, -1 on error.
 */
int l1c_generate_seq(int prn, uint8_t *l1cd_out, uint8_t *l1cp_out);

#endif /* L1C_SEQ_H */
