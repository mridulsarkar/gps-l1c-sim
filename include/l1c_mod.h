#ifndef L1C_MOD_H
#define L1C_MOD_H

#include <stdint.h>

/*
 * L1C modulation functions.
 *
 * l1c_mod_boc11:
 *   Apply BOC(1,1) modulation to a chip sequence (0/1).
 *   - chips: input code sequence
 *   - waveform: output buffer (±1 waveform)
 *   - Returns number of waveform samples generated.
 *
 * l1c_mod_tmboc:
 *   Apply TMBOC modulation (pilot channel).
 *   - chips: input pilot code sequence (0/1)
 *   - waveform: output buffer (±1 waveform)
 *   - Returns number of waveform samples generated.
 */
int l1c_mod_boc11(const uint8_t *chips, int *waveform);
int l1c_mod_tmboc(const uint8_t *chips, int *waveform);

#endif /* L1C_MOD_H */
