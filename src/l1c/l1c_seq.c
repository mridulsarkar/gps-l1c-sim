#include "l1c_seq.h"
#include "l1c.h"
#include <string.h>

/*
 * Generate L1CD (data channel) and L1CP (pilot channel) sequences.
 *  - prn: 1..63
 *  - l1cd_out: 10230 chips, data channel (ranging only)
 *  - l1cp_out: 10230 chips, pilot channel (ranging × overlay)
 *
 * Returns 0 on success, -1 on error.
 */
int l1c_generate_seq(int prn, uint8_t *l1cd_out, uint8_t *l1cp_out) {
    uint8_t ranging[10230];
    uint8_t overlay[2047];   // full overlay sequence

    if (l1c_generate_prn(prn, ranging, overlay) != 0) {
        return -1;
    }

    // L1CD = ranging only
    memcpy(l1cd_out, ranging, 10230);

    // L1CP = ranging × overlay (overlay repeated to cover 10230 chips)
    for (int i = 0; i < 10230; i++) {
        l1cp_out[i] = ranging[i] ^ overlay[i % 2047];
    }

    return 0;
}
