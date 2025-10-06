#include "l1c_mod.h"

/* Generate BOC(1,1) waveform */
int l1c_mod_boc11(const uint8_t *chips, int *waveform) {
    int len = 10230; // one code period length
    int outpos = 0;

    for (int i = 0; i < len; i++) {
        int val = chips[i] ? 1 : -1;
        // BOC(1,1) → split into 2 half-chip intervals
        waveform[outpos++] =  val;
        waveform[outpos++] = -val;
    }
    return outpos;
}

/* Generate TMBOC waveform (pilot channel) */
int l1c_mod_tmboc(const uint8_t *chips, int *waveform) {
    int len = 10230;  // one code period
    int outpos = 0;

    // TMBOC positions for BOC(1,1) within a 33-chip block (ICD Table 3.3-1)
    int boc11_positions[4] = {1, 11, 17, 29};

    for (int i = 0; i < len; i++) {
        int val = chips[i] ? 1 : -1;

        // Determine chip index in 33-chip cycle
        int idx = (i % 33) + 1;  // 1-based index

        // Check if this index is one of the BOC(1,1) slots
        int use_boc11 = 0;
        for (int j = 0; j < 4; j++) {
            if (idx == boc11_positions[j]) {
                use_boc11 = 1;
                break;
            }
        }

        if (use_boc11) {
            // BOC(1,1): 2 half-chip intervals
            waveform[outpos++] =  val;
            waveform[outpos++] = -val;
        } else {
            // BOC(6,1): 12 half-chip intervals
            for (int k = 0; k < 12; k++) {
                int sign = (k < 6) ? val : -val;
                waveform[outpos++] = sign;
            }
        }
    }
    return outpos;
}
