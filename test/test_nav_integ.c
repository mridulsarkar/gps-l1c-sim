#include <stdio.h>
#include <stdint.h>
#include "l1c_seq.h"
#include "l1c_mod.h"
#include "nav_cnav.h"
#include "nav_scheduler.h"   // <-- new include

/* Helper: print a slice of bits */
static void print_bits(const uint8_t *buf, int start, int n) {
    for (int i = 0; i < n; i++) {
        printf("%d", buf[start + i]);
    }
}

int main(void) {
    uint8_t nav_bits[300];
    int boc_wave[20460]; // waveform buffer (dummy)

    // Init scheduler once
    nav_scheduler_init();

    for (int prn = 1; prn <= 3; prn++) {
        printf("\n=== PRN %d ===\n", prn);

        // Build CNAV frame directly (sanity check)
        int nav_len = nav_cnav_generate(prn, nav_bits, 300);
        if (nav_len != 300) {
            printf("Error generating CNAV frame for PRN %d\n", prn);
            continue;
        }

        // Print first 40 bits of the frame
        printf("CNAV Frame (first 40 bits): ");
        print_bits(nav_bits, 0, 40);
        printf("\n");

        // Print last 24 bits (CRC parity)
        printf("CNAV Frame (last 24 CRC bits): ");
        print_bits(nav_bits, 276, 24);
        printf("\n");

        // Convert CRC bits to integer (hex print)
        uint32_t crc_val = 0;
        for (int i = 0; i < 24; i++) {
            crc_val = (crc_val << 1) | nav_bits[276 + i];
        }
        printf("CRC-24Q parity (hex): %06X\n", crc_val);

        // --- Now simulate modulation with scheduled nav bits ---
        int boc_len = 20; // short waveform for test
        for (int i = 0; i < boc_len; i++) {
            boc_wave[i] = (i % 2 == 0) ? 1 : -1; // simple alternating +/-1
        }

        // samples_per_bit = 2 (2 waveform samples per nav bit, just for demo)
        nav_scheduler_apply_to_waveform(prn, boc_wave, boc_len, 2);

        printf("BOC waveform after NAV (first %d samples): ", boc_len);
        for (int i = 0; i < boc_len; i++) {
            printf("%2d ", boc_wave[i]);
        }
        printf("\n");
    }

    return 0;
}
