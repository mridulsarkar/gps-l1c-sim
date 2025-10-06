// nav_scheduler.c
#include "nav_scheduler.h"
#include "nav_cnav.h"
#include <string.h>
#include <stdlib.h>

#define MAX_PRN 64
#define FRAME_BITS 300

/* Per-PRN state */
static uint8_t frames[MAX_PRN][FRAME_BITS];
static int frame_pos[MAX_PRN];
static int initialized[MAX_PRN];

/* Initialize scheduler (clear all state). */
void nav_scheduler_init(void) {
    memset(frames, 0, sizeof(frames));
    memset(frame_pos, 0, sizeof(frame_pos));
    memset(initialized, 0, sizeof(initialized));
}

/* Ensure that a frame exists for this PRN. */
static void ensure_frame(int prn) {
    if (prn < 1) prn = 1;
    if (prn > 63) prn = 63;

    if (!initialized[prn]) {
        nav_cnav_generate(prn, frames[prn], FRAME_BITS);
        frame_pos[prn] = 0;
        initialized[prn] = 1;
    }
}

/* Get nbits nav bits for a PRN (streamed across frames). */
int nav_scheduler_get_bits(int prn, uint8_t *dest, int nbits) {
    if (!dest || nbits <= 0) return 0;
    if (prn < 1 || prn > 63) return 0;

    ensure_frame(prn);

    int written = 0;
    while (written < nbits) {
        int remain = FRAME_BITS - frame_pos[prn];
        if (remain == 0) {
            nav_cnav_generate(prn, frames[prn], FRAME_BITS);
            frame_pos[prn] = 0;
            remain = FRAME_BITS;
        }
        int tocopy = (nbits - written < remain) ? (nbits - written) : remain;
        memcpy(&dest[written], &frames[prn][frame_pos[prn]], tocopy);
        written += tocopy;
        frame_pos[prn] += tocopy;
    }
    return written;
}

/* Apply nav bits to a waveform (±1). */
int nav_scheduler_apply_to_waveform(int prn, int *boc_wave, int boc_len, int samples_per_bit) {
    if (!boc_wave || boc_len <= 0 || samples_per_bit <= 0) return -1;
    if (prn < 1 || prn > 63) return -1;

    int needed_bits = (boc_len + samples_per_bit - 1) / samples_per_bit;
    uint8_t *navbuf = (uint8_t*)malloc(needed_bits);
    if (!navbuf) return -1;

    int got = nav_scheduler_get_bits(prn, navbuf, needed_bits);
    if (got != needed_bits) {
        free(navbuf);
        return -1;
    }

    for (int s = 0; s < boc_len; s++) {
        int b = s / samples_per_bit;
        if (navbuf[b]) {
            boc_wave[s] = -boc_wave[s];
        }
    }

    free(navbuf);
    return 0;
}
