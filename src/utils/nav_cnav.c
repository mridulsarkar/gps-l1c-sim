/*
 * nav_cnav.c  -- minimal safe CNAV frame generator (300-bit) with CRC-24Q.
 *
 * This implementation intentionally avoids depending on your ephem_t layout.
 * Payload bits are currently zero-filled. CRC-24Q is calculated properly.
 *
 * Later we can replace pack_payload() with a real ephemeris-to-bits packer
 * that uses your ephem_t fields. For now this compiles and integrates with
 * the rest of your simulator code.
 */

#include "nav_cnav.h"
#include <string.h>
#include <stdint.h>
#include <stdio.h>

#define CRC24Q_POLY 0x1864CFBU
#define CRC24Q_MASK 0xFFFFFFU

/* No-op storage for ephemeris pointer (kept for API compatibility) */
static void *g_ephem_ptr = NULL;
static int g_ieph_count = 0;

void nav_cnav_init(void *eph_ptr, int ieph_count)
{
    /* currently we don't use ephemeris data; store pointer for future use */
    g_ephem_ptr = eph_ptr;
    g_ieph_count = ieph_count;
}

/* compute CRC-24Q over nbits of bit-array (bits are 0/1 bytes), MSB-first */
static uint32_t crc24q_compute(const uint8_t *bits, int nbits)
{
    uint32_t crc = 0;
    for (int i = 0; i < nbits; i++) {
        uint32_t bit = bits[i] & 1U;
        uint32_t top = (crc >> 23) & 1U;
        crc = ((crc << 1) & CRC24Q_MASK) | bit;
        if (top) crc ^= CRC24Q_POLY;
    }
    return crc & CRC24Q_MASK;
}

/* nav_cnav_generate:
 *  Frame layout:
 *    0..7    : preamble 10001011
 *    8..15   : message type (0x1F)
 *    16..21  : sat id (6 bits)
 *    22..275 : payload (254 bits) -- currently zero-filled
 *    276..299: CRC-24Q over bits 0..275
 */
int nav_cnav_generate(int prn, uint8_t *frame300, int max_bits)
{
    if (!frame300 || max_bits < 300) return -1;
    if (prn < 1 || prn > 63) return -1;

    memset(frame300, 0, 300);

    const uint8_t preamble[8] = {1,0,0,0,1,0,1,1};
    for (int i = 0; i < 8; i++) frame300[i] = preamble[i];

    const int msg_type = 31; /* 0x1F */
    for (int i = 0; i < 8; i++) {
        frame300[8 + i] = ( (msg_type >> (7 - i)) & 1 ) ? 1 : 0;
    }

    /* sat id 6 bits MSB-first into bits 16..21 */
    for (int i = 0; i < 6; i++) {
        frame300[16 + i] = ((prn >> (5 - i)) & 1) ? 1 : 0;
    }

    /* payload bits 22..275 left as zeros for now (254 bits) */

    /* compute CRC over bits 0..275 (276 bits) */
    uint32_t crc = crc24q_compute(frame300, 276);

    /* write CRC bits MSB-first into bits 276..299 */
    for (int i = 0; i < 24; i++) {
        frame300[276 + i] = (crc >> (23 - i)) & 1U;
    }

    return 300;
}

/* Stateful per-PRN streaming helper */
uint8_t nav_cnav_next_bit_for_prn(int prn)
{
    #define MAX_PRN_IDX 64
    #define FRAME_BITS 300
    static uint8_t frames[MAX_PRN_IDX][FRAME_BITS];
    static int pos[MAX_PRN_IDX] = {0};
    static uint8_t inited[MAX_PRN_IDX] = {0};

    if (prn < 1) prn = 1;
    if (prn > 63) prn = 63;

    if (!inited[prn]) {
        nav_cnav_generate(prn, frames[prn], FRAME_BITS);
        pos[prn] = 0;
        inited[prn] = 1;
    }

    uint8_t b = frames[prn][ pos[prn] ];
    pos[prn]++;
    if (pos[prn] >= FRAME_BITS) {
        nav_cnav_generate(prn, frames[prn], FRAME_BITS);
        pos[prn] = 0;
    }
    return b & 1U;
}
