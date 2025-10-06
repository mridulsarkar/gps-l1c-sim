#include "l1c.h"
#include <string.h>
#include <stdio.h>

/* ---------- Constants ---------- */
#define LEGENDRE_MOD 10223
static const uint8_t expansion_seq[7] = {0,1,1,0,1,0,0};

/* ---------- Tables from IS-GPS-800E ---------- */
/* Table 3.2-2: L1CP/L1CD parameters (Weil w, insertion p) */
static const int w_cp[64] = {
    0,
    5111,5109,5108,5106,5103,5101,5100,5098,5095,5094,5093,5091,5090,5081,5080,5069,5068,5054,5044,5027,5026,
    5014,5004,4980,4915,4909,4893,4885,4832,4824,4591,3706,5092,4986,4965,4920,4917,4858,4847,4790,4770,4318,
    4126,3961,3790,4911,4881,4827,4795,4789,4725,4675,4539,4535,4458,4197,4096,3484,3481,3393,3175,2360,1852
};

static const int p_cp[64] = {
    0,
    412,161,1,303,207,4971,4496,5,4557,485,253,4676,1,66,4485,282,193,5211,729,4848,982,
    5955,9805,670,464,29,429,394,616,9457,4429,4771,365,9705,9489,4193,9947,824,864,347,677,6544,
    6312,9804,278,9461,444,4839,4144,9875,197,1156,4674,10035,4504,5,9937,430,5,355,909,1622,6284
};

/* Table 3.2-3: L1CO overlay code S1 parameters (octal) */
static const int s1_poly_coef_octal[64] = {
    0,
    5111,5421,5501,5403,6417,6141,6351,6501,6205,6235,7751,6623,6733,7627,5667,5051,7665,6325,4365,4745,7633,
    6747,4475,4225,7063,4423,6651,4161,7237,4473,5477,6163,7223,6323,7125,7035,4341,4353,4107,5735,6741,7071,
    4563,5755,6127,4671,4511,4533,5357,5607,6673,6153,7565,7107,6211,4321,7201,4451,5411,5141,7041,6637,4577
};

static const int s1_init_octal[64] = {
    0,
    3266,2040,1527,3307,3756,3026,0562,0420,3415,0337,0265,1230,2204,1440,2412,3516,2761,3750,2701,1206,1544,
    1774,0546,2213,3707,2051,3650,1777,3203,1762,2100,0571,3710,3535,3110,1426,0255,0321,3124,0572,1736,3306,
    1307,3763,1604,1021,2624,0406,0114,0077,3477,1000,3460,2607,2057,3467,0706,2032,1464,0520,1766,3270,0341
};

/* ---------- Helpers ---------- */

void l1c_legendre(uint8_t L[10223]) {
    memset(L,0,10223);
    for (int x=1;x<LEGENDRE_MOD;x++) {
        int t = (int)((1LL*x*x)%LEGENDRE_MOD);
        L[t]=1;
    }
    L[0]=0;
}

static void build_weil(const uint8_t L[10223], int w, uint8_t W[10223]) {
    for (int t=0;t<LEGENDRE_MOD;t++) {
        int idx=(t+w)%LEGENDRE_MOD;
        W[t]=(L[t]^L[idx])&1;
    }
}

static void insert_expansion(const uint8_t W[10223], int p, uint8_t out[10230]) {
    int outpos = 0;

    for (int t = 0; t <= p-2; t++) {
        out[outpos++] = W[t];
    }
    for (int i = 0; i < 7; i++) {
        out[outpos++] = expansion_seq[i];
    }
    for (int t = p+6; t < 10230; t++) {
        out[outpos++] = W[t-7];
    }
}

/*
 * Generate the 2047-bit S1 overlay sequence using
 * polynomial and initial register values (ICD Table 3.2-3).
 */
void generate_s1_overlay(int poly_octal, int init_octal,
                         uint8_t out[], int length, int *final_reg)
{
    int poly = 0, init = 0;

    /* Convert octal -> binary integer */
    {
        int tmp = poly_octal, base = 1;
        while (tmp > 0) {
            int d = tmp % 10;
            poly += d * base;
            base *= 8;
            tmp /= 10;
        }
    }
    {
        int tmp = init_octal, base = 1;
        while (tmp > 0) {
            int d = tmp % 10;
            init += d * base;
            base *= 8;
            tmp /= 10;
        }
    }

    uint16_t x1 = 0x7FF;   // X1 always starts all-ones
    uint16_t x2 = init & 0x7FF;

    for (int i = 0; i < length; i++) {
        int chip = ((x1 >> 10) ^ (x2 >> 10)) & 1;
        out[i] = (uint8_t)chip;

        int fb1 = ((x1 >> 10) ^ (x1 >> 8)) & 1;
        x1 = ((x1 << 1) & 0x7FF) | fb1;

        int fb2 = 0;
        for (int b = 0; b < 11; b++) {
            if ((poly >> b) & 1) {
                fb2 ^= (x2 >> b) & 1;
            }
        }
        x2 = ((x2 << 1) & 0x7FF) | fb2;
    }

    if (final_reg) {
        *final_reg = x2;
    }
}

/* ---------- Public API ---------- */
int l1c_generate_prn(int prn, uint8_t *ranging_out, uint8_t *overlay_out) {
    if (prn < 1 || prn > 63) return -1;

    uint8_t L[10223], W[10223];
    l1c_legendre(L);
    build_weil(L, w_cp[prn], W);
    insert_expansion(W, p_cp[prn], ranging_out);

    uint8_t s1[2047];
    int final_reg = 0;

    generate_s1_overlay(s1_poly_coef_octal[prn],
                        s1_init_octal[prn],
                        s1,
                        2047,
                        &final_reg);

    memcpy(overlay_out, s1, 2047);

    return 0;
}
