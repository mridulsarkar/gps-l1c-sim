#ifndef GPSSIM_H
#define GPSSIM_H

#include "config.h"

#define DEFAULT_SAMPLE_RATE 50.0e6  // 50 MHz
#define DEFAULT_IQ_BITS 8
#define DEFAULT_DURATION 1.0        // 1 second
#define AWGN_SEED 12345     // Random seed for noise
#define C_LIGHT 299792458.0  // Speed of light (m/s)
#define L1_FREQ 1575.42e6   // L1 frequency (Hz)

/* -----------------
 * Basic time types
 * ----------------- */
typedef struct {
    int y;   /* year */
    int m;   /* month */
    int d;   /* day */
    int hh;  /* hour */
    int mm;  /* minute */
    double sec; /* seconds */
} datetime_t;

typedef struct {
    int week;   /* GPS week number */
    double sec; /* seconds into week */
    int y;      /* year */
    int m;      /* month */
    int d;      /* day */
    int hh;     /* hour */
    int mm;     /* minute */
} gpstime_t;

typedef struct {
    char nav_file[256];
    double lat, lon, alt;          // Static position
    datetime_t start_time;
    double duration;
    double sample_rate;
    int iq_bits;
    char out_file[256];
} sim_options_t;

/* -----------------
 * Ephemeris & iono container types
 * ----------------- */
typedef struct {
    datetime_t t;
    gpstime_t toc;
    gpstime_t toe;

    double af0, af1, af2; /* SV clock correction parameters */
    double crs, deltan, m0;
    double cuc, ecc, cus, sqrta;
    double cic, omg0, cis, inc0;
    double crc, aop, omgdot, idot;

    double A;       /* semi-major axis */
    double n;       /* mean motion */
    double sq1e2;   /* sqrt(1-e^2) */
    double omgkdot; /* corrected omega dot */

    int iode;
    int iodc;
    int codeL2;
    int svhlth;

    double tgd;  /* group delay */
    int vflg;    /* valid flag */
} ephem_t;


typedef struct {
    double alpha0, alpha1, alpha2, alpha3;
    double beta0, beta1, beta2, beta3;

    double A0, A1; /* delta-UTC */
    int tot, wnt, dtls;

    int vflg; /* valid flag */
} ionoutc_t;

/* -----------------
 * Helper functions
 * ----------------- */
void date2gps(const datetime_t *t, gpstime_t *g);
double subGpsTime(gpstime_t a, gpstime_t b);

#endif /* GPSSIM_H */
