#ifndef RINEX_H
#define RINEX_H

#include "gps-l1c-sim.h"   /* contains ephem_t, ionoutc_t, datetime_t, gpstime_t, MAX_SAT, EPHEM_ARRAY_SIZE, etc */
#include <stdint.h>

/* Navigation data container */
typedef struct {
    ephem_t eph[EPHEM_ARRAY_SIZE][MAX_SAT];  /* Ephemeris array [ieph][sv] */
    ionoutc_t ion;                           /* Ionosphere parameters */
    double utc_gps[4];                       /* GPS UTC parameters */
    int n;                                   /* Number of ephemeris epochs */
} nav_t;

/* RINEX navigation message ephemeris */
typedef struct {
    int sat;            /* Satellite number */
    gpstime_t toe;      /* Time of ephemeris */
    double af0, af1;    /* SV clock bias/drift */
    double M0;          /* Mean anomaly */
    double deltan;      /* Mean motion difference */
    double ecc;         /* Eccentricity */
    double sqrta;       /* Square root of semi-major axis */
    double omega0;      /* Longitude of ascending node */
    double i0;          /* Inclination angle */
    double w;           /* Argument of perigee */
    double omegadot;    /* Rate of right ascension */
    double idot;        /* Rate of inclination angle */
    double cuc, cus;    /* Argument of latitude correction */
    double crc, crs;    /* Orbit radius correction */
    double cic, cis;    /* Inclination correction */
    int valid;          /* Valid flag */
} eph_t;

/*
 * Read RINEX navigation (broadcast) file.
 *
 * eph: output ephemeris array eph[ieph][sv] where sv in 0..MAX_SAT-1, ieph up to EPHEM_ARRAY_SIZE.
 *      caller must provide storage sized eph[EPHEM_ARRAY_SIZE][MAX_SAT]
 *
 * ionoutc: optional output ionosphere/UTC parameters (if present in header)
 *
 * fname: filename of RINEX navigation file (text)
 *
 * Returns: number of ephemeris epochs (ieph count) read on success, or -1 on error/open failure.
 */
int readRinexNavAll(ephem_t eph[][MAX_SAT], ionoutc_t *ionoutc, const char *fname);

/* Helper used by parser: convert 'D' scientific notation to 'E' for atof */
//void replaceExpDesignator(char *s, int n);

#endif /* RINEX_H */
