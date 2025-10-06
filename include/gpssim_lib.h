#ifndef GPSSIM_LIB_H
#define GPSSIM_LIB_H

#include "gps-l1c-sim.h"  /* For datetime_t and gpstime_t types */

/* Convert calendar date/time to GPS time */
void date2gps(const datetime_t *t, gpstime_t *g);

/* Compute difference between GPS times in seconds */
double subGpsTime(gpstime_t a, gpstime_t b);

/* GPS time increment by dt seconds */
gpstime_t incGpsTime(gpstime_t g0, double dt);

#endif /* GPSSIM_LIB_H */
