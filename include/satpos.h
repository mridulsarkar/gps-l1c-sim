#ifndef SATPOS_H
#define SATPOS_H

#include "gps-l1c-sim.h"

typedef struct {
    double pos[3];    // ECEF position (m)
    double vel[3];    // ECEF velocity (m/s)
    double clock_bias;  // Clock bias (s)
    double clock_drift; // Clock drift (s/s)
} sat_state_t;

/* Calculate satellite position/velocity/clock at given time */
int compute_satellite_state(const ephem_t *eph, const gpstime_t *t, sat_state_t *state);

/* Compute signal propagation effects */
double compute_troposphere_delay(double elev, double alt);
double compute_ionosphere_delay(const ionoutc_t *ionoutc, double lat, double lon, 
                              double elev, double az, const gpstime_t *t);

/* Compute geometric range and Doppler */
void compute_geometric_range(const double *sat_pos, const double *user_pos,
                           const double *sat_vel, const double *user_vel,
                           double *range, double *doppler);

/* Compute elevation and azimuth angles from user to satellite (radians) */
void sat_elevation(const double *user_pos, const double *sat_pos,
                  double *elevation, double *azimuth);

#endif /* SATPOS_H */
