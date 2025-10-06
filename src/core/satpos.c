#include <math.h>
#include "satpos.h"

/* Compute satellite position from ephemeris */
int compute_satellite_state(const ephem_t *eph, const gpstime_t *t, sat_state_t *state) {
    if (!eph->vflg) return -1;

    double dt = subGpsTime(*t, eph->toe);
    double a = eph->sqrta * eph->sqrta;
    double n0 = sqrt(GM_EARTH/(a*a*a));
    double n = n0 + eph->deltan;
    double M = eph->m0 + n*dt;
    
    // Solve Kepler's equation iteratively
    double E = M;
    for (int i = 0; i < 8; i++) {
        E = M + eph->ecc * sin(E);
    }
    
    double v = atan2(sqrt(1.0-eph->ecc*eph->ecc)*sin(E), cos(E)-eph->ecc);
    double u = v + eph->aop;
    
    // Position computation
    double r = a*(1.0 - eph->ecc*cos(E));
    double xp = r*cos(u);
    double yp = r*sin(u);
    
    double omg = eph->omg0 + (eph->omgdot - OMEGA_EARTH)*dt;
    
    // ECEF position
    state->pos[0] = xp*cos(omg) - yp*cos(eph->inc0)*sin(omg);
    state->pos[1] = xp*sin(omg) + yp*cos(eph->inc0)*cos(omg);
    state->pos[2] = yp*sin(eph->inc0);
    
    // Velocity computation (simplified)
    double Edot = n/(1.0 - eph->ecc*cos(E));
    double vdot = Edot * (1.0 + eph->ecc*cos(v))*(1.0 + eph->ecc*cos(v)) / 
                  ((1.0 - eph->ecc*cos(E))*(1.0 - eph->ecc*eph->ecc));
    
    double xpdot = r*(-vdot*sin(u));
    double ypdot = r*(vdot*cos(u));
    
    state->vel[0] = xpdot*cos(omg) - ypdot*cos(eph->inc0)*sin(omg);
    state->vel[1] = xpdot*sin(omg) + ypdot*cos(eph->inc0)*cos(omg);
    state->vel[2] = ypdot*sin(eph->inc0);
    
    // Clock correction
    state->clock_bias = eph->af0 + eph->af1*dt + eph->af2*dt*dt;
    state->clock_drift = eph->af1 + 2.0*eph->af2*dt;
    
    return 0;
}

/* Compute elevation and azimuth angles from user to satellite */
void sat_elevation(const double *user_pos, const double *sat_pos,
                  double *elevation, double *azimuth) 
{
    // Convert user position to local geodetic frame
    double rx = sat_pos[0] - user_pos[0];
    double ry = sat_pos[1] - user_pos[1];
    double rz = sat_pos[2] - user_pos[2];
    
    // Compute range
    double r = sqrt(rx*rx + ry*ry + rz*rz);
    
    // User position magnitude
    double u = sqrt(user_pos[0]*user_pos[0] + 
                   user_pos[1]*user_pos[1] + 
                   user_pos[2]*user_pos[2]);
    
    // Convert to topocentric-horizon coordinates (ENU)
    double e = asin(((sat_pos[2] - user_pos[2])*u - 
                     user_pos[2]*r)/(r*u));
    double n = atan2(rx*user_pos[1] - ry*user_pos[0],
                    rx*user_pos[0]*user_pos[2] + 
                    ry*user_pos[1]*user_pos[2] - 
                    rz*u*u);
    
    *elevation = e;
    *azimuth = n;
}

/* Compute tropospheric delay using simplified model */
double compute_troposphere_delay(double elev, double h) {
    double delay = 2.47/sin(sqrt(elev*elev + 0.0121));
    return delay * exp(-h/7000.0);  // Scale with height
}

/* Compute geometric range and Doppler */
void compute_geometric_range(const double *sat_pos, const double *user_pos,
                           const double *sat_vel, const double *user_vel,
                           double *range, double *doppler) {
    double dx = sat_pos[0] - user_pos[0];
    double dy = sat_pos[1] - user_pos[1];
    double dz = sat_pos[2] - user_pos[2];
    
    *range = sqrt(dx*dx + dy*dy + dz*dz);
    
    // Line of sight velocity for Doppler
    double dvx = sat_vel[0] - user_vel[0];
    double dvy = sat_vel[1] - user_vel[1];
    double dvz = sat_vel[2] - user_vel[2];
    
    *doppler = -(dx*dvx + dy*dvy + dz*dvz)/(*range) * L1_FREQ/C_LIGHT;
}

/* Compute ionospheric delay using Klobuchar model */
double compute_ionosphere_delay(const ionoutc_t *ionoutc, double lat, double lon, 
                              double elev, double azim, const gpstime_t *t)
{
    const double SECONDS_PER_DAY = 86400.0;
    
    // Convert inputs to semicircles
    double lat_semi = lat / 180.0;
    double lon_semi = lon / 180.0;
    
    // Earth-centered angle (semicircles)
    double psi = 0.0137 / (elev/M_PI + 0.11) - 0.022;
    
    // Subionospheric latitude (semicircles)
    double phi_i = lat_semi + psi * cos(azim);
    if (phi_i > 0.416)
        phi_i = 0.416;
    else if (phi_i < -0.416)
        phi_i = -0.416;
    
    // Subionospheric longitude (semicircles)
    double lambda_i = lon_semi + psi * sin(azim) / cos(phi_i * M_PI);
    
    // Geomagnetic latitude (semicircles)
    double phi_m = phi_i + 0.064 * cos((lambda_i - 1.617) * M_PI);
    
    // Local time (seconds)
    double t_local = SECONDS_PER_DAY * lambda_i + t->sec;
    while (t_local >= SECONDS_PER_DAY)
        t_local -= SECONDS_PER_DAY;
    while (t_local < 0)
        t_local += SECONDS_PER_DAY;
    
    // Amplitude of ionospheric delay
    double amp = ionoutc->alpha0 + phi_m * (ionoutc->alpha1 + phi_m * 
                (ionoutc->alpha2 + phi_m * ionoutc->alpha3));
    if (amp < 0.0)
        amp = 0.0;
    
    // Period of ionospheric delay
    double per = ionoutc->beta0 + phi_m * (ionoutc->beta1 + phi_m * 
                (ionoutc->beta2 + phi_m * ionoutc->beta3));
    if (per < 72000.0)
        per = 72000.0;
    
    // Phase of ionospheric delay
    double x = 2.0 * M_PI * (t_local - 50400.0) / per;
    
    // Obliquity factor
    double f = 1.0 + 16.0 * pow((0.53 - elev/M_PI), 3);
    
    // Ionospheric time delay (seconds)
    double delay;
    if (fabs(x) >= 1.57)
        delay = f * 5.0e-9;
    else
        delay = f * (5.0e-9 + amp * (1.0 - pow(x,2)/2.0 + pow(x,4)/24.0));
    
    return delay * C_LIGHT;  // Convert to meters
}
