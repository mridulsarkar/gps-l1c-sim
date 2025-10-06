#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include "gps-l1c-sim.h"
#include "rinex.h"
#include "l1c_seq.h"
#include "l1c_mod.h"
#include "nav_scheduler.h"
#include "satpos.h"
#include "channel.h"
#include "signal.h"
#include "gpssim_lib.h"

static void print_usage(void) {
    printf("Usage: gpssim -e <nav_file> [options]\n");
    printf("Options:\n");
    printf("  -p <lat,lon,alt>  Static position (degrees,degrees,meters)\n");
    printf("  -t <YYYY,MM,DD,HH,mm,SS>  Start time\n");
    printf("  -d <seconds>      Duration (default: %.1f)\n", DEFAULT_DURATION);
    printf("  -s <Hz>          Sampling rate (default: %.1e)\n", DEFAULT_SAMPLE_RATE);
    printf("  -b <bits>        IQ bit depth (default: %d)\n", DEFAULT_IQ_BITS);
    printf("  -o <file>        Output file (default: out.bin)\n");
}

/* Difference between two GPS times, in seconds */
double subGpsTime(gpstime_t a, gpstime_t b)
{
    double dt = (a.week - b.week) * SECONDS_IN_WEEK + (a.sec - b.sec);
    return dt;
}

/* ECEF to geodetic coordinate conversion */
static void ecef2geodetic(const double *ecef, double *llh) {
    const double a = 6378137.0;  // WGS84 semi-major axis
    const double f = 1.0/298.257223563;  // flattening
    const double b = a*(1.0-f);  // semi-minor axis
    const double e2 = 2.0*f - f*f;  // eccentricity squared
    
    double x = ecef[0], y = ecef[1], z = ecef[2];
    double lat, lon, h, N;
    
    lon = atan2(y, x);
    
    double p = sqrt(x*x + y*y);
    lat = atan2(z, p*(1.0-e2));
    
    for (int i = 0; i < 5; i++) {
        N = a/sqrt(1.0-e2*sin(lat)*sin(lat));
        h = p/cos(lat) - N;
        lat = atan2(z, p*(1.0-e2*N/(N+h)));
    }
    
    llh[0] = lat * 180.0/M_PI;  // degrees
    llh[1] = lon * 180.0/M_PI;  // degrees
    llh[2] = h;  // meters
}

static void geodetic2ecef(const double *llh, double *xyz) {
    const double a = 6378137.0;  // WGS84 semi-major axis
    const double f = 1.0/298.257223563;  // flattening
    const double e2 = 2.0*f - f*f;  // eccentricity squared
    
    double lat = llh[0] * M_PI/180.0;
    double lon = llh[1] * M_PI/180.0;
    double h = llh[2];
    
    double N = a/sqrt(1.0-e2*sin(lat)*sin(lat));
    
    xyz[0] = (N + h)*cos(lat)*cos(lon);
    xyz[1] = (N + h)*cos(lat)*sin(lon);
    xyz[2] = (N*(1-e2) + h)*sin(lat);
}

/* Main simulator function */
int main(int argc, char **argv)
{
    sim_options_t options;
    memset(&options, 0, sizeof(options));
    options.duration = DEFAULT_DURATION;
    options.sample_rate = DEFAULT_SAMPLE_RATE;
    options.iq_bits = DEFAULT_IQ_BITS;
    strcpy(options.out_file, "out.bin");

    int opt;
    while ((opt = getopt(argc, argv, "e:p:t:d:s:b:o:")) != -1) {
        switch (opt) {
            case 'e':
                strncpy(options.nav_file, optarg, sizeof(options.nav_file) - 1);
                break;
            case 'p':
                sscanf(optarg, "%lf,%lf,%lf", &options.lat, &options.lon, &options.alt);
                break;
            case 't':
                sscanf(optarg, "%d,%d,%d,%d,%d,%lf", &options.start_time.y, &options.start_time.m,
                       &options.start_time.d, &options.start_time.hh, &options.start_time.mm,
                       &options.start_time.sec);
                break;
            case 'd':
                options.duration = atof(optarg);
                break;
            case 's':
                options.sample_rate = atof(optarg);
                break;
            case 'b':
                options.iq_bits = atoi(optarg);
                break;
            case 'o':
                strncpy(options.out_file, optarg, sizeof(options.out_file) - 1);
                break;
            default:
                print_usage();
                return EXIT_FAILURE;
        }
    }

    if (options.nav_file[0] == '\0') {
        fprintf(stderr, "Navigation file is required.\n");
        print_usage();
        return EXIT_FAILURE;
    }

    /* Convert start time to GPS time */
    gpstime_t gps_start_time;
    date2gps(&options.start_time, &gps_start_time);

    /* Load ephemeris data */
    ephem_t eph[EPHEM_ARRAY_SIZE][MAX_SAT];
    ionoutc_t ionoutc;
    
    printf("Reading RINEX nav file: %s\n", options.nav_file);
    int num_eph = readRinexNavAll(eph, &ionoutc, options.nav_file);
    if (num_eph < 0) {
        fprintf(stderr, "Failed to read navigation file\n");
        return EXIT_FAILURE;
    }
    
    /* Initialize L1C signal components */
    nav_scheduler_init();
    
    /* Calculate number of samples */
    int num_samples = (int)(options.duration * options.sample_rate);
    
    /* Allocate sample buffers */
    double *combined_i = calloc(num_samples, sizeof(double));
    double *combined_q = calloc(num_samples, sizeof(double));
    double *sat_i = malloc(num_samples * sizeof(double));
    double *sat_q = malloc(num_samples * sizeof(double));
    
    if (!combined_i || !combined_q || !sat_i || !sat_q) {
        fprintf(stderr, "Memory allocation failed\n");
        return EXIT_FAILURE;
    }
    
    /* User position in ECEF */
    double user_pos[3], user_llh[3] = {options.lat, options.lon, options.alt};
    geodetic2ecef(user_llh, user_pos);
    double user_vel[3] = {0,0,0};  // Static user
    
    /* Main simulation loop */
    printf("Generating %d samples at %.1f MHz...\n", 
           num_samples, options.sample_rate/1e6);
    
    gpstime_t sim_time = gps_start_time;
    double sample_time = 0.0;
    
    for (int sv = 1; sv <= MAX_SAT; sv++) {
        if (!eph[0][sv-1].vflg) continue;
        
        /* Compute satellite state */
        sat_state_t sat_state;
        if (compute_satellite_state(&eph[0][sv-1], &sim_time, &sat_state) < 0) {
            continue;
        }
        
        /* Compute geometry and delays */
        double range, doppler;
        compute_geometric_range(sat_state.pos, user_pos,
                              sat_state.vel, user_vel,
                              &range, &doppler);
                              
        /* Compute propagation delays */
        double elev, azim;
        sat_elevation(user_pos, sat_state.pos, &elev, &azim);
        
        double trop_delay = compute_troposphere_delay(elev, options.alt);
        double iono_delay = compute_ionosphere_delay(&ionoutc, options.lat,
                                                   options.lon, elev, azim,
                                                   &sim_time);
                                                   
        /* Total delay and phase */
        double total_delay = (range + trop_delay + iono_delay)/C_LIGHT;
        double carrier_phase = -2.0*M_PI*L1_FREQ*total_delay;
        double code_phase = fmod(total_delay*1.023e6, 10230.0);
        
        /* Setup channel parameters */
        channel_params_t chan = {
            .power_dbw = -130.0,  // Nominal power
            .num_paths = 2,      // Direct + 1 multipath
            .multipath_delays = {0.0, 0.5e-6, 0.0},  // 0.5us delayed path
            .multipath_powers = {0.0, -3.0, 0.0},    // -3dB relative power
            .multipath_phases = {0.0, M_PI/3, 0.0},  // Random initial phase
            .cn0_db = 45.0       // C/N0 = 45 dB-Hz
        };
        
        /* Generate this satellite's signal */
        int gen = generate_satellite_signal(
            sv,                  // PRN
            sat_i, sat_q,       // Output buffers
            num_samples,        // Number of samples
            options.sample_rate,// Sample rate
            code_phase,         // Initial code phase
            carrier_phase,      // Initial carrier phase
            doppler,           // Computed Doppler
            chan.power_dbw     // Signal power
        );
        
        if (gen > 0) {
            /* Apply channel effects */
            apply_channel_effects(sat_i, sat_q, num_samples,
                                options.sample_rate, &chan);
            
            /* Add to combined signal */
            for (int i = 0; i < num_samples; i++) {
                combined_i[i] += sat_i[i];
                combined_q[i] += sat_q[i];
            }
            printf("Added PRN %d (elev=%.1f deg, C/N0=%.1f dB-Hz)\n", 
                   sv, elev*180.0/M_PI, chan.cn0_db);
        }
    }
    
    /* Prepare output buffer */
    int sample_size = (options.iq_bits <= 8) ? 1 : 2;
    void *output = malloc(num_samples * 2 * sample_size);
    if (!output) {
        fprintf(stderr, "Failed to allocate output buffer\n");
        return EXIT_FAILURE;
    }
    
    /* Interleave and quantize I/Q samples */
    double *interleaved = malloc(num_samples * 2 * sizeof(double));
    for (int i = 0; i < num_samples; i++) {
        interleaved[i*2] = combined_i[i];
        interleaved[i*2+1] = combined_q[i];
    }
    
    quantize_iq_samples(interleaved, output, num_samples, options.iq_bits);
    
    /* Write to file */
    FILE *fp = fopen(options.out_file, "wb");
    if (!fp) {
        fprintf(stderr, "Failed to open output file\n");
        return EXIT_FAILURE;
    }
    
    fwrite(output, sample_size, num_samples * 2, fp);
    fclose(fp);
    
    /* Cleanup */
    free(combined_i);
    free(combined_q);
    free(sat_i);
    free(sat_q);
    free(interleaved);
    free(output);
    
    printf("Simulation completed. Output written to %s\n", options.out_file);
    return EXIT_SUCCESS;
}

