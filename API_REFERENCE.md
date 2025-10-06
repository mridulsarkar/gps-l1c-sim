# GPS L1C Simulator API Reference

## Table of Contents

1. [Core API Functions](#core-api-functions)
2. [Signal Processing APIs](#signal-processing-apis)
3. [Navigation Data APIs](#navigation-data-apis)
4. [Time System APIs](#time-system-apis)
5. [Channel Modeling APIs](#channel-modeling-apis)
6. [Utility Functions](#utility-functions)
7. [Data Structures](#data-structures)
8. [Constants and Enumerations](#constants-and-enumerations)
9. [Error Codes](#error-codes)
10. [Usage Examples](#usage-examples)

## Core API Functions

### Simulation Control Functions

#### `int main(int argc, char **argv)`
**Header**: `gps-l1c-sim.c`  
**Purpose**: Main entry point for the GPS L1C simulator

**Parameters**:
- `argc`: Number of command-line arguments
- `argv`: Array of command-line argument strings

**Returns**:
- `EXIT_SUCCESS` (0): Simulation completed successfully
- `EXIT_FAILURE` (1): Error occurred during simulation

**Usage**:
```c
// Command line execution
./gps-l1c-sim -e nav.22n -p 40.7,-74.0,100 -d 10.0 -o output.bin
```

#### `static void print_usage(void)`
**Header**: `gps-l1c-sim.c`  
**Purpose**: Display command-line usage information

**Parameters**: None

**Returns**: void

**Usage**:
```c
if (invalid_args) {
    print_usage();
    return EXIT_FAILURE;
}
```

### Coordinate Conversion Functions

#### `static void ecef2geodetic(const double *ecef, double *llh)`
**Header**: `gps-l1c-sim.c`  
**Purpose**: Convert Earth-Centered Earth-Fixed (ECEF) coordinates to geodetic coordinates

**Parameters**:
- `ecef[3]`: Input ECEF coordinates [X, Y, Z] in meters
- `llh[3]`: Output geodetic coordinates [lat, lon, height]
  - `lat`: Latitude in degrees (-90 to +90)
  - `lon`: Longitude in degrees (-180 to +180)
  - `height`: Height above WGS84 ellipsoid in meters

**Returns**: void

**Algorithm**: Iterative solution using WGS84 ellipsoid parameters

**Usage**:
```c
double ecef[3] = {4194304.0, 176466.0, 4487348.0};
double llh[3];
ecef2geodetic(ecef, llh);
printf("Lat: %.6f°, Lon: %.6f°, Alt: %.2f m\n", llh[0], llh[1], llh[2]);
```

#### `static void geodetic2ecef(const double *llh, double *xyz)`
**Header**: `gps-l1c-sim.c`  
**Purpose**: Convert geodetic coordinates to ECEF coordinates

**Parameters**:
- `llh[3]`: Input geodetic coordinates [lat°, lon°, height_m]
- `xyz[3]`: Output ECEF coordinates [X, Y, Z] in meters

**Returns**: void

**Usage**:
```c
double llh[3] = {40.7589, -73.9851, 100.0};  // NYC coordinates
double xyz[3];
geodetic2ecef(llh, xyz);
printf("ECEF: %.2f, %.2f, %.2f\n", xyz[0], xyz[1], xyz[2]);
```

## Signal Processing APIs

### L1C Signal Generation

#### `int generate_satellite_signal(int prn, double *i_samples, double *q_samples, int num_samples, double sample_rate, double code_phase, double carrier_phase, double doppler, double power_dbw)`
**Header**: `signal.h`  
**Purpose**: Generate complete L1C baseband signal for one satellite

**Parameters**:
- `prn`: PRN number (1-63)
- `i_samples`: Output I channel buffer (pre-allocated)
- `q_samples`: Output Q channel buffer (pre-allocated)
- `num_samples`: Number of complex samples to generate
- `sample_rate`: Sampling rate in Hz
- `code_phase`: Initial code phase in chips (0.0 to 10229.0)
- `carrier_phase`: Initial carrier phase in radians
- `doppler`: Doppler shift in Hz (positive = approaching)
- `power_dbw`: Signal power in dBW

**Returns**:
- Number of samples generated on success
- Negative value on error

**Usage**:
```c
double *i_buf = malloc(100000 * sizeof(double));
double *q_buf = malloc(100000 * sizeof(double));

int samples = generate_satellite_signal(
    1,                  // PRN 1
    i_buf, q_buf,      // Output buffers
    100000,            // 100k samples
    25e6,              // 25 MHz sampling
    0.0,               // Initial code phase
    0.0,               // Initial carrier phase
    -1500.0,           // -1.5 kHz Doppler
    -130.0             // -130 dBW power
);
```

### L1C Code Generation

#### `int l1c_generate_prn(int prn, uint8_t *ranging_out, uint8_t *overlay_out)`
**Header**: `l1c.h`  
**Purpose**: Generate L1C PRN sequences for ranging and overlay codes

**Parameters**:
- `prn`: PRN number (1-63)
- `ranging_out`: Output buffer for 10,230-chip ranging code (0/1 values)
- `overlay_out`: Output buffer for 2,047-chip overlay code (0/1 values)

**Returns**:
- `0`: Success
- `-1`: Error (invalid PRN or buffer allocation failure)

**Usage**:
```c
uint8_t ranging[10230];
uint8_t overlay[2047];

if (l1c_generate_prn(15, ranging, overlay) == 0) {
    printf("Generated PRN 15 codes successfully\n");
    printf("First 10 ranging chips: ");
    for (int i = 0; i < 10; i++) {
        printf("%d", ranging[i]);
    }
    printf("\n");
}
```

#### `int l1c_generate_seq(int prn, uint8_t *l1cd_out, uint8_t *l1cp_out)`
**Header**: `l1c_seq.h`  
**Purpose**: Generate complete L1CD and L1CP sequences

**Parameters**:
- `prn`: PRN number (1-63)
- `l1cd_out`: Output L1CD sequence (10,230 chips)
- `l1cp_out`: Output L1CP sequence (10,230 chips)

**Returns**:
- `0`: Success
- `-1`: Error

**Usage**:
```c
uint8_t l1cd[10230];
uint8_t l1cp[10230];

if (l1c_generate_seq(23, l1cd, l1cp) == 0) {
    // L1CD contains ranging code only
    // L1CP contains ranging XOR overlay
    printf("L1CD[0]=%d, L1CP[0]=%d\n", l1cd[0], l1cp[0]);
}
```

### Modulation Functions

#### `int l1c_mod_boc11(const uint8_t *chips, int *waveform)`
**Header**: `l1c_mod.h`  
**Purpose**: Generate BOC(1,1) modulated waveform

**Parameters**:
- `chips`: Input chip sequence (0/1 values)
- `waveform`: Output waveform buffer (sized for 2×chip_length)

**Returns**: Number of waveform samples generated

**Algorithm**: Each chip produces 2 sub-chips: [+1, -1] or [-1, +1]

**Usage**:
```c
uint8_t chips[100] = {1,0,1,1,0,0,1,0,1,0,...};
int waveform[200];  // 2x chip length

int samples = l1c_mod_boc11(chips, waveform);
printf("Generated %d BOC(1,1) samples\n", samples);
```

#### `int l1c_mod_tmboc(const uint8_t *chips, int *waveform)`
**Header**: `l1c_mod.h`  
**Purpose**: Generate TMBOC (Time-Multiplexed BOC) waveform

**Parameters**:
- `chips`: Input chip sequence
- `waveform`: Output waveform buffer

**Returns**: Number of waveform samples generated

**Algorithm**: 
- Positions 1,11,17,29 in 33-chip cycle: BOC(1,1) (2 samples/chip)
- Other positions: BOC(6,1) (12 samples/chip)

**Usage**:
```c
uint8_t chips[10230];
int waveform[20460];  // Sized for TMBOC expansion

int samples = l1c_mod_tmboc(chips, waveform);
printf("TMBOC waveform: %d samples\n", samples);
```

### Sample Quantization

#### `int quantize_iq_samples(const double *samples, void *output, int num_samples, int bits)`
**Header**: `signal.h`  
**Purpose**: Convert floating-point I/Q samples to integer format

**Parameters**:
- `samples`: Input floating-point samples (interleaved I,Q,I,Q,...)
- `output`: Output buffer (int8_t* for 8-bit, int16_t* for 16-bit)
- `num_samples`: Number of complex samples
- `bits`: Bit depth (8 or 16)

**Returns**:
- `0`: Success
- `-1`: Invalid bit depth

**Usage**:
```c
double float_samples[200];  // 100 complex samples
int16_t int_samples[200];

// Auto-scale and quantize to 16-bit
if (quantize_iq_samples(float_samples, int_samples, 100, 16) == 0) {
    FILE *fp = fopen("output.bin", "wb");
    fwrite(int_samples, sizeof(int16_t), 200, fp);
    fclose(fp);
}
```

## Navigation Data APIs

### RINEX File Processing

#### `int readRinexNavAll(ephem_t eph[][MAX_SAT], ionoutc_t *ionoutc, const char *fname)`
**Header**: `rinex.h`  
**Purpose**: Read complete RINEX navigation file

**Parameters**:
- `eph`: Output ephemeris array [epoch][satellite-1]
- `ionoutc`: Output ionosphere/UTC parameters
- `fname`: Path to RINEX navigation file

**Returns**:
- Number of ephemeris epochs loaded (>= 0)
- `-1`: File open/parse error

**Usage**:
```c
ephem_t eph[EPHEM_ARRAY_SIZE][MAX_SAT];
ionoutc_t ionutc;

int epochs = readRinexNavAll(eph, &ionutc, "brdc0010.22n");
if (epochs > 0) {
    printf("Loaded %d ephemeris epochs\n", epochs);
    
    // Check for PRN 15 in first epoch
    if (eph[0][14].vflg) {
        printf("PRN 15 available: toe=%.1f\n", eph[0][14].toe.sec);
    }
}
```

### Navigation Message Generation

#### `int nav_cnav_generate(int prn, uint8_t *bits_out, int max_bits)`
**Header**: `nav_cnav.h`  
**Purpose**: Generate CNAV-2 navigation message

**Parameters**:
- `prn`: PRN number
- `bits_out`: Output buffer for navigation bits
- `max_bits`: Maximum bits to generate

**Returns**: Number of bits generated

**Usage**:
```c
uint8_t nav_bits[300];
int bit_count = nav_cnav_generate(12, nav_bits, 300);

printf("Generated %d navigation bits for PRN 12\n", bit_count);
printf("First 8 bits: ");
for (int i = 0; i < 8; i++) {
    printf("%d", nav_bits[i]);
}
printf("\n");
```

#### `void nav_scheduler_init(void)`
**Header**: `nav_scheduler.h`  
**Purpose**: Initialize navigation message scheduler

**Parameters**: None

**Returns**: void

**Usage**:
```c
// Call once before using navigation functions
nav_scheduler_init();

// Now safe to call other nav_scheduler_* functions
```

## Time System APIs

### Time Conversion Functions

#### `void date2gps(const datetime_t *t, gpstime_t *g)`
**Header**: `gpssim_lib.h`  
**Purpose**: Convert calendar date/time to GPS time

**Parameters**:
- `t`: Input calendar date and time
- `g`: Output GPS time (week number and seconds)

**Returns**: void

**Usage**:
```c
datetime_t calendar = {2022, 1, 15, 12, 30, 45.5};
gpstime_t gps_time;

date2gps(&calendar, &gps_time);
printf("GPS time: week %d, seconds %.1f\n", gps_time.week, gps_time.sec);
```

#### `double subGpsTime(gpstime_t a, gpstime_t b)`
**Header**: `gpssim_lib.h`  
**Purpose**: Compute difference between two GPS times

**Parameters**:
- `a`: Later GPS time
- `b`: Earlier GPS time

**Returns**: Time difference in seconds (a - b)

**Usage**:
```c
gpstime_t time1 = {2200, 345600.0};  // Week 2200, Wed 12:00:00
gpstime_t time2 = {2200, 342000.0};  // Week 2200, Tue 23:00:00

double diff = subGpsTime(time1, time2);
printf("Time difference: %.1f seconds (%.1f hours)\n", diff, diff/3600.0);
```

#### `gpstime_t incGpsTime(gpstime_t g0, double dt)`
**Header**: `gpssim_lib.h`  
**Purpose**: Advance GPS time by specified seconds

**Parameters**:
- `g0`: Base GPS time
- `dt`: Time increment in seconds

**Returns**: New GPS time (g0 + dt)

**Usage**:
```c
gpstime_t start = {2200, 86400.0};  // Week 2200, Sunday 24:00:00
gpstime_t later = incGpsTime(start, 3661.5);  // Add 1h 1m 1.5s

printf("Later time: week %d, sec %.1f\n", later.week, later.sec);
```

## Channel Modeling APIs

### Channel Effects

#### `void apply_channel_effects(double *i_samples, double *q_samples, int num_samples, double sample_rate, const channel_params_t *params)`
**Header**: `channel.h`  
**Purpose**: Apply complete channel model to I/Q samples

**Parameters**:
- `i_samples`: Input/output I channel samples (modified in-place)
- `q_samples`: Input/output Q channel samples (modified in-place)
- `num_samples`: Number of complex samples
- `sample_rate`: Sampling rate in Hz
- `params`: Channel model parameters

**Returns**: void

**Usage**:
```c
channel_params_t channel = {
    .power_dbw = -130.0,
    .num_paths = 2,
    .multipath_delays = {0.0, 0.5e-6, 0.0},      // 0.5 μs delay
    .multipath_powers = {0.0, -3.0, 0.0},        // -3 dB relative power
    .multipath_phases = {0.0, M_PI/4, 0.0},      // 45° phase shift
    .cn0_db = 45.0
};

apply_channel_effects(i_samples, q_samples, 10000, 25e6, &channel);
```

## Satellite Position APIs

### Position Computation

#### `int compute_satellite_state(const ephem_t *eph, const gpstime_t *t, sat_state_t *state)`
**Header**: `satpos.h`  
**Purpose**: Compute satellite position, velocity, and clock at specified time

**Parameters**:
- `eph`: Satellite ephemeris data
- `t`: GPS time for computation
- `state`: Output satellite state

**Returns**:
- `0`: Success
- `-1`: Error (invalid ephemeris or time)

**Usage**:
```c
ephem_t eph = /* loaded from RINEX */;
gpstime_t time = {2200, 345600.0};
sat_state_t state;

if (compute_satellite_state(&eph, &time, &state) == 0) {
    printf("Satellite position: %.2f, %.2f, %.2f\n", 
           state.pos[0], state.pos[1], state.pos[2]);
    printf("Clock bias: %.3e seconds\n", state.clock_bias);
}
```

### Atmospheric Delay Models

#### `double compute_troposphere_delay(double elev, double alt)`
**Header**: `satpos.h`  
**Purpose**: Compute tropospheric delay using Saastamoinen model

**Parameters**:
- `elev`: Elevation angle in radians (0 to π/2)
- `alt`: Receiver altitude in meters

**Returns**: Tropospheric delay in meters

**Usage**:
```c
double elevation = 30.0 * M_PI / 180.0;  // 30 degrees
double altitude = 100.0;                 // 100 meters

double trop_delay = compute_troposphere_delay(elevation, altitude);
printf("Tropospheric delay: %.2f meters\n", trop_delay);
```

#### `double compute_ionosphere_delay(const ionoutc_t *ionoutc, double lat, double lon, double elev, double az, const gpstime_t *t)`
**Header**: `satpos.h`  
**Purpose**: Compute ionospheric delay using Klobuchar model

**Parameters**:
- `ionoutc`: Ionospheric correction parameters
- `lat`: Receiver latitude in radians
- `lon`: Receiver longitude in radians
- `elev`: Elevation angle in radians
- `az`: Azimuth angle in radians
- `t`: GPS time

**Returns**: Ionospheric delay in meters

**Usage**:
```c
ionoutc_t ion = /* loaded from RINEX */;
double lat = 40.7589 * M_PI / 180.0;
double lon = -73.9851 * M_PI / 180.0;
double elev = 45.0 * M_PI / 180.0;
double azim = 120.0 * M_PI / 180.0;
gpstime_t time = {2200, 345600.0};

double iono_delay = compute_ionosphere_delay(&ion, lat, lon, elev, azim, &time);
printf("Ionospheric delay: %.2f meters\n", iono_delay);
```

### Geometric Functions

#### `void compute_geometric_range(const double *sat_pos, const double *user_pos, const double *sat_vel, const double *user_vel, double *range, double *doppler)`
**Header**: `satpos.h`  
**Purpose**: Compute geometric range and Doppler shift

**Parameters**:
- `sat_pos[3]`: Satellite ECEF position (meters)
- `user_pos[3]`: User ECEF position (meters)
- `sat_vel[3]`: Satellite ECEF velocity (m/s)
- `user_vel[3]`: User ECEF velocity (m/s)
- `range`: Output geometric range (meters)
- `doppler`: Output Doppler shift (Hz)

**Returns**: void

**Usage**:
```c
double sat_pos[3] = {15945123.45, -21734567.89, 7842156.23};
double user_pos[3] = {1234567.89, -4567890.12, 4345678.90};
double sat_vel[3] = {-1234.56, 2345.67, -987.65};
double user_vel[3] = {0.0, 0.0, 0.0};  // Static user

double range, doppler;
compute_geometric_range(sat_pos, user_pos, sat_vel, user_vel, &range, &doppler);

printf("Range: %.2f km, Doppler: %.1f Hz\n", range/1000.0, doppler);
```

#### `void sat_elevation(const double *user_pos, const double *sat_pos, double *elevation, double *azimuth)`
**Header**: `satpos.h`  
**Purpose**: Compute elevation and azimuth angles from user to satellite

**Parameters**:
- `user_pos[3]`: User ECEF position (meters)
- `sat_pos[3]`: Satellite ECEF position (meters)
- `elevation`: Output elevation angle (radians, 0 to π/2)
- `azimuth`: Output azimuth angle (radians, 0 to 2π)

**Returns**: void

**Usage**:
```c
double user_pos[3] = {1234567.89, -4567890.12, 4345678.90};
double sat_pos[3] = {15945123.45, -21734567.89, 7842156.23};
double elev, azim;

sat_elevation(user_pos, sat_pos, &elev, &azim);

printf("Elevation: %.1f°, Azimuth: %.1f°\n", 
       elev * 180.0/M_PI, azim * 180.0/M_PI);
```

## Data Structures

### Simulation Configuration

#### `sim_options_t`
```c
typedef struct {
    char nav_file[256];        // RINEX navigation file path
    double lat, lon, alt;      // Receiver position (deg, deg, meters)
    datetime_t start_time;     // Simulation start time
    double duration;           // Simulation duration (seconds)
    double sample_rate;        // Sampling rate (Hz)
    int iq_bits;              // I/Q bit depth (8 or 16)
    char out_file[256];       // Output file path
} sim_options_t;
```

### Time Structures

#### `datetime_t`
```c
typedef struct {
    int y;         // Year (4 digits)
    int m;         // Month (1-12)
    int d;         // Day (1-31)
    int hh;        // Hour (0-23)
    int mm;        // Minute (0-59)
    double sec;    // Seconds (0-59.999...)
} datetime_t;
```

#### `gpstime_t`
```c
typedef struct {
    int week;      // GPS week number
    double sec;    // Seconds into week (0 to 604799.999...)
    int y, m, d;   // Calendar date (computed)
    int hh, mm;    // Time of day (computed)
} gpstime_t;
```

### Satellite Data

#### `ephem_t`
```c
typedef struct {
    datetime_t t;              // Time of clock
    gpstime_t toc, toe;       // Time of clock/ephemeris
    
    // Clock correction parameters
    double af0, af1, af2;     // Clock bias, drift, drift rate
    
    // Orbital elements
    double crs, deltan, m0;   // Orbit radius, mean motion, mean anomaly
    double cuc, ecc, cus;     // Argument of latitude, eccentricity
    double sqrta;             // Square root of semi-major axis
    double cic, omg0, cis;    // Inclination, longitude of ascending node
    double inc0;              // Inclination angle
    double crc, aop;          // Orbit radius, argument of perigee
    double omgdot, idot;      // Rate of right ascension, inclination
    
    // Derived parameters
    double A;                 // Semi-major axis
    double n;                 // Mean motion
    double sq1e2;            // sqrt(1-e^2)
    double omgkdot;          // Corrected omega dot
    
    // Data quality indicators
    int iode, iodc;          // Issue of data
    int codeL2;              // Code on L2
    int svhlth;              // Satellite health
    double tgd;              // Group delay
    int vflg;                // Valid flag
} ephem_t;
```

#### `sat_state_t`
```c
typedef struct {
    double pos[3];           // ECEF position (meters)
    double vel[3];           // ECEF velocity (m/s)
    double clock_bias;       // Clock bias (seconds)
    double clock_drift;      // Clock drift (s/s)
} sat_state_t;
```

### Channel Model

#### `channel_params_t`
```c
typedef struct {
    double power_dbw;              // Direct path power (dBW)
    double multipath_delays[3];    // Multipath delays (seconds)
    double multipath_powers[3];    // Relative powers (dB)
    double multipath_phases[3];    // Initial phases (radians)
    int num_paths;                 // Number of paths (1-3)
    double cn0_db;                 // Carrier-to-noise ratio (dB-Hz)
} channel_params_t;
```

### Ionosphere/UTC Parameters

#### `ionoutc_t`
```c
typedef struct {
    // Ionospheric correction coefficients
    double alpha0, alpha1, alpha2, alpha3;  // Alpha coefficients
    double beta0, beta1, beta2, beta3;      // Beta coefficients
    
    // UTC correction parameters
    double A0, A1;                          // Delta-UTC coefficients
    int tot, wnt, dtls;                     // Time reference, week, leap seconds
    
    int vflg;                               // Valid flag
} ionoutc_t;
```

## Constants and Enumerations

### Physical Constants

```c
#define C_LIGHT             299792458.0      // Speed of light (m/s)
#define L1_FREQ             1575.42e6        // L1 frequency (Hz)
#define L1C_CHIP_RATE_HZ    1.023e6         // L1C chip rate (Hz)
#define GM_EARTH            3.986005e14      // Earth's gravitational parameter
#define OMEGA_EARTH         7.2921151467e-5  // Earth rotation rate (rad/s)
```

### System Limits

```c
#define MAX_SAT             64               // Maximum satellites
#define EPHEM_ARRAY_SIZE    40               // Ephemeris storage slots
#define L1C_CODE_LENGTH     10230            // L1C code length (chips)
#define L1C_OVERLAY_LENGTH  2047             // Overlay sequence length
```

### Default Values

```c
#define DEFAULT_SAMPLE_RATE 50.0e6           // 50 MHz
#define DEFAULT_IQ_BITS     8                // 8-bit I/Q
#define DEFAULT_DURATION    1.0              // 1 second
#define AWGN_SEED          12345             // Random seed
```

### Time Constants

```c
#define SECONDS_IN_HOUR     3600
#define SECONDS_IN_DAY      86400
#define SECONDS_IN_WEEK     604800
```

## Error Codes

### Return Value Conventions

```c
#define SUCCESS              0               // Operation successful
#define ERROR_GENERAL       -1               // General error
#define ERROR_INVALID_PARAM -2               // Invalid parameter
#define ERROR_MEMORY        -3               // Memory allocation failure
#define ERROR_FILE_IO       -4               // File I/O error
#define ERROR_DATA_INVALID  -5               // Data validation error
#define ERROR_TIME_INVALID  -6               // Invalid time specification
#define ERROR_PRN_INVALID   -7               // Invalid PRN number
```

## Usage Examples

### Complete Simulation Example

```c
#include "gps-l1c-sim.h"
#include "rinex.h"
#include "signal.h"
#include "satpos.h"

int main() {
    // Configuration
    sim_options_t options = {
        .nav_file = "brdc0010.22n",
        .lat = 40.7589, .lon = -73.9851, .alt = 100.0,
        .start_time = {2022, 1, 1, 12, 0, 0.0},
        .duration = 10.0,
        .sample_rate = 25e6,
        .iq_bits = 16,
        .out_file = "gps_sim.bin"
    };
    
    // Load navigation data
    ephem_t eph[EPHEM_ARRAY_SIZE][MAX_SAT];
    ionoutc_t ionutc;
    int epochs = readRinexNavAll(eph, &ionutc, options.nav_file);
    if (epochs < 0) return -1;
    
    // Convert user position
    double user_llh[3] = {options.lat, options.lon, options.alt};
    double user_pos[3];
    geodetic2ecef(user_llh, user_pos);
    
    // Convert start time
    gpstime_t sim_time;
    date2gps(&options.start_time, &sim_time);
    
    // Initialize signal generation
    int num_samples = (int)(options.duration * options.sample_rate);
    double *combined_i = calloc(num_samples, sizeof(double));
    double *combined_q = calloc(num_samples, sizeof(double));
    
    // Process each satellite
    for (int prn = 1; prn <= 32; prn++) {
        if (!eph[0][prn-1].vflg) continue;
        
        // Compute satellite state
        sat_state_t sat_state;
        if (compute_satellite_state(&eph[0][prn-1], &sim_time, &sat_state) < 0)
            continue;
            
        // Compute geometry
        double range, doppler;
        compute_geometric_range(sat_state.pos, user_pos,
                              sat_state.vel, (double[]){0,0,0},
                              &range, &doppler);
        
        // Generate signal
        double *sat_i = malloc(num_samples * sizeof(double));
        double *sat_q = malloc(num_samples * sizeof(double));
        
        int result = generate_satellite_signal(
            prn, sat_i, sat_q, num_samples,
            options.sample_rate, 0.0, 0.0, doppler, -130.0);
            
        if (result > 0) {
            // Add to combined signal
            for (int i = 0; i < num_samples; i++) {
                combined_i[i] += sat_i[i];
                combined_q[i] += sat_q[i];
            }
        }
        
        free(sat_i);
        free(sat_q);
    }
    
    // Quantize and save
    int16_t *output = malloc(num_samples * 2 * sizeof(int16_t));
    double *interleaved = malloc(num_samples * 2 * sizeof(double));
    
    for (int i = 0; i < num_samples; i++) {
        interleaved[i*2] = combined_i[i];
        interleaved[i*2+1] = combined_q[i];
    }
    
    quantize_iq_samples(interleaved, output, num_samples, options.iq_bits);
    
    FILE *fp = fopen(options.out_file, "wb");
    fwrite(output, sizeof(int16_t), num_samples * 2, fp);
    fclose(fp);
    
    // Cleanup
    free(combined_i);
    free(combined_q);
    free(interleaved);
    free(output);
    
    return 0;
}
```

### Quick Signal Analysis

```c
// Analyze generated signal quality
void analyze_signal_quality(double *i_samples, double *q_samples, int num_samples) {
    // Compute signal power
    double power = 0.0;
    for (int i = 0; i < num_samples; i++) {
        power += i_samples[i] * i_samples[i] + q_samples[i] * q_samples[i];
    }
    power /= num_samples;
    
    printf("Signal power: %.2f dBW\n", 10.0 * log10(power));
    
    // Check dynamic range
    double max_val = 0.0, min_val = 0.0;
    for (int i = 0; i < num_samples; i++) {
        double magnitude = sqrt(i_samples[i]*i_samples[i] + q_samples[i]*q_samples[i]);
        if (magnitude > max_val) max_val = magnitude;
        if (magnitude < min_val || min_val == 0.0) min_val = magnitude;
    }
    
    printf("Dynamic range: %.1f dB\n", 20.0 * log10(max_val / min_val));
}
```

This API reference provides comprehensive documentation for all public functions, data structures, and usage patterns in the GPS L1C simulator. Use this reference to understand function interfaces, parameter requirements, and implementation examples.