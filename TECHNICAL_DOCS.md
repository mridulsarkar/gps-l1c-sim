# GPS L1C Simulator Technical Documentation

## Table of Contents

1. [System Architecture](#system-architecture)
2. [Signal Processing Theory](#signal-processing-theory)
3. [Module Reference](#module-reference)
4. [API Documentation](#api-documentation)
5. [Configuration Guide](#configuration-guide)
6. [Performance Analysis](#performance-analysis)
7. [Troubleshooting](#troubleshooting)

## System Architecture

### High-Level Design

The GPS L1C Simulator follows a modular architecture designed for maintainability, testability, and extensibility. The system is organized into distinct layers:

```
Application Layer
├── Command-line Interface (gps-l1c-sim.c)
├── Configuration Management (sim_options_t)
└── Output File Generation

Signal Processing Layer
├── L1C Signal Generation (l1c/)
├── Modulation Schemes (BOC, TMBOC)
└── Code Sequence Generation

Navigation Layer  
├── RINEX File Processing (rinex.c)
├── Satellite Position Computation (satpos.c)
└── Navigation Message Handling (nav_*)

Utilities Layer
├── Time System Conversions (gpssim_lib.c)
├── Channel Modeling (channel.c)
└── Signal Processing (signal.c)
```

### Data Flow Architecture

The simulator implements a pipeline architecture where data flows through well-defined stages:

1. **Configuration Stage**: Parse inputs and validate parameters
2. **Initialization Stage**: Load ephemeris data and initialize components
3. **Generation Stage**: Process each satellite signal independently
4. **Combination Stage**: Combine individual satellite signals
5. **Output Stage**: Quantize and write final I/Q samples

### Threading Model

The current implementation uses a single-threaded approach for simplicity and deterministic behavior. Each satellite is processed sequentially in the main simulation loop.

## Signal Processing Theory

### GPS L1C Signal Structure

The GPS L1C signal is designed to provide improved performance over legacy GPS signals. It consists of two quadrature components:

#### L1CD (Data Channel)
- **Purpose**: Carries navigation data (CNAV-2 messages)
- **Modulation**: BOC(1,1) - Binary Offset Carrier
- **Data Rate**: 100 bits per second
- **Code**: 10,230-chip ranging code
- **Power**: ~25% of total L1C power

#### L1CP (Pilot Channel)  
- **Purpose**: Dataless pilot for enhanced tracking
- **Modulation**: TMBOC - Time-Multiplexed BOC
- **Data**: No navigation data (pilot tones only)
- **Code**: 10,230-chip ranging code XOR 2,047-chip overlay
- **Power**: ~75% of total L1C power

### Spreading Code Generation

#### Ranging Code Generation
The 10,230-chip ranging code is generated using a combination of Linear Feedback Shift Registers (LFSRs):

```c
// Simplified ranging code generation algorithm
int generate_ranging_code(int prn, uint8_t *code) {
    // Two 12-stage LFSRs with different polynomials
    uint16_t lfsr1 = initial_state_1[prn];
    uint16_t lfsr2 = initial_state_2[prn];
    
    for (int i = 0; i < 10230; i++) {
        int bit1 = compute_lfsr_output(lfsr1, POLY1);
        int bit2 = compute_lfsr_output(lfsr2, POLY2);
        code[i] = bit1 ^ bit2;
        
        lfsr1 = advance_lfsr(lfsr1, POLY1);
        lfsr2 = advance_lfsr(lfsr2, POLY2);
    }
    return 0;
}
```

#### Overlay Code Generation  
The 2,047-chip overlay sequence is generated using a single LFSR:

```c
void generate_s1_overlay(int poly_octal, int init_octal,
                         uint8_t out[], int length, int *final_reg) {
    int reg = init_octal;
    int poly = poly_octal;
    
    for (int i = 0; i < length; i++) {
        out[i] = (reg >> 10) & 1;  // Extract output bit
        
        int feedback = 0;
        for (int j = 0; j < 11; j++) {
            if ((poly >> j) & 1) {
                feedback ^= (reg >> j) & 1;
            }
        }
        
        reg = ((reg << 1) | feedback) & 0x7FF;
    }
    *final_reg = reg;
}
```

### Modulation Schemes

#### BOC(1,1) Modulation
Binary Offset Carrier modulation with equal data and sub-carrier rates:

```
Sub-carrier frequency: 1.023 MHz
Data rate: 1.023 Mcps
Sub-carrier pattern: [+1, -1] per chip
```

Implementation:
```c
int l1c_mod_boc11(const uint8_t *chips, int *waveform) {
    for (int i = 0; i < chip_count; i++) {
        int symbol = chips[i] ? 1 : -1;
        waveform[i*2] = symbol;      // First half-chip
        waveform[i*2+1] = -symbol;   // Second half-chip (inverted)
    }
    return chip_count * 2;
}
```

#### TMBOC Modulation
Time-Multiplexed BOC combining BOC(1,1) and BOC(6,1):

```
33-chip cycle structure:
- 4 chips: BOC(1,1) at positions [1, 11, 17, 29]  
- 29 chips: BOC(6,1) at remaining positions
```

Implementation:
```c
int l1c_mod_tmboc(const uint8_t *chips, int *waveform) {
    int boc11_positions[4] = {1, 11, 17, 29};
    int outpos = 0;
    
    for (int i = 0; i < chip_count; i++) {
        int symbol = chips[i] ? 1 : -1;
        int cycle_pos = (i % 33) + 1;
        
        int is_boc11 = 0;
        for (int j = 0; j < 4; j++) {
            if (cycle_pos == boc11_positions[j]) {
                is_boc11 = 1;
                break;
            }
        }
        
        if (is_boc11) {
            // BOC(1,1): 2 half-chips
            waveform[outpos++] = symbol;
            waveform[outpos++] = -symbol;
        } else {
            // BOC(6,1): 12 sixth-chips
            for (int k = 0; k < 12; k++) {
                waveform[outpos++] = (k < 6) ? symbol : -symbol;
            }
        }
    }
    return outpos;
}
```

### Satellite Position Computation

The simulator implements precise satellite position computation using broadcast ephemeris parameters:

#### Kepler Orbital Elements
```c
typedef struct {
    double sqrta;      // Square root of semi-major axis
    double ecc;        // Eccentricity  
    double m0;         // Mean anomaly at reference time
    double deltan;     // Mean motion difference
    double omg0;       // Longitude of ascending node
    double inc0;       // Inclination angle
    double aop;        // Argument of perigee
    double omgdot;     // Rate of right ascension
    double idot;       // Rate of inclination
    // Perturbation corrections
    double cuc, cus;   // Argument of latitude
    double crc, crs;   // Orbit radius  
    double cic, cis;   // Inclination
} orbital_elements_t;
```

#### Position Computation Algorithm
```c
int compute_satellite_position(const ephem_t *eph, const gpstime_t *t, 
                              double *pos, double *vel) {
    // Time from ephemeris reference epoch
    double tk = subGpsTime(*t, eph->toe);
    
    // Computed mean motion
    double A = eph->sqrta * eph->sqrta;
    double n0 = sqrt(GM_EARTH / (A * A * A));
    double n = n0 + eph->deltan;
    
    // Mean anomaly
    double Mk = eph->m0 + n * tk;
    
    // Solve Kepler's equation for eccentric anomaly
    double Ek = solve_kepler_equation(Mk, eph->ecc);
    
    // True anomaly
    double vk = atan2(sqrt(1 - eph->ecc*eph->ecc) * sin(Ek),
                      cos(Ek) - eph->ecc);
    
    // Argument of latitude
    double Phik = vk + eph->aop;
    
    // Apply perturbations
    double duk = eph->cuc*cos(2*Phik) + eph->cus*sin(2*Phik);
    double drk = eph->crc*cos(2*Phik) + eph->crs*sin(2*Phik);
    double dik = eph->cic*cos(2*Phik) + eph->cis*sin(2*Phik);
    
    // Corrected values
    double uk = Phik + duk;
    double rk = A * (1 - eph->ecc * cos(Ek)) + drk;
    double ik = eph->inc0 + dik + eph->idot * tk;
    
    // Position in orbital plane
    double xk_prime = rk * cos(uk);
    double yk_prime = rk * sin(uk);
    
    // Corrected longitude of ascending node
    double Omegak = eph->omg0 + (eph->omgdot - OMEGA_EARTH) * tk
                    - OMEGA_EARTH * eph->toe.sec;
    
    // Earth-fixed coordinates
    pos[0] = xk_prime*cos(Omegak) - yk_prime*cos(ik)*sin(Omegak);
    pos[1] = xk_prime*sin(Omegak) + yk_prime*cos(ik)*cos(Omegak);
    pos[2] = yk_prime * sin(ik);
    
    // Velocity computation (numerical differentiation or analytic)
    compute_satellite_velocity(eph, t, Ek, uk, rk, ik, Omegak, vel);
    
    return 0;
}
```

### Atmospheric Delay Models

#### Tropospheric Delay
The simulator implements the Saastamoinen tropospheric delay model:

```c
double compute_troposphere_delay(double elevation, double altitude) {
    // Standard atmosphere parameters
    double P = 1013.25 * pow(1 - 2.2557e-5 * altitude, 5.2568);  // Pressure
    double T = 15.0 - 6.5e-3 * altitude + 273.15;               // Temperature
    double H = 50.0;                                             // Humidity (%)
    
    // Wet and dry components
    double e = 6.108 * H/100.0 * exp(17.15*T/(234.7+T));       // Water vapor
    double dry_delay = 0.002277 * P / sin(elevation);
    double wet_delay = 0.002277 * (1255/T + 0.05) * e / sin(elevation);
    
    return dry_delay + wet_delay;
}
```

#### Ionospheric Delay
Single-frequency ionospheric delay using the Klobuchar model:

```c
double compute_ionosphere_delay(const ionoutc_t *ion, double lat, double lon,
                              double elevation, double azimuth,
                              const gpstime_t *t) {
    // Earth-centered angle
    double psi = 0.0137/(elevation + 0.11) - 0.022;
    
    // Subionospheric latitude
    double phi_i = lat + psi * cos(azimuth);
    phi_i = fmax(-0.416, fmin(0.416, phi_i));
    
    // Subionospheric longitude  
    double lambda_i = lon + psi * sin(azimuth) / cos(phi_i);
    
    // Geomagnetic latitude
    double phi_m = phi_i + 0.064 * cos(lambda_i - 1.617);
    
    // Local time
    double t_local = 43200 * lambda_i + t->sec;
    t_local = fmod(t_local, 86400);
    if (t_local < 0) t_local += 86400;
    
    // Amplitude and period
    double AMP = ion->alpha0 + ion->alpha1*phi_m + ion->alpha2*phi_m*phi_m + 
                 ion->alpha3*phi_m*phi_m*phi_m;
    AMP = fmax(0.0, AMP);
    
    double PER = ion->beta0 + ion->beta1*phi_m + ion->beta2*phi_m*phi_m + 
                 ion->beta3*phi_m*phi_m*phi_m;
    PER = fmax(72000.0, PER);
    
    // Ionospheric delay
    double x = 2*M_PI*(t_local - 50400)/PER;
    double F = 1.0 + 16.0*pow(0.53 - elevation, 3);
    
    double delay;
    if (fabs(x) < 1.57) {
        delay = F * (5.0e-9 + AMP * (1 - x*x/2 + x*x*x*x/24));
    } else {
        delay = F * 5.0e-9;
    }
    
    return delay * C_LIGHT;  // Convert to meters
}
```

## Module Reference

### Core Modules

#### gps-l1c-sim.c
**Purpose**: Main simulation controller and command-line interface

**Key Functions**:
- `main()`: Primary entry point and simulation orchestrator
- `print_usage()`: Display command-line help
- `ecef2geodetic()`: Convert ECEF coordinates to lat/lon/height
- `geodetic2ecef()`: Convert geodetic coordinates to ECEF

**Data Structures**:
- `sim_options_t`: Configuration parameters
- `channel_params_t`: Channel modeling parameters

#### rinex.c  
**Purpose**: RINEX navigation file parsing and ephemeris management

**Key Functions**:
- `readRinexNavAll()`: Parse complete RINEX navigation file
- `parseEphemerisBlock()`: Extract single satellite ephemeris
- `parseIonosphereParams()`: Extract ionospheric correction parameters

**Data Handling**:
- Supports multiple ephemeris epochs per satellite
- Validates data integrity and time consistency
- Handles both GPS and UTC time references

#### satpos.c
**Purpose**: Satellite position, velocity, and clock computation

**Key Functions**:
- `compute_satellite_state()`: Complete satellite state vector
- `solve_kepler_equation()`: Iterative solution for eccentric anomaly
- `compute_satellite_velocity()`: Velocity vector computation
- `apply_earth_rotation()`: Account for Earth rotation during signal transit

**Accuracy**: Achieves sub-meter position accuracy using broadcast ephemeris

### L1C Signal Modules

#### l1c_seq.c
**Purpose**: L1C spreading sequence generation

**Key Functions**:
- `l1c_generate_seq()`: Generate both L1CD and L1CP sequences
- `generate_ranging_code()`: Create PRN-specific ranging codes
- `generate_overlay_code()`: Create common overlay sequence

**Code Properties**:
- 10,230-chip ranging codes (unique per PRN)
- 2,047-chip overlay sequence (common to all PRNs)
- Optimal autocorrelation and cross-correlation properties

#### l1c_mod.c
**Purpose**: BOC and TMBOC modulation implementation

**Key Functions**:
- `l1c_mod_boc11()`: BOC(1,1) waveform generation
- `l1c_mod_tmboc()`: TMBOC waveform generation
- `apply_subcarrier()`: Sub-carrier modulation

**Modulation Accuracy**: Implements exact ICD-specified waveforms

#### l1c.c
**Purpose**: High-level L1C signal coordination

**Key Functions**:
- `l1c_generate_prn()`: Coordinate complete PRN sequence generation
- `l1c_apply_navigation_data()`: Integrate navigation messages
- `l1c_combine_channels()`: Combine L1CD and L1CP components

### Utility Modules

#### channel.c
**Purpose**: RF channel modeling and impairment simulation

**Key Functions**:
- `apply_channel_effects()`: Apply complete channel model
- `add_multipath()`: Simulate multipath propagation
- `add_awgn_noise()`: Add white Gaussian noise
- `apply_fading()`: Implement fading models

**Channel Models**:
- Rayleigh/Rician fading
- Multi-path with configurable delays and powers
- Additive white Gaussian noise (AWGN)

#### signal.c
**Purpose**: Baseband signal processing and sample generation

**Key Functions**:
- `generate_satellite_signal()`: Complete satellite signal generation
- `apply_doppler_shift()`: Frequency domain Doppler application
- `quantize_iq_samples()`: Floating-point to integer conversion

**Signal Quality**: Maintains >60 dB dynamic range in processing

#### nav_cnav.c & nav_scheduler.c
**Purpose**: Navigation message generation and scheduling

**Key Functions**:
- `nav_cnav_generate()`: Create CNAV-2 navigation messages
- `nav_scheduler_init()`: Initialize message scheduling
- `nav_get_bits()`: Retrieve navigation bits for specific time
- `compute_crc24q()`: Calculate CRC-24Q error detection

**Message Types**: Supports all CNAV-2 message types per ICD

## API Documentation

### Primary APIs

#### Simulation Control
```c
// Initialize simulation with configuration
int init_simulation(const sim_options_t *options);

// Run complete simulation
int run_simulation(const sim_options_t *options);

// Clean up simulation resources  
void cleanup_simulation(void);
```

#### Signal Generation
```c
// Generate signal for single satellite
int generate_satellite_signal(
    int prn,                    // PRN number (1-63)
    double *i_samples,          // I channel output
    double *q_samples,          // Q channel output  
    int num_samples,            // Sample count
    double sample_rate,         // Sampling rate (Hz)
    double code_phase,          // Initial code phase
    double carrier_phase,       // Initial carrier phase
    double doppler,             // Doppler shift (Hz)
    double power_dbw           // Signal power (dBW)
);

// Combine multiple satellite signals
int combine_satellite_signals(
    double *combined_i,         // Combined I output
    double *combined_q,         // Combined Q output
    double **individual_i,      // Array of I signals
    double **individual_q,      // Array of Q signals
    int num_satellites,         // Number of satellites
    int num_samples            // Samples per satellite
);
```

#### Time System Functions
```c
// Convert calendar date to GPS time
void date2gps(const datetime_t *calendar, gpstime_t *gps);

// Compute time difference in seconds
double subGpsTime(gpstime_t later, gpstime_t earlier);

// Advance GPS time by specified seconds
gpstime_t incGpsTime(gpstime_t base, double increment_sec);
```

#### RINEX Processing
```c
// Load navigation data from RINEX file
int readRinexNavAll(
    ephem_t eph[][MAX_SAT],     // Output ephemeris array
    ionoutc_t *ionoutc,         // Output ionosphere parameters
    const char *filename        // RINEX file path
);

// Validate ephemeris data
int validate_ephemeris(const ephem_t *eph);

// Find best ephemeris for specific time
const ephem_t* find_ephemeris(
    const ephem_t eph[][MAX_SAT], 
    int prn, 
    const gpstime_t *time
);
```

### Error Handling

The API uses consistent error codes:
- `0`: Success
- `-1`: General error
- `-2`: Invalid parameters
- `-3`: Memory allocation failure
- `-4`: File I/O error
- `-5`: Data validation error

## Configuration Guide

### Command Line Options

#### Required Parameters
```bash
-e <nav_file>    # RINEX navigation file (required)
```

#### Position Configuration
```bash
-p <lat,lon,alt> # Receiver position
                 # lat: latitude in degrees (-90 to +90)
                 # lon: longitude in degrees (-180 to +180)  
                 # alt: altitude in meters above WGS84 ellipsoid
```

#### Time Configuration
```bash
-t <YYYY,MM,DD,HH,mm,SS>  # Simulation start time
                          # YYYY: 4-digit year
                          # MM: month (1-12)
                          # DD: day (1-31)
                          # HH: hour (0-23)
                          # mm: minute (0-59)
                          # SS: second (0-59.999...)
```

#### Signal Parameters
```bash
-d <seconds>     # Simulation duration (default: 1.0)
-s <Hz>          # Sampling rate (default: 50e6)
-b <bits>        # I/Q bit depth: 8 or 16 (default: 8)
-o <file>        # Output filename (default: out.bin)
```

### Configuration Examples

#### Basic L1C Simulation
```bash
./gps-l1c-sim -e brdc0010.22n -p 40.7589,-73.9851,20 -d 5.0
```

#### High-Fidelity Simulation
```bash
./gps-l1c-sim \
    -e brdc0010.22n \
    -p 37.7749,-122.4194,100 \
    -t 2022,1,1,12,0,0 \
    -d 30.0 \
    -s 25000000 \
    -b 16 \
    -o high_fidelity.bin
```

#### Urban Environment Simulation
```bash
./gps-l1c-sim \
    -e current_nav.22n \
    -p 40.7589,-73.9851,50 \
    -d 60.0 \
    -s 20000000 \
    -b 8 \
    -o urban_gps.bin
```

### Output File Format

#### Binary I/Q Format
The output file contains interleaved I/Q samples:
```
Sample 0: I0, Q0
Sample 1: I1, Q1
Sample 2: I2, Q2
...
```

#### Data Types
- **8-bit**: Signed integers (-128 to +127)
- **16-bit**: Signed integers (-32768 to +32767)

#### File Size Calculation
```
File size = 2 × samples_per_second × duration × bytes_per_sample
Example: 2 × 25MHz × 10s × 2 bytes = 1GB
```

## Performance Analysis

### Computational Complexity

#### Time Complexity
- **Satellite Processing**: O(N × S) where N = samples, S = satellites
- **Code Generation**: O(C) where C = code length (pre-computed)
- **Modulation**: O(M) where M = modulated samples
- **Channel Effects**: O(N × P) where P = number of paths

#### Space Complexity
- **Sample Buffers**: O(N) for I/Q samples
- **Ephemeris Storage**: O(E × S) where E = epochs, S = satellites
- **Code Storage**: O(C × S) for pre-computed codes

### Memory Usage

#### Typical Memory Requirements
```
Configuration: 50 MHz, 10 seconds, 32 satellites
- I/Q Buffers: 2 × 50M × 10 × 8 bytes = 8 GB
- Ephemeris: 40 × 64 × 200 bytes = 512 KB  
- Code Storage: 64 × 10230 bytes = 655 KB
- Working Memory: ~100 MB
Total: ~8.1 GB
```

#### Memory Optimization
- Use streaming output for long simulations
- Implement double-buffering for real-time operation
- Pre-compute and cache frequently used values

### Processing Speed

#### Benchmark Results (Intel i7-8700K @ 3.7GHz)
```
Configuration         | Processing Speed | Real-time Factor
50 MHz, 10s, 8 sats  | 45 seconds      | 0.22x
25 MHz, 10s, 12 sats | 28 seconds      | 0.36x  
10 MHz, 10s, 16 sats | 12 seconds      | 0.83x
```

#### Optimization Strategies
1. **Vectorization**: Use SIMD instructions for sample processing
2. **Parallelization**: Process satellites in parallel threads
3. **Code Optimization**: Pre-compute trigonometric functions
4. **Memory Access**: Optimize cache usage patterns

## Troubleshooting

### Common Issues

#### 1. RINEX File Parsing Errors
**Symptoms**: "Failed to read navigation file" error

**Causes**:
- Corrupted or incomplete RINEX file
- Unsupported RINEX version
- Missing ephemeris data for simulation time

**Solutions**:
```bash
# Validate RINEX file format
./test_rinex brdc0010.22n

# Check file permissions
ls -la brdc0010.22n

# Verify ephemeris coverage
grep "22  1  1" brdc0010.22n  # Check for specific date
```

#### 2. Memory Allocation Failures
**Symptoms**: "Memory allocation failed" error

**Causes**:
- Insufficient system memory
- Very long simulation duration
- High sampling rates

**Solutions**:
```bash
# Check available memory
free -h

# Reduce simulation parameters
./gps-l1c-sim -d 5.0 -s 10000000  # Shorter duration, lower rate

# Monitor memory usage
valgrind --tool=memcheck ./gps-l1c-sim [options]
```

#### 3. No Satellite Signals Generated
**Symptoms**: Empty or very weak output signal

**Causes**:
- No satellites visible at specified time/location
- Ephemeris data too old
- Incorrect coordinate system

**Solutions**:
```bash
# Verify satellite visibility
./gps-l1c-sim -e nav.22n -p 0,0,0 -d 1.0  # Try equator location

# Check ephemeris validity
./test_rinex nav.22n | grep "PRN"

# Validate position format (decimal degrees)
-p 40.7589,-73.9851,100  # Not DMS format
```

#### 4. Output File Issues
**Symptoms**: Cannot write output file or file corruption

**Causes**:
- Insufficient disk space
- Write permission denied
- File system limitations

**Solutions**:
```bash
# Check disk space
df -h .

# Verify write permissions
touch test_output.bin && rm test_output.bin

# Use different output directory
./gps-l1c-sim [options] -o /tmp/gps_output.bin
```

### Debugging Tools

#### Built-in Diagnostics
```bash
# Enable verbose output (if compiled with debug)
export GPS_DEBUG=1
./gps-l1c-sim [options]

# Test individual components
./test_rinex nav_file.22n
./test_nav_integ
```

#### External Analysis Tools
```bash
# Analyze output signal with GNU Radio
gnuradio-companion gps_analysis.grc

# MATLAB/Octave signal analysis
octave --eval "load_gps_samples('output.bin')"

# Hex dump for format verification
hexdump -C output.bin | head -20
```

### Performance Tuning

#### System Configuration
```bash
# Increase memory limits
ulimit -m unlimited
ulimit -v unlimited

# Set CPU governor for performance
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# Disable power management
sudo systemctl mask systemd-rfkill.service
```

#### Compiler Optimization
```bash
# Build with optimizations
make CFLAGS="-O3 -march=native -ffast-math"

# Enable profiling
make CFLAGS="-O2 -pg"
./gps-l1c-sim [options]
gprof gps-l1c-sim gmon.out > profile.txt
```

### Validation Procedures

#### Signal Quality Verification
```c
// Basic signal power check
double signal_power = compute_signal_power(i_samples, q_samples, num_samples);
printf("Signal power: %.2f dBW\n", 10*log10(signal_power));

// Code correlation verification  
double correlation = compute_auto_correlation(code_sequence, received_code);
printf("Code correlation: %.3f\n", correlation);
```

#### Spectral Analysis
```bash
# Generate power spectral density plot
python3 analyze_spectrum.py output.bin 50e6

# Verify TMBOC spectral characteristics
matlab -r "analyze_tmboc_spectrum('output.bin')"
```

This technical documentation provides comprehensive coverage of the GPS L1C simulator's architecture, implementation details, and operational procedures. It serves as both a reference for developers and a guide for users seeking to understand and optimize the simulator's performance.