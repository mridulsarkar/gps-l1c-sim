# GPS L1C Signal Simulator

## Overview

This project is a comprehensive GPS L1C signal simulator that generates realistic baseband I/Q samples for GPS modernized L1C signals. The simulator implements the GPS L1C signal structure according to the Interface Control Document (ICD), including proper spreading codes, modulation schemes, and navigation message encoding.

## Features

- **GPS L1C Signal Generation**: Complete implementation of L1C data (L1CD) and pilot (L1CP) channels
- **RINEX Navigation Support**: Reads standard RINEX navigation files for ephemeris data
- **BOC/TMBOC Modulation**: Implements BOC(1,1) for data channel and TMBOC for pilot channel
- **Realistic Channel Effects**: Includes multipath, noise, and atmospheric delay modeling
- **High-Fidelity Simulation**: Accurate satellite orbit computation and signal propagation
- **Flexible Output**: Configurable sample rates, bit depths, and simulation duration

## Architecture

The simulator is organized into several key modules:

### Core Components

1. **Signal Generation Engine** (`src/core/`)
   - Main simulation controller
   - Coordinate transformations
   - Sample management

2. **L1C Signal Processing** (`src/l1c/`)
   - PRN sequence generation
   - BOC/TMBOC modulation
   - Code overlay operations

3. **Navigation & Timing** (`src/utils/`)
   - RINEX file parsing
   - Satellite position computation
   - Navigation message scheduling
   - Atmospheric delay modeling

### Key Data Structures

```c
// Simulation configuration
typedef struct {
    char nav_file[256];
    double lat, lon, alt;          // Static position
    datetime_t start_time;
    double duration;
    double sample_rate;
    int iq_bits;
    char out_file[256];
} sim_options_t;

// Satellite ephemeris data
typedef struct {
    datetime_t t;
    gpstime_t toc, toe;
    double af0, af1, af2;          // Clock correction
    double crs, deltan, m0;        // Orbit parameters
    double cuc, ecc, cus, sqrta;
    double cic, omg0, cis, inc0;
    double crc, aop, omgdot, idot;
    // ... additional orbital elements
} ephem_t;

// Channel modeling parameters
typedef struct {
    double power_dbw;
    double multipath_delays[3];
    double multipath_powers[3];
    double multipath_phases[3];
    int num_paths;
    double cn0_db;
} channel_params_t;
```

## Signal Processing Flow

### 1. Initialization Phase

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│  Parse Command  │───▶│  Load RINEX      │───▶│  Initialize     │
│  Line Options   │    │  Navigation File │    │  L1C Components │
└─────────────────┘    └──────────────────┘    └─────────────────┘
```

### 2. Signal Generation Loop

```
┌──────────────────┐
│  For each PRN    │
│  (1 to 63)       │
└─────────┬────────┘
          │
          ▼
┌─────────────────────────┐
│  Compute Satellite      │
│  Position & Velocity    │
│  at simulation time     │
└─────────┬───────────────┘
          │
          ▼
┌─────────────────────────┐
│  Calculate Geometric    │
│  Range & Doppler Shift  │
└─────────┬───────────────┘
          │
          ▼
┌─────────────────────────┐
│  Compute Atmospheric    │
│  Delays (Tropo + Iono)  │
└─────────┬───────────────┘
          │
          ▼
┌─────────────────────────┐
│  Generate L1C Codes:    │
│  • L1CD (ranging only)  │
│  • L1CP (ranging×ovly)  │
└─────────┬───────────────┘
          │
          ▼
┌─────────────────────────┐
│  Apply Modulation:      │
│  • BOC(1,1) for L1CD    │
│  • TMBOC for L1CP       │
└─────────┬───────────────┘
          │
          ▼
┌─────────────────────────┐
│  Generate Complex       │
│  Baseband Samples       │
│  (I/Q components)       │
└─────────┬───────────────┘
          │
          ▼
┌─────────────────────────┐
│  Apply Channel Effects: │
│  • Multipath           │
│  • AWGN                │
│  • Power scaling       │
└─────────┬───────────────┘
          │
          ▼
┌─────────────────────────┐
│  Add to Combined        │
│  Signal Buffer          │
└─────────────────────────┘
```

### 3. Output Processing

```
┌─────────────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│  Interleave I/Q         │───▶│  Quantize to     │───▶│  Write Binary   │
│  Samples                │    │  Target Bit      │    │  Output File    │
│                         │    │  Depth           │    │                 │
└─────────────────────────┘    └──────────────────┘    └─────────────────┘
```

## L1C Signal Structure

The GPS L1C signal consists of two components transmitted in quadrature:

### L1CD (Data Channel)
- **Modulation**: BOC(1,1)
- **Code**: 10,230-chip ranging code
- **Data Rate**: 100 bps navigation data
- **Purpose**: Carries CNAV-2 navigation messages

### L1CP (Pilot Channel)  
- **Modulation**: TMBOC (Time-Multiplexed BOC)
- **Code**: 10,230-chip ranging code XOR 2,047-chip overlay
- **Data**: Dataless (pilot signal)
- **Purpose**: Enhanced tracking performance

### Code Generation Process

```
┌─────────────────┐    ┌──────────────────┐
│  Generate       │───▶│  Generate        │
│  10,230-chip    │    │  2,047-chip      │
│  Ranging Code   │    │  Overlay Code    │
│  (PRN specific) │    │  (S1 sequence)   │
└─────────┬───────┘    └─────────┬────────┘
          │                      │
          ▼                      │
┌─────────────────┐              │
│  L1CD = Ranging │              │
│  Code Only      │              │
└─────────────────┘              │
                                 │
          ┌──────────────────────┘
          │
          ▼
┌─────────────────┐
│  L1CP = Ranging │
│  XOR Overlay    │
└─────────────────┘
```

## Module Descriptions

### Core Modules

#### `gps-l1c-sim.c`
Main simulation engine that:
- Parses command-line options
- Coordinates the simulation process
- Manages memory allocation
- Controls output file generation

#### `rinex.c`
RINEX navigation file parser:
- Reads broadcast ephemeris data
- Extracts ionospheric parameters
- Handles time format conversions
- Validates data integrity

#### `satpos.c`
Satellite position computation:
- Implements Kepler orbital mechanics
- Computes satellite position and velocity
- Calculates signal propagation delays
- Handles coordinate transformations

### L1C Signal Modules

#### `l1c_seq.c`
PRN sequence generation:
- Generates satellite-specific ranging codes
- Creates overlay sequences
- Combines codes for L1CD and L1CP channels

#### `l1c_mod.c`
Signal modulation:
- Implements BOC(1,1) for data channel
- Implements TMBOC for pilot channel
- Handles sub-carrier generation

#### `l1c.c`
High-level L1C signal coordination:
- Manages PRN code generation
- Coordinates modulation schemes
- Interfaces with navigation scheduler

### Utility Modules

#### `channel.c`
Channel modeling:
- Simulates multipath propagation
- Adds atmospheric noise (AWGN)
- Applies fading and interference effects

#### `signal.c`
Baseband signal processing:
- Generates complex I/Q samples
- Applies Doppler shifts
- Quantizes floating-point samples

#### `nav_cnav.c` & `nav_scheduler.c`
Navigation message handling:
- Generates CNAV-2 messages
- Schedules navigation data transmission
- Implements CRC-24Q error detection

## Build System

The project uses a comprehensive Makefile supporting:

- **Cross-platform builds** (Linux, Windows, macOS)
- **Modular compilation** with separate object files
- **Static library creation** (`libgpssim.a`)
- **Test program generation**

### Build Targets

```bash
# Build everything
make all

# Clean build artifacts
make clean

# Build specific components
make $(LIBGPSSIM)    # Static library only
make $(MAIN_EXE)     # Main simulator
make $(TEST_EXES)    # Test programs
```

## Usage Examples

### Basic Simulation
```bash
./bin/gps-l1c-sim -e brdc0010.22n -p 37.7749,-122.4194,100 -d 10.0 -o samples.bin
```

### Advanced Configuration
```bash
./bin/gps-l1c-sim \
    -e navigation.nav \           # RINEX navigation file
    -p 40.7128,-74.0060,50 \     # NYC coordinates, 50m altitude
    -t 2022,1,1,12,0,0 \         # Start time: Jan 1, 2022 12:00:00
    -d 30.0 \                    # 30-second simulation
    -s 25000000 \                # 25 MHz sampling rate
    -b 16 \                      # 16-bit I/Q samples
    -o gps_samples.bin           # Output file
```

## Testing Framework

### Test Programs

1. **`test_rinex`**: Validates RINEX file parsing
2. **`test_nav_integ`**: Tests navigation message integration

### Validation Process

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│  Load Test      │───▶│  Execute Signal  │───▶│  Verify Output  │
│  Configuration  │    │  Generation      │    │  Characteristics│
└─────────────────┘    └──────────────────┘    └─────────────────┘
```

## Technical Specifications

### Signal Parameters
- **Carrier Frequency**: 1575.42 MHz (L1)
- **Chip Rate**: 1.023 Mcps
- **Code Length**: 10,230 chips (L1CD/L1CP)
- **Overlay Length**: 2,047 chips (L1CP only)
- **Navigation Data Rate**: 100 bps (L1CD)

### Modulation Schemes
- **L1CD**: BOC(1,1) - Binary Offset Carrier
- **L1CP**: TMBOC - Time-Multiplexed BOC
  - 4 BOC(1,1) positions per 33-chip cycle
  - 29 BOC(6,1) positions per 33-chip cycle

### Output Formats
- **Sample Rates**: Configurable (default: 50 MHz)
- **Bit Depths**: 8-bit or 16-bit I/Q
- **File Format**: Binary interleaved I/Q samples

## Dependencies

### External Libraries
- **Standard C Library**: Math functions, file I/O
- **POSIX**: getopt for command-line parsing

### Internal Dependencies
```
gps-l1c-sim
├── libgpssim.a
│   ├── Core modules (rinex, satpos)
│   ├── L1C modules (l1c_seq, l1c_mod, l1c)
│   └── Utility modules (channel, signal, nav_*)
└── System libraries (-lm)
```

## Performance Considerations

### Memory Usage
- **Ephemeris Storage**: ~40 epochs × 64 satellites
- **Sample Buffers**: Proportional to duration × sample_rate
- **Code Generation**: Pre-computed and cached

### Computational Complexity
- **O(N)**: Linear with number of samples
- **O(S)**: Linear with number of visible satellites
- **Real-time Capability**: Depends on target hardware

## Future Enhancements

### Planned Features
1. **Dynamic User Motion**: Support for trajectory-based simulation
2. **Multi-Constellation**: GLONASS, Galileo, BeiDou support
3. **Advanced Channel Models**: Urban canyon, indoor propagation
4. **Real-time Operation**: Streaming output capabilities
5. **Signal Quality Metrics**: Built-in C/N0 estimation

### Extension Points
- **Custom Channel Models**: Plugin architecture for channel effects
- **Navigation Message Customization**: User-defined CNAV content
- **Output Format Support**: MATLAB, GNU Radio integration

## Contributing

### Code Organization
- **Header Files**: `include/` - API definitions and data structures
- **Source Files**: `src/` - Implementation organized by functionality
- **Test Files**: `test/` - Unit and integration tests
- **Build System**: `Makefile` - Cross-platform build configuration

### Development Guidelines
1. Follow existing code style and naming conventions
2. Add comprehensive comments for new algorithms
3. Include test cases for new functionality
4. Update documentation for API changes

## License

This GPS L1C signal simulator is provided for educational and research purposes. Users should comply with applicable regulations regarding GPS signal simulation and RF emissions.