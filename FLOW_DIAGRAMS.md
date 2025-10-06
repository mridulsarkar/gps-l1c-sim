# GPS L1C Simulator Flow Diagrams

## Overall System Flow

```mermaid
graph TD
    A[Command Line Arguments] --> B[Parse Options]
    B --> C{Navigation File Valid?}
    C -->|No| D[Error: Exit]
    C -->|Yes| E[Load RINEX Navigation Data]
    
    E --> F[Initialize Simulation Parameters]
    F --> G[Convert Start Time to GPS Time]
    G --> H[Initialize L1C Components]
    H --> I[Allocate Sample Buffers]
    
    I --> J[Main Simulation Loop]
    J --> K{More PRNs?}
    K -->|Yes| L[Process Next Satellite]
    K -->|No| M[Finalize Output]
    
    L --> N[Compute Satellite State]
    N --> O[Calculate Geometry & Delays]
    O --> P[Generate L1C Signal]
    P --> Q[Apply Channel Effects]
    Q --> R[Add to Combined Signal]
    R --> K
    
    M --> S[Quantize I/Q Samples]
    S --> T[Write Binary Output File]
    T --> U[Cleanup & Exit]
```

## Detailed Signal Processing Flow

```mermaid
graph TB
    subgraph "Satellite Processing"
        A1[PRN Input 1-63] --> B1[Compute Satellite Position/Velocity]
        B1 --> C1[Calculate Range & Doppler]
        C1 --> D1[Compute Atmospheric Delays]
        D1 --> E1[Generate L1C Codes]
    end
    
    subgraph "L1C Code Generation"
        E1 --> F1[Generate 10,230-chip Ranging Code]
        F1 --> G1[Generate 2,047-chip Overlay Code]
        G1 --> H1[L1CD = Ranging Only]
        G1 --> I1[L1CP = Ranging XOR Overlay]
    end
    
    subgraph "Modulation"
        H1 --> J1[BOC(1,1) Modulation]
        I1 --> K1[TMBOC Modulation]
        J1 --> L1[L1CD Waveform]
        K1 --> M1[L1CP Waveform]
    end
    
    subgraph "Baseband Generation"
        L1 --> N1[Generate I/Q Samples]
        M1 --> N1
        N1 --> O1[Apply Carrier & Code Phase]
        O1 --> P1[Apply Doppler Shift]
        P1 --> Q1[Scale Signal Power]
    end
    
    subgraph "Channel Effects"
        Q1 --> R1[Add Multipath Components]
        R1 --> S1[Add AWGN Noise]
        S1 --> T1[Final Satellite Signal]
    end
    
    T1 --> U1[Combine with Other Satellites]
```

## L1C Signal Structure

```mermaid
graph LR
    subgraph "L1C Signal Components"
        A[10,230-chip Ranging Code] --> B[L1CD Data Channel]
        A --> C[L1CP Pilot Channel]
        D[2,047-chip Overlay Code] --> C
        
        B --> E[BOC(1,1) Modulation]
        C --> F[TMBOC Modulation]
        
        E --> G[100 bps Navigation Data]
        F --> H[Dataless Pilot]
        
        G --> I[In-phase Component]
        H --> J[Quadrature Component]
        
        I --> K[Complex Baseband Output]
        J --> K
    end
```

## TMBOC Modulation Detail

```mermaid
graph TD
    A[Input Chip] --> B{Position in 33-chip cycle}
    B -->|Positions 1,11,17,29| C[BOC(1,1)]
    B -->|Other 29 positions| D[BOC(6,1)]
    
    C --> E[2 subcodes per chip]
    D --> F[12 subcodes per chip]
    
    E --> G[+1, -1 pattern]
    F --> H[+1,+1,+1,+1,+1,+1, -1,-1,-1,-1,-1,-1]
    
    G --> I[Output Waveform]
    H --> I
```

## RINEX Data Processing Flow

```mermaid
graph TD
    A[RINEX Navigation File] --> B[Parse Header]
    B --> C[Extract Ionosphere Parameters]
    B --> D[Extract UTC Parameters]
    
    C --> E[Store Ionosphere Data]
    D --> F[Store UTC Data]
    
    B --> G[Parse Navigation Records]
    G --> H{More Records?}
    H -->|Yes| I[Parse Ephemeris Block]
    H -->|No| J[Validation Complete]
    
    I --> K[Extract Orbital Elements]
    K --> L[Convert Time Formats]
    L --> M[Store in Ephemeris Array]
    M --> H
    
    J --> N[Return Ephemeris Count]
```

## Channel Model Implementation

```mermaid
graph TB
    A[Clean Signal Input] --> B[Direct Path Processing]
    B --> C[Apply Path Delay = 0]
    C --> D[Apply Power Scaling]
    
    A --> E[Multipath Component 1]
    E --> F[Apply Delay τ1]
    F --> G[Apply Power P1]
    G --> H[Apply Phase φ1]
    
    A --> I[Multipath Component 2]
    I --> J[Apply Delay τ2]
    J --> K[Apply Power P2]
    K --> L[Apply Phase φ2]
    
    D --> M[Sum All Components]
    H --> M
    L --> M
    
    M --> N[Add AWGN Noise]
    N --> O[Final Channel Output]
```

## Memory Management Flow

```mermaid
graph TD
    A[Start Simulation] --> B[Allocate Combined I/Q Buffers]
    B --> C[Allocate Per-Satellite Buffers]
    C --> D[Allocate Ephemeris Arrays]
    
    D --> E{Simulation Loop}
    E --> F[Generate Satellite Signal]
    F --> G[Add to Combined Buffer]
    G --> H{More Satellites?}
    H -->|Yes| E
    H -->|No| I[Allocate Output Buffer]
    
    I --> J[Quantize Samples]
    J --> K[Write to File]
    K --> L[Free All Buffers]
    L --> M[Exit Clean]
```

## Time System Conversions

```mermaid
graph LR
    subgraph "Input Time Formats"
        A[Calendar Date/Time]
        B[GPS Week/Seconds]
    end
    
    subgraph "Conversion Functions"
        A --> C[date2gps()]
        B --> D[Direct Use]
        C --> E[GPS Time Structure]
        D --> E
    end
    
    subgraph "Time Operations"
        E --> F[subGpsTime()]
        E --> G[incGpsTime()]
        F --> H[Time Differences]
        G --> I[Time Advancement]
    end
    
    subgraph "Applications"
        H --> J[Satellite Position Computation]
        I --> K[Simulation Time Stepping]
    end
```

## Build System Dependencies

```mermaid
graph TD
    A[Makefile] --> B[Detect OS Platform]
    B --> C[Set Compiler Flags]
    C --> D[Define Source Directories]
    
    D --> E[Core Sources]
    D --> F[L1C Sources]
    D --> G[Utils Sources]
    D --> H[Test Sources]
    
    E --> I[Compile Core Objects]
    F --> J[Compile L1C Objects]
    G --> K[Compile Utils Objects]
    H --> L[Compile Test Objects]
    
    I --> M[Create libgpssim.a]
    J --> M
    K --> M
    
    M --> N[Link Main Executable]
    M --> O[Link Test Executables]
    L --> O
    
    N --> P[gps-l1c-sim]
    O --> Q[test_rinex]
    O --> R[test_nav_integ]
```

## Signal Quality and Validation

```mermaid
graph TB
    A[Generated Signal] --> B[Power Measurement]
    B --> C[C/N0 Estimation]
    C --> D{Within Spec?}
    D -->|Yes| E[Signal Valid]
    D -->|No| F[Adjust Parameters]
    F --> A
    
    A --> G[Code Correlation]
    G --> H[Peak Detection]
    H --> I{Correlation Peak OK?}
    I -->|Yes| J[Code Valid]
    I -->|No| K[Regenerate Code]
    K --> A
    
    A --> L[Spectral Analysis]
    L --> M[TMBOC Spectrum Check]
    M --> N{Spectrum Correct?}
    N -->|Yes| O[Modulation Valid]
    N -->|No| P[Fix Modulation]
    P --> A
    
    E --> Q[Final Validation]
    J --> Q
    O --> Q
    Q --> R[Output Approved]
```