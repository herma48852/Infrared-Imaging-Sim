# Infrared-Imaging-Sim: Airborne EO/IR Tactical Sensor & Target Detection Simulation

A production-grade, real-time Passive Infrared (LWIR 8–14 µm / MWIR 3–5 µm) Radiometric Simulation, Sensor Degradation, and Tactical Detection/Tracking Engine for UAV/Drone Platforms.

Built specifically for aerospace EO/IR engineering and aligned with tactical requirements for **Anduril Imaging Group**, this project models the complete electro-optical imaging chain—from **first-principles Planck radiometry** and **atmospheric MODTRAN radiative transfer**, through **FPA microbolometer noise physics**, to **embedded ISP calibration**, **multiscale CA-CFAR / LCM point-target detection**, and **aerospace Kalman multi-target tracking**.

---

## Key Highlights & Performance

- **Sub-Pixel Point Target Resolution**: Explicitly capable of resolving and detecting objects as small as **wild birds and micro-drones** ($1\times1$ to $5\times5$ pixel point targets at $500\text{ m} - 1200\text{ m}$ slant range) using **Multiscale Local Contrast Measure (LCM)** and **2D CA-CFAR** with sub-pixel **Gaussian optical PSF radiant flux conservation**.
- **Extreme Throughput on Apple Silicon**:
  - **Scene Radiative Transfer**: $\sim 2.56\text{ ms}$
  - **FPA Sensor Transduction**: $\sim 1.76\text{ ms}$
  - **Embedded ISP Pipeline**: $\sim 0.39\text{ ms}$
  - **CFAR / LCM Tactical Detection**: $\sim 1.09\text{ ms}$
  - **Multi-Target Kalman Tracking**: $\sim 0.003\text{ ms}$
  - **Total Pipeline Latency**: **$\sim 5.8\text{ ms}$** per $640\times512$ frame (**$172\text{+ FPS}$** theoretical maximum throughput).
- **Native Apple Metal 3 + Dear ImGui UI**: Zero-allocation texture upload via `MTLTexture replaceRegion` with interactive 4-viewport tactical display (Raw FPA, Cleaned NUC, Visual AGC Display, and Detection SCR Map).
- **Flight & Gimbal Controls**: Interactive flight modes (**Waypoint**, **Orbit**, **Manual Teleoperation**, **Target Pursuit**) and stabilized gimbal (**Nadir**, **GeoLock**, and **high-frequency structural vibration jitter** at 50, 120, and 200 Hz).
- **Full Test Coverage**: 36 comprehensive test suites validating Planck Stefan-Boltzmann quadrature, MODTRAN Beer-Lambert transmittance, 6-DOF RK4 kinematics, 2-point NUC recovery, and ByteTrack track continuity.

---

## System Architecture

```
                  ┌────────────────────────────────────────┐
                  │       Scenario Manifest (JSON)         │
                  │   Environment, 4D Waypoints, Materials │
                  └───────────────────┬────────────────────┘
                                      │
                                      ▼
                  ┌────────────────────────────────────────┐
                  │       Synthetic Scene Radiometry       │
                  │   Diurnal Solar Heating, Planck LWIR,  │
                  │   Beer-Lambert Extinction, Sub-px PSF  │
                  └───────────────────┬────────────────────┘
                                      │ [At-Aperture Radiance]
                                      ▼
                  ┌────────────────────────────────────────┐
                  │       FPA Sensor Transduction          │
                  │   Non-Uniformity (Gain/Offset FPN),    │
                  │   Column Striping, NETD Noise, ADC     │
                  └───────────────────┬────────────────────┘
                                      │ [14-bit Raw Counts]
                                      ▼
                  ┌────────────────────────────────────────┐
                  │       Embedded ISP Pipeline            │
                  │   2-Point NUC Calibration (>95% FPN),  │
                  │   8-Neighbor BPR, Plateau AGC / CLAHE  │
                  └───────────────────┬────────────────────┘
                                      │ [Calibrated NUC Frame]
                                      ▼
                  ┌────────────────────────────────────────┐
                  │       Tactical Detection Engine        │
                  │   Dual-Window 2D CA-CFAR, O(1) Integral│
                  │   Images, Multiscale LCM Saliency      │
                  └───────────────────┬────────────────────┘
                                      │ [Candidate Detections]
                                      ▼
                  ┌────────────────────────────────────────┐
                  │      Multi-Target Kalman Tracker       │
                  │   8-State Continuous-Time Filter,      │
                  │   Velocity Estimation, Track Lifecycle │
                  └───────────────────┬────────────────────┘
                                      │ [Confirmed Tracks]
                                      ▼
                  ┌────────────────────────────────────────┐
                  │       Apple Metal 3 ImGui Display      │
                  │   4 Viewports, Telemetry HUD, Reticle  │
                  └────────────────────────────────────────┘
```

---

## Subsystems & Mathematical Models

### 1. Core Physics & Radiometry Engine (`ir_sim_physics`)
- **Planck Blackbody Exitance**:
  Simpson's 3/8 adaptive quadrature with pre-integrated sub-nanosecond look-up tables (`RadianceLUT`), verified to Stefan-Boltzmann law $M = \sigma T^4$ within $0.05\%$.
- **MODTRAN Atmospheric Transfer**:
  $\tau(R) = e^{-\beta_{\text{ext}} R}$, $L_{\text{path}}(R) = (1 - \tau(R)) \cdot L_{\text{bb}}(T_{\text{ambient}})$.
  Incorporates atmospheric temperature lapse rate ($-6.5\text{ K/km}$) and weather conditions (*Clear Summer*, *Winter*, *Haze*, *Fog*, *Rain*).
- **Diurnal Solar Heating**:
  Calculated across 12 materials (*Armor steel*, *CARC paint*, *Exhaust metal*, *Asphalt*, *Soil*, *Vegetation*, *Water*, etc.).

### 2. Platform Kinematics & Geo-Projection (`ir_sim_platform`)
- **6-DOF Airframe Dynamics**:
  4th-order Runge-Kutta (RK4) numerical integration at 100 Hz modeling gravity, lift, parasitic drag, and wind gust forces.
- **Stabilized Gimbal & Structural Jitter**:
  2-axis/3-axis gimbal with Nadir, Fixed Look, and continuous GeoLock coordinate tracking. Injects realistic multi-harmonic motor vibration jitter (50, 120, 200 Hz).
  Synthesizes orthonormal camera basis avoiding Euler angle gimbal lock at Nadir.
- **Pinhole Camera & WGS84 Projection**:
  $\text{IFOV} = 0.24\text{ mrad}$, $\text{GSD} = 12\text{ cm/px}$ at $500\text{ m}$ altitude. Implements 3D-to-pixel projection, ground raycasting, and WGS84 GPS geodetic translation.

### 3. FPA Sensor Degradation & Onboard ISP (`ir_sim_sensor`)
- **Microbolometer Non-Idealities**:
  - Spatial Fixed Pattern Noise (FPN) with gain variance ($\sigma_g \sim 2.5\%$) and offset variance ($\sigma_o \sim 150$ counts).
  - Correlated column-to-column read-out ADC striping ($\sigma_{\text{col}} \sim 45$ counts).
  - NETD temporal Gaussian noise ($40\text{ mK}$).
  - Random hot/dead defective pixel masking ($0.1\%$).
  - 14-bit ADC quantization ($0 - 16,383$ digital counts).
- **Real-Time Embedded ISP Pipeline**:
  - **2-Point Non-Uniformity Correction (NUC)**: Achieves $>95\%$ FPN reduction.
  - **Bad Pixel Replacement (BPR)**: 8-neighbor median filtering on defective pixels.
  - **Plateau Histogram AGC / CLAHE**: Non-linear dynamic range mapping of 14-bit thermal dynamic range to 8-bit visual display with plateau threshold clipping to prevent noise inflation in uniform scenes.

### 4. Tactical Detection & Multi-Target Tracking (`ir_sim_detection`)
- **2D Cell-Averaging CFAR (CA-CFAR)**:
  Dual integral images for $O(1)$ computation of local clutter mean ($\mu$) and variance ($\sigma^2$) across non-stationary background gradients:
  $$\text{Threshold} = \mu + \alpha \cdot \sigma, \quad \text{SCR} = \frac{\text{CUT} - \mu}{\sigma}$$
- **Multiscale Local Contrast Measure (LCM)**:
  Patch-based saliency measure isolating dim, sub-pixel targets against complex terrain clutter:
  $$\text{LCM}(x, y) = \min_{i=1..8} \left( \frac{m_0^2}{m_i} \right)$$
- **Aerospace Kalman Multi-Target Tracker (ByteTrack / SORT)**:
  Continuous-time 8-state kinematic state vector $[x, y, w, h, \dot{x}, \dot{y}, \dot{w}, \dot{h}]^T$ with Hungarian/Greedy association, state lifecycle machine (`Tentative` $\to$ `Confirmed` $\to$ `Lost`), and velocity prediction across dropouts.

---

## Building & Running

### Requirements
- **macOS** with Apple Silicon (M1/M2/M3/M4)
- **Xcode Command Line Tools** (`clang++` supporting C++20)
- **CMake** $\ge 3.22$
- **GLFW 3** (`brew install glfw`)

### Compilation
```bash
cd /Users/fherman/Documents/Infrared-Imaging-Sim

# Configure for optimized Release build
cmake -B build -DCMAKE_BUILD_TYPE=Release

# Build all libraries, CLI utilities, and tests
cmake --build build -j8
```

### Running the Test Suite
```bash
./build/ir_sim_tests
```
*Output: 36 test suites executed with 0 failures.*

### Scenario Generator (`target_gen`)
Create customized tactical scenarios with dynamic targets:
```bash
./build/target_gen --output scenarios/my_scenario.json --type convoy --targets 4 --duration 60.0
```

### Headless Benchmark & ROC Evaluation (`ir_sim_cli`)
Run full-frame batch simulations and export per-frame telemetry to CSV:
```bash
# Micro-Drone Incursion Scenario
./build/ir_sim_cli --scenario scenarios/micro_drone_incursion.json --duration 4.0 --fps 30

# Woodland Bird Swarm Scenario
./build/ir_sim_cli --scenario scenarios/woodland_bird_swarm.json --duration 4.0 --fps 30

# Desert Convoy with Telemetry Export
./build/ir_sim_cli --scenario scenarios/desert_convoy_day.json --duration 4.0 --fps 30 --csv convoy_telemetry.csv
```

### Interactive Dear ImGui + Apple Metal 3 GUI (`ir_drone_sim`)
Launch the interactive graphical simulation:
```bash
./build/ir_drone_sim --scenario scenarios/desert_convoy_day.json
```

#### Interactive GUI Features:
1. **4 Synchronized Viewports**:
   - **Viewport 1: Raw 14-bit FPA** (Displays sensor artifacts: FPN, column striping, dead pixels).
   - **Viewport 2: Calibrated NUC** (Cleaned sensor output after 2-point calibration & BPR).
   - **Viewport 3: Enhanced Visual AGC** (Plateau AGC 8-bit rendering with HUD overlays, target reticles, and Kalman track vectors).
   - **Viewport 4: CFAR / SCR Map** (Signal-to-Clutter Ratio heat map showing detection thresholds).
2. **Interactive Flight Controls**:
   - Switch between **Orbit**, **Waypoint**, **Manual Teleoperation**, and **Target Pursuit** flight modes.
   - Real-time manual pitch/roll/yaw/throttle sliders.
3. **Gimbal & Optics Controls**:
   - Toggle **Nadir**, **GeoLock**, and **Manual Pan/Tilt**.
   - Structural vibration jitter toggle & amplitude slider.
4. **Onboard ISP & Detection Tuners**:
   - Run live **Two-Point NUC Calibration** with custom cold/hot temperatures.
   - Toggle Bad Pixel Replacement and Multiscale LCM filter.
   - Adjust CFAR Guard, Training window size, and $\alpha$ threshold factor.

---

## Benchmark Results (Apple Silicon M4)

| Metric | Micro-Drone Incursion | Woodland Bird Swarm | Desert Convoy |
| :--- | :---: | :---: | :---: |
| **Target Scale** | Sub-pixel ($1\times1 - 3\times3$ px) | Sub-pixel ($1\times1 - 5\times5$ px) | Extended ($25\times30$ px) |
| **Probability of Detection ($P_d$)** | **100.0%** (120/120) | **95.4%** (104/109) | **63.3%** (88/139) |
| **False Alarm Rate / Frame** | **0.20** | **1.12** | **0.27** |
| **Active Confirmed Tracks** | 1 | Coated / Multiple | Coated |
| **Scene Transfer Latency** | 2.57 ms | 2.59 ms | 2.62 ms |
| **FPA Sensor Latency** | 1.76 ms | 1.77 ms | 1.76 ms |
| **Embedded ISP Latency** | 0.39 ms | 0.40 ms | 0.39 ms |
| **CFAR / LCM Latency** | 1.09 ms | 4.02 ms | 6.35 ms |
| **Total Pipeline Latency** | **5.80 ms** | **8.79 ms** | **11.13 ms** |
| **Effective Throughput** | **172.3 FPS** | **113.8 FPS** | **89.8 FPS** |
