# Project Instructions for Claude

## Project Overview

**Course:** Mechanical Engineering Modeling
**Project:** Flywheel Energy Storage System Analysis for eXtreme Storage Inc.

This project analyzes and optimizes a flywheel energy storage system for grid frequency regulation. The analysis is divided into three deliverables:
- **Deliverable 1:** Baseline system characterization (COMPLETE)
- **Deliverable 2:** New design development
- **Deliverable 3:** Final design optimization and comparison

---

## EE Functions - CRITICAL REQUIREMENT

**Always use the official `.p` files from `Project3_Functions/`, NOT the approximations in `ee_functions/`.**

The `ee_functions/` directory contains physics-based approximations created for reference/understanding purposes only. These may not match the actual behavior of the protected functions.

### Correct Usage

```matlab
% Add the official functions to path
addpath('../Project3_Functions');

% Use the .p files directly
shear = magneticShear(magnetThickness, statorCurrent);
losses = rotorLosses(magnetThickness, rotorDiameter, axialLength, statorCurrent, rotorSpeed);
losses = statorLosses(magnetThickness, rotorDiameter, axialLength, statorCurrent, rotorSpeed);
params = ambParameters(rotorDiameter, forceRating);
power = baselineStorageCycle(time);         % 15-min baseline cycle (0-900 s)
power = team_16_cycle(time);                % Team 16's 6-hour cycle (0-21600 s)
power = genericStorageCycle(time);
params = elecMachineParams(...);
```

### Function Input/Output Notes

- **IMPORTANT:** EE functions use **per-unit current (0-1.0 range)**, NOT absolute amperes
- `rotorSpeed` is in **r/min (RPM)**, not rad/s
- `ambParameters()` returns a struct with fields:
  - `.stiffnessConstant` - Position stiffness magnitude (use as negative for Ks)
  - `.forceConstant` - Current-to-force constant Ki [N/A]
  - `.coilInductance` - Coil inductance [H]
  - `.coilResistance` - Coil resistance [Ohms]

### Do NOT Use

- `ee_functions/magneticShear.m`
- `ee_functions/rotorLosses.m`
- `ee_functions/statorLosses.m`
- `ee_functions/ambParameters.m`
- `ee_functions/baselineStorageCycle.m`

---

## Baseline System Specifications (Appendix B - Table A.1)

### Geometric Parameters
| Parameter | Value | Units |
|-----------|-------|-------|
| Flywheel length | 1.000 | m |
| Flywheel diameter | 0.430 | m |
| Motor axial length | 0.250 | m |
| Shaft/PM diameter | 0.084 | m |
| Magnet thickness | 0.006 | m |

### Operational Parameters
| Parameter | Value | Units |
|-----------|-------|-------|
| Max speed | 40,000 | RPM |
| Min speed (0% SoC) | 20,000 | RPM |
| AMB rated force | 5,780 | N |
| Max safe temperature | 100 | °C |

### Material Properties
| Material | Density [kg/m³] |
|----------|-----------------|
| Composite (flywheel) | 1600 |
| Steel (shaft) | 7850 |
| Permanent magnets | 7850 |

### Material Limits (Tip Speed)
| Material | Max Tip Speed [m/s] |
|----------|---------------------|
| Composite | 900 |
| Steel | 175 |
| Permanent magnets | 175 |

### Current Controller (from Appendix B)
```
G_ci(s) = 345 + 2149/s  (PI controller)
Kp_current = 345
Ki_current = 2149
```

### Position Controller - Radial x (from Appendix B)
```
G_ocx(s) = k_px + k_ix/s + s*k_dx/(1+s/omega_px)
k_px = 1.2639e8
k_ix = 1.16868e9
k_dx = 252790
omega_px = 3770 rad/s
```

### Position Controller - Tilting (from Appendix B)
```
G_dcx(s) = k_palpha + k_ialpha/s + s*k_dalpha/(1+s/omega_palpha)
k_palpha = 7.6992e7
k_ialpha = 1.18953e9
k_dalpha = 80294
omega_palpha = 6283 rad/s
```

---

## State of Charge Definition

- **0% SoC** = 50% of max speed = 20,000 RPM
- **100% SoC** = 100% of max speed = 40,000 RPM
- Linear relationship: `omega = omega_min + (omega_max - omega_min) * SoC/100`

---

## Thermal Model

- Flywheel operates in **vacuum** (no convection)
- Heat transfer via **radiation only** (two-surface enclosure model)
- **Only rotor losses** heat the rotor (stator is outside vacuum, cooled separately)
- Rotor emissivity: **0.4** (from Table 1) - LOW emissivity limits heat dissipation!
- Housing emissivity: **0.9** (from Table 1)
- Housing temperature: **30°C (303 K)** (from Appendix B)
- Two-surface radiation formula:
  ```
  Q = σ × A_rotor × (T_rotor⁴ - T_housing⁴) / F_rad
  F_rad = 1/ε_rotor + (A_rotor/A_housing) × (1/ε_housing - 1)
  ```
- With ε_rotor = 0.4: **F_rad ≈ 2.6** (high radiation resistance)
- Max rotor loss for T ≤ 100°C: **~391 W**

---

## Key Assumptions and Derived Values

1. **Rated current**: I_rated_pu = **1.0** (maximum available from EE functions)
2. Total shaft length ≈ 1.5 m
3. AMB rotor components ≈ 10% of shaft mass
4. Position controllers: use values from Appendix B (PID with derivative filter)
5. AMB separation distance ≈ 1.3 m
6. ISO G2.5 balance grade for mass imbalance (from Table 1)
7. Rigid rotor assumption (no bending modes)

---

## File Structure

```
Project3/
├── CLAUDE.md                    # This file - project instructions
├── README.md                    # Project overview
├── deliverable_1/               # Deliverable 1: Baseline Analysis
│   ├── baseline_analysis.m      # Main analysis script
│   ├── README.md                # Deliverable documentation
│   ├── ASSUMPTIONS.md           # Detailed assumptions
│   └── *.fig/*.png              # Generated plots
├── Project3_Functions/          # OFFICIAL EE Functions (P-code) - USE THESE
│   ├── magneticShear.p
│   ├── rotorLosses.p
│   ├── statorLosses.p
│   ├── ambParameters.p
│   ├── baselineStorageCycle.p    # 15-min baseline cycle (Deliverable 1)
│   ├── genericStorageCycle.p
│   ├── team_16_cycle.p           # Team 16's 6-hour storage cycle (Deliverable 2)
│   └── elecMachineParams.p
├── ee_functions/                # DO NOT USE - approximations only
└── Project03_*.pdf              # Project documentation
```

---

## Deliverable Requirements Summary

### Deliverable 1 Parts
- **1a:** Losses and temperature vs SoC at rated power
- **1b:** Specific power [kW/kg] and specific energy [Wh/kg]
- **1c:** 15-minute storage cycle efficiency (start at 50% SoC)
- **1d:** AMB step response (10% rated force disturbance at top AMB, **zero speed only**)
- **1e:** Dynamic stiffness vs frequency (1-1000 Hz, radial and tilting)
- **1f:** Rotor runout from mass imbalance vs SoC

---

## Calculated Baseline Results (Deliverable 1)

| Metric | Value | Notes |
|--------|-------|-------|
| Total rotating mass | 298.6 kg | Flywheel + shaft + magnets |
| Moment of inertia | 5.43 kg·m² | About spin axis |
| **Rated current** | **1.0 pu** | Maximum available |
| Specific energy | 33.2 Wh/kg | |
| Total energy storage | 9.92 kWh | Between 0-100% SoC |
| Rotor runout | 0.004-0.005 µm | ISO G2.5 balance |

*Note: Rated power, specific power, losses, temperature, and efficiency are calculated by running the scripts with the .p functions.*

---

## Common Calculations

### Angular Velocity Conversion
```matlab
omega_rad = rpm * 2*pi / 60;     % RPM to rad/s
rpm = omega_rad * 60 / (2*pi);   % rad/s to RPM
```

### Kinetic Energy
```matlab
E = 0.5 * I_total * omega^2;     % [J]
E_Wh = E / 3600;                 % [Wh]
```

### Torque from Magnetic Shear
```matlab
shear = magneticShear(magnet_thickness, I_pu);
motor_area = pi * shaft_diameter * motor_length;
torque = shear * motor_area * (shaft_diameter/2);
power = torque * omega;
```

### Rated Current
```matlab
% Use maximum available current (1.0 pu)
I_rated_pu = 1.0;
```
