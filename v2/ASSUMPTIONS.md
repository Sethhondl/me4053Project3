# Assumptions Documentation - Version 2

This document provides comprehensive documentation of all assumptions made in the flywheel energy storage system analysis. Each assumption includes justification, impact assessment, and what information would be needed to remove the assumption.

---

## Table of Contents

1. [Geometric Assumptions](#1-geometric-assumptions)
2. [Mass and Inertia Assumptions](#2-mass-and-inertia-assumptions)
3. [Thermal Model Assumptions](#3-thermal-model-assumptions)
4. [Motor/Generator Assumptions](#4-motorgenerator-assumptions)
5. [AMB System Assumptions](#5-amb-system-assumptions)
6. [Controller Design Assumptions](#6-controller-design-assumptions)
7. [Balance and Runout Assumptions](#7-balance-and-runout-assumptions)
8. [Energy and Efficiency Assumptions](#8-energy-and-efficiency-assumptions)

---

## 1. Geometric Assumptions

### 1.1 Total Shaft Length

**Assumption:** Total shaft length = 1.5 m

**Justification:**
- Flywheel section: 1.0 m (specified)
- Motor section: 0.25 m (specified)
- Two AMB sections: ~0.1 m each
- Clearances and end supports: ~0.05 m total

**Impact:** Medium - Affects shaft mass (11% of total) and moment of inertia calculations.

**To Remove:** Provide complete assembly drawing with dimensioned shaft length.

---

### 1.2 Housing Radial Clearance

**Assumption:** 20 mm radial clearance between flywheel outer diameter and vacuum housing inner surface.

**Justification:** Typical for high-speed rotors operating in vacuum. Provides:
- Sufficient gap for thermal expansion
- Clearance for rotor runout
- Manufacturing tolerance accommodation

**Impact:** Low - Only affects radiation view factor calculation (F_rad = 2.6).

**To Remove:** Specify vacuum housing geometry.

---

### 1.3 AMB Axial Separation

**Assumption:** L_amb = flywheel_length + 0.3 m ≈ 1.3 m

**Justification:** AMBs positioned near flywheel ends with some overhang for motor and sensor placement.

**Impact:** Medium - Affects tilting mode stiffness calculations.

**To Remove:** Provide AMB axial positions from center of mass.

---

## 2. Mass and Inertia Assumptions

### 2.1 AMB Rotor Component Mass

**Assumption:** AMB rotor components = 10% of shaft mass

**Justification:** AMB rotor includes:
- Laminated iron target rings
- Position sensor targets
- Mounting hardware

Industry rule-of-thumb: AMB rotor mass typically 8-15% of shaft mass for high-speed applications.

**Impact:** Low - AMB rotors contribute ~1% of total mass.

**To Remove:** Provide AMB rotor component specifications or masses.

---

### 2.2 Moment of Inertia Calculations

**Assumption:** Components modeled as ideal cylinders:
- Flywheel: Hollow cylinder with uniform density
- Shaft: Solid cylinder
- Magnets: Hollow cylinder on motor section
- No consideration of keyways, grooves, or fillets

**Justification:** First-order approximation sufficient for energy storage and dynamics analysis.

**Impact:** Low - Error typically < 5% for well-designed rotors.

**To Remove:** Provide CAD model or detailed component drawings.

---

### 2.3 Transverse Moment of Inertia

**Assumption:** Calculated using parallel axis theorem for multi-component rotor:
```
I_transverse = Σ (I_cm_i + m_i * d_i^2)
```
Where d_i is each component's distance from the system center of mass.

**Justification:** Proper accounting for COM offsets gives more accurate transverse inertia than simple cylinder formula.

**Impact:** Medium - Affects tilting mode natural frequency and controller design.

**To Remove:** Provide measured or FEA-computed transverse inertia.

---

### 2.4 Center of Mass and AMB Distances (4-DOF Geometry)

**Assumption:** COM position and distances to each AMB are calculated from component geometry:
- z_com = (Σ m_i × z_i) / m_total
- d_amb_top = z_amb_top - z_com
- d_amb_bottom = z_com - z_amb_bottom

**Justification:** Proper 4-DOF model requires actual lever arm distances for tilting dynamics, not symmetric approximation (L_amb/2).

**Impact:** Medium - Affects tilting stiffness calculations and runout analysis.

**To Remove:** Provide CAD-based COM analysis.

---

## 3. Thermal Model Assumptions

### 3.1 Radiation-Only Heat Transfer

**Assumption:** Heat transfer from rotor to housing occurs only via radiation (no convection).

**Justification:** Flywheel operates in vacuum to eliminate windage losses. In vacuum:
- No convective heat transfer
- Conduction only through small contact points (bearings)
- Radiation is the dominant heat transfer mechanism

**Impact:** High - Directly determines rotor temperature at given power loss.

**To Remove:** N/A - This is a physical requirement of the vacuum environment.

---

### 3.2 Only Rotor Losses Heat the Rotor

**Assumption:** Stator losses do not contribute to rotor heating.

**Justification:**
- Stator is stationary and located outside the vacuum chamber
- Stator has its own cooling system (typically water or forced air)
- No thermal coupling between stator and rotor in vacuum

**Impact:** High - Including stator losses would approximately double the thermal load.

**To Remove:** N/A - This is a physical design characteristic.

---

### 3.3 Two-Surface Enclosure Radiation Model

**Assumption:** Radiation heat transfer uses two-surface enclosure formula:
```
Q = σ * A_rotor * (T_rotor^4 - T_housing^4) / F_rad
F_rad = 1/ε_rotor + (A_rotor/A_housing) * (1/ε_housing - 1)
```

**Justification:** Standard radiation analysis for concentric cylinders. With ε_rotor = 0.4 and ε_housing = 0.9:
- F_rad ≈ 2.6 (high radiation resistance due to low rotor emissivity)
- Max sustainable rotor loss ≈ 391 W for T_rotor ≤ 100°C

**Impact:** High - F_rad determines steady-state temperature.

**To Remove:** Provide detailed radiation analysis with view factors.

---

### 3.4 Housing Temperature

**Assumption:** Housing temperature = 30°C (constant)

**Justification:** From Appendix B specification. Assumes active cooling of vacuum housing maintains constant temperature.

**Impact:** Medium - 10°C change in housing temp changes rotor temp by similar amount.

**To Remove:** Specify housing thermal management system performance.

---

### 3.5 Material Emissivities

**Assumption:**
- Rotor emissivity ε = 0.4 (from Table 1)
- Housing emissivity ε = 0.9 (from Table 1)

**Justification:** Per project specification. Low rotor emissivity (0.4) is typical for:
- Polished metal surfaces
- Composite surfaces with metallic coating

**Impact:** High - Rotor emissivity is critical for thermal analysis.

**To Remove:** Measure actual surface emissivities.

---

## 4. Motor/Generator Assumptions

### 4.1 Rated Current Definition

**Assumption:** Rated current = 1.0 pu (per-unit), representing maximum available current.

**Justification:** The EE functions use per-unit current (0-1.0 range). At 1.0 pu:
- Maximum magnetic shear stress
- Maximum torque capability
- Maximum motor losses

**Impact:** High - Determines rated power, losses, and thermal limits.

**To Remove:** Provide explicit rated current or power specification.

---

### 4.2 Current Scaling for Constant Power

**Assumption:** To maintain constant power at varying speeds:
```
I_stator(ω) = I_rated * (ω_max / ω)
```
Limited to I_max = 1.0 pu.

**Justification:** From power equation P = T * ω, and torque proportional to current:
- At low speed (high SoC): current at maximum
- At high speed (low SoC): current reduced proportionally

**Impact:** Medium - Affects loss calculations across SoC range.

**To Remove:** Provide motor control strategy and current limits.

---

### 4.3 Magnetic Shear Stress

**Assumption:** Magnetic shear stress from magneticShear() function is uniform over motor surface.

**Justification:** First-order approximation for permanent magnet motor. Actual distribution depends on:
- Pole pitch and number of poles
- Winding configuration
- Rotor saliency

**Impact:** Low - Integrated torque is primary concern, not local distribution.

**To Remove:** Provide detailed motor electromagnetic model.

---

## 5. AMB System Assumptions

### 5.1 Negative Stiffness Convention

**Assumption:** AMB position stiffness K_s is negative (destabilizing).

**Justification:** AMBs inherently have negative stiffness:
- Moving rotor toward magnet increases attractive force
- This creates unstable equilibrium requiring active control
- ambParameters() returns stiffnessConstant as positive magnitude

**Usage:** K_s = -stiffnessConstant (use negative sign)

**Impact:** High - Sign error would cause incorrect stability analysis.

**To Remove:** N/A - This is fundamental AMB physics.

---

### 5.2 Rigid Rotor Assumption

**Assumption:** Rotor is rigid with no bending flexibility.

**Justification:** For a well-designed flywheel system:
- First bending mode should be well above operating speed range
- Bending stiffness dominated by shaft diameter
- Composite flywheel adds stiffness

**Impact:** Medium - Valid if first bending mode > 667 Hz (40,000 RPM).

**To Remove:** Provide rotor FEA showing bending mode frequencies.

---

### 5.3 Two-AMB Stiffness Factor

**Assumption:** Dynamic stiffness uses 2×K_s to account for both AMBs:
```
K_radial_total = 2 × K_s (two bearings in parallel)
K_tilt = (d_top² + d_bot²) × K_s (lever arm effects)
```

**Justification:** Both AMBs contribute to the total radial stiffness of the system. For tilting, each AMB's contribution is weighted by the square of its distance from COM.

**Impact:** High - Significantly affects dynamic stiffness magnitude at low frequencies.

**To Remove:** N/A - This is correct AMB physics.

---

### 5.4 Decoupled Axes

**Assumption:** Radial x-y axes and tilting modes are analyzed independently (no gyroscopic coupling).

**Justification:**
- Gyroscopic effects are secondary for step response at 0 RPM
- Full gyroscopic analysis would require 4-DOF coupled model
- Project focuses on baseline characterization

**Impact:** Low for 0 RPM analysis, Medium for high-speed dynamics.

**To Remove:** Implement full 4-DOF rotor dynamics model.

---

### 5.5 AMB Force Linearity

**Assumption:** AMB force is linear with current: F = K_i * i

**Justification:** Valid for:
- Operation within rated force range
- Bias linearization scheme
- Small perturbations from equilibrium

**Impact:** Medium - Nonlinearity affects large disturbance response.

**To Remove:** Include magnetic saturation model.

---

## 6. Controller Design Assumptions

### 6.1 Baseline Controller from Appendix B

**Assumption:** Use controller parameters exactly as specified in Appendix B:

| Parameter | Radial (x) | Tilting (α) |
|-----------|------------|-------------|
| kp | 1.2639e8 | 7.6992e7 |
| ki | 1.16868e9 | 1.18953e9 |
| kd | 252790 | 80294 |
| ω_p | 3770 rad/s | 6283 rad/s |

**Justification:** These are the specified baseline controller gains.

**Impact:** High - Controller defines stability margins and performance.

**To Remove:** N/A - These are specifications.

---

### 6.2 Current Loop Approximation

**Assumption:** For position loop analysis, current loop is assumed fast: G_ci ≈ 1

**Justification:**
- Current controller has ~1.5 kHz bandwidth
- Position controller has ~100 Hz bandwidth
- Separation of ~10x allows decoupled analysis

**Impact:** Low - Only affects detailed time-domain response.

**To Remove:** Include full cascaded loop model.

---

### 6.3 New Controller Design (Loop-Shaping Methodology)

**Assumption:** New system controllers designed using frequency-domain loop-shaping:

**Controller structure:**
```
G_c(s) = K_o × (1 + s/w_z1) × (1 + s/w_z2) / [s × (1 + s/w_p)]
```
Equivalent to PID: `G_c(s) = k_p + k_i/s + k_d×s/(1+s/w_p)`

**Design rules:**
1. **Crossover frequency:** f_c ≥ 5× unstable pole frequency (minimum 100 Hz)
2. **Target phase margin:** 60 degrees
3. **Zero placement:**
   - w_z1 = w_c / 100 (far below crossover for robustness)
   - w_z2 = solved to achieve target phase margin
4. **Derivative filter:** w_p = 4 × w_c (above crossover)
5. **Gain K_o:** Set for unity loop gain at crossover

**PID conversion:**
- k_i = K_o
- k_p = K_o × (1/w_z1 + 1/w_z2)
- k_d = K_o / (w_z1 × w_z2)

**Tilting controller:** Scaled from baseline maintaining stiffness margin (k_p/|K_s|).

**Justification:**
- Loop-shaping provides explicit phase margin control
- Crossover >> unstable pole ensures robust stability
- Stiffness margin scaling maintains relative performance

**Impact:** High - Determines new system stability and performance.

**To Remove:** Provide detailed controller design specifications.

---

## 7. Balance and Runout Assumptions

### 7.1 ISO G2.5 Balance Grade

**Assumption:** Rotor is balanced to ISO G2.5 quality.

**Justification:** From Table 1 in project specification. G2.5 means:
```
e × ω = 2.5 mm/s
```
Where e = eccentricity and ω = angular velocity (rad/s).

This is typical for:
- Machine tool spindles
- Medium-precision turbomachinery
- High-speed electric motors

**Impact:** High - Directly determines unbalance force and runout.

**To Remove:** Provide actual balance test data.

---

### 7.2 Synchronous Excitation

**Assumption:** Runout calculated for synchronous (1X) excitation only.

**Justification:** Mass imbalance creates force at rotation frequency:
```
F_unbalance = m × e × ω²
```
Other sources (harmonics, electromagnetic) not considered.

**Impact:** Medium - Higher harmonics may contribute to total runout.

**To Remove:** Include complete excitation spectrum analysis.

---

### 7.3 Eccentricity Calculation

**Assumption:** Eccentricity inversely proportional to speed for constant balance grade:
```
e = G / ω = (2.5 × 10⁻³) / ω [m]
```

**Justification:** This is the definition of ISO balance grade. At higher speeds, less eccentricity is permitted.

**Impact:** High - Determines unbalance force magnitude.

**To Remove:** N/A - This is the ISO standard definition.

---

## 8. Energy and Efficiency Assumptions

### 8.1 State of Charge Definition

**Assumption:**
- 0% SoC = 50% of max speed (20,000 RPM) = 25% of max kinetic energy
- 100% SoC = 100% of max speed (40,000 RPM) = 100% of max kinetic energy
- Energy-based relationship (SoC ∝ stored energy ∝ ω²):
  - Forward: ω = √(ω_min² + (ω_max² - ω_min²) × SoC/100)
  - Reverse: SoC = 100 × (ω² - ω_min²) / (ω_max² - ω_min²)

**Justification:** SoC represents "State of Charge" = fraction of usable stored energy.
Since E = ½Iω², the energy is quadratic with speed:
- At 0% SoC (ω_min): E_min = 0.25 × E_max
- At 100% SoC (ω_max): E_max
- Usable energy = E_max - E_min = 0.75 × E_max
- At 50% SoC: ω ≈ 34,641 RPM (not 30,000 RPM)

**Impact:** High - Defines usable energy capacity and speed-SoC mapping.

**To Remove:** N/A - This is the correct physics-based definition.

---

### 8.2 Efficiency Calculation Method

**Assumption:** Round-trip efficiency calculated as:
```
η = E_discharged / (E_charged + E_recovery)
```
Where E_recovery = energy needed to restore initial SoC.

**Justification:** This accounts for net energy input/output and SoC change over the cycle.

**Impact:** Medium - Different definitions may give slightly different values.

**To Remove:** Provide explicit efficiency calculation definition.

---

### 8.3 Simulation Time Step

**Assumption:**
- Part 1c (15-min cycle): dt = 1.0 s
- Part 2b (6-hour cycle): dt = 10.0 s

**Justification:** Time steps chosen to capture cycle dynamics while maintaining reasonable computation time. Energy balance verified by conservation checks.

**Impact:** Low - Smaller time steps give same results.

**To Remove:** Perform convergence study.

---

### 8.4 Speed Limits During Cycle

**Assumption:** Speed constrained to ±10% of nominal range:
```
0.9 × ω_min ≤ ω ≤ 1.05 × ω_max
```

**Justification:** Prevents simulation instability if cycle demands exceed storage capacity.

**Impact:** Low - Well-designed cycles stay within limits.

**To Remove:** Implement proper saturation handling.

---

## Summary Table

| Category | Assumption | Impact | Confidence |
|----------|------------|--------|------------|
| **Geometry** | Shaft length 1.5 m | Medium | High |
| | Housing clearance 20 mm | Low | High |
| | AMB separation 1.3 m | Medium | Medium |
| **Mass** | AMB rotors 10% of shaft | Low | Medium |
| | Cylindrical components | Low | High |
| **Thermal** | Radiation only | High | High |
| | Only rotor losses heat rotor | High | High |
| | F_rad = 2.6 | High | High |
| | Housing at 30°C | Medium | High |
| **Motor** | Rated current 1.0 pu | High | Medium |
| | Current scaling I ∝ ω_max/ω | Medium | High |
| **AMB** | K_s negative | High | High |
| | Rigid rotor | Medium | Medium |
| | Decoupled axes | Medium | High |
| **Control** | Appendix B parameters | High | High |
| | HW3 design methodology | High | High |
| **Balance** | ISO G2.5 | High | High |
| | Synchronous excitation | Medium | High |
| **Efficiency** | SoC definition | High | High |
| | η = E_out / (E_in + E_rec) | Medium | High |

---

## References

1. Project Specification: "Project 3: Flywheel Energy Storage System"
2. Appendix A: Material Properties and Limits
3. Appendix B: Baseline System Parameters and Controller Gains
4. Table 1: System Specifications
5. Homework 3: AMB Controller Design Methodology

---

*Last updated: December 3, 2025*

---

## Changes from v1

1. **4-DOF Geometry:** Added proper COM calculation and individual AMB distances (Section 2.4)
2. **2×Ks Stiffness:** Added two-AMB stiffness factor for dynamic stiffness (Section 5.3)
3. **Loop-Shaping Controller:** Replaced HW3 methodology with frequency-domain loop-shaping (Section 6.3)
