# Assumptions Documentation for Deliverable 1

This document details all assumptions made during the baseline flywheel system analysis, explains why each assumption was necessary, and identifies what additional information would eliminate the need for each assumption.

---

## 1. Rated Power Definition

### Assumption Made
Rated power is calculated from the maximum motor capability at maximum speed:
- Power = Torque × Angular Velocity
- Torque = Magnetic Shear Stress × Motor Surface Area × Motor Radius
- Used an iterative approach to find stator current that produces the rated power
- Calculated rated power ≈ 100-150 kW (exact value depends on magnetic shear characteristics)

### Why This Assumption Was Needed
The project specification mentions "rated power" in Part 1a but does not explicitly define what value or criterion determines the rated power of the baseline system. Without a specified rated power value, we cannot calculate losses at the specified operating condition.

### Information Needed to Remove Assumption
- **Explicit rated power specification** (e.g., "The baseline system has a rated power of 100 kW")
- **OR** a definition of rated power in terms of:
  - Maximum continuous power output
  - Power at rated motor current
  - Power limited by thermal constraints
  - Power corresponding to a specific torque rating

---

## 2. Shaft Total Length

### Assumption Made
Total shaft length assumed to be 1.5 m, which includes:
- Flywheel section: 1.0 m
- Motor section: 0.25 m
- Two AMB sections: ~0.1 m each (estimated)
- Clearance sections between components: multiple 0.02 m sections

### Why This Assumption Was Needed
The project provides individual component axial lengths but not the total shaft length. The shaft extends beyond the flywheel and motor to accommodate AMBs, provide structural support, and maintain clearances. This total length is needed to calculate:
- Total shaft mass
- Shaft contribution to moment of inertia
- AMB axial positions (for tilting mode analysis)

### Information Needed to Remove Assumption
- **Complete shaft length specification** from end to end
- **OR** a detailed assembly drawing showing:
  - Axial positions of all components
  - AMB locations
  - End cap/bearing locations
  - All clearance gaps between components

---

## 3. AMB Rotor Component Mass

### Assumption Made
AMB rotor components (target laminations, position sensors, etc.) assumed to be 10% of the shaft mass.

### Why This Assumption Was Needed
The rotating portion of the AMB system adds mass to the rotating group but is not specified in the given parameters. This mass affects:
- Total rotating group mass (for specific power/energy calculations)
- Moment of inertia
- AMB dynamics

### Information Needed to Remove Assumption
- **AMB rotor component mass** or detailed specifications of:
  - Target lamination stack dimensions and material
  - Sensor rotor component dimensions
  - Any other AMB components that rotate with the shaft

---

## 4. Thermal Model - Radiation Heat Transfer

### Assumption Made
Temperature rise calculated using radiation-only heat transfer:
- Stefan-Boltzmann radiation: Q = ε × σ × A × (T⁴ - T_amb⁴)
- Emissivity (ε) assumed to be 0.8 (typical for composite/steel surfaces)
- Ambient temperature assumed to be 20°C (293 K)
- Radiating surface area based on flywheel outer cylinder geometry
- **Only rotor losses** used for rotor temperature (stator is outside vacuum, cools separately)

### Why This Assumption Was Needed
The flywheel rotor operates in vacuum (for reduced windage losses), so convective heat transfer is negligible. The stator is stationary and outside the vacuum chamber, so stator losses are dissipated through the stator's own cooling system and do not directly heat the rotor. However, the project does not provide:
- Thermal properties of materials (thermal conductivity, specific heat)
- Vacuum chamber wall temperature
- Surface emissivity values
- Thermal resistance network between rotor and environment

Without this information, we cannot build a detailed thermal model and must rely on simplified radiation heat transfer.

### Information Needed to Remove Assumption
- **Material thermal properties**:
  - Thermal conductivity of composite flywheel
  - Thermal conductivity of steel shaft
  - Specific heat capacity of all materials
- **Vacuum chamber specifications**:
  - Chamber wall temperature or cooling system details
  - View factors between rotor and chamber walls
- **Surface emissivity values** for:
  - Composite flywheel surface finish
  - Steel shaft surface finish
- **Thermal contact resistances** between components
- **Heat transfer path details** from rotor to heat sink

---

## 5. Position Controller Design for AMB

### Assumption Made
A PD (Proportional-Derivative) position controller was designed with gains:
- Proportional gain (Kp) selected for target natural frequency of 50 Hz
- Derivative gain (Kd) selected for damping ratio of 0.7
- Controller designed using pole placement for stable, well-damped response

Design equations used:
```
Kp = ω_n² × m / K_i
Kd = 2 × ζ × ω_n × m / K_i
```

Where:
- ω_n = 2π × 50 rad/s (target natural frequency)
- ζ = 0.7 (target damping ratio)
- m = rotor mass from ambParameters
- K_i = current stiffness from ambParameters

### Why This Assumption Was Needed
The project provides only the **current controller transfer function**:
- G_ci(s) = 258.75 + 1611.54/s (PI controller)

However, AMB analysis (Parts 1d and 1e) requires a **position controller** to close the outer position loop. Without this controller specification, we cannot simulate the AMB response or calculate dynamic stiffness.

### Information Needed to Remove Assumption
- **Position controller transfer function** G_pos(s) for the baseline system
- **OR** position controller gains (Kp, Ki, Kd) used in the original design
- **OR** performance specifications:
  - Required bandwidth
  - Settling time requirements
  - Maximum overshoot allowed
  - Stability margins (gain/phase)

---

## 6. AMB Axial Separation Distance

### Assumption Made
AMB separation distance assumed to be approximately 1.3 m for tilting stiffness calculations:
- Based on flywheel length (1.0 m) + clearances + AMB positioning
- Used to calculate effective mass for tilting mode: m_eff = I_polar / L_amb²

### Why This Assumption Was Needed
Part 1e requires calculation of dynamic stiffness in the tilting direction, which depends on two AMBs working as a couple to resist tilting moments. The moment arm (AMB separation) is needed to convert translational forces to tilting moments.

### Information Needed to Remove Assumption
- **AMB axial locations** (distances from center of mass or specific datum)
- **OR** complete assembly drawing with dimensioned AMB positions
- **OR** effective moment arm for tilting mode analysis

---

## 7. Mass Imbalance Magnitude (ISO Balance Grade)

### Assumption Made
Rotor mass imbalance characterized by ISO G2.5 balance grade:
- ISO G2.5: e × ω = 2.5 mm/s
- Where e = eccentricity of center of mass
- G2.5 is typical for precision rotors, machine tool spindles

This gives eccentricity as a function of speed:
- e = 2.5 mm/s / ω [rad/s]
- At 40,000 RPM: e ≈ 0.6 μm

### Why This Assumption Was Needed
Part 1f requires calculation of rotor runout due to mass imbalance, but the project does not specify:
- Manufacturing tolerances
- Balance quality achieved
- Residual unbalance magnitude
- Center of mass offset

Without knowing the actual imbalance, we must assume a reasonable value based on typical precision rotor standards.

### Information Needed to Remove Assumption
- **Actual balance grade** or quality specification (e.g., ISO G1.0, G2.5, G6.3)
- **Residual unbalance** specification in standard units:
  - e (eccentricity) in μm
  - U (specific unbalance) in g·mm/kg
  - Permissible residual unbalance in g·mm
- **Manufacturing tolerances** that determine maximum possible imbalance
- **Measured balance test data** from baseline system

---

## 8. Effective Damping in AMB System

### Assumption Made
System damping comes primarily from the derivative term in the position controller:
- c_eff = Kd × K_i (controller damping)
- Structural damping of rotor assumed negligible
- Eddy current damping not explicitly modeled

### Why This Assumption Was Needed
Accurate damping values are needed for:
- Unbalance response calculation (Part 1f)
- Step response analysis (Part 1d)
- Stability margins

The project does not provide damping coefficients for the mechanical system or electromagnetic damping effects.

### Information Needed to Remove Assumption
- **Material damping coefficients** for composite and steel
- **Eddy current damping** effects in AMB and motor
- **Measured damping ratios** from baseline system testing
- **Structural damping** from finite element analysis or testing

---

## 9. Motor Stator Current Distribution

### Assumption Made
Stator current assumed to be uniform around the motor circumference and adjusted to maintain constant power at different speeds:
- I_stator(ω) = I_rated × (ω_max / ω)
- This maintains constant power: P = T × ω = constant
- No consideration of current harmonics or field distribution

### Why This Assumption Was Needed
The provided functions (magneticShear, rotorLosses, statorLosses) accept a single stator current value but don't specify:
- How current varies during acceleration/deceleration
- Current distribution in multi-phase motor
- Harmonic content effects on losses

For loss calculations at various speeds, we need a relationship between speed, power, and current.

### Information Needed to Remove Assumption
- **Motor control strategy** specification:
  - Field-oriented control parameters
  - Current vector control method
  - d-axis and q-axis current relationships
- **Current limit curves** as function of speed
- **Efficiency maps** or current-torque-speed relationships
- **Detailed motor model** with phase current calculations

---

## 10. Energy Recovery Efficiency

### Assumption Made
In Part 1c efficiency calculation, energy recovery to restore initial SoC is assumed to occur at the same efficiency as normal charging:
- No separate efficiency factor for recovery
- Uses same motor/power electronics losses

### Why This Assumption Was Needed
The problem asks for efficiency accounting for "energy needed to recover self-discharge losses" but doesn't specify:
- How recovery occurs (rapid charge vs. slow charge)
- Efficiency of the recovery process
- Whether recovery is included in the cycle or happens afterward

### Information Needed to Remove Assumption
- **Recovery process specification**:
  - Timeframe for recovery (immediate vs. delayed)
  - Power level during recovery
  - Efficiency during recovery charging
- **Power electronics efficiency map** as function of power level and direction
- **Clarification** on whether recovery is part of cycle efficiency or separate

---

## 11. Rotor Structural Stiffness

### Assumption Made
For runout analysis, the rotor is assumed to be rigid (no bending flexibility), with all compliance coming from the AMB system:
- No shaft bending modes considered
- Critical speeds determined only by AMB dynamics
- Rotor treated as rigid body in AMB controller design

### Why This Assumption Was Needed
Accurate runout and critical speed prediction requires knowing the rotor's structural stiffness, but the project does not provide:
- Shaft bending stiffness
- Flywheel attachment stiffness
- Rotor natural frequencies

For high-speed rotors, bending flexibility can significantly affect dynamics.

### Information Needed to Remove Assumption
- **Finite element analysis** results showing:
  - Rotor bending mode shapes and frequencies
  - Shaft stiffness distribution
- **Measured critical speeds** from baseline system testing
- **Modal analysis data** for the rotating assembly
- **Detailed CAD model** for structural FEA

---

## 12. Constant Ambient/Environment Temperature

### Assumption Made
Ambient temperature and vacuum chamber wall temperature assumed constant at 20°C (293 K) throughout all analyses.

### Why This Assumption Was Needed
Temperature calculations require a heat sink temperature, but the project does not specify:
- Operating environment temperature range
- Vacuum chamber thermal management
- Whether chamber walls are actively cooled

### Information Needed to Remove Assumption
- **Environmental specifications**:
  - Ambient temperature range
  - Chamber wall temperature or cooling setpoint
  - Thermal interface details
- **Thermal management system** description:
  - Active cooling method (if any)
  - Cooling capacity
  - Temperature control strategy

---

## 13. Gyroscopic Effects in AMB Analysis

### Assumption Made
For Part 1d (AMB step response), gyroscopic coupling between radial axes was simplified:
- Analysis performed in single axis (x-direction)
- Cross-coupling effects approximated or neglected
- Assumption: gyroscopic effects are small for the step disturbance magnitude used

### Why This Assumption Was Needed
Full gyroscopic analysis requires:
- Polar and transverse moments of inertia
- Cross-coupled differential equations
- Multi-axis simultaneous solution

The project provides limited rotor inertia information and focuses on single-axis response.

### Information Needed to Remove Assumption
- **Complete inertia tensor**:
  - Transverse moment of inertia (I_xx, I_yy)
  - Polar moment of inertia (I_zz)
  - Products of inertia (if rotor asymmetric)
- **Specification of whether gyroscopic analysis is required**
- **Cross-coupling controller gains** if used in actual system
- **Full 3D rotor dynamics model** specification

---

## Summary Table

| # | Assumption | Impact | Information Needed |
|---|------------|--------|-------------------|
| 1 | Rated power calculation method | Determines loss and temperature calculations | Explicit rated power value or definition |
| 2 | Total shaft length (1.5 m) | Affects mass and inertia | Complete assembly drawing |
| 3 | AMB rotor mass (10% of shaft) | Affects total mass and inertia | AMB component specifications |
| 4 | Radiation-only heat transfer | Temperature prediction accuracy | Thermal properties and model details |
| 5 | PD controller design (50 Hz, ζ=0.7) | AMB response characteristics | Position controller specification |
| 6 | AMB separation (1.3 m) | Tilting stiffness calculation | AMB axial positions |
| 7 | ISO G2.5 balance grade | Runout amplitude prediction | Actual balance specification |
| 8 | Controller damping only | Dynamic response accuracy | System damping measurements |
| 9 | Uniform stator current | Loss calculations | Motor control strategy details |
| 10 | Recovery efficiency = charging | Cycle efficiency value | Recovery process specification |
| 11 | Rigid rotor | Critical speed and runout | Structural FEA or modal test data |
| 12 | Constant 20°C ambient | Thermal steady-state values | Environmental specifications |
| 13 | Simplified gyroscopic effects | Step response accuracy | Complete inertia tensor |

---

## Impact Assessment

### Critical Assumptions (High Impact)
These assumptions significantly affect the results and should be validated:
1. **Rated power definition** - Directly affects all Part 1a results
2. **Thermal model** - Determines if system meets temperature safety limits
3. **AMB controller design** - Affects stability and performance predictions
4. **Mass imbalance magnitude** - Determines runout acceptability

### Moderate Assumptions (Medium Impact)
These assumptions affect quantitative accuracy but not qualitative trends:
5. Total shaft length and mass distribution
6. AMB separation distance
7. System damping values
8. Gyroscopic coupling effects

### Minor Assumptions (Low Impact)
These assumptions have limited effect on overall results:
9. Stator current distribution details
10. Recovery efficiency specifics
11. Constant ambient temperature
12. Rotor structural stiffness (at speeds well below critical)

---

## Validation Recommendations

To improve accuracy of this baseline analysis, the following validation steps are recommended:

1. **Compare calculated rated power** with manufacturer specifications or measured data
2. **Validate temperature predictions** against thermal sensors if available
3. **Verify AMB controller** design against actual baseline controller
4. **Check mass calculations** against weighed components or CAD model
5. **Confirm balance quality** from manufacturing records
6. **Measure actual cycle efficiency** for comparison with predictions

---

This assumptions document should be updated as additional information becomes available from the eXtreme Storage Inc. team or through further analysis.
