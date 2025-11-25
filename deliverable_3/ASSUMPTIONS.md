# Assumptions Documentation for Deliverable 3

This document details all assumptions made during the AMB controller design and comparison analysis.

---

## 1. Position Controller Type

### Assumption Made
A PD (Proportional-Derivative) position controller was selected for both baseline and new designs:
- G_pos(s) = Kp + Kd*s
- Design targets: 50 Hz natural frequency, 0.7 damping ratio

### Why This Assumption Was Needed
The project provides the current controller transfer function but not the position controller. A PD controller was chosen because:
- PD provides necessary damping for the inherently unstable AMB system
- Derivative action counters the negative stiffness
- This is standard practice for AMB control

### Information Needed to Remove Assumption
- **Actual position controller specification** from baseline system
- **Performance requirements** (bandwidth, settling time, overshoot)
- **Preferred controller architecture** (PID, state feedback, etc.)

---

## 2. Controller Design Targets

### Assumption Made
Controller gains designed for:
- Natural frequency: 50 Hz
- Damping ratio: 0.7 (slightly underdamped)

### Why This Assumption Was Needed
These values provide:
- Good transient response (fast settling, minimal overshoot)
- Adequate disturbance rejection
- Reasonable stability margins

The same targets are used for both systems to ensure fair comparison.

### Information Needed to Remove Assumption
- **Specified bandwidth requirements**
- **Settling time constraints**
- **Maximum overshoot limits**
- **Stability margin requirements**

---

## 3. AMB Force Rating

### Assumption Made
AMB rated force = 2 × rotating group weight (as specified in project requirements)
- Baseline: 5780 N (given)
- New design: 2 × m × g = 2 × 448.8 × 9.81 ≈ 8800 N

### Why This Assumption Was Needed
Directly from project specification for the new design. The baseline value is given in Appendix B.

### Information Needed to Remove Assumption
None - this is specified in the project requirements.

---

## 4. Current Controller Dynamics

### Assumption Made
Current controller treated as a first-order lag for simulation:
- Time constant τ_i = 1/Kp_current = 1/258.75 ≈ 3.86 ms

### Why This Assumption Was Needed
The project provides PI current controller gains. For step response simulation, the current loop dynamics were simplified to a first-order approximation.

### Information Needed to Remove Assumption
- **Complete current loop model** including PWM dynamics
- **Amplifier bandwidth specifications**
- **Current sensor dynamics**

---

## 5. Single-Axis Analysis

### Assumption Made
Step response analysis performed on single radial axis, neglecting:
- Cross-coupling between x and y axes
- Gyroscopic effects at high speeds
- Axial dynamics

### Why This Assumption Was Needed
Single-axis analysis provides clear comparison of controller performance. Full coupled analysis requires more detailed system models.

### Information Needed to Remove Assumption
- **Full rotor dynamic model** (Campbell diagram, mode shapes)
- **Cross-coupled controller gains** if used
- **Gyroscopic coupling coefficients**

---

## 6. AMB Axial Separation

### Assumption Made
AMB separation distance estimated as:
- Baseline: flywheel length + 0.3 m = 1.3 m
- New design: flywheel length + 0.3 m = 2.3 m

### Why This Assumption Was Needed
Needed for tilting stiffness calculation. The 0.3 m accounts for motor, clearances, and AMB positioning.

### Information Needed to Remove Assumption
- **Complete assembly drawing** with AMB positions
- **Detailed rotor layout** with dimensional tolerances

---

## 7. Mass Imbalance Grade

### Assumption Made
ISO G2.5 balance grade for both systems:
- Corresponds to e × ω = 2.5 mm/s
- Typical for precision rotating machinery

### Why This Assumption Was Needed
Consistent with Deliverable 1 assumption. Same balance grade applied to both systems for fair comparison.

### Information Needed to Remove Assumption
- **Actual balance specification** for baseline system
- **Target balance grade** for new design
- **Manufacturing capability constraints**

---

## 8. Damping Sources

### Assumption Made
Damping primarily from PD controller derivative term:
- c_eff = Kd × Ki
- Structural and eddy current damping neglected

### Why This Assumption Was Needed
Controller damping dominates in typical AMB systems. Other damping sources require detailed electromagnetic and structural models.

### Information Needed to Remove Assumption
- **Material damping coefficients**
- **Eddy current loss measurements**
- **Structural damping from FEA or test data**

---

## 9. Steady-State Disturbance Response

### Assumption Made
Step disturbance maintained throughout simulation (no removal). Analysis focuses on initial transient response.

### Why This Assumption Was Needed
Demonstrates controller ability to reject constant disturbances (e.g., static unbalance, gravity loading).

### Information Needed to Remove Assumption
- **Actual disturbance profiles** (shock, vibration spectra)
- **Operational environment specifications**

---

## Summary Table

| # | Assumption | Impact | Information Needed |
|---|------------|--------|-------------------|
| 1 | PD position controller | Controller structure | Actual controller specification |
| 2 | 50 Hz, ζ=0.7 targets | Gain values | Performance requirements |
| 3 | AMB force = 2×weight | AMB sizing | (Specified in project) |
| 4 | First-order current loop | Simulation accuracy | Complete current loop model |
| 5 | Single-axis analysis | Response accuracy | Full rotor dynamics model |
| 6 | AMB separation estimate | Tilting stiffness | Assembly drawing |
| 7 | ISO G2.5 balance | Runout predictions | Actual balance specifications |
| 8 | Controller damping only | Dynamic response | System damping measurements |
| 9 | Constant disturbance | Transient analysis | Actual disturbance profiles |

---

## Impact Assessment

### Critical Assumptions (High Impact)
1. **Controller type and targets** - Directly determines controller gains
2. **Current loop dynamics** - Affects transient response accuracy

### Moderate Assumptions (Medium Impact)
3. AMB separation distance - Affects tilting analysis
4. Single-axis simplification - Affects high-speed response

### Minor Assumptions (Low Impact)
5. Balance grade (consistent with baseline)
6. Damping model (controller-dominated)

---

## Validation Recommendations

1. **Compare controller bandwidth** to any available baseline specifications
2. **Verify stability margins** using Bode or Nyquist analysis
3. **Check current loop bandwidth** is >> position loop bandwidth
4. **Validate runout predictions** against sub-micron measurement capability
