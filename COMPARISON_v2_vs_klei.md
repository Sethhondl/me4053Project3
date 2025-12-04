# Comparison: Team 16 (v2) vs. Klei Repository

This document compares your `v2/flywheel_analysis_v2.m` with the `klei_repo/MagLevFlywheelModel.m` implementation.

## Executive Summary

| Aspect | Your v2 Code | Klei Code |
|--------|--------------|-----------|
| **Structure** | Single flat script (~1800 lines) | Modular with helper functions (~2100+ lines) |
| **Team Cycle** | Uses `team_16_cycle` | Uses `team_2_cycle` |
| **AMB Modeling** | Separate radial + tilting transfer functions | Full 4-DOF ODE simulation |
| **Controller Design** | HW3 methodology with simple scaling | Loop-shaping with frequency domain design |
| **Complexity** | More straightforward, easier to follow | More sophisticated, production-quality |

---

## 1. Code Organization

### Your v2 Code
- **Single monolithic script** with all calculations inline
- Sequential execution of Deliverables 1, 2, and 3
- Uses `s = tf('s')` for transfer function analysis
- Approximately 1800 lines

### Klei Code
- **Highly modular** with many helper functions:
  - `createParams()` - Parameter structure creation
  - `runDeliverable1()`, `runDeliverable2()` - Main deliverable functions
  - `simulate4DOF()` - Full 4-DOF ODE simulation
  - `computeDynamicStiffness4DOF()` - Dynamic stiffness calculation
  - `computeRunoutVsSoc4DOF()` - Runout analysis
  - `evaluateDesignPoint()` - Design space exploration
  - `designLoopShapingController()` - Controller design
- Approximately 2100+ lines with extensive documentation

---

## 2. Key Technical Differences

### 2.1 Mass/Inertia Calculations

**Both approaches are similar:**
```matlab
% v2 (lines 98-121)
V_flywheel_bl = pi * (r_outer_bl^2 - r_inner_bl^2) * baseline.flywheel_length;
m_flywheel_bl = rho_composite * V_flywheel_bl;
I_flywheel_bl = 0.5 * m_flywheel_bl * (r_outer_bl^2 + r_inner_bl^2);

% klei (similar approach in compute4DOFGeometry())
m_fly = rho_fly * pi * (r_fly^2 - r_shaft^2) * h_fly;
J_p_fly = 0.5 * m_fly * (r_fly^2 + r_shaft^2);
```

**Difference:** Klei also calculates COM positions and distances to each AMB for proper 4-DOF modeling.

### 2.2 Torque Calculation

**v2 (lines 172-176):**
```matlab
rotor_radius_bl = (baseline.shaft_diameter / 2) + baseline.magnet_thickness;
torque_rated_bl = 2 * pi * rotor_radius_bl^2 * baseline.motor_length * shear_rated;
```

**Klei (lines 481-484):**
```matlab
r_rotor = r_shaft;  % Uses shaft radius directly
T_rated = 2 * pi * r_rotor^2 * h_motor * tau_rated;
```

**Key Difference:**
- v2 uses `rotor_radius = shaft_radius + magnet_thickness` (air gap surface)
- Klei uses `rotor_radius = shaft_radius` (shaft surface)

This will produce **different rated power values**!

### 2.3 AMB Step Response Analysis

**v2 Approach (Part 1d, lines 463-900):**
- Uses **transfer function approach** with decoupled radial and tilting modes
- Simulates using `lsim()` with state-space conversion
- Combines x_cm and α·a to get bearing positions
- Manually calculates control forces and currents

**Klei Approach:**
- Uses **full 4-DOF ODE simulation** (`simulate4DOF()`)
- 20-state vector: [x, y, α, β, velocities, controller states, currents]
- Solves using `ode45` with proper disturbance injection
- More physically accurate but more complex

### 2.4 Dynamic Stiffness Calculation

**v2 (lines 924-945):**
```matlab
denom_radial = baseline.total_mass*s_val^2 + baseline.K_s + G_cx_val*baseline.K_i;
stiffness_radial(idx) = abs(denom_radial);
```

**Klei (computeDynamicStiffness4DOF, lines 1146-1150):**
```matlab
K_rad_complex = m*s^2 - 2*Ks + G_cx;  % Note: -2*Ks for two bearings
K_radial(k) = abs(K_rad_complex);
```

**Key Difference:** Klei uses `-2*Ks` accounting for two AMBs contributing to radial stiffness.

### 2.5 Deliverable 2 Design Study

**v2 Approach:**
- Fixed design variable ranges: `magnet_thickness = 2-10mm`, `max_speed = 20k-60k RPM`
- Energy target: 40 kWh
- Simple grid search with viability checks
- Uses `team_16_cycle`

**Klei Approach:**
- Analyzes `team_2_cycle` requirements first
- Applies safety factors (1.2× energy, 1.1× power)
- Sizes flywheel to meet requirements dynamically
- More comprehensive design evaluation function

### 2.6 Deliverable 3 Controller Design

**v2 (HW3 methodology, lines 1526-1583):**
```matlab
% Current controller: Pole-zero cancellation
Kp_current_new = L_coil_new * omega_bw_current;
Ki_current_new = Kp_current_new * R_coil_new / L_coil_new;

% Position controller: Unity gain at crossover
plant_mag_at_crossover = K_i_new / abs(-mass * omega^2 - K_s);
new_kpx = 1 / plant_mag_at_crossover;
```

**Klei (Loop-shaping method, lines 1740-1952):**
```matlab
% Full frequency-domain loop shaping
% G_c(s) = K_o * (1 + s/w_z1)(1 + s/w_z2) / [s(1 + s/w_p)]
% Solves for zeros to achieve target phase margin
```

**Key Difference:** Klei's approach is more rigorous with explicit phase margin design.

---

## 3. Numerical Differences Expected

| Parameter | v2 | Klei | Reason |
|-----------|----|----|--------|
| Rated torque | Higher | Lower | Different rotor radius definition |
| Rated power | Higher | Lower | Torque difference |
| Dynamic stiffness | Lower | Higher | 2×Ks factor |
| Step response | Similar | Similar | Same physics |

---

## 4. Team Cycle Difference

**v2:** Uses `team_16_cycle` (6-hour cycle)
**Klei:** Uses `team_2_cycle` (6-hour cycle, different team)

The cycle power profiles are likely different, leading to different:
- Energy capacity requirements
- Efficiency calculations
- Optimal design selection

---

## 5. Strengths and Weaknesses

### Your v2 Code Strengths:
1. **Straightforward and readable** - Easy to follow the analysis flow
2. **Consistent with baseline formula** - Uses same torque calculation throughout
3. **Good documentation** - Clear comments explaining each step
4. **Control System Toolbox integration** - Leverages tf/ss for stability analysis

### Your v2 Code Weaknesses:
1. **Not modular** - Hard to reuse components
2. **Missing 4-DOF COM geometry** - Doesn't calculate actual bearing-to-COM distances
3. **Simplified AMB model** - Doesn't include gyroscopic effects

### Klei Code Strengths:
1. **Highly modular and reusable**
2. **Full 4-DOF simulation** - More accurate AMB dynamics
3. **Sophisticated controller design** - Proper loop-shaping methodology
4. **Comprehensive design study** - Requirements-driven sizing

### Klei Code Weaknesses:
1. **More complex** - Harder to debug
2. **Torque radius inconsistency** - Uses shaft radius instead of air gap

---

## 6. Recommendations

### Consider adopting from Klei:
1. **4-DOF geometry calculations** - Proper COM distances to AMBs
2. **Dynamic stiffness with 2×Ks** - More physically accurate
3. **Loop-shaping controller design** - Better phase margin control

### Keep from your v2:
1. **Torque at air gap** - More physically correct
2. **Clear structure** - Easy to understand and verify
3. **team_16_cycle** - Your team's specific cycle

---

## 7. Files for Reference

- Your code: `v2/flywheel_analysis_v2.m`
- Klei code: `klei_repo/MagLevFlywheelModel.m`
- Klei's additional analysis: `klei_repo/FlywheelDynamicsModeling.m`
- Klei's cycle analysis: `klei_repo/analyze_team2_cycle.m`
