# Deliverable 1 Comparison: v2 vs Peyton's Code

This document compares the key calculations and approaches between `flywheel_analysis_v2.m` and `peytonVersion.m` for Deliverable 1.

---

## Summary Table

| Metric | v2 | Peyton | Difference |
|--------|-----|--------|------------|
| Inertia Formula | Correct | **BUG** | Peyton uses R²-r² instead of R²+r² |
| Thermal Model | Two-surface enclosure | Single surface | v2 more accurate |
| AMB Plant Model | Unstable (correct) | **Stable (wrong)** | Sign of Ks |
| Runout Calculation | Force/Stiffness | Eccentricity only | Different definition |
| Efficiency Definition | E_out/(E_in+E_rec) | E_net/E_in | Different formulas |
| Effective Mass for AMB | Full mass | Half mass | Factor of 2 |

---

## 1. Mass and Inertia Calculations

### v2 (Lines 98-121):
```matlab
% Flywheel inertia (hollow cylinder) - CORRECT
I_flywheel_bl = 0.5 * m_flywheel_bl * (r_outer_bl^2 + r_inner_bl^2);

% Shaft inertia (solid cylinder)
I_shaft_bl = 0.5 * m_shaft_bl * r_inner_bl^2;

baseline.I_total = I_flywheel_bl + I_shaft_bl;
```

### Peyton (Lines 277-278):
```matlab
% Inertia (shaft + hollow flywheel) - BUG: uses minus instead of plus!
J = 0.5 * m_fly * (r_fly^2 - r_shaft^2) + 0.5 * m_shaft * r_shaft^2;
```

### Analysis:
**Peyton has a BUG.** The moment of inertia for a hollow cylinder is:
```
I = ½m(R² + r²)   ← CORRECT (v2)
```
NOT:
```
I = ½m(R² - r²)   ← WRONG (Peyton)
```

**Impact:** Peyton's inertia will be significantly LOWER than correct. This affects:
- Energy storage calculations
- Specific energy
- All dynamic calculations

For the baseline (R=0.215m, r=0.042m):
- Correct: I ∝ (0.215² + 0.042²) = 0.0481
- Peyton: I ∝ (0.215² - 0.042²) = 0.0445

**Peyton's inertia is ~7.5% low** just from the flywheel term.

---

## 2. Torque and Rated Power

### v2 (Lines 172-179):
```matlab
rotor_radius_bl = (baseline.shaft_diameter / 2) + baseline.magnet_thickness;
torque_rated_bl = 2 * pi * rotor_radius_bl^2 * baseline.motor_length * shear_rated;
baseline.power_rated = torque_rated_bl * baseline.omega_min;
```

### Peyton (Lines 422-432):
```matlab
r_rotor = r_shaft + t_mag;  % Same radius definition
T_rated = 2 * pi * r_rotor^2 * h_motor * tau_rated;
P_rated_W = T_rated * omega_min;
```

### Analysis:
**Both are equivalent.** Same formula, same radius (shaft + magnets), same omega_min.

---

## 3. Thermal Model / Surface Area

### v2 (Lines 207-251):
```matlab
% Complex surface area calculation:
A_rotor = A_fly_cylinder + A_fly_ends + A_motor_cylinder + A_shaft_cylinder + A_shaft_ends;

% Two-surface enclosure radiation factor:
rad_factor = 1/epsilon_rotor + (A_rotor/A_housing)*(1/epsilon_housing - 1);

% Temperature calculation:
T_rotor_K = (T_housing_K^4 + P_rotor*rad_factor/(sigma*A_rotor))^0.25;
```

### Peyton (Lines 409-417):
```matlab
% Simpler surface area (flywheel only):
A_surface = pi * (2 * r_o * h_fly) + 2 * pi * (r_o^2 - r_i^2);

% Single surface radiation (no view factor):
T_K = ((Q_gen / (epsilon * sigmaSB * A_surface)) + T_housing_K^4)^(1/4);
```

### Analysis:
**Different thermal models:**

| Aspect | v2 | Peyton |
|--------|-----|--------|
| Surface area | All rotating surfaces | Flywheel only |
| Radiation model | Two-surface enclosure | Single surface to black body |
| Effective emissivity | ~0.38 (due to F_rad≈2.6) | 0.4 directly |

v2's model is more physically accurate for a concentric cylinder enclosure.
Peyton's simpler model may give different (likely higher) temperatures.

---

## 4. AMB Plant Model - CRITICAL DIFFERENCE

### v2 (Lines 496-512):
```matlab
baseline.K_s = -amb_bl.stiffnessConstant;  % NEGATIVE stiffness

% Plant: G = Ki / (m*s² + K_s) where K_s < 0
% This gives: G = Ki / (m*s² - |Ks|)  ← UNSTABLE poles at ±sqrt(|Ks|/m)
G_plant_bl = baseline.K_i / (baseline.total_mass*s^2 + baseline.K_s);
```

### Peyton (Lines 527-528):
```matlab
Ks_mag = abs(Ks);  % Takes magnitude
% Plant: G = Ki / (m*s² + |Ks|)  ← STABLE poles at ±j*sqrt(|Ks|/m)
G_pos = Ki / (m*s^2 + Ks_mag);
```

### Analysis:
**Peyton's AMB plant model is WRONG.**

AMBs have inherently **negative position stiffness** - moving the rotor toward a magnet increases the attractive force, creating instability. The correct plant transfer function is:

```
G(s) = Ki / (m*s² - |Ks|)
```

This has poles at `s = ±sqrt(|Ks|/m)` (real, unstable).

Peyton's model:
```
G(s) = Ki / (m*s² + |Ks|)
```

Has poles at `s = ±j*sqrt(|Ks|/m)` (imaginary, stable oscillation).

**Impact:** Peyton's model doesn't capture the fundamental instability of AMBs. The step response and stability analysis will be fundamentally different.

---

## 5. AMB Effective Mass

### v2:
```matlab
% Uses total rotating mass for AMB analysis
baseline.total_mass  % Full mass
```

### Peyton (Line 501):
```matlab
% Uses half the mass per bearing
m = m_effective * 0.5;
```

### Analysis:
Different assumptions about how mass is distributed between bearings. For a single DOF radial analysis, using full mass or half mass changes:
- Unstable pole frequency
- Dynamic stiffness
- Step response magnitude

v2's approach assumes both bearings support the full mass in parallel.
Peyton assumes each bearing supports half the mass independently.

---

## 6. Dynamic Stiffness

### v2 (Lines 624-633):
```matlab
% Uses negative K_s in calculation
denom_radial = baseline.total_mass*s_val^2 + baseline.K_s + G_cx_val*baseline.K_i;
stiffness_radial(idx) = abs(denom_radial);
```

### Peyton (Lines 583-594):
```matlab
% Uses positive Ks (magnitude)
K_dyn_rad = Ks + Ki * G_cp_rad;  % Ks is positive magnitude here
K_rad = squeeze(freqresp(K_dyn_rad, freq_rad_s));
```

### Analysis:
Different sign conventions for Ks lead to different stiffness calculations. v2 includes the mass term `m*s²` while Peyton appears to compute just `Ks + Ki*G_cp`.

---

## 7. Rotor Runout - VERY DIFFERENT APPROACHES

### v2 (Lines 689-706):
```matlab
% Calculates actual dynamic response
e = (G_grade * 1e-3) / omega;  % Eccentricity from balance grade
F_unbalance = baseline.total_mass * e * omega^2;  % Unbalance force

% Get dynamic stiffness at synchronous frequency
s_val = 1j * omega;
G_cx_val = ...  % Controller at this frequency
dyn_stiff = abs(baseline.total_mass*s_val^2 + baseline.K_s + G_cx_val*baseline.K_i);

% Runout = Force / Stiffness (dynamic response)
runout_1f(idx) = (F_unbalance / dyn_stiff) * 1e6;  % [um]
```

### Peyton (Lines 599-612):
```matlab
% Just returns the eccentricity directly
e_m = G_grade ./ max(omega, 1e-3);  % [m]
runout_mm = e_m * 1e3;  % [mm]
```

### Analysis:
**Completely different definitions:**

| Aspect | v2 | Peyton |
|--------|-----|--------|
| Definition | Displacement due to unbalance force | Raw eccentricity from ISO grade |
| Formula | x = F_unbalance / K_dynamic | e = G / ω |
| Units | μm | mm |
| Physics | Includes AMB stiffness response | Just balance grade definition |

v2 calculates the actual rotor displacement (runout) that would occur due to the unbalance force acting against the AMB's dynamic stiffness.

Peyton returns the eccentricity parameter from the balance grade, which is NOT the same as runout.

**Expected values:**
- v2: Very small (sub-μm) because AMB stiffness is very high
- Peyton: Much larger (mm range) because it's just G/ω

---

## 8. Cycle Efficiency

### v2 (Lines 420-426):
```matlab
E_recovery = max(0, 0.5 * baseline.I_total * (omega_initial^2 - omega_final^2));
efficiency_1c = (E_out / (E_in + E_recovery)) * 100;
```

### Peyton (Lines 476-493):
```matlab
E_out_Wh = sum(max(P_grid_W, 0)) * dt / 3600;
E_in_Wh = -sum(min(P_grid_W, 0)) * dt / 3600;
E_loss_Wh = sum(P_loss_W) * dt / 3600;
E_net_Wh = E_out_Wh - E_loss_Wh;

% Round-trip efficiency
cycleResults.eta_roundtrip = max(E_net_Wh, 0) / E_in_Wh;
```

### Analysis:
Different efficiency definitions:

| | v2 | Peyton |
|--|-----|--------|
| Formula | E_out / (E_in + E_recovery) | (E_out - E_loss) / E_in |
| Accounts for SoC change | Via E_recovery term | Subtracts losses from output |

Both approaches have issues:
- v2's E_recovery logic is questionable (see audit report)
- Peyton's formula subtracts losses from output, which is non-standard

Standard round-trip efficiency would be: η = E_out / E_in (for cycle returning to same SoC)

---

## 9. Numerical Comparisons

Based on the baseline parameters (assuming both run without errors):

| Parameter | v2 (Expected) | Peyton (Expected) | Notes |
|-----------|---------------|-------------------|-------|
| Total mass | ~230-250 kg | ~230-250 kg | Should match |
| Spin inertia | Higher | **~7% Lower** | Due to inertia bug |
| Specific energy | Higher | **Lower** | Due to inertia bug |
| Max temp | ~XX°C | Different | Different thermal models |
| Runout | ~0.001-0.01 μm | ~0.06-0.12 mm | Different definitions! |
| AMB stability | Unstable plant | **Stable plant** | Sign error |

---

## Recommendations

### For v2:
1. Fix efficiency calculation (see audit report)
2. Verify thermal model assumptions

### For Peyton:
1. **CRITICAL:** Fix inertia formula: change `(r_fly^2 - r_shaft^2)` to `(r_fly^2 + r_shaft^2)`
2. **CRITICAL:** Fix AMB plant model sign: use negative stiffness
3. Reconsider runout definition - current version returns eccentricity, not displacement
4. Consider using two-surface enclosure thermal model

---

## Which Code to Trust?

For Deliverable 1:

| Part | Recommendation |
|------|----------------|
| 1a (Losses/Temp) | v2 (better thermal model) |
| 1b (Specific power/energy) | v2 (correct inertia) |
| 1c (Efficiency) | Neither - both have issues |
| 1d (AMB step response) | v2 (correct plant model) |
| 1e (Dynamic stiffness) | v2 (correct sign) |
| 1f (Runout) | v2 (calculates actual displacement) |

**Overall:** v2 is more physically correct, but has some efficiency calculation bugs. Peyton's code has fundamental errors in inertia and AMB modeling.

---

*Generated: December 2, 2025*
