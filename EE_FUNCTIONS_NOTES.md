# EE Functions & Thermal Model Notes

## Findings from HW3 Debugging (Nov 2025)

These notes document important learnings about the ee_functions and thermal modeling for the flywheel energy storage system.

---

## 1. EE Functions Input Units

**IMPORTANT:** The ee_functions in `ee_functions/` expect current in **Amps**, not per-unit as documented in Appendix A.

### Evidence:
- `rotorLosses.m` line 27: `B_field = Br * (statorCurrent / 100);`
- This divides input by 100, so passing `100` gives B_field = 1.2 T (correct)
- Passing `1.0` (per-unit) gives B_field = 0.012 T (essentially zero losses)

### Recommendation:
```matlab
stator_current = 100;  % Use Amps, not per-unit (functions divide by 100 internally)
```

### Impact of Wrong Units:
| Input Value | B_field | Shear Stress | Motor Length | Rotor Losses |
|-------------|---------|--------------|--------------|--------------|
| 100 (Amps)  | 1.2 T   | 13,500 Pa    | Reasonable   | Small but nonzero |
| 1.0 (pu)    | 0.012 T | 135 Pa       | 100x too long | ~0 |

---

## 2. Thermal Model Requirements

### Correct Two-Surface Enclosure Radiation Formula

In vacuum, heat transfer is radiation only between rotor and housing:

```matlab
% Parameters from Table 1
epsilon_rotor = 0.4;        % Rotor emissivity
epsilon_housing = 0.9;      % Housing emissivity
sigma = 5.67e-8;            % Stefan-Boltzmann constant [W/(m^2*K^4)]
T_housing = 30;             % Housing temperature [C] (from Appendix B)

% Surface areas
A_rotor = pi * D_flywheel * L_flywheel + 2 * pi * (r_o^2 - r_i^2);
housing_inner_d = D_flywheel + 2 * radial_clearance;
A_housing = pi * housing_inner_d * L_flywheel + 2 * pi * (housing_inner_d/2)^2;

% Radiation resistance factor for two-surface enclosure
rad_factor = 1/epsilon_rotor + (A_rotor/A_housing)*(1/epsilon_housing - 1);

% Solve for rotor temperature
T_housing_K = T_housing + 273.15;
T_rotor_K = (T_housing_K^4 + P_rotor * rad_factor / (sigma * A_rotor))^(1/4);
T_rotating = T_rotor_K - 273.15;  % Convert to Celsius
```

### Key Points:
- **Only rotor losses heat the rotating group** (stator is stationary, outside vacuum)
- Use housing temperature (30°C) as reference, not ambient (25°C)
- Must account for view factors via the `rad_factor` term

---

## 3. Table 1 Parameters (Quick Reference)

| Parameter | Value | Notes |
|-----------|-------|-------|
| Max safe steel shaft tip speed | 175 m/s | |
| Max safe PM tip speed | 175 m/s | |
| Max safe composite flywheel tip speed | 900 m/s | |
| Mass density - composite | 1600 kg/m³ | |
| Mass density - steel | 7850 kg/m³ | |
| Mass density - PM | 7850 kg/m³ | |
| Max safe rotating group temp | 100°C | |
| Min magnet thickness | 2 mm | |
| **Rotor emissivity** | **0.4** | Often missed! |
| **Housing emissivity** | **0.9** | Often missed! |
| Axial clearances | 20 mm | Between components |
| Radial clearance (flywheel-housing) | 20 mm | |
| Balance grade | G2.5 (ISO 1940) | |

---

## 4. Rotor Losses Observations

The `rotorLosses.m` function produces **very small values** compared to stator losses:
- Rotor losses: ~1-100 W range
- Stator losses: ~1-10 kW range

This is because:
1. Small eddy current coefficient: `k_eddy = 2.5e-3`
2. Small magnet thickness squared: `(6e-3)^2 = 36e-6`
3. Eddy losses formula: `P_eddy = k_eddy * freq^2 * B^2 * thickness^2 * A`

**Result:** Temperature stays close to housing temperature (~30-31°C) since rotor losses are small.

---

## 5. Stator Losses Breakdown

Stator losses have two components:

1. **Copper losses:** `P_copper = 3 * I^2 * R_phase`
   - Small when I = 1.0 (per-unit)
   - Significant when I = 100 (Amps)

2. **Iron losses:** `P_iron = k_iron * freq^1.5 * B^2 * mass_iron`
   - Dominates at high speeds
   - Scales with frequency^1.5

---

## 6. Common Mistakes to Avoid

1. **Using per-unit current (1.0) instead of Amps (100)**
   - Causes near-zero rotor losses and wrong shear stress

2. **Using wrong emissivity (0.3 instead of 0.4)**
   - Affects thermal calculation accuracy

3. **Using simple radiation instead of two-surface enclosure**
   - Must account for view factors between rotor and housing

4. **Using ambient temp (25°C) instead of housing temp (30°C)**
   - Housing is the thermal reference in vacuum

5. **Using total losses for temperature calculation**
   - Only rotor losses heat the rotating group (stator is external)

---

## 7. Baseline System Reference (Appendix B)

For comparison, the baseline system uses:
- Max speed: 40,000 r/min
- Flywheel diameter: 430 mm
- Flywheel length: 1000 mm
- Motor length: 250 mm
- Shaft diameter: 84 mm
- Magnet thickness: 6 mm
- Current controller: Kp = 345, Ki = 2149

---

*Last updated: November 30, 2025*
