# Numeric Comparison: V2 vs Klei (using team_16_cycle)

## Deliverable 1: Baseline Analysis

| Metric | V2 Code | Klei Code | Match? | Notes |
|--------|---------|-----------|--------|-------|
| **Total Mass** | 293.10 kg | 293.10 kg | ✅ | Exact match |
| **Specific Power** | 0.693 kW/kg | 0.528 kW/kg | ❌ | 31% diff - torque radius |
| **Specific Energy** | 33.82 Wh/kg | 33.47 Wh/kg | ✅ | 1% diff |
| **15-min Efficiency** | 95.47% | 94.0% | ✅ | ~1.5% diff |
| **K_s** | -1.348e+07 N/m | -1.348e+07 N/m | ✅ | Exact match |
| **K_i** | 819.10 N/A | 819.10 N/A | ✅ | Exact match |
| **Dynamic Stiffness @ 100 Hz** | 153.0 MN/m | 155.0 MN/m | ✅ | 1.3% diff |
| **Peak Step Displacement** | 3.56 μm | 16.98 μm | ❌ | Different AMB models |
| **Max Runout** | 1.47 μm | 1.5 mm | ❌ | Units differ (see note) |

### Key Differences Explained

1. **Specific Power (0.693 vs 0.528 kW/kg)**
   - V2 uses torque at air gap: `r = shaft_radius + magnet_thickness`
   - Klei uses shaft surface: `r = shaft_radius`
   - This gives v2 ~31% higher rated power

2. **Peak Step Displacement (3.56 vs 16.98 μm)**
   - V2 uses transfer function approach with separate radial/tilting modes
   - Klei uses full 4-DOF ODE simulation with coupled dynamics
   - Different controller gain interpretations may contribute

3. **Runout Units**
   - V2 reports in **μm** (micrometers)
   - Klei reports in **mm** (millimeters)
   - If klei's 1.5 mm = 1500 μm, there's a large discrepancy
   - If klei's 0.0015 mm = 1.5 μm, then values are similar

---

## Deliverable 2: Optimal Design

| Metric | V2 Code | Klei Code | Notes |
|--------|---------|-----------|-------|
| **Magnet Thickness** | 10.0 mm | 3.37 mm | Different optima |
| **Max Speed** | 40,000 RPM | 33,125 RPM | Different ranges |
| **Flywheel Diameter** | 430 mm | 518.9 mm | Fixed vs variable |
| **Flywheel Length** | 4000 mm | 2854.9 mm | Different sizing |
| **Total Mass** | 1042 kg | 1227 kg | ~18% diff |
| **Rated Power** | 494.5 kW | 487.8 kW | ~1% diff |
| **Specific Power** | 0.475 kW/kg | 0.397 kW/kg | Torque effect |
| **Specific Energy** | 37.68 Wh/kg | 33.55 Wh/kg | Different sizes |
| **6-hr Efficiency** | 97.44% | 97.2% | ~0.2% diff |

### Design Approach Differences

- **V2**: Fixed diameter (430mm), varies length and speed
- **Klei**: Variable diameter, height, and speed optimization
- Both meet the team_16_cycle requirements (~488 kW, ~40 kWh)

---

## Deliverable 3: Controller Design

| Metric | V2 Code | Klei Code | Notes |
|--------|---------|-----------|-------|
| **Position k_p** | 4.83e+04 | 5.27e+08 | Different controller types |
| **Position k_i** | 4.69e+05 | 4.87e+09 | V2 is position-to-current |
| **Position k_d** | 4.10e+02 | 1.05e+06 | Klei is position-to-force |
| **Crossover freq** | 170 Hz | ~600 Hz | Different bandwidths |
| **Phase margin** | 69.7° | Not reported | V2 targets 60° |

### Controller Design Differences

1. **V2**: Loop-shaping, position-to-current output, multiply by K_i
2. **Klei**: Stiffness-margin scaling from baseline position-to-force gains

---

## Plot Comparison

Both scripts generated the following plots (saved in `comparison_plots/`):

| Part | V2 File | Klei File | Content |
|------|---------|-----------|---------|
| 1a | `part1a_losses_temperature.png` | `klei_part1a_losses_temperature.png` | Losses & Temp vs SoC |
| 1c | `part1c_storage_cycle.png` | `klei_part1c_storage_cycle.png` | 15-min baseline cycle |
| 1d | `part1d_amb_step_response.png` | `klei_part1d_amb_step_response.png` | Step response |
| 1e | `part1e_dynamic_stiffness.png` | `klei_part1e_dynamic_stiffness.png` | Dynamic stiffness |
| 1f | `part1f_rotor_runout.png` | `klei_part1f_rotor_runout.png` | Runout vs SoC |
| 2a | `part2a_design_tradeoffs.png` | `klei_part2a_design_tradeoffs.png` | Design space |
| 2b | `part2b_team16_cycle.png` | `klei_part2b_optimal_cycle.png` | Team 16 cycle |
| 3b | `part3b_step_response.png` | `klei_part3b_step_response.png` | Step comparison |
| 3c | `part3c_stiffness_runout.png` | `klei_part3c_dynamic_stiffness.png` | Stiffness comparison |

---

## Summary

### What Matches Well (✅)
- Total rotating mass (identical)
- AMB parameters (K_s, K_i)
- Specific energy (~1% diff)
- Cycle efficiency (~1.5% diff)
- Dynamic stiffness @ 100 Hz (~1.3% diff after bug fix)
- Design requirements analysis (both identify ~488 kW, ~40 kWh needed)

### What Differs Significantly (❌)
- **Specific power** (31% diff) - due to torque radius definition
- **Optimal design geometry** - different optimization approaches
- **Step response amplitude** - different AMB simulation models
- **Controller gains** - different design methodologies

### Recommendations

1. The **torque radius** difference is the most significant physics choice
   - V2's air-gap approach is more correct for surface-mounted PM motors
   - Klei's shaft surface approach is conservative

2. The **dynamic stiffness** values now match well after fixing the K_i bug

3. Both designs successfully meet the team_16_cycle requirements
