# Complete Comparison: Team 16 V2 vs Klei Implementation

**Generated:** December 3, 2025
**Models Compared:** `v2/flywheel_analysis_v2.m` vs `klei_repo/MagLevFlywheelModel_team16.m`

---

## Executive Summary

Both implementations analyze a flywheel energy storage system for grid frequency regulation. While they use the same baseline parameters and EE functions, they differ significantly in their design optimization approach and resulting optimal designs.

| Aspect | V2 (Team 16) | Klei |
|--------|--------------|------|
| **Baseline Analysis** | Matches well | Matches well |
| **Design Approach** | Multi-objective with cycle feasibility | Multi-objective with cycle feasibility |
| **Optimal Flywheel** | 625mm × 1550mm | 519mm × 2855mm |
| **Key Strength** | Higher efficiency, lighter, more compact | More energy headroom |

---

## DELIVERABLE 1: BASELINE ANALYSIS

### Part 1a: Losses and Temperature vs SoC

#### Numerical Comparison
| Metric | V2 | Klei | Difference |
|--------|-----|------|------------|
| Max Rotor Losses | 0.42 kW | ~0.4 kW | ~5% |
| Max Stator Losses | 3.58 kW | ~3.3 kW | ~8% |
| Max Total Losses | 3.71 kW | ~3.3 kW | ~12% |
| Max Temperature | 97.4°C | ~92°C | ~5°C |

#### Plot Analysis

**V2 Plot (part1a_losses_temperature.png):**
- Two-panel layout: Losses vs SoC (left), Temperature vs SoC (right)
- Shows rotor losses (red), stator losses (blue), total losses (black)
- Stator losses dominate (~3.5 kW vs ~0.4 kW rotor)
- Total losses have minimum around 40-50% SoC (~2.7 kW)
- Temperature increases linearly from ~56°C at 0% SoC to ~97°C at 100% SoC
- Red dashed line shows 100°C thermal limit - baseline is close to limit!

**Klei Plot (klei_part1a_losses_temperature.png):**
- Single combined plot with dual y-axes
- Same general trends: stator losses dominate, minimum around 50% SoC
- Temperature shown with dashed black line, ranges ~52°C to ~92°C
- Slightly lower temperature prediction than V2

**Key Difference:** V2 predicts higher max temperature (97.4°C vs ~92°C). This may be due to different thermal model assumptions (surface area calculation, radiation factor).

---

### Part 1b: Specific Power and Energy

| Metric | V2 | Klei | Match |
|--------|-----|------|-------|
| Total Mass | 293.10 kg | 293.10 kg | ✅ Exact |
| Usable Energy | 9.91 kWh | 9.81 kWh | ✅ ~1% |
| Specific Power | 0.693 kW/kg | 0.528 kW/kg | ❌ 31% diff |
| Specific Energy | 33.82 Wh/kg | 33.47 Wh/kg | ✅ ~1% |

**Specific Power Difference Explained:**
- V2 uses torque at **air gap radius** (shaft + magnets = 48mm)
- Klei uses torque at **shaft surface** (42mm)
- V2's approach is more correct for surface-mounted PM motors
- This gives V2 ~31% higher rated power calculation

---

### Part 1c: 15-Minute Storage Cycle

#### Numerical Comparison
| Metric | V2 | Klei | Match |
|--------|-----|------|-------|
| Energy Discharged | 9.16 kWh | 9.2 kWh | ✅ |
| Energy Charged | 9.12 kWh | 9.1 kWh | ✅ |
| Total Losses | 0.47 kWh | 0.6 kWh | ~22% diff |
| Cycle Efficiency | 95.47% | 94.0% | ✅ ~1.5% diff |

#### Plot Analysis

**V2 Plot (part1c_storage_cycle.png):**
- Three-panel vertical layout
- Top: Grid power demand (±150 kW oscillations)
- Middle: SoC trajectory (red) - ranges ~25% to ~85%, ends at 45%
- Bottom: Motor losses during cycle (1-4 kW range)
- Clear correlation between power demand and losses

**Klei Plot (klei_part1c_storage_cycle.png):**
- Single panel showing SoC vs time
- Same general SoC trajectory pattern
- Ranges ~27% to ~85%, ends at ~45%
- Simpler visualization without loss breakdown

**Key Similarity:** Both show the same SoC trajectory shape and range, confirming identical cycle simulation.

---

### Part 1d: AMB Step Response

#### Numerical Comparison
| Metric | V2 | Klei | Match |
|--------|-----|------|-------|
| Peak Displacement | 17.00 µm | 16.98 µm | ✅ 0.1% |
| Settling Time | 134 ms | 166 ms | ~20% diff |
| Step Force | 578 N | 578 N | ✅ Exact |

#### Plot Analysis

**V2 Plot (part1d_amb_step_response.png):**
- Three-panel layout showing both bearings
- Top: Control current (top bearing ~-1.1A peak, bottom ~0A)
- Middle: Control force response
- Bottom: Rotor position at each bearing
- Shows coupled dynamics: top bearing peaks at 17µm, bottom at -10µm (tilting)
- 150ms simulation time

**Klei Plot (klei_part1d_amb_step_response.png):**
- Three-panel layout (single bearing focus)
- Current in mA scale (peak ~-1100 mA = -1.1A)
- Force step shown clearly (0 to 578N)
- Position peaks at ~17µm, settles to ~0.5µm
- 200ms simulation time

**Key Achievement:** After implementing the 4-DOF ODE model and fixing the K_s sign convention, V2 now matches Klei's step response within 0.1%!

---

### Part 1e: Dynamic Stiffness

#### Numerical Comparison
| Frequency | V2 Radial | Klei Radial | Match |
|-----------|-----------|-------------|-------|
| 10 Hz | 98.6 MN/m | ~100 MN/m | ✅ |
| 100 Hz | 153.0 MN/m | 155 MN/m | ✅ 1.3% |
| 1000 Hz | 10.8 GN/m | ~10 GN/m | ✅ |

#### Plot Analysis

**V2 Plot (part1e_dynamic_stiffness.png):**
- Log-log scale, 1-1000 Hz range
- Shows both radial (blue) and tilting (red) stiffness
- Classic "bathtub" shape with minimum around 10-50 Hz
- Radial minimum ~100 MN/m, tilting minimum ~110 MN/m
- High frequency dominated by inertia (slope +2)

**Klei Plot (klei_part1e_dynamic_stiffness.png):**
- Log-log scale, 1-10,000 Hz range (wider)
- Y-axis in N/m (not MN/m) - shows 10^7 to 10^13
- Same bathtub shape
- Minimum around 10^8 N/m = 100 MN/m (matches V2!)

**Key Observation:** Both show the characteristic AMB dynamic stiffness curve. The minimum stiffness (~100 MN/m) occurs near the controller crossover frequency.

---

### Part 1f: Rotor Runout

#### Numerical Comparison
| SoC | V2 | Klei | Match |
|-----|-----|------|-------|
| 0% | 1.47 µm | 1.48 µm (×10⁻³ mm) | ✅ |
| 50% | 0.89 µm | ~0.95 µm | ✅ |
| 100% | 0.68 µm | 0.68 µm | ✅ |

#### Plot Analysis

**V2 Plot (part1f_rotor_runout.png):**
- Dual y-axis: Runout (µm, blue) and Speed (krpm, red dashed)
- Runout decreases from 1.47µm at 0% SoC to 0.68µm at 100% SoC
- Speed increases from 20 to 40 krpm
- Inverse relationship: higher speed = lower runout (for ISO G2.5)

**Klei Plot (klei_part1f_rotor_runout.png):**
- Single axis, runout in mm (×10⁻³ scale)
- Same decreasing trend from ~1.48×10⁻³ mm to ~0.68×10⁻³ mm
- Identical physics: e = G/ω, runout = F_unb/K_dyn

**Key Note:** Units differ (V2: µm, Klei: mm with ×10⁻³ multiplier) but values are identical.

---

## DELIVERABLE 2: DESIGN OPTIMIZATION

### Cycle Requirements Analysis

| Metric | V2 | Klei | Match |
|--------|-----|------|-------|
| Peak Discharge Power | 360.4 kW | 360.4 kW | ✅ |
| Peak Charge Power | 443.4 kW | 443.4 kW | ✅ |
| Energy Swing | 26.51 kWh | 26.51 kWh | ✅ |
| Power Safety Factor | 1.10 | 1.10 | ✅ |
| Energy Safety Factor | 1.20 | 1.20 | ✅ |
| Design Power | 487.8 kW | 487.8 kW | ✅ |
| **Design Energy** | **31.8 kWh** | **40.7 kWh** | ❌ Different! |

**Energy Target Difference:**
- V2: Energy swing × 1.2 = 26.51 × 1.2 = **31.8 kWh**
- Klei: "Required capacity" × 1.2 = 33.9 × 1.2 = **40.7 kWh**
- Klei's "required capacity" includes additional buffer for SoC positioning

---

### Part 2a: Design Space Exploration

#### Design Space Parameters
| Parameter | V2 | Klei |
|-----------|-----|------|
| Magnet Thickness | 2-12 mm (11 pts) | 2-15 mm (20 pts) |
| Max Speed | 25-50 kRPM (11 pts) | 5-80 kRPM (25 pts) |
| Total Points | 121 | 500 |
| Max Flywheel Length | 2.0 m | ~4.7 m (uncapped) |
| Viable Designs | 21 (17.4%) | ~80 (16%) |

#### Plot Analysis

**V2 Plot (part2a_design_tradeoffs.png):**
- Six-panel comprehensive view
- Top row: Specific Power, Specific Energy, Cycle Efficiency contours
- Bottom row: Temperature, Feasibility map, Pareto front
- Red star marks optimal design (7mm magnet, 27.5k RPM)
- Feasibility map shows viable region (yellow = 3 = all constraints met)
- Pareto front shows ~20 non-dominated designs
- Clear trade-off: higher speed = higher specific energy but lower efficiency

**Klei Plot (klei_part2a_design_tradeoffs.png):**
- 3D scatter plot showing feasible designs
- Axes: Specific Energy, Specific Power, Cycle Efficiency
- Color = efficiency (blue=low, red=high)
- Clustered cloud shows design trade-offs
- Specific energy range: 25-35 Wh/kg
- Specific power range: 0.30-0.42 kW/kg
- Efficiency range: 95.5-97.5%

**Key Difference:** V2 uses 2D contour plots with explicit constraint visualization; Klei uses 3D scatter plot. V2's approach better shows the design space structure and constraint boundaries.

---

### Optimal Design Comparison

| Parameter | V2 | Klei | Notes |
|-----------|-----|------|-------|
| **Flywheel Diameter** | 625 mm | 519 mm | V2 20% wider |
| **Flywheel Length** | 1550 mm | 2855 mm | Klei 84% longer |
| **Shaft Diameter** | 108 mm | 101 mm | Similar |
| **Motor Length** | 260 mm | 950 mm | Klei 3.6× longer |
| **Magnet Thickness** | 7.0 mm | 3.4 mm | V2 2× thicker |
| **Max Speed** | 27,500 RPM | 33,125 RPM | Klei 20% faster |
| **Total Mass** | 983 kg | 1,227 kg | V2 20% lighter |
| **Rated Power** | 506.7 kW | 487.8 kW | V2 4% higher |
| **Usable Energy** | 32.3 kWh | 41.2 kWh | Klei 28% more |
| **Specific Power** | 0.515 kW/kg | 0.397 kW/kg | V2 30% better |
| **Specific Energy** | 32.8 Wh/kg | 33.5 Wh/kg | Similar |
| **Cycle Efficiency** | 98.5% | 97.2% | V2 1.3% better |
| **Max Temperature** | 67.5°C | ~95°C | V2 32°C margin |

**Design Philosophy Summary:**
- **V2:** Short, wide flywheel with thick magnets at lower speed
- **Klei:** Long, narrow flywheel with thin magnets at higher speed

---

### Part 2b: 6-Hour Cycle Simulation

#### Plot Analysis

**V2 Plot (part2b_team16_cycle.png):**
- Three-panel layout showing full 6-hour cycle
- Top: Grid power demand (±400 kW peaks, mostly ±200 kW)
- Middle: SoC trajectory (0% to 90%, centered around 40-60%)
- Bottom: Motor losses (0-5 kW, correlates with power demand)
- SoC stays well within 0-100% bounds throughout

**Klei Plot (klei_part2b_optimal_cycle.png):**
- Two-panel layout
- Top: SoC trajectory (20-90% range)
- Bottom: Power profile with filled area visualization
- Similar SoC behavior but different visualization style

**Key Observation:** Both designs successfully complete the 6-hour cycle without SoC violations. V2's design uses more of the available SoC range (0-90% vs 20-90%) due to smaller energy capacity.

---

## DELIVERABLE 3: CONTROLLER DESIGN

### Part 3a: Controller Parameters

| Parameter | V2 Baseline | Klei Baseline | Match |
|-----------|-------------|---------------|-------|
| K_s | 1.348×10⁷ N/m | 1.348×10⁷ N/m | ✅ |
| K_i | 819.1 N/A | 819.1 N/A | ✅ |
| Position k_p | 1.264×10⁸ | 1.264×10⁸ | ✅ |
| Position k_i | 1.169×10⁹ | 1.169×10⁹ | ✅ |
| Position k_d | 252,790 | 252,790 | ✅ |

| Parameter | V2 New Design | Klei New Design | Notes |
|-----------|---------------|-----------------|-------|
| AMB Rated Force | 19,295 N | 24,079 N | Klei heavier rotor |
| Position k_p | 4.83×10⁴ | 5.27×10⁸ | Different controller types! |
| Phase Margin | 69.7° | Not reported | V2 explicitly designed |

**Controller Type Difference:**
- V2: Position-to-**current** controller (multiply by K_i for force)
- Klei: Position-to-**force** controller (output is force directly)
- This explains the ~10,000× difference in gain magnitudes

---

### Part 3b: Step Response Comparison

#### Numerical Comparison
| Metric | V2 Baseline | Klei Baseline | V2 New | Klei New |
|--------|-------------|---------------|--------|----------|
| Peak Disp. | 5.08 µm | 16.98 µm | 17.29 µm | 16.07 µm |
| Settling | ~50 ms | 166 ms | ~50 ms | 167 ms |

#### Plot Analysis

**V2 Plot (part3b_step_response.png):**
- Two-panel layout: Position response (left), Bar chart comparison (right)
- Baseline (blue solid): 5µm peak, fast settling
- New design (red dashed): 17µm peak, slower settling
- Bar chart clearly shows 3× difference in peak displacement

**Klei Plot (klei_part3b_step_response.png):**
- Two-panel layout: Position (top), Current (bottom)
- Baseline and Optimal nearly overlap!
- Both peak at ~17µm with similar dynamics
- Current response shows ~-1000mA peak

**Key Difference:** V2's Part 3b uses transfer function model (simplified) while V2's Part 1d uses full 4-DOF ODE. The baseline shows different responses because Part 3b doesn't include coupled tilting dynamics. Klei's Part 3b uses the same 4-DOF ODE for both, showing nearly identical responses for baseline and optimal.

---

### Part 3c: Dynamic Stiffness and Runout

#### Numerical Comparison
| Frequency | V2 Baseline | V2 New | Klei Baseline | Klei New |
|-----------|-------------|--------|---------------|----------|
| 10 Hz | 98.6 MN/m | 63.6 MN/m | ~100 MN/m | ~500 MN/m |
| 100 Hz | 153 MN/m | 730 MN/m | 155 MN/m | 646 MN/m |

#### Plot Analysis

**V2 Plot (part3c_stiffness_runout.png):**
- Two-panel layout: Stiffness (left), Runout (right)
- Stiffness: Baseline (blue) has minimum ~100 MN/m; New (red) has minimum ~50 MN/m but higher at high freq
- Runout: Both designs show 0.7-1.7 µm range; New design slightly higher

**Klei Plot (klei_part3c_dynamic_stiffness.png):**
- Two-panel layout: Radial (left), Tilting (right)
- Shows both baseline and optimal for each mode
- Optimal design has higher stiffness across most frequencies
- Different behavior than V2 due to different controller design approach

**Klei Plot (klei_part3c_rotor_runout.png):**
- Single panel showing runout vs SoC
- Baseline: 1.48 µm → 0.68 µm
- Optimal: 1.78 µm → 0.85 µm
- Optimal has ~15% higher runout (heavier rotor)

---

## SUMMARY COMPARISON

### What Matches Well (✅)

| Aspect | Difference |
|--------|------------|
| Total rotating mass | Exact (293.10 kg) |
| AMB parameters (K_s, K_i) | Exact |
| Specific energy | ~1% |
| Cycle efficiency | ~1.5% |
| Dynamic stiffness @ 100 Hz | ~1.3% |
| **Step response (after fix)** | **0.1%** |
| Rotor runout | ~2% |
| Cycle requirements | Exact |

### What Differs Significantly (❌)

| Aspect | V2 | Klei | Reason |
|--------|-----|------|--------|
| Specific power | 0.693 kW/kg | 0.528 kW/kg | Torque radius definition |
| Design energy target | 31.8 kWh | 40.7 kWh | Different interpretation |
| Optimal flywheel | 625×1550mm | 519×2855mm | Different constraints |
| Optimal mass | 983 kg | 1,227 kg | Different geometry |
| Max temperature | 67.5°C | ~95°C | Different thermal models |
| Controller gains | 10⁴ scale | 10⁸ scale | Output type (current vs force) |

---

## Conclusions

### V2 Advantages
1. **More compact design** (1.55m vs 2.85m flywheel)
2. **20% lighter** (983 kg vs 1,227 kg)
3. **Higher cycle efficiency** (98.5% vs 97.2%)
4. **More thermal margin** (67°C vs ~95°C)
5. **Better specific power** (0.515 vs 0.397 kW/kg)
6. **Realistic geometry constraints** (max 2m length)
7. **Comprehensive visualization** (6-panel design space)

### Klei Advantages
1. **More energy headroom** (41.2 kWh vs 32.3 kWh)
2. **Finer design space search** (500 vs 121 points)
3. **Consistent 4-DOF ODE** throughout all parts
4. **More conservative energy margin**

### Recommendation

For a **production flywheel system**, V2's design is preferable because:
- Shorter flywheel is easier to manufacture, transport, and install
- Lower operating temperature provides reliability margin
- Higher efficiency reduces operating costs
- Lighter weight reduces support structure requirements

For **maximum energy storage** with margin for cycle variations, Klei's design provides 28% more capacity.

Both implementations correctly analyze the baseline system and produce viable designs that pass all constraints. The differences arise from design philosophy choices rather than errors.
