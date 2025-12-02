# Code Audit Report: flywheel_analysis_v2.m

**Date:** December 1, 2025
**Auditor:** Claude Code
**File:** `v2/flywheel_analysis_v2.m` (1461 lines)

---

## Executive Summary

This audit reviews the MATLAB code for bugs, logical/physical correctness, and formatting consistency. The code is generally well-structured but contains several issues that could affect numerical accuracy and result interpretation.

| Category | Critical | Major | Minor |
|----------|----------|-------|-------|
| Bugs | 2 | 4 | 3 |
| Logic/Physics | 1 | 3 | 2 |
| Formatting | 0 | 2 | 8 |

---

## 1. BUGS

### 1.1 CRITICAL

#### B1. Efficiency Calculation Logic Error (Lines 420-426)

```matlab
E_recovery = max(0, 0.5 * baseline.I_total * (omega_initial^2 - omega_final^2));
efficiency_1c = (E_out / (E_in + E_recovery)) * 100;
```

**Issue:** The `E_recovery` term is logically inverted for some scenarios:
- If `omega_final > omega_initial` (flywheel gained energy), `E_recovery = 0`
- But the flywheel has MORE stored energy, not less
- The denominator should account for net energy input, not add recovery energy

**Correct Approach:**
```matlab
% Net energy into flywheel = E_in - E_out - E_loss = dE_stored
dE_stored = 0.5 * baseline.I_total * (omega_final^2 - omega_initial^2);
% Round-trip efficiency = E_out / (E_in - dE_stored)
efficiency_1c = (E_out / (E_in - dE_stored)) * 100;
```

**Impact:** Efficiency values may be significantly incorrect, especially for cycles that don't return to initial SoC.

---

#### B2. Part 2b Efficiency Division by Zero/Negative (Lines 1084-1091)

```matlab
dE_stored = E_final_2b - E_initial_2b;
efficiency_2b = (E_out_2b / (E_in_2b - dE_stored)) * 100;

% Sanity check: if efficiency > 100 or < 0, use throughput-based calculation
if efficiency_2b > 100 || efficiency_2b < 0
    ...
end
```

**Issue:** If `E_in_2b < dE_stored` (net energy extracted from flywheel), the denominator becomes negative, causing negative efficiency before the sanity check catches it.

**Fix:** Add explicit check before division:
```matlab
if (E_in_2b - dE_stored) <= 0
    % Alternative calculation needed
    ...
else
    efficiency_2b = (E_out_2b / (E_in_2b - dE_stored)) * 100;
end
```

---

### 1.2 MAJOR

#### B3. Floating-Point Comparison for Optimal Design (Lines 934-935)

```matlab
[max_se, idx] = max(results.specific_energy(results.viable == 1));
[opt_i, opt_j] = ind2sub([n_mag, n_speed], ...
    find(results.specific_energy == max_se & results.viable == 1, 1));
```

**Issue:** Comparing floating-point numbers with `==` is unreliable due to precision errors.

**Fix:** Use proper indexing from the viable designs:
```matlab
viable_mask = results.viable == 1;
[viable_rows, viable_cols] = find(viable_mask);
se_viable = results.specific_energy(viable_mask);
[~, best_idx] = max(se_viable);
opt_i = viable_rows(best_idx);
opt_j = viable_cols(best_idx);
```

---

#### B4. Settling Time Calculation Error (Lines 566-572)

```matlab
ss_value = x_1d(end);
settling_idx = find(abs(x_1d - ss_value) > 0.02*peak_displacement*1e-6, 1, 'last');
```

**Issues:**
1. 2% settling criterion should be based on steady-state value, not peak displacement
2. The criterion mixes µm and m units (though mathematically cancels correctly)

**Standard Definition:**
```matlab
% 2% settling: |y(t) - y_ss| < 0.02 * |y_ss|
tolerance = 0.02 * abs(ss_value);
settling_idx = find(abs(x_1d - ss_value) > tolerance, 1, 'last');
```

---

#### B5. Energy Clipping Not Reflected in Energy Tracking (Lines 1057-1073)

```matlab
dE = -(P_grid + P_loss) * dt_2b;
E_curr_2b = E_curr_2b + dE;
E_curr_2b = max(E_min_opt, min(E_max_opt, E_curr_2b));  % Clipping happens here

if P_grid > 0
    E_out_2b = E_out_2b + P_grid * dt_2b;  % But tracking uses unclipped value
```

**Issue:** When energy is clipped (SoC at 0% or 100%), the `E_in_2b` and `E_out_2b` accumulators still count the full `P_grid * dt`, leading to energy accounting errors.

**Fix:** Track energy relative to actual state change:
```matlab
E_prev = E_curr_2b;
E_curr_2b = E_curr_2b + dE;
E_curr_2b = max(E_min_opt, min(E_max_opt, E_curr_2b));
actual_dE = E_curr_2b - E_prev;
% Use actual_dE for energy tracking
```

---

#### B6. SoC Display at Max Loss (Lines 323-324)

```matlab
fprintf('  Max total losses: %.2f kW at %.0f%% SoC\n', max(total_loss_1a)/1000, ...
    SoC_1a(total_loss_1a == max(total_loss_1a)));
```

**Issue:** `SoC_1a(...)` could return multiple values if there are ties, causing fprintf to fail or display incorrectly.

**Fix:**
```matlab
[max_loss, max_idx] = max(total_loss_1a);
fprintf('  Max total losses: %.2f kW at %.0f%% SoC\n', max_loss/1000, SoC_1a(max_idx));
```

---

### 1.3 MINOR

#### B7. Unused Variable `idx` (Line 934)

```matlab
[max_se, idx] = max(results.specific_energy(results.viable == 1));
```

The variable `idx` is never used. Should be `~` for clarity.

---

#### B8. Hardcoded Magic Number 450000 (Lines 829, 970)

```matlab
L_motor_min = 450000 / (torque_per_length * omega_max);
```

The value 450000 appears to be a target power in Watts (450 kW) but is not defined as a named constant, making the code less maintainable.

---

#### B9. Duplicate Code Blocks

The motor sizing calculations (lines 827-833) and flywheel sizing (lines 838-846) are repeated almost identically in Part 2a and Part 2b (lines 968-1004). This should be refactored into a function.

---

## 2. LOGIC/PHYSICS ISSUES

### 2.1 CRITICAL

#### P1. Torque Formula Discrepancy (Lines 175-176 vs 828-829)

**Part 1 uses:**
```matlab
torque_rated_bl = 2 * pi * rotor_radius_bl^2 * baseline.motor_length * shear_rated;
```

**Part 2 uses:**
```matlab
torque_per_length = shear * pi * d_shaft * (d_shaft/2);
% Which simplifies to: pi * d^2 / 2 * shear = 2 * pi * r^2 * shear
```

Both are equivalent and correct: T = τ × 2πr²L

However, **Part 2 uses `d_shaft` (shaft diameter) while Part 1 uses `rotor_radius_bl` (shaft + magnets)**. This inconsistency could lead to different torque values for the same geometry.

**Recommendation:** Verify whether torque acts at shaft surface or air gap (outer magnet surface). The physics suggests air gap (magnet outer surface) is correct.

---

### 2.2 MAJOR

#### P2. Tilting Mode Stiffness Conversion (Lines 620-621, 641-644)

```matlab
K_s_tilt = baseline.K_s * (baseline.L_amb/2)^2 * 2;
K_i_tilt = baseline.K_i * (baseline.L_amb/2) * 2;
...
stiffness_tilt(idx) = abs(denom_tilt) / (baseline.L_amb/2)^2;
```

**Issue:** The physics is unclear:
- `K_s_tilt` has units [N/m × m²] = [N·m] (rotational stiffness) - correct
- `K_i_tilt` has units [N/A × m] = [N·m/A] (rotational force constant) - correct
- Final division by `(L_amb/2)^2` converts back to linear stiffness at AMB location

However, the factor of 2 for "two AMBs" should only apply if both AMBs contribute equally to tilting, which depends on pivot point location.

**Recommendation:** Add comments explaining the tilting mode model and verify the geometric factors.

---

#### P3. Transverse Inertia Approximation (Lines 124-125)

```matlab
baseline.I_transverse = (1/12) * baseline.total_mass * ...
    (3*(baseline.flywheel_diameter/2)^2 + baseline.flywheel_length^2);
```

**Issue:** This formula is for a solid cylinder about its center perpendicular to the axis. The actual rotor is:
- A hollow composite flywheel
- Plus a solid steel shaft
- With magnets on the motor section

The formula ignores:
1. The hollow nature of the flywheel
2. Different densities of components
3. Parallel axis contributions if CM is offset

**Impact:** Tilting mode frequency and controller gains may be inaccurate.

---

#### P4. Surface Area for Radiation (Lines 217-240)

```matlab
% 5. Shaft end caps (two circular ends)
A_shaft_ends = 2 * pi * r_inner_bl^2;
```

**Issue:** The shaft passes through AMBs which are outside the vacuum chamber. The "end caps" of the shaft may not be exposed radiating surfaces - they might be:
- Inside the AMB housing
- Connected to external components
- Not at the same temperature as the rotor

**Recommendation:** Clarify which surfaces are actually inside the vacuum envelope.

---

### 2.3 MINOR

#### P5. Current Controller Pole-Zero Cancellation (Lines 1167-1168)

```matlab
Kp_current_new = L_coil_new * omega_bw_current;
Ki_current_new = Kp_current_new * R_coil_new / L_coil_new;
```

For pole-zero cancellation of the coil (L + R), the zero should match the pole:
- Plant pole at s = -R/L
- Controller zero at s = -Ki/Kp = -R/L ✓

This is correct, but `Ki = Kp × R/L = L×ω × R/L = R×ω`, which is very different from the baseline `Ki_current = 2149`. Verify this produces stable current loop behavior.

---

#### P6. AMB Force Rating for New Design (Lines 1147-1148)

```matlab
new_amb_force = 2 * optimal.total_mass * g;
```

**Issue:** Using `2 × m × g` gives 2× the weight as rated force. This is reasonable for static capacity but:
- Dynamic disturbances may require higher force
- The baseline uses 5780 N which is ~2× baseline weight

The assumption is consistent but should be documented.

---

## 3. FORMATTING ISSUES

### 3.1 MAJOR

#### F1. Inconsistent Section Delimiters

```matlab
%% ========================================================================   (most sections)
%% ########################################################################   (deliverable headers)
```

**Recommendation:** Use consistent delimiter style throughout.

---

#### F2. Magic Numbers Without Named Constants

Throughout the code, unexplained numeric values appear:
- `0.020` - 20 mm clearance (lines 106, 244, etc.)
- `0.38` - shaft extension beyond flywheel+motor (line 852)
- `450000` - target power? (lines 829, 970)
- `0.150`, `0.600` - motor length limits (lines 830, 970)
- `0.500`, `4.000` - flywheel length limits (line 846)

**Recommendation:** Define all constants at the top of the file with descriptive names.

---

### 3.2 MINOR

#### F3. Inconsistent Variable Naming

Mix of conventions:
- `snake_case`: `rotor_loss_1a`, `specific_power_bl`, `omega_max_opt`
- `camelCase`: `ambParams_temp`
- `SCREAMING_CASE`: not used for constants

**Recommendation:** Choose one convention and apply consistently.

---

#### F4. Hardcoded Figure Positions

```matlab
figure('Name', '...', 'Position', [100 100 1000 450]);
figure('Name', '...', 'Position', [100 100 1000 700]);
```

These will behave differently on different screen sizes/resolutions.

**Recommendation:** Use relative positioning or remove Position specification.

---

#### F5. Inconsistent Comment Placement

```matlab
baseline.flywheel_length = 1.000;      % Flywheel axial length
baseline.omega_max = baseline.max_speed_rpm * 2*pi / 60;  % [rad/s]
```

Some lines have aligned comments, others have inline comments at varying positions.

---

#### F6. Redundant Comments

```matlab
D_rotor_opt = d_shaft_opt + 2*t_mag_opt;  % Outer diameter of magnets
```

This same comment appears 4 times in the code (lines 261, 877, 1051, etc.). Consider defining this relationship once.

---

#### F7. fprintf Formatting Inconsistency

Some `fprintf` statements end with `\n\n`, others with `\n`, and some with nothing. Vertical spacing is inconsistent.

---

#### F8. Long Lines

Several lines exceed 100 characters (e.g., lines 107-109, 323-324), reducing readability.

---

#### F9. Missing Semicolons (None Found)

All statements correctly use semicolons - no output suppression issues.

---

#### F10. Incomplete Input Validation

The code assumes EE functions always return valid values. No checks for:
- NaN or Inf returns
- Negative values where positive expected
- Out-of-range inputs

---

## 4. RECOMMENDATIONS

### High Priority

1. **Fix efficiency calculations** (B1, B2) - These directly affect reported results
2. **Fix floating-point comparison** (B3) - Could select wrong optimal design
3. **Clarify torque radius** (P1) - Verify shaft vs air gap surface

### Medium Priority

4. **Fix energy tracking with clipping** (B5)
5. **Add input validation** for EE function outputs
6. **Refactor duplicate code** into functions
7. **Document tilting mode model** (P2)

### Low Priority

8. **Define named constants** for magic numbers
9. **Standardize formatting** (comments, delimiters, variable names)
10. **Add unit tests** for key calculations

---

## 5. CODE QUALITY METRICS

| Metric | Value | Rating |
|--------|-------|--------|
| Lines of code | 1461 | - |
| Comment density | ~25% | Good |
| Function count | 0 (single script) | Poor |
| Cyclomatic complexity | High (nested loops) | Fair |
| Documentation | Good (ASSUMPTIONS.md) | Good |

---

## Appendix: Files Reviewed

- `v2/flywheel_analysis_v2.m` (main script)
- `v2/ASSUMPTIONS.md` (documentation)

---

*End of Audit Report*
