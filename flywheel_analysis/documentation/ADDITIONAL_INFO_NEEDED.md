# Additional Information Needed

## CRITICAL - Required for Analysis

### 1. Appendix A - Baseline System Specifications
**Priority**: CRITICAL
**Location**: Should be in project document (currently missing)

**Required Parameters**:
- Maximum rotational speed [RPM or rad/s]
- Rated power output [kW or W]
- Energy storage capacity [kWh or J]
- Component dimensions:
  - Shaft inner and outer diameter
  - Motor/generator diameter and length
  - Flywheel inner diameter, outer diameter, and thickness
  - AMB locations and dimensions
- Magnet thickness [mm]
- Motor pole configuration (number of poles)
- AMB force ratings
- Any other specified baseline design parameters

**Action**: Check project PDF appendices or Canvas for Appendix A

**Impact**: Without this, all baseline analysis uses assumed values that may be incorrect.

**Where to update**:
- `baseline_analysis/baseline_config.m` (lines marked "ASSUMED - REPLACE WITH APPENDIX A")

---

### 2. New Storage Cycle Data (Team #16 Specific)
**Priority**: CRITICAL
**Location**: Canvas - shared with your team

**Required Format**:
- Time vector [s] and Power vector [W]
- OR: CSV/Excel file with columns [time, power]
- OR: Power profile description (piecewise constant, ramp rates, etc.)

**Typical Format**:
```
Time [s],  Power [W]
0,         0
60,        100000
120,       100000
180,       -50000
...
```

**Action**: Download from Canvas, Team #16 assignment area

**Impact**: Placeholder cycle may not represent actual requirements. Design may be under/over-sized.

**Where to update**:
- `new_design/design_optimization.m` (lines marked "PLACEHOLDER - Replace with actual")
- Can load from CSV: `data = readmatrix('team16_cycle.csv');`

---

### 3. Electrical Engineering Team MATLAB Functions
**Priority**: CRITICAL
**Location**: Canvas - Course files or Project 3 materials

**Required Files**:
- `magneticShear.m`
- `rotorLosses.m`
- `statorLosses.m`
- `ambParameters.m`

**Action**: Download from Canvas and replace placeholder files in `ee_functions/` folder

**Impact**: Loss calculations, torque capability, and AMB properties are currently using simplified models. Actual functions will have accurate magnetic FEA results.

**Where to update**:
- Replace entire files in `flywheel_analysis/ee_functions/`
- Run `help magneticShear` (etc.) to check units and ensure correct usage

---

## IMPORTANT - Improves Accuracy

### 4. AMB Bearing Catalog/Database
**Priority**: High
**Source**: Electrical engineering team or manufacturer data

**Information Needed**:
- Available AMB sizes (diameter ranges)
- Force rating vs. size
- Current requirements
- Electrical parameters (L, R) vs. size
- Axial length vs. force rating

**Current Status**: Using parametric model in `ambParameters.m`

**Impact**: AMB sizing may not match available products

---

### 5. Motor/Generator Efficiency Map
**Priority**: Medium
**Source**: Electrical engineering team

**Information Needed**:
- Efficiency vs. speed and torque
- OR: Efficiency vs. speed and power
- Maximum current limits vs. speed

**Current Status**: Using constant 95% efficiency assumption

**Impact**: Energy balance and loss calculations during charge/discharge may be inaccurate

---

### 6. Thermal Boundary Conditions
**Priority**: Medium

**Information Needed**:
- Vacuum chamber wall temperature [°C]
- Stator cooling method (if any):
  - Natural convection to ambient?
  - Water cooling?
  - Heat sink design?
- External heat transfer coefficient (if not in vacuum)

**Current Status**:
- Assuming vacuum chamber at 25°C
- Radiation-only heat transfer

**Impact**: Temperature predictions may be off by 10-20°C

---

### 7. Mass Imbalance Specification
**Priority**: Low to Medium

**Information Needed**:
- Balancing grade (ISO 1940): G0.4, G1, G2.5, G6.3?
- OR: Specified maximum imbalance [g·mm or kg·m]
- OR: Maximum allowable runout [μm]

**Current Status**: Using 1 g·m imbalance (moderate)

**Impact**: Rotor runout predictions affected. Does not impact main design.

---

### 8. Control Performance Requirements
**Priority**: Medium

**Information Needed**:
- Required AMB position loop bandwidth [Hz]
- Maximum allowable position error [μm]
- Settling time requirement [ms]
- Overshoot limit [%]

**Current Status**: Using 50 Hz bandwidth, 0.7 damping ratio

**Impact**: Controller may be under/over-designed

---

### 9. Cost Information (Optional for Design Selection)
**Priority**: Low (unless cost optimization required)

**Information Needed**:
- Composite material cost [$/kg]
- Steel cost [$/kg]
- Magnet cost [$/kg]
- AMB cost vs. force rating
- Motor cost vs. power rating

**Current Status**: Not included in optimization

**Impact**: Optimal design selection currently ignores cost

---

## NICE TO HAVE - Validation and Refinement

### 10. Experimental Data from Baseline System
- Measured losses at different speeds
- Measured temperatures during operation
- Measured AMB current and position signals
- Vibration measurements

**Use**: Validate models before applying to new design

---

### 11. Manufacturing Constraints
- Minimum/maximum shaft diameter
- Minimum/maximum flywheel outer diameter
- Available composite materials and properties
- Available magnet grades

**Use**: Constrain design space to manufacturable designs

---

### 12. Rotor Dynamics Information
- Critical speeds (if known from baseline)
- Damping ratios for structural modes
- Shaft material and properties

**Use**: Check for resonances, refine dynamic model

---

## Summary Table

| Item | Priority | Status | Impact | Location to Update |
|------|----------|--------|--------|-------------------|
| Appendix A | CRITICAL | Missing | High | `baseline_config.m` |
| Team #16 Cycle | CRITICAL | Missing | High | `design_optimization.m` |
| EE Functions | CRITICAL | Placeholder | High | `ee_functions/*.m` |
| AMB Catalog | High | Model-based | Medium | `ambParameters.m` |
| Motor Efficiency | Medium | Constant 95% | Medium | `simulateStorageCycle.m` |
| Thermal BCs | Medium | Assumed | Medium | `calculateRotorTemperature.m` |
| Imbalance Spec | Low-Med | Assumed 1 g·m | Low | `analyze_baseline.m` |
| Control Specs | Medium | Assumed | Low | `design_amb_controller.m` |
| Cost Data | Low | N/A | Low | Optional |

---

## How to Obtain Missing Information

1. **Check Project PDF**: Look for all appendices (A, B, C, etc.)

2. **Canvas Course Page**:
   - Files → Project 3 folder
   - Look for:
     - "Appendix_A.pdf" or similar
     - "Team_16_Storage_Cycle.csv" or similar
     - "EE_Functions.zip" or individual .m files

3. **Email Course Instructors**: M. Anderson & E. Severson
   - Ask specifically for missing Appendix A
   - Confirm where Team #16 storage cycle is posted

4. **Check with Team Members**:
   - Pokhrel, Nesha
   - Zhang, Zhihan
   - Rios, Madigan
   - One of them may have already downloaded the files

5. **Office Hours**: Bring specific questions about missing data

---

## What You Can Do Now (Without Missing Data)

The current implementation allows you to:

1. ✅ Understand the overall analysis framework
2. ✅ Test all scripts with placeholder data
3. ✅ Verify code runs without errors
4. ✅ Generate example plots and results
5. ✅ Understand the design trade-offs qualitatively
6. ✅ Prepare report structure and outline

Once you obtain the actual data, updates will be straightforward:
- Replace configuration parameters
- Replace EE function files
- Re-run scripts
- Update plots in report

The analysis framework is complete and ready for the real data!
