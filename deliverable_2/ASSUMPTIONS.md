# Assumptions Documentation for Deliverable 2

This document details all assumptions made during the new flywheel design study, explains why each assumption was necessary, and identifies what additional information would eliminate the need for each assumption.

---

## 1. New Storage Cycle Definition

### Assumption Made
A new 6-hour storage cycle was created with the following characteristics:
- Duration: 21,600 seconds (6 hours)
- Base power amplitude: 75 kW (50% higher than baseline's 50 kW)
- Multi-frequency components simulating grid frequency regulation
- Sustained discharge peaks (simulating high demand periods)
- Recovery charging periods

### Why This Assumption Was Needed
The project specification mentions that each team has a unique storage cycle provided via Canvas, but the team-specific function (`team_16_cycle`) was not available in the provided function set. The `genericStorageCycle` function requires additional arguments that were not documented.

### Information Needed to Remove Assumption
- **Team-specific storage cycle function** (`team_16_cycle.p` or equivalent)
- **OR** documentation on `genericStorageCycle` input arguments
- **OR** explicit cycle profile data (power vs. time)

---

## 2. Design Variable Ranges

### Assumption Made
Design variable sweep ranges were selected as:
- Magnet thickness: 2-10 mm (9 values)
- Maximum rotational speed: 20,000-60,000 RPM (9 values)

### Why This Assumption Was Needed
The project specifies magnet thickness minimum (2 mm) but not maximum. Speed range must balance energy storage (higher speed = more energy) against tip speed limits and losses.

### Information Needed to Remove Assumption
- **Maximum practical magnet thickness** from EE team
- **Preferred speed range** based on motor/bearing design constraints
- **Commercial flywheel benchmarks** for typical operating speeds

---

## 3. Flywheel Sizing Methodology

### Assumption Made
Flywheel dimensions are determined by:
1. **Shaft diameter**: Limited by PM tip speed (175 m/s) and steel tip speed (175 m/s)
2. **Flywheel diameter**: Limited by composite tip speed (900 m/s)
3. **Flywheel length**: Sized to achieve target energy storage (~15 kWh)
4. **Motor length**: Sized to achieve minimum power (150 kW)

### Why This Assumption Was Needed
The project requires sizing "all dimensions of the flywheel system based on satisfying requirements" but doesn't specify which dimension should be optimized first or what power level to target.

### Information Needed to Remove Assumption
- **Target power specification** for the new cycle
- **Priority ranking** for design constraints (e.g., minimize size vs. maximize energy)
- **Manufacturing constraints** on component dimensions

---

## 4. Target Energy Storage

### Assumption Made
Target energy storage set to 15 kWh, which is approximately 50% higher than the baseline system's 9.92 kWh.

### Why This Assumption Was Needed
The new 6-hour cycle is more demanding than the baseline 15-minute cycle. Energy storage must be sufficient to:
- Start at 50% SoC
- Maintain SoC > 0% throughout the entire 6-hour cycle
- Handle sustained discharge periods without depletion

### Information Needed to Remove Assumption
- **Actual storage cycle profile** to calculate exact energy requirements
- **Minimum SoC margin** specification
- **Energy buffer requirements** for safety

---

## 5. Motor Sizing Power Target

### Assumption Made
Motor sized to achieve minimum 150 kW rated power, which is approximately 60% of the baseline's 249 kW.

### Why This Assumption Was Needed
The new cycle has higher base amplitude but longer duration. Peak power demands may differ from baseline. A lower power target was chosen to:
- Reduce losses (lower current)
- Allow for higher speeds (smaller motor diameter)
- Balance power vs. energy trade-off

### Information Needed to Remove Assumption
- **Peak power demand** from actual storage cycle
- **Power factor requirements** for grid regulation
- **Transient power specifications** for frequency response

---

## 6. Composite Objective Function

### Assumption Made
Optimal design selected using weighted composite score:
- Specific power: 30% weight
- Specific energy: 30% weight
- Efficiency: 25% weight
- Temperature margin: 15% weight

### Why This Assumption Was Needed
The project lists three objectives (minimize losses, maximize specific power, maximize specific energy) without specifying relative importance. A balanced weighting was chosen.

### Information Needed to Remove Assumption
- **Customer priority ranking** for performance metrics
- **Cost model** relating mass, power, and efficiency to value
- **Application-specific requirements** (e.g., portable vs. stationary)

---

## 7. Thermal Model

### Assumption Made
Same thermal model as Deliverable 1:
- Radiation-only heat transfer in vacuum
- Emissivity ε = 0.8
- Ambient temperature 20°C (293 K)
- Only rotor losses heat the rotor
- Steady-state temperature calculation

### Why This Assumption Was Needed
Consistency with baseline analysis. Vacuum operation eliminates convection. Stator cooling is external to vacuum chamber.

### Information Needed to Remove Assumption
- **Transient thermal analysis** requirements
- **Thermal mass and capacitance** of rotor
- **Vacuum chamber specifications**
- **Cooling system capabilities**

---

## 8. AMB Force Rating

### Assumption Made
AMB rated force = 2 × rotating group weight (as specified in project requirements). This scales with total mass for each design.

### Why This Assumption Was Needed
Directly from project specification: "Radial magnetic bearings should be selected so that each bearing has a rated force that is twice the rotating group weight."

### Information Needed to Remove Assumption
None - this is specified in the project requirements.

---

## 9. Shaft Length Estimation

### Assumption Made
Total shaft length estimated as:
- Flywheel length + Motor length + 4×axial clearance + 0.3 m (for AMBs and end clearances)

### Why This Assumption Was Needed
Complete assembly drawing not provided. Shaft extends beyond flywheel to accommodate AMBs, sensors, and coupling.

### Information Needed to Remove Assumption
- **Detailed assembly drawing** with AMB positions
- **AMB axial dimensions** from ambParameters function
- **End coupling/mounting requirements**

---

## 10. AMB Rotor Mass Fraction

### Assumption Made
AMB rotor components (target laminations, sensors) estimated as 10% of shaft mass.

### Why This Assumption Was Needed
Same assumption as Deliverable 1 for consistency. AMB rotor component specifications not provided.

### Information Needed to Remove Assumption
- **AMB rotor component specifications**
- **Target lamination dimensions and material**
- **Sensor rotor component details**

---

## 11. Design Viability Criteria

### Assumption Made
A design is considered viable if:
- Maximum temperature < 100°C
- Minimum SoC during cycle > 0%
- Cycle efficiency between 80% and 100%

### Why This Assumption Was Needed
Temperature limit from project specifications. SoC requirement from deliverable 2 description. Efficiency range for physical reasonableness.

### Information Needed to Remove Assumption
- **Minimum acceptable efficiency** specification
- **SoC margin requirements** (e.g., must stay above 5%?)
- **Other operational constraints** (e.g., current limits, voltage limits)

---

## 12. Rated Current Definition

### Assumption Made
Rated current = 0.8 per-unit (same as Deliverable 1), representing continuous operation below maximum capability.

### Why This Assumption Was Needed
Provides thermal margin for continuous operation. Allows for transient peaks up to 1.0 pu.

### Information Needed to Remove Assumption
- **Motor thermal rating** specification
- **Duty cycle requirements**
- **Peak vs. continuous current limits**

---

## Summary Table

| # | Assumption | Impact | Information Needed |
|---|------------|--------|-------------------|
| 1 | New storage cycle profile | Cycle simulation results | Team-specific cycle function |
| 2 | Design variable ranges | Design space exploration | Practical limits from EE team |
| 3 | Flywheel sizing methodology | All derived dimensions | Priority ranking for constraints |
| 4 | Target energy (15 kWh) | Flywheel size | Actual cycle energy requirements |
| 5 | Motor power target (150 kW) | Motor size and losses | Peak power from cycle |
| 6 | Objective weights | Optimal design selection | Customer priorities |
| 7 | Thermal model (radiation) | Temperature predictions | Thermal system details |
| 8 | AMB force rating (2×weight) | AMB sizing | (Specified in project) |
| 9 | Shaft length estimate | Mass and inertia | Assembly drawing |
| 10 | AMB rotor mass (10%) | Total mass | AMB component specs |
| 11 | Viability criteria | Design filtering | Operational requirements |
| 12 | Rated current (0.8 pu) | Power and loss calculations | Motor rating specifications |

---

## Impact Assessment

### Critical Assumptions (High Impact)
1. **New storage cycle** - Directly affects all design requirements and results
2. **Design variable ranges** - Determines feasible design space
3. **Objective function weights** - Determines optimal design selection

### Moderate Assumptions (Medium Impact)
4. Target energy storage
5. Motor power target
6. Viability criteria

### Minor Assumptions (Low Impact)
7. Thermal model (consistent with baseline)
8. Shaft length estimation
9. AMB rotor mass fraction
10. Rated current definition

---

## Design Study Methodology

The design study follows this process:
1. Define design variable grid (magnet thickness × max speed)
2. For each design point:
   - Calculate shaft diameter from tip speed limits
   - Calculate flywheel diameter from composite tip speed
   - Size motor length for target power
   - Size flywheel length for target energy
   - Calculate total mass and inertia
   - Simulate storage cycle
   - Calculate efficiency and temperature
   - Check viability constraints
3. Evaluate composite objective for viable designs
4. Select optimal design from Pareto front

This systematic approach ensures all design constraints are satisfied while exploring trade-offs in the multi-objective design space.

---

## Recommendations

To improve the design study accuracy:
1. **Obtain team-specific storage cycle** from Canvas
2. **Validate thermal model** against any available test data
3. **Perform sensitivity analysis** on objective weights
4. **Consider additional constraints** (cost, manufacturability)
5. **Iterate design** based on Deliverable 3 AMB analysis results
