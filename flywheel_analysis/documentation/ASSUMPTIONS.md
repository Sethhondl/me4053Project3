# Assumptions Made in Flywheel Energy Storage Analysis

## Baseline System Assumptions (MUST BE REPLACED WITH APPENDIX A)

Since Appendix A specifications were not provided, the following assumptions were made for the baseline system:

### Operating Parameters
- **Maximum speed**: 30,000 RPM (3142 rad/s)
- **Rated power**: 100 kW
- **Usable energy storage**: 1.39 kWh (5 MJ)
- **Magnet thickness**: 4 mm

**ACTION REQUIRED**: Replace these with actual Appendix A values in `baseline_analysis/baseline_config.m`

### Geometric Assumptions
- **Shaft inner radius**: 10 mm (hollow shaft)
- **Motor/generator axial length**: 80 mm (initial estimate)
- **AMB axial length**: 30 mm per bearing
- **Motor rotor back iron thickness**: 3 mm
- **Air gap (stator-rotor)**: 5 mm

### Material Properties
Used values from Table 1:
- Composite flywheel density: 1600 kg/m³
- Steel density: 7850 kg/m³
- Permanent magnet density: 7850 kg/m³

### Environmental Conditions
- **Ambient temperature**: 25°C
- **Operating in vacuum**: Minimal convection heat transfer
- **Emissivity of steel**: 0.6 (oxidized)
- **Emissivity of composite**: 0.9

### Storage Cycle (Baseline)
- **Cycle duration**: 1 hour
- **Profile**: 15 min charge, 30 min standby, 15 min discharge
- **Starting SoC**: 50%

**ACTION REQUIRED**: This is a placeholder. Use actual storage cycle if provided.

## New Storage Cycle Assumptions (MUST BE REPLACED)

The new storage cycle used a 2-hour placeholder profile:
- Variable power charge/discharge pattern
- Maximum power: 200 kW

**ACTION REQUIRED**: Replace with actual Team #16 storage cycle from Canvas in `new_design/design_optimization.m`

## AMB System Assumptions

### Load Distribution
- Equal load distribution between top and bottom radial AMBs
- Each AMB rated for 2× total rotating group weight

### Control Architecture
- Cascaded position and current control
- Inner current loop bandwidth: 10× outer position loop
- Position controller: PD (Proportional-Derivative)
- Current controller: PI (Proportional-Integral)

### Controller Design Targets
- **Position loop bandwidth**: 50 Hz (assumed reasonable for this application)
- **Damping ratio**: 0.7 (critically damped)
- **Current loop bandwidth**: 500 Hz

**ACTION REQUIRED**: Adjust based on system requirements and stability analysis

### Imbalance Assumptions
- **Mass imbalance**: 1 g·m (mass × eccentricity) for runout calculations
- This is a typical value for balanced rotors

**ACTION REQUIRED**: Adjust based on balancing grade requirements (ISO 1940)

## Thermal Analysis Assumptions

### Heat Transfer Model
- **Primary heat transfer**: Radiation (vacuum environment)
- **Conduction through AMBs**: Negligible (air gap)
- **Stefan-Boltzmann constant**: 5.67×10⁻⁸ W/(m²·K⁴)

### Simplified Thermal Network
1. Rotor losses → Rotor body → Radiation to stator/housing
2. Stator losses → Stator body → Radiation to housing
3. Quasi-static thermal analysis (not transient)

### Surface Area Calculations
- Flywheel dominates rotor radiating area
- Includes top, bottom, and outer cylindrical surfaces
- Stator area based on motor outer diameter

**LIMITATION**: This is a simplified model. Actual system may have:
- Conduction through shaft
- Heat sinks on stator
- Active cooling in stator housing

## Electrical Machine Assumptions

### Motor/Generator Type
- **Type**: Permanent magnet synchronous machine (PMSM)
- **Pole pairs**: 2 (4-pole machine assumed)
- **Efficiency**: 95% (simplified for cycle simulation)

### Loss Models (PLACEHOLDERS)
The electrical engineering team functions are placeholders with simplified physics:
- Rotor losses: Eddy currents + windage + harmonic losses
- Stator losses: Copper (I²R) + core losses (Steinmetz equation)
- Magnetic shear stress: Simplified linear relationship

**ACTION REQUIRED**: Replace placeholder functions with actual functions from Canvas:
- `ee_functions/magneticShear.m`
- `ee_functions/rotorLosses.m`
- `ee_functions/statorLosses.m`
- `ee_functions/ambParameters.m`

## Structural and Safety Assumptions

### Tip Speed Limits (from Table 1)
- Steel shaft: 175 m/s
- Permanent magnets: 175 m/s
- Composite flywheel: 900 m/s

These are **hard constraints** in the design optimization.

### Clearances (from Table 1)
- Axial clearance between components: 20 mm
- Radial clearance (flywheel to housing): 20 mm
- Radial clearance (other components): 1 mm

### Temperature Limit (from Table 1)
- Maximum rotating group temperature: 100°C

This is enforced as a **constraint** in design feasibility.

## Simplifications and Limitations

### 1. Rigid Body Dynamics
- No shaft bending modes
- No gyroscopic coupling (simplified)
- Symmetric bearing properties

### 2. Control System
- Linear time-invariant (LTI) control
- No saturation effects in main analysis
- No sensor dynamics modeled

### 3. Power Electronics
- Ideal voltage/current sources
- No switching losses
- No PWM effects

### 4. Mechanical
- No bearing compliance (AMBs are stiff)
- No structural damping
- Perfect balancing except for specified imbalance

### 5. Vacuum Chamber
- Perfect vacuum (no windage on flywheel)
- Chamber wall at ambient temperature
- No outgassing or contamination

## Statistical/Uncertainty Considerations

The following were **not** considered in this analysis:
- Manufacturing tolerances
- Material property variations
- Temperature-dependent properties (simplified)
- Aging and degradation
- Sensor noise and uncertainties

For a production design, these should be included via:
- Monte Carlo analysis
- Worst-case analysis
- Robust design optimization

## Design Space Exploration Assumptions

### Optimization Ranges
- **Speed**: 20,000 - 50,000 RPM (8 points)
- **Magnet thickness**: 2 - 8 mm (6 points)
- Total: 48 design points evaluated

**ACTION REQUIRED**: Adjust ranges based on:
- Power requirements from actual storage cycle
- Manufacturing capabilities
- Cost constraints

### Objective Weights
- Specific power: 30%
- Specific energy: 30%
- Efficiency: 40%

**ACTION REQUIRED**: Adjust weights based on eXtreme Storage Inc priorities

## Model Validation

**IMPORTANT**: None of these models have been validated against:
- Experimental data
- High-fidelity FEA
- Manufacturer specifications

The results should be considered **preliminary estimates** suitable for:
- Design space exploration
- Trade-off studies
- Proof-of-concept analysis

For final design, require:
- Validation of electrical machine models
- FEA of AMB system
- Thermal CFD analysis
- Rotor dynamics analysis with commercial software
