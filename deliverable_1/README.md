# Deliverable 1: Baseline Flywheel System Analysis

## Overview

This deliverable contains a comprehensive analysis of the baseline flywheel energy storage system developed by eXtreme Storage Inc. The analysis characterizes the performance of the existing system to establish a benchmark for future design improvements.

## Contents

- `baseline_analysis.m` - Main MATLAB script performing all analyses
- `README.md` - This documentation file
- `ASSUMPTIONS.md` - Detailed assumptions, justifications, and information needs
- Output figures (generated when script runs):
  - `part1a_losses_temperature.fig/.png`
  - `part1c_storage_cycle.fig/.png`
  - `part1d_amb_step_response.fig/.png`
  - `part1e_dynamic_stiffness.fig/.png`
  - `part1f_rotor_runout.fig/.png`

## What the Script Does

### Part 1a: Losses and Temperature vs State of Charge
- Calculates electromagnetic losses (rotor and stator) across the full SoC range (0-100%)
- Operates at rated power (calculated from maximum motor capability)
- Computes rotating group temperature using radiation heat transfer model
- **Outputs**: Two-panel plot showing losses and temperature vs SoC

### Part 1b: Specific Power and Specific Energy
- Calculates total rotating group mass (flywheel, shaft, magnets, AMB rotors)
- Determines moment of inertia for energy storage calculations
- Computes specific power (kW/kg) based on rated power output
- Computes specific energy (Wh/kg) from kinetic energy storage capacity
- **Outputs**: Console display of performance metrics

### Part 1c: Storage Cycle Efficiency
- Simulates the provided 15-minute baseline storage cycle
- Starts simulation at 50% SoC as specified
- Tracks energy flow to/from grid and self-discharge losses
- Calculates roundtrip efficiency including energy recovery requirements
- **Outputs**: Three-panel plot showing grid power, SoC variation, and losses over time; efficiency percentage

### Part 1d: AMB Step Response
- Designs a PD position controller for the Active Magnetic Bearing system
- Uses the provided current controller transfer function
- Simulates response to a step force disturbance (10% of rated AMB force)
- Analyzes system at two conditions: zero speed and 100% SoC (maximum speed)
- **Outputs**: Six-panel plot showing current, force, and rotor orbit for both conditions

### Part 1e: Dynamic Stiffness
- Performs frequency domain analysis of the AMB system (1-1000 Hz)
- Calculates dynamic stiffness magnitude vs frequency
- Evaluates two modes:
  - Radial direction (single AMB translation)
  - Tilting direction (coupled AMB rotation)
- **Outputs**: Log-log plot of stiffness vs frequency for both modes

### Part 1f: Rotor Runout
- Models mass imbalance effects on rotor displacement
- Uses ISO G2.5 balance grade (precision rotor standard)
- Calculates runout amplitude as function of rotational speed/SoC
- **Outputs**: Plot of runout displacement vs SoC

## How to Run

### Prerequisites
1. MATLAB R2020a or newer recommended
2. Required EE team functions must be in MATLAB path:
   - `magneticShear.m`
   - `rotorLosses.m`
   - `statorLosses.m`
   - `ambParameters.m`
   - `baselineStorageCycle.m`

### Execution Steps
1. Navigate to the Project3 directory
2. Ensure all EE functions are accessible (script adds parent directory to path)
3. Run the script:
   ```matlab
   cd deliverable_1
   baseline_analysis
   ```
4. Review console output for numerical results
5. Check generated figures for visual analysis

### Expected Runtime
Approximately 30-60 seconds depending on system performance. Most time is spent on the storage cycle simulation (900 time steps) and AMB response analysis.

## Key Results

The script provides comprehensive baseline characterization:

- **Performance Metrics**: Specific power and energy density for comparison with new designs
- **Thermal Analysis**: Temperature rise under rated power operation
- **Efficiency**: Roundtrip cycle efficiency accounting for all losses
- **Stability**: AMB controller design and dynamic response verification
- **Mechanical**: Rotor runout predictions for quality assessment

## Output Interpretation

### Losses and Temperature (Part 1a)
- Higher SoC (higher speed) generally produces more losses due to increased electromagnetic effects
- Temperature must remain below 100°C safety limit
- Provides operating envelope for thermal management

### Storage Cycle (Part 1c)
- SoC variation shows energy buffering capability
- Efficiency indicates energy cost of frequency regulation service
- Loss profile shows dominant loss mechanisms during operation

### AMB Response (Part 1d)
- Fast settling time indicates good disturbance rejection
- Small orbit indicates stable levitation
- Comparison between 0 RPM and max speed shows gyroscopic effects

### Dynamic Stiffness (Part 1e)
- Low-frequency stiffness determines static load capacity
- High-frequency rolloff shows inertial effects
- Resonance peaks (if present) indicate critical speeds

### Rotor Runout (Part 1f)
- Runout should be much smaller than AMB clearance
- Speed-dependent behavior from unbalance dynamics
- Critical speed identification for safe operation

## Notes

- The script is thoroughly commented for educational purposes
- All major calculations include physical units in comments
- Figures are automatically saved in both .fig and .png formats
- Console output provides detailed progress tracking
- See ASSUMPTIONS.md for all modeling assumptions

## Troubleshooting

**Error: "Undefined function 'magneticShear'"**
- Ensure EE functions are in the MATLAB path
- Check that function files are in the parent directory or add their location manually

**Warning: "Division by zero" or "Invalid result"**
- Check that all parameters are defined correctly
- Verify that speed ranges are valid (positive values)

**Figures not saving**
- Ensure write permissions in deliverable_1 folder
- Check available disk space

## Contact

For questions about this analysis, refer to:
- Project documentation (Project03_FlywheelEnergyStorageSystem.pdf)
- Appendices A and B for specifications
- Course instructor or TA
