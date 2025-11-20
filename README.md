# Flywheel Energy Storage System Analysis - Project 3

**Course:** Mechanical Engineering Modeling  
**Project:** Baseline and New Design Analysis for eXtreme Storage Inc.

## Project Overview

This project involves analyzing and optimizing a flywheel energy storage system for grid frequency regulation. The analysis is divided into three deliverables:

1. **Deliverable 1:** Baseline system characterization
2. **Deliverable 2:** New design development
3. **Deliverable 3:** Final design optimization and comparison

## Repository Structure

```
Project3/
├── deliverable_1/              # Deliverable 1: Baseline Analysis
│   ├── baseline_analysis.m     # Main analysis script
│   ├── README.md              # Deliverable 1 documentation
│   ├── ASSUMPTIONS.md         # Detailed assumptions and justifications
│   └── *.fig/*.png           # Generated plots
│
├── Project3_Functions/         # EE Team Functions (P-code)
│   ├── magneticShear.p
│   ├── rotorLosses.p
│   ├── statorLosses.p
│   ├── ambParameters.p
│   ├── baselineStorageCycle.p
│   ├── genericStorageCycle.p
│   └── elecMachineParams.p
│
└── Project03_*.pdf            # Project documentation and appendices

```

## Deliverable 1: Baseline System Analysis

### What It Does

Comprehensive characterization of the existing flywheel system including:
- **1a.** Losses and temperature analysis across state of charge (SoC)
- **1b.** Specific power and specific energy calculations
- **1c.** Storage cycle efficiency for 15-minute frequency regulation
- **1d.** Active Magnetic Bearing (AMB) step response analysis
- **1e.** Dynamic stiffness frequency response
- **1f.** Rotor runout due to mass imbalance

### Key Results (Baseline System)

| Metric | Value | Status |
|--------|-------|--------|
| Specific Power | 0.833 kW/kg | ✅ Excellent |
| Specific Energy | 33.2 Wh/kg | ✅ Good |
| Total Storage | 9.92 kWh | ✅ |
| Rated Power | 248.7 kW | ✅ |
| Cycle Efficiency | 97.4% | ✅ Excellent |
| Max Temperature | 63.8°C | ✅ Safe (< 100°C) |
| Rotor Runout | 0.6-1.2 µm | ✅ Sub-micron |

### How to Run

```matlab
cd deliverable_1
baseline_analysis
```

**Prerequisites:**
- MATLAB R2020a or newer
- Project3_Functions in parent directory

## System Specifications

### Baseline System (from Appendix B)
- **Flywheel:** 430mm diameter × 1000mm length, composite (1600 kg/m³)
- **Max Speed:** 40,000 RPM
- **Speed Range:** 20,000 - 40,000 RPM (0-100% SoC)
- **Shaft/PM Diameter:** 84mm
- **Magnet Thickness:** 6mm
- **Motor Length:** 250mm
- **AMB Rated Force:** 5,780 N
- **Max Safe Temperature:** 100°C

### Material Limits
- Max composite tip speed: 900 m/s
- Max steel tip speed: 175 m/s
- Max PM tip speed: 175 m/s

## Documentation

- **Deliverable 1 README:** `deliverable_1/README.md` - Detailed script documentation
- **Assumptions:** `deliverable_1/ASSUMPTIONS.md` - All modeling assumptions with justifications
- **Project Spec:** `Project03_FlywheelEnergyStorageSystem - Google Docs.pdf`
- **Appendices:** `Project03_*_Appendix*.pdf`

## Notes

- EE functions are provided as P-code (compiled MATLAB) by the electrical engineering team
- All functions use **per-unit current** (0-1.0 range), not absolute amperes
- Thermal model uses radiation-only heat transfer (vacuum environment)
- Only rotor losses heat the rotor; stator is cooled separately

## Future Work

- Deliverable 2: Design new flywheel system meeting performance targets
- Deliverable 3: Optimize and compare final design to baseline

## Author

[Your Name]  
Mechanical Engineering Modeling  
Fall 2025
