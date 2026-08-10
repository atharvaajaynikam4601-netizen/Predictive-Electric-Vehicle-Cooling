# Literature review plan

## What we must establish

1. Why lithium-ion battery temperature affects power capability, charging, and aging.
2. How liquid cold plates, radiators, and refrigerant chillers are represented in an EV thermal-management plant.
3. Why reactive cooling can be late for a thermally slow pack.
4. What predictive/MPC approaches already achieve and what practical issue this project tests.

## Core sources to read and cite

| Source | Use in this project |
|---|---|
| Huria, Ceraolo, Gazzarri, Jackey (2012), *High Fidelity Electrical Model with Thermal Dependence for Characterization and Simulation of High Power Lithium Battery Cells* | Electrical/thermal battery-model basis and the source used by the MathWorks reference model |
| MathWorks, *EV Battery Thermal Management System* | Reference transient Simscape Fluids architecture: battery packs, cold plate, radiator/refrigerant/heater, drive and charge cycles |
| MathWorks, *EV Battery Cooling System Design* | Heat-transfer checks and sizing logic for cold plate, radiator, and evaporator |
| Wang et al. (2024), *Modeling and Model Predictive Control of a Battery Thermal Management System Based on Thermoelectric Cooling for Electric Vehicles* | Comparison point for MPC versus conventional control; do not reuse its thermoelectric architecture |
| Sachs et al. (2025), *Predictive battery thermal management for fast charging of electric vehicles using nonlinear model predictive control and dynamic programming* | Fast-charge predictive-control context and coolant-loop modeling limitations |

## Intended contribution

This is not presented as inventing MPC. The contribution is a transparent, reproducible Simulink comparison of reactive and preview-based cooling for a **combined hot-day driving plus fast-charge event**, including prediction error and cooling-energy accounting. This makes the result more relevant to deployable vehicle supervisory control.
