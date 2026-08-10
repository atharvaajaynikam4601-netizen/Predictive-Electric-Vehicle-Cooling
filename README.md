# Predictive Electric Vehicle Cooling

Student project for the MathWorks Challenge Project: Predictive Electric Vehicle Cooling.

## Project aim

Design and evaluate a **forecast-aware battery cooling controller** that reduces cooling energy while keeping an EV battery pack within safe thermal limits. The controller will be compared with a conventional reactive controller under identical drive, ambient-temperature, and fast-charge conditions.

## Project architecture

1. **High-fidelity reference plant** — Simscape Battery and Simscape Fluids model of a liquid-cooled battery pack.
2. **Control-oriented plant** — reduced-order, lumped thermal model for rapid controller tuning and prediction.
3. **Controllers** — reactive thermostat/PID baseline, preview rule-based controller, and constrained MPC.
4. **Evaluation** — peak temperature, temperature uniformity, cooling energy, usable discharge/charge-power envelope, battery-energy impact, and constraint violations.

## Folders

- `models/` — Simulink models created for this project.
- `scripts/` — repeatable parameter, scenario, simulation, and plotting scripts.
- `data/` — drive-cycle and parameter data.
- `results/` — generated figures and tables; do not edit manually.
- `docs/` — engineering notes and final report material.

## Start here

In MATLAB, set the Current Folder to this project directory, then run:

```matlab
run("scripts/initializeProject.m")
run("scripts/createHotHillScenario.m")
```

The second command creates and plots our first reproducible validation scenario. It is an engineering test scenario, not a claim of real-road data.
