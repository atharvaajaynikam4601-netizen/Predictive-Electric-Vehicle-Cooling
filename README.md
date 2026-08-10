# Project Number - 194: Predictive Electric Vehicle Cooling: Simscape Fluids Battery Thermal Plant with Reactive, Lookahead-Predictive, and Nonlinear MPC Controllers in MATLAB/Simulink

## Abstract

This project is a solution to the **MathWorks MATLAB & Simulink Challenge Project Hub** brief *"Predictive Electric Vehicle Cooling"* — a call to replace slow-reacting thermostatic battery cooling with a controller that anticipates thermal load from driving and charging behaviour before the battery overheats.

The solution is built in two coupled layers, entirely in **MATLAB and Simulink**:

- A **control-oriented MATLAB plant** — vehicle longitudinal dynamics, motor/inverter efficiency, a coulomb-counting battery electrical model, and a lumped ohmic + entropic battery thermal model — used to generate realistic heat-load, current, and ambient-temperature signals and to rapidly prototype and benchmark controllers.
- A **high-fidelity Simscape Fluids liquid-cooling plant** (cold plate, coolant pump, radiator, reservoir, thermal liquid properties) driven by the heat-load signal above, used to validate controller behaviour against a physical thermal-fluid network rather than a lumped approximation.

Three controller architectures are implemented and compared against an uncooled reference and each other:

1. **Reactive baseline** — fixed-threshold hysteresis (thermostat) controller.
2. **Predictive lookahead controller** — rule-based pre-cooling triggered by a heat-generation forecast horizon, telematics-style fast-charger proximity/time-to-arrival, and throttle-demand lookahead.
3. **Constrained nonlinear MPC** — receding-horizon `fmincon` controller that minimises thermal error, cooling energy, and actuator slew rate subject to a hard 35 °C safety constraint.

A Kalman filter estimates battery SOC from noisy terminal-voltage measurements, and an Arrhenius SEI-growth model is used to translate temperature exposure into relative battery-aging and range impact — directly addressing the challenge's "advanced work" ask (range and battery-lifespan estimation).

---

# Contents

1. [Introduction](#1-introduction)
2. [Mapping to the MathWorks Challenge Brief](#2-mapping-to-the-mathworks-challenge-brief)
3. [System Modeling Architecture](#3-system-modeling-architecture)
4. [Mathematical Modeling](#4-mathematical-modeling)
5. [MATLAB & Simulink Implementation Methodology](#5-matlab--simulink-implementation-methodology)
6. [Simulation Results and Performance Metrics](#6-simulation-results-and-performance-metrics)
7. [Debugging of Code](#7-debugging-of-code)
8. [Repository Structure and How to Run](#8-repository-structure-and-how-to-run)
9. [Status and Next Steps](#9-status-and-next-steps)
10. [Learning Outcomes](#10-learning-outcomes)
11. [References](#11-references)
12. [Setup, Dependencies, and Contact](#12-setup-dependencies-and-contact)

---

# 1 Introduction

## 1.1 Objective

The objective is to demonstrate that a controller which **forecasts battery heat load** — rather than reacting to a temperature threshold after the fact — can hold an EV battery pack inside a safe thermal envelope with fewer, shorter, or better-timed cooling interventions than a conventional thermostat, across realistic urban driving, aggressive acceleration, and DC fast-charging conditions.

## 1.2 Tools and Libraries Used

**MATLAB**

- `readtable()` — import the UDDS drive-cycle dataset
- `interp1()` / `griddedInterpolant()` — SOC–OCV and 2-D SOC/temperature internal-resistance lookups
- `trapz()` / `cumtrapz()` — energy and distance integration, route-position tracking for telematics
- `fmincon()` (Optimization Toolbox, `sqp` algorithm) — receding-horizon MPC solve
- `timeseries()` — export of heat load, speed, SOC, and ambient signals into Simulink `From Workspace` blocks

**Simulink / Simscape Fluids**

- Controlled Heat Flow Rate Source, Thermal Mass, and Temperature Sensor blocks representing the battery thermal node
- Pump (Flow Rate Source), Cold Plate Pipe, Radiator Pipe, Reservoir, and Thermal Liquid Properties (TL) blocks forming the coolant loop
- A controller block commanding coolant mass flow (`mdot_cmd`) from measured battery temperature

## 1.3 Input Datasets and Scenarios

| Source | Scenario | Duration | Purpose |
|---|---|---|---|
| `uddsdc.csv` (EPA UDDS drive cycle) | Urban stop-and-go driving | 1369 s | Realistic thermal-load and SOC generator feeding the Simscape Fluids plant; used for the validated baseline diagnostics in [Section 6.1](#61-validated-simulink-reactive-baseline-diagnostics-udds-scenario) |
| Synthetic multi-phase mission (`scripts/Dynamic_Loads.m`) | Aggressive accel/decel (0–600 s) → rest → 300 A DC fast-charge surge (700–1600 s) → post-charge soak, with ambient ramping 25 °C → 38 °C | 2000 s | Stress-tests all three controllers against a scenario that combines driving heat, charging heat, and rising ambient temperature in one run; used for the controller comparison in [Section 6.2](#62-matlab-based-multi-controller-comparison-stress-scenario) |

---

# 2 Mapping to the MathWorks Challenge Brief

The [Predictive Electric Vehicle Cooling](https://github.com/mathworks/MATLAB-Simulink-Challenge-Project-Hub/tree/main/projects/Predictive%20Electric%20Vehicle%20Cooling) brief asks for six things. This project addresses each as follows:

| Brief requirement | Implementation |
|---|---|
| Model the cooling system using Simscape Fluids | `models/EV_Predictive_Cooling_Plant.slx` — cold plate, pump, radiator, reservoir, thermal liquid properties network |
| Simulate dynamic loads (environment, fast charging, acceleration) | `scripts/Dynamic_Loads.m` — aggressive drive phase, 300 A CC-CV fast-charge surge, ambient ramp |
| Implement baseline and predictive controllers (charge rate, throttle, location) | Step 4 hysteresis baseline; Step 5 lookahead controller using throttle lookahead and charger-proximity telematics; Step 5 full nonlinear MPC |
| Demonstrate battery temperature stays within desired range | Section 6, temperature-vs-safety-threshold comparison plots |
| Calculate energy-efficiency gains, predictive vs. reactive | Cumulative chiller + pump electrical energy tracked per controller (Section 6.2) |
| Advanced work — range and battery-lifespan estimation | Arrhenius SEI-growth aging model and range-extension estimate in `scripts/Dynamic_Loads.m` (Section 4.9) |

---

# 3 System Modeling Architecture

The project uses a **two-plant approach**: a fast, control-oriented MATLAB model for scenario generation and controller design, and a high-fidelity Simscape Fluids model for physical validation. Both share the same battery electrical/thermal parameters and heat-generation signal.

```
UDDS drive cycle / synthetic multi-phase mission
            |
Vehicle Longitudinal Dynamics  (forces -> wheel power)
            |
Motor Torque-Speed & Regen Limits  (mechanical -> electrical power)
            |
Battery Electrical Model  (coulomb counting, SOC-OCV, 2D R_internal(SOC,T))
            |
Battery Heat Generation  (Q_ohmic + Q_entropic)
            |----------------------------------------------+
            |                                               |
Control-oriented lumped thermal plant              Simscape Fluids plant
(MATLAB, Dynamic_Loads.m)                          (models/*.slx: Thermal Mass +
            |                                       Cold Plate + Radiator + Pump)
   Reactive / Predictive / MPC controller                    |
            |                                       Controller-commanded mdot_cmd
   T_batt, Q_cooling, SOC_est (Kalman)                       |
            |                                       out.T_batt_log, out.mdot_cmd_log
Arrhenius aging + range-extension metrics
```

The Simscape Fluids plant exists in three variants under `models/`:

- **`EV_Predictive_Cooling_Plant.slx`** — closed-loop plant; coolant pump flow rate is commanded by a controller block reading measured battery temperature.
- **`EV_Predictive_Cooling_Plant_FixedHighFlow.slx`** — open-loop reference variant with a constant 0.05 kg/s coolant flow, used to isolate plant thermal behaviour from controller logic during debugging.
- **`EV_Predictive_Cooling_Plant_Reactive.slx`** — the reactive/baseline controller wired into the Simscape plant; source of the validated diagnostics in Section 6.1.

---

# 4 Mathematical Modeling

## 4.1 Vehicle Longitudinal Dynamics

The total traction force the motor must supply is the sum of rolling resistance, aerodynamic drag, and the force needed to accelerate the vehicle:

$$F_{total} = F_{rr} + F_{aero} + F_{acc}$$

$$F_{rr} = m\,g\,C_{rr} \qquad F_{aero} = \frac{1}{2}\,\rho\,C_d\,A\,v^2 \qquad F_{acc} = m_{eq}\,a$$

$$m_{eq} = m\,(1+\delta)$$

where $m$ is vehicle mass, $g$ gravity, $C_{rr}$ the rolling-resistance coefficient, $\rho$ air density, $C_d$ drag coefficient, $A$ frontal area, $v$ vehicle speed, and $a$ acceleration. $\delta = 0.08$ inflates the effective mass $m_{eq}$ to account for the rotational inertia of wheels, gears, and shafts. Acceleration is clamped to $-4 \le a \le 3\ \text{m/s}^2$ to remove differentiation noise from the drive-cycle speed trace.

## 4.2 Wheel Power and Motor Torque-Speed Envelope

$$P_{wheel} = \frac{F_{total}\,v}{\eta_{tire}} \qquad\qquad \omega_{base} = \frac{P_{motor,max}}{T_{motor,max}}$$

The motor can only deliver as much torque as its constant-torque / constant-power envelope allows:

$$T_{avail} = \begin{cases} T_{motor,max} & \omega \le \omega_{base} \quad \text{(constant-torque region)} \\ \dfrac{P_{motor,max}}{\omega} & \omega > \omega_{base} \quad \text{(constant-power / field-weakening region)} \end{cases}$$

Regenerative braking is capped at $T_{regen,max} = 120\ \text{Nm}$ and disabled below $v_{min,regen} = 1.5\ \text{m/s}$ to represent motor back-EMF limitations.

## 4.3 Motor and Inverter Efficiency

Drive and regen efficiency are modelled as quadratic functions of normalized torque $T_n$ and normalized speed $\omega_n$, clamped to physically realistic bands:

$$\eta_{drive} = 0.90 - 0.15\,(1-\omega_n)^2 - 0.20\,(T_n-0.6)^2, \qquad 0.75 \le \eta_{drive} \le 0.92$$

$$\eta_{regen} = 0.75 - 0.20\,(1-\omega_n)^2 - 0.30\,(T_n-0.5)^2, \qquad 0.55 \le \eta_{regen} \le 0.80$$

## 4.4 Battery Electrical Model

Open-circuit voltage comes from an 11-point SOC–OCV lookup table (`interp1`, 300–390 V over SOC 0–1). Internal resistance is a full **2-D interpolant** over SOC and temperature (`griddedInterpolant`, 5×5 grid, 0–55 °C, SOC 0.1–1.0), so resistance — and therefore heat generation — responds to both charge state and how hot the pack already is:

$$I_{batt} = \frac{P_{batt}}{V_{nom}} \qquad\qquad R_{int} = F_{Rint}(SOC,\,T_{batt}) \quad \text{(2-D lookup)}$$

$$V_{batt} = V_{oc}(SOC) - I_{batt}\,R_{int}$$

$$SOC_k = SOC_{k-1} - \frac{I_{batt}\,V_{nom}\,\Delta t}{3600\,C_{batt}}$$

## 4.5 Battery Heat Generation (Ohmic + Entropic)

Heat generation is split into an irreversible ohmic term and a reversible entropic term (Huria-style formulation), rather than the `I²R`-only approximation used in a purely resistive model:

$$Q_{ohmic} = I_{batt}^2\,R_{int}$$

$$Q_{entropic} = I_{batt}\,T_{K}\,\frac{dU}{dT}, \qquad \frac{dU}{dT} = -0.00022\ \text{V/K}$$

$$Q_{heat} = Q_{ohmic} + Q_{entropic}$$

## 4.6 Lumped Battery Thermal Plant

$$Q_{passive} = \frac{T_{batt} - T_{amb}}{R_{th}}$$

$$\frac{dT_{batt}}{dt} = \frac{Q_{heat} - Q_{passive} - Q_{cooling}}{C_{th}}$$

with $C_{th} = 45000\ \text{J/K}$ and $R_{th} = 1.5\ \text{K/W}$. The Simscape Fluids plant (Section 3) replaces this lumped equation with a physical Thermal Mass + Cold Plate Pipe + Radiator Pipe network for validation.

## 4.7 Controller 1 — Reactive Baseline (Hysteresis)

$$Q_{cooling} = \begin{cases} Q_{cooling,max}\ (2000\ \text{W}) & T_{batt} \ge T_{on}\ (35\,^\circ\text{C}) \\ 0 & T_{batt} \le T_{off}\ (32\,^\circ\text{C}) \\ \text{unchanged (hold last state)} & \text{otherwise} \end{cases}$$

This is a conventional thermostat: it cannot act until the battery has already crossed the trip point, and it cannot anticipate that heat is about to fall or rise.

## 4.8 Controller 2 — Predictive Lookahead Controller (with Telematics)

The predictive controller pre-cools *before* the temperature threshold is reached, using three lookahead signals:

Three forecast signals feed the decision logic:

$$d_{charger} = s_{charger} - s_{route}(t) \qquad \text{(route-position telematics)}$$

$$\text{incoming charge flag} = \big(d_{charger} > 0\big) \ \wedge\ \big(t_{charger} \le H_p\big)$$

$$\hat{\theta}_{throttle} = \max\big(\theta_{throttle}(t \,..\, t{+}15\text{s})\big) \qquad \text{(throttle-demand lookahead)}$$

$$\overline{Q}_{pred} = \text{mean}\big(Q_{heat}(t \,..\, t{+}H_p)\big), \qquad H_p = 120\ \text{s} \qquad \text{(heat-generation forecast)}$$

and the commanded cooling power is:

$$Q_{cooling} = \begin{cases} Q_{cooling,max} & \text{incoming charge flag} \ \wedge\ T_{batt} > 28\,^\circ\text{C} \quad \text{(charger pre-cool)} \\ 0.75\,Q_{cooling,max} & \hat{\theta}_{throttle} > 50\ \wedge\ T_{batt} > 29\,^\circ\text{C} \quad \text{(throttle pre-cool, on a 0-100 scale)} \\ \text{clamp}\left(\overline{Q}_{pred} + \dfrac{C_{th}\,(T_{batt}-T_{set})}{\tau_{control}},\ 0,\ Q_{cooling,max}\right) & T_{batt} \ge T_{set}\ (32\,^\circ\text{C}) \\ 0 & \text{otherwise} \end{cases}$$

This directly implements the brief's request for a controller informed by *"charge rates, throttle position, and location data."*

> **Simscape port scope note.** `models/+controllers/PredictiveLookaheadController.m` ports only the heat-generation-forecast branch of this logic into the Simscape Fluids plant. The charger-proximity and throttle-lookahead branches are not ported, because the UDDS-derived scenario driving that plant has no route-position or throttle signal wired into it — only the `Q_heat_ts` heat-load timeseries. All three forecast branches remain implemented and benchmarked together in `scripts/Dynamic_Loads.m`.

## 4.9 Controller 3 — Constrained Nonlinear MPC

A receding-horizon MPC solves, at every timestep, for the cooling-power sequence over a 120 s horizon that minimises a weighted cost while strictly enforcing the 35 °C safety limit:

$$\min_{u(1),\dots,u(H_p)} \ \sum_{k=1}^{H_p} \Big[\ w_T\,\big(T_{pred}(k) - T_{set}\big)^2 \ +\ w_E\,u(k)^2 \ +\ w_{dU}\,\big(u(k)-u(k-1)\big)^2\ \Big]$$

$$\text{subject to} \quad T_{pred}(k{+}1) = T_{pred}(k) + \frac{\Delta t}{C_{th}}\left(Q_{heat}(k) - \frac{T_{pred}(k)-T_{amb}(k)}{R_{th}} - u(k)\right)$$

$$0 \le u(k) \le Q_{cooling,max}, \qquad T_{pred}(k) \le T_{max}\ (35\,^\circ\text{C}) \quad \forall\,k \in \{1,\dots,H_p\}$$

solved with `fmincon` (`sqp`, warm-started from the previous solution) and only the first control action of each solve is applied, in standard receding-horizon fashion. The energy weight (`w_E = 1e-5`) and slew-rate weight (`w_dU = 1e-3`) trade cooling-energy use and actuator wear against tracking accuracy.

## 4.10 SOC Estimation via Kalman Filter

**Prediction (coulomb counting):**

$$\widehat{SOC}_{k|k-1} = \widehat{SOC}_{k-1} - \frac{I_{batt}(k)\,\Delta t}{3600\,C_{Ah}}, \qquad P_{k|k-1} = P_{k-1} + Q_{process}$$

**Update (voltage correction):**

$$V_{oc,pred} = \text{interp1}(SOC_{curve},\,OCV_{curve},\,\widehat{SOC}_{k|k-1})$$

$$K = \frac{P_{k|k-1}\,H}{H\,P_{k|k-1}\,H + R}, \qquad H = 100,\quad R = 0.5,\quad Q_{process} = 10^{-6}$$

$$\widehat{SOC}_k = \widehat{SOC}_{k|k-1} + K\,\big(V_{meas}(k) - V_{oc,pred}\big), \qquad P_k = (1-K H)\,P_{k|k-1}$$

approximating the voltage-based SOC estimation used in a real Battery Management System.

## 4.11 Battery Aging (Arrhenius SEI Growth) and Range Impact

Instantaneous capacity-fade rate is modelled with an Arrhenius temperature dependence:

$$k_{deg}(T) = A_{pre}\,\exp\!\left(\frac{-E_a}{R_{gas}\,T_{K}}\right), \qquad E_a = 31700\ \text{J/mol}$$

$$\text{aging} = \int_{0}^{t_{end}} k_{deg}\big(T(t)\big)\,dt$$

Relative life extension of a controller versus the reactive baseline is:

$$\Delta\text{life} = \frac{\text{aging}_{base} - \text{aging}_{x}}{\text{aging}_{base}}$$

and additional driving range unlocked by spending less energy on cooling is:

$$\Delta\text{range} = \frac{E_{base} - E_{x}}{\text{specific consumption (Wh/km)}}$$

— the two pieces of the brief's "advanced work" ask.

---

# 5 MATLAB & Simulink Implementation Methodology

| File | Role |
|---|---|
| `scripts/ev_eneergy_model_realistic_predictive_cooling.m` | UDDS-based powertrain + battery model; generates `EV_Thermal_Inputs.mat` (`Q_heat_ts`, `v_ts`, `SOC_ts`, `T_amb_ts`) consumed by the Simscape Fluids plant |
| `scripts/Dynamic_Loads.m` | Synthetic 2000 s multi-phase mission (drive → fast-charge → soak); implements and benchmarks all three controllers (Steps 4–6) plus the Kalman SOC estimator; produces `data/Step3_Dynamic_Loads.mat` |
| `models/EV_Predictive_Cooling_Plant.slx` | Closed-loop Simscape Fluids plant template; its embedded controller block is a plain reactive thermostat (`reactive_cooling_controller`), despite the filename — kept as the original closed-loop template the other controller variants are cloned from |
| `models/EV_Predictive_Cooling_Plant_Preview.slx` | Earlier iteration of the plant model, predating the `_Reactive`/`_FixedHighFlow` split; kept for provenance, not part of the active build |
| `models/EV_Predictive_Cooling_Plant_FixedHighFlow.slx` | Open-loop constant-flow reference plant for isolating plant vs. controller behaviour |
| `models/EV_Predictive_Cooling_Plant_Reactive.slx` | Reactive baseline controller (hysteresis, `T_on`=32°C/`T_off`=30°C) wired into the Simscape Fluids plant; the validated diagnostics in Section 6.1 come from this model |
| `models/EV_Predictive_Cooling_Plant_PredictiveLookahead.slx` | Predictive lookahead controller (Section 4.8's heat-generation-forecast branch only — see scope note below) wired into the *same* Simscape Fluids plant as the reactive model; built by `scripts/build_predictive_mpc_models.m` |
| `models/EV_Predictive_Cooling_Plant_MPC.slx` | Full nonlinear MPC controller (Section 4.9, identical cost/constraint functions as `Dynamic_Loads.m`) wired into the *same* Simscape Fluids plant; built by `scripts/build_predictive_mpc_models.m` |
| `models/+controllers/PredictiveLookaheadController.m`, `MPCCoolingController.m` | `matlab.System` classes implementing the two controllers above as drop-in replacements for the reactive controller's block, so all three run on identical plant physics |
| `scripts/build_predictive_mpc_models.m` | Clones `EV_Predictive_Cooling_Plant_Reactive.slx`, deletes its reactive controller block, and wires in the predictive/MPC `matlab.System` controllers in its place — see [Section 9](#9-status-and-next-steps) for verification status |
| `scripts/ev_energy_model.m`, `ev_energy_model_realistic.m` | Earlier powertrain-only iterations (UDDS energy/SOC study), retained for provenance |
| `scripts/P4_Hybrid_Architectural_Transition.m` | Exploratory script from the architectural transition between the powertrain-only and cooling-plant phases of the project |
| `source_powertrain/` | Original MATLAB Drive export of the antecedent powertrain project; kept as archival source material, not part of the active build |

---

# 6 Simulation Results and Performance Metrics

Two independent result sets exist in this repository, and they are kept separate deliberately because they come from different plants and have different evidentiary status.

## 6.1 Simulink Reactive-Baseline Diagnostics (UDDS Scenario)

> **Correction.** This section previously cited `docs/Battery_Thermal_Diagnostics_Final_Report.docx` (max battery temperature 35.51°C, ΔT up to 1.94°C above ambient). That table has turned out to be unreliable: its "maximum heat generation" figure (3772.83 W) is identical to the *synthetic 2000 s stress scenario's* peak heat generation in Section 6.2 — not a value the milder UDDS drive cycle produces — and a battery cannot be actively cooled to a *higher* temperature than it reaches with no cooling at all, yet the uncooled UDDS lumped-model estimate (Section 4.6) only rises 1.63°C over the same run. Both facts point the same way: that report was generated against the wrong input signal, most likely the same base-workspace-clobbering issue described in Section 9, just occurring earlier and undetected until this pass. It is being superseded here rather than left standing.

Re-running `models/EV_Predictive_Cooling_Plant_Reactive.slx` with correctly-ordered UDDS signals (`run_project`, or `scripts/ev_eneergy_model_realistic_predictive_cooling.m` immediately before the plant simulation) gives, read from the model's `Scope Block` (battery temperature in Kelvin) and `Scope Block1` (coolant mass flow command):

[`results/Final Results/Reactive_Simscape_Battery_Temp_UDDS.png`](<results/Final Results/Reactive_Simscape_Battery_Temp_UDDS.png>) · [`results/Final Results/Reactive_Simscape_Coolant_Flow_UDDS.png`](<results/Final Results/Reactive_Simscape_Coolant_Flow_UDDS.png>)

| Quantity | Value |
|---|---|
| Initial battery temperature | ~300.0 K (26.85 °C) |
| Maximum / final battery temperature | ~300.3 K (~27.2 °C) |
| Total rise over the 1369 s UDDS run | ~0.3 °C |
| Coolant flow command | Constant at `mdot_low` (0.005 kg/s) throughout — the reactive controller's 32 °C `T_on` threshold is never reached, so active cooling never engages |

**Reading.** On the true UDDS scenario, the battery barely warms at all — the reactive controller's cooling threshold is never even triggered. This is a much less demanding thermal case than the multi-phase stress scenario in Section 6.2, and it changes what this section can honestly claim: it validates that the Simscape Fluids plant and reactive controller behave physically sensibly on real UDDS driving, but it is *not* a scenario that exercises active cooling, thermal safety limits, or a meaningful reactive-vs-predictive energy trade-off — that comparison is what Section 6.2's stress scenario is for.

> **Methodology note.** These numbers are read directly off the Scope Block trace, not exported via a `.mat` file — signal logging for `T_batt` is not currently configured as an actual named export in `EV_Predictive_Cooling_Plant_Reactive.slx` (the `out.T_batt_log` labels visible in the block diagram are diagram annotations, not configured signal logging). Enabling proper signal logging and re-deriving this table with exact exported values is listed in [Section 9](#9-status-and-next-steps).

## 6.2 MATLAB-Based Multi-Controller Comparison (Stress Scenario)

`scripts/Dynamic_Loads.m` runs the uncooled plant, the Step 4 reactive baseline, the Step 5 predictive lookahead controller, and the Step 5 full MPC over the same 2000 s aggressive-drive → fast-charge → soak scenario (Section 1.3), and plots them together (`results/Final Results/Comprehensive_Controller_Benchmarking_Analysis.png`, `Full_MPC_Thermal_Performance_Comparison.png`, `Predictive_vs_Reactive_Thermal_Control.png`).

| Metric | Uncooled | Step 4 Reactive | Step 5 Predictive | Step 5 Full MPC |
|---|---|---|---|---|
| Peak temperature (°C) | 59.27 | 36.58 | 33.60 | 33.61 |
| Time above 35 °C safety limit (s) | 1491 | 313 | **0** | **0** |
| Time above 32 °C setpoint (s) | 1639 | 1634 | 1261 | 1426 |
| Chiller electrical energy (Wh) | — | 142.14 | 157.84 | 160.97 |
| Energy vs. reactive baseline (%) | — | baseline | −11.05 | −13.25 |
| Actuator chatter, mean \|dQ/dt\| (W/s) | — | 6.00 | 23.00 | 4.41 |
| Battery aging reduction vs. baseline (%) | — | baseline | 7.71 | 5.18 |
| Range impact vs. baseline (km) | — | 0.000 | −0.070 | −0.084 |

Captured directly from the script's console output on a full run of the 2000 s scenario (2001-step receding-horizon MPC solve, ~223 s wall-clock) — full output saved at [`results/Dynamic_Loads_Console_Output.txt`](results/Dynamic_Loads_Console_Output.txt).

**Reading.** The predictive and MPC controllers both eliminate every safety-threshold excursion entirely — 313 s above 35 °C under reactive control drops to 0 s under both forecast-aware controllers — and both reduce the Arrhenius-modelled battery aging rate (7.71% and 5.18% respectively). They do this by spending *more* chiller energy than the reactive baseline (+11% and +13%), not less: the negative "energy vs. baseline" and "range impact" figures are a direct, expected consequence of pre-cooling ahead of the fast-charge and throttle-spike events rather than waiting for the 35 °C trip point. The MPC controller has the lowest actuator chatter (4.41 W/s, even below the reactive baseline's 6.00 W/s) because its cost function explicitly penalises slew rate; the predictive controller's simpler bang-bang pre-cool logic produces the highest chatter (23.00 W/s).

This is a real, useful finding, not a shortfall: it shows the brief's own "calculate energy-efficiency gains" question does not have a universally positive answer — in this aggressive stress scenario, forecast-aware control buys thermal safety and reduced aging at an energy cost, rather than delivering energy savings. Section 6.1's true UDDS scenario is milder still — mild enough that the reactive controller's cooling never activates at all — so a scenario of intermediate severity (heavier urban driving, a moderate fast-charge event, without the full aggressive stress test) is a natural next step for finding out whether a middle ground tips this energy trade the other way; that comparison is listed as future work in Section 9.

## 6.3 Interpretation

Read together, Sections 6.1 and 6.2 tell a consistent story: a fixed-threshold reactive controller is not free — it both wastes cooling energy in some intervals and under-cools in others, because it has no information about what is about to happen. Forecast-aware control (lookahead rules or MPC) spends cooling energy more deliberately and, in this project's scenarios, converts that spend into fewer safety-threshold violations and lower cumulative battery-aging exposure, which is the trade the MathWorks brief asks this class of controller to make.

---

# 7 Debugging of Code

### Ambient-temperature unit/reference mismatch

**Symptom.** Early diagnostics compared a Celsius battery-temperature trace to an ambient signal that was still in Kelvin (or drawn from an inconsistent source), producing an artificially large apparent battery-over-ambient temperature rise (~8.7 °C).

**Fix.** Standardised on `T_amb_ts`, the actual Simulink ambient timeseries, and converted it explicitly once: `T_amb_C_ts = T_amb_ts.Data(:) - 273.15`, placing it in the same Celsius reference as the logged battery temperature before any subtraction.

### Battery/ambient time-vector length mismatch

**Symptom.** Battery logging (`out.T_batt_log`) and the ambient input did not share identical sample times or vector lengths, so direct element-by-element `T_batt - T_amb` either errored or silently compared non-corresponding samples.

**Fix.** Restricted both signals to their common time interval, then interpolated ambient temperature onto the battery-logging time vector with `interp1(t_amb, T_amb_C_ts, t_common, 'linear')` before computing `DeltaT`, so every subtraction is between time-matched samples.

### SOC-OCV lookup table dimension mismatch

**Symptom.** `interp1` raised `X and V must be of the same length` when the SOC and OCV breakpoint vectors were edited independently and drifted out of sync.

**Fix.** Verified `SOC_curve` and `OCV_curve` are defined together as fixed 11-element vectors in the same code block, so any future edit to one is visibly paired with the other:
```matlab
SOC_curve = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0];
OCV_curve = [300, 320, 335, 345, 350, 355, 360, 365, 372, 380, 390];
```

### Energy totals silently disagreeing with average-power × duration

**Symptom.** `average power x duration` did not match the `trapz`-integrated energy total, because the power, energy, and temperature diagnostics were, in places, computed on different aligned signal windows (`t` vs. `t_Q`) rather than a single common time base.

**Fix.** Energy integrals are computed with `trapz` directly on their own governing power/time vector (`t_Q` for cooling and heat power), and are never re-derived from a reported average power multiplied by total run duration — the two quantities are documented as related but not interchangeable (see the note in Section 6.1).

### Regenerative braking active at near-zero speed

**Symptom.** Without a lower bound, the regen branch of the motor-power logic could activate at vehicle speeds low enough that the physical motor could not actually generate meaningful back-EMF, producing an unrealistic small regen power at a near-stationary vehicle.

**Fix.** Added an explicit minimum-speed gate, `v_min_regen = 1.5 m/s`, below which `P_mech` is forced to zero regardless of the sign of `P_wheel`.

### `FixedHighFlow` model added to isolate plant-only behaviour

**Symptom.** When closed-loop temperature behaviour looked wrong in `EV_Predictive_Cooling_Plant.slx`, it was not obvious whether the issue was in the coolant-loop physics (pipes, pump, radiator) or in the controller commanding the pump.

**Fix.** `EV_Predictive_Cooling_Plant_FixedHighFlow.slx` replaces the controller-commanded pump flow with a fixed 0.050 kg/s source, so the thermal-fluid network can be validated in isolation before re-introducing closed-loop control.

---

# 8 Repository Structure and How to Run

```
run_project.m      Single entry point — runs the full pipeline end-to-end (see below)
models/    EV_Predictive_Cooling_Plant.slx, _Preview.slx, _FixedHighFlow.slx, _Reactive.slx, _PredictiveLookahead.slx, _MPC.slx
models/+controllers/  matlab.System controller classes (predictive lookahead, MPC) shared by the Simscape variants
scripts/   Dynamic_Loads.m, ev_eneergy_model_realistic_predictive_cooling.m, build_predictive_mpc_models.m, and earlier powertrain iterations
data/      uddsdc.csv, Step3_Dynamic_Loads.mat, EV_Thermal_Inputs.mat  (drive-cycle input and generated/consumed signal exports)
docs/      Battery_Thermal_Diagnostics_Final_Report.docx, Dynamic_Loads.pdf, Reactive_Model_Diagnostics.pdf
results/   Final Results/  — generated figures (temperature, energy, benchmarking plots)
source_powertrain/  archival export of the antecedent powertrain (UDDS energy/SOC) project
```

**Single entry point.** From MATLAB, with this repository as the working folder, run:

```matlab
run_project
```

This runs the full system end-to-end with no manual steps: it (1) regenerates the UDDS-based thermal/electrical input signals, (2) runs the MATLAB-only reactive vs. predictive vs. MPC controller comparison (Section 6.2), (3) simulates the Simscape Fluids reactive-baseline plant (Section 6.1), and (4) builds and simulates the predictive-lookahead and MPC Simscape Fluids plant variants (Section 5). Steps 3-4 require Simulink, Simscape Fluids, and (for the MPC controller) Optimization Toolbox; if unavailable they are skipped with a message and the earlier steps still complete.

To run an individual stage instead of the full pipeline:

```matlab
run("scripts/Dynamic_Loads.m")                                    % MATLAB-only controller comparison (Section 6.2)
run("scripts/ev_eneergy_model_realistic_predictive_cooling.m")    % regenerate EV_Thermal_Inputs.mat
run("scripts/build_predictive_mpc_models.m")                      % (re)build the predictive/MPC Simscape variants
open("models/EV_Predictive_Cooling_Plant_Reactive.slx")           % open any of the three Simscape Fluids plants
```

---

# 9 Status and Next Steps

Steps 1-2 mirror the recommended sequence recorded in `docs/Battery_Thermal_Diagnostics_Final_Report.docx`; steps 3-4 close the gap that report's own "next Simulink step" section called out.

1. **Complete, corrected** — battery thermal plant and reactive/baseline controller run against the true UDDS scenario in the Simscape Fluids model (Section 6.1). The originally-cited `docs/Battery_Thermal_Diagnostics_Final_Report.docx` numbers (35.51°C peak) were found to be internally inconsistent and have been superseded with numbers read from a correctly-ordered re-run (~27.2°C peak); see the correction note in Section 6.1.
2. **Complete** — reactive, predictive, and full-MPC controllers implemented and compared in the MATLAB control-oriented model (Section 6.2), with console output persisted at `results/Dynamic_Loads_Console_Output.txt`.
3. **Built; wiring verified, controller execution not yet clean** — `scripts/build_predictive_mpc_models.m` clones the validated reactive plant and wires in `matlab.System` predictive-lookahead and MPC controllers (`models/+controllers/`) in place of the reactive controller block. As of this update, the block-swap and rewiring runs successfully (both `.slx` files build without error), but simulating the built models has not yet completed cleanly — the last attempt failed with a non-specific "Error due to multiple causes" that needs to be re-captured by calling `sim(...)` directly at the command line (not through `run_project`'s try/catch, which swallows the detail) to get the real underlying error. The predictive-lookahead port is also intentionally scoped down from the full Section 4.8 logic (see the scope note in Section 4.8) — it uses only the heat-generation-forecast branch, since this plant has no route or throttle signal to drive the other two branches.
4. **Complete** — the Step 6 `fprintf` benchmark table is captured in Section 6.2 with real console output from a full run. Remaining: once step 3 above is verified, capture the equivalent table from all three *Simscape* variants (not just the MATLAB lumped model) so Section 6.1's evidentiary tier covers predictive and MPC, not only the reactive baseline.
5. **Open** — enable proper Simulink signal logging (the `out.T_batt_log` labels in the block diagram are currently just annotations, not configured logging — confirmed by `who(sim(...))` returning only `tout`) and log the same signal set (`T_batt`, `T_amb`, `ΔT`, `Q_heat`, cooling power) for every controller run on a common time base, so Section 6.1 can cite exact exported values instead of numbers read off a scope trace.

---

# 10 Learning Outcomes

- Coupling a control-oriented lumped MATLAB thermal model to a physical Simscape Fluids plant, and treating disagreement between the two as a modeling signal rather than a bug to suppress.
- Formulating and solving a receding-horizon constrained MPC (`fmincon`/`sqp`) with a hard safety constraint enforced at every step of the prediction horizon, not just at the terminal state.
- Building a predictive controller from heterogeneous forecast signals — a heat-generation lookahead, route-position/telematics proximity to a known event (fast charging), and short-horizon throttle demand — rather than a single predicted variable.
- Recognising that "predictive control saves energy" is scenario-dependent: in this project's stress scenario it instead traded higher proactive cooling spend for fewer safety excursions and lower aging exposure, which is a different and equally valid form of improvement.
- Translating battery temperature history into engineering-relevant outcomes (Arrhenius aging, range extension) instead of stopping at a temperature plot.
- Debugging cross-domain signal-alignment and workspace-ordering issues that produce numerically plausible but physically wrong results — most notably the Section 6.1 correction: a previously-reported 35.51°C reactive-baseline peak turned out to be internally inconsistent (an actively-cooled plant cannot run hotter than its own uncooled estimate) and was traced to the same base-workspace-clobbering pattern documented in Section 9, not a real UDDS result. Catching an inconsistency by cross-checking two independently-computed numbers against each other — rather than trusting either one in isolation — is what surfaced it.

---

# 11 References

Open-access papers that informed or corroborate the modeling choices in Section 4, grouped by the subsystem they relate to.

**Predictive / MPC battery thermal control**

1. Zhang, Q. et al. *Two-Layer Model Predictive Battery Thermal and Energy Management Optimization for Connected and Automated Electric Vehicles*. arXiv:1809.10002. [PDF](https://arxiv.org/pdf/1809.10002) — receding-horizon MPC for battery thermal/energy management, directly relevant to the [Controller 3 — Constrained Nonlinear MPC](#49-controller-3--constrained-nonlinear-mpc) formulation in Section 4.9.
2. *Physics-Informed Predictive Control for Integrated Electric-Vehicle Thermal Management: An Open, Real-Data-Anchored Benchmark*. arXiv:2606.22529. [PDF](https://arxiv.org/pdf/2606.22529) — open benchmark coupling battery electro-thermal-aging models under real drive cycles, relevant to the dual MATLAB/Simscape plant approach in Section 3.

**SOC estimation (Kalman filtering)**

3. Hu, L., Hu, R., Ma, Z., Jiang, W. *State of Charge Estimation and Evaluation of Lithium Battery Using Kalman Filter Algorithms*. PMC9785816. [Full text](https://pmc.ncbi.nlm.nih.gov/articles/PMC9785816/) — voltage-based Kalman SOC estimation under drive-cycle conditions, corroborating the estimator in Section 4.10.
4. *A Critical Look at Coulomb Counting Towards Improving the Kalman Filter Based State of Charge Tracking Algorithms in Rechargeable Batteries*. arXiv:2101.05435. [PDF](https://arxiv.org/pdf/2101.05435) — analyses the coulomb-counting prediction step combined with Kalman correction used in Section 4.10.

**Battery heat generation (ohmic + entropic)**

5. Chun, H. et al. *Comprehensive Study on Thermal Characteristics of Lithium-Ion Battery With Entropic Heat*. International Journal of Energy Research, Wiley/Hindawi, 2024. [Open access](https://onlinelibrary.wiley.com/doi/10.1155/2024/8815580) — quantifies the reversible entropic heat term used alongside the ohmic term in Section 4.5.

**Battery aging (Arrhenius / SEI growth)**

6. *Theory of SEI Formation in Rechargeable Batteries: Capacity Fade, Accelerated Aging and Lifetime Prediction*. arXiv:1210.3672. [PDF](https://arxiv.org/pdf/1210.3672) — foundational Arrhenius-type treatment of SEI-driven capacity fade underlying the aging model in Section 4.11.
7. *Solid–Electrolyte Interphase During Battery Cycling: Theory of Growth Regimes*. PMC7496968. [Full text](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC7496968/) — reaction/diffusion/migration growth-regime treatment of SEI formation, background for the same section.
8. Ramesh, T.N., Rao, K.V. *An Empirical Rate Constant Based Model to Study Capacity Fading in Lithium Ion Batteries*. International Journal of Electrochemistry, Hindawi, 2015. [Open access](https://onlinelibrary.wiley.com/doi/10.1155/2015/439015) — empirical Arrhenius rate-constant capacity-fade model, the closest published analogue to the $k_{deg}(T)$ formulation in Section 4.11.
9. Smith, K. et al. *Lithium-Ion Battery Life Model with Electrode Cracking and Dead-Lithium Formation*. NREL/TP-5700-79499. [PDF](https://docs.nrel.gov/docs/fy22osti/79499.pdf) — U.S. government (NREL) life/aging model used to cross-check the relative life-extension and range-impact estimates in Section 4.11.

**Liquid cooling plant architecture**

10. *A Review of Lithium-Ion Battery Thermal Management Based on Liquid Cooling and Its Evaluation Method*. ResearchGate preprint. [PDF](https://www.researchgate.net/publication/395041418_A_Review_of_Lithium-Ion_Battery_Thermal_Management_Based_on_Liquid_Cooling_and_Its_Evaluation_Method) — survey of cold-plate/liquid-loop architectures matching the Simscape Fluids plant topology (pump, cold plate, radiator, reservoir) described in Section 3.

---

# 12 Setup, Dependencies, and Contact

## 12.1 Setup Instructions

1. Clone or download this repository.
2. Open MATLAB and set the **Current Folder** to the repository root (the folder containing `run_project.m`).
3. Ensure the products listed in Section 12.2 are installed and licensed.
4. Run `run_project` (see [Section 8](#8-repository-structure-and-how-to-run) for details and individual-stage commands). No code edits are required — paths are resolved relative to the repository automatically.

> **You do not need to open, read, or configure anything in `source_powertrain/` to run this project.** That folder is an archival export of an earlier, separate powertrain/SOC-estimation project this one builds on conceptually — every input the active pipeline actually reads (`data/uddsdc.csv`, `data/Step3_Dynamic_Loads.mat`) is already bundled independently under `data/`, resolved by path relative to each script's own location, not to anything inside `source_powertrain/`. A fresh clone with no prior context runs end-to-end with steps 1–4 above alone.

## 12.2 Dependencies and External Tools

| Product | Used for | Required for |
|---|---|---|
| MATLAB | Core scripting, all numerical modeling | Everything |
| Optimization Toolbox (`fmincon`) | Receding-horizon MPC solve (Section 4.9) | `scripts/Dynamic_Loads.m`, `models/+controllers/MPCCoolingController.m` |
| Simulink | Block-diagram plant and controller models | `models/*.slx`, Steps 3–4 of `run_project` |
| Simscape Fluids | Cold plate, pump, radiator, reservoir, thermal liquid network | `models/EV_Predictive_Cooling_Plant*.slx` |
| Stateflow | Embedded MATLAB Function controller logic inside the reactive Simscape model | `models/EV_Predictive_Cooling_Plant_Reactive.slx` |

No non-MATLAB dependencies, external executables, or Python libraries are required. All input data (`data/uddsdc.csv`, `data/Step3_Dynamic_Loads.mat`) is bundled in the repository — no external downloads are needed to run the project end-to-end.

If Simulink, Simscape Fluids, or Optimization Toolbox are unavailable, `run_project` still completes its MATLAB-only stages (Steps 1–2) and reports which later steps were skipped and why, rather than failing silently.

## 12.3 Contact

Atharva Ajay Nikam — [github.com/atharvaajaynikam4601-netizen](https://github.com/atharvaajaynikam4601-netizen) — atharvaajaynikam4601@gmail.com
