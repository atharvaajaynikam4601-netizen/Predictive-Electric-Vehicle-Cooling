# Predictive Electric Vehicle Cooling: Simscape Fluids Battery Thermal Plant with Reactive, Lookahead-Predictive, and Nonlinear MPC Controllers in MATLAB/Simulink

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

$$T_{avail} = \begin{cases} T_{motor,max} & \omega \le \omega_{base} \quad \text{(constant-torque region)} \\[4pt] \dfrac{P_{motor,max}}{\omega} & \omega > \omega_{base} \quad \text{(constant-power / field-weakening region)} \end{cases}$$

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

$$Q_{cooling} = \begin{cases} Q_{cooling,max}\ (2000\ \text{W}) & T_{batt} \ge T_{on}\ (35\,^\circ\text{C}) \\[4pt] 0 & T_{batt} \le T_{off}\ (32\,^\circ\text{C}) \\[4pt] \text{unchanged (hold last state)} & \text{otherwise} \end{cases}$$

This is a conventional thermostat: it cannot act until the battery has already crossed the trip point, and it cannot anticipate that heat is about to fall or rise.

## 4.8 Controller 2 — Predictive Lookahead Controller (with Telematics)

The predictive controller pre-cools *before* the temperature threshold is reached, using three lookahead signals:

Three forecast signals feed the decision logic:

$$d_{charger} = s_{charger} - s_{route}(t) \qquad \text{(route-position telematics)}$$

$$\text{incoming\_charge} = \big(d_{charger} > 0\big) \ \wedge\ \big(t_{charger} \le H_p\big)$$

$$\hat{\theta}_{throttle} = \max\big(\theta_{throttle}(t \,..\, t{+}15\text{s})\big) \qquad \text{(throttle-demand lookahead)}$$

$$\overline{Q}_{pred} = \text{mean}\big(Q_{heat}(t \,..\, t{+}H_p)\big), \qquad H_p = 120\ \text{s} \qquad \text{(heat-generation forecast)}$$

and the commanded cooling power is:

$$Q_{cooling} = \begin{cases} Q_{cooling,max} & \text{incoming\_charge} \ \wedge\ T_{batt} > 28\,^\circ\text{C} \quad \text{(charger pre-cool)} \\[6pt] 0.75\,Q_{cooling,max} & \hat{\theta}_{throttle} > 50\%\ \wedge\ T_{batt} > 29\,^\circ\text{C} \quad \text{(throttle pre-cool)} \\[6pt] \text{clamp}\!\left(\overline{Q}_{pred} + \dfrac{C_{th}\,(T_{batt}-T_{set})}{\tau_{control}},\ 0,\ Q_{cooling,max}\right) & T_{batt} \ge T_{set}\ (32\,^\circ\text{C}) \\[10pt] 0 & \text{otherwise} \end{cases}$$

This directly implements the brief's request for a controller informed by *"charge rates, throttle position, and location data."*

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
| `models/EV_Predictive_Cooling_Plant.slx` | Closed-loop Simscape Fluids plant, controller-commanded coolant flow |
| `models/EV_Predictive_Cooling_Plant_FixedHighFlow.slx` | Open-loop constant-flow reference plant for isolating plant vs. controller behaviour |
| `models/EV_Predictive_Cooling_Plant_Reactive.slx` | Reactive baseline controller wired into the Simscape Fluids plant |
| `scripts/ev_energy_model.m`, `ev_energy_model_realistic.m` | Earlier powertrain-only iterations (UDDS energy/SOC study), retained for provenance |
| `scripts/P4_Hybrid_Architectural_Transition.m` | Exploratory script from the architectural transition between the powertrain-only and cooling-plant phases of the project |
| `source_powertrain/` | Original MATLAB Drive export of the antecedent powertrain project; kept as archival source material, not part of the active build |

---

# 6 Simulation Results and Performance Metrics

Two independent result sets exist in this repository, and they are kept separate deliberately because they come from different plants and have different evidentiary status.

## 6.1 Validated Simulink Reactive-Baseline Diagnostics (UDDS Scenario)

These figures come from `docs/Battery_Thermal_Diagnostics_Final_Report.docx`, a MATLAB post-processing pass over the logged outputs of `models/EV_Predictive_Cooling_Plant_Reactive.slx` running the 1369 s UDDS-derived scenario. They are the one set of numbers in this repository that has been through an explicit correction-and-verification pass (see [Section 7](#7-debugging-of-code)), so they are reported here exactly as validated.

| Quantity | Initial | Maximum | Minimum | Final |
|---|---|---|---|---|
| Battery temperature (°C) | 26.85 | 35.51 | — | 35.51 |
| Ambient temperature (°C) | 25.00 | 33.85 | — | 33.85 |
| Battery − ambient ΔT (°C) | 1.85 | 1.94 | −1.05 | 1.66 |

| Metric | Validated result | Reading |
|---|---|---|
| Maximum heat generation | 3772.83 W | Short-duration electrical-loss peak |
| Average heat generation | 777.41 W | Representative mean load |
| Maximum baseline cooling | 2000 W | Reactive controller saturates at its power limit |
| Average baseline cooling | 601.70 W | Substantial average cooling response |
| Total generated heat | 196.58 Wh | Integrated over the diagnostic time base |
| Total baseline cooling energy | 167.20 Wh | Integrated cooling delivered |
| Excess cooling above instantaneous heat | 67.53 Wh | Cooling applied when not immediately needed — the opportunity predictive control targets |
| Time heat > cooling | 419.5 s | Intervals where the reactive controller is under-cooling |
| Time cooling > heat | 228.5 s | Intervals where it is over-cooling |
| Time above 32 °C setpoint | 539.3 s | |
| Time above 35 °C safety limit | 162.7 s | |
| Cooling-active samples | 602 | Controller duty cycle |

**Reading.** Once the ambient reference is correctly aligned in Celsius and interpolated onto the battery time base (see Section 7), the battery never runs away from ambient — it stays within about 2 °C of it throughout. The real finding is not thermal runaway; it is *timing*: the reactive controller spends 67.53 Wh cooling when heat generation didn't demand it, while simultaneously leaving 162.7 s above the 35 °C safety line elsewhere in the run. That combination — wasted energy *and* unresolved excursions — is precisely the inefficiency a forecast-aware controller is designed to remove, and is the baseline the predictive and MPC controllers in Section 6.2 are benchmarked against.

## 6.2 MATLAB-Based Multi-Controller Comparison (Stress Scenario)

`scripts/Dynamic_Loads.m` runs the uncooled plant, the Step 4 reactive baseline, the Step 5 predictive lookahead controller, and the Step 5 full MPC over the same 2000 s aggressive-drive → fast-charge → soak scenario (Section 1.3), and plots them together (`results/Final Results/Comprehensive_Controller_Benchmarking_Analysis.png`, `Full_MPC_Thermal_Performance_Comparison.png`, `Predictive_vs_Reactive_Thermal_Control.png`).

> **Evidentiary note.** Unlike Section 6.1, these figures have not yet been through a persisted, re-verified numeric export — the `fprintf` summary table these scripts print is generated at MATLAB runtime and was not captured to a log file in this repository. The description below is therefore a qualitative reading of the generated plots, not a cited numeric table. Re-running `scripts/Dynamic_Loads.m` and saving its console output reproduces exact figures; that step is listed in [Section 9](#9-status-and-next-steps).

Observed from the benchmarking plot:

- **Temperature control.** The uncooled pack climbs past 55 °C by the end of the fast-charge phase. The reactive baseline oscillates around the 35 °C safety threshold and briefly exceeds it around the driving-to-charging transition. The predictive controller pre-cools ahead of both the throttle spikes and the fast-charge event, holding the pack in a 28–31 °C band *before* the demand arrives. The full MPC tracks the 32 °C setpoint the tightest of the three, with no visible excursion above the safety line.
- **Cooling energy.** Cumulative chiller + pump electrical energy is *higher* for the predictive and MPC controllers than for the reactive baseline in this particular stress scenario (roughly 160 Wh vs. 145 Wh by the end of the run). This is a legitimate and expected trade-off, not a regression: both forecast-aware controllers spend energy proactively, ahead of the fast-charge and throttle events, in exchange for materially fewer and shorter safety-threshold excursions and a tighter, flatter temperature trajectory. Reactive control is cheaper only because it under-delivers cooling during part of the run — the same 67.53 Wh-of-waste-alongside-unresolved-excursions pattern quantified in Section 6.1.
- **Battery aging (Arrhenius k_deg).** The reactive baseline shows the highest instantaneous degradation-rate excursions, particularly around the throttle-spike and charge-transition regions; the predictive and MPC controllers track a visibly lower and smoother aging-rate curve through the same intervals, consistent with holding temperature closer to the 32 °C setpoint rather than letting it walk up to 35 °C between hysteresis cycles.

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
models/    EV_Predictive_Cooling_Plant.slx, _FixedHighFlow.slx, _Reactive.slx  (Simscape Fluids plant + controllers)
scripts/   Dynamic_Loads.m, ev_eneergy_model_realistic_predictive_cooling.m, and earlier powertrain iterations
data/      Step3_Dynamic_Loads.mat, EV_Thermal_Inputs.mat  (generated/consumed signal exports)
docs/      Battery_Thermal_Diagnostics_Final_Report.docx, Dynamic_Loads.pdf, Reactive_Model_Diagnostics.pdf
results/   Final Results/  — generated figures (temperature, energy, benchmarking plots)
source_powertrain/  archival export of the antecedent powertrain (UDDS energy/SOC) project
```

To reproduce the MATLAB-only controller comparison (Section 6.2):

```matlab
run("scripts/Dynamic_Loads.m")
```

To regenerate the thermal-load signals feeding the Simscape Fluids plant, then open and run the plant models:

```matlab
run("scripts/ev_eneergy_model_realistic_predictive_cooling.m")   % writes EV_Thermal_Inputs.mat
open("models/EV_Predictive_Cooling_Plant_Reactive.slx")
```

---

# 9 Status and Next Steps

This mirrors the recommended sequence recorded in `docs/Battery_Thermal_Diagnostics_Final_Report.docx`:

1. **Complete** — battery thermal plant and reactive/baseline controller validated against the UDDS scenario in the Simscape Fluids model (Section 6.1).
2. **Complete** — reactive, predictive, and full-MPC controllers implemented and compared in the MATLAB control-oriented model (Section 6.2).
3. **Open** — port the predictive and MPC controller logic from `scripts/Dynamic_Loads.m` into `models/EV_Predictive_Cooling_Plant.slx`, so all three controllers are benchmarked on the *same* Simscape Fluids plant, not only the lumped MATLAB approximation.
4. **Open** — capture the Step 6 `fprintf` benchmark table (peak temperature, threshold exposure, chiller energy, actuator chatter, aging reduction, range gain) to a persisted results file so Section 6.2 can cite exact figures instead of plot-level trends.
5. **Open** — log the same signal set (`T_batt`, `T_amb`, `ΔT`, `Q_heat`, cooling power) for every controller run on the common time base, per the diagnostic methodology already validated for the reactive case.

---

# 10 Learning Outcomes

- Coupling a control-oriented lumped MATLAB thermal model to a physical Simscape Fluids plant, and treating disagreement between the two as a modeling signal rather than a bug to suppress.
- Formulating and solving a receding-horizon constrained MPC (`fmincon`/`sqp`) with a hard safety constraint enforced at every step of the prediction horizon, not just at the terminal state.
- Building a predictive controller from heterogeneous forecast signals — a heat-generation lookahead, route-position/telematics proximity to a known event (fast charging), and short-horizon throttle demand — rather than a single predicted variable.
- Recognising that "predictive control saves energy" is scenario-dependent: in this project's stress scenario it instead traded higher proactive cooling spend for fewer safety excursions and lower aging exposure, which is a different and equally valid form of improvement.
- Translating battery temperature history into engineering-relevant outcomes (Arrhenius aging, range extension) instead of stopping at a temperature plot.
- Debugging cross-domain signal-alignment issues (unit mismatches, non-matching time vectors) that produce numerically plausible but physically wrong results — the ambient-reference correction in Section 7 changed the headline finding from an apparent 8.7 °C battery self-heating rise to a validated 1.94 °C rise above a correctly-referenced, rising ambient temperature.
