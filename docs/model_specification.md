# Model specification — version 0.1

## Design question

Can a controller that knows a short preview of upcoming traction demand, fast-charge demand, and ambient conditions cool an EV battery pack earlier and with less auxiliary energy than a reactive controller?

## Scope

- Battery: 60 kWh, nominal 400 V lithium-ion pack.
- Thermal plant: liquid cooling loop with a pump and chiller/radiator heat rejection.
- Dynamic loads: hot ambient conditions, rapid acceleration/deceleration, and DC fast charging.
- Driving: a reproducible urban–highway–hill scenario, followed by standard drive-cycle tests where time permits.
- Prediction horizon: 300 s; controller sample time: 1 s initially.
- Thermal safety constraint: battery temperature must remain below 40 °C in all reported tests.

## Two-model strategy

The Simscape model is the **reference plant**: it gives electro-thermal and coolant-loop realism. The controller is first developed against a reduced-order model:

\[
C_b \dot{T}_b = \dot Q_{gen} - UA(T_b-T_c)
\]

\[
C_c \dot{T}_c = UA(T_b-T_c)-\dot Q_{reject}
\]

where battery heat generation begins with \(\dot Q_{gen}=I^2R\). The reduced model will be calibrated against the Simscape response before MPC results are reported.

## Controller inputs and outputs

Inputs: measured battery temperature, coolant temperature, estimated SOC, ambient temperature, and a preview of traction power, braking/regen demand, and incoming charge power.

Outputs: coolant-pump command and cooling/chiller command, each constrained to physical bounds and rate limits.

## Required comparisons

1. Reactive temperature-threshold controller.
2. Preview rule-based controller.
3. Constrained predictive controller.

All controllers use the same plant, initial state, scenario, limits, and measurement assumptions.

## Acceptance evidence

- Temperature: battery temperature stays in the defined desired/safe operating band.
- Performance envelope: report the largest allowed discharge and charge power before the thermal limit is reached.
- Energy: report the integrated pump, fan, and refrigeration electrical energy in kWh.
- Fairness: run the reactive and predictive controllers on identical load and ambient profiles.
- Practicality: repeat the predictive test with imperfect preview data (for example, a 20% power forecast error).
