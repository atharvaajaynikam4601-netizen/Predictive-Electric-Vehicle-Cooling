# Challenge requirements traceability

| Challenge requirement | Project implementation | Evidence in final report |
|---|---|---|
| Dynamic EV cooling plant in Simscape Fluids | Liquid coolant loop with battery/cold plate, pump, radiator, and chiller/refrigeration path | Plant architecture and validation figure |
| Outside environmental conditions | 31–35 °C hot-day profile; later cold-day sensitivity test | Scenario plot and temperature results |
| Fast charging | 150 kW DC fast-charge segment following the drive | Charge-power, temperature, and cooling-energy plot |
| Rapid acceleration/deceleration | Dynamic urban/highway/hill traction and regenerative-power profile | Speed, grade, acceleration, and battery-power plot |
| Simple baseline | Temperature-threshold reactive controller | Baseline controller diagram and results |
| Predictive controller | Preview of traction/charge power and ambient temperature; constrained actuator commands | Predictive-controller diagram and results |
| Keep battery in desired range | Temperature target and upper safety constraint, applied identically across controllers | Temperature/time plot and violations table |
| Compare cooling energy | Integrate electrical power of pump, fan, and refrigeration components | kWh comparison table |
| Advanced: range/life | Optional range-equivalent energy and thermal-exposure proxy | Clearly labelled exploratory result |
