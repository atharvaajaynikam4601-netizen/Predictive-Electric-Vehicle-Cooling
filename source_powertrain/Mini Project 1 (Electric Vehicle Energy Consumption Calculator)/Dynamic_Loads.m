clc;
clear;
close all;
tic; % Start execution timer

%% ========================================================================
%% PART 1: VEHICLE, DRIVETRAIN & BATTERY PARAMETERS (ORIGINAL MODEL)
%% ========================================================================
% Vehicle Dynamics Parameters
m = 1800;                 % Vehicle mass (kg)
g = 9.81;                 % Gravity (m/s^2)
Crr = 0.01;               % Rolling resistance coefficient
rho = 1.225;              % Air density (kg/m^3)
Cd = 0.29;                % Drag coefficient
A = 2.2;                  % Frontal area (m^2)
delta = 0.08;             % Rotational mass factor
m_eq = m * (1 + delta);   % Equivalent inertial mass (kg)
r_wheel = 0.3;            % Wheel radius (m)
gear_ratio = 9;           % Transmission gear ratio
eta_tire = 0.95;          % Tire mechanical efficiency
eta_drivetrain = 0.95;    % Drivetrain efficiency

% Motor & Inverter Parameters
T_motor_max = 250;        % Max motoring torque (Nm)
T_regen_max = 120;        % Max regenerative torque (Nm)
P_motor_max = 80e3;       % Max motor mechanical power (W)
v_min_regen = 1.5;        % Minimum velocity for regen braking (m/s)

% Battery Pack Physical Parameters (Restored to Baseline Design)
Batt_capacity = 50;       % Battery capacity (kWh)
V_nom = 350;              % Nominal pack voltage (V)
Batt_Cap_Ah = (Batt_capacity * 1000) / V_nom; % Ah Capacity (~142.86 Ah)
SOC_init = 0.90;          % Initial SOC (90%)
dU_dT = -0.00022;         % Entropic coefficient (V/K) [Reversible Heat]
C_th = 45000;             % Thermal capacitance (J/K)
R_th = 1.5;               % Passive thermal resistance to ambient (K/W)
P_accessory = 900;        % Continuous accessory electrical power (W)

%% ========================================================================
%% PART 2: MULTI-PHASE MISSION PROFILE GENERATION (2000s Total)
%% ========================================================================
dt = 1;                   % Simulation timestep (s)
t = (0:dt:2000)';         % Master time vector (0 to 2000 seconds)
N = length(t);

v = zeros(N, 1);               % Vehicle speed (m/s)
T_amb = zeros(N, 1);           % Ambient temperature (°C)
I_charge_demand = zeros(N, 1); % External fast charge current profile (A)

% --- Scenario 3: Dynamic Ambient Temperature Ramp ---
% Ambient temperature ramps from 25°C to 38°C (Extreme Summer Conditions)
T_amb = 25 + 13 * (1 - exp(-t / 1200));

% --- Scenario 1: Aggressive Driving / Rapid Accel & Decel (0s to 600s) ---
t_drive = t(1:601);
v_drive = 22 * sin(2*pi*t_drive/120) + 12 * sin(2*pi*t_drive/30) + 15;
v_drive(v_drive < 0) = 0;
v(1:601) = v_drive;

% --- Transition Period (601s to 700s) ---
v(602:701) = 0; % Vehicle comes to rest at fast-charging station

% --- Scenario 2: High-Power DC Fast Charging (701s to 1600s) ---
% 250 kW High-Current surge (300A CC-CV fast charging sequence)
t_charge = t(702:1601) - t(701);
I_charge_profile = -300 * exp(-t_charge / 600); % Negative sign = Charging current
I_charge_profile(I_charge_profile > -100) = -100; % Charge floor
I_charge_demand(702:1601) = I_charge_profile;

% --- Post-Charge Soak / Idle Period (1601s to 2000s) ---
v(1602:end) = 0;

%% ========================================================================
%% PART 3: LONGITUDINAL VEHICLE DYNAMICS & POWER MAPPING
%% ========================================================================
% Acceleration vector calculation
a = [diff(v)/dt; 0];
a_max = 3.0; a_min = -4.0;
a(a > a_max) = a_max;
a(a < a_min) = a_min;

% Throttle Position Calculation (%) for Telematics Lookahead
theta_throttle = min(100, max(0, (a / a_max) * 100));

% Wheel rotational speed
omega_wheel = v / r_wheel;
omega = omega_wheel * gear_ratio;
omega_base = P_motor_max / T_motor_max;
omega_max = max(omega) + eps;

% Force calculations
F_rr = m * g * Crr;
F_aero = 0.5 * rho * Cd * A .* v.^2;
F_acc = m_eq .* a;
F_total = F_rr + F_aero + F_acc;

% Mechanical wheel power
P_wheel = (F_total .* v) / eta_tire;

% Motor torque limits and mechanical power flow
P_mech = zeros(N, 1);
for i = 1:N
    if P_wheel(i) > 0
        if omega(i) <= omega_base
            T_avail = T_motor_max;
        else
            T_avail = P_motor_max / omega(i);
        end
        P_avail = T_avail * omega(i);
        P_mech(i) = min(P_wheel(i), P_avail);
    elseif P_wheel(i) < 0 && v(i) > v_min_regen
        P_regen_limit = T_regen_max * omega(i);
        P_mech(i) = -min(abs(P_wheel(i)), P_regen_limit);
    else
        P_mech(i) = 0;
    end
end
P_mech = P_mech / eta_drivetrain;

% Motor and inverter efficiency mapping
drive_eff = ones(N, 1);
regen_eff = ones(N, 1);
for i = 1:N
    if P_mech(i) > 0
        T_req = P_mech(i) / max(omega(i), eps);
        Tn = abs(T_req) / T_motor_max;
        on = omega(i) / omega_max;
        drive_eff(i) = 0.90 - 0.15*(1-on)^2 - 0.20*(Tn-0.6)^2;
        drive_eff(i) = min(max(drive_eff(i), 0.75), 0.92);
    elseif P_mech(i) < 0
        T_req = abs(P_mech(i)) / max(omega(i), eps);
        Tn = T_req / T_regen_max;
        on = omega(i) / omega_max;
        regen_eff(i) = 0.75 - 0.20*(1-on)^2 - 0.30*(Tn-0.5)^2;
        regen_eff(i) = min(max(regen_eff(i), 0.55), 0.80);
    end
end

% Electrical power calculations
P_batt_disch_max = 90e3;
P_batt_char_max  = 40e3;
P_drive = zeros(N, 1);
P_regen = zeros(N, 1);

for i = 1:N
    if P_mech(i) > 0
        P_drive(i) = P_mech(i) / drive_eff(i);
        P_drive(i) = min(P_drive(i), P_batt_disch_max);
    elseif P_mech(i) < 0
        P_regen(i) = abs(P_mech(i)) * regen_eff(i);
        P_regen(i) = min(P_regen(i), P_batt_char_max);
    end
end

% Total battery power required during drive cycle
P_batt_drive = P_drive - P_regen + (v > 0) * P_accessory;

%% ========================================================================
%% PART 4: 2D INTERNAL RESISTANCE & ELECTRICAL LOOKUP DEFINITION
%% ========================================================================
SOC_curve = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0];
OCV_curve = [300, 320, 335, 345, 350, 355, 360, 365, 372, 380, 390]; % OCV (V)

% 2D Lookup Table for Internal Resistance R_int(SOC, T) in Ohms
T_vec   = [0,   15,   25,   40,   55];      % Temperature grid (°C)
SOC_vec = [0.1, 0.2,  0.5,  0.8,  1.0];     % SOC grid

R_i_map = [
    0.150, 0.080, 0.060, 0.045, 0.042; % SOC 10%
    0.130, 0.070, 0.052, 0.040, 0.038; % SOC 20%
    0.120, 0.065, 0.048, 0.035, 0.034; % SOC 50%
    0.125, 0.068, 0.050, 0.037, 0.035; % SOC 80%
    0.140, 0.075, 0.055, 0.042, 0.040  % SOC 100%
];

% Interpolant Object
F_Rint = griddedInterpolant({SOC_vec, T_vec}, R_i_map, 'linear', 'linear');

%% ========================================================================
%% PART 5: COULOMB COUNTING & HIGH-FIDELITY HEAT GENERATION LOOP
%% ========================================================================
I_batt = zeros(N, 1);
SOC = zeros(N, 1);
T_batt = zeros(N, 1);
V_oc_vec = zeros(N, 1);
V_batt = zeros(N, 1);
R_internal_vec = zeros(N, 1);

Q_ohmic = zeros(N, 1);
Q_entropic = zeros(N, 1);
Q_heat = zeros(N, 1);

% Initial conditions
SOC(1) = SOC_init;
T_batt(1) = T_amb(1);


% Active HVAC & Cooling Circuit Parameters
COP_chiller = 2.5;        % Coefficient of Performance for chiller
P_pump = 50;              % Coolant pump electrical power consumption (W)
s_charger = 8820;         % DC Fast Charger location along route (meters)

for i = 1:N
    % Current calculation: sum of traction load and fast charge demand
    if I_charge_demand(i) < 0
        I_batt(i) = I_charge_demand(i); % DC Fast Charge phase
    else
        I_batt(i) = P_batt_drive(i) / V_nom; % Traction / Regen phase
    end

    % Coulomb counting SOC update
    if i > 1
        E_step_Wh = (I_batt(i) * V_nom * dt) / 3600;
        SOC(i) = SOC(i-1) - E_step_Wh / (Batt_capacity * 1000);
        SOC(i) = max(0.05, min(1.0, SOC(i)));
    end

    % OCV Lookup
    V_oc_vec(i) = interp1(SOC_curve, OCV_curve, SOC(i), 'linear', 'extrap');

    % 2D Lookup for internal resistance R_int(SOC, T_batt)
    % CORRECTED CODE FOR PART 5
    T_curr = T_batt(i);
    SOC_curr = SOC(i);
    R_internal_vec(i) = F_Rint(SOC_curr, T_curr);

    % Thermodynamic Heat Formulation:
    % 1. Ohmic (Irreversible) loss: Q_ohmic = I^2 * R
    Q_ohmic(i) = (I_batt(i)^2) * R_internal_vec(i);

    % 2. Entropic (Reversible) loss: Q_entropic = I * T_kelvin * (dU/dT)
    T_kelvin = T_curr + 273.15;
    Q_entropic(i) = I_batt(i) * T_kelvin * dU_dT;

    % 3. Total Heat Generation
    Q_heat(i) = Q_ohmic(i) + Q_entropic(i);

    % Terminal Voltage Calculation
    V_batt(i) = V_oc_vec(i) - I_batt(i) * R_internal_vec(i);

    % Uncooled open-loop baseline thermal update
    if i < N
        Q_passive_cooling = (T_curr - T_amb(i)) / R_th;
        dT = (Q_heat(i) - Q_passive_cooling) / C_th;
        T_batt(i+1) = T_curr + dT * dt;
    end
end

%% ========================================================================
%% PART 5B: STEP 4 - BASELINE ACTIVE THERMAL CONTROLLER (HYSTERESIS)
%% ========================================================================
% Controller Settings
T_on  = 35.0;     % Turn-ON threshold (°C)
T_off = 32.0;     % Turn-OFF threshold (°C)
Q_cooling_max = 2000; % Active cooling power capacity (W)

% Tracking Vectors
T_batt_baseline      = zeros(N, 1);
Q_cooling_baseline   = zeros(N, 1);
cooling_active_state = zeros(N, 1); % Binary state: 1 = ON, 0 = OFF

% Initial Condition
T_batt_baseline(1) = T_amb(1);
is_cooling_on = false;

% Baseline Thermal Simulation Loop
for i = 1:N
    T_curr = T_batt_baseline(i);
    
    % Hysteresis Controller Logic
    if T_curr >= T_on
        is_cooling_on = true;
    elseif T_curr <= T_off
        is_cooling_on = false;
    end
    
    % Apply Active Cooling Power
    if is_cooling_on
        Q_cooling_baseline(i) = Q_cooling_max;
        cooling_active_state(i) = 1;
    else
        Q_cooling_baseline(i) = 0;
        cooling_active_state(i) = 0;
    end
    
    % Dynamic Thermal Update (Heat Gen - Passive Heat Loss - Active Cooling)
    if i < N
        Q_passive = (T_curr - T_amb(i)) / R_th;
        dT = (Q_heat(i) - Q_passive - Q_cooling_baseline(i)) / C_th;
        T_batt_baseline(i+1) = T_curr + dT * dt;
    end
end

% Energy Metrics for Baseline Controller
% Energy Metrics for Baseline Controller (accounting for Chiller COP and Pump Power)
E_cooling_baseline_Wh = trapz(t, (Q_cooling_baseline / COP_chiller) + (Q_cooling_baseline > 0) * P_pump) / 3600;

% Print Baseline Controller Performance
fprintf('\n========================================================\n');
fprintf('     STEP 4: BASELINE CONTROLLER PERFORMANCE SUMMARY    \n');
fprintf('========================================================\n');
fprintf('Uncooled Max Temperature    : %.2f °C\n', max(T_batt));
fprintf('Baseline Max Temperature    : %.2f °C\n', max(T_batt_baseline));
fprintf('Total Active Cooling Energy : %.2f Wh\n', E_cooling_baseline_Wh);
fprintf('Total Cooling Runtime       : %.0f seconds\n', sum(cooling_active_state) * dt);
fprintf('========================================================\n\n');

%% Visualizing Step 4 Performance
figure('Name', 'Step 4: Baseline Active Cooling Performance', 'NumberTitle', 'off');
subplot(2,1,1);
plot(t, T_batt, 'r--', 'LineWidth', 1.5); hold on;
plot(t, T_batt_baseline, 'b', 'LineWidth', 2);
yline(T_on, 'k--', 'Turn-ON (35°C)');
yline(T_off, 'g--', 'Turn-OFF (32°C)');
ylabel('Temp [°C]'); grid on;
legend('Uncooled Baseline', 'Step 4 Reactive Baseline', 'Location', 'northwest');
title('Battery Temperature: Uncooled vs. Step 4 Active Controller');

subplot(2,1,2);
plot(t, Q_cooling_baseline, 'm', 'LineWidth', 1.5);
ylabel('Cooling Power [W]'); xlabel('Time [s]'); grid on;
title('Step 4 Baseline Active Cooling Power Applied');

%% ========================================================================
%% PART 5C: STEP 5 - PREDICTIVE THERMAL CONTROLLER (LOOKAHEAD PRE-COOLING)
%% ========================================================================
% Predictive Controller Hyperparameters
H_p = 120;                       % Lookahead prediction horizon (seconds)
Q_thresh = 1200;                 % Predicted heat generation threshold (W)
T_pre_target = 28.0;             % Pre-cooling lower limit target (°C)
T_set_predictive = 32.0;         % Target controlled temperature (°C)
tau_control = 30.0;              % Proportional control time constant (s)

% Tracking Vectors
T_batt_predictive     = zeros(N, 1);
Q_cooling_predictive  = zeros(N, 1);
Q_pred_future         = zeros(N, 1);

% Initial Condition
T_batt_predictive(1) = T_amb(1);

% Predictive Simulation Loop
% Track Route Position for Telematics
s_route = cumtrapz(t, v); 

% Predictive Simulation Loop with Telematics & Lookahead
for i = 1:N
    T_curr = T_batt_predictive(i);
    v_curr = v(i);
    
    % 1. Telematics: Charger Proximity & Time-to-Arrival
    d_to_charger = s_charger - s_route(i);
    if v_curr > 0.5
        t_to_charger = d_to_charger / v_curr;
    else
        t_to_charger = d_to_charger / max(1, mean(v));
    end
    incoming_charge_flag = (d_to_charger > 0) && (t_to_charger <= H_p);
    
    % 2. Telematics: Throttle Lookahead (15-second peak horizon)
    idx_throttle_end = min(N, i + 15);
    peak_throttle_lookahead = max(theta_throttle(i:idx_throttle_end));
    
    % 3. Lookahead Heat Generation
    idx_end = min(N, i + H_p);
    Q_pred_future(i) = mean(Q_heat(i:idx_end));
    
    % --- DECISION LOGIC INCORPORATING TELEMATICS ---
    if incoming_charge_flag && T_curr > T_pre_target
        % Pre-cooling triggered by Location & Fast-Charge proximity
        Q_cooling_predictive(i) = Q_cooling_max;
        
    elseif peak_throttle_lookahead > 50 && T_curr > 29.0
        % Pre-cooling triggered by Throttle demand spike
        Q_cooling_predictive(i) = 0.75 * Q_cooling_max;
        
    elseif T_curr >= T_set_predictive
        % Proportional active setpoint maintenance
        P_term = C_th * (T_curr - T_set_predictive) / tau_control;
        Q_req = Q_pred_future(i) + P_term;
        Q_cooling_predictive(i) = min(Q_cooling_max, max(0, Q_req));
        
    else
        Q_cooling_predictive(i) = 0;
    end
    
    % Dynamic Thermal State Integration
    if i < N
        Q_passive = (T_curr - T_amb(i)) / R_th;
        dT = (Q_heat(i) - Q_passive - Q_cooling_predictive(i)) / C_th;
        T_batt_predictive(i+1) = T_curr + dT * dt;
    end
end

% Energy Metrics Calculation (accounting for Chiller COP and Pump Power)
E_cooling_pred_Wh = trapz(t, (Q_cooling_predictive / COP_chiller) + (Q_cooling_predictive > 0) * P_pump) / 3600;
E_savings_percent = ((E_cooling_baseline_Wh - E_cooling_pred_Wh) / E_cooling_baseline_Wh) * 100;

% Print Step 5 Performance Summary
fprintf('\n========================================================\n');
fprintf('     STEP 5: PREDICTIVE CONTROLLER PERFORMANCE SUMMARY  \n');
fprintf('========================================================\n');
fprintf('Uncooled Max Temperature       : %.2f °C\n', max(T_batt));
fprintf('Step 4 Baseline Max Temp       : %.2f °C\n', max(T_batt_baseline));
fprintf('Step 5 Predictive Max Temp     : %.2f °C\n', max(T_batt_predictive));
fprintf('Baseline Active Energy         : %.2f Wh\n', E_cooling_baseline_Wh);
fprintf('Predictive Active Energy       : %.2f Wh\n', E_cooling_pred_Wh);
fprintf('Energy Savings vs. Baseline    : %.2f %%\n', E_savings_percent);
fprintf('========================================================\n\n');

%% Visualizing Step 4 vs. Step 5 Comparison
figure('Name', 'Step 5: Predictive vs. Reactive Thermal Control', 'NumberTitle', 'off');
subplot(2,1,1);
plot(t, T_batt, 'r--', 'LineWidth', 1.2); hold on;
plot(t, T_batt_baseline, 'b', 'LineWidth', 1.5);
plot(t, T_batt_predictive, 'g', 'LineWidth', 2);
yline(35.0, 'k--', 'Safety Limit (35°C)');
yline(32.0, 'm--', 'Target Operating Setpoint (32°C)');
ylabel('Temp [°C]'); grid on;
legend('Uncooled Baseline', 'Step 4 Reactive Hysteresis', 'Step 5 Predictive Lookahead', 'Location', 'northwest');
title('Battery Pack Temperature Response: Step 4 (Reactive) vs. Step 5 (Predictive)');

subplot(2,1,2);
plot(t, Q_cooling_baseline, 'b', 'LineWidth', 1.2); hold on;
plot(t, Q_cooling_predictive, 'g', 'LineWidth', 1.5);
ylabel('Cooling Power [W]'); xlabel('Time [s]'); grid on;
legend('Step 4 Reactive Power', 'Step 5 Predictive Power');
title('Active Cooling Power Demand Comparison');

%% ========================================================================
%% PART 5D: STEP 5 - FULL CONSTRAINED MODEL PREDICTIVE CONTROLLER (MPC)
%% ========================================================================
% MPC Hyperparameters
H_p = 120;                  % Prediction horizon (seconds)
T_set = 32.0;               % Target operational setpoint (°C)
T_max_limit = 35.0;         % Hard maximum temperature safety constraint (°C)
w_T = 100.0;                % Thermal error penalty weight
w_E = 1e-5;                 % Energy consumption penalty weight
w_dU = 1e-3;                % Actuator slew rate penalty weight

% Tracking Vectors
T_batt_MPC = zeros(N, 1);
Q_cooling_MPC = zeros(N, 1);
T_batt_MPC(1) = T_amb(1);

% Optimization Options (Fast execution using interior-point / sqp)
opts = optimoptions('fmincon', 'Display', 'off', 'Algorithm', 'sqp', ...
                    'MaxIterations', 30, 'OptimalityTolerance', 1e-3);

% Pre-allocate Decision Variable Initial Guess
u_init = zeros(H_p, 1);

fprintf('Running Full Receding-Horizon MPC Simulation over %d steps...\n', N);

% Receding-Horizon Control Loop
for i = 1:N
    % Define Current State and Disturbance Vectors across Horizon
    T_curr = T_batt_MPC(i);
    idx_end = min(N, i + H_p - 1);
    horizon_len = idx_end - i + 1;
    
    Q_dist = Q_heat(i:idx_end);
    T_a_dist = T_amb(i:idx_end);
    
    % Zero-pad disturbance vectors if near the end of simulation
    if horizon_len < H_p
        Q_dist = [Q_dist; repmat(Q_dist(end), H_p - horizon_len, 1)];
        T_a_dist = [T_a_dist; repmat(T_a_dist(end), H_p - horizon_len, 1)];
    end
    
    % Last applied control action for slew rate calculation
    if i == 1
        u_prev = 0;
    else
        u_prev = Q_cooling_MPC(i-1);
    end
    
    % Cost Handle and Strict Nonlinear Constraint
    cost_fun    = @(u) mpc_cost_function(u, T_curr, Q_dist, T_a_dist, u_prev, ...
                                         C_th, R_th, dt, H_p, T_set, w_T, w_E, w_dU);
    nonlcon_fun = @(u) mpc_nonlcon(u, T_curr, Q_dist, T_a_dist, C_th, R_th, dt, H_p, T_max_limit);
    
    lb = zeros(H_p, 1); 
    ub = Q_cooling_max * ones(H_p, 1);
    
    % Solve with nonlcon passed as 9th argument
    [u_opt, ~] = fmincon(cost_fun, u_init, [], [], [], [], lb, ub, nonlcon_fun, opts);
    
    % Receding Horizon Execution: Apply First Control Action
    Q_cooling_MPC(i) = u_opt(1);
    
    % Shift initial guess for next step (Warm Start)
    u_init = [u_opt(2:end); u_opt(end)];
    
    % Dynamic Thermal Integration of Plant
    if i < N
        Q_passive = (T_curr - T_amb(i)) / R_th;
        dT = (Q_heat(i) - Q_passive - Q_cooling_MPC(i)) / C_th;
        T_batt_MPC(i+1) = T_curr + dT * dt;
    end
end

% Compute Final Metrics (accounting for Chiller COP and Pump Power)
E_cooling_MPC_Wh = trapz(t, (Q_cooling_MPC / COP_chiller) + (Q_cooling_MPC > 0) * P_pump) / 3600;
MPC_energy_savings = ((E_cooling_baseline_Wh - E_cooling_MPC_Wh) / E_cooling_baseline_Wh) * 100;

% Print Fully Completed Performance Results
fprintf('\n========================================================\n');
fprintf('     STEP 5: FULL MPC PERFORMANCE SUMMARY (COMPLETED)   \n');
fprintf('========================================================\n');
fprintf('Uncooled Peak Temperature      : %.2f °C\n', max(T_batt));
fprintf('Step 4 Baseline Max Temp       : %.2f °C\n', max(T_batt_baseline));
fprintf('Step 5 Full MPC Max Temp       : %.2f °C\n', max(T_batt_MPC));
fprintf('Baseline Active Energy         : %.2f Wh\n', E_cooling_baseline_Wh);
fprintf('Full MPC Active Energy         : %.2f Wh\n', E_cooling_MPC_Wh);
fprintf('Energy Savings vs. Baseline    : %.2f %%\n', MPC_energy_savings);
fprintf('========================================================\n\n');




%% Visualizing Full Step 5 MPC vs. Predictive vs. Baseline
figure('Name', 'Step 5: Full MPC Thermal Performance Comparison', 'NumberTitle', 'off');
subplot(2,1,1);
plot(t, T_batt, 'r--', 'LineWidth', 1.2); hold on;
plot(t, T_batt_baseline, 'b:', 'LineWidth', 1.5);
plot(t, T_batt_predictive, 'g--', 'LineWidth', 1.5);
plot(t, T_batt_MPC, 'm', 'LineWidth', 2);
yline(T_max_limit, 'k--', 'Safety Threshold (35°C)');
yline(T_set, 'c--', 'Target Setpoint (32°C)');
ylabel('Temp [°C]'); grid on;
legend('Uncooled', 'Step 4 Baseline (Hysteresis)', 'Step 5 Predictive', 'Step 5 Full MPC', 'Location', 'northwest');
title('Battery Pack Temperature Control: Step 4 vs. Step 5 (Predictive & MPC)');

subplot(2,1,2);
plot(t, Q_cooling_baseline, 'b:', 'LineWidth', 1.2); hold on; % Corrected comma here
plot(t, Q_cooling_predictive, 'g--', 'LineWidth', 1.2);
plot(t, Q_cooling_MPC, 'm', 'LineWidth', 1.5);
ylabel('Cooling Power [W]'); xlabel('Time [s]'); grid on;
legend('Baseline Power', 'Predictive Power', 'MPC Power');
title('Active Cooling Power Applied Across Controller Architectures');



%% ========================================================================
%% PART 5E: STEP 6 - COMPREHENSIVE CONTROLLER BENCHMARKING & ENERGY EVALUATION
%% ========================================================================
fprintf('\nComputing Step 6 Comparative Benchmarks & Advanced Degradation Metrics...\n');

% -------------------------------------------------------------------------
% 1. THERMAL PERFORMANCE & LIMIT VIOLATION METRICS
% -------------------------------------------------------------------------
T_peak_uncooled = max(T_batt);
T_peak_base     = max(T_batt_baseline);
T_peak_pred     = max(T_batt_predictive);
T_peak_mpc      = max(T_batt_MPC);

% Time spent above safety limit (35°C) and target setpoint (32°C) in seconds
time_above_35_uncooled = sum(T_batt > 35.0) * dt;
time_above_35_base     = sum(T_batt_baseline > 35.0) * dt;
time_above_35_pred     = sum(T_batt_predictive > 35.0) * dt;
time_above_35_mpc      = sum(T_batt_MPC > 35.0) * dt;

time_above_32_base     = sum(T_batt_baseline > 32.0) * dt;
time_above_32_pred     = sum(T_batt_predictive > 32.0) * dt;
time_above_32_mpc      = sum(T_batt_MPC > 32.0) * dt;

% -------------------------------------------------------------------------
% 2. ENERGY CONSUMPTION & SAVINGS CALCULATIONS
% -------------------------------------------------------------------------
% Energy values already computed in earlier parts (Wh)
E_base_Wh = E_cooling_baseline_Wh;
E_pred_Wh = E_cooling_pred_Wh;
E_mpc_Wh  = E_cooling_MPC_Wh;

% Relative Energy Savings vs Step 4 Reactive Baseline (%)
savings_pred_pct = ((E_base_Wh - E_pred_Wh) / E_base_Wh) * 100;
savings_mpc_pct  = ((E_base_Wh - E_mpc_Wh) / E_base_Wh) * 100;

% Cumulative Chiller Electrical Energy Trajectories over time (Wh)
E_cum_base = cumtrapz(t, (Q_cooling_baseline / COP_chiller) + (Q_cooling_baseline > 0) * P_pump) / 3600;
E_cum_pred = cumtrapz(t, (Q_cooling_predictive / COP_chiller) + (Q_cooling_predictive > 0) * P_pump) / 3600;
E_cum_mpc  = cumtrapz(t, (Q_cooling_MPC / COP_chiller) + (Q_cooling_MPC > 0) * P_pump) / 3600;

% -------------------------------------------------------------------------
% 3. ACTUATOR WEAR & SMOOTHNESS INDEX (Mean |dQ/dt|)
% -------------------------------------------------------------------------
wear_base = mean(abs(diff(Q_cooling_baseline))) / dt;
wear_pred = mean(abs(diff(Q_cooling_predictive))) / dt;
wear_mpc  = mean(abs(diff(Q_cooling_MPC))) / dt;

% -------------------------------------------------------------------------
% 4. ADVANCED WORK: BATTERY DEGRADATION & SOH IMPACT (Arrhenius Model)
% -------------------------------------------------------------------------
% Empirical SEI Growth Kinetic parameters
E_a = 31700;    % Activation energy (J/mol)
R_gas = 8.314;  % Universal gas constant (J/mol*K)
A_pre = 1000;   % Pre-exponential scaling constant

% Instantaneous capacity degradation rate k_deg(T)
k_deg_uncooled = A_pre * exp(-E_a ./ (R_gas * (T_batt + 273.15)));
k_deg_base     = A_pre * exp(-E_a ./ (R_gas * (T_batt_baseline + 273.15)));
k_deg_pred     = A_pre * exp(-E_a ./ (R_gas * (T_batt_predictive + 273.15)));
k_deg_mpc      = A_pre * exp(-E_a ./ (R_gas * (T_batt_MPC + 273.15)));

% Cumulative aging factor relative to 25°C reference
aging_uncooled = trapz(t, k_deg_uncooled);
aging_base     = trapz(t, k_deg_base);
aging_pred     = trapz(t, k_deg_pred);
aging_mpc      = trapz(t, k_deg_mpc);

% Relative life extension vs Reactive Baseline (%)
life_ext_pred_pct = ((aging_base - aging_pred) / aging_base) * 100;
life_ext_mpc_pct  = ((aging_base - aging_mpc) / aging_base) * 100;

% Estimated full-equivalent battery cycle life extension
est_cycles_base = 1500; % Baseline reference cycles under thermal stress
est_cycles_mpc  = est_cycles_base * (aging_base / aging_mpc);

% -------------------------------------------------------------------------
% 5. ADVANCED WORK: VEHICLE RANGE EXTENSION IMPACT
% -------------------------------------------------------------------------
dist_total_km = trapz(t, v) / 1000;
E_traction_Wh = trapz(t, P_drive - P_regen) / 3600;
spec_cons_Wh_km = E_traction_Wh / max(dist_total_km, 0.001); % Energy per km

% Additional driving range gained by reducing active chiller draw
range_gain_pred_km = (E_base_Wh - E_pred_Wh) / spec_cons_Wh_km;
range_gain_mpc_km  = (E_base_Wh - E_mpc_Wh) / spec_cons_Wh_km;

% -------------------------------------------------------------------------
% 6. PRINT STEP 6 BENCHMARK COMPARISON TABLE
% -------------------------------------------------------------------------
fprintf('\n=========================================================================================\n');
fprintf('                 STEP 6: COMPREHENSIVE THERMAL CONTROLLER BENCHMARK                     \n');
fprintf('=========================================================================================\n');
fprintf('%-30s | %-10s | %-12s | %-12s | %-10s\n', 'Metric', 'Uncooled', 'Step 4 Base', 'Step 5 Pred', 'Step 5 MPC');
fprintf('-----------------------------------------------------------------------------------------\n');
fprintf('%-30s | %-10.2f | %-12.2f | %-12.2f | %-10.2f\n', 'Peak Temperature (°C)', T_peak_uncooled, T_peak_base, T_peak_pred, T_peak_mpc);
fprintf('%-30s | %-10.0f | %-12.0f | %-12.0f | %-10.0f\n', 'Time > 35°C Limit (s)', time_above_35_uncooled, time_above_35_base, time_above_35_pred, time_above_35_mpc);
fprintf('%-30s | %-10.0f | %-12.0f | %-12.0f | %-10.0f\n', 'Time > 32°C Setpoint (s)', sum(T_batt > 32.0)*dt, time_above_32_base, time_above_32_pred, time_above_32_mpc);
fprintf('%-30s | %-10s | %-12.2f | %-12.2f | %-10.2f\n', 'Chiller Energy (Wh)', 'N/A', E_base_Wh, E_pred_Wh, E_mpc_Wh);
fprintf('%-30s | %-10s | %-12s | %-12.2f | %-10.2f\n', 'Energy Savings vs Base (%)', 'N/A', 'Baseline', savings_pred_pct, savings_mpc_pct);
fprintf('%-30s | %-10s | %-12.2f | %-12.2f | %-10.2f\n', 'Actuator Chatter (W/s)', 'N/A', wear_base, wear_pred, wear_mpc);
fprintf('%-30s | %-10s | %-12s | %-12.2f | %-10.2f\n', 'Battery Aging Reduction (%)', 'N/A', 'Baseline', life_ext_pred_pct, life_ext_mpc_pct);
fprintf('%-30s | %-10s | %-12.3f | %-12.3f | %-10.3f\n', 'Added Range Gained (km)', 'N/A', 0.0, range_gain_pred_km, range_gain_mpc_km);
fprintf('=========================================================================================\n\n');

% -------------------------------------------------------------------------
% 7. STEP 6 DIAGNOSTIC VISUALIZATIONS
% -------------------------------------------------------------------------
figure('Name', 'Step 6: Comprehensive Controller Benchmarking Analysis', 'NumberTitle', 'off');

% Subplot 1: Temperature Trajectories vs Safety & Setpoint Limits
subplot(3,1,1);
plot(t, T_batt, 'r--', 'LineWidth', 1.2); hold on;
plot(t, T_batt_baseline, 'b:', 'LineWidth', 1.5);
plot(t, T_batt_predictive, 'g--', 'LineWidth', 1.5);
plot(t, T_batt_MPC, 'm', 'LineWidth', 2.0);
yline(35.0, 'k--', 'Safety Threshold (35°C)', 'LineWidth', 1.2);
yline(32.0, 'c--', 'Target Setpoint (32°C)', 'LineWidth', 1.2);
ylabel('Temp [°C]'); grid on;
title('1. Temperature Profile Comparison across Controller Architectures');
legend('Uncooled', 'Step 4 Reactive Baseline', 'Step 5 Predictive', 'Step 5 Full MPC', 'Location', 'northwest');

% Subplot 2: Cumulative HVAC Electrical Energy Draw
subplot(3,1,2);
plot(t, E_cum_base, 'b:', 'LineWidth', 1.5); hold on;
plot(t, E_cum_pred, 'g--', 'LineWidth', 1.5);
plot(t, E_cum_mpc, 'm', 'LineWidth', 2.0);
ylabel('Energy [Wh]'); grid on;
title('2. Cumulative Cooling Electrical Energy Draw (Chiller + Pump)');
legend('Step 4 Baseline', 'Step 5 Predictive', 'Step 5 Full MPC', 'Location', 'northwest');

% Subplot 3: Relative Battery Degradation Kinetics k_deg(T)
subplot(3,1,3);
plot(t, k_deg_base, 'b:', 'LineWidth', 1.5); hold on;
plot(t, k_deg_pred, 'g--', 'LineWidth', 1.5);
plot(t, k_deg_mpc, 'm', 'LineWidth', 2.0);
ylabel('k_{deg} Rate'); xlabel('Time [s]'); grid on;
title('3. Instantaneous Battery Aging Rate (Arrhenius SEI Growth Model)');
legend('Step 4 Baseline', 'Step 5 Predictive', 'Step 5 Full MPC', 'Location', 'northwest');


%% ========================================================================
%% PART 6: KALMAN FILTER SOC ESTIMATOR
%% ========================================================================
noise = randn(N, 1) * 0.5;
V_meas = V_batt + noise;
SOC_est = zeros(N, 1);
SOC_est(1) = SOC_init;

P_cov = 1e-4;   % Estimation covariance
Q_proc = 1e-6;  % Process noise
R_meas = 0.5;   % Measurement noise

for k = 2:N
    % SOC prediction step
    SOC_pred = SOC_est(k-1) - (I_batt(k) * dt) / (Batt_Cap_Ah * 3600);
    P_cov = P_cov + Q_proc;

    % Measurement prediction
    V_oc_pred = interp1(SOC_curve, OCV_curve, SOC_pred, 'linear', 'extrap');
    H = 100; % Sensitivity linear fit factor

    % Kalman gain calculation
    K = P_cov * H / (H * P_cov * H + R_meas);

    % State update
    SOC_est(k) = SOC_pred + K * (V_meas(k) - V_oc_pred);
    SOC_est(k) = max(0.05, min(1.0, SOC_est(k)));
    P_cov = (1 - K * H) * P_cov;
end

%% ========================================================================
%% PART 7: PERFORMANCE & ENERGY METRICS PRINT OUT
%% ========================================================================
dist_km = trapz(t, v) / 1000;
E_drive_Wh = trapz(t, P_drive) / 3600;
E_regen_Wh = trapz(t, P_regen) / 3600;
E_batt_net = E_drive_Wh - E_regen_Wh;
T_max = max(T_batt);
T_rise = T_max - T_amb(1);

fprintf('\n========================================================\n');
fprintf('         UNIFIED EV THERMAL SIMULATION SUMMARY           \n');
fprintf('========================================================\n');
fprintf('Total Duration              : %.0f s\n', t(end));
fprintf('Total Distance Travelled    : %.3f km\n', dist_km);
fprintf('Net Battery Energy Consumed : %.2f Wh\n', E_batt_net);
fprintf('Peak Heat Generation        : %.2f W\n', max(Q_heat));
fprintf('Peak Ohmic Heat Loss        : %.2f W\n', max(Q_ohmic));
fprintf('Maximum Battery Temperature : %.2f °C\n', T_max);
fprintf('Temperature Rise (Uncooled) : %.2f °C\n', T_rise);
fprintf('Initial SOC                 : %.2f %%\n', SOC(1)*100);
fprintf('Final SOC                   : %.2f %%\n', SOC(end)*100);
fprintf('========================================================\n\n');

%% ========================================================================
%% PART 8: SYSTEM VISUALIZATION PLOTS (ANNOTATED SCENARIOS)
%% ========================================================================
figure('Name', 'Step 3: Multi-Phase Dynamic Load Profiles', 'NumberTitle', 'off');

% --- Subplot 1: Speed ---
subplot(4,1,1);
plot(t, v * 3.6, 'b', 'LineWidth', 1.5); hold on;
xline(600, 'k--', 'End of Driving');
ylabel('Speed [km/h]'); grid on;
title('Scenario 1: Vehicle Speed Profile (Aggressive Driving)');

% --- Subplot 2: Current ---
subplot(4,1,2);
plot(t, I_batt, 'r', 'LineWidth', 1.5); hold on;
xline(700, 'g--', 'Fast Charge Start');
xline(1600, 'g--', 'Fast Charge End');
ylabel('Current [A]'); grid on;
title('Scenario 1 & 2: Battery Current (Drive/Regen Pulses & 300A Charge Surge)');

% --- Subplot 3: Heat Rate ---
subplot(4,1,3);
plot(t, Q_ohmic, 'r--', 'LineWidth', 1.2); hold on;
plot(t, Q_entropic, 'g--', 'LineWidth', 1.2);
plot(t, Q_heat, 'k', 'LineWidth', 1.5);
xline(700, 'b--', 'Scenario 2 Spike');
ylabel('Heat Rate [W]'); grid on;
legend('Q_{ohmic}', 'Q_{entropic}', 'Q_{total}');
title('Heat Generation Decomposition (Ohmic + Entropic Formulation)');

% --- Subplot 4: Thermal Response ---
subplot(4,1,4);
plot(t, T_amb, 'm', 'LineWidth', 1.5); hold on;
plot(t, T_batt, 'c', 'LineWidth', 1.5);
yline(35, 'r--', 'Thermal Safety Limit (35°C)');
xline(700, 'k--', 'Scenario 2 Heatup');
xlabel('Time [s]'); ylabel('Temp [°C]'); grid on;
legend('T_{ambient}', 'T_{batt} (Uncooled)');
title('Scenario 3: Dynamic Ambient & Uncooled Pack Thermal Response');

figure('Name', 'Electrical & State of Charge Tracking', 'NumberTitle', 'off');
subplot(2,1,1);
plot(t, V_batt, 'b', 'LineWidth', 1.5); grid on;
ylabel('Terminal Voltage [V]');
title('Battery Voltage Behavior Under Dynamic Loads');

subplot(2,1,2);
plot(t, SOC * 100, 'k', 'LineWidth', 2); hold on;
plot(t, SOC_est * 100, 'r--', 'LineWidth', 1.5);
ylabel('SOC [%]'); xlabel('Time [s]'); grid on;
legend('True SOC', 'Kalman Estimated SOC');
title('State of Charge Estimation Tracking');

%% ========================================================================
%% PART 9: EXPORT SIGNALS FOR SIMULINK / SIMSCAPE PLANT
%% ========================================================================
Q_heat_ts = timeseries(Q_heat, t);
Q_heat_ts.Name = 'Q_heat_ts';
v_ts = timeseries(v, t);
v_ts.Name = 'v_ts';
SOC_ts = timeseries(SOC, t);
SOC_ts.Name = 'SOC_ts';
I_batt_ts = timeseries(I_batt, t);
I_batt_ts.Name = 'I_batt_ts';
% Convert Ambient Temperature to Kelvin for Simscape physical block
T_amb_ts = timeseries(T_amb + 273.15, t);
T_amb_ts.Name = 'T_amb_ts';
% Save workspace variables to EV_Thermal_Inputs.mat
save('EV_Thermal_Inputs.mat', 'Q_heat_ts', 'v_ts', 'SOC_ts', 'T_amb_ts', 'I_batt_ts');
fprintf('[SUCCESS] All time-series objects exported to Base Workspace & EV_Thermal_Inputs.mat!\n');

%% ========================================================================
%% MPC OBJECTIVE & STRICT NONLINEAR CONSTRAINT FUNCTIONS
%% (MUST BE AT THE VERY BOTTOM OF THE SCRIPT FILE)
%% ========================================================================
function J = mpc_cost_function(u, T_0, Q_h, T_a, u_prev, C_th, R_th, dt, H_p, T_set, w_T, w_E, w_dU)
    T_pred = zeros(H_p + 1, 1);
    T_pred(1) = T_0;
    J = 0;
    
    for k = 1:H_p
        Q_pas = (T_pred(k) - T_a(k)) / R_th;
        dT = (Q_h(k) - Q_pas - u(k)) / C_th;
        T_pred(k+1) = T_pred(k) + dT * dt;
        
        e_T = T_pred(k+1) - T_set;
        
        if k == 1
            du = u(k) - u_prev;
        else
            du = u(k) - u(k-1);
        end
        
        J = J + w_T * (e_T^2) + w_E * (u(k)^2) + w_dU * (du^2);
    end
end

function [c, ceq] = mpc_nonlcon(u, T_0, Q_h, T_a, C_th, R_th, dt, H_p, T_max)
    T_pred = zeros(H_p + 1, 1);
    T_pred(1) = T_0;
    
    for k = 1:H_p
        Q_pas = (T_pred(k) - T_a(k)) / R_th;
        dT = (Q_h(k) - Q_pas - u(k)) / C_th;
        T_pred(k+1) = T_pred(k) + dT * dt;
    end
    
    % Enforce T_pred <= 35.0 °C strictly at all prediction steps
    c = T_pred(2:end) - T_max; 
    ceq = [];
end





% =========================================================================
% RUNTIME BENCHMARKING
% =========================================================================
elapsed_time = toc; % Stop execution timer

fprintf('\n=======================================================\n');
fprintf('  Total Script Execution Time: %.3f seconds\n', elapsed_time);
fprintf('=======================================================\n');