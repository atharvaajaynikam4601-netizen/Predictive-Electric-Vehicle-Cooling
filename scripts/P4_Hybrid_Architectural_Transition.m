clc; clear; close all;

%% =================================================
%% PART 1: VEHICLE & POWERTRAIN PARAMETERS
%% =================================================
m = 1800; g = 9.81; Crr = 0.01; rho = 1.225; Cd = 0.29; A = 2.2; r_wheel = 0.3;

% ICE Parameters (Front Axle - P1/Engine)
P_ice_max = 100e3;        % 100 kW Engine
G_front = 4.0;            % Front Differential Ratio
ice_eff = 0.30;           % Brake Thermal Efficiency (Avg)
LHV = 44e6;               % Lower Heating Value of Petrol (J/kg)

% Motor Parameters (Rear Axle - P4 Motor)
T_motor_max = 250;        % Max drive torque (Nm)
T_regen_max = 150;        % Max regen torque (Nm)
P_motor_max = 80e3;       % Max motor power (W)
G_rear = 8.5;             % Rear Reduction Ratio
v_min_regen = 1.5;        % Min speed for regen (m/s)

% Battery & Accessories
Batt_capacity_kWh = 15;   % Typical PHEV battery (15 kWh)
P_accessory = 900;        % 900W constant load (AC, Sensors)

%% =================================================
%% PART 2: IMPORT UDDS DRIVE CYCLE
%% =================================================
% Ensure "uddsdc.csv" is in your current MATLAB folder
udds_data = readtable("uddsdc.csv");
t = udds_data.UDDS;       % Adjust header name if needed
v = udds_data.Var2 * 0.44704; % mph to m/s
dt = mean(diff(t));
a = [diff(v)/dt; 0];      
F_total = (m*g*Crr) + (0.5*rho*Cd*A.*v.^2) + (m.*a);

%% =================================================
%% PART 3: INTEGRATED SIMULATION LOOP (THE "BRAIN" & "BODY")
%% =================================================


% Preallocate for speed and avoid "Unrecognized Variable" errors
F_ice = zeros(size(t));  F_em = zeros(size(t));
P_em_batt = zeros(size(t)); P_fuel_W = zeros(size(t));
SOC = zeros(size(t));    SOC(1) = 0.80; % Starting at 80% | SOC(1) = 0.80; try this 

omega_max_motor = max(v/r_wheel * G_rear) + eps;

for i = 1:length(t)-1
    % --- 1. HEURISTIC EMS (Torque Split Logic) ---
    if F_total(i) > 0 % DRIVING
        if SOC(i) > 0.25 && v(i) < 15 % EV Mode: SOC > 25% and Speed < 54 km/h
            F_em_req = F_total(i);
            F_em(i) = min(F_em_req, (T_motor_max * G_rear)/r_wheel);
            F_ice(i) = max(0, F_total(i) - F_em(i));
        else % HYBRID MODE (70/30 Split)
            F_ice(i) = F_total(i) * 0.7;
            F_em(i) = F_total(i) * 0.3;
        end
    else % BRAKING (Regen)
        if v(i) > v_min_regen
            F_em(i) = max(F_total(i), -(T_regen_max * G_rear)/r_wheel);
        else
            F_em(i) = 0; % Below min speed, use friction brakes only
        end
        F_ice(i) = 0;
    end

    % --- 2. MOTOR EFFICIENCY MODEL (Dynamic) ---
    w_m = (v(i) / r_wheel) * G_rear;
    T_m = (F_em(i) * r_wheel) / G_rear;
    Tn = abs(T_m) / T_motor_max;
    on = w_m / omega_max_motor;

    if F_em(i) >= 0 % Motoring
        eff = min(max(0.90 - 0.15*(1-on)^2 - 0.20*(Tn-0.6)^2, 0.75), 0.92);
        P_em_batt(i) = (F_em(i) * v(i)) / eff;
    else % Regenerating
        eff = min(max(0.75 - 0.20*(1-on)^2 - 0.30*(Tn-0.5)^2, 0.55), 0.80);
        P_em_batt(i) = (F_em(i) * v(i)) * eff; % Becomes negative
    end

    % --- 3. ICE FUEL FLOW ---
    P_ice_mech = F_ice(i) * v(i);
    P_fuel_W(i) = P_ice_mech / ice_eff;

    % --- 4. BATTERY UPDATE ---
    P_batt = P_em_batt(i) + P_accessory;
    E_step_kWh = (P_batt * dt) / (3600 * 1000);
    SOC(i+1) = SOC(i) - (E_step_kWh / Batt_capacity_kWh);
    
    % Safety clip for SOC
    SOC(i+1) = min(max(SOC(i+1), 0), 1);
end

%% =================================================
%% PART 4: RESULTS & PUBLISHABLE METRICS
%% =================================================
dist_km = trapz(t, v) / 1000;
Fuel_kg = trapz(t, P_fuel_W) / LHV;
Fuel_Liters = (Fuel_kg / 0.74); % Approx density of petrol

fprintf('\n--- P4 HYBRID RESEARCH RESULTS ---\n');
fprintf('Total Distance       : %.2f km\n', dist_km);
fprintf('Final SOC            : %.2f %%\n', SOC(end)*100);
fprintf('Total Fuel Used      : %.3f Liters\n', Fuel_Liters);
fprintf('Fuel Consumption     : %.2f L/100km\n', (Fuel_Liters/dist_km)*100);

% Visualization
figure('Name', 'P4 Hybrid Performance Analysis');
subplot(3,1,1); 
plot(t, v*3.6, 'r', 'LineWidth', 1.2); 
ylabel('Speed (km/h)'); title('UDDS Drive Cycle Trace'); grid on;

subplot(3,1,2); 
plot(t, F_ice, 'k', 'LineWidth', 1); hold on;
plot(t, F_em, 'b', 'LineWidth', 1); 
ylabel('Tractive Force (N)'); title('P4 Torque Split: Front (ICE) vs Rear (EM)');
legend('Front Axle (ICE)', 'Rear Axle (EM)'); grid on;

subplot(3,1,3); 
plot(t, SOC*100, 'g', 'LineWidth', 1.5); 
ylabel('SOC (%)'); title('Battery State of Charge Evolution'); grid on;