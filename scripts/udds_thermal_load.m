clc;
clear;
close all;



%% =================================================
%% PART 1: VEHICLE PARAMETERS
%% =================================================
m = 1800;                 % Vehicle mass (kg)
g = 9.81;                 % Gravity (m/s^2)
Crr = 0.01;               % Rolling resistance coefficient
rho = 1.225;              % Air density (kg/m^3)
Cd = 0.29;                % Drag coefficient
A = 2.2;                  % Frontal area (m^2)

delta = 0.08;             % Rotational Mass factor
m_eq = m*(1 + delta);

r_wheel = 0.3;            % Wheel radius (m)

% BATTERY PARAMETERS

Batt_capacity = 50;   % Battery capacity (kWh)

SOC_init = 0.9;       % Initial SOC (90%)

%% =================================================
%% PART 2: IMPORT UDDS DRIVE CYCLE
%% =================================================
udds_data = readtable("uddsdc.csv");

t = udds_data.UDDS;       % Time (s)
v_mph = udds_data.Var2;   % Speed (mph)

v = v_mph * 0.44704;      % Convert mph → m/s

dt = mean(diff(t));

a = [diff(v)/dt; 0];      % Acceleration

% Acceleration limits
a_max = 3;
a_min = -4;

a(a>a_max) = a_max;
a(a<a_min) = a_min;

%% =================================================
%% PART 3: LONGITUDINAL FORCES
%% =================================================

gear_ratio = 9;

omega_wheel = v / r_wheel;
omega = omega_wheel * gear_ratio;

F_rr   = m * g * Crr;
F_aero = 0.5 * rho * Cd * A .* v.^2;
F_acc  = m_eq .* a;

theta = deg2rad(0);
F_rg = m*g*sin(theta);

F_total = F_rr + F_aero + F_acc + F_rg;

%% =================================================
%% PART 4: WHEEL MECHANICAL POWER
%% =================================================

P_wheel = F_total .* v;

eta_tire = 0.95;
P_wheel = P_wheel / eta_tire;

%% =================================================
%% PART 5: MOTOR + REGEN LIMITS
%% =================================================

T_motor_max = 250;
T_regen_max = 120;
P_motor_max = 80e3;

v_min_regen = 1.5;

eta_drivetrain = 0.95;

P_mech = zeros(size(P_wheel));

omega_base = P_motor_max / T_motor_max;

for i = 1:length(P_wheel)

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

% drivetrain efficiency
P_mech = P_mech / eta_drivetrain;

%% =================================================
%% PART 6: DISTANCE & IDEAL ENERGY
%% =================================================

E_Wh = trapz(t, P_mech) / 3600;

dist_km = trapz(t, v) / 1000;

E_Wh_per_km = E_Wh / dist_km;

%% =================================================
%% PART 7: MOTOR + INVERTER EFFICIENCY
%% =================================================

omega_max = max(omega) + eps;

drive_eff = ones(size(P_mech));
regen_eff = ones(size(P_mech));

for i = 1:length(P_mech)

    if P_mech(i) > 0

        T_req = P_mech(i) / max(omega(i),eps);

        Tn = abs(T_req) / T_motor_max;

        on = omega(i) / omega_max;

        drive_eff(i) = 0.90 ...
            - 0.15*(1-on)^2 ...
            - 0.20*(Tn-0.6)^2;

        drive_eff(i) = min(max(drive_eff(i),0.75),0.92);

    elseif P_mech(i) < 0

        T_req = abs(P_mech(i)) / max(omega(i),eps);

        Tn = T_req / T_regen_max;

        on = omega(i) / omega_max;

        regen_eff(i) = 0.75 ...
            - 0.20*(1-on)^2 ...
            - 0.30*(Tn-0.5)^2;

        regen_eff(i) = min(max(regen_eff(i),0.55),0.80);

    end

end

%% =================================================
%% PART 8A: BATTERY POWER FLOW
%% =================================================

P_batt_disch_max = 90e3;
P_batt_char_max  = 40e3;

P_accessory = 900;

P_drive = zeros(size(P_mech));
P_regen = zeros(size(P_mech));

for i = 1:length(P_mech)

    if P_mech(i) > 0

        P_drive(i) = P_mech(i) / drive_eff(i);

        P_drive(i) = min(P_drive(i), P_batt_disch_max);

    elseif P_mech(i) < 0

        P_regen(i) = abs(P_mech(i)) * regen_eff(i);

        P_regen(i) = min(P_regen(i), P_batt_char_max);

    end

end

P_batt = P_drive - P_regen + P_accessory;

%% =================================================
%% PART 8B: BATTERY THERMAL PARAMETERS
%% =================================================


projectRoot = fileparts(fileparts(mfilename('fullpath')));
sourceFolder = fullfile(projectRoot,'source_powertrain', ...
    'Mini Project 1 (Electric Vehicle Energy Consumption Calculator)');

udds_data = readtable(fullfile(sourceFolder,'uddsdc.csv'));


V_nom = 350;        % Nominal battery voltage (V)

R_internal = 0.05;  % Internal resistance (Ohm)

% BATTERY CURRENT
I_batt = P_batt ./ V_nom;

% HEAT GENERATION
Q_heat = (I_batt.^2) * R_internal;

%% Thermal interface for the cooling project
thermalLoad = timetable(seconds(t), P_batt, I_batt, Q_heat);

figure
plot(t,Q_heat/1000,'LineWidth',1.2)
xlabel('Time (s)')
ylabel('Battery heat generation (kW)')
title('UDDS Battery Thermal Load')
grid on