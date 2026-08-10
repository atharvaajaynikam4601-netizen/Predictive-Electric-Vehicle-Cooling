%% Predictive EV cooling study — code-first, learning-oriented model
% This script uses the real UDDS speed trace from the earlier EV project,
% appends a DC fast-charge event, and compares reactive and predictive
% battery-cooling control. Read one section at a time and relate each
% equation to the physical system before changing its parameters.

clear; close all; clc

%% 1. Load the real UDDS speed trace
projectRoot = fileparts(fileparts(mfilename('fullpath')));
uddsFile = fullfile(projectRoot,'source_powertrain', ...
    'Mini Project 1 (Electric Vehicle Energy Consumption Calculator)', ...
    'uddsdc.csv');
uddsData = readtable(uddsFile);

tDrive = uddsData{:,1};                  % s
speed_mps = uddsData{:,2} * 0.44704;     % mph -> m/s
dt = median(diff(tDrive));               % s

%% 2. Convert UDDS vehicle motion into battery power
% Longitudinal dynamics: wheel force = rolling + aerodynamic + inertia.
mVehicle = 1800;                         % kg
g = 9.81;                                % m/s^2
Crr = 0.010;
rhoAir = 1.225;                          % kg/m^3
Cd = 0.29;
frontalArea = 2.2;                       % m^2
rotationalMassFactor = 1.08;

accel_mps2 = [diff(speed_mps)./diff(tDrive); 0];
accel_mps2 = min(max(accel_mps2,-4),3); % realistic acceleration limits

Frolling = mVehicle*g*Crr;
Faero = 0.5*rhoAir*Cd*frontalArea*speed_mps.^2;
Finertia = rotationalMassFactor*mVehicle.*accel_mps2;
Pwheel_W = (Frolling + Faero + Finertia).*speed_mps;

% Positive battery power means discharge. Negative means charge.
motorEfficiency = 0.90;
regenEfficiency = 0.70;
Paccessory_W = 900;
PdriveMax_W = 90e3;
PregenMax_W = 40e3;

Ptraction_W = min(max(Pwheel_W,0)/motorEfficiency,PdriveMax_W);
Pregen_W = min(max(-Pwheel_W,0)*regenEfficiency,PregenMax_W);
PbattDrive_W = Ptraction_W - Pregen_W + Paccessory_W;

%% 3. Append a realistic DC fast-charge event
% UDDS alone does not exercise the high charging thermal load required by
% the challenge. The vehicle is stationary for a 20-minute, 150 kW charge.
tCharge = (tDrive(end)+dt:dt:tDrive(end)+1200)';
Pcharge_W = -150e3*ones(size(tCharge)); % negative: energy enters pack

t = [tDrive; tCharge];
speed_mps = [speed_mps; zeros(size(tCharge))];
Pbatt_W = [PbattDrive_W; Pcharge_W];
isCharging = [false(size(tDrive)); true(size(tCharge))];

%% 4. Electrical heat generation: P_loss = I^2 R
Vnom = 350;                              % V, pack nominal voltage
Rinternal = 0.050;                       % ohm, effective pack resistance
Ibatt_A = Pbatt_W/Vnom;
Qgen_W = Ibatt_A.^2*Rinternal;

%% 5. Two-state thermal plant
% State 1: battery temperature Tb.
% State 2: coolant temperature Tc.
% Qplate is the heat transferred from battery to coolant.
% Qchiller and Qradiator remove heat from coolant.
C_batt = 250e3;                          % J/K, pack thermal capacitance
C_coolant = 18e3;                        % J/K, coolant-loop thermal mass
UAplateMin = 80;                         % W/K, pump almost off
UAplateMax = 1000;                       % W/K, pump fully on
UAradiator = 250;                        % W/K, hot-day radiator conductance
QchillerMax = 15e3;                      % W, maximum chiller cooling capacity
COPchiller = 2.5;                        % cooling W / electrical W
PpumpMax = 250;                          % W at full-speed command

ambient_C = 33 + 2*sin(2*pi*t/(t(end))); % 31-35 C hot-day condition
Tinitial_C = 30;

%% 6. Controllers
% Reactive controller: waits until the measured pack temperature is high.
reactive.Ton_C = 33;
reactive.Toff_C = 30;

% Predictive controller: looks ahead at future electrical heat. It begins
% cooling before expected heat would push the battery over Tpreview.
predictive.Tpreview_C = 31;
predictive.Toff_C = 29;
predictive.horizon_s = 300;

reactiveResult = simulateCoolingPlant(t,ambient_C,Qgen_W,C_batt,C_coolant, ...
    UAplateMin,UAplateMax,UAradiator,QchillerMax,COPchiller,PpumpMax, ...
    Tinitial_C,'reactive',reactive,dt);

predictiveResult = simulateCoolingPlant(t,ambient_C,Qgen_W,C_batt,C_coolant, ...
    UAplateMin,UAplateMax,UAradiator,QchillerMax,COPchiller,PpumpMax, ...
    Tinitial_C,'predictive',predictive,dt);

%% 7. Engineering metrics
% Auxiliary energy is what the cooling system consumes, not battery heat.
Ereactive_kWh = trapz(t,reactiveResult.Paux_W)/3.6e6;
Epredictive_kWh = trapz(t,predictiveResult.Paux_W)/3.6e6;
TmaxReactive_C = max(reactiveResult.Tb_C);
TmaxPredictive_C = max(predictiveResult.Tb_C);
limit_C = 40;

summary = table([TmaxReactive_C; TmaxPredictive_C], ...
    [Ereactive_kWh; Epredictive_kWh], ...
    [sum(reactiveResult.Tb_C > limit_C)*dt; sum(predictiveResult.Tb_C > limit_C)*dt], ...
    'VariableNames',{'PeakBatteryTemp_C','CoolingEnergy_kWh','SecondsAbove40C'}, ...
    'RowNames',{'Reactive','Predictive'});
disp(summary)

%% 8. Plot the causal chain: drive/charge -> current -> heat -> temperature
fig = figure('Color','w','Name','UDDS Predictive EV Cooling Study');
tiledlayout(4,1,'TileSpacing','compact');

nexttile
plot(t/60,speed_mps*3.6,'LineWidth',1.1); hold on
xline(tDrive(end)/60,'k--','Fast charge begins','LabelVerticalAlignment','bottom');
ylabel('Speed (km/h)'); grid on

nexttile
plot(t/60,Pbatt_W/1000,'LineWidth',1.1); yline(0,'k:');
ylabel('Battery power (kW)'); grid on

nexttile
plot(t/60,Qgen_W/1000,'LineWidth',1.1);
ylabel('I^2R heat (kW)'); grid on

nexttile
plot(t/60,reactiveResult.Tb_C,'LineWidth',1.4); hold on
plot(t/60,predictiveResult.Tb_C,'LineWidth',1.4);
plot(t/60,ambient_C,'k:','LineWidth',1.1);
yline(limit_C,'r--','40 C limit');
xlabel('Time (min)'); ylabel('Temperature (C)'); grid on
legend('Reactive battery','Predictive battery','Ambient','Location','best')

resultsFolder = fullfile(projectRoot,'results');
savefig(fig,fullfile(resultsFolder,'udds_predictive_cooling_study.fig'));
exportgraphics(fig,fullfile(resultsFolder,'udds_predictive_cooling_study.png'),'Resolution',200);
save(fullfile(resultsFolder,'udds_predictive_cooling_results.mat'), ...
    't','Pbatt_W','Ibatt_A','Qgen_W','ambient_C','reactiveResult','predictiveResult','summary');

fprintf('\nKey learning check:\n');
fprintf('Peak I^2R heat during UDDS + charging: %.2f kW\n',max(Qgen_W)/1000);
fprintf('Results saved in: %s\n',resultsFolder);

%% Local function: physical plant and controller simulation
function result = simulateCoolingPlant(t,Tamb,Qgen,Cb,Cc,UAmin,UAmax,UArad, ...
    QchillerMax,COP,PpumpMax,Tinitial,controllerType,controller,dt)

n = numel(t);
Tb = zeros(n,1); Tc = zeros(n,1); pump = zeros(n,1); chiller = zeros(n,1);
Paux = zeros(n,1); Qplate = zeros(n,1); Qchiller = zeros(n,1);
Tb(1) = Tinitial; Tc(1) = Tinitial;

for k = 1:n-1
    switch controllerType
        case 'reactive'
            if Tb(k) >= controller.Ton_C
                command = 1;
            elseif Tb(k) <= controller.Toff_C
                command = 0;
            else
                command = pump(max(k-1,1)); % hysteresis: hold last state
            end

        case 'predictive'
            lastIndex = min(n,k + round(controller.horizon_s/dt));
            % Adiabatic preview: the temperature rise if no cooling occurred.
            predictedRise_C = sum(Qgen(k:lastIndex))*dt/Cb;
            predictedTemp_C = Tb(k) + predictedRise_C;
            if predictedTemp_C >= controller.Tpreview_C
                command = min(1,(predictedTemp_C-controller.Tpreview_C)/3 + 0.35);
            elseif Tb(k) <= controller.Toff_C
                command = 0;
            else
                command = pump(max(k-1,1));
            end
    end

    pump(k) = command;
    chiller(k) = command;
    UAplate = UAmin + pump(k)*(UAmax-UAmin);
    Qplate(k) = UAplate*(Tb(k)-Tc(k));
    Qradiator = max(0,UArad*(Tc(k)-Tamb(k)));
    Qchiller(k) = chiller(k)*QchillerMax;

    % Explicit Euler integration of the two energy-balance equations.
    Tb(k+1) = Tb(k) + dt*(Qgen(k)-Qplate(k))/Cb;
    Tc(k+1) = Tc(k) + dt*(Qplate(k)-Qradiator-Qchiller(k))/Cc;
    Paux(k) = pump(k)^3*PpumpMax + Qchiller(k)/COP;
end

pump(end) = pump(end-1); chiller(end) = chiller(end-1);
Paux(end) = Paux(end-1); Qplate(end) = Qplate(end-1); Qchiller(end) = Qchiller(end-1);
result = struct('Tb_C',Tb,'Tc_C',Tc,'pumpCommand',pump,'chillerCommand',chiller, ...
    'Paux_W',Paux,'Qplate_W',Qplate,'Qchiller_W',Qchiller);
end
