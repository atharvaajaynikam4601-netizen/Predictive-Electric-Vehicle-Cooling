%% Create a repeatable hot-day, high-load EV scenario
% This is an engineering test scenario for controller development. It is not
% a measured drive cycle. Assumptions are deliberately explicit so they can
% later be replaced by logged or standardized drive-cycle data.

projectRoot = fileparts(fileparts(mfilename('fullpath')));
dataFolder = fullfile(projectRoot,'data');

dt = 1;                         % s
t = (0:dt:3000)';               % 50 min
g = 9.81;                       % m/s^2
m = 1900;                       % kg, vehicle including occupants
rhoAir = 1.184;                 % kg/m^3, warm air
CdA = 0.65;                     % m^2
Crr = 0.010;                    % rolling-resistance coefficient
etaDrive = 0.90;                % traction efficiency

% 0-600 s: urban; 600-660 s: transition to highway; 660-1200 s: highway;
% 1200-1260 s: transition to hill; 1260-1800 s: hilly high demand;
% 1800-1860 s: smooth stop; 1860-3000 s: parked DC fast charge.
% The transitions prevent nonphysical one-second speed jumps and the resulting
% unrealistic traction-power spikes.
speedKph = zeros(size(t));
urban = t <= 600;
rampToHighway = t > 600 & t <= 660;
highway = t > 660 & t <= 1200;
rampToHill = t > 1200 & t <= 1260;
hill = t > 1260 & t <= 1800;
rampToStop = t > 1800 & t <= 1860;

urbanSpeed = @(x) max(0,30 + 22*sin(2*pi*x/95));
highwaySpeed = @(x) 92 + 8*sin(2*pi*x/180);
hillSpeed = @(x) 76 + 16*sin(2*pi*x/75);

speedKph(urban) = urbanSpeed(t(urban));
speedKph(highway) = highwaySpeed(t(highway));
speedKph(hill) = hillSpeed(t(hill));

blend = @(x) x.^2.*(3 - 2*x);  % smooth step with zero end slopes
s = blend((t(rampToHighway)-600)/60);
speedKph(rampToHighway) = (1-s)*urbanSpeed(600) + s*highwaySpeed(660);
s = blend((t(rampToHill)-1200)/60);
speedKph(rampToHill) = (1-s)*highwaySpeed(1200) + s*hillSpeed(1260);
s = blend((t(rampToStop)-1800)/60);
speedKph(rampToStop) = (1-s)*hillSpeed(1800);
speed = speedKph / 3.6;

grade = zeros(size(t));
hillGrade = @(x) 0.045 + 0.025*sin(2*pi*x/180);
grade(hill) = hillGrade(t(hill));
s = blend((t(rampToHill)-1200)/60);
grade(rampToHill) = s*hillGrade(1260);
s = blend((t(rampToStop)-1800)/60);
grade(rampToStop) = (1-s)*hillGrade(1800);
acceleration = [0; diff(speed)/dt];

rollingForce = m*g*Crr;
aeroForce = 0.5*rhoAir*CdA.*speed.^2;
gradeForce = m*g.*grade;
inertialForce = m.*acceleration;
wheelPower = (rollingForce + aeroForce + gradeForce + inertialForce).*speed;

% Positive power is battery discharge. Regenerative power is capped at 30 kW.
tractionPower_W = max(wheelPower,0)/etaDrive;
regenPower_W = min(wheelPower,0)*0.65;
batteryPower_W = tractionPower_W + regenPower_W;

% A 150 kW DC fast-charge event after the drive. Negative is battery charge.
fastCharge = t > 1860;
batteryPower_W(fastCharge) = -150e3;

ambientTemp_C = 33 + 2*sin(2*pi*t/1800);  % 31-35 C hot-day profile
scenario = timetable(seconds(t),speedKph,grade,acceleration,batteryPower_W, ...
    ambientTemp_C,'VariableNames',{'Speed_kph','RoadGrade','Acceleration_mps2', ...
    'BatteryPower_W','AmbientTemp_C'});

save(fullfile(dataFolder,'hot_hill_scenario.mat'),'scenario','dt');

fig = figure('Name','Hot-Hill and Fast-Charge Development Scenario','Color','w');
tiledlayout(3,1,'TileSpacing','compact');
nexttile; plot(t/60,speedKph,'LineWidth',1.2); ylabel('Speed (km/h)'); grid on
nexttile; plot(t/60,100*grade,'LineWidth',1.2); ylabel('Grade (%)'); grid on
nexttile; yyaxis left; plot(t/60,batteryPower_W/1000,'LineWidth',1.2); ylabel('Battery power (kW)')
ylim([-200 250])
yyaxis right; plot(t/60,ambientTemp_C,'LineWidth',1.2); ylabel('Ambient (°C)')
xlabel('Time (min)'); grid on

fprintf('Saved scenario to %s\n', fullfile(dataFolder,'hot_hill_scenario.mat'));
resultsFolder = fullfile(projectRoot,'results');
savefig(fig,fullfile(resultsFolder,'hot_hill_fast_charge_scenario.fig'));
exportgraphics(fig,fullfile(resultsFolder,'hot_hill_fast_charge_scenario.png'),'Resolution',200);
fprintf('Saved plot to %s\n', resultsFolder);
