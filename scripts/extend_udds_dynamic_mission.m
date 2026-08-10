function mission = extend_udds_dynamic_mission()
%EXTEND_UDDS_DYNAMIC_MISSION Continue the exact UDDS result with charging.
% No new vehicle motion or second powertrain model is generated here.

projectRoot = fileparts(fileparts(mfilename('fullpath')));
inputFile = fullfile(projectRoot,'data','powertrain_udds_result.mat');
if ~isfile(inputFile)
    export_udds_powertrain_result();
end
S = load(inputFile,'powertrain');
powertrain = S.powertrain;

% These are explicit test conditions, not production-vehicle claims.
charge.current_A = -300;       % A; negative means current into the battery
charge.duration_s = 900;       % s
charge.ambient_C = 25;         % same ambient condition as the UDDS case

dt = powertrain.dt_s;
tDrive = powertrain.t_s;
tCharge = (tDrive(end)+dt:dt:tDrive(end)+charge.duration_s)';
nCharge = numel(tCharge);

mission.t_s = [tDrive; tCharge];
mission.speed_mps = [powertrain.speed_mps; zeros(nCharge,1)];
mission.P_drive_W = [powertrain.P_drive_W; zeros(nCharge,1)];
mission.P_regen_W = [powertrain.P_regen_W; zeros(nCharge,1)];
mission.P_batt_W = [powertrain.P_batt_W; charge.current_A*350*ones(nCharge,1)];
mission.I_batt_A = [powertrain.I_batt_A; charge.current_A*ones(nCharge,1)];
mission.SOC = [powertrain.SOC; zeros(nCharge,1)];
mission.isFastCharge = [false(size(tDrive)); true(nCharge,1)];
mission.ambient_C = charge.ambient_C*ones(size(mission.t_s));
mission.dt_s = dt;

% Continue SOC from the final UDDS state using the existing 50 kWh/350 V
% reference parameters. This assumption is explicit for later replacement.
capacity_Ah = 50e3/350;
% Preserve every original UDDS SOC sample. Update only the appended charge.
firstChargeIndex = numel(tDrive) + 1;
for k = firstChargeIndex:numel(mission.t_s)
    if mission.isFastCharge(k)
        mission.SOC(k) = mission.SOC(k-1) - ...
            mission.I_batt_A(k)*dt/(capacity_Ah*3600);
    end
    mission.SOC(k) = min(max(mission.SOC(k),0),1);
end

save(fullfile(projectRoot,'data','udds_extended_dynamic_mission.mat'), ...
    'mission','charge');

figure('Color','w','Name','UDDS Continuation Mission');
tiledlayout(3,1,'TileSpacing','compact');
nexttile; plot(mission.t_s/60,mission.speed_mps*3.6,'LineWidth',1.1);
ylabel('Speed (km/h)'); grid on
nexttile; plot(mission.t_s/60,mission.P_batt_W/1000,'LineWidth',1.1);
yline(0,'k:'); ylabel('Battery power (kW)'); grid on
nexttile; plot(mission.t_s/60,mission.SOC*100,'LineWidth',1.1);
xlabel('Time (min)'); ylabel('SOC (%)'); grid on

fprintf('Extended mission: %.1f s UDDS + %.1f s fast charge.\n', ...
    tDrive(end),charge.duration_s);
fprintf('Charge begins at SOC = %.3f and ends at SOC = %.3f.\n', ...
    mission.SOC(firstChargeIndex),mission.SOC(end));
end
