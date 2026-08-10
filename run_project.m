%RUN_PROJECT  Single entry point for the Predictive Electric Vehicle Cooling project.
%   Runs the full pipeline end-to-end from the repository root, in order:
%     1. Run the MATLAB-only controller comparison: reactive baseline vs.
%        predictive lookahead vs. full nonlinear MPC, over the synthetic
%        drive/fast-charge/soak stress scenario.
%     2. Regenerate UDDS-based thermal/electrical input signals
%        (battery heat load, speed, SOC, ambient temperature).
%     3. Simulate the Simscape Fluids reactive-baseline cooling plant using
%        the signals generated in step 2.
%     4. Build (if needed) and simulate the predictive-lookahead and MPC
%        Simscape Fluids plant variants, so all three controllers run on
%        the identical physical plant, not only the MATLAB approximation.
%
%   Step 1 runs before step 2, not after, because Dynamic_Loads.m (step 1)
%   starts with `clear` and ends by writing its OWN Q_heat_ts/T_amb_ts
%   (from its synthetic 2000 s scenario) into the base workspace. The
%   Simscape models in steps 3-4 read Q_heat_ts/T_amb_ts from the base
%   workspace via From Workspace blocks -- if step 1 ran after step 2, it
%   would silently overwrite the UDDS-based signals with the wrong
%   dataset before the plant simulations ever ran. Keep this ordering.
%
%   Usage: open this file in MATLAB and press Run, or from the command
%   window with the repository as the current folder: run_project
%
%   Requirements: MATLAB, Optimization Toolbox (fmincon, used by the MPC
%   controller in steps 1 and 4), Simulink and Simscape Fluids (used by
%   steps 3 and 4). Steps 3-4 are skipped with a message if these are not
%   installed.

clc; close all; clearvars;

repoRoot = fileparts(mfilename('fullpath'));
addpath(fullfile(repoRoot, 'scripts'));
addpath(fullfile(repoRoot, 'models'));
addpath(fullfile(repoRoot, 'data'));
cd(repoRoot);

fprintf('\n================================================================\n');
fprintf(' STEP 1/4: MATLAB controller comparison\n');
fprintf(' (reactive baseline vs. predictive lookahead vs. full nonlinear MPC)\n');
fprintf('================================================================\n');
run('Dynamic_Loads.m');

fprintf('\n================================================================\n');
fprintf(' STEP 2/4: Generating UDDS-based thermal/electrical input signals\n');
fprintf(' (this MUST run after step 1 -- see note at the top of this file)\n');
fprintf('================================================================\n');
run('ev_eneergy_model_realistic_predictive_cooling.m');

fprintf('\n================================================================\n');
fprintf(' STEP 3/4: Simulating the Simscape Fluids reactive-baseline plant\n');
fprintf('================================================================\n');
try
    sim('EV_Predictive_Cooling_Plant_Reactive.slx');
    fprintf('Simscape reactive-plant simulation complete. Inspect logged signals via the Simulation Data Inspector.\n');
catch simErr
    fprintf(2, 'Simscape simulation skipped: %s\n', simErr.message);
    fprintf('(Step 3 requires Simulink and Simscape Fluids. Steps 1-2 above still completed successfully.)\n');
end

fprintf('\n================================================================\n');
fprintf(' STEP 4/4: Building and simulating the predictive/MPC Simscape variants\n');
fprintf('================================================================\n');
try
    run('build_predictive_mpc_models.m');
    sim('EV_Predictive_Cooling_Plant_PredictiveLookahead.slx');
    fprintf('Predictive-lookahead Simscape plant simulation complete.\n');
    sim('EV_Predictive_Cooling_Plant_MPC.slx');
    fprintf('MPC Simscape plant simulation complete.\n');
catch stepErr
    fprintf(2, 'Step 4 skipped: %s\n', stepErr.message);
    fprintf('(Requires Simulink, Simscape Fluids, and Optimization Toolbox. Steps 1-3 above still completed.)\n');
end

fprintf('\n[DONE] Full pipeline complete.\n');
fprintf('Generated figures are open in MATLAB; EV_Thermal_Inputs.mat and results/ hold the saved outputs.\n');
