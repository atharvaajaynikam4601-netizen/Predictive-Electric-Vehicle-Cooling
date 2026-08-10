%RUN_PROJECT  Single entry point for the Predictive Electric Vehicle Cooling project.
%   Runs the full pipeline end-to-end from the repository root, in order:
%     1. Regenerate UDDS-based thermal/electrical input signals
%        (battery heat load, speed, SOC, ambient temperature).
%     2. Run the MATLAB-only controller comparison: reactive baseline vs.
%        predictive lookahead vs. full nonlinear MPC, over the synthetic
%        drive/fast-charge/soak stress scenario.
%     3. Simulate the Simscape Fluids reactive-baseline cooling plant using
%        the signals generated in step 1.
%
%   Usage: open this file in MATLAB and press Run, or from the command
%   window with the repository as the current folder: run_project
%
%   Requirements: MATLAB, Optimization Toolbox (fmincon, used by the MPC
%   controller in step 2), Simulink and Simscape Fluids (used by step 3 —
%   step 3 is skipped with a message if these are not installed).

clc; close all; clearvars;

repoRoot = fileparts(mfilename('fullpath'));
addpath(fullfile(repoRoot, 'scripts'));
addpath(fullfile(repoRoot, 'models'));
addpath(fullfile(repoRoot, 'data'));
cd(repoRoot);

fprintf('\n================================================================\n');
fprintf(' STEP 1/3: Generating UDDS-based thermal/electrical input signals\n');
fprintf('================================================================\n');
run('ev_eneergy_model_realistic_predictive_cooling.m');

fprintf('\n================================================================\n');
fprintf(' STEP 2/3: MATLAB controller comparison\n');
fprintf(' (reactive baseline vs. predictive lookahead vs. full nonlinear MPC)\n');
fprintf('================================================================\n');
run('Dynamic_Loads.m');

fprintf('\n================================================================\n');
fprintf(' STEP 3/3: Simulating the Simscape Fluids reactive-baseline plant\n');
fprintf('================================================================\n');
try
    sim('EV_Predictive_Cooling_Plant_Reactive.slx');
    fprintf('Simscape reactive-plant simulation complete. Inspect logged signals via the Simulation Data Inspector.\n');
catch simErr
    fprintf(2, 'Simscape simulation skipped: %s\n', simErr.message);
    fprintf('(Step 3 requires Simulink and Simscape Fluids. Steps 1-2 above still completed successfully.)\n');
end

fprintf('\n[DONE] Full pipeline complete.\n');
fprintf('Generated figures are open in MATLAB; EV_Thermal_Inputs.mat and results/ hold the saved outputs.\n');
