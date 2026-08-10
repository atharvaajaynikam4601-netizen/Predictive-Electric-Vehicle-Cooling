%% build_predictive_mpc_models.m
%
% Builds two new Simscape Fluids plant variants by cloning the validated
% reactive baseline (models/EV_Predictive_Cooling_Plant_Reactive.slx) and
% swapping ONLY its embedded controller block, so all three controllers
% -- reactive, predictive lookahead, and full nonlinear MPC -- run on the
% identical physical plant (cold plate, radiator, pump, reservoir),
% instead of the predictive/MPC controllers only being validated in the
% MATLAB lumped-parameter model (scripts/Dynamic_Loads.m).
%
% Produces:
%   models/EV_Predictive_Cooling_Plant_PredictiveLookahead.slx
%   models/EV_Predictive_Cooling_Plant_MPC.slx
%
% Prerequisite: Q_heat_ts (and T_amb_ts, for the MPC variant) must be in
% the base workspace before you SIMULATE either new model -- both
% controllers pull the full timeseries from the base workspace at
% simulation setup. Run scripts/ev_eneergy_model_realistic_predictive_cooling.m
% (or run_project) first.
%
% This script only rebuilds the model files; it does not need
% Q_heat_ts/T_amb_ts to be loaded to run itself.
%
% Run once from anywhere with this repository intact:
%   run("scripts/build_predictive_mpc_models.m")
% Re-run it if you change the controller class files under
% models/+controllers/.

thisDir = fileparts(mfilename('fullpath'));
repoRoot = fileparts(thisDir);
modelsDir = fullfile(repoRoot, 'models');
addpath(modelsDir);

reactiveModel = fullfile(modelsDir, 'EV_Predictive_Cooling_Plant_Reactive.slx');
if ~isfile(reactiveModel)
    error('build_predictive_mpc_models:missingTemplate', ...
        'Cannot find %s. This script must clone the validated reactive plant.', reactiveModel);
end

% {new model name, controller class, human-readable label}
variants = {
    'EV_Predictive_Cooling_Plant_PredictiveLookahead', 'controllers.PredictiveLookaheadController', 'predictive lookahead'
    'EV_Predictive_Cooling_Plant_MPC',                 'controllers.MPCCoolingController',           'full nonlinear MPC'
};

% Position of the reactive controller's "MATLAB Function" block, reused
% for its replacement so the new block lands in the same spot.
controllerPosition = [1200, 487, 1270, 533];

for i = 1:size(variants, 1)
    newName = variants{i, 1};
    controllerClass = variants{i, 2};
    label = variants{i, 3};
    newPath = fullfile(modelsDir, [newName '.slx']);

    fprintf('\nBuilding %s (%s controller) ...\n', newName, label);

    if bdIsLoaded(newName)
        close_system(newName, 0);
    end
    copyfile(reactiveModel, newPath, 'f');
    load_system(newPath);

    controllerBlock = [newName '/MATLAB Function'];
    newBlockName = 'Cooling Controller';
    newBlockPath = [newName '/' newBlockName];

    % The reactive controller is wired as:
    %   Unit Delay/out:1  -> MATLAB Function/in:1
    %   MATLAB Function/out:1 -> Simulink-PS Converter1/in:1
    %   MATLAB Function/out:1 -> Scope Block1/in:1
    % Delete it and add a MATLAB System block with the same 1-in/1-out
    % interface, then reconnect it exactly the same way.
    delete_block(controllerBlock);

    add_block('simulink/User-Defined Functions/MATLAB System', newBlockPath, ...
        'Position', controllerPosition);
    set_param(newBlockPath, 'System', controllerClass);

    add_line(newName, 'Unit Delay/1', [newBlockName '/1'], 'autorouting', 'on');
    add_line(newName, [newBlockName '/1'], 'Simulink-PS Converter1/1', 'autorouting', 'on');
    add_line(newName, [newBlockName '/1'], 'Scope Block1/1', 'autorouting', 'on');

    save_system(newName, newPath);
    close_system(newName, 0);

    fprintf('  Saved %s\n', newPath);
end

fprintf('\n[DONE] Built predictive-lookahead and MPC Simscape Fluids plant variants.\n');
fprintf('Load Q_heat_ts/T_amb_ts into the base workspace, then simulate either model, e.g.:\n');
fprintf('  run("scripts/ev_eneergy_model_realistic_predictive_cooling.m")\n');
fprintf('  sim("EV_Predictive_Cooling_Plant_PredictiveLookahead.slx")\n');
