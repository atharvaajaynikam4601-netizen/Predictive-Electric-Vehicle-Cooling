function powertrain = export_udds_powertrain_result()
%EXPORT_UDDS_POWERTRAIN_RESULT Run the existing UDDS model and package outputs.
% The original powertrain script is intentionally not modified.

projectRoot = fileparts(fileparts(mfilename('fullpath')));
sourceDir = fullfile(projectRoot,'source_powertrain', ...
    'Mini Project 1 (Electric Vehicle Energy Consumption Calculator)');
oldDir = pwd;
cleanup = onCleanup(@() cd(oldDir)); %#ok<NASGU>
assert(isfile(fullfile(sourceDir,'uddsdc.csv')), ...
    'Expected UDDS file was not found: %s',fullfile(sourceDir,'uddsdc.csv'));
fprintf('Running existing powertrain from: %s\n',sourceDir);

% The original script begins with CLEAR. Execute it in the normal base
% workspace, where its relative CSV path and existing behavior are preserved.
sourceDirEscaped = strrep(sourceDir,'''','''''');
evalin('base',sprintf("cd('%s'); run('ev_energy_model_realistic.m');", ...
    sourceDirEscaped));

required = {'t','v','P_drive','P_regen','P_batt','I_batt','SOC','dt'};
for k = 1:numel(required)
    assert(evalin('base',sprintf("exist('%s','var')",required{k})) == 1, ...
        'Powertrain script did not produce variable: %s',required{k});
end

powertrain = struct();
powertrain.t_s = evalin('base','t(:)');
powertrain.speed_mps = evalin('base','v(:)');
powertrain.P_drive_W = evalin('base','P_drive(:)');
powertrain.P_regen_W = evalin('base','P_regen(:)');
powertrain.P_batt_W = evalin('base','P_batt(:)');
powertrain.I_batt_A = evalin('base','I_batt(:)');
powertrain.SOC = evalin('base','SOC(:)');
powertrain.dt_s = evalin('base','dt');

assert(numel(powertrain.t_s) == numel(powertrain.P_batt_W));
assert(all(diff(powertrain.t_s) > 0));
assert(all(powertrain.SOC >= 0 & powertrain.SOC <= 1));

outDir = fullfile(projectRoot,'data');
if ~isfolder(outDir), mkdir(outDir); end
save(fullfile(outDir,'powertrain_udds_result.mat'),'powertrain');

fprintf('Exported %d UDDS samples from the existing powertrain model.\n', ...
    numel(powertrain.t_s));
fprintf('Final UDDS state: t = %.1f s, SOC = %.3f, I = %.2f A.\n', ...
    powertrain.t_s(end),powertrain.SOC(end),powertrain.I_batt_A(end));
end
