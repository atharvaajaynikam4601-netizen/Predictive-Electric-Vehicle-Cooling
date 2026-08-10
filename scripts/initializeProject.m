%% Initialize Predictive Electric Vehicle Cooling project
% Run this file from the project root. It only configures the MATLAB path
% and creates any missing output folders.

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(projectRoot,'scripts')));

outputFolders = {'data','models','results','docs'};
for k = 1:numel(outputFolders)
    folder = fullfile(projectRoot,outputFolders{k});
    if ~isfolder(folder)
        mkdir(folder);
    end
end

format compact
fprintf('Project initialized: %s\n', projectRoot);
