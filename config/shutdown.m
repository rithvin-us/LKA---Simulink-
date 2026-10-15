% ==============================================================================
% shutdown.m
% Cleans up paths, closes open models, and resets the MATLAB workspace.
% ==============================================================================

fprintf('\n[LKA System] Shutting down environment...\n');

projectRoot = fileparts(fileparts(mfilename('fullpath')));

% Close all open Simulink models without saving uncommitted changes
if bdIsLoaded('lka_main_system')
    close_system('lka_main_system', 0);
end

% Reset Simulink cache folder back to default
try
    Simulink.fileGenControl('reset');
catch
    % ignored if Simulink is not present
end

% Remove project directories from MATLAB path
pathsToRemove = {
    fullfile(projectRoot, 'config');
    fullfile(projectRoot, 'data');
    fullfile(projectRoot, 'data', 'bus_definitions');
    fullfile(projectRoot, 'data', 'parameters');
    fullfile(projectRoot, 'data', 'scenarios');
    fullfile(projectRoot, 'models');
    fullfile(projectRoot, 'src');
    fullfile(projectRoot, 'src', 'controllers');
    fullfile(projectRoot, 'src', 'perception');
    fullfile(projectRoot, 'src', 'utilities');
    fullfile(projectRoot, 'scripts');
    fullfile(projectRoot, 'tests');
    fullfile(projectRoot, 'tests', 'unit_tests');
    fullfile(projectRoot, 'tests', 'scenario_tests');
};

for i = 1:numel(pathsToRemove)
    if exist(pathsToRemove{i}, 'dir')
        rmpath(pathsToRemove{i});
    end
end

fprintf('[LKA System] Paths removed and environment successfully cleaned.\n\n');
