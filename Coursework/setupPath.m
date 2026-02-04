function setupPath()
% SETUPPATH Add all coursework folders to MATLAB path
%
% Run this once at the start of each MATLAB session:
%   >> setupPath
%
% This adds all Task1 and Task2 subfolders so functions can find each other.

    % Get coursework root directory
    rootDir = fileparts(mfilename('fullpath'));
    
    % Add Task1 folders
    addpath(fullfile(rootDir, 'Task1', 'core'));
    addpath(fullfile(rootDir, 'Task1', 'hardware'));
    addpath(fullfile(rootDir, 'Task1', 'visualization'));
    addpath(fullfile(rootDir, 'Task1', 'demos'));
    addpath(fullfile(rootDir, 'Task1', 'tests'));
    
    % Add Task2 folders
    addpath(fullfile(rootDir, 'Task2', 'config'));
    addpath(fullfile(rootDir, 'Task2', 'core'));
    addpath(fullfile(rootDir, 'Task2', 'tasks'));
    addpath(fullfile(rootDir, 'Task2', 'calibration'));
    
    fprintf('=============================================\n');
    fprintf('  Robotic Manipulation Coursework - Ready\n');
    fprintf('=============================================\n');
    fprintf('Task1 folders: core, hardware, visualization, demos, tests\n');
    fprintf('Task2 folders: config, core, tasks, calibration\n');
    fprintf('\nType "help functionName" for usage info.\n');
end
