function task2_main()
% TASK2_MAIN Main script to run all Task 2 sub-tasks
%
% Runs the complete Task 2 sequence:
%   2.a - Move cubes to holders (9 points)
%   2.b - Stack cubes with red faces out (16 points)
%   2.c - Navigate gates (15 points)
%
% Total possible: 40 points
%
% Usage:
%   task2_main
%
% Before running:
%   1. Run setupPath to add all folders to MATLAB path
%   2. Run calibrationHelper to verify positions
%   3. Edit task2Config.m if positions need adjustment
%
% Author: OpenManipulator-X Task 2
% Date: February 2026

clear; clc;

fprintf('================================================\n');
fprintf('   OPENMANIPULATOR-X TASK 2\n');
fprintf('   Pick, Place, Stack & Navigate\n');
fprintf('================================================\n\n');

%% Configuration
COM_PORT = 'COM3';  % <-- CHANGE TO YOUR COM PORT!
VELOCITY = 50;

%% Load Configuration
fprintf('Loading configuration...\n');
config = task2Config();
config.motion.velocity = VELOCITY;
config.motion.velocity_slow = 30;

fprintf('Grid size: %d mm\n', config.grid_size);
fprintf('Cubes: %d\n', size(config.cube.positions, 1));
fprintf('Holders: %d\n', size(config.holder.positions, 1));
fprintf('Gates: %d\n', size(config.gates.positions, 1));

%% Task Selection
fprintf('\n--- Task Selection ---\n');
fprintf('1. Run ALL tasks (2.a + 2.b + 2.c)\n');
fprintf('2. Task 2.a only (Move cubes to holders)\n');
fprintf('3. Task 2.b only (Stack cubes)\n');
fprintf('4. Task 2.c only (Navigate gates)\n');
fprintf('5. Task 2.a + 2.b (Cube manipulation)\n');
fprintf('0. Cancel\n');
fprintf('----------------------\n');

selection = input('Select task(s) to run [1-5, 0=cancel]: ');

if selection == 0
    fprintf('Cancelled.\n');
    return;
end

%% Initialize Robot
fprintf('\nInitializing robot on %s...\n', COM_PORT);
fprintf('>>> Press Ctrl+C for EMERGENCY STOP <<<\n\n');

try
    [port_num, lib_name, cleanup] = robotSafeInit(COM_PORT, VELOCITY);
    fprintf('Robot initialized!\n');
catch ME
    fprintf('ERROR: Could not initialize robot.\n');
    fprintf('%s\n', ME.message);
    return;
end

%% Move to Home Position
fprintf('\nMoving to home position...\n');
moveToHome(port_num, lib_name);
pause(2);

%% Run Selected Tasks
try
    switch selection
        case 1
            % All tasks
            task2a_moveCubes(port_num, lib_name, config);
            pause(1);
            task2b_stackCubes(port_num, lib_name, config);
            pause(1);
            task2c_navigateGates(port_num, lib_name, config);
            
        case 2
            % Task 2.a only
            task2a_moveCubes(port_num, lib_name, config);
            
        case 3
            % Task 2.b only
            task2b_stackCubes(port_num, lib_name, config);
            
        case 4
            % Task 2.c only
            task2c_navigateGates(port_num, lib_name, config);
            
        case 5
            % Tasks 2.a + 2.b
            task2a_moveCubes(port_num, lib_name, config);
            pause(1);
            task2b_stackCubes(port_num, lib_name, config);
    end
    
catch ME
    fprintf('\n!!! TASK ERROR !!!\n');
    fprintf('%s\n', ME.message);
    fprintf('Returning to home position...\n');
end

%% Cleanup
fprintf('\nReturning to home position...\n');
moveToHome(port_num, lib_name);
pause(2);

% Close gripper partially for safety
gripperControl(port_num, lib_name, 'close', config);

% Cleanup happens automatically when function exits
clear cleanup;

fprintf('\n================================================\n');
fprintf('   TASK 2 COMPLETE\n');
fprintf('================================================\n');

end

%% Helper: Move to Home
function moveToHome(port_num, lib_name)
    PROTOCOL_VERSION = 2.0;
    DXL_IDS = [11, 12, 13, 14];
    ADDR_GOAL_POSITION = 116;
    HOME_ENCODER = 2048;
    
    for i = 1:4
        write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_IDS(i), ADDR_GOAL_POSITION, HOME_ENCODER);
    end
    pause(2);
end
