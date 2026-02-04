function calibrationHelper()
% CALIBRATIONHELPER Interactive calibration for Task 2 positions
%
% Moves the robot to configured positions so you can verify alignment.
% Use this to check and adjust task2Config.m values before running tasks.
%
% Usage:
%   calibrationHelper
%
% Commands:
%   cube N      - Move above cube N (1, 2, or 3)
%   holder N    - Move above holder N (1, 2, or 3)
%   gate N      - Move to gate N entry point
%   grid X Y Z  - Move to grid position (X, Y, Z)
%   home        - Return to home position
%   test        - Visit all positions in sequence
%   config      - Display current configuration
%   quit        - Exit calibration
%
% Author: OpenManipulator-X Task 2
% Date: February 2026

clear; clc;

fprintf('=============================================\n');
fprintf('   Task 2 Calibration Helper\n');
fprintf('=============================================\n\n');

%% Configuration
COM_PORT = 'COM3';  % <-- CHANGE TO YOUR COM PORT!
VELOCITY = 40;      % Slow speed for calibration

%% Load Configuration
config = task2Config();
GRID = config.grid_size;

fprintf('Grid size: %d mm\n', GRID);
fprintf('Loaded %d cubes, %d holders, %d gates\n', ...
    size(config.cube.grid, 1), ...
    size(config.holder.grid, 1), ...
    size(config.gates.grid, 1));

%% Initialize Robot
fprintf('\nInitializing robot on %s...\n', COM_PORT);
fprintf('Press Ctrl+C for EMERGENCY STOP\n\n');

try
    [port_num, lib_name, cleanup] = robotSafeInit(COM_PORT, VELOCITY);
    fprintf('Robot initialized!\n\n');
catch ME
    fprintf('ERROR: Could not initialize robot.\n');
    fprintf('Check COM port and connections.\n');
    fprintf('Error: %s\n', ME.message);
    return;
end

%% Setup Visualization
fig = figure('Name', 'Calibration Helper', 'Position', [100 100 800 600], 'Color', 'w');

%% Move to Home
fprintf('Moving to HOME position...\n');
moveToHome(port_num, lib_name, VELOCITY);
pause(1);

%% Display Help
printHelp();

%% Main Command Loop
while true
    cmd = input('\nCalibration> ', 's');
    
    if isempty(cmd)
        continue;
    end
    
    tokens = strsplit(lower(cmd));
    action = tokens{1};
    
    try
        switch action
            case 'quit'
                fprintf('Returning to home and exiting...\n');
                moveToHome(port_num, lib_name, VELOCITY);
                break;
                
            case 'home'
                fprintf('Moving to HOME...\n');
                moveToHome(port_num, lib_name, VELOCITY);
                
            case 'cube'
                if length(tokens) < 2
                    fprintf('Usage: cube N (where N = 1, 2, or 3)\n');
                    continue;
                end
                n = str2double(tokens{2});
                if n < 1 || n > size(config.cube.grid, 1)
                    fprintf('Invalid cube number. Use 1-%d\n', size(config.cube.grid, 1));
                    continue;
                end
                pos_grid = config.cube.grid(n, :);
                pos_mm = pos_grid * GRID;
                % Move above cube
                target = pos_mm + [0, 0, config.motion.approach_height];
                fprintf('Moving above Cube %d at grid [%d, %d, %d] = [%.0f, %.0f, %.0f] mm\n', ...
                    n, pos_grid, target);
                moveAndVisualize(port_num, lib_name, target, fig, config);
                
            case 'holder'
                if length(tokens) < 2
                    fprintf('Usage: holder N (where N = 1, 2, or 3)\n');
                    continue;
                end
                n = str2double(tokens{2});
                if n < 1 || n > size(config.holder.grid, 1)
                    fprintf('Invalid holder number. Use 1-%d\n', size(config.holder.grid, 1));
                    continue;
                end
                pos_grid = config.holder.grid(n, :);
                pos_mm = pos_grid * GRID;
                target = pos_mm + [0, 0, config.motion.approach_height + config.cube.size];
                fprintf('Moving above Holder %d at grid [%d, %d, %d] = [%.0f, %.0f, %.0f] mm\n', ...
                    n, pos_grid, target);
                moveAndVisualize(port_num, lib_name, target, fig, config);
                
            case 'gate'
                if length(tokens) < 2
                    fprintf('Usage: gate N (where N = 1 or 2)\n');
                    continue;
                end
                n = str2double(tokens{2});
                if n < 1 || n > size(config.gates.grid, 1)
                    fprintf('Invalid gate number. Use 1-%d\n', size(config.gates.grid, 1));
                    continue;
                end
                pos_grid = config.gates.grid(n, :);
                pos_mm = pos_grid * GRID;
                % Approach from -X side
                target = pos_mm + [-50, 0, 0];
                fprintf('Moving to Gate %d approach at grid [%d, %d, %d]\n', n, pos_grid);
                fprintf('Target: [%.0f, %.0f, %.0f] mm\n', target);
                moveAndVisualize(port_num, lib_name, target, fig, config);
                
            case 'grid'
                if length(tokens) < 4
                    fprintf('Usage: grid X Y Z (grid units)\n');
                    continue;
                end
                gx = str2double(tokens{2});
                gy = str2double(tokens{3});
                gz = str2double(tokens{4});
                target = [gx, gy, gz] * GRID;
                fprintf('Moving to grid [%d, %d, %d] = [%.0f, %.0f, %.0f] mm\n', ...
                    gx, gy, gz, target);
                moveAndVisualize(port_num, lib_name, target, fig, config);
                
            case 'test'
                runTestSequence(port_num, lib_name, config, fig, VELOCITY);
                
            case 'config'
                displayConfig(config);
                
            case 'help'
                printHelp();
                
            case 'gripper'
                if length(tokens) < 2
                    fprintf('Usage: gripper open|close|cube\n');
                    continue;
                end
                gripperAction(port_num, lib_name, tokens{2}, config);
                
            otherwise
                fprintf('Unknown command: %s\n', action);
                fprintf('Type "help" for available commands.\n');
        end
    catch ME
        fprintf('ERROR: %s\n', ME.message);
    end
end

%% Cleanup
close(fig);
clear cleanup;
fprintf('Calibration complete.\n');

end

%% Helper Functions

function printHelp()
    fprintf('\n--- Calibration Commands ---\n');
    fprintf('  cube N      - Move above cube N (1-3)\n');
    fprintf('  holder N    - Move above holder N (1-3)\n');
    fprintf('  gate N      - Move to gate N approach\n');
    fprintf('  grid X Y Z  - Move to grid position\n');
    fprintf('  gripper X   - Control gripper (open/close/cube)\n');
    fprintf('  home        - Return to home position\n');
    fprintf('  test        - Visit all positions\n');
    fprintf('  config      - Show current configuration\n');
    fprintf('  help        - Show this help\n');
    fprintf('  quit        - Exit calibration\n');
    fprintf('-----------------------------\n');
end

function displayConfig(config)
    GRID = config.grid_size;
    fprintf('\n=== Current Configuration ===\n');
    fprintf('\nCubes (grid -> mm):\n');
    for i = 1:size(config.cube.grid, 1)
        g = config.cube.grid(i,:);
        m = g * GRID;
        fprintf('  Cube %d: [%2d, %2d, %2d] -> [%3.0f, %3.0f, %3.0f] mm, red face: %d°\n', ...
            i, g, m, config.cube.red_face(i));
    end
    fprintf('\nHolders (grid -> mm):\n');
    for i = 1:size(config.holder.grid, 1)
        g = config.holder.grid(i,:);
        m = g * GRID;
        fprintf('  Holder %d: [%2d, %2d, %2d] -> [%3.0f, %3.0f, %3.0f] mm\n', i, g, m);
    end
    fprintf('\nGates (grid -> mm):\n');
    for i = 1:size(config.gates.grid, 1)
        g = config.gates.grid(i,:);
        m = g * GRID;
        fprintf('  Gate %d: [%2d, %2d, %2d] -> [%3.0f, %3.0f, %3.0f] mm\n', i, g, m);
    end
    fprintf('\nHeight limit: %d grid = %.0f mm\n', ...
        config.gates.height_limit_grid, config.gates.height_limit);
    fprintf('Stack holder: %d\n', config.holder.stack_index);
    fprintf('Red face target: %d°\n', config.holder.red_face_target);
    fprintf('==============================\n');
end

function moveToHome(port_num, lib_name, velocity)
    q_home = [0, 0, 0, 0];
    PROTOCOL_VERSION = 2.0;
    DXL_IDS = [11, 12, 13, 14];
    ADDR_GOAL_POSITION = 116;
    
    % Convert to encoder
    home_encoder = 2048;  % 0 degrees
    
    for i = 1:4
        write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_IDS(i), ADDR_GOAL_POSITION, home_encoder);
    end
    pause(2);
end

function moveAndVisualize(port_num, lib_name, target, fig, config)
    % Compute IK
    [q, success, info] = inverseKinematicsAuto(target);
    
    if ~success
        fprintf('WARNING: Cannot reach [%.0f, %.0f, %.0f]: %s\n', target, info.message);
        return;
    end
    
    fprintf('Joint angles: [%.1f, %.1f, %.1f, %.1f] deg\n', rad2deg(q));
    
    % Visualize
    figure(fig);
    clf;
    ax = axes('Parent', fig);
    plotRobotArm(q, ax, true, 25);
    hold(ax, 'on');
    
    % Plot all cubes
    for i = 1:size(config.cube.positions, 1)
        pos = config.cube.positions(i, :);
        plotCube(ax, pos, config.cube.size, [0 0.7 1]);  % Cyan cubes
        text(ax, pos(1), pos(2), pos(3)+40, sprintf('C%d', i), 'FontSize', 10);
    end
    
    % Plot all holders
    for i = 1:size(config.holder.positions, 1)
        pos = config.holder.positions(i, :);
        plot3(ax, pos(1), pos(2), pos(3), 'g^', 'MarkerSize', 15, 'LineWidth', 2);
        text(ax, pos(1), pos(2), pos(3)+30, sprintf('H%d', i), 'FontSize', 10);
    end
    
    % Plot target
    plot3(ax, target(1), target(2), target(3), 'rp', 'MarkerSize', 20, ...
        'MarkerFaceColor', 'r', 'LineWidth', 2);
    
    title(ax, sprintf('Target: [%.0f, %.0f, %.0f] mm', target), 'FontSize', 12);
    hold(ax, 'off');
    drawnow;
    
    % Move robot
    moveToPosition(port_num, lib_name, target, 'auto', 40);
end

function plotCube(ax, center, size, color)
    % Draw a simple cube representation
    s = size/2;
    x = center(1);
    y = center(2);
    z = center(3) - s;  % Center is at top surface
    
    % Bottom face
    plot3(ax, [x-s x+s x+s x-s x-s], [y-s y-s y+s y+s y-s], [z z z z z], '-', 'Color', color, 'LineWidth', 1);
    % Top face
    plot3(ax, [x-s x+s x+s x-s x-s], [y-s y-s y+s y+s y-s], [z+size z+size z+size z+size z+size], '-', 'Color', color, 'LineWidth', 1);
    % Vertical edges
    for dx = [-s, s]
        for dy = [-s, s]
            plot3(ax, [x+dx x+dx], [y+dy y+dy], [z z+size], '-', 'Color', color, 'LineWidth', 1);
        end
    end
end

function runTestSequence(port_num, lib_name, config, fig, velocity)
    fprintf('\n=== Running Test Sequence ===\n');
    
    % Visit each cube
    for i = 1:size(config.cube.positions, 1)
        fprintf('\nMoving to Cube %d...\n', i);
        target = config.cube.positions(i, :) + [0, 0, config.motion.approach_height];
        moveAndVisualize(port_num, lib_name, target, fig, config);
        pause(1);
    end
    
    % Visit each holder
    for i = 1:size(config.holder.positions, 1)
        fprintf('\nMoving to Holder %d...\n', i);
        target = config.holder.positions(i, :) + [0, 0, config.motion.approach_height + config.cube.size];
        moveAndVisualize(port_num, lib_name, target, fig, config);
        pause(1);
    end
    
    % Return home
    fprintf('\nReturning to home...\n');
    moveToHome(port_num, lib_name, velocity);
    
    fprintf('\n=== Test Sequence Complete ===\n');
end

function gripperAction(port_num, lib_name, action, config)
    PROTOCOL_VERSION = 2.0;
    ADDR_GOAL_POSITION = 116;
    GRIPPER_ID = config.gripper.id;
    
    switch lower(action)
        case 'open'
            target = config.gripper.open;
        case 'close'
            target = config.gripper.closed;
        case 'cube'
            target = config.gripper.cube_grip;
        otherwise
            fprintf('Unknown gripper action: %s\n', action);
            return;
    end
    
    fprintf('Gripper %s (encoder=%d)\n', upper(action), target);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, GRIPPER_ID, ADDR_GOAL_POSITION, target);
    pause(0.5);
end
