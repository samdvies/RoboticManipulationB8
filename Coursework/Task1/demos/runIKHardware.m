%% Run IK on Physical Robot
% Move the OpenManipulator-X to XYZ target positions using inverse kinematics
%
% SAFETY FEATURES:
%   - Slow velocity (50) for safe movement
%   - Joint limits enforced (±90°)
%   - Emergency stop on cleanup (Ctrl+C or close window)
%   - Visual confirmation before each move
%
% Author: OpenManipulator-X IK Implementation
% Date: February 2026

clear all; close all; clc;

fprintf('=============================================\n');
fprintf('   OpenManipulator-X IK Hardware Control\n');
fprintf('=============================================\n\n');

%% Configuration - EDIT THESE
COM_PORT = 'COM3';      % <-- Change to your COM port
VELOCITY = 50;          % Safe slow speed (increase for faster, max ~200)

%% Initialize Robot
fprintf('Initializing robot on %s...\n', COM_PORT);
fprintf('Press Ctrl+C at any time for EMERGENCY STOP\n\n');

try
    [port_num, lib_name, cleanup] = robotSafeInit(COM_PORT, VELOCITY);
    fprintf('Robot initialized successfully!\n\n');
catch ME
    fprintf('ERROR: Failed to initialize robot.\n');
    fprintf('Check that:\n');
    fprintf('  1. COM port is correct (currently: %s)\n', COM_PORT);
    fprintf('  2. Robot is powered on\n');
    fprintf('  3. USB cable is connected\n');
    fprintf('  4. No other program is using the port\n\n');
    fprintf('Error message: %s\n', ME.message);
    return;
end

%% Setup Visualization
fig = figure('Name', 'IK Hardware Control', 'Position', [100 100 900 700], 'Color', 'w');

%% Move to Home Position First
fprintf('Moving to HOME position first...\n');
q_home = [0, 0, 0, 0];
moveToJointAngles(port_num, lib_name, q_home, VELOCITY);
pause(2);

% Visualize home
clf;
ax = axes('Parent', fig);
plotRobotArm(q_home, ax, true, 30);
title(ax, 'Home Position - Ready for Commands', 'FontSize', 14);
drawnow;

%% Main Control Loop
fprintf('\n--- Interactive IK Control ---\n');
fprintf('Enter target positions as: x y z (in mm)\n');
fprintf('Example: 200 0 200\n');
fprintf('Commands:\n');
fprintf('  home     - Return to home position\n');
fprintf('  demo     - Run demo sequence\n');
fprintf('  quit/q   - Exit program\n');
fprintf('--------------------------------\n\n');

while true
    % Get user input
    input_str = input('Target [x y z]: ', 's');
    
    % Check for commands
    if isempty(input_str)
        continue;
    elseif strcmpi(input_str, 'quit') || strcmpi(input_str, 'q')
        fprintf('\nReturning to home and exiting...\n');
        moveToJointAngles(port_num, lib_name, [0,0,0,0], VELOCITY);
        break;
    elseif strcmpi(input_str, 'home')
        fprintf('Moving to home position...\n');
        moveToJointAngles(port_num, lib_name, [0,0,0,0], VELOCITY);
        clf; plotRobotArm([0,0,0,0], ax, true, 30);
        title(ax, 'Home Position', 'FontSize', 14);
        drawnow;
        continue;
    elseif strcmpi(input_str, 'demo')
        runHardwareDemo(port_num, lib_name, VELOCITY, fig);
        continue;
    end
    
    % Parse XYZ coordinates
    coords = sscanf(input_str, '%f %f %f');
    if length(coords) ~= 3
        fprintf('Invalid input. Enter 3 numbers: x y z\n');
        fprintf('Or commands: home, demo, quit\n\n');
        continue;
    end
    target = coords';
    
    % Compute IK
    fprintf('\nComputing IK for [%.0f, %.0f, %.0f] mm...\n', target);
    [q, success, info] = inverseKinematicsAuto(target);
    
    if ~success
        fprintf('Cannot reach target: %s\n\n', info.message);
        continue;
    end
    
    fprintf('Solution found!\n');
    fprintf('  Joint angles: [%.1f, %.1f, %.1f, %.1f] deg\n', rad2deg(q));
    fprintf('  Pitch: %.1f deg\n', rad2deg(info.pitch_used));
    
    % Visualize before moving
    clf;
    ax = axes('Parent', fig);
    plotRobotArm(q, ax, true, 30);
    hold(ax, 'on');
    plot3(ax, target(1), target(2), target(3), 'gp', 'MarkerSize', 20, ...
        'MarkerFaceColor', 'g', 'LineWidth', 2);
    title(ax, sprintf('Target: [%.0f, %.0f, %.0f] mm - Confirm move?', target), 'FontSize', 14);
    drawnow;
    
    % Confirm before moving
    confirm = input('Move robot? (y/n): ', 's');
    if ~strcmpi(confirm, 'y')
        fprintf('Move cancelled.\n\n');
        continue;
    end
    
    % Execute move
    fprintf('Moving robot...\n');
    moveToPosition(port_num, lib_name, target, 'auto', VELOCITY);
    
    % Update visualization
    title(ax, sprintf('At Position: [%.0f, %.0f, %.0f] mm', target), 'FontSize', 14);
    fprintf('Move complete!\n\n');
end

%% Cleanup
fprintf('Shutting down...\n');
clear cleanup;  % Triggers emergency stop / torque disable
close(fig);
fprintf('Done.\n');

%% Helper Functions
function moveToJointAngles(port_num, lib_name, q_rad, velocity)
    % Move robot to specific joint angles
    PROTOCOL_VERSION = 2.0;
    DXL_IDS = [11, 12, 13, 14];
    ADDR_GOAL_POSITION = 116;
    ADDR_PRESENT_POSITION = 132;
    
    % Convert to encoder values
    encoder_targets = zeros(1, 4);
    for i = 1:4
        encoder_targets(i) = angleConversion('rad2enc', q_rad(i));
    end
    
    % Send commands
    for i = 1:4
        write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_IDS(i), ...
            ADDR_GOAL_POSITION, encoder_targets(i));
    end
    
    % Wait for movement to complete
    timeout = 10;
    tic;
    while toc < timeout
        all_arrived = true;
        for i = 1:4
            pos = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_IDS(i), ADDR_PRESENT_POSITION);
            if abs(pos - encoder_targets(i)) > 20
                all_arrived = false;
                break;
            end
        end
        if all_arrived
            break;
        end
        pause(0.05);
    end
end

function runHardwareDemo(port_num, lib_name, velocity, fig)
    % Run a predefined demo sequence
    fprintf('\n--- Running Demo Sequence ---\n');
    
    targets = [
        200,   0, 200;
        250,   0, 150;
        180,  60, 220;
        180, -60, 220;
        200,   0, 250;
        200,   0, 200;
    ];
    
    names = {'Center', 'Forward', 'Right', 'Left', 'Up', 'Home'};
    
    for i = 1:size(targets, 1)
        target = targets(i, :);
        fprintf('\nPosition %d/%d: %s [%.0f, %.0f, %.0f]\n', ...
            i, size(targets, 1), names{i}, target);
        
        [q, success, info] = inverseKinematicsAuto(target);
        if ~success
            fprintf('Skipping - unreachable\n');
            continue;
        end
        
        % Visualize
        clf(fig);
        ax = axes('Parent', fig);
        plotRobotArm(q, ax, true, 30);
        hold(ax, 'on');
        plot3(ax, target(1), target(2), target(3), 'gp', 'MarkerSize', 20, ...
            'MarkerFaceColor', 'g', 'LineWidth', 2);
        title(ax, sprintf('Demo: %s', names{i}), 'FontSize', 14);
        drawnow;
        
        % Move
        moveToPosition(port_num, lib_name, target, 'auto', velocity);
        pause(1);
    end
    
    fprintf('\nDemo complete!\n\n');
end
