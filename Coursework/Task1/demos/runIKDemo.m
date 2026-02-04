function runIKDemo(port_num, lib_name, mode)
% RUNIKDEMO Interactive demonstration of inverse kinematics
%
% Moves the robot through a sequence of target positions or allows
% interactive input of target coordinates.
%
% Usage:
%   % Simulation only (no hardware)
%   runIKDemo()
%
%   % With hardware
%   [port_num, lib_name, cleanup] = robotSafeInit('COM3');
%   runIKDemo(port_num, lib_name)
%   runIKDemo(port_num, lib_name, 'interactive')
%
% Modes:
%   'demo'        - Predefined sequence of positions (default)
%   'interactive' - User enters XYZ coordinates
%   'square'      - Draw a square pattern
%
% Author: OpenManipulator-X IK Implementation
% Date: February 2026

    %% Determine Mode
    hardware_connected = (nargin >= 2) && ~isempty(port_num) && ~isempty(lib_name);
    
    if nargin < 3 || isempty(mode)
        mode = 'demo';
    end
    
    fprintf('\n===========================================\n');
    fprintf('   OpenManipulator-X IK Demonstration\n');
    fprintf('===========================================\n');
    fprintf('Hardware connected: %s\n', string(hardware_connected));
    fprintf('Mode: %s\n', mode);
    fprintf('-------------------------------------------\n\n');
    
    %% Setup Visualization
    fig = figure('Name', 'IK Demo', 'Position', [100 100 1000 700], 'Color', 'w');
    ax = gca;
    
    %% Run Selected Mode
    switch lower(mode)
        case 'demo'
            runDemoSequence(ax, port_num, lib_name, hardware_connected);
            
        case 'interactive'
            runInteractive(ax, port_num, lib_name, hardware_connected);
            
        case 'square'
            runSquarePattern(ax, port_num, lib_name, hardware_connected);
            
        otherwise
            error('Unknown mode: %s. Use ''demo'', ''interactive'', or ''square''.', mode);
    end
    
    fprintf('\nDemo complete!\n');
end

%% Demo Sequence - Predefined Positions
function runDemoSequence(ax, port_num, lib_name, hardware)
    % Sequence of target positions (designed for auto-pitch within ±90° limits)
    targets = [
        200,   0, 200;    % Front center
        200,  50, 200;    % Right
        200,  50, 250;    % Right high
        200, -50, 250;    % Left high
        200, -50, 200;    % Left
        250,   0, 150;    % Front low
        200,   0, 200;    % Back to start
    ];
    
    fprintf('Running demo sequence with %d positions...\n\n', size(targets, 1));
    
    for i = 1:size(targets, 1)
        target = targets(i, :);
        
        fprintf('Position %d/%d: [%.0f, %.0f, %.0f] mm\n', i, size(targets, 1), target);
        
        % Compute IK with auto pitch search
        [q, success, info] = inverseKinematicsAuto(target);
        
        if ~success
            fprintf('  Skipping - %s\n\n', info.message);
            continue;
        end
        
        fprintf('  Joint angles: [%.1f, %.1f, %.1f, %.1f] deg\n', rad2deg(q));
        fprintf('  Pitch used: %.1f deg\n', rad2deg(info.pitch_used));
        
        % Visualize
        cla(ax);
        hold(ax, 'on');
        plotRobotArm(q, ax, true, 25);
        plot3(ax, target(1), target(2), target(3), 'r*', 'MarkerSize', 20, 'LineWidth', 3);
        title(ax, sprintf('Target %d: [%.0f, %.0f, %.0f] mm, pitch=%.0f°', ...
            i, target, rad2deg(info.pitch_used)));
        drawnow;
        
        % Move hardware if connected
        if hardware
            moveToPosition(port_num, lib_name, target, 'auto', 80);
        else
            pause(1);  % Simulation delay
        end
        
        fprintf('\n');
    end
end

%% Interactive Mode - User Input
function runInteractive(ax, port_num, lib_name, hardware)
    fprintf('Interactive mode - Enter target positions\n');
    fprintf('Enter coordinates as: x y z (in mm)\n');
    fprintf('Type "quit" or "q" to exit\n\n');
    
    while true
        % Get user input
        input_str = input('Target [x y z]: ', 's');
        
        if strcmpi(input_str, 'quit') || strcmpi(input_str, 'q')
            fprintf('Exiting interactive mode.\n');
            break;
        end
        
        % Parse input
        try
            coords = sscanf(input_str, '%f %f %f');
            if length(coords) ~= 3
                fprintf('Invalid input. Enter 3 numbers: x y z\n\n');
                continue;
            end
            target = coords';
        catch
            fprintf('Invalid input. Enter 3 numbers: x y z\n\n');
            continue;
        end
        
        % Check reachability with auto pitch
        [q, success, info] = inverseKinematicsAuto(target);
        
        if ~success
            fprintf('Cannot reach [%.0f, %.0f, %.0f]: %s\n\n', target, info.message);
            continue;
        end
        
        fprintf('Solution: [%.1f, %.1f, %.1f, %.1f] deg\n', rad2deg(q));
        fprintf('Pitch used: %.1f deg, FK Error: %.3f mm\n', rad2deg(info.pitch_used), info.fk_error);
        
        % Visualize
        cla(ax);
        hold(ax, 'on');
        plotRobotArm(q, ax, true, 25);
        plot3(ax, target(1), target(2), target(3), 'r*', 'MarkerSize', 20, 'LineWidth', 3);
        title(ax, sprintf('Target: [%.0f, %.0f, %.0f] mm, pitch=%.0f°', target, rad2deg(info.pitch_used)));
        drawnow;
        
        % Move hardware
        if hardware
            confirm = input('Move robot? (y/n): ', 's');
            if strcmpi(confirm, 'y')
                moveToPosition(port_num, lib_name, target, 'auto', 80);
            end
        end
        
        fprintf('\n');
    end
end

%% Square Pattern - Draw a Square in 3D
function runSquarePattern(ax, port_num, lib_name, hardware)
    fprintf('Drawing square pattern in XZ plane...\n\n');
    
    % Square center and size - adjusted for ±90° limits with auto pitch
    center = [200, 0, 200];
    half_size = 30;  % mm
    
    % Square corners
    corners = [
        center(1)-half_size, 0, center(3)-half_size;
        center(1)+half_size, 0, center(3)-half_size;
        center(1)+half_size, 0, center(3)+half_size;
        center(1)-half_size, 0, center(3)+half_size;
        center(1)-half_size, 0, center(3)-half_size;  % Back to start
    ];
    
    % Interpolate between corners
    points_per_side = 10;
    trajectory = [];
    
    for i = 1:(size(corners, 1)-1)
        for t = linspace(0, 1, points_per_side)
            pt = corners(i,:) * (1-t) + corners(i+1,:) * t;
            trajectory = [trajectory; pt];
        end
    end
    
    fprintf('Trajectory has %d points\n\n', size(trajectory, 1));
    
    % Execute trajectory
    path_history = [];
    
    for i = 1:size(trajectory, 1)
        target = trajectory(i, :);
        
        [q, success, ~] = inverseKinematicsAuto(target);
        
        if ~success
            continue;
        end
        
        path_history = [path_history; target];
        
        % Visualize
        cla(ax);
        hold(ax, 'on');
        
        % Draw traced path
        if size(path_history, 1) > 1
            plot3(ax, path_history(:,1), path_history(:,2), path_history(:,3), ...
                'b-', 'LineWidth', 2);
        end
        
        % Draw robot
        plotRobotArm(q, ax, true, 20);
        
        % Draw target square outline
        plot3(ax, corners(:,1), corners(:,2), corners(:,3), 'r--', 'LineWidth', 1);
        
        % Current target point
        plot3(ax, target(1), target(2), target(3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
        
        title(ax, sprintf('Square Pattern - Point %d/%d', i, size(trajectory, 1)));
        drawnow;
        
        % Move hardware
        if hardware
            moveToPosition(port_num, lib_name, target, 'auto', 100);
        else
            pause(0.05);  % Simulation speed
        end
    end
    
    fprintf('Square pattern complete!\n');
end
