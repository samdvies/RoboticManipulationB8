%% OpenManipulator-X Forward Kinematics Simulation
% Main script to run FK visualization and optionally record video
%
% This script demonstrates the forward kinematics of the OpenManipulator-X
% robot arm using DH parameters. It can run in simulation-only mode or
% connect to real hardware.
%
% Author: OpenManipulator-X FK Simulation
% Date: February 2026

clear all; close all; clc;

%% Configuration
RECORD_VIDEO = true;          % Set true to record video
VIDEO_FILENAME = 'FK_Demo.mp4';
SHOW_WORKSPACE = false;       % Set true to show workspace cloud
INTERACTIVE_MODE = false;     % Set false for batch mode (no figure interaction)

%% Display Angle Conversion Reference
fprintf('=== OpenManipulator-X FK Simulation ===\n\n');
angleConversion('table');

%% Test Forward Kinematics at Home Position
fprintf('\n--- Testing FK at Home Position ---\n');
q_home = [0, 0, 0, 0];
[T_all, pos, R] = forwardKinematics(q_home);

fprintf('Home position joint angles: [%.2f, %.2f, %.2f, %.2f] rad\n', q_home);
fprintf('End-effector position: [%.2f, %.2f, %.2f] mm\n', pos(1), pos(2), pos(3));

%% Create Interactive Figure
fig = figure('Name', 'OpenManipulator-X FK Simulation', ...
    'NumberTitle', 'off', 'Position', [100, 100, 1200, 800], 'Color', 'w');

%% Plot 1: Robot at Home Position
subplot(1, 2, 1);
ax1 = gca;
plotRobotArm(q_home, ax1, true, 25);
title(ax1, 'Home Position (q = [0, 0, 0, 0])');

%% Plot 2: Robot at Test Configuration
subplot(1, 2, 2);
ax2 = gca;
q_test = [deg2rad(45), deg2rad(-30), deg2rad(60), deg2rad(-15)];
plotRobotArm(q_test, ax2, true, 25);

% Calculate end-effector position for test config
[~, pos_test, ~] = forwardKinematics(q_test);
title(ax2, sprintf('Test Config | End-Effector: [%.1f, %.1f, %.1f] mm', ...
    pos_test(1), pos_test(2), pos_test(3)));

%% Interactive Joint Control Demo (skip in batch mode)
if INTERACTIVE_MODE
    fprintf('\n--- Interactive Demo ---\n');
    fprintf('Press any key to start joint sweep animation...\n');
    fprintf('(Close figure window to exit)\n\n');
    waitforbuttonpress;

    % Single axis for animation
    figure('Name', 'FK Animation', 'NumberTitle', 'off', ...
        'Position', [150, 150, 1000, 700], 'Color', 'w');
    ax_anim = gca;

    % Sweep each joint
    joint_names = {'Base (q1)', 'Shoulder (q2)', 'Elbow (q3)', 'Wrist (q4)'};
    sweep_angles = linspace(-pi/2, pi/2, 45);

    for joint_idx = 1:4
        fprintf('Sweeping %s...\n', joint_names{joint_idx});
        
        for angle = sweep_angles
            if ~isvalid(ax_anim)
                fprintf('Figure closed. Exiting.\n');
                return;
            end
            
            % Set joint angles
            q = [0, 0, 0, 0];
            q(joint_idx) = angle;
            
            % Update plot
            cla(ax_anim);
            plotRobotArm(q, ax_anim, true, 25);
            
            % Add info text
            [~, pos_curr, ~] = forwardKinematics(q);
            title(ax_anim, sprintf('%s: %.1f° | End-Effector: [%.1f, %.1f, %.1f] mm', ...
                joint_names{joint_idx}, rad2deg(angle), pos_curr(1), pos_curr(2), pos_curr(3)));
            
            drawnow;
            pause(0.03);
        end
        
        % Return to home
        for angle = flip(sweep_angles)
            if ~isvalid(ax_anim)
                return;
            end
            
            q = [0, 0, 0, 0];
            q(joint_idx) = angle;
            
            cla(ax_anim);
            plotRobotArm(q, ax_anim, true, 25);
            drawnow;
            pause(0.02);
        end
    end
else
    fprintf('\n--- Batch Mode: Skipping interactive demo ---\n');
end

%% Optional: Plot Workspace
if SHOW_WORKSPACE
    fprintf('\n--- Computing Workspace ---\n');
    figure('Name', 'Workspace Visualization', 'Color', 'w');
    plotWorkspace(gca, 12, true);
end

%% Optional: Record Video
if RECORD_VIDEO
    fprintf('\n--- Recording Video ---\n');
    recordFKDemo(VIDEO_FILENAME, 30, false);
end

fprintf('\n=== Simulation Complete ===\n');
