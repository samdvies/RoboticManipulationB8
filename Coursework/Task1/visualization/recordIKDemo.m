%% Record IK Demo Video with 3D Coordinate Frames
% Creates a video demonstrating inverse kinematics for OpenManipulator-X
% Shows the robot arm with coordinate frames at each joint
%
% Author: OpenManipulator-X IK Implementation
% Date: February 2026

clear all; close all; clc;

fprintf('=== Recording IK Demo Video with Coordinate Frames ===\n\n');

%% Video Setup
video_filename = 'IK_Demo.mp4';
frame_rate = 30;

v = VideoWriter(video_filename, 'MPEG-4');
v.FrameRate = frame_rate;
v.Quality = 95;
open(v);

%% Figure Setup - Proper 3D configuration
fig = figure('Name', 'IK Demo Recording', 'Position', [100 100 1280 720], 'Color', 'w');

%% Helper function to setup consistent 3D view
    function setupAxes(ax)
        axis(ax, 'equal');
        grid(ax, 'on');
        xlabel(ax, 'X (mm)', 'FontSize', 12);
        ylabel(ax, 'Y (mm)', 'FontSize', 12);
        zlabel(ax, 'Z (mm)', 'FontSize', 12);
        xlim(ax, [-300, 400]);
        ylim(ax, [-350, 350]);
        zlim(ax, [-50, 450]);
        view(ax, 135, 25);  % Good angle to see 3D structure
        set(ax, 'FontSize', 10);
    end

%% Demo Sequence
fprintf('Recording demo sequence...\n\n');

%% Part 1: Title Screen (2 seconds)
clf;
text(0.5, 0.65, 'Inverse Kinematics Demo', 'FontSize', 36, 'FontWeight', 'bold', ...
    'HorizontalAlignment', 'center', 'Units', 'normalized');
text(0.5, 0.50, 'OpenManipulator-X', 'FontSize', 24, ...
    'HorizontalAlignment', 'center', 'Units', 'normalized');
text(0.5, 0.38, 'with 3D Coordinate Frames', 'FontSize', 18, ...
    'HorizontalAlignment', 'center', 'Units', 'normalized', 'Color', [0.2 0.5 0.2]);
text(0.5, 0.25, 'X = Red  |  Y = Green  |  Z = Blue', 'FontSize', 14, ...
    'HorizontalAlignment', 'center', 'Units', 'normalized', 'Color', [0.4 0.4 0.4]);
axis off;

for f = 1:round(2.5*frame_rate)
    writeVideo(v, getframe(fig));
end

%% Part 2: Initial Pose with Frame Labels
fprintf('Showing initial pose with frames...\n');

clf;
ax = axes('Parent', fig);
q_home = [0, 0, 0, 0];
plotRobotArm(q_home, ax, true, 35);  % Show frames with 35mm scale
setupAxes(ax);
title(ax, 'Home Position - Coordinate Frames at Each Joint', 'FontSize', 14, 'FontWeight', 'bold');

% Add legend for frame colors
annotation(fig, 'textbox', [0.02, 0.85, 0.15, 0.12], ...
    'String', {'Frame Colors:', 'X-axis: Red', 'Y-axis: Green', 'Z-axis: Blue'}, ...
    'FontSize', 10, 'BackgroundColor', 'w', 'EdgeColor', 'k');

for f = 1:round(3*frame_rate)
    writeVideo(v, getframe(fig));
end

%% Part 3: Target Position Reaching Demo
targets = [
    200,   0, 200;    % Front center
    250,   0, 150;    % Forward low
    180,  80, 220;    % Right side
    180, -80, 220;    % Left side
    200,   0, 300;    % High up
    220,  40, 180;    % Mixed position
    200,   0, 200;    % Back to start
];

target_names = {
    'Front Center', 'Forward Low', 'Right Side', 'Left Side', ...
    'High Position', 'Mixed Position', 'Return Home'
};

% Initial pose
q_current = [0, 0, 0, 0];

for t = 1:size(targets, 1)
    target = targets(t, :);
    
    % Compute IK with auto pitch
    [q_target, success, info] = inverseKinematicsAuto(target);
    
    if ~success
        fprintf('Skipping target %d - unreachable\n', t);
        continue;
    end
    
    fprintf('Target %d: [%.0f, %.0f, %.0f] mm - %s\n', t, target, target_names{t});
    
    % Animate transition (1.5 seconds)
    n_frames = round(1.5 * frame_rate);
    
    for f = 1:n_frames
        % Interpolate joint angles with smooth easing
        alpha = f / n_frames;
        alpha_smooth = 0.5 * (1 - cos(pi * alpha));
        q_interp = q_current * (1 - alpha_smooth) + q_target * alpha_smooth;
        
        % Compute current end-effector position
        [~, current_pos, ~] = forwardKinematics(q_interp);
        
        % Clear and redraw
        clf;
        ax = axes('Parent', fig);
        hold(ax, 'on');
        
        % Draw robot arm WITH coordinate frames
        plotRobotArm(q_interp, ax, true, 30);  % Frames enabled, 30mm scale
        hold(ax, 'on');  % Re-enable hold after plotRobotArm
        
        % Target marker (green star)
        plot3(ax, target(1), target(2), target(3), 'p', ...
            'MarkerSize', 25, 'MarkerFaceColor', [0.2 0.8 0.2], ...
            'MarkerEdgeColor', 'k', 'LineWidth', 2);
        
        % Line from EE to target (dashed)
        plot3(ax, [current_pos(1), target(1)], [current_pos(2), target(2)], ...
            [current_pos(3), target(3)], 'm--', 'LineWidth', 2);
        
        % Setup consistent 3D view
        setupAxes(ax);
        
        % Title
        title(ax, sprintf('Moving to: %s [%.0f, %.0f, %.0f] mm', ...
            target_names{t}, target), 'FontSize', 14, 'FontWeight', 'bold');
        
        % Info box with joint angles
        dist = norm(current_pos' - target);
        info_str = sprintf(['Target: [%.0f, %.0f, %.0f] mm\n' ...
                           'Pitch: %.0f°\n' ...
                           'q1=%.1f° q2=%.1f°\n' ...
                           'q3=%.1f° q4=%.1f°\n' ...
                           'Dist: %.1f mm'], ...
            target, rad2deg(info.pitch_used), ...
            rad2deg(q_interp(1)), rad2deg(q_interp(2)), ...
            rad2deg(q_interp(3)), rad2deg(q_interp(4)), dist);
        
        annotation(fig, 'textbox', [0.02, 0.02, 0.18, 0.22], ...
            'String', info_str, 'FontSize', 9, 'BackgroundColor', 'w', ...
            'EdgeColor', 'k', 'FitBoxToText', 'off');
        
        hold(ax, 'off');
        drawnow;
        
        writeVideo(v, getframe(fig));
    end
    
    % Hold at target (0.5 seconds)
    for f = 1:round(0.5 * frame_rate)
        writeVideo(v, getframe(fig));
    end
    
    q_current = q_target;
end

%% Part 4: Rotating View Demo
fprintf('\nRecording rotating view...\n');

% Title card
clf;
text(0.5, 0.5, '360° View of Robot Configuration', 'FontSize', 28, 'FontWeight', 'bold', ...
    'HorizontalAlignment', 'center', 'Units', 'normalized');
axis off;
for f = 1:round(1*frame_rate)
    writeVideo(v, getframe(fig));
end

% Set a interesting pose
q_demo = deg2rad([30, 20, 45, -30]);
[~, pos_demo, ~] = forwardKinematics(q_demo);

% Rotate around the arm
n_rotation_frames = round(4 * frame_rate);  % 4 second rotation

for f = 1:n_rotation_frames
    clf;
    ax = axes('Parent', fig);
    hold(ax, 'on');
    
    % Draw robot with frames
    plotRobotArm(q_demo, ax, true, 35);
    hold(ax, 'on');  % Re-enable hold after plotRobotArm
    
    % Mark end-effector target
    plot3(ax, pos_demo(1), pos_demo(2), pos_demo(3), 'rp', ...
        'MarkerSize', 20, 'MarkerFaceColor', 'r', 'LineWidth', 2);
    
    % Setup axes
    axis(ax, 'equal');
    grid(ax, 'on');
    xlabel(ax, 'X (mm)', 'FontSize', 12);
    ylabel(ax, 'Y (mm)', 'FontSize', 12);
    zlabel(ax, 'Z (mm)', 'FontSize', 12);
    xlim(ax, [-300, 400]);
    ylim(ax, [-350, 350]);
    zlim(ax, [-50, 450]);
    
    % Rotating view
    azimuth = 360 * f / n_rotation_frames;
    view(ax, azimuth, 25);
    
    title(ax, '360° View - Observe Frame Orientations', 'FontSize', 14, 'FontWeight', 'bold');
    
    % Frame legend
    annotation(fig, 'textbox', [0.82, 0.82, 0.16, 0.15], ...
        'String', {'Frames:', '{0} Base', '{1}-{4} Joints', '{tool} End'}, ...
        'FontSize', 9, 'BackgroundColor', 'w', 'EdgeColor', 'k');
    
    hold(ax, 'off');
    drawnow;
    writeVideo(v, getframe(fig));
end

%% Part 5: Square Drawing Demo
fprintf('\nRecording square pattern...\n');

% Title card
clf;
text(0.5, 0.5, 'Drawing a Square Pattern', 'FontSize', 28, 'FontWeight', 'bold', ...
    'HorizontalAlignment', 'center', 'Units', 'normalized');
axis off;
for f = 1:round(1.5*frame_rate)
    writeVideo(v, getframe(fig));
end

% Square parameters
center = [200, 0, 200];
half_size = 25;

corners = [
    center(1)-half_size, 0, center(3)-half_size;
    center(1)+half_size, 0, center(3)-half_size;
    center(1)+half_size, 0, center(3)+half_size;
    center(1)-half_size, 0, center(3)+half_size;
    center(1)-half_size, 0, center(3)-half_size;
];

% Generate trajectory
points_per_side = 12;
trajectory = [];
for i = 1:(size(corners, 1)-1)
    for t_val = linspace(0, 1, points_per_side)
        pt = corners(i,:) * (1-t_val) + corners(i+1,:) * t_val;
        trajectory = [trajectory; pt];
    end
end

% Trace the square
path_history = [];
[q_current, ~, ~] = inverseKinematicsAuto(trajectory(1,:));

for i = 1:size(trajectory, 1)
    target = trajectory(i, :);
    [q_target, success, ~] = inverseKinematicsAuto(target);
    
    if ~success
        continue;
    end
    
    path_history = [path_history; target];
    
    % Quick transition (4 frames)
    for f = 1:4
        alpha = f / 4;
        q_interp = q_current * (1 - alpha) + q_target * alpha;
        
        clf;
        ax = axes('Parent', fig);
        hold(ax, 'on');
        
        % Robot with frames
        plotRobotArm(q_interp, ax, true, 25);
        hold(ax, 'on');  % Re-enable hold after plotRobotArm
        
        % Traced path (thick blue)
        if size(path_history, 1) > 1
            plot3(ax, path_history(:,1), path_history(:,2), path_history(:,3), ...
                'b-', 'LineWidth', 4);
        end
        
        % Target square outline
        plot3(ax, corners(:,1), corners(:,2), corners(:,3), 'r--', 'LineWidth', 2);
        
        % Current point
        plot3(ax, target(1), target(2), target(3), 'go', ...
            'MarkerSize', 15, 'MarkerFaceColor', 'g', 'LineWidth', 2);
        
        setupAxes(ax);
        title(ax, sprintf('Tracing Square - Point %d/%d', i, size(trajectory, 1)), ...
            'FontSize', 14, 'FontWeight', 'bold');
        
        hold(ax, 'off');
        drawnow;
        writeVideo(v, getframe(fig));
    end
    
    q_current = q_target;
end

% Hold final frame
for f = 1:round(1.5*frame_rate)
    writeVideo(v, getframe(fig));
end

%% Part 6: End Screen
clf;
text(0.5, 0.65, 'IK Demo Complete', 'FontSize', 36, 'FontWeight', 'bold', ...
    'HorizontalAlignment', 'center', 'Units', 'normalized');
text(0.5, 0.45, 'OpenManipulator-X Inverse Kinematics', 'FontSize', 20, ...
    'HorizontalAlignment', 'center', 'Units', 'normalized');
text(0.5, 0.30, 'with Coordinate Frames Visualization', 'FontSize', 16, ...
    'HorizontalAlignment', 'center', 'Units', 'normalized', 'Color', [0.3 0.3 0.3]);
axis off;

for f = 1:round(2*frame_rate)
    writeVideo(v, getframe(fig));
end

%% Finalize
close(v);
close(fig);

% Get video info
info = dir(video_filename);
fprintf('\n=== Video Recording Complete ===\n');
fprintf('Filename: %s\n', video_filename);
fprintf('Size: %.2f MB\n', info.bytes / 1024 / 1024);
fprintf('Duration: ~%.0f seconds\n', v.FrameCount / frame_rate);
fprintf('\nVideo shows:\n');
fprintf('  - Robot arm with 3D coordinate frames at each joint\n');
fprintf('  - RGB axes: X=Red, Y=Green, Z=Blue\n');
fprintf('  - Smooth transitions between target positions\n');
fprintf('  - 360 degree rotating view\n');
fprintf('  - Square pattern tracing\n');
