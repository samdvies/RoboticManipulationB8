function h = plotRobotArm(q, ax, show_frames, frame_scale)
% PLOTROBOTARM Visualizes OpenManipulator-X using line objects and coordinate frames
%
% Draws the robot arm as thick line segments connecting joint positions,
% with optional RGB coordinate frames at each joint for debugging FK/IK.
%
% COORDINATE FRAME COLORS (Standard Convention):
%   X-axis: RED
%   Y-axis: GREEN
%   Z-axis: BLUE
%
% Usage:
%   h = plotRobotArm(q)                    % Basic plot in current axes
%   h = plotRobotArm(q, ax)                % Plot in specified axes
%   h = plotRobotArm(q, ax, true)          % With coordinate frames
%   h = plotRobotArm(q, ax, true, 30)      % Custom frame axis length (mm)
%
% Inputs:
%   q           - 1x4 joint angles in radians [q1, q2, q3, q4]
%   ax          - (Optional) Axes handle to plot into (default: gca)
%   show_frames - (Optional) Boolean to show coordinate frames (default: true)
%   frame_scale - (Optional) Length of frame axes in mm (default: 25)
%
% Output:
%   h - Structure containing handles to all graphics objects for updating
%
% Author: OpenManipulator-X FK Simulation
% Date: February 2026

    %% Handle Optional Arguments
    if nargin < 2 || isempty(ax)
        ax = gca;
    end
    if nargin < 3 || isempty(show_frames)
        show_frames = true;
    end
    if nargin < 4 || isempty(frame_scale)
        frame_scale = 25;  % mm
    end
    
    %% Compute Forward Kinematics
    [T_all, ~, ~] = forwardKinematics(q);
    
    %% Extract Joint Positions
    % Base frame origin
    p0 = [0; 0; 0];
    
    % Joint positions from transformation matrices
    p1 = T_all{1}(1:3, 4);      % Joint 1
    p2 = T_all{2}(1:3, 4);      % Joint 2
    p3 = T_all{3}(1:3, 4);      % Joint 3
    p4 = T_all{4}(1:3, 4);      % Joint 4
    p_tool = T_all{5}(1:3, 4);  % End-effector
    
    % Combine into matrix for plotting
    pts = [p0, p1, p2, p3, p4, p_tool];
    
    %% Plot Robot Arm Links
    hold(ax, 'on');
    
    % Main arm structure - thick black line with circular markers at joints
    h.arm_line = plot3(ax, pts(1,:), pts(2,:), pts(3,:), ...
        '-k', 'LineWidth', 5, 'MarkerSize', 10);
    
    % Joint markers - red filled circles
    h.joint_markers = plot3(ax, pts(1,1:5), pts(2,1:5), pts(3,1:5), ...
        'o', 'MarkerSize', 8, 'MarkerFaceColor', 'r', ...
        'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
    
    % End-effector marker - larger green diamond
    h.endeff_marker = plot3(ax, p_tool(1), p_tool(2), p_tool(3), ...
        'd', 'MarkerSize', 12, 'MarkerFaceColor', 'g', ...
        'MarkerEdgeColor', 'k', 'LineWidth', 2);
    
    %% Plot Coordinate Frames (Optional)
    if show_frames
        % Frame labels
        frame_names = {'{0}', '{1}', '{2}', '{3}', '{4}', '{tool}'};
        
        % Transformation matrices including base frame
        T_frames = [{eye(4)}, T_all];
        
        % Store frame handles
        h.frames = cell(1, 6);
        h.frame_labels = cell(1, 6);
        
        for i = 1:6
            T = T_frames{i};
            origin = T(1:3, 4);
            
            % Extract rotation axes (columns of rotation matrix)
            x_axis = T(1:3, 1) * frame_scale;
            y_axis = T(1:3, 2) * frame_scale;
            z_axis = T(1:3, 3) * frame_scale;
            
            % Draw X-axis (RED)
            h.frames{i}.x = plot3(ax, ...
                [origin(1), origin(1) + x_axis(1)], ...
                [origin(2), origin(2) + x_axis(2)], ...
                [origin(3), origin(3) + x_axis(3)], ...
                'r-', 'LineWidth', 2);
            
            % Draw Y-axis (GREEN)
            h.frames{i}.y = plot3(ax, ...
                [origin(1), origin(1) + y_axis(1)], ...
                [origin(2), origin(2) + y_axis(2)], ...
                [origin(3), origin(3) + y_axis(3)], ...
                'g-', 'LineWidth', 2);
            
            % Draw Z-axis (BLUE)
            h.frames{i}.z = plot3(ax, ...
                [origin(1), origin(1) + z_axis(1)], ...
                [origin(2), origin(2) + z_axis(2)], ...
                [origin(3), origin(3) + z_axis(3)], ...
                'b-', 'LineWidth', 2);
            
            % Add frame label
            h.frame_labels{i} = text(ax, ...
                origin(1) + 10, origin(2) + 10, origin(3) + 10, ...
                frame_names{i}, 'FontSize', 10, 'FontWeight', 'bold');
        end
    end
    
    %% Configure Axes
    axis(ax, 'equal');
    grid(ax, 'on');
    xlabel(ax, 'X (mm)');
    ylabel(ax, 'Y (mm)');
    zlabel(ax, 'Z (mm)');
    title(ax, sprintf('OpenManipulator-X | q = [%.1f°, %.1f°, %.1f°, %.1f°]', ...
        rad2deg(q(1)), rad2deg(q(2)), rad2deg(q(3)), rad2deg(q(4))));
    
    % Set reasonable view angle
    view(ax, 45, 30);
    
    % Set axis limits to show full workspace
    xlim(ax, [-400, 400]);
    ylim(ax, [-400, 400]);
    zlim(ax, [-50, 450]);
    
    hold(ax, 'off');
end
