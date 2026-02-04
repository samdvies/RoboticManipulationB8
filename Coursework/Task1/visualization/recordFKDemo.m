function recordFKDemo(filename, fps, show_workspace)
% RECORDFKDEMO Records a video demonstrating forward kinematics of OpenManipulator-X
%
% Creates a video showing each joint rotating independently from -90° to +90°,
% demonstrating correct DH parameter implementation. Optionally shows workspace.
%
% VIDEO SEQUENCE:
%   1. Home position (all joints at 0°)
%   2. Joint 1 sweep (-90° to +90° and back)
%   3. Joint 2 sweep
%   4. Joint 3 sweep
%   5. Joint 4 sweep
%   6. Combined motion demo
%
% Usage:
%   recordFKDemo()                              % Default: 'FK_Demo.mp4', 30fps
%   recordFKDemo('my_video.mp4')                % Custom filename
%   recordFKDemo('my_video.mp4', 30, true)      % With workspace visualization
%
% Inputs:
%   filename       - (Optional) Output video filename (default: 'FK_Demo.mp4')
%   fps            - (Optional) Frames per second (default: 30)
%   show_workspace - (Optional) Show workspace boundary (default: false)
%
% Author: OpenManipulator-X FK Simulation
% Date: February 2026

    %% Handle Optional Arguments
    if nargin < 1 || isempty(filename)
        filename = 'FK_Demo.mp4';
    end
    if nargin < 2 || isempty(fps)
        fps = 30;
    end
    if nargin < 3 || isempty(show_workspace)
        show_workspace = false;
    end
    
    %% Setup Video Writer
    [~, ~, ext] = fileparts(filename);
    if strcmpi(ext, '.mp4')
        v = VideoWriter(filename, 'MPEG-4');
    else
        v = VideoWriter(filename);
    end
    v.FrameRate = fps;
    v.Quality = 95;
    
    %% Setup Figure
    fig = figure('Name', 'FK Demo Recording', 'NumberTitle', 'off', ...
        'Position', [100, 100, 1280, 720], 'Color', 'w');
    ax = axes('Parent', fig);
    
    %% Compute Workspace (if requested)
    workspace_pts = [];
    if show_workspace
        fprintf('Computing workspace for visualization...\n');
        [workspace_pts, ~] = computeWorkspacePoints(10);  % Quick computation
    end
    
    %% Animation Parameters
    joint_range = linspace(-pi/2, pi/2, 60);    % 60 frames per sweep
    joint_range_back = flip(joint_range);
    
    joint_names = {'Base (q1)', 'Shoulder (q2)', 'Elbow (q3)', 'Wrist (q4)'};
    
    %% Open Video File
    open(v);
    fprintf('Recording started: %s\n', filename);
    
    %% Section 1: Home Position (2 seconds)
    fprintf('Recording home position...\n');
    q_home = [0, 0, 0, 0];
    
    for frame = 1:(2*fps)
        cla(ax);
        plotFrame(ax, q_home, 'Home Position (All joints at 0°)', workspace_pts);
        drawnow;
        writeVideo(v, getframe(fig));
    end
    
    %% Section 2-5: Individual Joint Sweeps
    for joint_idx = 1:4
        fprintf('Recording Joint %d sweep...\n', joint_idx);
        
        % Sweep forward (-90° to +90°)
        for angle = joint_range
            q = [0, 0, 0, 0];
            q(joint_idx) = angle;
            
            cla(ax);
            title_str = sprintf('%s: %.1f°', joint_names{joint_idx}, rad2deg(angle));
            plotFrame(ax, q, title_str, workspace_pts);
            drawnow;
            writeVideo(v, getframe(fig));
        end
        
        % Sweep back (+90° to -90°)
        for angle = joint_range_back
            q = [0, 0, 0, 0];
            q(joint_idx) = angle;
            
            cla(ax);
            title_str = sprintf('%s: %.1f°', joint_names{joint_idx}, rad2deg(angle));
            plotFrame(ax, q, title_str, workspace_pts);
            drawnow;
            writeVideo(v, getframe(fig));
        end
        
        % Pause at home (0.5 seconds)
        q = [0, 0, 0, 0];
        for frame = 1:round(0.5*fps)
            cla(ax);
            plotFrame(ax, q, 'Home Position', workspace_pts);
            drawnow;
            writeVideo(v, getframe(fig));
        end
    end
    
    %% Section 6: Combined Motion Demo
    fprintf('Recording combined motion demo...\n');
    t = linspace(0, 4*pi, 120);  % 4 seconds of motion
    
    for i = 1:length(t)
        % Create interesting combined motion
        q = [0.5*sin(t(i)), ...
             0.4*sin(t(i)*0.7), ...
             0.6*sin(t(i)*1.3 + pi/4), ...
             0.3*sin(t(i)*2)];
        
        cla(ax);
        plotFrame(ax, q, 'Combined Motion Demo', workspace_pts);
        drawnow;
        writeVideo(v, getframe(fig));
    end
    
    %% Section 7: Return to Home (1 second)
    fprintf('Recording return to home...\n');
    q_final = q;
    for alpha = linspace(0, 1, fps)
        q = q_final * (1 - alpha);  % Linear interpolation to home
        
        cla(ax);
        plotFrame(ax, q, 'Returning to Home', workspace_pts);
        drawnow;
        writeVideo(v, getframe(fig));
    end
    
    %% Finalize
    close(v);
    close(fig);
    
    fprintf('\n=== Recording Complete ===\n');
    fprintf('Video saved: %s\n', filename);
    fprintf('Duration: %.1f seconds\n', v.FrameCount / fps);
    fprintf('Resolution: 1280x720 @ %d fps\n', fps);
end

%% Helper Function: Plot Single Frame
function plotFrame(ax, q, title_str, workspace_pts)
    hold(ax, 'on');
    
    % Plot workspace points if provided
    if ~isempty(workspace_pts)
        scatter3(ax, workspace_pts(:,1), workspace_pts(:,2), workspace_pts(:,3), ...
            1, [0.8, 0.8, 0.8], 'filled', 'MarkerFaceAlpha', 0.1);
    end
    
    % Plot robot
    plotRobotArm(q, ax, true, 25);
    
    % Configure axes
    axis(ax, 'equal');
    grid(ax, 'on');
    xlabel(ax, 'X (mm)', 'FontSize', 12);
    ylabel(ax, 'Y (mm)', 'FontSize', 12);
    zlabel(ax, 'Z (mm)', 'FontSize', 12);
    title(ax, title_str, 'FontSize', 14, 'FontWeight', 'bold');
    
    xlim(ax, [-400, 400]);
    ylim(ax, [-400, 400]);
    zlim(ax, [-50, 450]);
    view(ax, 45, 30);
    
    % Add angle readout
    text(ax, -350, -350, 400, ...
        sprintf('q1: %+6.1f°\nq2: %+6.1f°\nq3: %+6.1f°\nq4: %+6.1f°', ...
        rad2deg(q(1)), rad2deg(q(2)), rad2deg(q(3)), rad2deg(q(4))), ...
        'FontSize', 11, 'FontName', 'FixedWidth', ...
        'BackgroundColor', 'white', 'EdgeColor', 'black');
    
    % Add coordinate frame legend
    text(ax, 300, -350, 400, ...
        sprintf('Frame Colors:\nX = Red\nY = Green\nZ = Blue'), ...
        'FontSize', 10, 'BackgroundColor', 'white', 'EdgeColor', 'black');
    
    hold(ax, 'off');
end

%% Helper Function: Quick Workspace Computation
function pts = computeWorkspacePoints(num_samples)
    q_range = linspace(-pi/2, pi/2, num_samples);
    total = num_samples^4;
    pts = zeros(total, 3);
    
    idx = 1;
    for q1 = q_range
        for q2 = q_range
            for q3 = q_range
                for q4 = q_range(1:3:end)  % Fewer wrist samples
                    [~, pos, ~] = forwardKinematics([q1, q2, q3, q4]);
                    pts(idx, :) = pos';
                    idx = idx + 1;
                    if idx > total
                        return;
                    end
                end
            end
        end
    end
    pts = pts(1:idx-1, :);
end
