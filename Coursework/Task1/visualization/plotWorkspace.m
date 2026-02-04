function [workspace_pts, boundary_handle] = plotWorkspace(ax, num_samples, show_boundary)
% PLOTWORKSPACE Visualizes the reachable workspace of OpenManipulator-X
%
% Samples the joint space within operating limits and computes FK for each
% configuration to generate a 3D point cloud of reachable end-effector positions.
%
% JOINT LIMITS (matching Dynamixel ±90° from home):
%   All joints: [-π/2, +π/2] radians
%   Encoder range: [1024, 3072]
%
% Usage:
%   plotWorkspace()                           % Plot in new figure
%   plotWorkspace(ax)                         % Plot in specified axes
%   plotWorkspace(ax, 20)                     % 20 samples per joint
%   [pts, h] = plotWorkspace(ax, 15, true)    % Return points and boundary
%
% Inputs:
%   ax           - (Optional) Axes handle (default: new figure)
%   num_samples  - (Optional) Samples per joint, total = n^4 (default: 12)
%   show_boundary- (Optional) Show convex hull boundary (default: false)
%
% Outputs:
%   workspace_pts   - Nx3 matrix of [x, y, z] reachable positions
%   boundary_handle - Handle to boundary surface (if show_boundary=true)
%
% Author: OpenManipulator-X FK Simulation
% Date: February 2026

    %% Handle Optional Arguments
    if nargin < 1 || isempty(ax)
        figure('Name', 'OpenManipulator-X Workspace', 'Color', 'w');
        ax = axes;
    end
    if nargin < 2 || isempty(num_samples)
        num_samples = 12;  % 12^4 = 20,736 configurations
    end
    if nargin < 3 || isempty(show_boundary)
        show_boundary = false;
    end
    
    %% Define Joint Limits (±90° from home position)
    q_min = -pi/2;  % -90 degrees
    q_max = +pi/2;  % +90 degrees
    
    % Create joint angle sample vectors
    q1_range = linspace(q_min, q_max, num_samples);
    q2_range = linspace(q_min, q_max, num_samples);
    q3_range = linspace(q_min, q_max, num_samples);
    q4_range = linspace(q_min, q_max, max(3, floor(num_samples/3)));  % Fewer samples for wrist
    
    %% Compute Workspace Points
    fprintf('Computing workspace with %d configurations...\n', ...
        length(q1_range) * length(q2_range) * length(q3_range) * length(q4_range));
    
    % Preallocate for speed
    total_configs = length(q1_range) * length(q2_range) * length(q3_range) * length(q4_range);
    workspace_pts = zeros(total_configs, 3);
    
    idx = 1;
    tic;
    
    for q1 = q1_range
        for q2 = q2_range
            for q3 = q3_range
                for q4 = q4_range
                    % Compute FK
                    [~, pos, ~] = forwardKinematics([q1, q2, q3, q4]);
                    workspace_pts(idx, :) = pos';
                    idx = idx + 1;
                end
            end
        end
    end
    
    elapsed = toc;
    fprintf('Workspace computation completed in %.2f seconds\n', elapsed);
    fprintf('Total points: %d\n', size(workspace_pts, 1));
    
    %% Plot Workspace
    hold(ax, 'on');
    
    % Point cloud visualization
    scatter3(ax, workspace_pts(:,1), workspace_pts(:,2), workspace_pts(:,3), ...
        1, workspace_pts(:,3), 'filled', 'MarkerFaceAlpha', 0.3);
    
    colormap(ax, 'jet');
    cb = colorbar(ax);
    ylabel(cb, 'Z Height (mm)');
    
    %% Optional: Convex Hull Boundary
    boundary_handle = [];
    if show_boundary
        try
            % Compute convex hull
            K = convhull(workspace_pts(:,1), workspace_pts(:,2), workspace_pts(:,3));
            
            % Plot as semi-transparent surface
            boundary_handle = trisurf(K, ...
                workspace_pts(:,1), workspace_pts(:,2), workspace_pts(:,3), ...
                'FaceColor', 'cyan', 'FaceAlpha', 0.2, ...
                'EdgeColor', 'none', 'Parent', ax);
        catch ME
            warning('Could not compute convex hull: %s', ME.message);
        end
    end
    
    %% Configure Axes
    axis(ax, 'equal');
    grid(ax, 'on');
    xlabel(ax, 'X (mm)');
    ylabel(ax, 'Y (mm)');
    zlabel(ax, 'Z (mm)');
    title(ax, sprintf('OpenManipulator-X Reachable Workspace (±90° joints)\nTotal Points: %d', ...
        size(workspace_pts, 1)));
    view(ax, 45, 30);
    
    hold(ax, 'off');
    
    %% Print Workspace Statistics
    fprintf('\n--- Workspace Statistics ---\n');
    fprintf('X range: [%.1f, %.1f] mm\n', min(workspace_pts(:,1)), max(workspace_pts(:,1)));
    fprintf('Y range: [%.1f, %.1f] mm\n', min(workspace_pts(:,2)), max(workspace_pts(:,2)));
    fprintf('Z range: [%.1f, %.1f] mm\n', min(workspace_pts(:,3)), max(workspace_pts(:,3)));
    fprintf('Max reach: %.1f mm\n', max(sqrt(sum(workspace_pts.^2, 2))));
end
