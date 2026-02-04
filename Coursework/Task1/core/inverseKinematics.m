function [q, success, info] = inverseKinematics(target_pos, target_orientation, elbow_config)
% INVERSEKINEMATICS Computes joint angles for OpenManipulator-X to reach target position
%
% Geometric inverse kinematics solution for 4-DOF arm. Since we have 4 joints
% and 6 DOF in task space, we can only fully control position (3 DOF) and
% one orientation parameter (pitch of end-effector).
%
% APPROACH:
%   1. q1 (base) determined by atan2(y, x)
%   2. Reduce to 2D planar problem in the vertical plane
%   3. Use law of cosines to solve the 2-link arm (q2, q3)
%   4. q4 adjusts end-effector pitch
%
% Usage:
%   [q, success, info] = inverseKinematics([x, y, z])
%   [q, success, info] = inverseKinematics([x, y, z], 'horizontal')
%   [q, success, info] = inverseKinematics([x, y, z], 'horizontal', 'elbow-up')
%
% Inputs:
%   target_pos         - [x, y, z] target position in mm
%   target_orientation - (Optional) End-effector orientation:
%                        'horizontal' - tool parallel to ground (default)
%                        'down'       - tool pointing downward
%                        angle        - explicit pitch angle in radians
%   elbow_config       - (Optional) 'elbow-up' or 'elbow-down' (default)
%
% Outputs:
%   q       - [q1, q2, q3, q4] joint angles in radians
%   success - true if solution found within joint limits
%   info    - Structure with solution details
%
% Author: OpenManipulator-X IK Implementation
% Date: February 2026

    %% Robot Physical Parameters (must match forwardKinematics.m)
    L_base = 77;                              % Height from base to joint 2
    L_prox_x = 24;                            % Horizontal offset of link 2
    L_prox_z = 128;                           % Vertical component of link 2
    L_prox = sqrt(L_prox_x^2 + L_prox_z^2);   % Diagonal length ≈ 130.23 mm
    L_dist = 124;                             % Link 3 length (joint 3 to 4)
    L_tool = 126;                             % Tool length (joint 4 to end-effector)
    
    % Mechanical offset angle
    beta = atan2(L_prox_x, L_prox_z);         % ≈ 0.1855 rad (10.62°)
    
    % Joint limits (±90° from home)
    JOINT_LIMIT = pi/2;
    
    %% Handle Optional Arguments
    if nargin < 2 || isempty(target_orientation)
        target_orientation = 'horizontal';
    end
    if nargin < 3 || isempty(elbow_config)
        elbow_config = 'elbow-down';
    end
    
    %% Extract Target Position
    x = target_pos(1);
    y = target_pos(2);
    z = target_pos(3);
    
    %% Initialize Output
    q = [0, 0, 0, 0];
    success = false;
    info = struct();
    info.reachable = false;
    info.within_limits = false;
    info.fk_error = Inf;
    info.fk_position = [0, 0, 0];
    info.message = '';
    
    %% Step 1: Base Angle (q1)
    % Rotation about Z-axis to align with target in XY plane
    q1 = atan2(y, x);
    
    %% Step 2: Reduce to 2D Planar Problem
    % Distance in XY plane
    r = sqrt(x^2 + y^2);
    
    % Height relative to shoulder joint (joint 2)
    z_rel = z - L_base;
    
    %% Step 3: Determine Wrist Position
    % The wrist (joint 4) position depends on desired end-effector orientation
    
    if ischar(target_orientation)
        switch lower(target_orientation)
            case 'horizontal'
                % Tool parallel to ground - wrist directly behind end-effector
                r_wrist = r - L_tool;
                z_wrist = z_rel;
                pitch_target = 0;
                
            case 'down'
                % Tool pointing straight down
                r_wrist = r;
                z_wrist = z_rel + L_tool;
                pitch_target = -pi/2;
                
            case 'forward'
                % Tool pointing in radial direction
                r_wrist = r - L_tool;
                z_wrist = z_rel;
                pitch_target = 0;
                
            otherwise
                warning('Unknown orientation: %s. Using horizontal.', target_orientation);
                r_wrist = r - L_tool;
                z_wrist = z_rel;
                pitch_target = 0;
        end
    else
        % Explicit pitch angle provided (radians)
        pitch_target = target_orientation;
        r_wrist = r - L_tool * cos(pitch_target);
        z_wrist = z_rel - L_tool * sin(pitch_target);
    end
    
    %% Step 4: Solve 2-Link Planar Arm (Law of Cosines)
    % Distance from shoulder to wrist
    D_sq = r_wrist^2 + z_wrist^2;
    D_reach = sqrt(D_sq);
    
    % Check reachability
    max_reach = L_prox + L_dist;
    min_reach = abs(L_prox - L_dist);
    
    if D_reach > max_reach || D_reach < min_reach
        info.reachable = false;
        info.message = sprintf('Target unreachable. Distance %.1f mm, range [%.1f, %.1f] mm', ...
            D_reach, min_reach, max_reach);
        return;
    end
    
    info.reachable = true;
    
    % Law of cosines to find elbow angle
    cos_q3 = (D_sq - L_prox^2 - L_dist^2) / (2 * L_prox * L_dist);
    
    % Clamp for numerical stability
    cos_q3 = max(-1, min(1, cos_q3));
    
    % Elbow angle (two solutions: elbow-up and elbow-down)
    if strcmpi(elbow_config, 'elbow-up')
        q3_raw = -atan2(sqrt(1 - cos_q3^2), cos_q3);
    else  % elbow-down (default)
        q3_raw = atan2(sqrt(1 - cos_q3^2), cos_q3);
    end
    
    %% Step 5: Shoulder Angle (q2)
    % Angle to wrist position
    phi = atan2(z_wrist, r_wrist);
    
    % Angle from shoulder-wrist line to link 2
    psi = atan2(L_dist * sin(q3_raw), L_prox + L_dist * cos(q3_raw));
    
    q2_raw = phi - psi;
    
    %% Step 6: Wrist Angle (q4)
    % Set wrist to achieve desired end-effector pitch
    % End-effector pitch = q2_raw + q3_raw + q4
    q4 = pitch_target - (q2_raw + q3_raw);
    
    %% Step 7: Apply Mechanical Offsets
    % Account for the slanted link geometry
    q2 = q2_raw - beta;
    q3 = q3_raw + beta;
    
    %% Step 8: Assemble Solution
    q = [q1, q2, q3, q4];
    
    %% Step 9: Check Joint Limits
    info.within_limits = all(abs(q) <= JOINT_LIMIT);
    
    if ~info.within_limits
        info.message = 'Solution exceeds joint limits (±90°):';
        for i = 1:4
            if abs(q(i)) > JOINT_LIMIT
                info.message = sprintf('%s q%d=%.1f°', info.message, i, rad2deg(q(i)));
            end
        end
    end
    
    %% Step 10: Verify with Forward Kinematics
    [~, fk_pos, ~] = forwardKinematics(q);
    info.fk_position = fk_pos';
    info.fk_error = norm(fk_pos' - target_pos);
    
    %% Determine Success
    success = info.reachable && info.within_limits && (info.fk_error < 1.0);
    
    if success
        info.message = sprintf('Solution found. FK error: %.3f mm', info.fk_error);
    elseif info.reachable && info.within_limits
        info.message = sprintf('FK verification failed. Error: %.3f mm', info.fk_error);
    end
end
