function [q, success, info] = inverseKinematicsAuto(target_pos, options)
% INVERSEKINEMATICSAUTO Find IK solution with automatic pitch search
%
% Enhanced IK that searches for a valid end-effector pitch angle when
% the requested orientation cannot be achieved within joint limits.
%
% Usage:
%   [q, success, info] = inverseKinematicsAuto([x, y, z])
%   [q, success, info] = inverseKinematicsAuto([x, y, z], options)
%
% Inputs:
%   target_pos - [x, y, z] target position in mm
%   options    - (Optional) struct with fields:
%                .orientation    - 'horizontal', 'down', 'auto', or angle (rad)
%                .elbow          - 'elbow-up' or 'elbow-down' (default)
%                .pitch_range    - [min, max] pitch search range (rad)
%                .pitch_priority - 'horizontal' prefers pitch near 0
%                                  'nearest' finds closest valid pitch
%
% Outputs:
%   q       - [q1, q2, q3, q4] joint angles in radians
%   success - true if solution found within joint limits
%   info    - Structure with solution details
%
% Author: OpenManipulator-X IK Implementation
% Date: February 2026

    %% Default Options
    if nargin < 2
        options = struct();
    end
    
    if ~isfield(options, 'orientation')
        options.orientation = 'auto';  % Default to auto-search
    end
    if ~isfield(options, 'elbow')
        options.elbow = 'elbow-down';
    end
    if ~isfield(options, 'pitch_range')
        options.pitch_range = [-pi/2, pi/2];  % ±90° pitch
    end
    if ~isfield(options, 'pitch_priority')
        options.pitch_priority = 'horizontal';
    end
    
    %% Initialize Output
    q = [0, 0, 0, 0];
    success = false;
    info = struct();
    info.pitch_searched = false;
    info.pitch_used = NaN;
    
    %% Check if Orientation is 'auto'
    if ischar(options.orientation) && strcmpi(options.orientation, 'auto')
        % Search for valid pitch
        info.pitch_searched = true;
        
        % Try pitches in order of preference
        if strcmpi(options.pitch_priority, 'horizontal')
            % Prefer pitch near 0 (horizontal)
            pitches = [0, deg2rad(10), deg2rad(-10), deg2rad(20), deg2rad(-20), ...
                       deg2rad(30), deg2rad(-30), deg2rad(45), deg2rad(-45), ...
                       deg2rad(60), deg2rad(-60), deg2rad(75), deg2rad(-75), ...
                       pi/2, -pi/2];
        else
            % Dense search
            pitches = linspace(options.pitch_range(1), options.pitch_range(2), 37);
        end
        
        % Also try both elbow configurations
        elbows = {options.elbow};
        if strcmpi(options.elbow, 'elbow-down')
            elbows{2} = 'elbow-up';
        else
            elbows{2} = 'elbow-down';
        end
        
        best_q = [];
        best_info = struct();
        best_pitch = NaN;
        best_error = Inf;
        
        for e = 1:length(elbows)
            for p = pitches
                [q_try, success_try, info_try] = inverseKinematics(target_pos, p, elbows{e});
                
                if success_try
                    % Found valid solution
                    q = q_try;
                    success = true;
                    info = info_try;
                    info.pitch_searched = true;
                    info.pitch_used = p;
                    info.elbow_used = elbows{e};
                    info.message = sprintf('Solution found with pitch=%.1f°, %s. FK error: %.3f mm', ...
                        rad2deg(p), elbows{e}, info.fk_error);
                    return;
                end
                
                % Track closest to valid (smallest joint limit violation)
                if info_try.reachable && info_try.fk_error < best_error
                    best_q = q_try;
                    best_info = info_try;
                    best_pitch = p;
                    best_error = info_try.fk_error;
                end
            end
        end
        
        % No valid solution found
        info.message = 'No valid solution found for any pitch angle.';
        if ~isempty(best_q)
            q = best_q;
            info.pitch_used = best_pitch;
            info.fk_error = best_error;
            info.message = sprintf('%s Best attempt: pitch=%.1f°, FK error=%.3f mm. %s', ...
                info.message, rad2deg(best_pitch), best_error, best_info.message);
        end
        
    else
        % Use specified orientation directly
        [q, success, info] = inverseKinematics(target_pos, options.orientation, options.elbow);
        info.pitch_searched = false;
        
        if ischar(options.orientation)
            switch lower(options.orientation)
                case 'horizontal', info.pitch_used = 0;
                case 'down', info.pitch_used = -pi/2;
                otherwise, info.pitch_used = NaN;
            end
        else
            info.pitch_used = options.orientation;
        end
    end
end
