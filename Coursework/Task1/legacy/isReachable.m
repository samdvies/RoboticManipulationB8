function reachable = isReachable(target_pos, target_orientation)
% ISREACHABLE Quick check if a target position is within the robot's workspace
%
% Returns true if the target can be reached with valid joint angles.
%
% Usage:
%   reachable = isReachable([x, y, z])
%   reachable = isReachable([x, y, z], 'horizontal')
%
% Inputs:
%   target_pos         - [x, y, z] target position in mm
%   target_orientation - (Optional) 'horizontal', 'down', or pitch angle
%
% Output:
%   reachable - true if position is reachable within joint limits
%
% Example:
%   if isReachable([250, 0, 150])
%       moveToPosition(port_num, lib_name, [250, 0, 150]);
%   end
%
% Author: OpenManipulator-X IK Implementation
% Date: February 2026

    if nargin < 2
        target_orientation = 'horizontal';
    end
    
    [~, success, ~] = inverseKinematics(target_pos, target_orientation);
    reachable = success;
end
