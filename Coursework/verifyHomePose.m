function verifyHomePose()
% VERIFYHOMEPOSE Moves robot to "Zero" (Home) pose for visual check
%
% This sends all motors to Encoder 2048 (0 degrees).
% The robot should look like an "L" shape:
%   - Arm sticking straight up
%   - Elbow bent 90 degrees? No...
%   - Wait, standard DH home (all zeros):
%       q1=0: Facing forward
%       q2=0: Vertical? Or Horizontal?
%           My code says: T_1_2 = buildDH(..., q2 + beta)
%           If q2=0, angle is beta (10deg).
%           For OpenManipulator, 0 (2048) usually means "L-shape" (Up and Forward).
%
% Usage: c = verifyHomePose();

    clc;
    setupPath();
    [port_num, lib_name, cleanup_obj] = robotSafeInit('COM4');
    
    fprintf('Moving to Encoder 2048 (Calibration Check)...\n');
    fprintf('The robot should form a specific shape (L-shape).\n');
    
    % Move all to 2048
    write4ByteTxRx(port_num, 2.0, 11, 116, 2048);
    write4ByteTxRx(port_num, 2.0, 12, 116, 2048);
    write4ByteTxRx(port_num, 2.0, 13, 116, 2048);
    write4ByteTxRx(port_num, 2.0, 14, 116, 2048);
    
    fprintf('Done. Please check if joints look centered.\n');
end
