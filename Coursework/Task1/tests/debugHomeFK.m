function debugHomeFK()
% DEBUGHOMEFK Calculates FK for Home pose (all 2048) and prints result
%
% Usage: debugHomeFK()

    addpath('Common');
    
    % Home Encoder = 2048
    % Angle Conversion for 2048 -> 0 rad
    q_home = [0, 0, 0, 0];
    
    [~, pos, ~] = forwardKinematics(q_home);
    
    fprintf('=== FK for Home Configuration (All Angles = 0) ===\n');
    fprintf('Position: X=%.2f, Y=%.2f, Z=%.2f mm\n', pos(1), pos(2), pos(3));
    fprintf('------------------------------------------------\n');
    
    % Also check L-Shape (if 0,0,0,0 is L-shape)
    % According to FK:
    % q2=0 (Shoulder vertical?)
    % q3=0 (Elbow straight?)
    
    % Let's check what 2048 means in angleConversion
    % angleConversion('enc2rad', 2048) -> 0
    
end
