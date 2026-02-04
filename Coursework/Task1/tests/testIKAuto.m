%% Simple IK Test with Auto Pitch
% Tests the inverse kinematics with automatic pitch search
%
% Author: OpenManipulator-X IK Implementation
% Date: February 2026

clear all; close all; clc;

fprintf('=== Testing IK with Auto Pitch ===\n\n');

%% Test Targets - reasonable positions in workspace
targets = [
    200,   0, 200;    % Straight ahead, mid-height
    250,   0, 150;    % Forward reach
    150, 100, 200;    % To the side
    150,-100, 200;    % Other side
    200,   0, 300;    % Higher up
    150,   0, 100;    % Lower
    180,  50, 250;    % Mixed
];

fprintf('Testing %d target positions:\n\n', size(targets, 1));

success_count = 0;
for i = 1:size(targets, 1)
    target = targets(i, :);
    
    % Use auto pitch search
    [q, success, info] = inverseKinematicsAuto(target);
    
    fprintf('Target %d: [%.0f, %.0f, %.0f] mm\n', i, target);
    
    if success
        fprintf('  SUCCESS!\n');
        fprintf('  Joint angles: [%.1f, %.1f, %.1f, %.1f] deg\n', rad2deg(q));
        fprintf('  Pitch used: %.1f deg\n', rad2deg(info.pitch_used));
        fprintf('  FK error: %.3f mm\n', info.fk_error);
        success_count = success_count + 1;
    else
        fprintf('  FAILED: %s\n', info.message);
    end
    fprintf('\n');
end

fprintf('=== Summary: %d/%d targets reached (%.0f%%) ===\n', ...
    success_count, size(targets, 1), 100*success_count/size(targets, 1));

%% Visual Demo of a Successful Solution
fprintf('\nVisualizing last successful solution...\n');

% Find first success
for i = 1:size(targets, 1)
    target = targets(i, :);
    [q, success, info] = inverseKinematicsAuto(target);
    if success
        figure('Name', 'IK Solution Demo', 'Position', [100 100 800 600], 'Color', 'w');
        plotRobotArm(q);
        
        % Add target marker
        hold on;
        plot3(target(1), target(2), target(3), 'go', 'MarkerSize', 15, 'LineWidth', 3, 'MarkerFaceColor', 'g');
        
        % Add FK position marker
        fk_pos = info.fk_position;
        plot3(fk_pos(1), fk_pos(2), fk_pos(3), 'rx', 'MarkerSize', 12, 'LineWidth', 2);
        
        legend('', '', '', '', '', '', 'Target', 'FK Result', 'Location', 'best');
        title(sprintf('IK Solution: Target [%.0f, %.0f, %.0f] mm, Pitch=%.1f°', ...
            target, rad2deg(info.pitch_used)));
        hold off;
        
        fprintf('Visualization showing target (green o) and FK result (red x)\n');
        break;
    end
end

%% Round-trip FK→IK→FK Test
fprintf('\n=== Round-Trip Verification Test ===\n');

% Use angles we know are valid (within ±90°)
test_angles = [
    0,    30,   45,  -30;
    30,   20,   30,  -20;
   -45,   10,   60,  -40;
    60,  -10,   30,   10;
];

fprintf('\nTesting FK→IK→FK round-trip:\n\n');

for i = 1:size(test_angles, 1)
    q_orig = deg2rad(test_angles(i, :));
    
    % Forward kinematics
    [~, pos, ~] = forwardKinematics(q_orig);
    target = pos';
    
    % Inverse kinematics
    [q_ik, success, info] = inverseKinematicsAuto(target);
    
    fprintf('Test %d: q_orig = [%.0f, %.0f, %.0f, %.0f] deg\n', i, test_angles(i, :));
    fprintf('  FK pos: [%.1f, %.1f, %.1f] mm\n', target);
    
    if success
        fprintf('  q_ik:   [%.1f, %.1f, %.1f, %.1f] deg\n', rad2deg(q_ik));
        fprintf('  FK error: %.4f mm - PASS\n', info.fk_error);
    else
        fprintf('  FAILED: %s\n', info.message);
    end
    fprintf('\n');
end
