%% OpenManipulator-X Inverse Kinematics Test
% Test script to validate IK implementation without hardware
%
% Author: OpenManipulator-X IK Implementation
% Date: February 2026

clear all; close all; clc;

%% Test 1: Basic IK at Various Positions
fprintf('=== Inverse Kinematics Test Suite ===\n\n');

% Test positions within ±90° joint limit workspace (x, y, z in mm)
% These are more conservative targets that should be reachable
test_positions = [
    280,   0, 200;    % Forward, medium height
    250,  50, 180;    % Slightly right
    250, -50, 180;    % Slightly left
    220,   0, 250;    % Higher position
    300,   0, 150;    % Forward reach
    200, 100, 200;    % Diagonal right
];

fprintf('--- Test 1: IK Solution Validation ---\n\n');

for i = 1:size(test_positions, 1)
    target = test_positions(i, :);
    
    fprintf('Target %d: [%.0f, %.0f, %.0f] mm\n', i, target);
    
    [q, success, info] = inverseKinematics(target, 'horizontal');
    
    if success
        fprintf('  SUCCESS: [%.1f, %.1f, %.1f, %.1f] deg\n', rad2deg(q));
        fprintf('  FK Error: %.3f mm\n', info.fk_error);
    else
        fprintf('  FAILED: %s\n', info.message);
    end
    fprintf('\n');
end

%% Test 2: Round-Trip FK -> IK -> FK
fprintf('--- Test 2: Round-Trip Validation (FK -> IK -> FK) ---\n\n');

% Start with known joint angles
test_angles_deg = [
    30, -20, 45, -25;
    -45, 10, 30, 15;
    0, -30, 60, -30;
    60, 0, 45, -45;
];

for i = 1:size(test_angles_deg, 1)
    q_original = deg2rad(test_angles_deg(i, :));
    
    % Compute FK to get position
    [~, pos, ~] = forwardKinematics(q_original);
    
    % Compute IK from position
    [q_ik, success, info] = inverseKinematics(pos', 'horizontal');
    
    % Compute FK from IK solution
    [~, pos_verify, ~] = forwardKinematics(q_ik);
    
    fprintf('Test %d:\n', i);
    fprintf('  Original angles: [%.1f, %.1f, %.1f, %.1f] deg\n', test_angles_deg(i,:));
    fprintf('  FK position:     [%.2f, %.2f, %.2f] mm\n', pos);
    fprintf('  IK angles:       [%.1f, %.1f, %.1f, %.1f] deg\n', rad2deg(q_ik));
    fprintf('  Position error:  %.4f mm\n', norm(pos - pos_verify));
    
    if norm(pos - pos_verify) < 0.1
        fprintf('  PASS\n');
    else
        fprintf('  FAIL - Large position error!\n');
    end
    fprintf('\n');
end

%% Test 3: Workspace Boundary Test
fprintf('--- Test 3: Workspace Boundary Exploration ---\n\n');

% Test points at workspace limits
boundary_tests = {
    400,   0, 100, 'Max reach (should fail)';
    380,   0, 100, 'Near max reach';
     50,   0, 100, 'Near min reach';
    300,   0, 400, 'Very high (should fail)';
    300,   0, -50, 'Below base (should fail)';
};

for i = 1:size(boundary_tests, 1)
    target = [boundary_tests{i, 1}, boundary_tests{i, 2}, boundary_tests{i, 3}];
    label = boundary_tests{i, 4};
    
    [~, success, info] = inverseKinematics(target, 'horizontal');
    
    fprintf('%s: [%.0f, %.0f, %.0f] mm\n', label, target);
    fprintf('  Reachable: %s\n', string(info.reachable));
    fprintf('  Within limits: %s\n', string(info.within_limits));
    if success
        fprintf('  Valid solution found\n');
    else
        fprintf('  %s\n', info.message);
    end
    fprintf('\n');
end

%% Test 4: Elbow Configurations
fprintf('--- Test 4: Elbow-Up vs Elbow-Down ---\n\n');

target = [220, 0, 220];  % Position more likely within workspace

[q_down, success_down, info_down] = inverseKinematics(target, 'horizontal', 'elbow-down');
[q_up, success_up, info_up] = inverseKinematics(target, 'horizontal', 'elbow-up');

fprintf('Target: [%.0f, %.0f, %.0f] mm\n\n', target);

fprintf('Elbow-Down:\n');
fprintf('  Angles: [%.1f, %.1f, %.1f, %.1f] deg\n', rad2deg(q_down));
fprintf('  Within limits: %s\n', string(info_down.within_limits));
fprintf('  Success: %s\n', string(success_down));

fprintf('\nElbow-Up:\n');
fprintf('  Angles: [%.1f, %.1f, %.1f, %.1f] deg\n', rad2deg(q_up));
fprintf('  Within limits: %s\n', string(info_up.within_limits));
fprintf('  Success: %s\n', string(success_up));

%% Test 5: Visualization
fprintf('\n--- Test 5: Visual Comparison ---\n');

figure('Name', 'IK Test - Elbow Configurations', 'Position', [100 100 1200 500], 'Color', 'w');

subplot(1, 2, 1);
plotRobotArm(q_down, gca, true, 25);
hold on;
plot3(target(1), target(2), target(3), 'r*', 'MarkerSize', 15, 'LineWidth', 2);
title(sprintf('Elbow-Down\nq = [%.1f, %.1f, %.1f, %.1f] deg', rad2deg(q_down)));

subplot(1, 2, 2);
if success_up
    plotRobotArm(q_up, gca, true, 25);
    hold on;
    plot3(target(1), target(2), target(3), 'r*', 'MarkerSize', 15, 'LineWidth', 2);
    title(sprintf('Elbow-Up\nq = [%.1f, %.1f, %.1f, %.1f] deg', rad2deg(q_up)));
else
    title('Elbow-Up: No valid solution');
end

%% Test 6: Single Target Visualization with Robot
fprintf('\n--- Test 6: Interactive Target Test ---\n');

figure('Name', 'IK Single Target Test', 'Position', [100 100 800 600], 'Color', 'w');
ax = gca;

target = [260, 30, 200];  % A reachable target
[q, success, info] = inverseKinematics(target, 'horizontal');

if success
    plotRobotArm(q, ax, true, 25);
    hold(ax, 'on');
    plot3(ax, target(1), target(2), target(3), 'r*', 'MarkerSize', 20, 'LineWidth', 3);
    
    % Draw line from end-effector to target (should overlap)
    [~, ee_pos, ~] = forwardKinematics(q);
    plot3(ax, [ee_pos(1), target(1)], [ee_pos(2), target(2)], [ee_pos(3), target(3)], ...
        'g--', 'LineWidth', 2);
    
    title(sprintf('Target: [%.0f, %.0f, %.0f] mm | Error: %.3f mm', target, info.fk_error));
    fprintf('Target reached with error: %.3f mm\n', info.fk_error);
else
    fprintf('Could not reach target: %s\n', info.message);
end

fprintf('\n=== All Tests Complete ===\n');
