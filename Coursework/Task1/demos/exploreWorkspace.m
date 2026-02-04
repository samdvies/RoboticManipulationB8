%% Explore Reachable Workspace with Joint Limits
% Find positions that are actually reachable within ±90° joint limits
%
% Author: OpenManipulator-X IK Implementation
% Date: February 2026

clear all; close all; clc;

fprintf('=== Exploring Reachable Workspace ===\n\n');

%% Method 1: Sample joint space and compute FK
fprintf('Sampling joint space to find reachable positions...\n');

% Joint limits
q_min = -pi/2;
q_max = pi/2;
n_samples = 10;

q1_range = linspace(q_min, q_max, n_samples);
q2_range = linspace(q_min, q_max, n_samples);
q3_range = linspace(q_min, q_max, n_samples);
q4_range = linspace(q_min, q_max, 5);

% Collect valid positions
valid_positions = [];
valid_angles = [];

for q1 = q1_range
    for q2 = q2_range
        for q3 = q3_range
            for q4 = q4_range
                q = [q1, q2, q3, q4];
                [~, pos, ~] = forwardKinematics(q);
                valid_positions = [valid_positions; pos'];
                valid_angles = [valid_angles; q];
            end
        end
    end
end

fprintf('Found %d valid configurations\n', size(valid_positions, 1));

%% Workspace Statistics
fprintf('\n--- Workspace Statistics (within ±90° limits) ---\n');
fprintf('X range: [%.1f, %.1f] mm\n', min(valid_positions(:,1)), max(valid_positions(:,1)));
fprintf('Y range: [%.1f, %.1f] mm\n', min(valid_positions(:,2)), max(valid_positions(:,2)));
fprintf('Z range: [%.1f, %.1f] mm\n', min(valid_positions(:,3)), max(valid_positions(:,3)));

%% Test IK on positions we KNOW are reachable
fprintf('\n--- Testing IK on Known-Reachable Positions ---\n\n');

% Pick 10 random valid positions
rng(42);  % Reproducible
test_idx = randperm(size(valid_positions, 1), 10);

success_count = 0;
for i = 1:length(test_idx)
    idx = test_idx(i);
    target = valid_positions(idx, :);
    original_q = valid_angles(idx, :);
    
    % Try IK
    [q_ik, success, info] = inverseKinematics(target, 'horizontal');
    
    fprintf('Test %d: Target [%.1f, %.1f, %.1f] mm\n', i, target);
    fprintf('  Original q: [%.1f, %.1f, %.1f, %.1f] deg\n', rad2deg(original_q));
    
    if success
        fprintf('  IK q:       [%.1f, %.1f, %.1f, %.1f] deg\n', rad2deg(q_ik));
        fprintf('  FK Error:   %.3f mm - SUCCESS\n', info.fk_error);
        success_count = success_count + 1;
    else
        fprintf('  IK Failed:  %s\n', info.message);
        
        % Try without orientation constraint
        [q_ik2, success2, info2] = inverseKinematics(target, 0);  % pitch=0 explicit
        if ~success2
            % Try different pitch
            for pitch = linspace(-pi/4, pi/4, 5)
                [q_ik3, success3, info3] = inverseKinematics(target, pitch);
                if success3
                    fprintf('  Found solution with pitch=%.1f deg\n', rad2deg(pitch));
                    fprintf('  IK q:       [%.1f, %.1f, %.1f, %.1f] deg\n', rad2deg(q_ik3));
                    break;
                end
            end
        end
    end
    fprintf('\n');
end

fprintf('Success rate: %d/%d (%.1f%%)\n', success_count, length(test_idx), 100*success_count/length(test_idx));

%% Visualize reachable workspace
figure('Name', 'Reachable Workspace', 'Position', [100 100 1000 500], 'Color', 'w');

subplot(1, 2, 1);
scatter3(valid_positions(:,1), valid_positions(:,2), valid_positions(:,3), ...
    1, valid_positions(:,3), 'filled');
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
title('All Reachable Positions (±90° joints)');
axis equal; grid on; view(45, 30);
colorbar;

subplot(1, 2, 2);
% Side view (XZ plane)
scatter(valid_positions(:,1), valid_positions(:,3), 1, valid_positions(:,2), 'filled');
xlabel('X (mm)'); ylabel('Z (mm)');
title('Side View (XZ plane, color=Y)');
axis equal; grid on;
colorbar;

%% Find sample reachable targets for demos
fprintf('\n--- Sample Reachable Targets for Demos ---\n');
fprintf('(Positions near center of workspace)\n\n');

% Find positions near Y=0 (straight ahead)
center_idx = abs(valid_positions(:,2)) < 30;
center_positions = valid_positions(center_idx, :);

% Sort by X (distance from base)
[~, sort_idx] = sort(center_positions(:,1));
center_positions = center_positions(sort_idx, :);

% Pick 5 evenly spaced
sample_idx = round(linspace(1, size(center_positions, 1), 5));
fprintf('Sample targets for demos:\n');
for i = 1:length(sample_idx)
    pos = center_positions(sample_idx(i), :);
    [q, success, ~] = inverseKinematics(pos, 'horizontal');
    if success
        fprintf('  [%.0f, %.0f, %.0f] - Valid\n', pos);
    else
        % Find a pitch that works
        for pitch = linspace(-pi/3, pi/3, 10)
            [q, success, ~] = inverseKinematics(pos, pitch);
            if success
                fprintf('  [%.0f, %.0f, %.0f] - Valid with pitch=%.0f deg\n', pos, rad2deg(pitch));
                break;
            end
        end
        if ~success
            fprintf('  [%.0f, %.0f, %.0f] - Needs different orientation\n', pos);
        end
    end
end
