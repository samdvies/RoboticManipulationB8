addpath('Task1/core');

fprintf('=== Verifying Inverse Kinematics Fixes ===\n');

%% Test Case 1: The User's Problematic Point
target_user = [300, 150, 0];
fprintf('\nTest 1: Reaching User Point [300, 150, 0]\n');
[q, success, info] = inverseKinematics(target_user, 'auto');

if success
    fprintf('SUCCESS: Point reached!\n');
    fprintf('  Configuration: %s\n', info.config);
    fprintf('  Joints: [%.1f, %.1f, %.1f, %.1f] deg\n', rad2deg(q));
    fprintf('  FK Error: %.2f mm\n', info.fk_error);
    if contains(info.message, 'Auto-Switched')
        fprintf('  Note: Auto-Configuration correctly switched to Elbow-Up.\n');
    end
else
    fprintf('FAILURE: %s\n', info.message);
end

%% Test Case 2: Extended Reach (High Y)
target_y = [0, 250, 100]; % High Y value
fprintf('\nTest 2: Reaching High Y (Base Rotation) [0, 250, 100]\n');
[q, success, info] = inverseKinematics(target_y, 'horizontal');

if success
    fprintf('SUCCESS: Point reached!\n');
    fprintf('  Base Angle: %.1f deg (Limit is +/- 90)\n', rad2deg(q(1)));
else
    fprintf('FAILURE: %s\n', info.message);
end

%% Test Case 3: Ground Collision Safety
target_low = [200, 0, -50]; % Below ground
fprintf('\nTest 3: Safety Check - Below Ground [200, 0, -50]\n');
[q, success, info] = inverseKinematics(target_low, 'auto');

if ~success && (info.ground_collision || contains(info.message, 'unreachable') || contains(info.message, 'limit'))
    fprintf('PASS: Correctly rejected unsafe point. Message: %s\n', info.message);
else
    fprintf('WARNING: Unexpected result. Success=%d, Message=%s\n', success, info.message);
end

%% Test Case 4: Random Workspace Sampling
fprintf('\nTest 4: Random Workspace Sampling (100 points)...\n');
valid_count = 0;
attempts = 100;

for i = 1:attempts
    % Random point in front hemisphere
    r = 150 + rand()*200; % Reach 150-350mm
    th = (rand() - 0.5) * pi; % -90 to +90 deg
    z = 10 + rand()*250; % Height 10-260mm
    
    x = r * cos(th);
    y = r * sin(th);
    
    [q, success, info] = inverseKinematics([x, y, z], 'auto');
    if success
        valid_count = valid_count + 1;
    end
end

fprintf('Success Rate: %d/%d (%.1f%%)\n', valid_count, attempts, (valid_count/attempts)*100);
fprintf('Note: <100%% is expected as not all random points are reachable.\n');

fprintf('\n=== Verification Complete ===\n');
