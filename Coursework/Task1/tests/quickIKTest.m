%% Quick IK Test
clear; clc;

fprintf('Quick IK Test with Auto Pitch\n\n');

% Single test target
target = [200, 0, 200];
fprintf('Target: [%.0f, %.0f, %.0f] mm\n\n', target);

% Test with auto pitch
[q, success, info] = inverseKinematicsAuto(target);

if success
    fprintf('SUCCESS!\n');
    fprintf('Joint angles: [%.1f, %.1f, %.1f, %.1f] deg\n', rad2deg(q));
    fprintf('Pitch used: %.1f deg\n', rad2deg(info.pitch_used));
    fprintf('FK error: %.4f mm\n', info.fk_error);
else
    fprintf('FAILED: %s\n', info.message);
end

% Also test round-trip
fprintf('\n--- Round-trip Test ---\n');
q_test = deg2rad([0, 30, 30, -30]);
[~, pos, ~] = forwardKinematics(q_test);
fprintf('Original q: [%.0f, %.0f, %.0f, %.0f] deg\n', rad2deg(q_test));
fprintf('FK position: [%.1f, %.1f, %.1f] mm\n', pos);

[q_ik, success2, info2] = inverseKinematicsAuto(pos');
if success2
    fprintf('IK q: [%.1f, %.1f, %.1f, %.1f] deg\n', rad2deg(q_ik));
    fprintf('FK error: %.4f mm\n', info2.fk_error);
else
    fprintf('IK failed: %s\n', info2.message);
end
