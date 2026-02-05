addpath('Task1/core');
target = [300, 150, 0];
fprintf('Testing reachability for [%.1f, %.1f, %.1f]\n', target);

modes = {'horizontal', 'down'};
for i = 1:length(modes)
    mode = modes{i};
    fprintf('\nMode: %s\n', mode);
    [q, success, info] = inverseKinematics(target, mode);
    if success
        fprintf('SUCCESS: q = [%.2f, %.2f, %.2f, %.2f] rad\n', q);
        disp(info);
    else
        fprintf('FAILED: %s\n', info.message);
        disp(info);
    end
end
