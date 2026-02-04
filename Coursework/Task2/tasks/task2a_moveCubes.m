function task2a_moveCubes(port_num, lib_name, config)
% TASK2A_MOVECUBES Move three cubes to empty cube holders (9 points)
%
% Task 2.a: Move each cube from its starting position to an empty V-shaped
% holder. Uses optimal matching to minimize total travel distance.
%
% Inputs:
%   port_num - Dynamixel port handle
%   lib_name - Dynamixel library name
%   config   - Configuration struct from task2Config()
%
% Scoring:
%   +3 points per cube successfully placed in a holder
%   Total: 9 points possible
%
% Author: OpenManipulator-X Task 2
% Date: February 2026

fprintf('========================================\n');
fprintf('  TASK 2.a: MOVE CUBES TO HOLDERS\n');
fprintf('========================================\n\n');

%% Get Positions
cube_positions = config.cube.positions;
holder_positions = config.holder.positions;
cube_size = config.cube.size;

fprintf('Cube positions:\n');
for i = 1:size(cube_positions, 1)
    fprintf('  Cube %d: [%.0f, %.0f, %.0f] mm\n', i, cube_positions(i,:));
end
fprintf('\nHolder positions:\n');
for i = 1:size(holder_positions, 1)
    fprintf('  Holder %d: [%.0f, %.0f, %.0f] mm\n', i, holder_positions(i,:));
end

%% Find Optimal Assignment
fprintf('\nCalculating optimal cube-to-holder assignment...\n');
[assignment, total_dist] = optimalMatching(cube_positions, holder_positions);

%% Execute Pick and Place for Each Cube
fprintf('\n--- Beginning cube transfers ---\n');
success_count = 0;

for i = 1:size(cube_positions, 1)
    holder_idx = assignment(i);
    
    if holder_idx == 0
        fprintf('\nCube %d: SKIPPED (no holder assigned)\n', i);
        continue;
    end
    
    fprintf('\n>>> Moving Cube %d to Holder %d <<<\n', i, holder_idx);
    
    % Pickup position is at top of cube
    pickup_pos = cube_positions(i, :);
    pickup_pos(3) = cube_size;  % Top of cube
    
    % Place position is at holder base
    place_pos = holder_positions(holder_idx, :);
    
    try
        pickAndPlace(port_num, lib_name, pickup_pos, place_pos, config);
        success_count = success_count + 1;
        fprintf('Cube %d placed successfully!\n', i);
    catch ME
        fprintf('ERROR placing cube %d: %s\n', i, ME.message);
    end
    
    % Brief pause between operations
    pause(0.5);
end

%% Summary
fprintf('\n========================================\n');
fprintf('  TASK 2.a COMPLETE\n');
fprintf('  Cubes placed: %d / %d\n', success_count, size(cube_positions, 1));
fprintf('  Points earned: %d / 9\n', success_count * 3);
fprintf('========================================\n\n');

end
