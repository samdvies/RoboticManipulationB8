function pickAndPlace(port_num, lib_name, pickup_pos, place_pos, config)
% PICKANDPLACE Execute a complete pick and place operation
%
% Picks up an object from pickup_pos and places it at place_pos.
% Includes safe approach heights and gripper control.
%
% Inputs:
%   port_num   - Dynamixel port handle
%   lib_name   - Dynamixel library name
%   pickup_pos - [x, y, z] pickup position (top of object)
%   place_pos  - [x, y, z] place position (where bottom of object goes)
%   config     - Configuration struct from task2Config()
%
% Author: OpenManipulator-X Task 2
% Date: February 2026

%% Extract Parameters
approach_height = config.motion.approach_height;
cube_size = config.cube.size;
velocity = config.motion.velocity;

%% Calculate Waypoints
approach_pickup = pickup_pos + [0, 0, approach_height];
approach_place = place_pos + [0, 0, approach_height + cube_size];
place_final = place_pos + [0, 0, cube_size/2];  % Place with cube center

fprintf('=== Pick and Place ===\n');
fprintf('Pickup: [%.0f, %.0f, %.0f] mm\n', pickup_pos);
fprintf('Place:  [%.0f, %.0f, %.0f] mm\n', place_pos);

%% Step 1: Open Gripper
fprintf('Step 1: Opening gripper...\n');
gripperControl(port_num, lib_name, 'open', config);
pause(0.3);

%% Step 2: Move to Approach Position Above Pickup
fprintf('Step 2: Approaching pickup...\n');
moveToPosition(port_num, lib_name, approach_pickup, 'auto', velocity);
pause(0.5);

%% Step 3: Descend to Pickup
fprintf('Step 3: Descending to pickup...\n');
moveToPosition(port_num, lib_name, pickup_pos, 'auto', velocity);
pause(0.3);

%% Step 4: Close Gripper on Object
fprintf('Step 4: Gripping object...\n');
gripperControl(port_num, lib_name, 'cube', config);
pause(0.5);

%% Step 5: Lift Object
fprintf('Step 5: Lifting object...\n');
moveToPosition(port_num, lib_name, approach_pickup, 'auto', velocity);
pause(0.3);

%% Step 6: Move to Approach Position Above Place
fprintf('Step 6: Moving to place position...\n');
moveToPosition(port_num, lib_name, approach_place, 'auto', velocity);
pause(0.5);

%% Step 7: Descend to Place Position
fprintf('Step 7: Descending to place...\n');
moveToPosition(port_num, lib_name, place_final, 'auto', velocity);
pause(0.3);

%% Step 8: Release Object
fprintf('Step 8: Releasing object...\n');
gripperControl(port_num, lib_name, 'open', config);
pause(0.3);

%% Step 9: Lift Away
fprintf('Step 9: Retracting...\n');
moveToPosition(port_num, lib_name, approach_place, 'auto', velocity);
pause(0.3);

fprintf('=== Pick and Place Complete ===\n\n');

end
