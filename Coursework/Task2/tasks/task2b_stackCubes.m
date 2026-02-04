function task2b_stackCubes(port_num, lib_name, config)
% TASK2B_STACKCUBES Stack all cubes with red faces outward (16 points)
%
% Task 2.b: Stack all three cubes on the designated stacking holder with
% their red faces oriented outward (facing +X direction).
%
% Inputs:
%   port_num - Dynamixel port handle
%   lib_name - Dynamixel library name
%   config   - Configuration struct from task2Config()
%
% Scoring:
%   +2 points per cube in stack
%   +3 bonus per cube with red face correctly oriented (+X)
%   Total: 6 + 9 = 15 points possible (+1 if already in holder = 16)
%
% Note: This task builds on 2.a. Cubes should already be in holders.
%
% Author: OpenManipulator-X Task 2
% Date: February 2026

fprintf('========================================\n');
fprintf('  TASK 2.b: STACK CUBES (RED FACES OUT)\n');
fprintf('========================================\n\n');

%% Configuration
cube_size = config.cube.size;
stack_holder_idx = config.holder.stack_index;
stack_position = config.holder.positions(stack_holder_idx, :);
holder_positions = config.holder.positions;
cube_red_faces = config.cube.red_face;  % Current orientations
target_orientation = config.holder.red_face_target;  % Desired orientation (0 = +X)

fprintf('Stack position: Holder %d at [%.0f, %.0f, %.0f] mm\n', ...
    stack_holder_idx, stack_position);
fprintf('Target red face orientation: %d° (facing +X)\n', target_orientation);

%% Determine Stacking Order
% Pick up from holders 1, 2, 3 (excluding the stack holder)
% Stack in order: bottom, middle, top
pickup_holders = setdiff(1:size(holder_positions, 1), stack_holder_idx);
fprintf('\nPickup order: Holders %s\n', mat2str(pickup_holders));

%% Calculate Rotation Needed for Each Cube
% Original red face orientations: 0°=+X, 90°=+Y, 180°=-X, 270°=-Y
% We want all red faces at 0° (facing +X)
% Base rotation (joint 1) will need to compensate

fprintf('\nRed face orientations:\n');
for i = 1:length(cube_red_faces)
    rotation_needed = -cube_red_faces(i);  % Negative to bring to 0
    if rotation_needed < -180
        rotation_needed = rotation_needed + 360;
    end
    fprintf('  Cube %d: currently %d°, need %d° base rotation\n', ...
        i, cube_red_faces(i), rotation_needed);
end

%% Stack Each Cube
fprintf('\n--- Beginning stacking sequence ---\n');
success_count = 0;

for stack_level = 1:length(pickup_holders)
    holder_idx = pickup_holders(stack_level);
    cube_idx = holder_idx;  % Assuming cube i is in holder i after task 2.a
    
    fprintf('\n>>> Stacking Cube %d (level %d) <<<\n', cube_idx, stack_level);
    
    % Pickup position (cube is in holder)
    pickup_pos = holder_positions(holder_idx, :);
    pickup_pos(3) = cube_size;  % Top of cube sitting in holder
    
    % Stack position (accounts for cubes already in stack)
    stack_height = (stack_level - 1) * cube_size;  % Height of existing stack
    place_pos = stack_position;
    place_pos(3) = stack_height;  % Place on top of stack
    
    fprintf('Pickup: [%.0f, %.0f, %.0f] mm\n', pickup_pos);
    fprintf('Place:  [%.0f, %.0f, %.0f] mm (stack level %d)\n', ...
        place_pos, stack_level);
    
    % Calculate base rotation needed for red face orientation
    current_face = cube_red_faces(cube_idx);
    rotation_deg = mod(target_orientation - current_face, 360);
    if rotation_deg > 180
        rotation_deg = rotation_deg - 360;
    end
    
    fprintf('Red face rotation: %d° -> %d° (rotate %d°)\n', ...
        current_face, target_orientation, rotation_deg);
    
    try
        % Pick up cube with rotation
        pickAndPlaceWithRotation(port_num, lib_name, pickup_pos, place_pos, ...
            rotation_deg, config);
        success_count = success_count + 1;
        fprintf('Cube %d stacked at level %d!\n', cube_idx, stack_level);
    catch ME
        fprintf('ERROR stacking cube %d: %s\n', cube_idx, ME.message);
    end
    
    pause(0.5);
end

%% Summary
points_stack = success_count * 2;
points_orientation = success_count * 3;  % Assuming rotation worked
total_points = points_stack + points_orientation;

fprintf('\n========================================\n');
fprintf('  TASK 2.b COMPLETE\n');
fprintf('  Cubes stacked: %d / 3\n', success_count);
fprintf('  Stack points: %d\n', points_stack);
fprintf('  Orientation points: %d (assumed)\n', points_orientation);
fprintf('  Total: %d / 16 points\n', total_points);
fprintf('========================================\n\n');

end

%% Pick and Place with Rotation
function pickAndPlaceWithRotation(port_num, lib_name, pickup_pos, place_pos, rotation_deg, config)
% Modified pick and place that rotates the base joint during transit
% to change the cube's final orientation

approach_height = config.motion.approach_height;
cube_size = config.cube.size;
velocity = config.motion.velocity;

% Key positions
approach_pickup = pickup_pos + [0, 0, approach_height];
approach_place = place_pos + [0, 0, approach_height + cube_size];
place_final = place_pos + [0, 0, cube_size/2];

fprintf('Pick and place with %.0f° rotation...\n', rotation_deg);

%% Step 1: Open gripper
gripperControl(port_num, lib_name, 'open', config);
pause(0.3);

%% Step 2-4: Approach, descend, grip (same as standard)
moveToPosition(port_num, lib_name, approach_pickup, 'auto', velocity);
pause(0.3);
moveToPosition(port_num, lib_name, pickup_pos, 'auto', velocity);
pause(0.3);
gripperControl(port_num, lib_name, 'cube', config);
pause(0.5);

%% Step 5: Lift
moveToPosition(port_num, lib_name, approach_pickup, 'auto', velocity);
pause(0.3);

%% Step 6: Rotate base joint while at safe height
if abs(rotation_deg) > 1  % Only rotate if significant
    fprintf('Rotating base joint by %.0f°...\n', rotation_deg);
    % Get current joint angles
    % Apply rotation to joint 1
    rotateBaseJoint(port_num, lib_name, rotation_deg, velocity);
    pause(0.5);
end

%% Step 7-9: Move to place, descend, release (position in rotated frame)
% After base rotation, we need to recalculate the approach in the new frame
% For simplicity, we'll use the fact that rotating base doesn't change
% the robot's reach, just orientation

% Compute place position in rotated frame
theta = deg2rad(rotation_deg);
R = [cos(theta), -sin(theta), 0;
     sin(theta),  cos(theta), 0;
     0, 0, 1];
approach_place_rot = (R * approach_place')';
place_final_rot = (R * place_final')';

moveToPosition(port_num, lib_name, approach_place_rot, 'auto', velocity);
pause(0.3);
moveToPosition(port_num, lib_name, place_final_rot, 'auto', velocity);
pause(0.3);
gripperControl(port_num, lib_name, 'open', config);
pause(0.3);
moveToPosition(port_num, lib_name, approach_place_rot, 'auto', velocity);
pause(0.3);

end

function rotateBaseJoint(port_num, lib_name, rotation_deg, velocity)
% Rotate only the base joint by specified degrees
PROTOCOL_VERSION = 2.0;
DXL_ID_BASE = 11;
ADDR_GOAL_POSITION = 116;
ADDR_PRESENT_POSITION = 132;

% Read current position
current_encoder = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_BASE, ADDR_PRESENT_POSITION);

% Convert rotation to encoder units (4096 per 360°)
encoder_delta = round(rotation_deg * 4096 / 360);
new_encoder = current_encoder + encoder_delta;

% Safety check
if new_encoder < 1024 || new_encoder > 3072
    warning('Base rotation would exceed limits. Clamping.');
    new_encoder = max(1024, min(3072, new_encoder));
end

write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_BASE, ADDR_GOAL_POSITION, new_encoder);
pause(abs(rotation_deg) / 90);  % Wait proportional to rotation

end
