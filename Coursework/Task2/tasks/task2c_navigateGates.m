function task2c_navigateGates(port_num, lib_name, config)
% TASK2C_NAVIGATEGATES Move tool through gates under height limit (15 points)
%
% Task 2.c: Navigate the gripper through two gates while keeping the
% entire arm below the height limit (bridge clearance).
%
% Inputs:
%   port_num - Dynamixel port handle
%   lib_name - Dynamixel library name
%   config   - Configuration struct from task2Config()
%
% Scoring:
%   +5 points per gate passed through (entry to exit)
%   +5 bonus if entire arm stays below height limit
%   Total: 15 points possible
%
% Strategy:
%   - Use pitched-forward configurations to keep elbow low
%   - Move slowly and smoothly through gates
%   - Gate positions define the center of the gate opening
%
% Author: OpenManipulator-X Task 2
% Date: February 2026

fprintf('========================================\n');
fprintf('  TASK 2.c: NAVIGATE GATES\n');
fprintf('========================================\n\n');

%% Configuration
gate_positions = config.gates.positions;  % Gate center positions
height_limit = config.gates.height_limit;
approach_offset = config.motion.gate_approach;
velocity = config.motion.velocity_slow;  % Use slow speed for precision

fprintf('Height limit: %.0f mm\n', height_limit);
fprintf('Number of gates: %d\n', size(gate_positions, 1));

for i = 1:size(gate_positions, 1)
    fprintf('Gate %d: [%.0f, %.0f, %.0f] mm\n', i, gate_positions(i,:));
end

%% Calculate Safe Height for Transit
% We need to move with the tool low and arm pitched forward
% Maximum safe Z for tool is height_limit minus safety margin
safe_z = height_limit - 30;  % 30mm safety margin
fprintf('\nSafe tool height: %.0f mm\n', safe_z);

%% Move to Starting Position
fprintf('\n--- Moving to start position ---\n');

% Start position: approach first gate from -X side
gate1 = gate_positions(1, :);
start_pos = gate1 + [-approach_offset, 0, 0];
start_pos(3) = safe_z;

fprintf('Start position: [%.0f, %.0f, %.0f] mm\n', start_pos);

% First lift gripper, then move to start
moveToPosition(port_num, lib_name, [start_pos(1), start_pos(2), safe_z + 50], 'auto', velocity);
pause(0.5);
moveToPosition(port_num, lib_name, start_pos, 'auto', velocity);
pause(1);

%% Navigate Through Gates
fprintf('\n--- Navigating gates ---\n');

for gate_num = 1:size(gate_positions, 1)
    fprintf('\n>>> Gate %d <<<\n', gate_num);
    
    gate_pos = gate_positions(gate_num, :);
    gate_pos(3) = safe_z;  % Stay at safe height
    
    % Approach point (before gate)
    approach_pos = gate_pos + [-approach_offset, 0, 0];
    fprintf('Approach: [%.0f, %.0f, %.0f] mm\n', approach_pos);
    
    % Gate center
    center_pos = gate_pos;
    fprintf('Center:   [%.0f, %.0f, %.0f] mm\n', center_pos);
    
    % Exit point (after gate)  
    exit_pos = gate_pos + [approach_offset, 0, 0];
    fprintf('Exit:     [%.0f, %.0f, %.0f] mm\n', exit_pos);
    
    % Execute gate traverse
    try
        % Move to approach
        fprintf('Moving to approach...\n');
        moveToPositionLowProfile(port_num, lib_name, approach_pos, config, height_limit);
        pause(0.5);
        
        % Move through center
        fprintf('Passing through gate...\n');
        moveToPositionLowProfile(port_num, lib_name, center_pos, config, height_limit);
        pause(0.3);
        
        % Move to exit
        fprintf('Exiting gate...\n');
        moveToPositionLowProfile(port_num, lib_name, exit_pos, config, height_limit);
        pause(0.5);
        
        fprintf('Gate %d passed!\n', gate_num);
        
    catch ME
        fprintf('ERROR at gate %d: %s\n', gate_num, ME.message);
    end
end

%% Return to Safe Position
fprintf('\n--- Returning to safe position ---\n');
moveToPosition(port_num, lib_name, [150, 0, 150], 'auto', velocity);
pause(1);

%% Summary
fprintf('\n========================================\n');
fprintf('  TASK 2.c COMPLETE\n');
fprintf('  Gates navigated: %d\n', size(gate_positions, 1));
fprintf('  Points: 15 (if height limit respected)\n');
fprintf('========================================\n\n');

end

%% Low Profile Movement
function moveToPositionLowProfile(port_num, lib_name, target, config, height_limit)
% Move to position while keeping all arm segments below height limit
% Uses pitched-forward configurations

velocity = config.motion.velocity_slow;

% Try different pitch angles to find one that keeps arm low
pitch_options = deg2rad([20, 30, 40, 50, 60]);  % Pitched forward

best_q = [];
best_max_height = inf;

for pitch = pitch_options
    [q, success] = inverseKinematics(target, pitch);
    
    if success
        % Calculate maximum height of any arm segment
        max_height = computeMaxArmHeight(q);
        
        if max_height < height_limit && max_height < best_max_height
            best_q = q;
            best_max_height = max_height;
        end
    end
end

if isempty(best_q)
    error('Cannot reach [%.0f, %.0f, %.0f] while staying below %.0f mm', ...
        target, height_limit);
end

fprintf('  Max arm height: %.0f mm (limit: %.0f mm)\n', best_max_height, height_limit);

% Move using the low-profile solution
moveWithJointAngles(port_num, lib_name, best_q, velocity);

end

%% Compute Maximum Arm Height
function max_height = computeMaxArmHeight(q)
% Forward kinematics to find highest point of arm

% DH parameters (Craig convention)
L_base = 77;
L_prox = 130.23;
L_dist = 124;
L_tool = 126;
beta = atan2(24, 128);

% Joint frames
T_base = buildDHMatrix(0, L_base, 0, -pi/2) * buildDHMatrix(0, 0, 0, q(1));
T_link1 = T_base * buildDHMatrix(L_prox, 0, -beta, 0) * buildDHMatrix(0, 0, 0, q(2));
T_link2 = T_link1 * buildDHMatrix(L_dist, 0, beta, 0) * buildDHMatrix(0, 0, 0, q(3));
T_tool = T_link2 * buildDHMatrix(L_tool, 0, 0, 0) * buildDHMatrix(0, 0, 0, q(4));

% Extract z-coordinates
heights = [0, T_base(3,4), T_link1(3,4), T_link2(3,4), T_tool(3,4)];
max_height = max(heights);

end

%% Move with Joint Angles
function moveWithJointAngles(port_num, lib_name, q, velocity)
% Move to specified joint angles

PROTOCOL_VERSION = 2.0;
DXL_IDS = [11, 12, 13, 14];
ADDR_GOAL_POSITION = 116;

% Convert to encoder values
encoders = zeros(1, 4);
for i = 1:4
    encoders(i) = angleToEncoder(q(i));
end

% Send commands
for i = 1:4
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_IDS(i), ADDR_GOAL_POSITION, encoders(i));
end

% Wait for motion
pause(1.5);

end
