function config = task2Config()
% TASK2CONFIG - Configuration for Task 2 Cube Manipulation
%
% =====================================================================
%   EDIT THIS FILE WITH YOUR MEASUREMENTS ON DEMO DAY
% =====================================================================
%
% Grid system: 25mm per unit. Count holes from robot base center.
% Coordinate system:
%   +X = forward (away from robot)
%   +Y = left
%   +Z = up
%   Origin (0,0,0) = robot base center at board surface
%
% Author: OpenManipulator-X Task 2
% Date: February 2026

GRID = 25;  % mm per grid unit (DO NOT CHANGE)

%% =====================================================================
%  QUICK EDIT SECTION - UPDATE THESE ON DEMO DAY!
%  =====================================================================

%% RED FACE ORIENTATIONS - UPDATE BEFORE EACH RUN!
% Direction each cube's red face is currently pointing:
%   0   = +X (toward front of arena)
%   90  = +Y (toward left)
%   180 = -X (toward back/robot)
%   270 = -Y (toward right)
%
% Visual reference:
%        +Y (left)
%           |
%   -X -----+------ +X (front)
%  (robot)  |
%        -Y (right)

config.cube.red_face = [
    0;      % Cube 1 red face direction (degrees) <-- UPDATE!
    90;     % Cube 2 red face direction (degrees) <-- UPDATE!
    180;    % Cube 3 red face direction (degrees) <-- UPDATE!
];

%% CUBE POSITIONS [grid_x, grid_y, grid_z]
% Count grid holes from robot base center
% Z = 1 means cube TOP surface is 25mm above board
config.cube.grid = [
%   X    Y    Z    <- Grid units (multiply by 25 for mm)
    6,   4,   1;   % Cube 1 (front-left area)      <-- MEASURE!
    8,   0,   1;   % Cube 2 (front-center)         <-- MEASURE!
    5,  -4,   1;   % Cube 3 (under bridge)         <-- MEASURE!
];

% Which cube is covered by the bridge? (1, 2, or 3)
config.cube.covered_index = 3;

%% HOLDER POSITIONS [grid_x, grid_y, grid_z]
% Z = surface where cube bottom will rest
config.holder.grid = [
%   X    Y    Z    <- Grid units
   10,   4,   0;   % Holder A (white, front-left)  <-- MEASURE!
   11,   0,   0;   % Holder B (white, center)      <-- MEASURE!
   10,  -4,   0;   % Holder C (red, front-right)   <-- MEASURE!
];

% Which holder to use for stacking in Task 2.b
config.holder.stack_index = 2;  % Stack on Holder B

% Target direction for red faces when stacked (degrees)
config.holder.red_face_target = 0;  % Red faces should point +X (front)

%% GATE POSITIONS [grid_x, grid_y, grid_z]
% Gate CENTER position
config.gates.grid = [
%   X    Y    Z    <- Grid units
    7,   6,   4;   % Gate 1 (left side)            <-- MEASURE!
    7,  -6,   4;   % Gate 2 (right side)           <-- MEASURE!
];

% Tool tip must stay BELOW this height (grid units)
config.gates.height_limit_grid = 5;  % 125mm                <-- MEASURE!

% Order to navigate gates
config.gates.sequence = [1, 2];

%% BRIDGE POSITION [grid_x, grid_y, grid_z]
config.bridge.grid = [5, -4, 3];  % Bridge center          <-- MEASURE!
config.bridge.clearance_grid = 5; % Safe height to clear (grid units)

%% =====================================================================
%  FIXED SETTINGS - Generally don't need to change
%  =====================================================================

%% AUTO-CONVERT GRID TO MM
config.grid_size = GRID;
config.cube.positions = config.cube.grid * GRID;
config.holder.positions = config.holder.grid * GRID;
config.gates.positions = config.gates.grid * GRID;
config.gates.height_limit = config.gates.height_limit_grid * GRID;
config.bridge.position = config.bridge.grid * GRID;
config.bridge.clearance_height = config.bridge.clearance_grid * GRID;

%% CUBE PROPERTIES
config.cube.size = 25;          % Cube side length (mm) = 1 grid unit
config.cube.grip_depth = 12;    % How deep to grip from top (mm)

%% GRIPPER SETTINGS (OpenManipulator-X)
config.gripper.id = 15;         % Dynamixel ID for gripper
config.gripper.open = 600;      % Encoder value for fully open
config.gripper.closed = 200;    % Encoder value for closed
config.gripper.cube_grip = 380; % Encoder value for gripping cube (TUNE THIS!)
config.gripper.velocity = 100;  % Gripper movement speed

%% MOTION SETTINGS
config.motion.velocity = 60;           % Arm movement speed (higher = faster)
config.motion.approach_height = 50;    % Height above objects for approach (mm)
config.motion.retreat_height = 60;     % Height to retreat after place (mm)
config.motion.grip_pause = 0.4;        % Pause after gripping (seconds)
config.motion.place_pause = 0.3;       % Pause after placing (seconds)
config.motion.trajectory_points = 30;  % Points for smooth trajectory

%% SAFETY SETTINGS
config.safety.min_z = 20;              % Never go below this Z (mm)
config.safety.confirm_moves = false;   % Set true to confirm each move

end
