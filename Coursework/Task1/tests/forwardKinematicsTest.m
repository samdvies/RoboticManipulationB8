%% Full 4-DOF OpenManipulator-X: Direct Call Method
clc; clear all; close all;

% --- 1. USER INPUT: SET ANGLES HERE ---
target_angles_deg = [180, -20, 45, 10]; % [Base, Shoulder, Elbow, Wrist]

% --- 2. SETUP & LIBRARY (Your Working Version) ---
lib_name = '';
if strcmp(computer, 'PCWIN64'), lib_name = 'dxl_x64_c';
elseif strcmp(computer, 'GLNXA64'), lib_name = 'libdxl_x64_c';
end

if ~libisloaded(lib_name)
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h');
end

% Settings
DEVICENAME = 'COM3'; % Ensure this matches your PC!
BAUDRATE = 1000000;
DXL_IDS = [11, 12, 13, 14]; % IDs for the full arm
PROTOCOL_VERSION = 2.0;

% Initialize Port
port_num = calllib(lib_name, 'portHandler', DEVICENAME);
calllib(lib_name, 'packetHandler');

if (calllib(lib_name, 'openPort', port_num))
    calllib(lib_name, 'setBaudRate', port_num, BAUDRATE);
    fprintf('Port Open Successfully\n');
else
    error('Failed to open port. Check COM port number.');
end

% --- 3. SAFETY CHECK & ENCODER CONVERSION ---
target_counts = zeros(1, 4);
for j = 1:4
    % Convert deg to counts (0 deg = 2048)
    target_counts(j) = round((target_angles_deg(j) * 4096 / 360) + 2048);
    
    % LECTURE 2 SAFETY: Joint 0 (ID 11) cap at 3400 (Page 58)
    if j == 1 && target_counts(j) > 3400
        target_counts(j) = 3400;
        warning('Base angle capped at 3400 to prevent internal collision.');
    end
end

% --- 4. HARDWARE EXECUTION ---

calllib(lib_name, 'write1ByteTxRx', port_num, PROTOCOL_VERSION, DXL_IDS(j), 64, 1);
    % Goal Position (Addr 116)
calllib(lib_name, 'write4ByteTxRx', port_num, PROTOCOL_VERSION, DXL_IDS(j), 116, target_counts(1));


pause(2); % Allow time to move

% --- 5. FORWARD KINEMATICS (DH Convention) ---
% Dimensions (mm)
L_base = 77; L_prox = 130.23; L_dist = 124; L_tool = 126;
angle_off = atan2(24, 128); 

q = deg2rad(target_angles_deg);

% Chaining transforms sequentially (Lecture 3, Page 77)
T01 = build_DH_matrix(0, 0, L_base, q(1));
T12 = T01 * build_DH_matrix(0, pi/2, 0, q(2) + angle_off);
T23 = T12 * build_DH_matrix(L_prox, 0, 0, q(3) - angle_off);
T34 = T23 * build_DH_matrix(L_dist, 0, 0, q(4));
T_tip = T34 * build_DH_matrix(L_tool, 0, 0, 0);

% Extracting coordinates for plotting [X; Y; Z]
pts = [0, T01(1,4), T12(1,4), T23(1,4), T34(1,4), T_tip(1,4);
       0, T01(2,4), T12(2,4), T23(2,4), T34(2,4), T_tip(2,4);
       0, T01(3,4), T12(3,4), T23(3,4), T34(3,4), T_tip(3,4)];

% --- 6. VISUALIZATION ---
figure('Color', 'w');
plot3(pts(1,:), pts(2,:), pts(3,:), '-ok', 'LineWidth', 3, 'MarkerSize', 8, 'MarkerFaceColor', 'r');
hold on; grid on; axis equal;
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
title('OpenManipulator-X 3D Stickman');
view(45, 30);

fprintf('End Effector at: X=%.2f, Y=%.2f, Z=%.2f\n', pts(1,6), pts(2,6), pts(3,6));

% Cleanup
calllib(lib_name, 'closePort', port_num);
% unloadlibrary(lib_name); % Optional

%% DH Function
function T = build_DH_matrix(a, alpha, d, theta)
    T = [cos(theta), -sin(theta), 0, a;
         sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -d*sin(alpha);
         sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), d*cos(alpha);
         0, 0, 0, 1];
end