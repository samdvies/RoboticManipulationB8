%% Manual Angle Input for OpenManipulator-X
clear all; close all; clc;

% --- 1. USER INPUT: SET YOUR ANGLES HERE (Degrees) ---
% Safe starting angles: [Base, Shoulder, Elbow, Wrist]
target_angles_deg = [180, 0, 45, 0]; 

% --- 2. Physical Constants & Hardware Config ---
L_base = 77; L_prox = 130.23; L_dist = 124; L_tool = 126;
angle_off = atan2(24, 128); 
DXL_IDS = [11, 12, 13, 14];
DEVICENAME = 'COM4'; 
BAUDRATE = 1000000;
PROTOCOL_VERSION = 2.0;

lib_name = 'dxl_x64_c';
% --- 3. Load Libraries ---
if ~libisloaded(lib_name)
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'alias', lib_name, 'addheader', 'port_handler.h');
    if ~isempty(notfound)
        disp('Functions not found in library:');
        disp(notfound);
        error('Library loading failed - missing functions');
    end
    if ~isempty(warnings)
        disp('Warnings during loading:');
        disp(warnings);
    end
end

libfunctions(lib_name)

port_num = portHandler(DEVICENAME);

if ~openPort(port_num) || ~setBaudRate(port_num, BAUDRATE)
    error('Hardware connection failed!');
end

% --- 4. Safety Check and Conversion ---
target_counts = zeros(1, 4);
for j = 1:4
    % Convert deg to encoder counts (0 deg = 2048)
    % Formula: (deg * 4096 / 360) + 2048
    target_counts(j) = round((target_angles_deg(j) * 4096 / 360) + 2048);
    
    % LECTURE 2 SAFETY LIMIT: Joint 0 (ID 11) collision guard
    if j == 1 && target_counts(j) > 3400
        warning('Base angle too high! Capping at 3400 to prevent collision.');
        target_counts(j) = 3400;
    end
end

% --- 5. Execution ---
fprintf('Moving to angles: [%.1f, %.1f, %.1f, %.1f]...\n', target_angles_deg);

% Enable Torque and Write Position

write1ByteTxRx(port_num, 2.0, 11, 64, 1); % Torque Enable
write4ByteTxRx(port_num, 2.0, 11, 116, target_counts(1));


pause(2); % Give the robot time to reach the position

% --- 6. Calculate & Print Current Position (Forward Kinematics) ---
q = deg2rad(target_angles_deg); % Use the angles we just set
T01 = build_DH_matrix(0, 0, L_base, q(1));
T12 = build_DH_matrix(0, pi/2, 0, q(2) + angle_off);
T23 = build_DH_matrix(L_prox, 0, 0, q(3) - angle_off);
T34 = build_DH_matrix(L_dist, 0, 0, q(4));
T_tip = T01 * T12 * T23 * T34 * build_DH_matrix(L_tool, 0, 0, 0);

fprintf('End Effector is at: X=%.2f, Y=%.2f, Z=%.2f\n', ...
    T_tip(1,4), T_tip(2,4), T_tip(3,4));

% --- 7. Plotting the Pose ---
% (Re-use your previous plotting logic here to see the "Stickman")
figure('Color', 'w');
plot3(pts(1,:), pts(2,:), pts(3,:), '-ok', 'LineWidth', 3, 'MarkerSize', 6, 'MarkerFaceColor', 'r');
hold on; grid on; axis equal;
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
title('Stickman Model');

% Draw Pincers
plot3([joint5(1,4) pincer1(1,4)], [joint5(2,4) pincer1(2,4)], [joint5(3,4) pincer1(3,4)], 'b', 'LineWidth', 2);
plot3([joint5(1,4) pincer2(1,4)], [joint5(2,4) pincer2(2,4)], [joint5(3,4) pincer2(3,4)], 'b', 'LineWidth', 2);

view(45, 30); 

% Optional: Disable torque if you want to move it by hand afterwards
% for id = DXL_IDS; write1ByteTxRx(port_num, 2.0, id, 64, 0); end

closePort(port_num);

%% DH Matrix Function
function T = build_DH_matrix(a, alpha, d, theta)
    T = [cos(theta), -sin(theta), 0, a;
         sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -d*sin(alpha);
         sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), d*cos(alpha);
         0, 0, 0, 1];
end