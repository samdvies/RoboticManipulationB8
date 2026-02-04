%% OpenManipulator-X DH Simulation (Craig Notation)
clear all; close all; clc;

% 1. Input Joint Angles (Degrees - Change these to move the robot)
theta1_deg = 0;   % Base rotation
theta2_deg = -15;  % Shoulder
theta3_deg = 150;   % Elbow
theta4_deg = 70;   % Wrist
gripper_gap = 20;  % Gap between pincers in mm

% Convert to Radians
q1 = deg2rad(theta1_deg);
q2 = deg2rad(theta2_deg);
q3 = deg2rad(theta3_deg);
q4 = deg2rad(theta4_deg);

L_base = 77;                             % Height of Joint 1 to Joint 2
L_prox_x = 24; L_prox_z = 128;           % Slanted link offsets
L_prox = sqrt(L_prox_x^2 + L_prox_z^2);  % Total diagonal length (~130.23mm)
L_dist = 124;                            % Joint 3 to Joint 4
L_tool = 126;                            % Joint 4 to End Effector
angle_off = atan2(L_prox_x, L_prox_z);   % Fixed mechanical tilt (~10.6 deg)

% 3. Define Transformations using DH Parameters (a, alpha, d, theta)
T01 = build_DH_matrix(0, 0, L_base, q1);               % Base to Shoulder
T12 = build_DH_matrix(0, pi/2, 0, q2 + angle_off);     % Shoulder to Elbow
T23 = build_DH_matrix(L_prox, 0, 0, q3 - angle_off);   % Elbow to Wrist
T34 = build_DH_matrix(L_dist, 0, 0, q4);               % Wrist to Tool
T4tool = build_DH_matrix(L_tool, 0, 0, 0);             % Tool Tip

% 4. Chain the Transformations (Forward Kinematics)
joint1 = T01;
joint2 = joint1 * T12;
joint3 = joint2 * T23;
joint4 = joint3 * T34;
joint5 = joint4 * T4tool;

pinc1_offset = [1 0 0 0; 0 1 0 0; 0 0 1 (gripper_gap/2); 0 0 0 1];
pinc2_offset = [1 0 0 0; 0 1 0 0; 0 0 1 -(gripper_gap/2); 0 0 0 1];
pincer1 = joint5 * pinc1_offset;
pincer2 = joint5 * pinc2_offset;

% 6. Extract Coordinates for Plotting (X, Y, Z are in the 4th column)
pts = [0, joint1(1,4), joint2(1,4), joint3(1,4), joint4(1,4), joint5(1,4);
       0, joint1(2,4), joint2(2,4), joint3(2,4), joint4(2,4), joint5(2,4);
       0, joint1(3,4), joint2(3,4), joint3(3,4), joint4(3,4), joint5(3,4)];

% 7. Visualization
figure('Color', 'w');
plot3(pts(1,:), pts(2,:), pts(3,:), '-ok', 'LineWidth', 3, 'MarkerSize', 6, 'MarkerFaceColor', 'r');
hold on; grid on; axis equal;
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
title('Stickman Model');

% Draw Pincers
plot3([joint5(1,4) pincer1(1,4)], [joint5(2,4) pincer1(2,4)], [joint5(3,4) pincer1(3,4)], 'b', 'LineWidth', 2);
plot3([joint5(1,4) pincer2(1,4)], [joint5(2,4) pincer2(2,4)], [joint5(3,4) pincer2(3,4)], 'b', 'LineWidth', 2);

view(45, 30); % Set perspective view

%% DH Matrix Function 
function T = build_DH_matrix(a, alpha, d, theta)
    T = [cos(theta),           -sin(theta),           0,           a;
         sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -d*sin(alpha);
         sin(theta)*sin(alpha), cos(theta)*sin(alpha),  cos(alpha),  d*cos(alpha);
         0,                     0,                     0,           1];
end