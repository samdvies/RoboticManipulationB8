world_frame = eye(4);
transform_fixed = eye(4); %temp transforms
joint1 = world_frame*fixed_transform;
transform_1 = eye(4);%this will define the rotation of the first servo at the base of the first length of arm
joint2 = joint1*trasnform_1;
transform_2 = eye(4); %this will define the second servo and the length of the first arm section
joint3 = transform_2*joint2;
transform_3 = eye(4); %this defines the third servo and second length of arm
joint4 = joint3*transform_3;
transform_5 = eye(4); %this defined the servo at the end of the second length of arm, we are now on the hand of the arm
joint5 = joint4*transform_5;
transform_6=eye(4); %these define pincer movement relative to the hand block
transform_7 = eye(4);
pincer1 = joint5*transform_6;
pincer2 = joint5*transform_7;
%Robot arm Lengths
L_base = 77; L_prox = 130.23; L_dist = 124; L_tool = 126;
function T = build_DH_matrix(a, alpha, d, theta)  
    T = eye(4);
    T(1:3,1) = [cos(theta); sin(theta)*cos(alpha); sin(theta)*sin(alpha);];
    T(1:3, 2) = [-sin(theta); cos(alpha)* cos(theta); cos(theta)*sin(alpha);];
    T(1:3, 3) = [0; -sin(alpha); cos(alpha);];
    T(1:3, 4) = [a; -d*sin(alpha); d*cos(alpha);];

end
T01 = build_DH_matrix(0, 0, L_base, theta1);
joint1 = world_frame * T01;
T12 = build_DH_matrix(0, pi/2, 0, theta2 + offset);
joint2 = joint1 * T12;
T23 = build_DH_matrix()