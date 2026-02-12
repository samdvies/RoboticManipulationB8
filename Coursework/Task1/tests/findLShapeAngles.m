function findLShapeAngles()
    % Simplified output format
    fprintf('\n\n--- RESULTS START ---\n');
    
    % Candidate 1: Standard L [0, pi/2, -pi/2, 0]
    pos = simpleFK([0, pi/2, -pi/2, 0]);
    fprintf('C1 %.2f %.2f\n', pos(1), pos(3));
    
    % Candidate 2: Vertical I [0, pi/2, 0, 0]
    pos = simpleFK([0, pi/2, 0, 0]);
    fprintf('C2 %.2f %.2f\n', pos(1), pos(3));
    
    % Candidate 3: Inverted L [0, -pi/2, pi/2, 0]
    pos = simpleFK([0, -pi/2, pi/2, 0]);
    fprintf('C3 %.2f %.2f\n', pos(1), pos(3));
    
    fprintf('--- RESULTS END ---\n');
end

function pos = simpleFK(q)
    L_base = 77;
    L_prox_x = 24; L_prox_z = 128;
    L_prox = sqrt(L_prox_x^2 + L_prox_z^2);
    L_dist = 124;
    L_tool = 126;
    beta = atan2(L_prox_x, L_prox_z);
    
    T01 = dh(0, 0, 77, q(1));
    T12 = dh(0, pi/2, 0, q(2) + beta);
    T23 = dh(L_prox, 0, 0, q(3) - beta);
    T34 = dh(L_dist, 0, 0, q(4));
    T4t = dh(L_tool, 0, 0, 0);
    
    T = T01 * T12 * T23 * T34 * T4t;
    pos = T(1:3, 4);
end

function T = dh(a, alpha, d, theta)
    ct = cos(theta); st = sin(theta);
    ca = cos(alpha); sa = sin(alpha);
    T = [ct, -st*ca,  st*sa, a*ct;
         st,  ct*ca, -ct*sa, a*st;
          0,     sa,     ca,    d;
          0,      0,      0,    1];
end
