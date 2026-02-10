function findLShape()
    % Try to find the angles that make the arm look like an L-shape (High Z)
    
    % Test candidates
    candidates = {
        [0, 0, 0, 0],         'All Zeros';
        [0, -pi/2, 0, 0],     'Shoulder -90';
        [0, pi/2, 0, 0],      'Shoulder +90';
        [0, -pi/2, pi/2, 0],  'Shoulder -90, Elbow +90';
        [0, -pi/2, -pi/2, 0], 'Shoulder -90, Elbow -90';
    };
    
    for i = 1:size(candidates, 1)
        q = candidates{i, 1};
        name = candidates{i, 2};
        pos = simpleFK(q);
        fprintf('%s: Z = %.2f mm\n', name, pos(3));
    end
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
