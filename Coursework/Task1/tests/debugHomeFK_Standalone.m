function debugHomeFK_Standalone()
    % Hardcoded FK for Home (0,0,0,0)
    
    L_base = 77;
    L_prox_x = 24; L_prox_z = 128;
    L_prox = sqrt(L_prox_x^2 + L_prox_z^2);
    L_dist = 124;
    L_tool = 126;
    beta = atan2(L_prox_x, L_prox_z);
    
    q = [0, 0, 0, 0];
    
    T01 = dh(0, 0, 77, 0);
    T12 = dh(0, pi/2, 0, 0 + beta);
    T23 = dh(L_prox, 0, 0, 0 - beta);
    T34 = dh(L_dist, 0, 0, 0);
    T4t = dh(L_tool, 0, 0, 0);
    
    T = T01 * T12 * T23 * T34 * T4t;
    pos = T(1:3, 4);
    
    % Print everything on one line to avoid truncation issues
    fprintf('HOME_FK_RESULT: X=%.2f, Y=%.2f, Z=%.2f\n', pos(1), pos(2), pos(3));
end

function T = dh(a, alpha, d, theta)
    ct = cos(theta); st = sin(theta);
    ca = cos(alpha); sa = sin(alpha);
    T = [ct, -st*ca,  st*sa, a*ct;
         st,  ct*ca, -ct*sa, a*st;
          0,     sa,     ca,    d;
          0,      0,      0,    1];
end
