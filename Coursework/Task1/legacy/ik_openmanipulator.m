function [q1, q2, q3, q4] = ik_openmanipulator(x, y, z)

    % --- Robot dimensions (must match your FK code) ---
    L_base = 77;
    L_prox_x = 24; 
    L_prox_z = 128;
    L_prox = sqrt(L_prox_x^2 + L_prox_z^2);
    L_dist = 124;
    L_tool = 126;
    angle_off = atan2(L_prox_x, L_prox_z);

    % --- 1) Base angle ---
    q1 = atan2(y, x);

    % --- 2) Reduce to 2D problem ---
    r = sqrt(x^2 + y^2);
    z2 = z - L_base;

    % Remove tool length
    r2 = r - L_tool;

    % --- 3) Triangle solve (law of cosines) ---
    D = (r2^2 + z2^2 - L_prox^2 - L_dist^2) / (2 * L_prox * L_dist);

    % Clamp for numerical safety
    D = max(min(D,1),-1);

    q3 = atan2( sqrt(1 - D^2), D );   % elbow-down solution

    % --- 4) Shoulder angle ---
    phi = atan2(z2, r2);
    psi = atan2(L_dist * sin(q3), L_prox + L_dist * cos(q3));

    q2 = phi - psi;

    % --- 5) Wrist angle (keep tool roughly horizontal) ---
    q4 = -(q2 + q3);

    % --- 6) Apply mechanical offsets ---
    q2 = q2 - angle_off;
    q3 = q3 + angle_off;

end