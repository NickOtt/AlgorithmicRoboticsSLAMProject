function [mu_new, P_new] = ekf_add_landmark(mu, P, z, sigma_r, sigma_phi)
    x = mu(1);
    y = mu(2);
    theta = mu(3);
    
    r = z(1);
    phi = z(2);
    
    landmark_x = x + r * cos(theta + phi);
    landmark_y = y + r * sin(theta + phi);
    
    % Partial of landmark location wrt robot position (x,y,theta)
    G_r = [1, 0, -r * sin(theta + phi);
        0, 1, r * cos(theta + phi)];
    
    % Partial of landmark location wrt measurement (r,phi)
    G_y = [cos(theta + phi), -r * sin(theta + phi);
        sin(theta + phi), r * cos(theta + phi)];
    
    % Measurement error
    R = [sigma_r^2, 0;
        0, sigma_phi^2];
    
    cur_state_len = length(mu);
    
    P_ll = (G_r * P(1:3, 1:3) * transpose(G_r)) + (G_y * R * transpose(G_y));
    P_lx = G_r * P(1:3, 1:cur_state_len);
    
    mu_new = [mu; landmark_x; landmark_y];
    P_new = [P, transpose(P_lx);
        P_lx, P_ll];
    
end

