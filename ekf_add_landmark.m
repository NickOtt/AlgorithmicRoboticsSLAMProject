function [mu_new, P_new] = ekf_add_landmark(mu, P, z, pos, sigma_r, sigma_phi)
    x = mu(1);
    y = mu(2);
    theta = mu(3);
    
    r = z(1);
    phi = z(2);
    
    p_x = pos(1);
    p_y = pos(2);
    
    landmark_x = x + r * cos(theta + phi) + (p_x - x) * (sin(theta + phi))^2 + (y - p_y) * sin(theta + phi) * cos(theta + phi);
    landmark_y = y + r * sin(theta + phi) + (x - p_x) * sin(theta + phi) * cos(theta + phi) + (p_y - y) * (cos(theta + phi))^2;
    
    partial_lx_x = 1 - (sin(theta + phi))^2;
    partial_lx_y = sin(theta + phi) * cos(theta + phi);
    
    partial_ly_x = sin(theta + phi) * cos(theta + phi);
    partial_ly_y = 1 - (cos(theta + phi))^2;
    
    partial_lx_angle = -r * sin(theta + phi) + 2 * (p_x - x) * sin(theta + phi) * cos(theta + phi) + (y - p_y) * ((cos(theta + phi))^2 - (sin(theta + phi))^2);
    partial_ly_angle = r * cos(theta + phi) + (x - p_x) * ((cos(theta + phi))^2 - (sin(theta + phi))^2) + 2 * (y - p_y) * cos(theta + phi) * sin(theta + phi);
    
    % Partial of landmark location wrt robot position (x,y,theta)
    G_r = [partial_lx_x, partial_lx_y, partial_lx_angle;
        partial_ly_x, partial_ly_y, partial_ly_angle];
    
    % Partial of landmark loca4tion wrt measurement (r,phi)
    G_y = [cos(theta + phi), partial_lx_angle;
        sin(theta + phi), partial_ly_angle];
    
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

