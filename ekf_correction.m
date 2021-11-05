function [mu, P] = ekf_correction(mu_bar, P_bar, z, landmark_idx, pos, sigma_r, sigma_phi)
    % Get estimated robot position
    x = mu_bar(1);
    y = mu_bar(2);
    theta = mu_bar(3);
    % Get estimated landmark position
    lambda_x = mu_bar(2 * landmark_idx + 2);
    lambda_y = mu_bar(2 * landmark_idx + 3);
    
%     r_sq = (lambda_x - x)^2 + (lambda_y - y)^2;
%     r = sqrt(r_sq);
%     
%     % Expected measurement
%     z_hat = [r; atan2(lambda_y - y, lambda_x - x) - theta];
%     
%     % Calculate partial derivative of expected measurement wrt state
    H = zeros(2, length(mu_bar));
%     
%     H(1:2, 1:3) = [(x - lambda_x) / r, (y - lambda_y) / r, 0;
%         (lambda_y - y) / r_sq, (lambda_x - x) / r_sq, -1];
%     
%     H(1:2, 2 * landmark_idx + 2: 2 * landmark_idx + 3) = -H(1:2, 1:2);
    
    p_x = pos(1);
    p_y = pos(2);
    
    phi_exp = atan2(lambda_y - p_y, lambda_x - p_x) - theta;
    
    r_sq = (lambda_x - p_x)^2 + (lambda_y - p_y)^2;
    r = sqrt(r_sq);
    
    partial_phi_lx = (p_y - lambda_y) / r_sq;
    partial_phi_ly = (lambda_x - p_x) / r_sq;
    
    dot = (lambda_x - p_x) * (lambda_x - x) + (lambda_y - p_y) * (lambda_y - y);
    range_exp = dot / r;
    
    partial_range_x = (p_x - lambda_x) / r;
    partial_range_y = (p_y - lambda_y) / r;
    
    partial_dot_lx = 2 * lambda_x - p_x - x;
    partial_dot_ly = 2 * lambda_y - p_y - y;
    
    partial_r_lx = (lambda_x - p_x) / r;
    partial_r_ly = (lambda_y - p_y) / r;
    
    partial_range_lx = (partial_dot_lx * r - partial_r_lx * dot) / r_sq;
    partial_range_ly = (partial_dot_ly * r - partial_r_ly * dot) / r_sq;
    
    if range_exp < 0
        range_exp = -range_exp;
        partial_range_x = -partial_range_x;
        partial_range_y = -partial_range_y;
        partial_range_lx = -partial_range_lx;
        partial_range_ly = -partial_range_ly;
        
        phi_exp = phi_exp + pi;
    end
    
    z_hat = [range_exp; phi_exp];
    
    H(1:2, 1:3) = [partial_range_x, partial_range_y, 0;
        0, 0, -1];
    
    H(1:2, 2 * landmark_idx + 2: 2 * landmark_idx + 3) = [
        partial_range_lx, partial_range_ly;
        partial_phi_lx, partial_phi_ly];
    
    
    
    Q = [sigma_r^2, 0;
        0, sigma_phi^2];
    
    S = H * P_bar * transpose(H) + Q;
    K = P_bar * transpose(H) / S;
    z_diff = z - z_hat;
    z_diff(2) = clamp(z_diff(2));
    
    mu = mu_bar + (K * z_diff);
    P = P_bar - (K * S * transpose(K));
end

function y = clamp(x)
    y = mod(x + pi, 2 * pi) - pi;
end