function [mu, P] = ekf_correction(mu_bar, P_bar, z, landmark_idx, sigma_r, sigma_phi)
    % Get estimated robot position
    x = mu_bar(1);
    y = mu_bar(2);
    theta = mu_bar(3);
    % Get estimated landmark position
    lambda_x = mu_bar(2 * landmark_idx + 2);
    lambda_y = mu_bar(2 * landmark_idx + 3);
    
    r_sq = (lambda_x - x)^2 + (lambda_y - y)^2;
    r = sqrt(r_sq);
    
    % Expected measurement
    z_hat = [r; atan2(lambda_y - y, lambda_x - x) - theta];
    
    % Calculate partial derivative of expected measurement wrt state
    H = zeros(2, length(mu_bar));
    
    H(1:2, 1:3) = [(x - lambda_x) / r, (y - lambda_y) / r, 0;
        (lambda_y - y) / r_sq, (lambda_x - x) / r_sq, -1];
    
    H(1:2, 2 * landmark_idx + 2: 2 * landmark_idx + 3) = -H(1:2, 1:2);
    
    Q = [sigma_r^2, 0;
        0, sigma_phi^2];
    
    S = H * P_bar * transpose(H) + Q;
    K = P_bar * transpose(H) / S;
    z_diff = z - z_hat;
    z_diff(2) = clamp(z_diff(2));
    
    mu = mu_bar + (K * z_diff);
    %P = (eye(length(P_bar)) - (K * H)) * P_bar;
    P = P_bar - (K * S * transpose(K));
end

function y = clamp(x)
    y = mod(x + pi, 2 * pi) - pi;
end