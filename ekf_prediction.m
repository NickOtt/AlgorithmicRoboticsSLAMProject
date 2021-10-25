function [mu_bar, P_bar] = ekf_prediction(mu, P, u, alpha)
    mu_bar = mu;
    mu_bar(1:3) = model_velocity(u, mu(1:3));
    
    v = u(1);
    omega = u(2);
    theta = mu(3);
    
    N = [alpha(1) * v^2 + alpha(2) * omega^2, 0, 0;
        0, alpha(3) * v^2 + alpha(3) * omega^2, 0;
        0, 0, alpha(5) * v^2 + alpha(6) * omega^2];
    

    if omega == 0
        F_r = [1, 0, -v * sin(theta);
             0, 1, v * cos(theta);
             0, 0, 1];
        
        F_n = [cos(theta), -v * sin(theta) / 2, 0;
            sin(theta), v * cos(theta) / 2, 0;
            0, 1, 1];
    else
        r = v / omega;
       
    	F_r = [1, 0, -r * cos(theta) + r * cos(theta + omega);
             0, 1, -r * sin(theta) + r * sin(theta + omega);
             0, 0, 1];
    	F_n = [ (-sin(theta) + sin(theta + omega)) / omega, v * (sin(theta) - sin(theta + omega)) / (omega^2) + v * cos(theta + omega) / omega, 0;
            (cos(theta) - cos(theta + omega)) / omega, -v * (cos(theta) - cos(theta + omega)) / (omega^2) + v * sin(theta + omega) / omega, 0;
            0, 1, 1];   
    end
    
    P_rr = (F_r * P(1:3, 1:3) * transpose(F_r)) + (F_n * N * transpose(F_n));
    
    state_size = length(mu);
    
    P_bar = P;
    P_bar(1:3,1:3) = P_rr;
    
    P_bar(1:3, 4:state_size) = F_r * P(1:3, 4:state_size);
    P_bar(4:state_size, 1:3) = transpose(P_bar(1:3, 4:state_size));
end

function x_new = model_velocity(u, x)
    v = u(1);
    omega = u(2);
    theta_prime = x(3) + omega;
    
    if omega == 0
        x_prime = x(1) + v * cos(x(3));
        y_prime = x(2) + v * sin(x(3));
        x_new = [x_prime; y_prime; theta_prime];
    else
        r = v / omega;

        x_c = x(1) - r * sin(x(3));
        y_c = x(2) + r * cos(x(3));
        
        x_prime = x_c + r * sin(x(3) + omega);
        y_prime = y_c - r * cos(x(3) + omega);
        x_new = [x_prime; y_prime; theta_prime];
    end
end

