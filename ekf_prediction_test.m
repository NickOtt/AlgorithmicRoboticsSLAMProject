% Test data
mu = [2; 2; 0; 0; 0];
P = ones(5, 5);
alpha = [0.0001;  0.0001;  0.01;  0.0001;  0.0001;  0.0001];

[mu_bar, P_bar] = ekf_prediction(mu, P, [1;0], alpha)