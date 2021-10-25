mu = [0; 0; 0; 5; 5; -5; 5];
P = ones(7, 7);
z = [2, pi];
landmark_idx = 1;
sigma_r = 0.01;
sigma_phi = 0.01;

[mu_new, P_new] = ekf_landmark_init(mu, P, z, sigma_r, sigma_phi)