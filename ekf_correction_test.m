mu_bar = [0; 0; 0; 5; 5; -5; 5];
P_bar = ones(7, 7);
z = [sqrt(50); pi / 4];
landmark_idx = 1;
sigma_r = 0.01;
sigma_phi = 0.01;

[mu, P] = ekf_correction(mu_bar, P_bar, z, landmark_idx, sigma_r, sigma_phi)