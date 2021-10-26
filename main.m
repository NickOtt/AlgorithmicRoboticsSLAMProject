% Environment setup
addpath(genpath('src'))

% Create lidar sensor
lidar = LidarSensor;
lidar.sensorOffset = [0,0];
lidar.scanAngles = linspace(-pi/2,pi/2,51);
lidar.maxRange = 5;

% General parameters
mu = [0; 0; 0];
P = [0.1, 0, 0;
    0, 0.1, 0;
    0, 0, 0.02];

alpha = [0.0001;  0.0001;  0.01;  0.0001;  0.0001;  0.0001];
commands = [[1;0], [1;0], [1;0], [pi/2;pi/2], [pi/2;pi/2], [1;0], [1;0], [1;0]];
sigma_r = 0.01;
sigma_phi = 0.01;

% Ransac Params
N = 100;
dist = 0.1;
L = 4;
n_thresh = 10;

landmark_counts = [];

for i=1:length(commands)
    [mu_bar, P_bar] = ekf_prediction(mu, P, commands(:, i), alpha);

    % Simulate motion
    curPose = sample_motion_model_velocity(command(i),mu_bar,alpha);
    % Simulate laser scan
    ranges = lidar(curPose); % might need  to add map and do more setup

    scan = []; % Array of distances
    coords = scan_to_xy(scan, mu_bar);
    lines = multi_line_ransac(coords, N, dist, L, n_thresh);
    % Robot position
    pos = [0; 0];

    % Find closest points as landmarks
    measured_landmarks = zeros(2, size(lines, 2));
    for j = 1:size(lines, 2)
        measured_landmarks(:, j) = closest_point_on_line(lines(:, j), pos);
    end


    landmarks = reshape(mu_bar(4:end), 2, []);
    [matched_indices, landmark_counts] = associate_landmarks(landmarks, measured_landmarks, landmark_covs, landmark_counts, lambda);
    
    for j = 1:size(measured_landmarks, 2)
        if matched_indices(j) ~= 0
            z_actual = landmark_measurement(measured_landmarks(:,j), mu_bar(1:3));
            [mu_bar, P_bar] = ekf_correction(mu_bar, P_bar, z_actual, matched_indices(j), sigma_r, sigma_phi);
        end
    end
    
    for j = 1:size(measured_landmarks, 2)
        if matched_indices(j) == 0
            z_actual = landmark_measurement(measured_landmarks(:,j), mu_bar(1:3));
            [mu_bar, P_bar] = ekf_add_landmark(mu_bar, P_bar, z_actual, sigma_r, sigma_phi);
        end
    end
    
    mu = mu_bar;
    P = P_bar;
end


