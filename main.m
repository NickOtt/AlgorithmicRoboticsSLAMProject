clear;
close all;

addpath(genpath('src'))

rng(499);

% Create map
p = zeros(20,20);
p(5,2) = 1;
p(5,7) = 1;
p(5,13) = 1;
p(16,2) = 1;
p(16,7) = 1;
p(16,13) = 1;
p(:,1) = 1;
p(:,20) = 1;
p(1,:) = 1;
p(20,:) = 1;
map = occupancyMap(p,2);

% Create lidar sensor
lidar = LidarSensor;
lidar.sensorOffset = [0,0];
lidar.scanAngles = linspace(-pi/2,pi/2,51);
lidar.maxRange = 5;

% Create visualizer
viz = Visualizer2D;
viz.hasWaypoints = false;
viz.mapName = 'map';
attachLidarSensor(viz,lidar);

mu = [1; 1.5; 0];
P = [0.01, 0, 0;
    0, 0.01, 0;
    0, 0, 0.02];

alpha = [0.0001;  0.0001;  0.001;  0.0001;  0.0001;  0.0001];
%commands = [[1;0], [1;0], [1;0], [pi/2;pi/2], [pi/2;pi/2], [1;0], [1;0], [1;0]];
commands = [[0.2;0], [0.2;0], [0.2;0], [0.2;0], [0.2;0], [0.2;0], [0.2;0], [0.2;0], [0.2;0], [0.2;0], [0.2;0], [0.2;0], [0.2;0], [0.2;0], [0.2;0], [0.2;0], [0.2;0], [0.2;0], [0.2;0], [0.2;0], [0.2;0], [0.2;0], [0.2;0]];
sigma_r = 0.01;
sigma_phi = 0.01;

% Ransac Params
N = 100; % Number of times to try to find a line
dist = 0.01; % Dist at which point is matched to a line
L = 5; % Max number of lines to extract
n_thresh = 5; % How many points need to be in a line

% Data Association params
lambda = 5; % std dev at which a landmark the same (%chi squared 95% confidence)

% How many times each landmark was found
landmark_counts = [];

for i=1:length(commands)
    % Step 1: Predict current state forward
    [mu_bar, P_bar] = ekf_prediction(mu, P, commands(:, i), alpha);

    % Simulate motion (This is where the robot "actually" is)
    % Simulate laser scan
    % Only pass non-nan values (max distance) to future functions
    curPose = sample_motion_model_velocity(commands(:,i),mu,alpha);
    scan = lidar(curPose);
    hits = ~isnan(scan);
    
    % Convert scan to xy coordinates based on where robot "thinks" it is
    coords = scan_to_xy(scan(hits), lidar.scanAngles(hits), mu_bar);
        
    % Extract lines using ransac
    lines = multi_line_ransac(coords, N, dist, L, n_thresh);
    
    % Plot visualization
    plot_world(mu_bar, P_bar, coords, map, lines);
    pause;

    % Find the closest points on each line to a given global position
    % This is the landmark point
    
    % Should the pos be based on where the robot was when the landmark was
    % first found?
    pos = [5; 5];
    measured_landmarks = zeros(2, size(lines, 2));
    for j = 1:size(lines, 2)
        measured_landmarks(:, j) = closest_point_on_line(lines(:, j), pos);
    end
    
    % Exctract the landmarks from the current state. Associate measured
    % landmarks with existing landmarks, update the counts. New landmarks
    % are those in the measured_landmarks array with matched_indices == 0
    % Landmakrs covs are extracted from matrix P. Potentially, they should
    % be a combination of the robot cov and the landmark cov. 
    landmarks = reshape(mu_bar(4:end), 2, []);
    landmark_covs = P_bar;

    [matched_indices, landmark_counts] = associate_landmarks(landmarks, measured_landmarks, landmark_covs, landmark_counts, lambda);
    
    % Run update using detected landmarks to update robot and landmarks
    for j = 1:size(measured_landmarks, 2)
        if matched_indices(j) ~= 0
            z_actual = landmark_measurement(measured_landmarks(:,j), mu_bar(1:3));
            [mu_bar, P_bar] = ekf_correction(mu_bar, P_bar, z_actual, matched_indices(j), sigma_r, sigma_phi);
        end
    end
       
    % Add newly detected landmarks to the array
    for j = 1:size(measured_landmarks, 2)
        if matched_indices(j) == 0
            z_actual = landmark_measurement(measured_landmarks(:,j), mu_bar(1:3));
            [mu_bar, P_bar] = ekf_add_landmark(mu_bar, P_bar, z_actual, sigma_r, sigma_phi);
        end
    end
    
    plot_world(mu_bar, P_bar, coords, map, lines);
    pause;
    
    mu = mu_bar;
    P = P_bar;
end
