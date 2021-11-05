clear;
close all;

addpath(genpath('src'))

rng(499);

% Create map
p = zeros(16,16);
p(3:4,9:10) = 1;
p(11:12,7:9) = 1;
p(5:6,12:16) = 1;
% p(8:9,5:6) = 1;
p(:,1) = 1;
p(:,end) = 1;
p(1,:) = 1;
p(end,:) = 1;
map = occupancyMap(p,2);

% Create lidar sensor
lidar = LidarSensor;
lidar.sensorOffset = [0,0];
lidar.scanAngles = linspace(-pi/2,pi/2,101);
lidar.maxRange = 10;
lidar_noise_sigma = 0.01;

% Create visualizer
viz = Visualizer2D;
viz.hasWaypoints = false;
viz.mapName = 'map';
attachLidarSensor(viz,lidar);

mu = [1; 1.5; 0];
P = [0.01, 0, 0;
    0, 0.01, 0;
    0, 0, 0.002];

alpha = [0.0001;  0.0001;  0.0001;  0.0001;  0.005;  0];
% alpha = zeros(1, 6);
%alpha = [0.05;  0.0;  0.002;  0.0;  0.0;  0.0];

commands = [[1;0], [1;0], [1;0], [1;0], [pi/4;pi/4], [pi/4;pi/4], [1;0], [pi/4;pi/4], [pi/4;pi/4], [1;0], [1;0], [1;0], [1;0]];

% Noise params
sigma_r = 0.1;
sigma_phi = 0.1;

% Ransac Params
N = 100; % Number of times to try to find a line
dist = 0.01; % Dist at which point is matched to a line
L = 5; % Max number of lines to extract
n_thresh = 5; % How many points need to be in a line

% Data Association params
lambda = 6; % std dev at which a landmark the same (%chi squared 95% confidence)

% How many times each landmark was found
landmark_counts = [];

% To store history
real_poses = [mu];
filtered_poses = [mu];
commanded_poses = [mu];
real_pos = real_poses(:, end);

for i=1:length(commands)
    % Simulate laser scan
    % Only pass non-nan values (max distance) to future functions
    scan = lidar(real_pos);
    noisy_scan = make_scan_noisy(scan,lidar_noise_sigma)';
    hits = ~isnan(noisy_scan);
    
    % Convert scan to xy coordinates based on where robot "thinks" it is
    coords = scan_to_xy(noisy_scan(hits), lidar.scanAngles(hits), mu);
        
    % Extract lines using ransac
    lines = multi_line_ransac(coords, N, dist, L, n_thresh);
    
    % Plot visualization
    plot_world(mu, P, coords, map, lines, real_poses, filtered_poses, commanded_poses, []);
    pause;

    % Find the closest points on each line to a given global position
    % This is the landmark point 
    
    % Should the pos be based on where the robot was when the landmark was
    % first found?
    pos = [4; 4];
    measured_landmarks = zeros(2, size(lines, 2));
    for j = 1:size(lines, 2)
        measured_landmarks(:, j) = closest_point_on_line(lines(:, j), pos);
    end
    
    plot_world(mu, P, coords, map, lines, real_poses, filtered_poses, commanded_poses, measured_landmarks);
    pause;
    
    % Exctract the landmarks from the current state. Associate measured
    % landmarks with existing landmarks, update the counts. New landmarks
    % are those in the measured_landmarks array with matched_indices == 0
    % Landmakrs covs are extracted from matrix P. Potentially, they should
    % be a combination of the robot cov and the landmark cov. 
    landmarks = reshape(mu(4:end), 2, []);
    landmark_covs = P;

    [matched_indices, landmark_counts] = associate_landmarks(landmarks, measured_landmarks, landmark_covs, landmark_counts, lambda);
        
    % Run update using detected landmarks to update robot and landmarks
    for j = 1:size(measured_landmarks, 2)
        if matched_indices(j) ~= 0
            landmark = closest_point_on_line(lines(:, j), mu(1:3));
            z_actual = landmark_measurement(landmark, mu(1:3));
            [mu, P] = ekf_correction(mu, P, z_actual, matched_indices(j), pos, sigma_r, sigma_phi);
        end
    end
    
    plot_world(mu, P, coords, map, lines, real_poses, filtered_poses, commanded_poses, measured_landmarks);
    pause;

    % Add newly detected landmarks to the array
    for j = 1:size(measured_landmarks, 2)
        if matched_indices(j) == 0
            landmark = closest_point_on_line(lines(:, j), mu(1:3));
            z_actual = landmark_measurement(landmark, mu(1:3));
            [mu, P] = ekf_add_landmark(mu, P, z_actual, pos, sigma_r, sigma_phi);
        end
    end
    
    % Off by one errors, becuase we measure landmarks first now
    if i ~= 1
        % Update filtered poses history
        filtered_poses = [filtered_poses, mu(1:3)];
    end
    plot_world(mu, P, coords, map, lines, real_poses, filtered_poses, commanded_poses, []);
    pause;

    % Off by one errors, becuase we measure landmarks first now
    if i ~= size(commands, 2)
        % Step 1: Predict current state forward
        [mu, P] = ekf_prediction(mu, P, commands(:, i), alpha);

        % Simulate motion (This is where the robot "actually" is)
        real_pos = sample_motion_model_velocity(commands(:,i),real_poses(:, end),alpha);
        
        % Update real poses history
        real_poses = [real_poses, real_pos];

        % Update ideal commanded poses history
        commanded_poses = [commanded_poses, sample_motion_model_velocity(commands(:,i),commanded_poses(:, end),zeros(1, 6))];
    end
end

commanded_poses
real_poses
filtered_poses
plot_world(mu, P, coords, map, lines, real_poses, filtered_poses, commanded_poses, []);

