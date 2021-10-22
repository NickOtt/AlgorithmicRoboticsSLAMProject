% Determines if landmarks are new landmarks or old landmarks being refound

% Remove all plots
clear;
close all;

% Create some random lines
lines = 3 * rand(3, 3) - 1.5;

% Robot position
pos = [0; 0];

% Find closest points as landmarks
closest = zeros(2, size(lines, 2));
for i = 1:size(lines, 2)
   closest(:, i) = closest_point_on_line(lines(:, i), pos);
end

hold on
scatter(pos(1), pos(2)); % Robot position
xlim([-4, 4]);
ylim([-4, 4]);

% Plot lines and closest points
for i = 1:size(lines, 2)
    line = lines(:, i);
    x = -4:0.5:4;
    y = (line(1) * x + line(3))/-line(2);
    plot(x, y);
    scatter(closest(1, i), closest(2, i));
end

function p = closest_point_on_line(line, point)
    % Finds the closest point on a line to the start point
    % From https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
    a = line(1); b = line(2); c = line(3);
    x0 = point(1); y0 = point(2);
    x = (b*(b*x0 - a*y0) - a*c) / (a^2 + b^2);
    y = (a*(-b*x0 + a*y0) - b*c) / (a^2 + b^2);
    p = [x; y];
end

function [new_landmarks, new_counts] = associate_landmarks(landmarks, counts, covs, detections, lambda)
    % Given a current selection of landmarks and how many times each has
    % been detected, along with position covariances for each,
    % associate new landmarks
    
    new_landmarks = landmarks;
    new_counts = counts;
    
    % For each landmark
    for i = 1:size(detections, 2)
        l = detections(:, i);
        
        % Find which it is closest to
        closest = closest_landmark(landmarks, l);
        if is_in_range(landmarks(:, closest), l, covs(:, :, i), lamdba)
            % If close to an existing landmark increment count by one
            counts(closest) = counts(closest) + 1;
        else
            % Otherwise, it is new so add it to the list
            new_landmarks = [new_landmarks, l];
            new_counts = [new_counts, 1];
        end
    end
end

function y = is_in_range(l_current, l_new, cov, lambda)
    % If a new observed landmark is some std devs close enough to
    % an existing one, it is considered an observation of the same one
    err = l_current - l_new; % Innovation is just the error term
       
    % Check if the new landmark is close enough
    y = (err.' * inv(cov) * err) <= lambda;
end

function index = closest_landmark(landmarks, l)
    % Returns the index of the landmark closest to a new, detected landmark
    min_dist = 1000;
    min_i = 0;
    for i = 1:size(landmarks, 2)
       dist = (landmarks(1) - l(1))^2 + (landmarks(2) - l(2))^2;
       if dist < min_dist
          min_dist = dist;
          min_i = i;
       end
    end
    index = min_i;
end