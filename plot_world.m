% Close everything for testing
close all;
clear;

mu = [1;2;3; 0; 1; 2; 1];
P = 0.1 * diag(rand(size(mu)));
coords = 10 * rand(2, 20) - 5;

plot_world(mu, P, coords);


% Plots the robot pose, landmarks, covariance ellipses, and the laser scan
function plot_world(mu, P, coords)
    hold on;
    
    % Plot robot position
    scatter(mu(1), mu(2), 'blue');
    
    % Extract and plot landmarks
    scatter(mu(4:2:end-1), mu(5:2:end), 'red');
    
    % Plot robot cov
    plot_cov_ellipse(mu(1:2), P(1:2, 1:2), 'blue');
    
    % Plot landmark covs
    for i = 4:2:size(mu, 1)
        plot_cov_ellipse(mu(i:i+1), P(i:i+1, i:i+1), 'red');
    end
    
    % Plot scan coordinates
    scatter(coords(1, :), coords(2, :), 100, '.');  % 100 makes the dots large
    
    % Limits of the graph
    xlim([-5, 5])
    ylim([-5; 5])
end
