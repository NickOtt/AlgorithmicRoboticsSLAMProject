% Plots the robot pose, landmarks, covariance ellipses, and the laser scan
% As well as the map and landmark positions


function plot_world(curPose, mu, P, coords, map, lines)
    hold on;
    cmap = hsv(15); % 15 color choices

    % Plot map in world coords
    show(map)
    
    % Plot actual position
    scatter(curPose(1), curPose(2), 50, 'green', 'filled');
    
    % Plot robot position
    scatter(mu(1), mu(2), 50, 'blue', 'filled');
    
    % Extract and plot landmarks
    cmap(max((length(mu)-3)/2,1), :);
    scatter(mu(4:2:end-1), mu(5:2:end), 50, cmap(1:(length(mu)-3)/2, :), 'filled');
    
    % Plot robot cov
    plot_cov_ellipse(mu(1:2), P(1:2, 1:2), 'blue');
    
    % Plot landmark covs
    for i = 4:2:size(mu, 1)
        plot_cov_ellipse(mu(i:i+1), P(i:i+1, i:i+1), cmap((i-4)/2+1, :));
    end
    
    % Plot scan coordinates
    scatter(coords(1, :), coords(2, :), 100, '.');  % 100 makes the dots large
    
    % Plot lines
    for i = 1:size(lines, 2)
        line = lines(:, i);
        x = 0:5:10;
        y = (line(1) * x + line(3))/-line(2);
        plot(x, y);
    end
    
    % Limits of the graph
    xlim([0, 10])
    ylim([0; 10])
end
