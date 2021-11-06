% Plots the robot pose, landmarks, covariance ellipses, and the laser scan
% As well as the map and landmark positions


function plot_world(mu, P, scan, map, lines, real_poses, filtered_poses, commanded_poses, detections)
    hold on;
    
    cmap = hsv(30); % 15 color choices

    % Plot map in world coords
    show(map)
    
    % Plot actual position (and angle)
    real_pos = real_poses(:, end);
    scatter(real_pos(1), real_pos(2), 100, 'green', 'filled');
    plot([real_pos(1), real_pos(1) + 0.5*cos(real_pos(3))], [real_pos(2), real_pos(2) + 0.5*sin(real_pos(3))], 'green');
    
    % Plot robot position (and angle)
    scatter(mu(1), mu(2), 50, 'blue', 'filled');
    plot([mu(1), mu(1) + 0.5*cos(mu(3))], [mu(2), mu(2) + 0.5*sin(mu(3))], 'blue');
    
    % Extract and plot landmarks
    cmap(max((length(mu)-3)/2,1), :);
    scatter(mu(4:2:end-1), mu(5:2:end), 50, cmap(1:(length(mu)-3)/2, :), 'filled');
    
    % Plot Detections
    if ~isempty(detections)
        scatter(detections(1,:), detections(2,:), 50, [0.1, 0.1, 0.1], 'filled');
    end
    
    % Plot robot cov
    plot_cov_ellipse(mu(1:2), P(1:2, 1:2), 'blue');
    
    % Plot landmark covs
    for i = 4:2:size(mu, 1)
        plot_cov_ellipse(mu(i:i+1), P(i:i+1, i:i+1), cmap((i-4)/2+1, :));
    end
    
    % Plot scan coordinates
    coords = scan_to_xy(scan(1, :), scan(2, :), mu);
    scatter(coords(1, :), coords(2, :), 100, '.');  % 100 makes the dots large
    
    % Plot lines
    for i = 1:size(lines, 2)
        line = lines(:, i);
        x = 0:5:10;
        y = (line(1) * x + line(3))/-line(2);
        plot(x, y);
    end
    
    % Plot history of real and filtered poses
    plot(real_poses(1, :), real_poses(2, :),'Color','green', 'LineStyle','--','Marker','o')
    plot(filtered_poses(1, :), filtered_poses(2, :),'Color','blue', 'LineStyle','--','Marker','o')
    plot(commanded_poses(1, :), commanded_poses(2, :),'Color','red', 'LineStyle','--','Marker','o')

    % Limits of the graph
    xlim([0, 8])
    ylim([0; 8])
end
