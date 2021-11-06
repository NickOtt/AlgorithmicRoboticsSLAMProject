function points = scan_to_xy(dist, angles, mu)
    % Given scan points in [dist], [theta] column arrays, convert to [x; y]
    points = zeros(2, length(dist));
    
    x = mu(1); y = mu(2); theta = mu(3);

    % Convert scan to x, y, taking into account robot position
    % This transformation could also be done on the resulting lines,
    % which would be more computantionally effecient. 
    for i = 1:length(dist)
        points(1, i) = dist(i) * cos(angles(i) + theta) + x;
        points(2, i) = dist(i) * sin(angles(i) + theta) + y;
    end
end