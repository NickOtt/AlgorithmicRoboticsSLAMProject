function points = scan_to_xy(scan, mu)
    % Given scan points in [dist; theta], convert to [x; y]
    points = zeros(size(scan));
    
    x = mu(1); y = mu(2); theta = mu(3);
    
    % Convert scan to x, y, taking into account robot position
    % This transformation could also be done on the resulting lines,
    % which would be more computantionally effecient. 
    for i = 1:size(scan, 2)
        points(1, i) = scan(1, i) * cos(scan(2, i) + theta) + x;
        points(2, i) = scan(1, i) * sin(scan(2, i) + theta) + y;
    end
end