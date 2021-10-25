function points = scan_to_xy(scan)
    % Given scan points in [dist; theta], convert to [x; y]
    points = zeros(size(scan));
    
    for i = 1:size(scan, 2)
        points(1, i) = scan(1, i) * cos(scan(2, i));
        points(2, i) = scan(1, i) * sin(scan(2, i));
    end
end