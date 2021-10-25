function p = closest_point_on_line(line, point)
    % Finds the closest point on a line to the start point
    % From https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
    a = line(1); b = line(2); c = line(3);
    x0 = point(1); y0 = point(2);
    x = (b*(b*x0 - a*y0) - a*c) / (a^2 + b^2);
    y = (a*(-b*x0 + a*y0) - b*c) / (a^2 + b^2);
    p = [x; y];
end