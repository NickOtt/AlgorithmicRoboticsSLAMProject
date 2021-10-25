
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