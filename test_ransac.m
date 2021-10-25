% Remove all plots
clear;
close all;

% % Create a fake scan that is a straight line
% scan = -1:0.1:1;
% scan = [zeros(size(scan)); scan];
% scan(1, :) = (1 + 0.01 * randn(size(scan(1, :)))) ./ cos(scan(2, :));
% 
% % Convert to xy
% x = scan_to_xy(scan);

% Create a random assortment of lines
x = zeros(2, 70);
for i = 1:3
    dx = rand*0.2;
    dy = rand*0.2;
    xs = rand * 4 - 2;
    ys = rand * 4 - 2;
    for j = 1:20
        x(1, 20*(i-1)+j) = xs + dx * (j-1) + rand*0.03;
        x(2, 20*(i-1)+j) = ys + dy * (j-1) + rand*0.03;
    end
end

for i = 1:10
    x(1, 60+i) = rand * 6 - 3;
    x(2, 60+i) = rand * 6 - 3;
end
% Do ransac
N = 100; % Num tries
dist = 0.05; % distance threshold
L = 5;
n_thresh = 10;
lines = multi_line_ransac(x, N, dist, L, n_thresh);

hold on;
scatter(0, 0, '.'); % Robot position
scatter(x(1, :), x(2, :), 50, '.'); % Scan points
xlim([-4, 4]);
ylim([-4, 4]);

% Plot lines
lines
for i = 1:size(lines, 2)
    line = lines(:, i);
    x = -4:0.5:4;
    y = (line(1) * x + line(3))/-line(2);
    plot(x, y);
end

