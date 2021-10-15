% Remove all plots
clear all;
close all;

% Create a fake scan that is a straight line
scan = -1:0.05:1;
scan = [zeros(size(scan)); scan];
scan(1, :) = 1 ./ cos(scan(2, :));

% Convert to xy
x = scan_to_xy(scan);

hold on;
scatter(0, 0, '.'); % Robot position
scatter(x(1, :), x(2, :), '.'); % xy coords
xlim([-2, 2]);
ylim([-2, 2]);


function points = scan_to_xy(scan)
    % Given scan points in [dist; theta], convert to [x; y]
    points = zeros(size(scan));
    
    for i = 1:size(scan, 2)
        points(1, i) = scan(1, i) * cos(scan(2, i));
        points(2, i) = scan(1, i) * sin(scan(2, i));
    end
end
