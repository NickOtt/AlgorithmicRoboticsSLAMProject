close all;
clear;

pos = [2; 3];
cov = [0.1, 0.05;
       0.05, 0.05];
   
lambda = 6;

xs = [0:0.01:5];
ys = [0:0.01:5];

data = zeros(size(xs, 2), size(ys, 2));
% data = [255, 0;
%         0, 255];

for i = 1:length(xs)
    for j = 1:length(ys)
        data(j, i) = is_in_range(pos, [xs(i); ys(j)], cov, lambda);
    end
end

   
hold on;
imagesc(xs, ys, data);
scatter(pos(1), pos(2), 'red');
plot_cov_ellipse(pos, cov, 'red');
xlim([0, 5]);
ylim([0, 5]);


function y = is_in_range(l_current, l_new, cov, lambda)
    % If a new observed landmark is some std devs close enough to
    % an existing one, it is considered an observation of the same one
    err = l_current - l_new; % Innovation is just the error term
       
    % Check if the new landmark is close enough
    y = (err.' * inv(cov) * err) <= lambda;
end
