function lines = multi_line_ransac(points, N, dist, L, n_thresh)
    % Uses ransac to find many lines in a laser scan
    % points: Input scan points as xy coords
    % N: Number of times for ransac to try to find one line
    % dist: dist below which point is considered an inlier
    % L: Maximum numbers of lines to find
    
    % Store results
    lines = zeros(3, 0);
    
    % For number of lines
    for i = 1:L
        [line, n, inliers] = run_ransac(points, N, dist);
        
        % Make sure this is a good match
        if n > n_thresh
            % Add to list
            lines = [lines, line];
            
            % Remove inliers and go again
            points = points(:, ~logical(inliers));
        end
    end
end

function [line, n, inliers] = run_ransac(points, N, dist)
    % Uses ransac to extract a line from a laserscan
    % points: Input points, ordered in laserscan order
    % N: Number of times to try
    % dist: max dist from point to line to be considered an insider
    
    best_line = [0; 0; 0];  % Coefficients of the best model so far
    best_n = 0; % Best number of inliers so far
    best_inliers = [];
    
    % Try N times
    for i = 1:N
        % Step 1: Pick two random points
        % TODO: What if these are the same
        % TODO: Pick indices close to each other because points are ordered
        % in a laserscan
        index1 = randi(size(points, 2));
        index2 = randi(size(points, 2));
        p1 = points(:, index1);
        p2 = points(:, index2);
        
        % Step 2: Find line model for these points
        line = fit_line(p1, p2);
        
        % Step 3: Find number of inliers
        [n, inliers] = count_inliers(points, line, dist);
        if n > best_n
            best_n = n;
            best_line = line;
            best_inliers = inliers;
        end
    end
    
    % Return best results
    n = best_n;
    line = best_line;
    inliers = best_inliers;
end

function line = fit_line(p1, p2)
   % Given two points, returns coefficient of line as ax + by + c = 0
   a = p1(2) - p2(2);
   b = p1(1) - p2(1);
   c = -(a*p1(1) + b*p1(2));
   
   line = [a; b; c];
end

function [n, inliers] = count_inliers(points, line, dist)
    % Counts the number of points less than dist from a line
    % given by ax+by+c = 0
    
    n = 0;
    inliers = zeros(1, size(points, 2));
    for i = 1:size(points, 2)
       p = points(:, i);
       % From en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
       d = abs(line(1)*p(1) + line(2)*p(2) + line(3)) / sqrt(line(1)^2 + line(2)^2);
       if d < dist
           n = n + 1;
           inliers(i) = 1;
       end
    end
end