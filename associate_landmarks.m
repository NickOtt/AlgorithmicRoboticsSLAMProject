% Determines if landmarks are new landmarks or old landmarks being refound

function [matched_indices, new_counts] = associate_landmarks(landmarks, detections, covs, counts, lambda)
    % Given a current selection of landmarks and how many times each has
    % been detected, along with position covariances for each,
    % associate new landmarks
    
    matched_indices = zeros(1, size(detections, 2));
    new_counts = counts;
    
    % For each landmark
    for i = 1:size(detections, 2)
        l = detections(:, i);
        
        % Find which it is closest to
        closest = closest_landmark(landmarks, l);
        
        % This occurs when there are currently no landmarks
        if closest == 0
            % All landmarks are new if there are no landmarks
            new_counts = [new_counts, 1];
            continue;
        end
        
        % Extract the covariance matrix for this landmark
        % covs are 2x2 on the diagonal, first 3x3 is for robot cov
        cov_i = 4+2*(closest-1);
        closest_cov = covs(cov_i:cov_i+1, cov_i:cov_i+1);
        closest_cov = closest_cov + covs(1:2, 1:2); % Add robot cov in
        if is_in_range(landmarks(:, closest), l, closest_cov, lambda)
            % If close to an existing landmark increment count by one
            % And add the matched index to array
            new_counts(closest) = counts(closest) + 1;
            matched_indices(i) = closest;
        else
            % Otherwise, it is new so add its count BUT leave the 
            % matched index as 0
            new_counts = [new_counts, 1];
        end
    end
end

function y = is_in_range(l_current, l_new, cov, lambda)
    % If a new observed landmark is some std devs close enough to
    % an existing one, it is considered an observation of the same one
    err = l_current - l_new; % Innovation is just the error term
       
    % Check if the new landmark is close enough
    y = (err.' * inv(cov) * err) <= lambda;
end

function index = closest_landmark(landmarks, l)
    % Returns the index of the landmark closest to a new, detected landmark
    min_dist = 1000;
    min_i = 0;
    for i = 1:size(landmarks, 2)
       dist = (landmarks(1,i) - l(1))^2 + (landmarks(2,i) - l(2))^2;
       if dist < min_dist
          min_dist = dist;
          min_i = i;
       end
    end
    index = min_i;
end