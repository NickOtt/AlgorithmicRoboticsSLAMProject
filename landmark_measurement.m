% Given a landmark and robot position, returns [dist; theta]
function z = landmark_measurement(landmark, pose)
    dist = sqrt((landmark(1) - pose(1))^2 + (landmark(2) - pose(2))^2);
    heading = atan2(landmark(2) - pose(2), landmark(1) - pose(1)) - pose(3);
    
    z = [dist; heading];
end