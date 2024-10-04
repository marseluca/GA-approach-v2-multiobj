function [x_position,y_position] = pp_getRobotPosition(trajectory,distance)
        
    x = trajectory.x_tot;
    y = trajectory.y_tot;
    
    % Calculate the cumulative distance along the interpolated trajectory
    dx = diff(x); % Differences between consecutive interpolated x points
    dy = diff(y); % Differences between consecutive interpolated y points
    distances = sqrt(dx.^2 + dy.^2); % Distance between consecutive points
    cumulativeDistance = [0, cumsum(distances)]; % Cumulative distance from start
    
    % Interpolate the x and y coordinates based on the traveled distance using pchip
    x_position = interp1(cumulativeDistance, x, distance, 'pchip');
    y_position = interp1(cumulativeDistance, y, distance, 'pchip');

end

