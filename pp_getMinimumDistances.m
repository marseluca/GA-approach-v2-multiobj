function [timeSteps,minDistances] = pp_getMinimumDistances(trajectories)
        
        global nRobots;
        
        k = 1;
        for i=1:nRobots
            
            for j=i+1:nRobots
                % Get all the distances over time
                maxLength = min(length(trajectories{i}.x_tot),length(trajectories{j}.x_tot));
                distances{k} = sqrt((trajectories{i}.x_tot(1:maxLength)-trajectories{j}.x_tot(1:maxLength)).^2+(trajectories{i}.y_tot(1:maxLength)-trajectories{j}.y_tot(1:maxLength)).^2);
                k=k+1;
            end

        end

        % Step 1: Find the maximum length (i.e., the largest time step)
        maxTimeStep = max(cellfun(@length, distances));
        
        % Step 2: Initialize an array to store the minimum distance at each time step
        minDistances = inf(1, maxTimeStep); % Initialize with infinity
        
        % Step 3: Iterate over each time step
        for t = 1:maxTimeStep
            % Collect distances for the current time step from all vectors that have this time step
            distancesAtT = [];
            for i = 1:length(distances)
                if t <= length(distances{i}) % If the vector has a value for this time step
                    distancesAtT = [distancesAtT, distances{i}(t)];
                end
            end
            % Find the minimum distance at this time step
            if ~isempty(distancesAtT)
                minDistances(t) = min(distancesAtT);
            end
        end
        
        % Step 4: Plot the minimum distances
        
        % Find trajectory with the longest time samples
        maxSamples = -Inf;
        for j=1:nRobots
            if trajectories{j}.t_tot(end)>maxSamples
                maxSamples = trajectories{j}.t_tot(end);
                maxSamplesIndex = j;
            end
        end

        timeSteps = trajectories{maxSamplesIndex}.t_tot(1:maxTimeStep); % Time steps corresponding to the minimum distances
       
end

