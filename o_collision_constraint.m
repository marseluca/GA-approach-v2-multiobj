function [c, ceq] = o_collision_constraint(x,paths,delta_s)
    
    global nRobots maxVelocity;
    c = [];
    ceq = [];
    numberOfConstraints = 10000;

    % for i = 1:nRobots
    %     d_i = x(3*(i-1) + 1); % Decision variable for robot i
    %     L_s = x(3*(i-1) + 2);
    %     alpha = x(3*(i-1) + 3);
    % 
    %     path_i = paths{i};
    %     if d_i == 1
    %         newSegment = 1;
    %         path_i = pp_addNewSegment(path_i, newSegment, 0, L_s);
    %         path_i = unique(path_i, 'rows', 'stable');
    %         traj_i = pp_interpolatePath(path_i, maxVelocity, newSegment, alpha);
    %     else
    %         traj_i = pp_interpolatePath(path_i, maxVelocity, 0, 0);
    %     end
    % 
    %     for j = i+1:nRobots
    %         % Repeat for robot j
    %         d_j = x(3*(j-1) + 1);
    %         L_s_j = x(3*(j-1) + 2);
    %         alpha_j = x(3*(j-1) + 3);
    % 
    %         path_j = paths{j};
    %         if d_j == 1
    %             newSegment = 1;
    %             path_j = pp_addNewSegment(path_j, newSegment, 0, L_s_j);
    %             path_j = unique(path_j, 'rows', 'stable');
    %             traj_j = pp_interpolatePath(path_j, maxVelocity, newSegment, alpha_j);
    %         else
    %             traj_j = pp_interpolatePath(path_j, maxVelocity, 0, 0);
    %         end
    % 
    %         % Add collision constraints for robots i and j
    %         minLength = min(length(traj_i.x_tot), length(traj_j.x_tot));
    %         for k = 1:minLength
    %             pos_i = [traj_i.x_tot(k), traj_i.y_tot(k)];
    %             pos_j = [traj_j.x_tot(k), traj_j.y_tot(k)];
    %             c = [c; delta_s - norm(pos_i - pos_j)];
    %         end
    %     end
    % 
    % end

    for i = 1:nRobots
        d_i = x(3*(i-1) + 1); % Decision variable for robot i
        L_s = x(3*(i-1) + 2); % Length of slow-down segment for robot i
        alpha = x(3*(i-1) + 3); % Velocity scaling factor for robot i

        % Get the path for robot i
        path = paths{i};
        if d_i == 1
            newSegment = 1;
            path = pp_addNewSegment(path, newSegment, 0, L_s);
            path = unique(path, 'rows', 'stable');
            trajectories{i} = pp_interpolatePath(path, maxVelocity, newSegment, alpha);
        else
            trajectories{i} = pp_interpolatePath(path, maxVelocity, 0, 0);
        end        
    end

    [~,minDistances] = pp_getMinimumDistances(trajectories);
    c = delta_s - min(minDistances);

    % if length(c)<numberOfConstraints
    %     c(end+1:numberOfConstraints) = 0;
    % end

end