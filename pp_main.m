%% TRAJECTORY COMPUTATION

clc
clear all
close all
global nRobots samplingTime pathColors maxVelocity;

openfig("warehouse.fig");
paths = {};
paths = load("C:\Users\Luca\Desktop\Collaborative path\paths_registration.mat").paths_registration;

nRobots = size(paths,2);

for j=1:nRobots
    paths{j} = unique(paths{j},'rows','stable');
end

%% VARIABLES
robotSize = 20;
collisionThreshold = 20;
maxVelocity = 20;

%% OPTIONS
animation = true;
animVelocity = 7;
recordAnimation = true;
solveCollisions = true;
plotVelocities = true;
plotCollisions = false;

samplingTime = 0.1;

% Create random path colors
pathColors = distinguishable_colors(nRobots);


%% INTERPOLATION
trajectories = {};
for j=1:nRobots
    trajectories{j} = pp_interpolatePath(paths{j},maxVelocity,0,0);
end


%% COLLISION CHECKING
collisions = {};
for j=1:nRobots
    collisions{j} = pp_checkCollisionForOneRobot(paths,trajectories,collisionThreshold,j);
end


if plotCollisions
    pp_plotCollisions(collisions,trajectories);
end

finishTimes = [];
for j=1:nRobots
    finishTimes = [finishTimes, trajectories{j}.t_tot(end)];
end
finishTimes


%% COLLISIONS
if ~all(cellfun(@isempty,collisions)) && solveCollisions

    delta_s = 20;
    global x_opt;

    % Set ub and lb
    lb = [];
    ub = [];

    for i = 1:nRobots
        % Lower bounds for d_i, L_s, and alpha
        lb = [lb, 0, 0, 0.1];

        % Calculate the upper bound for L_s based on segments
        max_Ls = norm(paths{i}(2,:) - paths{i}(1,:)); % Length of the first segment

        % Upper bounds for d_i (binary), L_s (max segment length), and alpha
        ub = [ub, 1, max_Ls, 1];
    end

    % Set the integer constraints for d_i
    intcon = 1:3:(3*nRobots); % Indices for d_i

    % Set optimization options
    % Set the options for the genetic algorithm
    options_ga = optimoptions('gamultiobj', ...
    'Display', 'iter', ...
    'MaxGenerations', 1000, ...
    'PopulationSize', 100, ...
    'CrossoverFraction', 0.8, ...
    'PlotFcn', {@gaplotgenealogy, @gaplotscores}, ... % Use a different plot function suitable for multi-objective
    'MaxStallGenerations', 20); % If you want to use a similar concept for multi-objective

    % Call the ga solver
    
    objectiveFun = @(x) o_objective(x,paths);
    constraintFun = @(x) o_collision_constraint(x,paths,delta_s);

    [x_opt, fval] = gamultiobj(objectiveFun, 3*nRobots, [], [], [], [], lb, ub, constraintFun, intcon, options_ga);
    
    % x_opt = load("x_opt.mat").x_opt;

    % Add the optimized segments
    for j=1:nRobots

        newSegment = 1;
        d_j = x_opt(3*(j-1) + 1);
        L_s_j = x_opt(3*(j-1) + 2);
        alpha_j = x_opt(3*(j-1) + 3);

        if d_j==1
            paths{j} = pp_addNewSegment(paths{j},newSegment,0,L_s_j);
            trajectories{j} = pp_interpolatePath(paths{j},maxVelocity,newSegment,alpha_j);
        end
        
    end
end

pp_plotPathOnMap(paths,trajectories,'-');

if plotVelocities
    % Plot positions, velocities and accelerations
    pp_producePlots(trajectories,plotVelocities);
end

%% ANIMATION
if animation
    fprintf("\nPress enter to record animation with velocity %dx...\n",animVelocity);
    pp_animateTrajectory(trajectories,robotSize,recordAnimation,animVelocity);
end

% figure(1)
% saveas(gcf,'warehouse.png')




