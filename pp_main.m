%% TRAJECTORY COMPUTATION

clear all
close all
global nRobots samplingTime pathColors;

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
plotCollisions = true;

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

finishTimes = [];
for j=1:nRobots
    finishTimes = [finishTimes, trajectories{j}.t_tot(end)];
end
finishTimes


if plotCollisions
    pp_plotCollisions(collisions,trajectories);
end


%% COLLISIONS
if ~isempty(collisions) && solveCollisions

    global L maxVelocity collision_position absoluteCollidingTime;
    
    slowDownRobotIndex = 2;
    collidingSegment = collisions{slowDownRobotIndex}(1,3);
    collidingRobotSegments = pp_identifySegments(paths{slowDownRobotIndex},trajectories{slowDownRobotIndex});

    for i=1:length(collidingRobotSegments)
        if collidingRobotSegments(i)==collidingSegment
            segmentEntranceTime = trajectories{slowDownRobotIndex}.t_tot(i);
            break;
        end
    end
    
    absoluteCollidingTime = collisions{slowDownRobotIndex}(1,4) - segmentEntranceTime;
    
    % Colliding segment length
    segmentStartPosition = paths{slowDownRobotIndex}(collidingSegment,:);
    segmentEndPosition = paths{slowDownRobotIndex}(collidingSegment+1,:);
    L = norm(segmentStartPosition-segmentEndPosition);
    
    % Collision position in terms of length with respect to the start of the
    % segment
    collisionCoordinates = collisions{slowDownRobotIndex}(1,5:6);
    collision_position = norm(collisionCoordinates-segmentStartPosition);
    
    pp_collisionOptimization;
    
    % Add a new segment inside the colliding segment
    paths{slowDownRobotIndex} = pp_addNewSegment(paths{slowDownRobotIndex},collidingSegment,0,x_opt(1));

    % Now the new segment acquires the index of the colliding segment
    addedSegment = collidingSegment;

    % Apply the velocity scaling to the new segment
    trajectories{slowDownRobotIndex} = pp_interpolatePath(paths{slowDownRobotIndex},maxVelocity,addedSegment,x_opt(2));

    pp_plotPathOnMap(paths,trajectories,'-');
end

pp_plotPathOnMap(paths,trajectories,'-');

%% ANIMATION
if animation
    fprintf("\nPress enter to record animation with velocity %dx...\n",animVelocity);
    pp_animateTrajectory(trajectories,robotSize,recordAnimation,animVelocity);
end

% Plot positions, velocities and accelerations
pp_producePlots(trajectories,plotVelocities);
% pp_computeVelocityOverTime(trajectories{1}.x_tot,trajectories{1}.y_tot,trajectories{1}.t_tot)

figure(1)
saveas(gcf,'warehouse.png')




