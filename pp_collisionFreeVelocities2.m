function optimizedVector = pp_collisionFreeVelocities2(paths,collisionThreshold,maxVelocity,maxVelocities)

    %% SOLVE COLLISIONS

    global nRobots;

    % Plan the path based on the distance of the starting point of the
    % trajectories to the goal point. The further starting points are given
    % the priority in planning the path, the closest ones are scaled in
    % velocity
    goalPoint = paths{1}(end,:);
    distances = [];
    optimizedVector = [maxVelocity];

    for j=1:nRobots

        % Calculate paths' distances
        totalDistance = 0;
        for i=2:size(paths{j},1)
            segmentDistance = norm(paths{j}(i-1,:)-paths{j}(i,:));
            totalDistance = totalDistance + segmentDistance;
        end

        distances = [distances; totalDistance, j];

    end
    
    orderedDistances = sortrows(distances,1,'descend');

    % Reorder the robots based on the distances
    orderedPaths = {};
    for j=1:nRobots
        orderedPaths{j} = paths{orderedDistances(j,2)};
    end

    orderedMaxVelocities = zeros(1,nRobots);
    for j=1:nRobots
        orderedMaxVelocities(j) = maxVelocities(orderedDistances(j,2))
    end

    % Set the max velocity of the first trajectory to the maximum possible
    vmax = maxVelocity;
    trajectories = {};
    trajectories{1} = pp_interpolatePath(orderedPaths,1,vmax);

    % Compute the max velocity of the other trajectories to avoid collision
    decreaseFactor = 0.1;
    for j=2:nRobots

        % Try decreasimg the velocity every time by a decreaseFactor
        for vmax=orderedMaxVelocities(j):-decreaseFactor:decreaseFactor

            fprintf("\nRobot %d: Trying with %d\n",j,vmax);
            
            trajectories{j} = pp_interpolatePath(orderedPaths,j,vmax);

            collisions = pp_checkCollisionForOneRobot(trajectories,collisionThreshold,j);

            if isempty(collisions)
                fprintf("\nNo collision velocity found!!!!\n")
                optimizedVector = [optimizedVector, vmax];
                break;
            end
        end
    end

    % Reorder the velocities vector
    reorderedOptimizedVector = zeros(1,nRobots);
    for i=1:nRobots
        j = orderedDistances(i,2);
        reorderedOptimizedVector(j) = optimizedVector(i);
    end
    optimizedVector = reorderedOptimizedVector;
    
    fprintf("\n\nMax velocities combination: [");
    fprintf("%d ",maxVelocities);
    fprintf("]\n");
    fprintf("Optimized combination: [");
    fprintf("%d ",optimizedVector);
    fprintf("]");

