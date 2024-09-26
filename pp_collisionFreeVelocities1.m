function optimizedVector = pp_collisionFreeVelocities1(paths,collisionThreshold,maxVelocity)

    %% SOLVE COLLISIONS
    % velStep is the reduction step for the velocities
    % for example, if the max velocity is 20, the algorithm
    % will try to reduce the velocity by 2 every time
    % so that it will try different combinations for 20, 18, 16 m/s and so on
    global nRobots;

    velStep = 2;
    velocityCombinations = pp_velocitiesCombinations(nRobots,maxVelocity,velStep);
    collisionFreeVelocityCombinations = [];

    for i = 1:size(velocityCombinations,1)

        fprintf("Combination #%d\n",i);

        % Compute the current combination for the max velocities
        for j=1:nRobots
            vmax = velocityCombinations(i,j);
            trajectories{j} = pp_interpolatePath(paths,j,vmax);
        end

        % Check if there are collisions with the current combination
        collisions = {};
        for k=1:nRobots
            collisions{k} = pp_checkCollisionForOneRobot(trajectories,collisionThreshold,k);
        end

        if all(cellfun(@isempty, collisions))
            disp("No collisions for this combination!!!!!!!!!!!!!!!");
            collisionFreeVelocityCombinations = [collisionFreeVelocityCombinations; velocityCombinations(i,:)];
        end

        % if i>1000
        %     break;
        % end

    end

    fprintf("Optimizing velocities...");
    targetVelocity = maxVelocity;
    optimizedVector = pp_optimizeVelocities(collisionFreeVelocityCombinations,1,1,targetVelocity);

    fprintf("Optimized combination: [");
    fprintf("%d ",optimizedVector);
    fprintf("]");
end

