function [collisions, newTrajectories] = pp_getCollisionFreeTrajectories(paths,trajectories,maxVelocities,collisions,collisionThreshold)
    
    global nRobots;
    goalPoint = paths{1}(end,:);
    newTrajectories = trajectories;
    increasedTimes = zeros(1,nRobots);
    

    while ~all(cellfun(@isempty, collisions))

        maxDistance = -Inf;
        for i=1:size(collisions,2)
            for j=1:size(collisions{i},1)
                currentPoint = collisions{i}(j,5:6);
                currentDistance = norm(goalPoint-currentPoint);
                if currentDistance>maxDistance
                    maxDistance = currentDistance;
                    maxDistancePath = i;
                end
            end
        end
    
        segments = pp_identifySegments(paths{maxDistancePath},newTrajectories{maxDistancePath});
        segmentToIncrease = segments(end);

        timeToIncrease = increasedTimes(maxDistancePath)+1;
        increasedTimes(maxDistancePath) = timeToIncrease;
        

        newTrajectories{maxDistancePath} = pp_interpolatePath(paths{maxDistancePath},maxVelocities(maxDistancePath),segmentToIncrease,timeToIncrease);
        
        collisions = {};
        for j=1:nRobots
            collisions{j} = pp_checkCollisionForOneRobot(paths,newTrajectories,collisionThreshold,j);
        end

    end
   

end

