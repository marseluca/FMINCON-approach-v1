function segments = pp_identifySegments(path,trajectory)
    
    % This function returns the segment
    % Where the robot is at each time step
    
    segments = [];
    currentSegment = 1;

    for i=1:length(trajectory.x_tot)
        segments(i) = currentSegment;

        currentTrajectoryPoint = [trajectory.x_tot(i), trajectory.y_tot(i)];
        currentPathPoint = path(min(length(path),currentSegment+1),:);

        if norm(currentTrajectoryPoint-currentPathPoint)<0.5
            currentSegment = currentSegment + 1;
        end
    end


end

