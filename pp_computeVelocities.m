function velocities = pp_computeVelocities(points,k1,k2,vmax)
    
    lengths = [];
    velocities = [0];
    vmin = 1; % The minimum velocity for the robot along the trajectory
              % So that the robot never stops
    lengthWeight = 1;
    angleWeight = 1-lengthWeight;

    currentLengthWeight = 0.2;
    nextLengthWeight = 1-currentLengthWeight;

    % First you need to compute all the segment lengths
    % The lengths refer to the segment lengths
    % Between the previous and the current point
    % You don't care about the last length
    % Because the final velocity will always be zero

    for j=2:size(points,1)-1
        currLength = norm(points(j,:)-points(j-1,:));
        lengths = [lengths, currLength];
    end

    minLength = min(lengths);
    maxLength = max(lengths);

    for j=2:size(points,1)-1

        P1 = points(j-1,:);
        P2 = points(j,:);
        P3 = points(j+1,:);

        alpha = pp_computeAngle(P1,P2,P3);
        
        velocityAngle = k1./(k2+alpha);
        
        currLength = lengths(j-1);

        if j<size(points,1)-1
            nextLength = lengths(j);
        end

        currentVelocityLength = vmin + (vmax-vmin)*((currLength-minLength)/(maxLength-minLength))^2;

        if j==size(points,1)-1
            nextVelocityLength = 0;
        else
            nextVelocityLength = vmin + (vmax-vmin)*((nextLength-minLength)/(maxLength-minLength))^2;
        end
        
        if nextLength<currLength
            velocityLength = currentLengthWeight*currentVelocityLength + nextLengthWeight*nextVelocityLength;
            velocityLength = nextVelocityLength*currLength/nextLength;
        else
            velocityLength = currentVelocityLength;
        end

        v = lengthWeight*velocityLength + angleWeight*velocityAngle;

        velocities = [velocities, v];

    end

    velocities = [velocities, 0];


    % plot the chart
    % figure
    % alpha_vector = 0:1:180;
    % vels = k1./(k2+alpha_vector);
    % % vels = 20-exp((1/180/(log(2.718)/log(20)))*alpha_vector);
    % plot(alpha_vector,vels,'LineWidth',1.2);
    % grid
    % xlabel("Angle between three points [deg]")
    % ylabel("Velocity [m/s]")
    % title("Angle-velocity chart")

end

