function animation = pp_animateTrajectory(trajectories,robotSize,recordAnimation,animVelocity)

    global pathColors;
    global samplingTime;
    global nRobots;


    maxTime = -inf;  
    for j=1:nRobots
        maxTime = max(maxTime, max(trajectories{j}.t_tot));
    end
    
    commonTimeVector = 0:samplingTime:maxTime;

    lengths = [];
    for j=1:nRobots
        lengths = [lengths; length(trajectories{j}.x_tot)];
    end
    maxLength = max(lengths);

    for j=1:nRobots
        trajectories{j}.x_tot(end+1:maxLength) = NaN;
    end

    %% VIDEO  
    if recordAnimation==true
        video_name = 'warehouse';  % Define video file name
        v = VideoWriter(video_name, 'MPEG-4');  % Create video writer object
        v.FrameRate = round(1/samplingTime)*animVelocity;  % Set frame rate
        open(v);
    end
    %%

    hold on;

    robotPlots = {};
    for j=1:nRobots
        % NOTE: "MarkerSize" does not represent
        % The real dimension of the robot!
        % To do it, you must use the rectangle function
        % Which uses the syntax: [x y width height]

        robotPlots{j} = rectangle('Position',[trajectories{j}.x_tot(1)-(robotSize/2), trajectories{j}.y_tot(1)-(robotSize/2), robotSize, robotSize],'Curvature',[1 1],'FaceColor',pathColors(j,:));
        
    end


    pause

    for k = 1:length(commonTimeVector)

        for i=1:nRobots
            % Calculate new position
            % You check whether the interpolated value is NaN first
            % because when interpolating different trajectories to the same
            % time vector you get NaNs
            if  ~isnan(trajectories{i}.x_tot(k))

                newPos = [trajectories{i}.x_tot(k)-(robotSize/2), trajectories{i}.y_tot(k)-(robotSize/2), robotSize, robotSize];
                
                set(robotPlots{i}, 'Position', newPos);
            end
            % Update the position of the rectangle

            % set(robotPlots{i}, 'XData', x_tot_interpolated{i}(k), 'YData', y_tot_interpolated{i}(k));
        end
        
        % drawnow;
        pause(samplingTime/animVelocity); % Adjust the speed of the animation
        
        tk = sprintf('%.2f', commonTimeVector(k));
        title("Animation speed: "+animVelocity+"x, t = "+tk)

        if recordAnimation==true
            figure(1)
            frame = getframe(gcf);  % Capture the figure as a frame
            % Write the frame to the video
            writeVideo(v, frame);
        end

        if commonTimeVector(k)>350
            break;
        end

    end

    if recordAnimation==true
        close(v);
        disp(['Video saved as ' video_name '.mp4']);
    else
        disp('Video recording was off');
    end

end

