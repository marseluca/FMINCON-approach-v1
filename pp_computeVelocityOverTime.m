function [outputArg1,outputArg2] = pp_computeVelocityOverTime(x,y,t)
    
    % Assuming you have the time vector t and position vectors x(t), y(t)
    % t: Time vector
    % x: X-position vector from pchip interpolation
    % y: Y-position vector from pchip interpolation
    
    % Calculate velocity using the gradient (more accurate than diff)
    v_x = gradient(x, t);  % Derivative of x with respect to time
    v_y = gradient(y, t);  % Derivative of y with respect to time
    
    % The magnitude of the velocity vector
    velocity_magnitude = sqrt(v_x.^2 + v_y.^2);
    
    % Plotting the velocity components and magnitude
    figure;
    subplot(3,1,1);
    plot(t, v_x);
    title('Velocity in X direction');
    xlabel('Time');
    ylabel('v_x');
    
    subplot(3,1,2);
    plot(t, v_y);
    title('Velocity in Y direction');
    xlabel('Time');
    ylabel('v_y');
    
    subplot(3,1,3);
    plot(t, velocity_magnitude);
    title('Velocity Magnitude');
    xlabel('Time');
    ylabel('Velocity Magnitude');

end

