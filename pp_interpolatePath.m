function trajectory = pp_interpolatePath(path,vmax,segmentToIncrease,scalingFactor)
    
    numberOfSamples = 1000;
    interpolatedPoints = {};
    trajectory = struct();
    hold on

    x = path(:,1);
    y = path(:,2);
    
    segment = 1;

    lengths = sqrt(diff(x).^2 + diff(y).^2);
    segment_times = lengths / vmax;

    % Increase segment time
    if segmentToIncrease~=0
        segment_times(segmentToIncrease) = lengths(segmentToIncrease) / (scalingFactor*vmax);
    end
    
    t = [0, cumsum(segment_times)'];

    spline_x = pchip(t,x);
    spline_y = pchip(t,y);
    
    trajectory.t_tot = linspace(t(1), t(end), numberOfSamples);
    
    % Evaluate the splines
    trajectory.x_tot = ppval(spline_x, trajectory.t_tot);
    trajectory.y_tot = ppval(spline_y, trajectory.t_tot);


    % Velocity
    n = spline_x.order;

    % Manual derivation for velocity
    spline_x_dot = spline_x;
    spline_x_dot.order = n-1;
    for i = 1:3
        spline_x_dot.coefs(:,i) = spline_x.coefs(:,i)*(n-i);
    end

    trajectory.xdot_tot = ppval(spline_x_dot, trajectory.t_tot);

    spline_y_dot = spline_y;
    spline_y_dot.order = n-1;
    for i = 1:3
        spline_y_dot.coefs(:,i) = spline_y.coefs(:,i)*(n-i);
    end

    trajectory.ydot_tot = ppval(spline_y_dot, trajectory.t_tot);


    % Manual derivation for acceleration
    spline_x_ddot = spline_x;
    spline_x_ddot.order = n-2;
    for i = 1:3
        spline_x_ddot.coefs(:,i) = spline_x.coefs(:,i)*(n-1-i);
    end

    trajectory.xddot_tot = ppval(spline_x_ddot, trajectory.t_tot);

    spline_y_ddot = spline_y;
    spline_y_ddot.order = n-2;
    for i = 1:3
        spline_y_ddot.coefs(:,i) = spline_y.coefs(:,i)*(n-1-i);
    end

    trajectory.yddot_tot = ppval(spline_y_ddot, trajectory.t_tot);

    trajectory = pp_commonTimeSampling(trajectory);
    
end

