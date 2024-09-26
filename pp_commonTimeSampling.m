function interpolatedTrajectory = pp_commonTimeSampling(trajectory)
    
    global samplingTime;
    interpolatedTrajectory = trajectory;
    timeVector = 0:samplingTime:trajectory.t_tot(end);
    
    interpolatedTrajectory.x_tot = interp1(trajectory.t_tot,trajectory.x_tot,timeVector);
    interpolatedTrajectory.y_tot = interp1(trajectory.t_tot,trajectory.y_tot,timeVector);
    interpolatedTrajectory.xdot_tot = interp1(trajectory.t_tot,trajectory.xdot_tot,timeVector);
    interpolatedTrajectory.ydot_tot = interp1(trajectory.t_tot,trajectory.ydot_tot,timeVector);
    interpolatedTrajectory.xddot_tot = interp1(trajectory.t_tot,trajectory.xddot_tot,timeVector);
    interpolatedTrajectory.yddot_tot = interp1(trajectory.t_tot,trajectory.yddot_tot,timeVector);
    interpolatedTrajectory.t_tot = timeVector;
end

