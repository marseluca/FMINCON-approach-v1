function plots = pp_producePlots(trajectories,flag)
    
    global nRobots pathColors;
    
    if flag

        figure(2)
        maxLength = min(length(trajectories{1}.x_tot),length(trajectories{2}.x_tot));
        distances = sqrt((trajectories{1}.x_tot(1:maxLength)-trajectories{2}.x_tot(1:maxLength)).^2+(trajectories{1}.y_tot(1:maxLength)-trajectories{2}.y_tot(1:maxLength)).^2);
        plot(trajectories{1}.t_tot(1:maxLength),distances,'LineWidth',1.2);
        hold on
        plot(trajectories{1}.t_tot(1:maxLength),20*ones(1,maxLength));
        grid
        xlabel("t [s]")
        ylabel("$d(t)\:[m]$",'Interpreter','latex')
        title("Distance between R1 and R2 over time")
        hold off

        for j=1:nRobots

            figure(j+2)
    
            subplot(3,2,1)
            sgtitle("Robot "+j)
    
            plot(trajectories{j}.t_tot,trajectories{j}.x_tot,'LineWidth',1.2,'Color',pathColors(j,:));
            grid
            xlabel("t [s]")
            ylabel("$x(t)\:[m]$",'Interpreter','latex')
    
    
            subplot(3,2,2)
            plot(trajectories{j}.t_tot,trajectories{j}.y_tot,'LineWidth',1.2,'Color',pathColors(j,:));
            grid
            xlabel("t [s]")
            ylabel("$y(t)\:[m]$",'Interpreter','latex')
    
    
            subplot(3,2,3)
            plot(trajectories{j}.t_tot,trajectories{j}.xdot_tot,'LineWidth',1.2,'Color',pathColors(j,:));
            grid
            xlabel("t [s]")
            ylabel("$\dot{x}(t)\:[m/s]$",'Interpreter','latex')
    
    
            subplot(3,2,4)
            plot(trajectories{j}.t_tot,trajectories{j}.ydot_tot,'LineWidth',1.2,'Color',pathColors(j,:));
            grid
            xlabel("t [s]")
            ylabel("$\dot{y}(t)\:[m/s]$",'Interpreter','latex')
    
    
            subplot(3,2,5)
            plot(trajectories{j}.t_tot,trajectories{j}.xddot_tot,'LineWidth',1.2,'Color',pathColors(j,:));
            grid
            xlabel("t [s]")
            ylabel("$\ddot{x}(t)\:[m/s^2]$",'Interpreter','latex')
    
    
            subplot(3,2,6)
            plot(trajectories{j}.t_tot,trajectories{j}.yddot_tot,'LineWidth',1.2,'Color',pathColors(j,:));
            grid
            xlabel("t [s]")
            ylabel("$\ddot{y}(t)\:[m/s^2]$",'Interpreter','latex')

        end

    end
    
end

