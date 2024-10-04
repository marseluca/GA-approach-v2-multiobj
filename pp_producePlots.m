function plots = pp_producePlots(trajectories,flag)
    
    global nRobots pathColors;
    
    if flag
        
        figure(3);
        
        [timeSteps,minDistances] = pp_getMinimumDistances(trajectories);
        
        plot(timeSteps, minDistances, '-','LineWidth',1.2);
        xlabel('Time Step');
        ylabel('Minimum Distance');
        title('Minimum Distance at Each Time Step');

        hold on
        plot(timeSteps,20*ones(1,length(timeSteps)));
        xlabel("t [s]")
        ylabel("$d(t)\:[m]$",'Interpreter','latex')
        title("Minimum distance between robots")
        legend("","Safety margin")
        grid
        hold off


        for j=1:nRobots

            figure
    
            subplot(2,2,1)
            sgtitle("Robot "+j)
    
            plot(trajectories{j}.t_tot,trajectories{j}.x_tot,'LineWidth',1.2,'Color',pathColors(j,:));
            grid
            xlabel("t [s]")
            ylabel("$x(t)\:[m]$",'Interpreter','latex')
    
    
            subplot(2,2,2)
            plot(trajectories{j}.t_tot,trajectories{j}.y_tot,'LineWidth',1.2,'Color',pathColors(j,:));
            grid
            xlabel("t [s]")
            ylabel("$y(t)\:[m]$",'Interpreter','latex')
   

            subplot(2,2,3)
            velocity_magnitude = sqrt(trajectories{j}.xdot_tot.^2 + trajectories{j}.ydot_tot.^2);
            plot(trajectories{j}.t_tot,velocity_magnitude,'LineWidth',1.2,'Color',pathColors(j,:));
            grid
            xlabel("t [s]")
            ylabel("$v(t)\:[m/s^]$",'Interpreter','latex')

            subplot(2,2,4)
            acc_magnitude = sqrt(trajectories{j}.xddot_tot.^2 + trajectories{j}.yddot_tot.^2);
            plot(trajectories{j}.t_tot,acc_magnitude,'LineWidth',1.2,'Color',pathColors(j,:));
            grid
            xlabel("t [s]")
            ylabel("$a(t)\:[m/s^2]$",'Interpreter','latex')

        end

    end
    
end

