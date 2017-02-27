function DrawPlot(Ballbot)
    %suptitle('Q-Baller Experiment 1')
    subplot(2,3,1)
    hold on;
    grid on;
    plot(Ballbot.Timer.Point,Ballbot.TrueState.Pos(1,:),':r','LineWidth',2);
    plot(Ballbot.Timer.Point,Ballbot.TrueState.Pos(2,:),'-.g','LineWidth',2);
    plot(Ballbot.Timer.Point,Ballbot.TrueState.Pos(3,:),'--b','LineWidth',2);
    title('State A, B, C - Position');
    xlabel('Time (s)')
    ylabel('Rotary Postion (rad)')
    legend('A','B','C','Location','northoutside','Orientation','horizontal')
    hold off;

    subplot(2,3,2)
    hold on;
    grid on;
    plot(Ballbot.Timer.Point,Ballbot.TrueState.Vel(1,:),':r','LineWidth',2);
    plot(Ballbot.Timer.Point,Ballbot.TrueState.Vel(2,:),'-.g','LineWidth',2);
    plot(Ballbot.Timer.Point,Ballbot.TrueState.Vel(3,:),'--b','LineWidth',2);
    title('State A, B, C - Velocity');
    xlabel('Time (s)')
    ylabel('Rotary Velocity (rad/s)')
    legend('A','B','C','Location','northoutside','Orientation','horizontal')
    hold off;


    subplot(2,3,3)
    hold on;
    grid on;
    plot(Ballbot.Timer.Point,Ballbot.TrueState.Pos(4,:),':r','LineWidth',2);
    plot(Ballbot.Timer.Point,Ballbot.TrueState.Pos(5,:),'-.g','LineWidth',2);
    title('State X, Y - Position');
    xlabel('Time (s)')
    ylabel('Translational Position (m)')
    legend('X','Y','Location','northoutside','Orientation','horizontal')
    hold off;

    subplot(2,3,4)
    hold on;
    grid on;
    plot(Ballbot.Timer.Point,Ballbot.TrueState.Vel(4,:),':r','LineWidth',2);
    plot(Ballbot.Timer.Point,Ballbot.TrueState.Vel(5,:),'-.g','LineWidth',2);
    title('State X, Y - Velocity');
    xlabel('Time (s)')
    ylabel('Translational Velocity (m/s)')
    legend('X','Y','Location','northoutside','Orientation','horizontal')
    hold off;

    subplot(2,3,5)
    hold on;
    grid on;
    plot(Ballbot.Timer.Point,Ballbot.Input(1,:),':r','LineWidth',2);
    plot(Ballbot.Timer.Point,Ballbot.Input(2,:),'-.g','LineWidth',2);
    plot(Ballbot.Timer.Point,Ballbot.Input(3,:),'--b','LineWidth',2);
    plot(Ballbot.Timer.Point,Ballbot.Input(4,:),'k','LineWidth',2);
    title('System Input - Voltage');
    xlabel('Time (s)')
    ylabel('Voltage (V)')
    legend('U++','U+-','U--','U-+','Location','northoutside','Orientation','horizontal')
    hold off;

    subplot(2,3,6)
    hold on;
    grid on;
    plot(Ballbot.Timer.Point,Ballbot.ObjectElement{1}.Position(1,:),':r','LineWidth',2);
    plot(Ballbot.Timer.Point,Ballbot.ObjectElement{1}.Position(2,:),'-.g','LineWidth',2);
    title('Ground Frame X, Y - Trajectory');
    xlabel('Time (s)')
    ylabel('Translational Position (m)')
    legend('X','Y','Location','northoutside','Orientation','horizontal')
    hold off;
end