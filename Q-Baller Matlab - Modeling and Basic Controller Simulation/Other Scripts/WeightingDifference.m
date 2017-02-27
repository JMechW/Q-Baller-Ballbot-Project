figure(1)
subplot(2,1,1)
    hold on;
    grid on;
    plot(Ballbot{1}.Timer.Point(1:end-1),fliplr(InputCline(1,:)),':r','LineWidth',2);
    plot(Ballbot{1}.Timer.Point(1:end-1),fliplr(InputCline(2,:)),'-.g','LineWidth',2);
    plot(Ballbot{1}.Timer.Point(1:end-1),fliplr(InputCline(3,:)),'--b','LineWidth',2);
    plot(Ballbot{1}.Timer.Point(1:end-1),fliplr(InputCline(4,:)),'.k','LineWidth',2);
    title('Difference of Weighted U0');
    xlabel('Time (s)')
    ylabel('Voltage (V)')
    legend('U++','U+-','U--','U-+','Location','northoutside','Orientation','horizontal')
    hold off;
    
subplot(2,1,2)

    hold on;
    grid on;
    plot(Ballbot{1}.Timer.Point(1:end-1),fliplr(InputCline(5,:)),'c','LineWidth',2);
    plot(Ballbot{1}.Timer.Point(1:end-1),fliplr(InputCline(6,:)),'m','LineWidth',2);    
    
    title('Controlled Velocity Point');
    xlabel('Time (s)')
    ylabel('Velocity (m/s)')
    legend('X Vel.','Y Vel.','Location','northoutside','Orientation','horizontal')
    hold off;