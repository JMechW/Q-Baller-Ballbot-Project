colormat={':r','-.g','--b','*k','c'};
legendmat={'100%e','75%e','50%e','25%e','0%e'};
figure(1)
subplot(1,2,1)
hold on
grid on
for ii=1:5
plot(Ballbot.Timer.Point,recstate{ii}(1,:),colormat{ii},'LineWidth',2);
end
title('State X - Velocity Control Comparison')
ylabel('Translational Velocity (m)');
xlabel('Time (s)')
legend(legendmat{1},legendmat{2},legendmat{3},legendmat{4},legendmat{5})
hold off
subplot(1,2,2)
hold on
grid on
for ii=1:5
plot(Ballbot.Timer.Point,recstate{ii}(2,:),colormat{ii},'LineWidth',2);
end
title('State Y - Velocity Control Comparison')
ylabel('Translational Velocity (m)');
xlabel('Time (s)')
legend(legendmat{1},legendmat{2},legendmat{3},legendmat{4},legendmat{5})
hold off
