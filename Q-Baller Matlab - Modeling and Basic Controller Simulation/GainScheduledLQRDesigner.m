clear all
close all
addpath(genpath(pwd))
load('StandardModelZeroPointGain.mat','KStdZero'); %Gain at Zero Point
load([pwd '\StandardModelSSMatrixAFcn.mat'],'SSMAFcn'); %Load State Space Matrix A 12x12
load([pwd '\StandardModelSSMatrixBFcn.mat'],'SSMBFcn'); %Load State Space Matrix B 12x4

LQR_Q=diag([100,100,5000,20,20,50,50,3000,10,10]); %Q Matrix
LQR_R=diag([50,50,50,50]); %R Matrix

LQRController.TierContent{1}.Gain=KStdZero;
LQRController.TierContent{1}.Velocity=zeros(2,1);
LQRController.TierContent{1}.State=zeros(12,1);
LQRController.TierContent{1}.Input=zeros(4,1);
LQRController.TierContent{1}.Angle=0;
LQRController.CompleteScheduleTier=1;
LQRController.Current.Tier=1;
LQRController.Current.Num=1;
LQRController.Current.Angle=0;
LQRController.Current.Velocity=zeros(2,1);
LQRController.Current.Gain=KStdZero;
LQRController.Current.Input=zeros(4,1);
LQRController.VelocityTier=0:0.5:3;
LQRController.AngleNum=[1,4*(1:(length(LQRController.VelocityTier)-1))];

Cnt_1_=0; %Local Counter
Cnt_2_=0; %Local Counter
Cnt_3_=1; %Local Counter

tic
for Cnt_3_=2:length(LQRController.VelocityTier)
    LQRController.TierContent{Cnt_3_}.State=zeros(12,LQRController.AngleNum(Cnt_3_));
    LQRController.TierContent{Cnt_3_}.Input=zeros(4,LQRController.AngleNum(Cnt_3_));
    LQRController.TierContent{Cnt_3_}.VelocityTier=LQRController.VelocityTier(Cnt_3_);
    LQRController.TierContent{Cnt_3_}.Angle=(0:(LQRController.AngleNum(Cnt_3_)-1))*2*pi/(LQRController.AngleNum(Cnt_3_));
    for Cnt_2_=1:(LQRController.AngleNum(Cnt_3_))
        LQRController.TierContent{Cnt_3_}.Velocity(:,Cnt_2_)=LQRController.VelocityTier(Cnt_3_)*[cos(LQRController.TierContent{Cnt_3_}.Angle(Cnt_2_)),sin(LQRController.TierContent{Cnt_3_}.Angle(Cnt_2_))];
        Environment=EnvironmentInit(0.01,150,0,0);
        Ballbot=BallbotSysInit(Environment);
        close all
        for Cnt_1_=(1:Environment.Timer.Length-1)
            Environment.Timer.Cnt=Environment.Timer.Cnt+1;    %Register the formal counter
            Ballbot=Ballbot.UpdateTimer(Environment.Timer.Cnt*Environment.Timer.Step);
            Ballbot=Ballbot.SimulationUpdate;
            
            LQRController=GSController(LQRController,Ballbot.TrueState.Vel(4:5,1));
            if (rem(Cnt_1_,100)==101)
                disp(diag([1 1 1 1 1 1])*[Ballbot.TrueState.Pos(:,1),Ballbot.TrueState.Vel(:,1),Ballbot.TrueState.Acc(:,1)]);
                disp(LQRController.Current.Gain-KStdZero);
            end
            
            Ballbot.Input=[zeros(4,1),Ballbot.Input(:,1:end-1)];
            Objective=[zeros(3,1);LQRController.TierContent{Cnt_3_}.Velocity(:,Cnt_2_)*Environment.Timer.Step*Cnt_1_;...
                       zeros(3,1);LQRController.TierContent{Cnt_3_}.Velocity(:,Cnt_2_)*0];
            Ballbot.Input(:,1)=(LQRController.Current.Gain)*(Objective-[Ballbot.TrueState.Pos(1:5,1);Ballbot.TrueState.Vel(1:5,1)])+(LQRController.Current.Input);
            %Ballbot=Ballbot.InputUpdate;
        end
        %disp(diag([1 1 1 1 1 1])*[Ballbot.TrueState.Pos(:,1),Ballbot.TrueState.Vel(:,1),Ballbot.TrueState.Acc(:,1)]); 
        disp(Ballbot.Input(:,1))
        %LQRSolveResult=fsolve(@(X) EqlPointCalcFunc(X,LQRController.TierContent{Cnt_3_}.Velocity(:,Cnt_2_)),[Ballbot.TrueState.Pos(1:2,1);Ballbot.Input(:,1)]);
        LQRSolveResult=[Ballbot.TrueState.Pos(1:2,1);Ballbot.Input(:,1)];
        disp(EqlPointCalcFunc(LQRSolveResult,LQRController.TierContent{Cnt_3_}.Velocity(:,Cnt_2_))');
        SSMA=SSMAFcn(LQRSolveResult(1),0,LQRSolveResult(2),0,0, ...
        LQRSolveResult(5),LQRSolveResult(6),LQRSolveResult(4),LQRSolveResult(3),...
        LQRController.TierContent{Cnt_3_}.Velocity(1,Cnt_2_),LQRController.TierContent{Cnt_3_}.Velocity(2,Cnt_2_));
        SSMB=SSMBFcn(LQRSolveResult(1),LQRSolveResult(2));
        LQRController.TierContent{Cnt_3_}.Gain(:,:,Cnt_2_)=lqr(SSMA,SSMB,LQR_Q,LQR_R);
        disp(LQRController.TierContent{Cnt_3_}.Gain(:,:,Cnt_2_))
        LQRController.TierContent{Cnt_3_}.State(:,Cnt_2_)=[Ballbot.TrueState.Pos(:,1);Ballbot.TrueState.Vel(:,1)];
        LQRController.TierContent{Cnt_3_}.Input(:,Cnt_2_)=LQRSolveResult(3:6);
        disp([Cnt_3_ Cnt_2_])
        toc;
    end
    LQRController.CompleteScheduleTier=Cnt_3_;
end
toc
save('LQRControllerBasic','LQRController');
%{
DrawPlot(Ballbot)
figure(2)
    subplot(2,3,1)
    hold on;
    grid on;
    plot(Ballbot.Timer.Point,Ballbot.TrueState.Pos(4,:),':r','LineWidth',2);
    plot(Ballbot.Timer.Point,Ballbot.TrueState.Pos(5,:),'-.g','LineWidth',2);
    title('State X, Y - Position');
    xlabel('Time (s)')
    ylabel('Translational Position (m)')
    legend('X','Y','Location','northoutside','Orientation','horizontal')
    hold off;

    subplot(2,3,2)
    hold on;
    grid on;
    plot(Ballbot.Timer.Point,Ballbot.TrueState.Vel(4,:),':r','LineWidth',2);
    plot(Ballbot.Timer.Point,Ballbot.TrueState.Vel(5,:),'-.g','LineWidth',2);
    title('State X, Y - Velocity');
    xlabel('Time (s)')
    ylabel('Translational Velocity (m/s)')
    legend('X','Y','Location','northoutside','Orientation','horizontal')
    hold off;

    subplot(2,3,4)
    hold on;
    grid on;
    plot(Ballbot.Timer.Point(end-100:end),Ballbot.TrueState.Pos(4,end-100:end),':r','LineWidth',2);
    plot(Ballbot.Timer.Point(end-100:end),Ballbot.TrueState.Pos(5,end-100:end),'-.g','LineWidth',2);
    title('State X, Y - Position');
    xlabel('Time (s)')
    ylabel('Translational Position (m)')
    legend('X','Y','Location','northoutside','Orientation','horizontal')
    hold off;

    subplot(2,3,5)
    hold on;
    grid on;
    plot(Ballbot.Timer.Point(end-100:end),Ballbot.TrueState.Vel(4,end-100:end),':r','LineWidth',2);
    plot(Ballbot.Timer.Point(end-100:end),Ballbot.TrueState.Vel(5,end-100:end),'-.g','LineWidth',2);
    title('State X, Y - Velocity');
    xlabel('Time (s)')
    ylabel('Translational Velocity (m/s)')
    legend('X','Y','Location','northoutside','Orientation','horizontal')
    hold off;
    
    subplot(2,3,3)
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
    plot(Ballbot.Timer.Point(end-100:end),Ballbot.Input(1,end-100:end),':r','LineWidth',2);
    plot(Ballbot.Timer.Point(end-100:end),Ballbot.Input(2,end-100:end),'-.g','LineWidth',2);
    plot(Ballbot.Timer.Point(end-100:end),Ballbot.Input(3,end-100:end),'--b','LineWidth',2);
    plot(Ballbot.Timer.Point(end-100:end),Ballbot.Input(4,end-100:end),'k','LineWidth',2);
    title('System Input - Voltage');
    xlabel('Time (s)')
    ylabel('Voltage (V)')
    legend('U++','U+-','U--','U-+','Location','northoutside','Orientation','horizontal')
    hold off;

figure(1)
plot(Ballbot.Timer.Point(end-100:end),Ballbot.TrueState.Vel(4,end-100:end),'k','LineWidth',3);
title('Position Control - ZeroPoint to (X=3m,Y=0m) - State X');
xlabel('Time(s)');
ylabel('X Velocity (m/s)');
figure(2)
hold on;
suptitle('Position Control - ZeroPoint to (X=3m,Y=0m) - Input');
subplot(2,2,1)
plot(Ballbot.Timer.Point(end-100:end),Ballbot.Input(1,end-100:end),'k','LineWidth',3);
legend('Motor ++')
xlabel('Time(s)');
ylabel('Voltage(V)');
subplot(2,2,2)
plot(Ballbot.Timer.Point(end-100:end),Ballbot.Input(2,end-100:end),'r','LineWidth',3);
legend('Motor +-')
xlabel('Time(s)');
ylabel('Voltage(V)');
subplot(2,2,3)
plot(Ballbot.Timer.Point(end-100:end),Ballbot.Input(3,end-100:end),'b','LineWidth',3);
legend('Motor --')
xlabel('Time(s)');
ylabel('Voltage(V)');
subplot(2,2,4)
plot(Ballbot.Timer.Point(end-100:end),Ballbot.Input(4,end-100:end),'g','LineWidth',3);
legend('Motor -+')
xlabel('Time(s)');
ylabel('Voltage(V)');
hold off;
%}