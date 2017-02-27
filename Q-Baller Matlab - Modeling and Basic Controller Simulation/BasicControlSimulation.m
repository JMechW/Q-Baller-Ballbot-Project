clear all
close all
addpath(genpath(pwd))
load('StandardModelZeroPointGain.mat','KStdZero'); %Gain at Zero Point
load('LQRControllerBasic.mat','LQRController');

LQRController.GSPoint=LQRController.TierContent{1}.Velocity;
for ii=2:7
    LQRController.GSPoint=[LQRController.GSPoint,LQRController.TierContent{ii}.Velocity];
end
LQRController.GSTri=delaunayn(LQRController.GSPoint');



Cnt_1_=0; %Local Counter
Cnt_2_=0; %Local Counter

SystemNum=1;
SystemCnt=0;

Environment=EnvironmentInit(0.01,30,0,0);
for SystemCnt=1:SystemNum
    Ballbot{SystemCnt}=BallbotSysInit(Environment);
end

tic

for Cnt_1_=(1:Environment.Timer.Length-1)
    Environment.Timer.Cnt=Environment.Timer.Cnt+1;    %Register the formal counter
    for SystemCnt=1:SystemNum
        Ballbot{SystemCnt}=Ballbot{SystemCnt}.UpdateTimer(Environment.Timer.Cnt*Environment.Timer.Step);
        Ballbot{SystemCnt}=Ballbot{SystemCnt}.UpdateObjectForce;
        Ballbot{SystemCnt}=Ballbot{SystemCnt}.UpdateObjectTorque;
        Ballbot{SystemCnt}=Ballbot{SystemCnt}.SimulationUpdate;
    
        
        disp(diag([180/pi 180/pi 1 1 1 1])*[Ballbot{SystemCnt}.TrueState.Pos(:,1),Ballbot{SystemCnt}.TrueState.Vel(:,1),Ballbot{SystemCnt}.TrueState.Acc(:,1)]); 

        
        Ballbot{SystemCnt}=Ballbot{SystemCnt}.UpdateObjectAttitude;
        Ballbot{SystemCnt}=Ballbot{SystemCnt}.UpdateObjectPosition;
        %Ballbot{SystemCnt}.ObjectElement{1}=Ballbot{SystemCnt}.ObjectElement{1}.VisualAll(0,1,3);
    
        
        Ballbot{SystemCnt}=Ballbot{SystemCnt}.UpdateSensorState;
        Ballbot{SystemCnt}=Ballbot{SystemCnt}.UpdateSensorNoise;
        
        aaa=toc;
        if SystemCnt==1
            LQRController=SphereCGSController(LQRController,Ballbot{SystemCnt}.TrueState.Vel(4:5,2));
            bbb=toc-aaa;
        else
            OldInput=LQRController.Current.Input;
            LQRController=CGSController(LQRController,Ballbot{SystemCnt}.TrueState.Vel(4:5,2));
            ccc=toc-aaa;
            InputCline(:,Cnt_1_)=[(LQRController.Current.Input-OldInput);(Ballbot{SystemCnt}.TrueState.Vel(4:5,2))];
            %disp(InputCline(:,Cnt_1_));
        end
        
        %a=(1/sqrt(3)*0.024*0.5*[sqrt(2) sqrt(2) -sqrt(2) -sqrt(2)]*Ballbot{SystemCnt}.SensorElement{4}.State(:,1)-Ballbot{SystemCnt}.TrueState.Vel(2,2)*0.1)*0.01+a
        %b=(1/sqrt(3)*0.024*0.5*[sqrt(2) -sqrt(2) -sqrt(2) sqrt(2)]*Ballbot{SystemCnt}.SensorElement{4}.State(:,1)+Ballbot{SystemCnt}.TrueState.Vel(1,2)*0.1)*0.01+b
        
        
        Ballbot{SystemCnt}.Input=[zeros(4,1),Ballbot{SystemCnt}.Input(:,1:end-1)];
        
        Angle=45;
        Velocity=1.8;
        if Cnt_1_<100*Velocity
            Objective=[zeros(3,1);0.01*Cnt_1_*Velocity*cos(pi*Angle/180);0.01*Cnt_1_*Velocity*sin(pi*Angle/180);...
                    zeros(3,1);cos(pi*Angle/180)*0.4*0.01*Cnt_1_;sin(pi*Angle/180)*0.4*0.01*Cnt_1_];
                   %{ 
        else
            Objective=[zeros(2,1);0*pi;0.01*Cnt_1_*Velocity*cos(pi*Angle/180);0.01*Cnt_1_*Velocity*sin(pi*Angle/180);...
                    zeros(3,1);Velocity*cos(pi*Angle/180)*0.4;Velocity*sin(pi*Angle/180)*0.4];
               %}   
        elseif Cnt_1_<100+100*Velocity
            Objective=[zeros(2,1);1*pi*0.01*(Cnt_1_-100*Velocity);0.01*Cnt_1_*Velocity*cos(pi*Angle/180);0.01*Cnt_1_*Velocity*sin(pi*Angle/180);...
                    zeros(3,1);Velocity*cos(pi*Angle/180)*0.4;Velocity*sin(pi*Angle/180)*0.4];
        elseif Cnt_1_<1000
            Objective=[zeros(2,1);1*pi;0.01*Cnt_1_*Velocity*cos(pi*Angle/180);0.01*Cnt_1_*Velocity*sin(pi*Angle/180);...
                    zeros(3,1);Velocity*cos(pi*Angle/180)*0.4;Velocity*sin(pi*Angle/180)*0.4];
        elseif Cnt_1_<1239
            Objective=[zeros(2,1);1*pi;0.01*1000*Velocity*cos(pi*Angle/180)+2*0.01*(Cnt_1_-1000);0.01*1000*Velocity*sin(pi*Angle/180)-1*0.01*(Cnt_1_-1000);...
                    zeros(3,1);Velocity*cos(pi*Angle/180)*0.4+0.7272*0.4*0.01*(Cnt_1_-1000)/2.3863;Velocity*sin(pi*Angle/180)*0.4-2.2728*0.4*0.01*(Cnt_1_-1000)/2.3863];
        else
            Objective=[zeros(2,1);1*pi;0.01*1000*Velocity*cos(pi*Angle/180)+2*0.01*(Cnt_1_-1000);0.01*1000*Velocity*sin(pi*Angle/180)-1*0.01*(Cnt_1_-1000);...
                    zeros(3,1);2*0.4;-1*0.4]; 
                    
                  
        end
        %disp(Objective')
        
        Ballbot{SystemCnt}.Input(:,1)=LQRController.Current.Gain*(Objective-[Ballbot{SystemCnt}.TrueState.Pos(1:5,2);Ballbot{SystemCnt}.TrueState.Vel(1:5,2)])+0*LQRController.Current.Input;
        Ballbot{SystemCnt}=Ballbot{SystemCnt}.InputUpdate;
    end
end

toc

for ii=1:SystemNum
    figure(ii)
    DrawPlot(Ballbot{ii})
end
%{
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