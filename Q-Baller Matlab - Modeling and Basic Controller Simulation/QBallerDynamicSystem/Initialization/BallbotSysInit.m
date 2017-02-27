function Ballbot=BallbotSysInit(Environment)
    load('WheelSpeedFunc.mat','WheelSpeedFunc');
    load Model3DData.mat

    Ballbot=ObjSystem(Environment.Timer.Length,6,4,'Name','Ballbot','Description','12 States and 4 Inputs accord to time length');
    Ballbot.ObjectElement{1}=ObjModel(Environment.Timer.Length,'Name','Ballbot-Body','Description','Body Part of Ballbot');
    Ballbot.ObjectElement{2}=ObjModel(Environment.Timer.Length,'Name','Ballbot-Ball','Description','Ball of Ballbot');
    Ballbot.SensorElement{1}=ObjSensor(Environment.Timer.Length,3,'Name','Ballbot-Gyro','Description','Gyro Sensor of Ballbot');
    Ballbot.SensorElement{2}=ObjSensor(Environment.Timer.Length,3,'Name','Ballbot-Acc','Description','Acc Sensor of Ballbot');
    Ballbot.SensorElement{3}=ObjSensor(Environment.Timer.Length,1,'Name','Ballbot-Compass','Description','Compass Sensor for yaw detection');
    Ballbot.SensorElement{4}=ObjSensor(Environment.Timer.Length,4,'Name','Ballbot-EncoderSet','Description','Encoder for Motor Speed Detection');
    
    Ballbot.ODE=@(State,Input,BodyForce,BallForce,BodyTorque,BallTorque) StandardModelFcn_mex(State,Input,BodyForce,BallForce,BodyTorque,BallTorque);
    Ballbot.Simulator=@(handle_)    BallbotSimulator(handle_);
    Ballbot.Inputer=@(handle_)  BallbotInputer(handle_);
    
    Ballbot.ObjectElement{1}.AttView.Model3D=BodyModel3D;
    Ballbot.ObjectElement{1}.AttView.ModelSize=BodyModelSize;
    Ballbot.ObjectElement{1}.AttView.Color=BodyModelColor;
    Ballbot.ObjectElement{1}.AttView.Edge=BodyModelEdge;
    Ballbot.ObjectElement{1}.AttView.FigElement{length(BodyModel3D)+1}='end';
    
    Ballbot.ObjectElement{2}.AttView.Model3D=BallModel3D;
    Ballbot.ObjectElement{2}.AttView.ModelSize=BallModelSize;
    Ballbot.ObjectElement{2}.AttView.Color=BallModelColor;
    Ballbot.ObjectElement{2}.AttView.Edge=BallModelEdge;    
    Ballbot.ObjectElement{1}.AttView.FigElement{length(BodyModel3D)+1}='end';
    
    
    
    Ballbot.OtherInfo_.Mass=7.9;
    Ballbot.OtherInfo_.BallRadius=0.1;
    Ballbot.OtherInfo_.MaximumInput=11.1;
    
    Ballbot.ObjectElement{1}.COG=[0;0;0.107];
    Ballbot.ObjectElement{1}.OD=[0;0;0.1];
    Ballbot.ObjectElement{2}.COG=[0;0;0.1];
    Ballbot.ObjectElement{2}.OD=[0;0;0.1];    
    
    %Initiation for Environment
    for ii=1:length(Ballbot.ObjectElement)
    Ballbot.ObjectElement{ii}.ForceFunc=@(handle_)    NoiseForce(Environment.Noise.Force{ii},handle_.ObjectElement{ii}.Force);
    Ballbot.ObjectElement{ii}.TorqueFunc=@(handle_)   NoiseForce(Environment.Noise.Torque{ii},handle_.ObjectElement{ii}.Torque);
    Ballbot.ObjectElement{ii}.PosFunc=@(handle_)      BallbotPositionFunc(handle_,ii);
    end

    Ballbot.ObjectElement{1}.AttFunc=@(handle_) [handle_.TrueState.Pos(1:3,1),handle_.ObjectElement{1}.Attitude(:,1:end-1)];
    Ballbot.ObjectElement{2}.AttFunc=@(handle_) BallAttitudeFunc(handle_);

    %Initiation for Sensor
    for ii=1:length(Ballbot.SensorElement)
        Ballbot.SensorElement{ii}.NoiseFunc=@(handle_)     NoiseSensor(Environment.Noise.Sensor{ii},handle_.SensorElement{ii});
    end

    Ballbot.SensorElement{1}.StateMax=[25*pi/18;25*pi/18;25*pi/18];
    Ballbot.SensorElement{2}.StateMax=[2*9.81;2*9.81;2*9.81];
    Ballbot.SensorElement{3}.StateMax=pi;
    Ballbot.SensorElement{4}.StateMax=[pi;pi;pi;pi];

    Ballbot.SensorElement{4}.OtherFunc_=WheelSpeedFunc;

    Ballbot.SensorElement{1}.StateFunc=@(handle_)    GyroFunc(handle_,1);
    Ballbot.SensorElement{2}.StateFunc=@(handle_)    AccFunc(handle_,2);
    Ballbot.SensorElement{3}.StateFunc=@(handle_)    CompassFunc(handle_,3);
    Ballbot.SensorElement{4}.StateFunc=@(handle_)    EncoderSetFunc(handle_,4);       
    
    
    figure(1)
    TrackFig=axes;
    Ballbot.ObjectElement{1}.AttView.Axes=TrackFig;
    Ballbot.ObjectElement{1}.TrackView.Axes=TrackFig;
    Ballbot.ObjectElement{2}.AttView.Axes=TrackFig;
end