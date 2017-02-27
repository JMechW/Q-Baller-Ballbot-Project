function Environment=EnvironmentInit(TimeStep,TimeEnd,Force0,Noise0)
    Environment.Timer.Step=TimeStep;
    Environment.Timer.End=TimeEnd;
    Environment.Timer.Span=0:Environment.Timer.Step:Environment.Timer.End;
    Environment.Timer.Length=length(Environment.Timer.Span);
    Environment.Timer.Cnt=0;
    
    Environment.Noise.Force{1}=[5;5;0]*Force0;
    Environment.Noise.Torque{1}=[1;1;1]*Force0;
    Environment.Noise.Force{2}=[5;5;0]*Force0;
    Environment.Noise.Torque{2}=[1;1;1]*Force0;
    
    Environment.Noise.Sensor{1}=0.005*Noise0;        %Noise Amplitude to the Maximum Cap Ratio
    Environment.Noise.Sensor{2}=0.005*Noise0;
    Environment.Noise.Sensor{3}=0.005*Noise0;
    Environment.Noise.Sensor{4}=0.005*Noise0;
end