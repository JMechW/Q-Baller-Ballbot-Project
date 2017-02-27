function NSensor=NoiseSensor(NoiseRatio,Obj)
    RawSensor=Obj.State;
    SensorRange=Obj.StateMax;
    NSensor=[RawSensor(:,1)+random('normal',0,NoiseRatio,length(RawSensor(:,1)),1).*SensorRange,RawSensor(:,2:end)];
end