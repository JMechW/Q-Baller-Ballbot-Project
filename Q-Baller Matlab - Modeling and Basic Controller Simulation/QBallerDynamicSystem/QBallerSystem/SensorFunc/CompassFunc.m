function output=CompassFunc(Handle,SenNum)
    output=[Handle.TrueState.Pos(3,2),Handle.SensorElement{SenNum}.State(:,1:end-1)];
end