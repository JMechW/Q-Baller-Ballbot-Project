function output=EncoderSetFunc(Handle_,SenNum)
    A0=Handle_.TrueState.Pos(1,2);
    A1=Handle_.TrueState.Vel(1,2);
    B0=Handle_.TrueState.Pos(2,2);
    B1=Handle_.TrueState.Vel(2,2);
    C1=Handle_.TrueState.Vel(3,2);
    x1=Handle_.TrueState.Vel(4,2);
    y1=Handle_.TrueState.Vel(5,2);
    c1=Handle_.TrueState.Vel(6,2);
    Now=Handle_.SensorElement{SenNum}.OtherFunc_(A0,A1,B0,B1,C1,c1,x1,y1);
    output=[Now,Handle_.SensorElement{SenNum}.State(:,1:end-1)];
end