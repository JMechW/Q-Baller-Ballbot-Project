function output=BallbotSimulator(obj)
    minh=obj.Timer.Step(1,1)/obj.Timer.Seg;
    VAL00=[obj.TrueState.Pos(:,1);obj.TrueState.Vel(:,1)];
    U=obj.Input(:,1);
    FD=obj.ObjectElement{1}.Force(:,1);
    Fd=obj.ObjectElement{2}.Force(:,1);
    TD=obj.ObjectElement{1}.Torque(:,1);
    Td=obj.ObjectElement{2}.Torque(:,1);
    VelAcc=obj.ODE(VAL00,U,FD,Fd,TD,Td);
    for ii=1:obj.Timer.Seg
        RKK1=obj.ODE(VAL00,U,FD,Fd,TD,Td);
        RKK2=obj.ODE(VAL00+0.5*minh*RKK1,U,FD,Fd,TD,Td);
        RKK3=obj.ODE(VAL00+0.5*minh*RKK2,U,FD,Fd,TD,Td);
        RKK4=obj.ODE(VAL00+minh*RKK3,U,FD,Fd,TD,Td);
        VAL00=VAL00+(1/6)*(RKK1+2*RKK2+2*RKK3+RKK4)*minh;
    end
    output.Pos=[VAL00(1:6),obj.TrueState.Pos(:,1:end-1)];
    output.Vel=[VAL00(7:12),obj.TrueState.Vel(:,1:end-1)];
    output.Acc=[VelAcc(7:12),obj.TrueState.Acc(:,1:end-1)];
end