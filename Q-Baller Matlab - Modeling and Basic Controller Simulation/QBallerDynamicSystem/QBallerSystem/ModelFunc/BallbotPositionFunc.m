function output=BallbotPositionFunc(Handle_,ii)
    O2G1=Handle_.ObjectElement{1}.EulerAngle('O2G',1,Handle_.TrueState.Pos(1:3,1));
    O2G2=Handle_.ObjectElement{1}.EulerAngle('O2G',2,Handle_.TrueState.Pos(1:3,2));
    V1=[Handle_.TrueState.Vel(4:5,1);0];
    V2=[Handle_.TrueState.Vel(4:5,2);0];
    h=Handle_.Timer.Step(1,1);
    output=[0.5*(O2G1*V1+O2G2*V2)*h+Handle_.ObjectElement{ii}.Position(:,1),Handle_.ObjectElement{ii}.Position(:,1:end-1)];
end