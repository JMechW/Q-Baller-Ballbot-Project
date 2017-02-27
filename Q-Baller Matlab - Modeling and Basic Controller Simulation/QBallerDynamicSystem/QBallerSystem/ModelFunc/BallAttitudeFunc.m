function output=BallAttitudeFunc(Handle_)
    R=Handle_.OtherInfo_.BallRadius;
	h=Handle_.Timer.Step(1,1);
    Conv1=Handle_.ObjectElement{2}.EulerAngle('G2L',1)*Handle_.ObjectElement{1}.EulerAngle('O2G',1);
    Conv2=Handle_.ObjectElement{2}.EulerAngle('G2L',2)*Handle_.ObjectElement{1}.EulerAngle('O2G',2);
    InvMj1=Handle_.ObjectElement{2}.EulerAngle('MJInv',1);
    InvMj2=Handle_.ObjectElement{2}.EulerAngle('MJInv',2);
    Speed1=[-Handle_.TrueState.Vel(5,1)/R;Handle_.TrueState.Vel(4,1)/R;Handle_.TrueState.Vel(6,1)];
    Speed2=[-Handle_.TrueState.Vel(5,2)/R;Handle_.TrueState.Vel(4,2)/R;Handle_.TrueState.Vel(6,2)];
    output=[0.5*h*(InvMj1*Conv1*Speed1+InvMj2*Conv2*Speed2)+Handle_.ObjectElement{2}.Attitude(:,1),       Handle_.ObjectElement{2}.Attitude(:,1:end-1)];
end