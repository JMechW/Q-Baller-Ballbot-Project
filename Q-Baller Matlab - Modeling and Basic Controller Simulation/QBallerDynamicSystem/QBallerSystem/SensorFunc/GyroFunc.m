function output=GyroFunc(ExHandle,SenNum)
    output=[ExHandle.ObjectElement{1}.EulerAngle('MJ',2)*ExHandle.TrueState.Vel(1:3,2),ExHandle.SensorElement{SenNum}.State(:,1:end-1)];
end