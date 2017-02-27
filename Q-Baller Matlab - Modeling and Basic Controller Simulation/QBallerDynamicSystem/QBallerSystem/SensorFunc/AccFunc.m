function output=AccFunc(ExHandle,SenNum)
    output=[ExHandle.ObjectElement{1}.EulerAngle('O2L',2)*(ExHandle.TrueState.Acc(4:6,2)+[0;0;-9.81]),ExHandle.SensorElement{SenNum}.State(:,1:end-1)];
end