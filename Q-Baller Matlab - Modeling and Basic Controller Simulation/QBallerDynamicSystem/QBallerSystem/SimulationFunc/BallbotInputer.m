function output=BallbotInputer(obj)
    newinput=obj.Input(:,1);
    oldinput=obj.Input(:,2);
    for ii=1:length(newinput)
        if abs(newinput(ii)-oldinput(ii))>(obj.Timer.Step(1)*obj.OtherInfo_.MaximumInput)
            newinput(ii)=oldinput(ii)+sign(newinput(ii)-oldinput(ii))*(obj.Timer.Step(1)*obj.OtherInfo_.MaximumInput);
        end
        if abs(newinput(ii))>obj.OtherInfo_.MaximumInput
            newinput(ii)=obj.OtherInfo_.MaximumInput*sign(newinput(ii));
        end
    end
    output=[newinput,obj.Input(:,2:end)];
end