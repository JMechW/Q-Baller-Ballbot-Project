function LQRController=SphereCGSController(LQRController,Velocity)
    LQRController.Current.Velocity=Velocity;
    VelNorm=norm(Velocity);
    if VelNorm==0
        LQRController.Current.Angle=0;
    elseif ((Velocity(1)>=0) && (Velocity(2)>=0))
        LQRController.Current.Angle=asin(Velocity(2)/VelNorm);
    elseif (Velocity(1)<0)
        LQRController.Current.Angle=pi-asin(Velocity(2)/VelNorm);
    else
        LQRController.Current.Angle=2*pi+asin(Velocity(2)/VelNorm);
    end
    
    LQRController.Current.Tier=floor(VelNorm/LQRController.VelocityTier(2))+1;
    
    if LQRController.Current.Tier>LQRController.CompleteScheduleTier
        LQRController.Current.Tier=LQRController.CompleteScheduleTier;
    end

    if LQRController.CompleteScheduleTier==1
        LQRController.Current.Num=1;
        LQRController.Current.Gain=LQRController.TierContent{1}.Gain;
        LQRController.Current.Input=zeros(4,1);
    elseif LQRController.Current.Tier==LQRController.CompleteScheduleTier
        NumLow=floor(LQRController.Current.Angle/2/pi*LQRController.AngleNum(LQRController.Current.Tier))+1;
        NumHigh=NumLow+1;
        if NumHigh>LQRController.AngleNum(LQRController.Current.Tier)
            NumHigh=1;
            WeighLow=abs(LQRController.Current.Angle-2*pi)...
                     /(2*pi/LQRController.AngleNum(LQRController.Current.Tier));
            WeighHigh=abs(LQRController.Current.Angle-LQRController.TierContent{LQRController.Current.Tier}.Angle(NumLow))...
                         /(2*pi/LQRController.AngleNum(LQRController.Current.Tier));
        else
            WeighLow=abs(LQRController.Current.Angle-LQRController.TierContent{LQRController.Current.Tier}.Angle(NumHigh))...
                         /(2*pi/LQRController.AngleNum(LQRController.Current.Tier));
            WeighHigh=abs(LQRController.Current.Angle-LQRController.TierContent{LQRController.Current.Tier}.Angle(NumLow))...
                         /(2*pi/LQRController.AngleNum(LQRController.Current.Tier));
        end
        LQRController.Current.Gain=WeighLow*LQRController.TierContent{LQRController.Current.Tier}.Gain(:,:,NumLow)+...
                                   WeighHigh*LQRController.TierContent{LQRController.Current.Tier}.Gain(:,:,NumHigh);
        LQRController.Current.Input=WeighLow*LQRController.TierContent{LQRController.Current.Tier}.Input(:,NumLow)+...
                                    WeighHigh*LQRController.TierContent{LQRController.Current.Tier}.Input(:,NumHigh);        
    else
        NumBottomLow=floor(LQRController.Current.Angle/2/pi*LQRController.AngleNum(LQRController.Current.Tier))+1;
        NumBottomHigh=NumBottomLow+1;
        NumTopLow=floor(LQRController.Current.Angle/2/pi*LQRController.AngleNum(LQRController.Current.Tier+1))+1;
        NumTopHigh=NumTopLow+1;
        
        if NumBottomHigh>LQRController.AngleNum(LQRController.Current.Tier)
            NumBottomHigh=1;
            WeighBottomLow=abs(LQRController.Current.Angle-2*pi)...
                         /(2*pi/LQRController.AngleNum(LQRController.Current.Tier));
            WeighBottomHigh=abs(LQRController.Current.Angle-LQRController.TierContent{LQRController.Current.Tier}.Angle(NumBottomLow))...
                         /(2*pi/LQRController.AngleNum(LQRController.Current.Tier));
        else
            WeighBottomLow=abs(LQRController.Current.Angle-LQRController.TierContent{LQRController.Current.Tier}.Angle(NumBottomHigh))...
                         /(2*pi/LQRController.AngleNum(LQRController.Current.Tier));
            WeighBottomHigh=abs(LQRController.Current.Angle-LQRController.TierContent{LQRController.Current.Tier}.Angle(NumBottomLow))...
                         /(2*pi/LQRController.AngleNum(LQRController.Current.Tier));
        end
        
        if NumTopHigh>LQRController.AngleNum(LQRController.Current.Tier+1)
            NumTopHigh=1;
            WeighTopLow=abs(LQRController.Current.Angle-2*pi)...
                         /(2*pi/LQRController.AngleNum(LQRController.Current.Tier+1));
            WeighTopHigh=abs(LQRController.Current.Angle-LQRController.TierContent{LQRController.Current.Tier+1}.Angle(NumTopLow))...
                         /(2*pi/LQRController.AngleNum(LQRController.Current.Tier+1));
        else
            WeighTopLow=abs(LQRController.Current.Angle-LQRController.TierContent{LQRController.Current.Tier+1}.Angle(NumTopHigh))...
                         /(2*pi/LQRController.AngleNum(LQRController.Current.Tier+1));
            WeighTopHigh=abs(LQRController.Current.Angle-LQRController.TierContent{LQRController.Current.Tier+1}.Angle(NumTopLow))...
                         /(2*pi/LQRController.AngleNum(LQRController.Current.Tier+1));
        end
        
        BottomGain=WeighBottomLow*LQRController.TierContent{LQRController.Current.Tier}.Gain(:,:,NumBottomLow)+...
                                   WeighBottomHigh*LQRController.TierContent{LQRController.Current.Tier}.Gain(:,:,NumBottomHigh);
        BottomInput=WeighBottomLow*LQRController.TierContent{LQRController.Current.Tier}.Input(:,NumBottomLow)+...
                                    WeighBottomHigh*LQRController.TierContent{LQRController.Current.Tier}.Input(:,NumBottomHigh);      
        
        TopGain=WeighTopLow*LQRController.TierContent{LQRController.Current.Tier+1}.Gain(:,:,NumTopLow)+...
                                   WeighTopHigh*LQRController.TierContent{LQRController.Current.Tier+1}.Gain(:,:,NumTopHigh);
        TopInput=WeighTopLow*LQRController.TierContent{LQRController.Current.Tier+1}.Input(:,NumTopLow)+...
                                    WeighTopHigh*LQRController.TierContent{LQRController.Current.Tier+1}.Input(:,NumTopHigh);                                
        
        BottomWeigh=abs(VelNorm-LQRController.VelocityTier(LQRController.Current.Tier+1))/LQRController.VelocityTier(2);
        TopWeigh=abs(VelNorm-LQRController.VelocityTier(LQRController.Current.Tier))/LQRController.VelocityTier(2);
        
        LQRController.Current.Gain=BottomWeigh*BottomGain+TopWeigh*TopGain;
        LQRController.Current.Input=BottomWeigh*BottomInput+TopWeigh*TopInput;
    end
end