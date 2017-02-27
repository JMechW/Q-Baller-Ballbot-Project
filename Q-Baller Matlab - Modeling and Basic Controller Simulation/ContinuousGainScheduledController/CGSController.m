function LQRController=CGSController(LQRController,Velocity)
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
        [TriLine,BaryCentric]=tsearchn(LQRController.GSPoint',LQRController.GSTri,Velocity');
        TriNum=LQRController.GSTri(TriLine,:);
        CurGain=0;
        CurInput=0;
        for ii=1:length(TriNum)
            WeightTier=floor(norm(LQRController.GSPoint(:,TriNum(ii)))/LQRController.VelocityTier(2)*1.01)+1;
            WeightNum=TriNum(ii)-sum(LQRController.AngleNum(1:(WeightTier-1)));
            CurGain=CurGain+BaryCentric(ii)*LQRController.TierContent{WeightTier}.Gain(:,:,WeightNum);
            CurInput=CurInput+BaryCentric(ii)*LQRController.TierContent{WeightTier}.Input(:,WeightNum);
        end

        LQRController.Current.Gain=CurGain;
        LQRController.Current.Input=CurInput;
    end
end