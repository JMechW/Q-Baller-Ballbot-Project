figure(1)
figAxes=axes;
hold on
pirange=linspace(0,2*pi,101);
GSPoint=LQRController.TierContent{1}.Velocity;
for ii=2:7
    GSPoint=[GSPoint,LQRController.TierContent{ii}.Velocity];
    PeriFig{ii-1}=plot(figAxes,(LQRController.VelocityTier(ii))*cos(pirange),(LQRController.VelocityTier(ii))*sin(pirange),'--k','LineWidth',3);
end
GSPointFig=plot(figAxes,GSPoint(1,:),GSPoint(2,:),'*r','LineWidth',10);
for ii=2:length(GSPoint(1,:))
    if norm(GSPoint(:,ii))<1.25
    RingFig{ii-1}=plot(figAxes,(0.95/((norm(GSPoint(:,ii)))^(1/10)))*cos(pirange)+GSPoint(1,ii),(0.95/((norm(GSPoint(:,ii)))^(1/10)))*sin(pirange)+GSPoint(2,ii),':g','LineWidth',1);
    elseif norm(GSPoint(:,ii))<2.25
    RingFig{ii-1}=plot(figAxes,(0.95/((norm(GSPoint(:,ii)))^(1/10)))*cos(pirange)+GSPoint(1,ii),(0.95/((norm(GSPoint(:,ii)))^(1/10)))*sin(pirange)+GSPoint(2,ii),':c','LineWidth',1); 
    else
    RingFig{ii-1}=plot(figAxes,(0.95/((norm(GSPoint(:,ii)))^(1/10)))*cos(pirange)+GSPoint(1,ii),(0.95/((norm(GSPoint(:,ii)))^(1/10)))*sin(pirange)+GSPoint(2,ii),':b','LineWidth',1); 
    end
end
GSPointT=GSPoint';
GSTri=delaunayn(GSPointT);
TirFig=triplot(GSTri,GSPointT(:,1),GSPointT(:,2),'-.m','LineWidth',2);
set(figAxes,'Units','pixels','Position',[100,100,800,800])
xlabel('X Velocity (m/s)')
ylabel('Y Velocity (m/s)')