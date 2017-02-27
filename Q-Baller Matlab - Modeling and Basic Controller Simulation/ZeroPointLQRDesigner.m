syms T_x1y1 T_x1y0 T_x0y0 T_x0y1 U_x1y1 U_x1y0 U_x0y0 U_x0y1... 
w_x1y1 w_x1y0 w_x0y0 w_x0y1 w1_x1y1 w1_x1y0 w1_x0y0 w1_x0y1...
A0 B0 C0 a0 b0 c0 X0 Y0 Z0 x0 y0 z0 ...
A1 B1 C1 a1 b1 c1 X1 Y1 Z1 x1 y1 z1 ...
A2 B2 C2 a2 b2 c2 X2 Y2 Z2 x2 y2 z2 ...%System Variables

syms T T_m F_w T_w F_GX F_GY N_G N0 F_dx F_dy F_dz T_dx T_dy T_dz F_DX F_DY F_DZ T_DX T_DY T_DZ; %Intermediates

load([pwd '\StandardModelSSMatrixA.mat'],'SSMA'); %Load State Space Matrix A 12x12
load([pwd '\StandardModelSSMatrixB.mat'],'SSMB'); %Load State Space Matrix B 12x4

SSMAModified=subs(SSMA([1:5 7:11],[1:5 7:11]),[c0 c1 c2],[0 0 0]); %State Space Matrix A 10x10
SSMBModified=subs(SSMB([1:5 7:11],:),[c0 c1 c2],[0 0 0]); %State Space Matrix B 10x4
SSMCModified=diag([1 1 1 1 1 1 1 1 1 1]);
SSMDModified=zeros(10,4);

SYM0=[A1 B1 C1 A0 B0 C0 x1 y1 c1 x0 y0 c0 U_x1y1 U_x1y0 U_x0y0 U_x0y1]; %Zero Point States
VAL0=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]; %Zero Point Values

SSMAModified0=double(subs(SSMAModified,SYM0,VAL0)); %Give Value to Matrix A
SSMBModified0=double(subs(SSMBModified,SYM0,VAL0)); %Give Value to Matrix B

%Modified0=ss(SSMAModified0,SSMBModified0,SSMCModified,SSMDModified);
%sigma(Original0,Modified0)

SSMAFcn=matlabFunction(SSMAModified); %Generate MATLAB Function A
SSMBFcn=matlabFunction(SSMBModified); %Generate MATLAB Function B

save([pwd '\StandardModelSSMatrixAFcn.mat'],'SSMAFcn'); %Save MATLAB Function A
save([pwd '\StandardModelSSMatrixBFcn.mat'],'SSMBFcn'); %Save MATLAB Function B

System_0=ss(SSMAModified0,SSMBModified0,SSMCModified,SSMDModified);

Q=diag([100,100,5000,20,20,50,50,3000,10,10]); %Q Matrix
R=diag([50,50,50,50]); %R Matrix
KStdZero=lqry(System_0,Q,R);%Generate K10 for Zero Point
save([pwd '\StandardModelZeroPointGain.mat'],'KStdZero')%Save Matrix for Future Use


