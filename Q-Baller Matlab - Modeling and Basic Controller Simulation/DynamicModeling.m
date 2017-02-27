tic;
digits(32);

syms Hx Hy Hz g M m R r T_0 n_0 U_0...
J_w b_w b_BL b_bL...
j_XX j_XY j_XZ j_YX j_YY j_YZ j_ZX j_ZY j_ZZ...
j_xx j_xy j_xz j_yx j_yy j_yz j_zx j_zy j_zz...
b_XX b_XY b_XZ b_YX b_YY b_YZ b_ZX b_ZY b_ZZ...
b_xx b_xy b_xz b_yx b_yy b_yz b_zx b_zy b_zz...%System Constants

syms T_x1y1 T_x1y0 T_x0y0 T_x0y1 U_x1y1 U_x1y0 U_x0y0 U_x0y1... 
w_x1y1 w_x1y0 w_x0y0 w_x0y1 w1_x1y1 w1_x1y0 w1_x0y0 w1_x0y1...
A0 B0 C0 a0 b0 c0 X0 Y0 Z0 x0 y0 z0 ...
A1 B1 C1 a1 b1 c1 X1 Y1 Z1 x1 y1 z1 ...
A2 B2 C2 a2 b2 c2 X2 Y2 Z2 x2 y2 z2 ...%System Variables
AS0 BS0 CS0 aS0 bS0 cS0 XS0 YS0 ZS0 xS0 yS0 zS0 ...
AS1 BS1 CS1 aS1 bS1 cS1 XS1 YS1 ZS1 xS1 yS1 zS1 ...
AS2 BS2 CS2 aS2 bS2 cS2 XS2 YS2 ZS2 xS2 yS2 zS2 ...%Sensor Variables

syms T T_m F_w T_w F_GX F_GY N_G N0 F_dx F_dy F_dz T_dx T_dy T_dz F_DX F_DY F_DZ T_DX T_DY T_DZ; %Intermediates

%Disturbance Forces and Torques
F_d=[F_dx;F_dy;F_dz];
T_d=[T_dx;T_dy;T_dz];
F_D=[F_DX;F_DY;F_DZ];
T_D=[T_DX;T_DY;T_DZ];
GM=[0 0 -g*M];

Q0T=[A0 B0 C0 x0 y0 c0]; %Variable States
Q0=[A0;B0;C0;x0;y0;c0];%Where A B C are Euler Angles of the Body
Q1T=[A1 B1 C1 x1 y1 c1];%And a b c are angular velocity of the Ball in Orientation Frame
Q1=[A1;B1;C1;x1;y1;c1];
Q2T=[A2 B2 C2 x2 y2 c2];
Q2=[A2;B2;C2;x2;y2;c2];

CLGA=[1 0 0; 0 cos(A0) -sin(A0); 0 sin(A0) cos(A0)];%X Rolling
CLGB=[cos(B0) 0 sin(B0); 0 1 0; -sin(B0) 0 cos(B0)];%Y Pitching
CLGC=[cos(C0) -sin(C0) 0; sin(C0) cos(C0) 0; 0 0 1];%Z Yawing

CGLA=[CLGA(1,1) CLGA(2,1) CLGA(3,1);...
      CLGA(1,2) CLGA(2,2) CLGA(3,2);...
      CLGA(1,3) CLGA(2,3) CLGA(3,3)]; %Coordinate Conversion
CGLB=[CLGB(1,1) CLGB(2,1) CLGB(3,1);...
      CLGB(1,2) CLGB(2,2) CLGB(3,2);...
      CLGB(1,3) CLGB(2,3) CLGB(3,3)]; %Coordinate Conversion
CGLC=[CLGC(1,1) CLGC(2,1) CLGC(3,1);...
      CLGC(1,2) CLGC(2,2) CLGC(3,2);...
      CLGC(1,3) CLGC(2,3) CLGC(3,3)]; %Coordinate Conversion
  
C_L_to_G=CLGC*CLGB*CLGA;
C_G_to_L=CGLA*CGLB*CGLC;
      
C_O_to_L=subs(C_G_to_L,C0,0);
C_L_to_O=subs(C_L_to_G,C0,0);%Coordinate Conversion

C_O_to_G=[cos(C0) -sin(C0) 0; sin(C0) cos(C0) 0; 0 0 1];
C_G_to_O=[cos(C0) sin(C0) 0; -sin(C0) cos(C0) 0; 0 0 1];%Coordinate Conversion

Mj=[1 0 -sin(B0); 0 cos(A0) sin(A0)*cos(B0); 0 -sin(A0) cos(B0)*cos(A0)]; %Jacobian for Euler Velocity
MjT=[Mj(1,1) Mj(2,1) Mj(3,1);...
     Mj(1,2) Mj(2,2) Mj(3,2);...
     Mj(1,3) Mj(2,3) Mj(3,3)]; %Coordinate Conversion
 
IX=sqrt(6)/4;IY=sqrt(6)/4;IZ=1/2;IA=sqrt(2)/2;IB=sqrt(2)/4;IC=sqrt(3)/2;
M_M=[IX -IY -IZ; -IX -IY IZ; -IX IY -IZ; IX IY IZ]'; %Motor Torque Postive Direction
M_R=[IB -IB IC; IB IB IC; -IB IB IC; -IB -IB IC]'; %Motor Contact Point Position Direction
M_L=[IA IA 0;IA -IA 0; -IA -IA 0; -IA IA 0]'; %Motor Contact Point Velocity Direction
T_M=[T_x1y1;T_x1y0;T_x0y0;T_x0y1]; %Motor X+Y+;X+Y-;X-Y-;X-Y+ respectively
T_W=-(R/r)*M_M*T_M;

J_B=[j_XX j_XY j_XZ; j_YX j_YY j_YZ; j_ZX j_ZY j_ZZ];
J_b=[j_xx j_xy j_xz; j_yx j_yy j_yz; j_zx j_zy j_zz];
B_BR=[b_XX b_XY b_XZ; b_YX b_YY b_YZ; b_ZX b_ZY b_ZZ];
B_bR=[b_xx b_xy b_xz; b_yx b_yy b_yz; b_zx b_zy b_zz];
B_BL=b_BL;
B_bL=b_bL;

VecR=R*M_R;
VecG=[0;0;-R];
VecH=[Hx;Hy;Hz];

%Velocities
VOT=[x1 y1 0];
VO=[x1;y1;0];%Velocity in Orientation Frame
VGT=[x1 y1 0]*C_G_to_O;
VG=C_O_to_G*[x1;y1;0];%Velocity in Ground Frame

WBL=Mj*Q1(1:3);
WBLT=Q1T(1:3)*MjT;
WbO=[-Q1(5)/R;Q1(4)/R;Q1(6)];
WbOT=[-Q1(5)/R Q1(4)/R Q1(6)];
WbL=C_O_to_L*WbO;
WbLT=WbOT*C_L_to_O;
WbG=C_O_to_G*WbO;
WbGT=WbOT*C_G_to_O;

%Kinematics
TL=0.5*(VGT*(M+m)*VG);
TR=0.5*(WBLT*J_B*WBL)+0.5*(WbOT*J_b*WbO);
TC=M*(VGT*C_L_to_G)*(cross(WBL,VecH));
%Potential
V=-GM*C_L_to_G*VecH;
%Lagrangian Factor
L=TL+TR+TC-V;

%Energy Dissipation
DR=0.5*VOT*(C_L_to_O*B_BL+B_bL)*VO+0.5*WBLT*B_BR*WBL+0.5*WbOT*B_bR*WbO;
%Input
Qdt=WbOT*C_L_to_O*(T_W+C_G_to_L*T_d)+WBLT*(-T_W+C_G_to_L*T_D)+VGT*(F_D+F_d)-DR;

%Lagrangian Equation
EQLdq_dt=jacobian(jacobian(L,Q1),Q1)*Q2+jacobian(jacobian(L,Q1),Q0)*Q1;
EQLdqT=jacobian(L,Q0);
EQLdq=[EQLdqT(1);EQLdqT(2);EQLdqT(3);EQLdqT(4);EQLdqT(5);EQLdqT(6)];
EQL=EQLdq_dt-EQLdq;
EQRT=jacobian(Qdt,Q1);
EQR=[EQRT(1);EQRT(2);EQRT(3);EQRT(4);EQRT(5);EQRT(6)];
EQL=EQL-EQR+jacobian(EQR,[T_d;T_D;F_D;F_d])*[T_d;T_D;F_D;F_d];
EQR=EQR-EQR+jacobian(EQR,[T_d;T_D;F_D;F_d])*[T_d;T_D;F_D;F_d];

%Motor Models
w_x1y1=M_L(:,1)'*(cross(C_O_to_L*WbO,VecR(:,1))-cross(WBL,VecR(:,1)))/r;
%w_x1y1*r-M_L(:,1)'*cross(WBL,VecR(:,1))=M_L(:,1)'*cross(C_O_to_L*WbO,VecR(:,1))
w_x1y0=M_L(:,2)'*(cross(C_O_to_L*WbO,VecR(:,2))-cross(WBL,VecR(:,2)))/r;
w_x0y0=M_L(:,3)'*(cross(C_O_to_L*WbO,VecR(:,3))-cross(WBL,VecR(:,3)))/r;
w_x0y1=M_L(:,4)'*(cross(C_O_to_L*WbO,VecR(:,4))-cross(WBL,VecR(:,4)))/r;
w1_x1y1=jacobian(w_x1y1,Q0)*Q1+jacobian(w_x1y1,Q1)*Q2;
w1_x1y0=jacobian(w_x1y0,Q0)*Q1+jacobian(w_x1y0,Q1)*Q2;
w1_x0y0=jacobian(w_x0y0,Q0)*Q1+jacobian(w_x0y0,Q1)*Q2;
w1_x0y1=jacobian(w_x0y1,Q0)*Q1+jacobian(w_x0y1,Q1)*Q2;

w=[w_x1y1;w_x1y0;w_x0y0;w_x0y1];
w1=[w1_x1y1;w1_x1y0;w1_x0y0;w1_x0y1];
U=[U_x1y1;U_x1y0;U_x0y0;U_x0y1];
T_MotorModel=(T_0/U_0)*U-J_w*w1-b_w*w-(60*T_0/(2*pi*n_0))*w;

%Complete System
EQR=simplify(EQR);
EQL=simplify(subs(EQL,T_M,T_MotorModel));


Name={'StandardModel';'RandomModel';'StandardModelSSMatrix';'RandomModelSSMatrix'};
for ii=0:0
    %System Constants
    CONS=[Hx Hy Hz g M m R r T_0 n_0 U_0...
           J_w b_w b_BL b_bL...
           j_XX j_XY j_XZ j_YX j_YY j_YZ j_ZX j_ZY j_ZZ...
           j_xx j_xy j_xz j_yx j_yy j_yz j_zx j_zy j_zz...
           b_XX b_XY b_XZ b_YX b_YY b_YZ b_ZX b_ZY b_ZZ...
           b_xx b_xy b_xz b_yx b_yy b_yz b_zx b_zy b_zz]; 

    VALS=vpa([0 0 0.107 9.81 6.4 1.5 0.1 0.024 2.5 2000 12 ...//Standard and Random Parameters
           1.25*10^(-5) 0.005 0.008 0.002 ...
           0.1488 0 0 0 0.1512 0 0 0 0.0746...
           0.00975 0 0 0 0.00975 0 0 0 0.00975...
           0.02 0 0 0 0.02 0 0 0 0.01...
           0.01 0 0 0 0.01 0 0 0 0.01]+...
           [0.1 0.1 0.1 0 6.4 1.5 0.1 0.024 2.5 2000 0 ...
           1.25*10^(-5) 0.005 0.008 0.002 ...
           0.1488 0 0 0 0.1512 0 0 0 0.0746...
           0.00975 0 0 0 0.00975 0 0 0 0.00975...
           0.02 0 0 0 0.02 0 0 0 0.01...
           0.01 0 0 0 0.01 0 0 0 0.01]*diag([random('unif',-0.3,-0.2,[2,1]);random('unif',0.2,0.3,[1,1]);random('unif',-0.3,0.3,[5,1]);random('unif',-0.3,0,[2,1]);random('unif',-0.3,0.3,[41,1])])*ii,4);
    
    digits(4);
    VALS=double(VALS)*1e6;
    digits(16);
    VALS=double(VALS);
    
    save(char(strcat(pwd,'\',Name(ii+1),'Data.mat')),'VALS');       
       
    EQLreal=simplify(subs(EQL,CONS,sym(VALS,'r')/1e6));
    EQRreal=simplify(subs(EQR,CONS,sym(VALS,'r')/1e6));
    
    %Standard Form
    Matrix_M=simplify(jacobian(EQLreal,Q2));
    invMatrix_M=simplify(inv(Matrix_M));
    disp(3*ii+1);
    
    %SEQreal=simplify(invMatrix_M*simplify(EQRreal-EQLreal+Matrix_M*Q2));
    SEQreal=invMatrix_M*simplify(EQRreal-EQLreal+Matrix_M*Q2);
    SEQreal1=simplify(SEQreal(1));
    disp('Simplification');
    SEQreal2=simplify(SEQreal(2));
    disp('Is');
    SEQreal3=simplify(SEQreal(3));
    disp('A');
    SEQreal4=simplify(SEQreal(4));
    disp('Tough');
    SEQreal5=simplify(SEQreal(5));
    disp('Job');
    SEQreal6=simplify(SEQreal(6));
    disp('Indeed');
    SEQreal=[SEQreal1;SEQreal2;SEQreal3;SEQreal4;SEQreal5;SEQreal6];
    SEQideal=subs(SEQreal,[T_d;T_D;F_D;F_d],zeros(12,1));
    disp(3*ii+2);
    
    SSMA=[zeros(6,6) eye(6,6);jacobian(SEQideal,[Q0;Q1])];
    SSMB=[zeros(6,4);jacobian(SEQideal,U)];
    disp(3*ii+3);
    
    save(char(strcat(pwd,'\',Name(ii+1),'Real.mat')),'SEQreal');
    save(char(strcat(pwd,'\',Name(ii+1),'Ideal.mat')),'SEQideal');
    save(char(strcat(pwd,'\',Name(ii+3),'A.mat')),'SSMA');
    save(char(strcat(pwd,'\',Name(ii+3),'B.mat')),'SSMB');

    FullODE=[Q1;SEQreal];
    matlabFunction(FullODE,'file',char(strcat(Name(ii+1),'Fcn')),'vars',{[Q0;Q1],[U],[F_D],[F_d],[T_D],[T_d]});
end
toc;

tic;
CoderType={coder.typeof(0,[12,1]),...
coder.typeof(0,[4,1]),...
coder.typeof(0,[3,1]),...
coder.typeof(0,[3,1]),...
coder.typeof(0,[3,1]),...
coder.typeof(0,[3,1])};
pathstandard=char(strcat(pwd,'\',Name(1),'Fcn.m'));
pathrandom=char(strcat(pwd,'\',Name(2),'Fcn.m'));
codegen -config:mex StandardModelFcn -args CoderType
codegen -config:mex RandomModelFcn -args CoderType
toc;
