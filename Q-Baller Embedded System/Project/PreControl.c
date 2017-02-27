#include "PreControl.h"


void _Func_State_Observer(void)
{	
	if(Sys_Error>0)
	{
		return;
	}
	
	u8 ii;
	
	for(ii=0;ii<3;ii++)
	{
		if(Timer_ClockLow>(TF_Control*5+Gyro_Time[ii][MCol(Gyro_Time)-1]))
		{
			Sys_Error=2;
			return;
		}//Test if the timing is correct
	}

	float _Medium_Float_1[3][1];
	float _Medium_Float_2[3][1];
	
	MatU_Wave(MAdrU(State_Time),MRow(State_Time),MCol(State_Time),1,0);
	MatU_Wave(MAdrU(State_TimeGap),MRow(State_TimeGap),MCol(State_TimeGap),1,0);
	State_Time[0][MCol(State_Time)-1]=Timer_ClockLow;
	State_TimeGap[0][MCol(State_TimeGap)-1]=State_Time[0][MCol(State_Time)-1]-State_Time[0][MCol(State_Time)-2];	
	
	Att_Euler(Gyro_Att_Final[0][MCol(Gyro_Att_Final)-1],Gyro_Att_Final[1][MCol(Gyro_Att_Final)-1],Gyro_Att_Final[2][MCol(Gyro_Att_Final)-1],0xFF);
	
	MatF_Wave(MAdrF(State_Pos),MRow(State_Pos),MCol(State_Pos),1,0);
	MatF_Wave(MAdrF(State_Vel),MRow(State_Vel),MCol(State_Vel),1,0);
	
	MatF_Cut(MAdrF(Gyro_Att_Final),MAdrF(_Medium_Float_1),MRow(Gyro_Att_Final),MCol(Gyro_Att_Final),
			 1,5,3,1);//Get The Angular Velocity out of the record matrix
	MatF_Mult(MAdrF(Euler_InvJacobian),MAdrF(_Medium_Float_1),MAdrF(_Medium_Float_2),1,
			  MRow(Euler_InvJacobian),MCol(Euler_InvJacobian),MRow(_Medium_Float_1),MCol(_Medium_Float_1));//Turn Angular Velocity into Euler Velocity
	
	for(ii=0;ii<3;ii++)
	{
		State_Pos[ii][MCol(State_Pos)-1]=Gyro_Att_Final[ii][MCol(Gyro_Att_Final)-1];
		State_Vel[ii][MCol(State_Pos)-1]=_Medium_Float_2[ii][0];
	}

	
	_Medium_Float_1[0][0]=(Motor_Vel[0][MCol(Motor_Vel)-1]+Motor_Vel[1][MCol(Motor_Vel)-1]
						  -Motor_Vel[2][MCol(Motor_Vel)-1]-Motor_Vel[3][MCol(Motor_Vel)-1])*((float)0.40824829046386301636621401245098)*Model_Wheel_Radius
						  -State_Vel[1][0]*Model_Ball_Radius;	//O System X Velocity
	_Medium_Float_1[1][0]=(Motor_Vel[0][MCol(Motor_Vel)-1]-Motor_Vel[1][MCol(Motor_Vel)-1]
						  -Motor_Vel[2][MCol(Motor_Vel)-1]+Motor_Vel[3][MCol(Motor_Vel)-1])*((float)0.40824829046386301636621401245098)*Model_Wheel_Radius
						  +State_Vel[0][0]*Model_Ball_Radius;	//O System Y Velocity
	_Medium_Float_1[2][0]=0;
	
	MatF_Mult(MAdrF(Euler_LtoO),MAdrF(_Medium_Float_1),MAdrF(_Medium_Float_2),1,
			  MRow(Euler_LtoO),MCol(Euler_LtoO),MRow(_Medium_Float_1),MCol(_Medium_Float_1));//Turn Angular Velocity into Euler Velocity
	
	for(ii=0;ii<2;ii++)
	{
		State_Vel[ii+3][MCol(State_Vel)-1]=_Medium_Float_2[ii][0];//New XY O Frame Velocity
		State_Pos[ii+3][MCol(State_Pos)-1]=(_Medium_Float_2[ii][0]+State_Vel[ii+3][MCol(State_Pos)-2])*((float)0.5)*State_TimeGap[0][MCol(State_TimeGap)-1]*Timer_MiniStep;//
										  //+State_Pos[ii+3][MCol(State_Pos)-2];//New XY O Frame Position
	}	
	
}




void _Func_Ground_Positioning(void)
{	
	if(Sys_Error>0)
	{
		return;
	}
	
	u8 ii;
	float _Medium_Float_1[3][1];
	float _Medium_Float_2[3][1];	
	
	MatF_Wave(MAdrF(Ground_Vel),MRow(Ground_Vel),MCol(Ground_Vel),1,0);
	MatF_Wave(MAdrF(Ground_Pos),MRow(Ground_Pos),MCol(Ground_Pos),1,0);
	
	for(ii=0;ii<2;ii++)
	{
		_Medium_Float_1[ii][0]=State_Vel[ii+3][MCol(State_Vel)-1];
	}
	
	_Medium_Float_1[2][0]=0;
	
	MatF_Mult(MAdrF(Euler_OtoG),MAdrF(_Medium_Float_1),MAdrF(_Medium_Float_2),1,
			  MRow(Euler_OtoG),MCol(Euler_OtoG),MRow(_Medium_Float_1),MCol(_Medium_Float_1));//Turn Angular Velocity into Euler Velocity
	
	for(ii=0;ii<3;ii++)
	{
		Ground_Vel[ii][MCol(Ground_Vel)-1]=_Medium_Float_2[ii][0];
		Ground_Pos[ii][MCol(Ground_Pos)-1]=(_Medium_Float_2[ii][0]+Ground_Vel[ii][MCol(Ground_Vel)-2])*((float)0.5)*Control_TimeGap[0][MCol(Control_TimeGap)-1]*Timer_MiniStep
										  +Ground_Pos[ii][MCol(Ground_Pos)-2];//New XY O Frame Position
	}
}


void _Func_PreControl(void)
{
	_Func_State_Observer();
	_Func_Ground_Positioning();
}


