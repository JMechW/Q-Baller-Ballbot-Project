#include "InControl.h"

void _Func_AB_Reference(void)
{
	u8 ii;

	if(Sys_Error>0)
	{
		return;
	}
	
	for(ii=0;ii<2;ii++)
	{
		Control_ObjState_Pos[ii][MCol(Control_ObjState_Pos)-1]=0;//Get Current Ground Velocity
		Control_ObjState_Vel[ii][MCol(Control_ObjState_Vel)-1]=0;//Get Objective Ground Velocity
	}		
	
}


void _Func_C_Reference(void)
{	
	if(Sys_Error>0)
	{
		return;
	}

	
	if(Control_Completion_C>0)
	{
		Control_ObjState_Vel[2][MCol(Control_ObjState_Vel)-1]=Control_ObjState_Vel[2][MCol(Control_ObjState_Vel)-2];
		Control_ObjState_Pos[2][MCol(Control_ObjState_Pos)-1]=Control_ObjState_Pos[2][MCol(Control_ObjState_Pos)-2];
		return;
	}

	
	float _Medium_1;
	
	float _Norm_1;
	float _Norm_2;
	
	float _Yaw_Vel;
	
	Control_ObjState_Vel[2][MCol(Control_ObjState_Vel)-1]=0;
	Control_ObjState_Pos[2][MCol(Control_ObjState_Pos)-1]=Control_Yaw_Pos[0][MCol(Control_Yaw_Pos)-1];
	
	if (fabsf(Control_Yaw_Vel[0][MCol(Control_Yaw_Vel)-1])<Control_Yaw_DefVel)
	{
		_Yaw_Vel=Control_Yaw_DefVel;
	}
	else
	{
		if(fabsf(Control_Yaw_Vel[0][MCol(Control_Yaw_Vel)-1])<Control_Yaw_MaxVel)
		{
			_Yaw_Vel=Control_Yaw_MaxVel;
		}
		else
		{
			_Yaw_Vel=fabsf(Control_Yaw_Vel[0][MCol(Control_Yaw_Vel)-1]);
		}
	}
	
	
	
	_Medium_1=Control_ObjState_Pos[2][MCol(Control_ObjState_Pos)-1]-Control_ObjState_Pos[2][MCol(Control_ObjState_Pos)-1];
	_Norm_1=fabsf(_Medium_1);
	_Norm_2=fabsf(Timer_MiniStep*((float) State_TimeGap[0][MCol(State_TimeGap)-1])*_Yaw_Vel);
	if (_Norm_1>_Norm_2)
	{
		Control_ObjState_Pos[2][MCol(Control_ObjState_Pos)-1]=Control_ObjState_Pos[2][MCol(Control_ObjState_Pos)-2]+_Norm_2*_Medium_1/_Norm_1;
	}
	else
	{
		Control_Completion_C=1;
	}
}


void _Func_XY_Reference(void)
{
	
	u8 ii;
	
	if(Sys_Error>0)
	{
		return;
	}
	
	if(Control_Completion_XY>0)
	{
		for (ii=0;ii<2;ii++)
		{
		Control_ObjState_Vel[ii+3][MCol(Control_ObjState_Vel)-1]=Control_ObjState_Vel[ii+3][MCol(Control_ObjState_Vel)-2];
		Control_ObjState_Pos[ii+3][MCol(Control_ObjState_Pos)-1]=Control_ObjState_Pos[ii+3][MCol(Control_ObjState_Pos)-2];
		}
		return;
	}


	float _Medium_Float_1[3][1]; //Calculation Use
	float _Medium_Float_2[3][1];
	
	//float _Medium_Float_3[3][1]; //Vel Result Use (Frame O)
	float _Medium_Float_4[3][1]; //Pos Result Use (Frame O)
	
	float _Store_Float_1[3][1];	//Save Ground Vel Error
	float _Store_Float_2[3][1]; //Save Pos Vel Error
	
	//float _Store_Float_3[3][1]; //Save Objective Pos Error
	float _Store_Float_4[3][1]; //Save Objective Pos Error
	
	
	//Norms of Aboves
	float _Norm_Vel_1;
	float _Norm_Vel_2;
	
	//float _Norm_Pos_1;
	float _Norm_Pos_2;

	
	for(ii=0;ii<2;ii++)
	{
		_Medium_Float_1[ii][0]=Ground_Vel[ii][MCol(Ground_Vel)-1];//Get Current Ground Velocity
		_Medium_Float_2[ii][0]=Control_Gnd_Vel[ii][MCol(Control_Gnd_Vel)-1];//Get Objective Ground Velocity
	}	
	
	_Medium_Float_1[2][0]=0;
	_Medium_Float_2[2][0]=0;
	
	MatF_Plus(MAdrF(_Medium_Float_1), MAdrF(_Medium_Float_2), MAdrF(_Store_Float_1), -1.0, 1.0, 
			  MRow(_Medium_Float_1), MCol(_Medium_Float_1), MRow(_Medium_Float_2), MCol(_Medium_Float_2));
	//Get Velocity Error

	
	for(ii=0;ii<2;ii++)
	{
		_Medium_Float_1[ii][0]=Ground_Pos[ii][MCol(Ground_Pos)-1];//Get Ground Position
		_Medium_Float_2[ii][0]=Control_Gnd_Pos[ii][MCol(Control_Gnd_Pos)-1];//Get Objective
	}	
	
	_Medium_Float_1[2][0]=0;
	_Medium_Float_2[2][0]=0;	
	
	MatF_Plus(MAdrF(_Medium_Float_1), MAdrF(_Medium_Float_2), MAdrF(_Store_Float_2), -1.0, 1.0, 
			  MRow(_Medium_Float_1), MCol(_Medium_Float_1), MRow(_Medium_Float_2), MCol(_Medium_Float_2));
	//Get Position Error
	
	
	_Norm_Vel_1=sqrtf(powf(_Store_Float_1[0][0],2)+powf(_Store_Float_1[1][0],2));//Norm of Velocity Error
	//_Norm_Pos_1=sqrtf(powf(_Store_Float_2[0][0],2)+powf(_Store_Float_2[1][0],2));//Norm of Position Error
	
	
	
	
	
	
	
	
	
	
	
	
	
	switch (Control_Mode_XY)
	{
		case 1://As the basic position controller
					
				MatF_Mult(MAdrF(Euler_GtoO),MAdrF(_Store_Float_2),MAdrF(_Medium_Float_4),1,
						  MRow(Euler_GtoO),MCol(Euler_GtoO),MRow(_Store_Float_2),MCol(_Store_Float_2));	

				//Get the Orientation Position Difference
				for(ii=0;ii<2;ii++)
				{
					Control_ObjState_Vel[ii+3][MCol(Control_ObjState_Vel)-1]=0;//Set Velocity Objective as 0
					Control_ObjState_Pos[ii+3][MCol(Control_ObjState_Vel)-1]=State_Pos[ii+3][MCol(State_Pos)-1]+_Medium_Float_4[ii][0];
					//New Orientation Translational Position Error is the Original State_Pos Plus the new error
				}
				
				
				
				

				
				
				
				for(ii=0;ii<2;ii++)
				{
					_Medium_Float_1[ii][0]=Control_ObjState_Pos[ii+3][MCol(Control_ObjState_Pos)-2];//Get Current Ground Velocity
					_Medium_Float_2[ii][0]=Control_ObjState_Pos[ii+3][MCol(Control_ObjState_Pos)-1];//Get Objective Ground Velocity
				}					
				
				_Medium_Float_1[2][0]=0;
				_Medium_Float_2[2][0]=0;
				
				MatF_Plus(MAdrF(_Medium_Float_1), MAdrF(_Medium_Float_2), MAdrF(_Store_Float_4), -1.0, 1.0, 
						  MRow(_Medium_Float_1), MCol(_Medium_Float_1), MRow(_Medium_Float_2), MCol(_Medium_Float_2));
				//Get Objective Position Difference
				
				_Norm_Vel_2=sqrtf(powf(Control_Gnd_Vel[2][MCol(Control_Gnd_Vel)-1],2)+powf(Control_Gnd_Vel[2][MCol(Control_Gnd_Vel)-1],2));
				_Norm_Pos_2=sqrtf(powf(_Store_Float_4[0][0],2)+powf(_Store_Float_4[1][0],2));//Norm of Objective Position Error
				
				
				
				
				
				
				if(_Norm_Vel_2<Control_Gnd_DefVel)//If any assigned velocity norm is smaller than default
				{
					_Norm_Vel_2=Control_Gnd_DefVel;
				}
				
				else if	(_Norm_Vel_1>Control_Gnd_MaxVel)
				{
					_Norm_Vel_2=Control_Gnd_MaxVel;//Never Exceed Maximum allow velocity
				}
				
				
				if	(_Norm_Pos_2>(Timer_MiniStep*((float) State_TimeGap[0][MCol(State_TimeGap)-1])*_Norm_Vel_2))
				{
					for(ii=0;ii<2;ii++)
					{
						Control_ObjState_Pos[ii+3][MCol(Control_ObjState_Vel)-1]=Control_ObjState_Pos[ii+3][MCol(Control_ObjState_Vel)-2]
																			  +Timer_MiniStep*((float) State_TimeGap[0][MCol(State_TimeGap)-1])
																			  *_Store_Float_4[ii][0]*_Norm_Vel_2/_Norm_Pos_2;
					}
				}
				else
				{
					Control_Completion_XY=1;
				}
				break;	
		
		default: Sys_Error=6;
				 break;
	}
}

	
	



void _Func_Reference_Update(void)
{	
	if(Sys_Error>0)
	{
		return;
	}
	
	MatF_Wave(MAdrF(Control_Time),MRow(Control_Time),MCol(Control_Time),1,0);	
	MatF_Wave(MAdrF(Control_TimeGap),MRow(Control_TimeGap),MCol(Control_TimeGap),1,0);
	
	Control_Time[0][MCol(Control_Time)-1]=Timer_ClockLow;
	Control_TimeGap[0][MCol(Control_Time)-1]=Control_Time[0][MCol(Control_Time)-1]-Control_Time[0][MCol(Control_Time)-2];
	

	MatF_Wave(MAdrF(Control_Gnd_Vel),MRow(Control_Gnd_Vel),MCol(Control_Gnd_Vel),1,0);//GetReady for New Obj
	MatF_Wave(MAdrF(Control_Gnd_Pos),MRow(Control_Gnd_Pos),MCol(Control_Gnd_Pos),1,0);
	
	_Func_AB_Reference();
	_Func_C_Reference();
	_Func_XY_Reference();
	
}



void _Func_Output_Update(void)
{
	if(Sys_Error>0)
	{
		return;
	}
	
	MatF_Wave(MAdrF(Output_Voltage_Raw),MRow(Output_Voltage_Raw),MCol(Output_Voltage_Raw),1,0);
	
	u8 ii;
	
	float _Medium_1[5][1];
	float _Medium_2[5][1];
	float _Medium_3[5][1];
	float _Medium_4[5][1];
	
	float _Medium_5[5][1];
	float _Medium_6[5][1];
	
	
	float _Output_1[4][1];
	float _Output_2[4][1];
	
	MatF_Cut(MAdrF(Control_ObjState_Pos),MAdrF(_Medium_1),MRow(Control_ObjState_Pos),MCol(Control_ObjState_Pos),1,MCol(Control_ObjState_Pos),MRow(_Medium_1),MCol(_Medium_1));	
	MatF_Cut(MAdrF(Control_ObjState_Vel),MAdrF(_Medium_2),MRow(Control_ObjState_Vel),MCol(Control_ObjState_Vel),1,MCol(Control_ObjState_Vel),MRow(_Medium_2),MCol(_Medium_2));	
	MatF_Cut(MAdrF(State_Pos),MAdrF(_Medium_3),MRow(State_Pos),MCol(State_Pos),1,MCol(State_Pos),MRow(_Medium_3),MCol(_Medium_3));
	MatF_Cut(MAdrF(State_Vel),MAdrF(_Medium_4),MRow(State_Vel),MCol(State_Vel),1,MCol(State_Vel),MRow(_Medium_4),MCol(_Medium_4));
	
	for(ii=0;ii<3;ii++)
	{
		_Medium_1[ii+2][0]=_Medium_1[ii+2][0]*Control_PosAdj[ii];
		_Medium_2[ii+2][0]=_Medium_2[ii+2][0]*Control_VelAdj[ii];
	}
	
	MatF_Plus(MAdrF(_Medium_1),MAdrF(_Medium_3),MAdrF(_Medium_5),1.0,-1.0,MRow(_Medium_1),MCol(_Medium_1),MRow(_Medium_3),MCol(_Medium_3));
	MatF_Plus(MAdrF(_Medium_2),MAdrF(_Medium_4),MAdrF(_Medium_6),1.0,-1.0,MRow(_Medium_2),MCol(_Medium_2),MRow(_Medium_4),MCol(_Medium_4));	
	
	MatF_Mult(MAdrF(Control_Gain_ZP_Pos),MAdrF(_Medium_5),MAdrF(_Output_1),1.0,MRow(Control_Gain_ZP_Pos),MCol(Control_Gain_ZP_Pos),MRow(_Medium_5),MCol(_Medium_5));
	MatF_Mult(MAdrF(Control_Gain_ZP_Vel),MAdrF(_Medium_6),MAdrF(_Output_2),1.0,MRow(Control_Gain_ZP_Vel),MCol(Control_Gain_ZP_Vel),MRow(_Medium_6),MCol(_Medium_6));
	
	for(ii=0;ii<4;ii++)
	{
		Output_Voltage_Raw[0][MCol(Output_Voltage_Raw)-1]=_Output_1[ii][0]+_Output_2[ii][0]+Control_InitOutput_ZP[ii][0];
	}
}

void _Func_InControl(void)
{
	_Func_Reference_Update();
	_Func_Output_Update();
}
