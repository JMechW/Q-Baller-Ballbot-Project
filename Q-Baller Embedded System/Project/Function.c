#include "Function.h"

void _Func_Encoder_Record(void)	//Totally 4 Encoders are used for the Ballbot Project
{	
	if(Sys_Error>0)
	{
		return;
	}
	
	u8 ii;
	
	MatU_Wave(MAdrU(Encoder_Count_Final),MRow(Encoder_Count_Final),MCol(Encoder_Count_Final),1,0);
	MatU_Wave(MAdrU(Encoder_Time),MRow(Encoder_Time),MCol(Encoder_Time),1,0);
	MatS_Wave(MAdrS(Encoder_Cycle_Final),MRow(Encoder_Cycle_Final),MCol(Encoder_Cycle_Final),1,0);
	MatS_Wave(MAdrS(Encoder_RecordGap),MRow(Encoder_RecordGap),MCol(Encoder_RecordGap),1,0);
	
	Encoder_Time[0][MCol(Encoder_Time)-1]=Timer_ClockLow;
	
	for(ii=0;ii<4;ii++)
	{
		Encoder_Count_Final[ii][MCol(Encoder_Count_Final)-1]=Encoder_Read(ii+1);
		Encoder_Cycle_Final[ii][MCol(Encoder_Cycle_Final)-1]=Encoder_Cycle_Raw[ii];
		Encoder_RecordGap[ii][MCol(Encoder_RecordGap)-1]=(Encoder_Cycle_Final[ii][MCol(Encoder_Cycle_Final)-1]-Encoder_Cycle_Final[ii][MCol(Encoder_Cycle_Final)-2])*2000
											+(Encoder_Count_Final[ii][MCol(Encoder_Count_Final)-1]-Encoder_Count_Final[ii][MCol(Encoder_Count_Final)-2]);
	}
	
}


void _Func_Motor_Speed(void)
{	
	if(Sys_Error>0)
	{
		return;
	}
	
	u8 ii;
	
	MatF_Wave(MAdrF(Motor_Pos),MRow(Motor_Pos),MCol(Motor_Pos),1,0);
	MatF_Wave(MAdrF(Motor_Vel),MRow(Motor_Vel),MCol(Motor_Vel),1,0);
	
	for(ii=0;ii<4;ii++)
	{
		Motor_Pos[ii][MCol(Motor_Pos)-1]=Encoder_Sign[ii]*((float)Encoder_Cycle_Final[ii][MCol(Encoder_Cycle_Final)-1])*((float) 2.0)*PI
										+Encoder_Sign[ii]*(((float)Encoder_Count_Final[ii][MCol(Encoder_Count_Final)-1])/((float) 2000.0)-((float)0.5))*((float) 2.0)*PI;
		Motor_Vel[ii][MCol(Motor_Vel)-1]=Encoder_Sign[ii]*(((float)(Encoder_RecordGap[ii][MCol(Encoder_RecordGap)-1]))
										/((float)(Encoder_Time[0][MCol(Encoder_Time)-1]-Encoder_Time[0][MCol(Encoder_Time)-2])))*PI/(((float) 500.0)*Timer_MiniStep)
										-Motor_Vel[ii][MCol(Motor_Vel)-2];
	}
}


void _Func_Gyro_Record(void)
{	
	if(Sys_Error>0)
	{
		return;
	}
	u8 ii;
	if (Gyro_Status[0]>0)
	{
		MatF_Wave(MAdrF(Gyro_Acc_Final),MRow(Gyro_Acc_Final),MCol(Gyro_Acc_Final),1,0);
		
		for(ii=0;ii<3;ii++)
		{	
			Gyro_Acc_Final[ii][MCol(Gyro_Acc_Final)-1]=((float)((s16)((Gyro_Acc_Buffer[(1+ii*2)]<<8)|Gyro_Acc_Buffer[(0+ii*2)])))*(Gyro_Multiplier[0]);
		}
		
		for(ii=0;ii<4;ii++)
		{
			Gyro_Time[0][ii]=Gyro_Time[0][ii+1];
		}
		Gyro_Time[0][MCol(Gyro_Time)-1]=Timer_ClockLow;
		
		Gyro_Status[0]=0;
	}
	
	if (Gyro_Status[1]>0)
	{
		MatF_Wave(MAdrF(Gyro_Ang_Final),MRow(Gyro_Ang_Final),MCol(Gyro_Ang_Final),1,0);
		
		for(ii=0;ii<3;ii++)
		{	
			Gyro_Ang_Final[ii][MCol(Gyro_Ang_Final)-1]=((float)((s16)((Gyro_Ang_Buffer[(1+ii*2)]<<8)|Gyro_Ang_Buffer[(0+ii*2)])))*(Gyro_Multiplier[1]);
		}
		
		for(ii=0;ii<4;ii++)
		{
			Gyro_Time[1][ii]=Gyro_Time[1][ii+1];
		}
		Gyro_Time[1][MCol(Gyro_Time)-1]=Timer_ClockLow;
		
		Gyro_Status[1]=0;
	}
	
	if (Gyro_Status[2]>0)
	{
		MatF_Wave(MAdrF(Gyro_Att_Final),MRow(Gyro_Att_Final),MCol(Gyro_Att_Final),1,0);
		MatF_Wave(MAdrF(Gyro_AttLoop_Final),MRow(Gyro_AttLoop_Final),MCol(Gyro_AttLoop_Final),1,0);
		
		for(ii=0;ii<3;ii++)
		{	
			Gyro_Att_Final[ii][MCol(Gyro_Att_Final)-1]=((float)((s16)((Gyro_Att_Buffer[(1+ii*2)]<<8)|Gyro_Att_Buffer[(0+ii*2)])))*(Gyro_Multiplier[2]);
			if((Gyro_Att_Final[ii][MCol(Gyro_Att_Final)-1]-Gyro_Att_Final[ii][MCol(Gyro_Att_Final)-2]>PI))
			{
				Gyro_AttLoop_Final[ii][MCol(Gyro_AttLoop_Final)-1]=Gyro_AttLoop_Final[ii][MCol(Gyro_AttLoop_Final)-1]-1;
			}
			else if((Gyro_Att_Final[ii][MCol(Gyro_Att_Final)-1]-Gyro_Att_Final[ii][MCol(Gyro_Att_Final)-2]<(-PI)))
			{
				Gyro_AttLoop_Final[ii][MCol(Gyro_AttLoop_Final)-1]=Gyro_AttLoop_Final[ii][MCol(Gyro_AttLoop_Final)-1]+1;
			}
			else
			{
				Gyro_AttLoop_Final[ii][MCol(Gyro_AttLoop_Final)-1]=Gyro_AttLoop_Final[ii][MCol(Gyro_AttLoop_Final)-1];
			}
		}
	
		
		for(ii=0;ii<4;ii++)
		{
			Gyro_Time[2][ii]=Gyro_Time[2][ii+1];
		}
		Gyro_Time[2][MCol(Gyro_Time)-1]=Timer_ClockLow;
		
		Gyro_Status[2]=0;
	}
}

void _Func_Motor_Control(void)
{	
	if(Sys_Error>0)
	{
		return;
	}
	
	u8 ii;
	
	MatF_Wave(MAdrF(Output_Value),MRow(Output_Value),MCol(Output_Value),1,0);
	MatF_Wave(MAdrF(Output_Sign),MRow(Output_Sign),MCol(Output_Sign),1,0);
	
	for(ii=0;ii<4;ii++)
	{
		Output_Value[ii][MCol(Output_Value)-1]=(u16) fabsf(Output_Voltage_Final[ii][MCol(Output_Voltage_Final)-1]/Output_Max[ii]);
		if (Output_Voltage_Final[ii][MCol(Output_Voltage_Final)-1]>0)
		{
			Output_Sign[ii][MCol(Output_Voltage_Final)-1]=(s8) Motor_Sign[ii];
		}
		else
		{
			Output_Sign[ii][MCol(Output_Voltage_Final)-1]=(s8) -Motor_Sign[ii];
		}
	}
	
	for(ii=0;ii<4;ii++)
	{
		if(Output_Sign[ii][MCol(Output_Voltage_Final)-1]>0)
		{
			GPIO_Write(ii,4,1);
			GPIO_Write(ii,5,0);
		}
		else if(Output_Sign[ii][MCol(Output_Voltage_Final)-1]<0)
		{
			GPIO_Write(ii,4,0);
			GPIO_Write(ii,5,1);		
		}
		else
		{
			GPIO_Write(ii,4,0);
			GPIO_Write(ii,5,0);		
		}
		PWM_Write(ii,1,Output_Value[ii][MCol(Output_Value)-1]);
	}
}
