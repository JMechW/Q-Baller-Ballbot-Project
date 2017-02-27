#include "PostControl.h"



void _Func_Output_RateLimit(void)
{	
	if(Sys_Error>0)
	{
		return;
	}
//Limit Riserate to avoid sparks
	u8 ii;
	
	float _Medium_1;
	float _Medium_2;
	
	for (ii=0;ii<4;ii++)
	{
		_Medium_1=Output_Voltage_Raw[ii][MCol(Output_Voltage_Raw)-1]-Output_Voltage_Raw[ii][MCol(Output_Voltage_Raw)-2];
		_Medium_2=Output_Max[ii]*((float) Control_TimeGap[0][MCol(Control_TimeGap)-1])*Timer_MiniStep/Output_RiseTime[ii];
		
		if (fabsf(_Medium_1)>(_Medium_2))
		{
			if (_Medium_1>0) 
			{
				Output_Voltage_Raw[ii][MCol(Output_Voltage_Raw)-1]=Output_Voltage_Raw[ii][MCol(Output_Voltage_Raw)-2]+_Medium_2;
			}
			else
			{
				Output_Voltage_Raw[ii][MCol(Output_Voltage_Raw)-1]=Output_Voltage_Raw[ii][MCol(Output_Voltage_Raw)-2]-_Medium_2;
			}
		}
	}
}


void _Func_Output_MaxLimit(void)
{	
	if(Sys_Error>0)
	{
		return;
	}
	
	u8 ii;
	
	for (ii=0;ii<4;ii++)
	{
		if (fabsf(Output_Voltage_Raw[ii][MCol(Output_Voltage_Raw)-1])>Output_Max[ii])
		{
			if (Output_Voltage_Raw[ii][MCol(Output_Voltage_Raw)-1]>0)
			{
				Output_Voltage_Raw[ii][MCol(Output_Voltage_Raw)-1]=Output_Max[ii];
			}
			else
			{
				Output_Voltage_Raw[ii][MCol(Output_Voltage_Raw)-1]=-Output_Max[ii];
			}
		}
	}
}



void _Func_Output_Deviation(void)
{	
	if(Sys_Error>0)
	{
		return;
	}
//Set Deviation for Real motor motions
	u8 ii;
	
	float _Medium_1;
	float _Medium_2;
	float _Slope_1;
	float _Slope_2;
	
	for (ii=0;ii<4;ii++)
	{
		_Medium_1=Output_Max[ii]-Output_Deviation[ii];
		_Medium_2=Output_Max[ii]-Output_DevSpot[ii];
		_Slope_1=Output_Deviation[ii]/Output_DevSpot[ii];
		_Slope_2=_Medium_1/_Medium_2;
		
		if(Output_Voltage_Raw[ii][MCol(Output_Voltage_Raw)-1]>(Output_DevSpot[ii]))
		{
			Output_Voltage_Medium[ii][MCol(Output_Voltage_Medium)-1]=Output_Deviation[ii]
															  +_Slope_2*(Output_Voltage_Raw[ii][MCol(Output_Voltage_Raw)-1]-Output_DevSpot[ii])/_Medium_2;
		}
		else if(Output_Voltage_Raw[ii][MCol(Output_Voltage_Raw)-1]<(-Output_DevSpot[ii]))
		{
			Output_Voltage_Medium[ii][MCol(Output_Voltage_Medium)-1]=-Output_Deviation[ii]
															  +_Slope_2*(Output_Voltage_Raw[ii][MCol(Output_Voltage_Raw)-1]+Output_DevSpot[ii])/_Medium_2;
		}
		else
		{
			Output_Voltage_Medium[ii][MCol(Output_Voltage_Medium)-1]=_Slope_1*Output_Voltage_Raw[ii][MCol(Output_Voltage_Raw)-1]/Output_DevSpot[ii];
		}		
	}
}





void _Func_OutputLimiter(void)
{
	if(Sys_Error>0)
	{
		return;
	}
	
	u8 ii;
	
	MatF_Wave(MAdrF(Output_Voltage_Final),MRow(Output_Voltage_Final),MCol(Output_Voltage_Final),1,0);
	
	void LimitOutput_RiseRate();
	void LimitOutput_Maximum();
	void LimitOutput_Deviation();
	
	for(ii=0;ii<4;ii++)
	{
		Output_Voltage_Final[ii][MCol(Output_Voltage_Final)-1]=Output_Voltage_Medium[ii][MCol(Output_Voltage_Medium)-1];
	}
}

void _Func_PostControl(void)
{
	_Func_OutputLimiter();
}
