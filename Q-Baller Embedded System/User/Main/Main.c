#include "Main.h"

int main(void)
{	
	_Func_Ballbot_Initialization();
	while(Sys_Error==0)
	{
			if (Timer_FlowAct[0]!=0)
			{
				Timer_FlowAct[0]=0;
				
			}
			
			if (Timer_FlowAct[1]!=0)
			{
				Timer_FlowAct[1]=0;
				_Func_Encoder_Record();
				_Func_Motor_Speed();
				_Func_Gyro_Record();
				_Func_PreControl();
				_Func_InControl();
				_Func_PostControl();
				//_Func_Motor_Control();
				
			}
			
			if (Timer_FlowAct[2]!=0)
			{
				Timer_FlowAct[2]=0;
			}
		
	}
}
