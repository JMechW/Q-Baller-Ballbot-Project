//Project: Robotic Board Template
//File: Timer.c
//Author: Jiamin Wang
//Revision: 0
//Date: 2017/1/10
//Introduction:
/*
//////////2017/1/10
Timer is used to calculate system time. Use TIM

Related Variable Types:
Timer_ARR;       	
Timer_PSC;
Timer_ClockLow;
Timer_ClockHigh;
Timer_FlowClick[10]; u16 Indicate the status of the loop, 
Timer_FlowAct[10];  Flag for the loop to be running

Timer_TimeStep;		Determine the minimum Time Setup
//////////
*/

#include "Timer.h"

void TIM7_IRQHandler(void) //TIM7 as System Timer
{
	if(TIM7->SR==1)
	{
		TIM7->SR=0;
		
		if(Timer_ClockLow==0XFFFFFFFF)
		{
				Timer_ClockHigh++;
				Timer_ClockLow=0;
		}
		else
		{
				Timer_ClockLow++;
		}
			
		for (Timer_Cnt=0;Timer_Cnt<Timer_CntMax;Timer_Cnt++)
		{
			if ((Timer_ClockLow%(Timer_FlowClick[Timer_Cnt]))==Timer_FlowRemainder[Timer_Cnt])
			{
					Timer_FlowAct[Timer_Cnt]=1;
			}
		}
		
		return;
	}
}





void Timer_Initialization(void)
{
	RCC->APB1ENR|=1<<5;
	
	TIM7->ARR=Timer_ARR[6];//usually set as 999 for 1MHz 1ms
	TIM7->PSC=Timer_PSC[6];//usually set as 41 for 1MHz 1ms
	
	TIM7->DIER=1<<0;//Enable Update Interupt
	TIM7->CR1=0X01;//Enable Timer
	
	MY_NVIC_Init(0,0,TIM7_IRQn,2); //Highest Priority of all Interupts
}




