#include "Initialization.h"

void _Func_Ballbot_Initialization(void)
{
	u8 ii;
	//System Timer Activation
	Stm32_Clock_Init(OSC_plln,OSC_pllm,OSC_pllp,OSC_pllq);
	
	//Main Transmitte Serial Activation
	USART_Freq[0]=84;
	USART_Baud[0]=115200;
	USART_TxCheck[0]=0x57;
	USART_RxCheck[0]=0x57;
	USART_TxCheck[0]=0x4A;
	USART_RxCheck[0]=0x4A;
	USART_TxCheck[0]=0x6D;
	USART_RxCheck[0]=0x6D;
	Usart_Main_Initialization(USART_Freq[0],USART_Baud[0]);
	
	//Gyro Serial Activation
	USART_Freq[1]=42;
	USART_Baud[1]=115200;
	Gyro_Multiplier[0]=4.7900390625e-3;
	Gyro_Multiplier[1]=1.331580545039619e-4;
	Gyro_Multiplier[2]=9.587379924285257e-5;
	Usart_Gyro_Initialization(USART_Freq[1],USART_Baud[1]);
	
	//GPIO Activation
	
	GPIO_Activation[0]=0x00022000;
	GPIO_Activation[1]=0x00022000;
	GPIO_Activation[2]=0x00022000;
	GPIO_Activation[3]=0x00022000;
	GPIO_Initialization();
	
	//PWM Activation
	Timer_ARR[4]=1999;
	Timer_PSC[4]=41;
	PWM_Activation[0]=0x01;
	PWM_Activation[1]=0x01;
	PWM_Activation[2]=0x01;
	PWM_Activation[3]=0x01;
	PWM_Initialization();

	
	//Encoder Activation	
	for (ii=0;ii<4;ii++)
	{
		Timer_ARR[ii]=1999;
		Timer_PSC[ii]=41;
		Encoder_Start[ii]=1000;
	}
	
	Encoder_Activation=0x0F;
	Encoder_Initialization();
	
	
	//Project Timer Activation
	Timer_FlowClick[0]=200;	//50  HZ
	Timer_FlowClick[1]=100;	//100 HZ
	Timer_FlowClick[2]=50;	//200 HZ
	Timer_FlowRemainder[0]=0;
	Timer_FlowRemainder[1]=0;
	Timer_FlowRemainder[2]=0;
	
	Timer_CntMax=3;
	Timer_ARR[6]=99;
	Timer_PSC[6]=41;
	Timer_MiniStep=0.0001;	
	Timer_Initialization();
	
	Sys_Error=0;
	Sys_Mode=0;
}
