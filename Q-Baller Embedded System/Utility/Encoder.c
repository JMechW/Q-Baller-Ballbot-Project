//Project: Robotic Board Template
//File: Encoder.c
//Author: Jiamin Wang
//Revision: 1
//Date: 2017/1/13
//Introduction:
/*
//////////2017/1/13
The Robotic Board supports multiple encoder input:
Encoder 1:TIM2
Encoder 2:TIM5
Encoder 3:TIM4
Encoder 4:TIM3
Encoder 5:TIM1
Encoder 6:TIM8

For all timers above, CH1 and CH2 will be occupied

Related Variable Types:
IO_Mode					Encoder Will Set Port to be Mode 3
Encoder_Activation		Status of Encoder Activation
Encoder_Status			Status of Encoder Functionality
Encoder_ARR				Status; Auto Fill
Encoder_PSC				Status; OSC Division
Encoder_Start			Status; The Starting Counter
Encoder_Count_Raw		Variable; 6*1
Encoder_Cycle_Raw		Variable; 6*1
//////////
*/

#include "Encoder.h"
//Interrupt
void TIM1_UP_TIM10_IRQHandler(void)//Encoder 5
{
	if(TIM1->SR&0X0001)
	{
		if (Encoder_Status&(1<<4))
		{
			if (TIM1->CNT>=Encoder_Start[4])
			{
				Encoder_Cycle_Raw[4]--;
			}
			else
			{
				Encoder_Cycle_Raw[4]++;
			}
		}			
	}
	
	TIM1->SR&=~(1<<0);
}

void TIM8_UP_TIM13_IRQHandler(void)//Encoder 6
{
	if(TIM8->SR&0X0001)
	{
		if (Encoder_Status&(1<<5))
		{
			if (TIM8->CNT>=Encoder_Start[5])
			{
				Encoder_Cycle_Raw[5]--;
			}
			else
			{
				Encoder_Cycle_Raw[5]++;
			}
		}			
	}
	TIM8->SR&=~(1<<0);
}

void TIM2_IRQHandler(void)//Encoder 1
{
	if(TIM2->SR&0X0001)
	{
		if (Encoder_Status&(1<<0))
		{
			if (TIM2->CNT>=Encoder_Start[0])
			{
				Encoder_Cycle_Raw[0]--;
			}
			else
			{
				Encoder_Cycle_Raw[0]++;
			}
		}			
	}
	TIM2->SR&=~(1<<0);
}

void TIM3_IRQHandler(void)//Encoder 4
{
	if(TIM3->SR&0X0001)
	{
		if (Encoder_Status&(1<<3))
		{
			if (TIM3->CNT>=Encoder_Start[3])
			{
				Encoder_Cycle_Raw[3]--;
			}
			else
			{
				Encoder_Cycle_Raw[3]++;
			}
		}			
	}
	TIM3->SR&=~(1<<0);
}

void TIM4_IRQHandler(void)//Encoder 3
{
	if(TIM4->SR&0X0001)
	{
		if (Encoder_Status&(1<<2))
		{
			if (TIM4->CNT>=Encoder_Start[2])
			{
				Encoder_Cycle_Raw[2]--;
			}
			else
			{
				Encoder_Cycle_Raw[2]++;
			}
		}			
	}
	TIM4->SR&=~(1<<0);
}

void TIM5_IRQHandler(void)//Encoder 2
{
	if(TIM5->SR&0X0001)
	{
		if (Encoder_Status&(1<<1))
		{
			if (TIM5->CNT>=Encoder_Start[1])
			{
				Encoder_Cycle_Raw[1]--;
			}
			else
			{
				Encoder_Cycle_Raw[1]++;
			}
		}			
	}
	TIM5->SR&=~(1<<0);
}



//Initialization
void Encoder_Initialization(void)
{
	if((Encoder_Activation&(1<<0))&&((IO_Mode[0]&0xF0)==0)&&((IO_Mode[0]&0xF00)==0))
	{
		RCC->APB1ENR|=1<<0;
		RCC->AHB1ENR|=1<<0;
		RCC->AHB1ENR|=1<<1;
		GPIO_Set(GPIOA,PIN15,
				GPIO_MODE_AF,GPIO_OTYPE_PP,
				GPIO_SPEED_50M,GPIO_PUPD_NONE);
		GPIO_AF_Set(GPIOA,15,1);
		GPIO_Set(GPIOB,PIN3,
				GPIO_MODE_AF,GPIO_OTYPE_PP,
				GPIO_SPEED_50M,GPIO_PUPD_NONE);
		GPIO_AF_Set(GPIOB,3,1);//Set CH1 CH2
		IO_Mode[0]|=3<<4;
		IO_Mode[0]|=3<<8;
		
		TIM2->DIER|=1<<0;	//Enable Interrupt
		TIM2->DIER|=1<<6; 	//Tigger Interrupt
		
		TIM2->ARR=Timer_ARR[0];	
		TIM2->PSC=Timer_PSC[0];

		TIM2->CR1&=~(3<<8);	//Keep Original Frequency
		TIM2->CR1&=~(3<<5);	//Align to Sides
		
		TIM2->CCMR1|=1<<0;	//CC1S Project IC1 to TI1
		TIM2->CCMR1|=1<<8;	//CC2S Project IC2 to TI2
		
		TIM2->CCER&=0XFF55; //Trigger at Upper Impulse
		
		TIM2->CCMR1|=3<<4;	//IC1F FILTER MODE 0011
		TIM2->CCMR1|=3<<12;	//IC2F FILTER MODE 0011
		
		TIM2->SMCR|=3<<0;	//Encoder Mode
		
		TIM2->CNT=Encoder_Start[0];	//Start to Count from the number
		
		TIM2->CR1|=1<<0;	//Enable Timer
		
		MY_NVIC_Init(0,2,TIM2_IRQn,2);
	}
	
	if((Encoder_Activation&(1<<1))&&((IO_Mode[1]&0xF0)==0)&&((IO_Mode[1]&0xF00)==0))
	{
		RCC->APB1ENR|=1<<3;
		RCC->AHB1ENR|=1<<0;
		GPIO_Set(GPIOA,PIN0,
				GPIO_MODE_AF,GPIO_OTYPE_PP,
				GPIO_SPEED_50M,GPIO_PUPD_NONE);
		GPIO_AF_Set(GPIOA,0,2);
		GPIO_Set(GPIOA,PIN0,
				GPIO_MODE_AF,GPIO_OTYPE_PP,
				GPIO_SPEED_50M,GPIO_PUPD_NONE);
		GPIO_AF_Set(GPIOA,1,2);//Set CH1 CH2			
		IO_Mode[1]|=3<<4;
		IO_Mode[1]|=3<<8;
		
		TIM5->DIER|=1<<0;	//Enable Interrupt
		TIM5->DIER|=1<<6; 	//Tigger Interrupt
		
		TIM5->ARR=Timer_ARR[1];	
		TIM5->PSC=Timer_PSC[1];

		TIM5->CR1&=~(3<<8);	//Keep Original Frequency
		TIM5->CR1&=~(3<<5);	//Align to Sides
		
		TIM5->CCMR1|=1<<0;	//CC1S Project IC1 to TI1
		TIM5->CCMR1|=1<<8;	//CC2S Project IC2 to TI2
		
		TIM5->CCER&=0XFF55; //Trigger at Upper Impulse
		
		TIM5->CCMR1|=3<<4;	//IC1F FILTER MODE 0011
		TIM5->CCMR1|=3<<12;	//IC2F FILTER MODE 0011
		
		TIM5->SMCR|=3<<0;	//Encoder Mode
		
		TIM5->CNT=Encoder_Start[1];	//Start to Count from the number
		
		TIM5->CR1|=1<<0;	//Enable Timer
		
		MY_NVIC_Init(0,2,TIM5_IRQn,2);
	}
	
	if((Encoder_Activation&(1<<2))&&((IO_Mode[2]&0xF0)==0)&&((IO_Mode[2]&0xF00)==0))
	{
		RCC->APB1ENR|=1<<2;
		RCC->AHB1ENR|=1<<3;
		GPIO_Set(GPIOD,PIN12,
				GPIO_MODE_AF,GPIO_OTYPE_PP,
				GPIO_SPEED_50M,GPIO_PUPD_NONE);
		GPIO_AF_Set(GPIOD,12,2);
		GPIO_Set(GPIOB,PIN13,
				GPIO_MODE_AF,GPIO_OTYPE_PP,
				GPIO_SPEED_50M,GPIO_PUPD_NONE);
		GPIO_AF_Set(GPIOD,13,2);//Set CH1 CH2				
		IO_Mode[2]|=3<<4;
		IO_Mode[2]|=3<<8;				

		TIM4->DIER|=1<<0;	//Enable Interrupt
		TIM4->DIER|=1<<6; 	//Tigger Interrupt
		
		TIM4->ARR=Timer_ARR[2];	
		TIM4->PSC=Timer_PSC[2];

		TIM4->CR1&=~(3<<8);	//Keep Original Frequency
		TIM4->CR1&=~(3<<5);	//Align to Sides
		
		TIM4->CCMR1|=1<<0;	//CC1S Project IC1 to TI1
		TIM4->CCMR1|=1<<8;	//CC2S Project IC2 to TI2
		
		TIM4->CCER&=0XFF55; //Trigger at Upper Impulse
		
		TIM4->CCMR1|=3<<4;	//IC1F FILTER MODE 0011
		TIM4->CCMR1|=3<<12;	//IC2F FILTER MODE 0011
		
		TIM4->SMCR|=3<<0;	//Encoder Mode
		
		TIM4->CNT=Encoder_Start[2];	//Start to Count from the number
		
		TIM4->CR1|=1<<0;	//Enable Timer
		
		MY_NVIC_Init(0,2,TIM4_IRQn,2);
	}
	
	if((Encoder_Activation&(1<<3))&&((IO_Mode[3]&0xF0)==0)&&((IO_Mode[3]&0xF00)==0))
	{
		RCC->APB1ENR|=1<<1;
		RCC->AHB1ENR|=1<<1;
		GPIO_Set(GPIOB,PIN4,
				GPIO_MODE_AF,GPIO_OTYPE_PP,
				GPIO_SPEED_50M,GPIO_PUPD_NONE);
		GPIO_AF_Set(GPIOB,4,2);
		GPIO_Set(GPIOB,PIN5,
				GPIO_MODE_AF,GPIO_OTYPE_PP,
				GPIO_SPEED_50M,GPIO_PUPD_NONE);
		GPIO_AF_Set(GPIOB,5,2);//Set CH1 CH2				
		IO_Mode[3]|=3<<4;
		IO_Mode[3]|=3<<8;
		
		TIM3->DIER|=1<<0;	//Enable Interrupt
		TIM3->DIER|=1<<6; 	//Tigger Interrupt
		
		TIM3->ARR=Timer_ARR[3];	
		TIM3->PSC=Timer_PSC[3];

		TIM3->CR1&=~(3<<8);	//Keep Original Frequency
		TIM3->CR1&=~(3<<5);	//Align to Sides
		
		TIM3->CCMR1|=1<<0;	//CC1S Project IC1 to TI1
		TIM3->CCMR1|=1<<8;	//CC2S Project IC2 to TI2
		
		TIM3->CCER&=0XFF55; //Trigger at Upper Impulse
		
		TIM3->CCMR1|=3<<4;	//IC1F FILTER MODE 0011
		TIM3->CCMR1|=3<<12;	//IC2F FILTER MODE 0011
		
		TIM3->SMCR|=3<<0;	//Encoder Mode
		
		TIM3->CNT=Encoder_Start[3];	//Start to Count from the number
		
		TIM3->CR1|=1<<0;	//Enable Timer
		
		MY_NVIC_Init(0,2,TIM3_IRQn,2);
	}
	
	if((Encoder_Activation&(1<<4))&&((IO_Mode[0]&0xF)==0)&&((IO_Mode[1]&0xF)==0))
	{
		RCC->APB2ENR|=1<<0;
		RCC->AHB1ENR|=1<<4;
		GPIO_Set(GPIOE,PIN9,
				GPIO_MODE_AF,GPIO_OTYPE_PP,
				GPIO_SPEED_50M,GPIO_PUPD_NONE);
		GPIO_AF_Set(GPIOE,9,1);
		GPIO_Set(GPIOE,PIN11,
				GPIO_MODE_AF,GPIO_OTYPE_PP,
				GPIO_SPEED_50M,GPIO_PUPD_NONE);
		GPIO_AF_Set(GPIOE,11,1);//Set CH1 CH2
		IO_Mode[0]|=3<<0;
		IO_Mode[1]|=3<<0;
		
		TIM1->DIER|=1<<0;	//Enable Interrupt
		TIM1->DIER|=1<<6; 	//Tigger Interrupt
		
		TIM1->ARR=Timer_ARR[4];	
		TIM1->PSC=Timer_PSC[4];

		TIM1->CR1&=~(3<<8);	//Keep Original Frequency
		TIM1->CR1&=~(3<<5);	//Align to Sides
		
		TIM1->CCMR1|=1<<0;	//CC1S Project IC1 to TI1
		TIM1->CCMR1|=1<<8;	//CC2S Project IC2 to TI2
		
		TIM1->CCER&=0XFF55; //Trigger at Upper Impulse
		
		TIM1->CCMR1|=3<<4;	//IC1F FILTER MODE 0011
		TIM1->CCMR1|=3<<12;	//IC2F FILTER MODE 0011
		
		TIM1->SMCR|=3<<0;	//Encoder Mode
		
		TIM1->CNT=Encoder_Start[4];	//Start to Count from the number
		
		TIM1->CR1|=1<<0;	//Enable Timer
		
		MY_NVIC_Init(0,2,TIM1_UP_TIM10_IRQn,2);	//Initialize Interupt Handle
	}
	
	if((Encoder_Activation&(1<<5))&&((IO_Mode[3]&0xF000)==0)&&((IO_Mode[3]&0xF0000)==0))
	{
		RCC->APB2ENR|=1<<1;
		RCC->AHB1ENR|=1<<2;
		GPIO_Set(GPIOC,PIN6,
				GPIO_MODE_AF,GPIO_OTYPE_PP,
				GPIO_SPEED_50M,GPIO_PUPD_NONE);
		GPIO_AF_Set(GPIOC,6,3);
		GPIO_Set(GPIOC,PIN7,
				GPIO_MODE_AF,GPIO_OTYPE_PP,
				GPIO_SPEED_50M,GPIO_PUPD_NONE);
		GPIO_AF_Set(GPIOC,7,3);//Set CH1 CH2
		IO_Mode[3]|=3<<12;
		IO_Mode[3]|=3<<16;
		
		TIM8->DIER|=1<<0;	//Enable Interrupt
		TIM8->DIER|=1<<6; 	//Tigger Interrupt
		
		TIM8->ARR=Timer_ARR[5];	
		TIM8->PSC=Timer_PSC[5];

		TIM8->CR1&=~(3<<8);	//Keep Original Frequency
		TIM8->CR1&=~(3<<5);	//Align to Sides
		
		TIM8->CCMR1|=1<<0;	//CC1S Project IC1 to TI1
		TIM8->CCMR1|=1<<8;	//CC2S Project IC2 to TI2
		
		TIM8->CCER&=0XFF55; //Trigger at Upper Impulse
		
		TIM8->CCMR1|=3<<4;	//IC1F FILTER MODE 0011
		TIM8->CCMR1|=3<<12;	//IC2F FILTER MODE 0011
		
		TIM8->SMCR|=3<<0;	//Encoder Mode
		
		TIM8->CNT=Encoder_Start[5];	//Start to Count from the number
		
		TIM8->CR1|=1<<0;	//Enable Timer
		
		MY_NVIC_Init(0,2,TIM8_UP_TIM13_IRQn,2);	//Initialize Interupt Handle
	}
}


u32 Encoder_Read(u8 num)
{	
	u32 _output;
	switch (num)
	{
		case 1: _output=(u32) TIM2->CNT;
				break;
		case 2: _output=(u32) TIM5->CNT;
				break;
		case 3: _output=(u32) TIM4->CNT;
				break;
		case 4: _output=(u32) TIM3->CNT;
				break;
		case 5: _output=(u32) TIM1->CNT;
				break;
		case 6: _output=(u32) TIM8->CNT;
				break;
		
		default:  Sys_Error=8;
				_output=0xFFFFFFFF;
				break;
	}
	return _output;
}
