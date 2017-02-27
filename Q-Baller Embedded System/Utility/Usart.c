//Project: Robotic Board Template
//File: pwm.c
//Author: Jiamin Wang
//Revision: 0
//Date: 2017/1/14
//Introduction:
/*
//////////2017/1/14
USART 2 are usually used as the sensor port,
USART 1 is used for interconnection with interface.

The above 2 USART ports has no conflict to other utilities.

Usart 3 initialization and usage will be fulfilled in future updates.

Related Variables:

//////////
*/
#include "Usart.h"

void USART1_IRQHandler(void) 	//Designed for USART Transmission
{
	if(USART1->SR&(1<<5))	//IF GOT THE DATA
	{
		USART_RxBuffer[0]=USART1->DR;
		USART1->SR&=~(1<<5);
		if(USART_RxReady[0]==0)
		{
			if((USART_RxCnt[0]<3))
			{
				if(USART_RxBuffer[0]!=USART_TxStarter[0][USART_RxCnt[0]])
				{
					USART_RxCnt[0]=0;
					USART_RxCheck[0]=0;
					return;
				}
				else
				{
					USART_RxCheck[0]++;
				}
			}
			
			if(USART_RxCnt[0]>2)
			{			
				if(USART_RxBuffer[0]==USART_RxEnder[0][3-USART_RxCheck[0]])
				{
					USART_RxCheck[0]--;
				}
				
				if(USART_RxCheck[0]==0)
				{
					if(USART_RxFull[0]==0)
					{
						USART_RxBuffCnt[0]=USART_RxCnt[0]-5;
						USART_RxReady[0]=1;
					}
					else
					{
						USART_RxBuffCnt[0]=200;
					}
					USART_RxCnt[0]=0;
					USART_RxFull[0]=0;
					return;
				}
				
				if(USART_RxFull[0]==0)
				{
					USART_RxCache[0][USART_RxCnt[0]-3]=USART_RxBuffer[0];
				}
			}
			
			USART_RxCnt[0]++;
			
			if(USART_RxCnt[0]==203)
			{
				USART_RxFull[0]=1;
			}
		}
	}	
}






void USART2_IRQHandler(void)	//Designed for GyroScope
{
	if(USART2->SR&(1<<5))	//IF GOT THE DATA
	{
		USART_RxBuffer[1]=USART2->DR; 
		USART2->SR&=~(1<<5);
		
		if(USART_RxCnt[1]>9)
		{
			if(Gyro_RxCheck==USART_RxBuffer[1])
			{
				u8 ii;
				
				switch (Gyro_RxType)
				{
					case 1:	for(ii=0;ii<6;ii++)
							{
								Gyro_Acc_Buffer[ii]=Gyro_Acc_Raw[ii];
							}
							break;
									
					case 2:	for(ii=0;ii<6;ii++)
							{
								Gyro_Ang_Buffer[ii]=Gyro_Ang_Raw[ii];
							}
							break;
									
					case 3:	for(ii=0;ii<6;ii++)
							{
								Gyro_Att_Buffer[ii]=Gyro_Att_Raw[ii];
							}
							break;
				}
				Gyro_RecCnt[Gyro_RxType-1]++;
				Gyro_Status[Gyro_RxType-1]=1;
			}
				USART_RxCnt[1]=0;
				Gyro_RxCheck=0;
				Gyro_RxType=0;
				return;
		}
			
		if((USART_RxCnt[1]>1)&&(USART_RxCnt[1]<8))
		{
			switch (Gyro_RxType)
			{
				case 1:
						Gyro_Acc_Raw[(USART_RxCnt[1]-2)]=USART_RxBuffer[1];
						break;
					
				case 2:
						Gyro_Ang_Raw[(USART_RxCnt[1]-2)]=USART_RxBuffer[1];
						break;
					
				case 3:
						Gyro_Att_Raw[(USART_RxCnt[1]-2)]=USART_RxBuffer[1];
						break;
			}
				
		}
			
		if(USART_RxCnt[1]==1)//Check Feedback Type
		{
			if ((USART_RxBuffer[1]==0X51)||(USART_RxBuffer[1]==0X52)||(USART_RxBuffer[1]==0X53))
			{
					Gyro_RxType=USART_RxBuffer[1]-0X50;
			}
			else
			{
					USART_RxCnt[1]=0;
					return;
			}
		}
			
		if((USART_RxCnt[1]==0)&&(USART_RxBuffer[1]!=0X55))
		{
			Gyro_RxType=0;
			return;
		}
			
		Gyro_RxCheck=(u8)(Gyro_RxCheck+USART_RxBuffer[1]);		
		USART_RxCnt[1]++;
		
		return;	
	}
}







	

void Usart_Main_Initialization(u32 pclk2,u32 bound)//PCLK2 - OCSILLATOR FREQUENCY (84MHZ);BOUND - BAUDRATE 
{  	 
	float temp;
	u16 mantissa;
	u16 fraction;	   
	
	temp=(float)(pclk2*1000000)/(bound*16);//GET USARTDIV
	mantissa=temp;				 //GET INTEGER PART
	fraction=(temp-mantissa)*16; //GET FRACTION PART	 
	mantissa<<=4;
	mantissa+=fraction; 	//COMBINE TOGETHER
	
	RCC->AHB1ENR|=1<<0;
	RCC->APB2ENR|=1<<4;
	
	GPIO_Set(GPIOA,PIN9|PIN10,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);
	GPIO_AF_Set(GPIOA,9,7);
	GPIO_AF_Set(GPIOA,10,7);
	
	RCC->APB2RSTR|=1<<4;   	//RESET USART1  ON APB2RST
	RCC->APB2RSTR&=~(1<<4);	//STOP RESET SUART1   ON APB2RST 	 
	
	//BAUDRATE
	USART1->BRR=mantissa; 	// BAUDRATE 
	USART1->CR1|=0XC;  	// BIT12 'M' SET 1 TO 9 DATA BIT; ENABLE TE AND RE  
	USART1->CR1|=1<<5;    	//SET TO ENABLE INTERRUPT RXNEIE
	MY_NVIC_Init(1,0,USART1_IRQn,2);//GROUP 2 		
	
	USART1->CR1|=1<<13; //Enable USART
	USART_Status[0]=1;
}










void Usart_Gyro_Initialization(u32 pclk2,u32 bound)//PCLK2 - OCSILLATOR FREQUENCY (84MHZ);BOUND - BAUDRATE 
{  	 
	float temp;
	u16 mantissa;
	u16 fraction;	   
	
	temp=(float)(pclk2*1000000)/(bound*16);//GET USARTDIV
	mantissa=temp;				 //GET INTEGER PART
	fraction=(temp-mantissa)*16; //GET FRACTION PART	 
    mantissa<<=4;
	mantissa+=fraction; 	//COMBINE TOGETHER
	
	RCC->AHB1ENR|=1<<0;
	RCC->APB1ENR|=1<<17;
	
	GPIO_Set(GPIOA,PIN2|PIN3,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);
	GPIO_AF_Set(GPIOA,2,7);
	GPIO_AF_Set(GPIOA,3,7);
	
	RCC->APB1RSTR|=1<<17;   	//RESET USART2  ON APB2RST
	RCC->APB1RSTR&=~(1<<17);	//STOP RESET SUART1   ON APB2RST 	 
	
	//BAUDRATE
 	USART2->BRR=mantissa; 	// BAUDRATE 
	USART2->CR1|=0XC;  	// BIT12 'M' SET 1 TO 9 DATA BIT; ENABLE TE AND RE  
	USART2->CR1|=1<<5;    	//SET TO ENABLE INTERRUPT RXNEIE
	MY_NVIC_Init(0,1,USART2_IRQn,2);//GROUP 2 		
	
	USART2->CR1|=1<<13; //Enable USART
	USART_Status[1]=1;
}


