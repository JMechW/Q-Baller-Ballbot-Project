//Project: Robotic Board Template
//File: pwm.c
//Author: Jiamin Wang
//Revision: 0
//Date: 2017/1/14
//Introduction:
/*
//////////2017/1/14
PWM Features are supported in IO Port Set Group 1 to 4

Group 1: TIM11 TIM21 TIM22 TIM23 TIM24
Group 2: TIM12 TIM51 TIM52 TIM33 TIM34
Group 3: TIM13 TIM41 TIM42 TIM43 TIM44
Group 4: TIM14 TIM31 TIM32 TIM81 TIM82

Different ARR and PSC are used for different timers. 
Encoder and PWM may share same ARR and PSC.

Related Variables:
PWM_Activation[4] 			u8 indicating activation of IO port
PWM_Buffer[4][5] 			buffer for update CCR

PWM_Status u8 indicating status for PWM in timer 1~8:
TIM1 BIT 4
TIM2 BIT 0
TIM3 BIT 3
TIM4 BIT 2
TIM5 BIT 1
TIM8 BIT 5

Timer_ChannelSetup[4][5] 	Port Setup for timer, detailed info for 0xIJ 
							I indicate timer number, J indicate channel number
//////////
*/
#include "Pwm.h"


void PWM_Initialization(void) //initialization of all ports
{

	u8 cnt1;
	u8 cnt2;
	
	u8 activenum;
	u8 modenum;
	
	u8 groupnum;
	u8 channelnum;
	u8 pinnum;
	u8 portnum;
	for (cnt1=0;cnt1<4;cnt1++)
	{
		for (cnt2=0;cnt2<5;cnt2++)
		{
			activenum=(PWM_Activation[cnt1]&(1<<(cnt2)))>>(cnt2);
			modenum=(IO_Mode[cnt1]&(0xF<<((cnt2)*4)))>>(cnt2*4);
			if ((activenum>0)&&(modenum==0))
			{
				portnum=(IO_PortSetup[cnt1][cnt2]&0xF0)>>4;
				pinnum=((IO_PortSetup[cnt1][cnt2]&0x0F));
				
				groupnum=(Timer_ChannelSetup[cnt1][cnt2]&0xF0)>>4;
				channelnum=((Timer_ChannelSetup[cnt1][cnt2]&0x0F));
				
				if((groupnum>1)&&(groupnum<6))
				{
					RCC->APB1ENR|=1<<(groupnum-2);
				}
				else if(groupnum==1)
				{
					RCC->APB2ENR|=1;
				}
				else if(groupnum==8)
				{
					RCC->APB2ENR|=2;
				}				
				
				switch (portnum)
				{
					case 0:GPIO_Set(GPIOA,1<<pinnum,
									GPIO_MODE_AF,GPIO_OTYPE_OD,
									GPIO_SPEED_50M,GPIO_PUPD_NONE);
						   GPIO_AF_Set(GPIOA,pinnum,
									(groupnum<3? 1:(groupnum<6?2:3)));
						   RCC->AHB1ENR|=1<<0;
									break;
					case 1:GPIO_Set(GPIOB,1<<pinnum,
									GPIO_MODE_AF,GPIO_OTYPE_OD,
									GPIO_SPEED_50M,GPIO_PUPD_NONE);
						   GPIO_AF_Set(GPIOA,pinnum,
									(groupnum<3? 1:(groupnum<6?2:3)));
						   RCC->AHB1ENR|=1<<1;
									break;
					case 2:GPIO_Set(GPIOC,1<<pinnum,
									GPIO_MODE_AF,GPIO_OTYPE_OD,
									GPIO_SPEED_50M,GPIO_PUPD_NONE);
						   GPIO_AF_Set(GPIOA,pinnum,
									(groupnum<3? 1:(groupnum<6?2:3)));
						   RCC->AHB1ENR|=1<<2;
									break;
					case 3:GPIO_Set(GPIOD,1<<pinnum,
									GPIO_MODE_AF,GPIO_OTYPE_OD,
									GPIO_SPEED_50M,GPIO_PUPD_NONE);
						   GPIO_AF_Set(GPIOA,pinnum,
									(groupnum<3? 1:(groupnum<6?2:3)));
						   RCC->AHB1ENR|=1<<3;
									break;
					case 4:GPIO_Set(GPIOE,1<<pinnum,
									GPIO_MODE_AF,GPIO_OTYPE_OD,
									GPIO_SPEED_50M,GPIO_PUPD_NONE);
						   GPIO_AF_Set(GPIOA,pinnum,
									(groupnum<3? 1:(groupnum<6?2:3)));
						   RCC->AHB1ENR|=1<<4;
									break;
					default:break;
				}
				
				IO_Mode[cnt1]|=4<<cnt2*4;
				
				switch (groupnum)
				{
					case 1:	PWM_Status|=1<<4;
						
							TIM1->ARR=Timer_ARR[4];
							TIM1->PSC=Timer_PSC[4];
							
							if((channelnum>0)&&(channelnum<3))
							{
								TIM1->CCMR1=0XD<(3+4*(channelnum-1));
							}
							else if((channelnum>2)&&(channelnum<5))
							{
								TIM1->CCMR2=0XD<(3+4*(channelnum-3));
							}
							
							TIM1->CCER|=1<<(4*(channelnum-1));
							TIM1->BDTR|=1<<15;
							TIM1->CR1|=1<<7;
							TIM1->CR1|=1<<0;
							
							break;
					
					case 2:	PWM_Status|=1<<0;				
							
							TIM2->ARR=Timer_ARR[0];
							TIM2->PSC=Timer_PSC[0];
							
							if((channelnum>0)&&(channelnum<3))
							{
								TIM2->CCMR1=0XD<(3+4*(channelnum-1));
							}
							else if((channelnum>2)&&(channelnum<5))
							{
								TIM2->CCMR2=0XD<(3+4*(channelnum-3));
							}
							
							TIM2->CCER|=1<<(4*(channelnum-1));
							TIM2->CR1|=1<<7;
							TIM2->CR1|=1<<0;
							
							break;
							
					case 3: PWM_Status|=1<<3;				
							
							TIM3->ARR=Timer_ARR[3];
							TIM3->PSC=Timer_PSC[3];
							
							if((channelnum>0)&&(channelnum<3))
							{
								TIM3->CCMR1=0XD<(3+4*(channelnum-1));
							}
							else if((channelnum>2)&&(channelnum<5))
							{
								TIM3->CCMR2=0XD<(3+4*(channelnum-3));
							}
							
							TIM3->CCER|=1<<(4*(channelnum-1));
							TIM3->CR1|=1<<7;
							TIM3->CR1|=1<<0;
							
							break;
						
					
						
					case 4: PWM_Status|=1<<2;				
							
							TIM4->ARR=Timer_ARR[2];
							TIM4->PSC=Timer_PSC[2];
							
							if((channelnum>0)&&(channelnum<3))
							{
								TIM4->CCMR1=0XD<(3+4*(channelnum-1));
							}
							else if((channelnum>2)&&(channelnum<5))
							{
								TIM4->CCMR2=0XD<(3+4*(channelnum-3));
							}
							
							TIM4->CCER|=1<<(4*(channelnum-1));
							TIM4->CR1|=1<<7;
							TIM4->CR1|=1<<0;
							
							break;
						
							
							
					case 5: PWM_Status|=1<<1;				
							
							TIM5->ARR=Timer_ARR[1];
							TIM5->PSC=Timer_PSC[1];
							
							if((channelnum>0)&&(channelnum<3))
							{
								TIM5->CCMR1=0XD<(3+4*(channelnum-1));
							}
							else if((channelnum>2)&&(channelnum<5))
							{
								TIM5->CCMR2=0XD<(3+4*(channelnum-3));
							}
							
							TIM5->CCER|=1<<(4*(channelnum-1));
							TIM5->CR1|=1<<7;
							TIM5->CR1|=1<<0;
							
							break;
						
							
							
					case 8:	PWM_Status|=1<<5;					
							
							TIM8->ARR=Timer_ARR[5];
							TIM8->PSC=Timer_PSC[5];
							
							if((channelnum>0)&&(channelnum<3))
							{
								TIM8->CCMR1=0XD<(3+4*(channelnum-1));
							}
							else if((channelnum>2)&&(channelnum<5))
							{
								TIM8->CCMR2=0XD<(3+4*(channelnum-3));
							}
							
							TIM8->CCER|=1<<(4*(channelnum-1));
							TIM8->BDTR|=1<<15;
							TIM8->CR1|=1<<7;
							TIM8->CR1|=1<<0;
							
							break;
						
					default:break;
				}
			}
		}
	}
}

	
	



void PWM_Write(u8 group, u8 port, u16 single)
{
	//Single number indicate 0.1%
	u16 buffer;
	u8 cnt1;
	u8 cnt2;
	u8 groupnum;
	u8 channelnum;
	u8 modenum;
	
	if((group<5)&&(port<6))
	{
		if((group>0)&&(port>0))
		{
			cnt1=group-1;
			cnt2=port-1;
			
			groupnum=(Timer_ChannelSetup[cnt1][cnt2]&0xF0)>>4;
			channelnum=((Timer_ChannelSetup[cnt1][cnt2]&0x0F));	
			modenum=(IO_Mode[cnt1]&(0xF<<((cnt2)*4)))>>((cnt2)*4);
			
			if(modenum==4)
			{
				switch (groupnum)
				{
					case 1: buffer=(u16)((((float) single)/((float) 1000.0))*((float) TIM1->ARR));
							switch(channelnum)
							{
								case 1:TIM1->CCR1=buffer;break;
								case 2:TIM1->CCR2=buffer;break;
								case 3:TIM1->CCR3=buffer;break;
								case 4:TIM1->CCR4=buffer;break;
								default:break;
							}
							break;
							
					case 2: buffer=(u16)((((float) single)/((float) 1000.0))*((float) TIM2->ARR));
							switch(channelnum)
							{
								case 1:TIM2->CCR1=buffer;break;
								case 2:TIM2->CCR2=buffer;break;
								case 3:TIM2->CCR3=buffer;break;
								case 4:TIM2->CCR4=buffer;break;
								default:break;
							}
							break;
							
					case 3: buffer=(u16)((((float) single)/((float) 1000.0))*((float) TIM3->ARR));
							switch(channelnum)
							{
								case 1:TIM3->CCR1=buffer;break;
								case 2:TIM3->CCR2=buffer;break;
								case 3:TIM3->CCR3=buffer;break;
								case 4:TIM3->CCR4=buffer;break;
								default:break;
							}
							break;
							
					case 4: buffer=(u16)((((float) single)/((float) 1000.0))*((float) TIM4->ARR));
							switch(channelnum)
							{
								case 1:TIM4->CCR1=buffer;break;
								case 2:TIM4->CCR2=buffer;break;
								case 3:TIM4->CCR3=buffer;break;
								case 4:TIM4->CCR4=buffer;break;
								default:break;
							}
							break;
							
					case 5: buffer=(u16)((((float) single)/((float) 1000.0))*((float) TIM5->ARR));
							switch(channelnum)
							{
								case 1:TIM5->CCR1=buffer;break;
								case 2:TIM5->CCR2=buffer;break;
								case 3:TIM5->CCR3=buffer;break;
								case 4:TIM5->CCR4=buffer;break;
								default:break;
							}
							break;
							
					case 8: buffer=(u16)((((float) single)/((float) 1000.0))*((float) TIM8->ARR));
							switch(channelnum)
							{
								case 1:TIM8->CCR1=buffer;break;
								case 2:TIM8->CCR2=buffer;break;
								case 3:TIM8->CCR3=buffer;break;
								case 4:TIM8->CCR4=buffer;break;
								default:break;
							}
							break;
						
					default:break;
				}
			}
		}
		
		else
		{
			u8 cnt1start;
			u8 cnt2start;
			u8 cnt1end;
			u8 cnt2end;
			if (group==0)
			{
				cnt1start=0;
				cnt1end=4;
			}
			else
			{
				cnt1start=group-1;
				cnt1end=group;
			}
			
			if (port==0)
			{
				cnt2start=0;
				cnt2end=5;
			}
			else
			{
				cnt2start=port-1;
				cnt2end=port;
			}
			
			for (cnt1=cnt1start;cnt1<cnt1end;cnt1++)
			{
				for (cnt2=cnt2start;cnt2<cnt2end;cnt2++)
				{
					groupnum=(Timer_ChannelSetup[cnt1][cnt2]&0xF0)>>4;
					channelnum=((Timer_ChannelSetup[cnt1][cnt2]&0x0F));	
					
					modenum=(IO_Mode[cnt1]&(0xF<<((cnt2)*4)))>>((cnt2)*4);
					
					single=PWM_Buffer[cnt1][cnt2];
					
					if(modenum==4)
					{
						switch (groupnum)
						{
							case 1: buffer=(u16)((((float) single)/((float) 1000.0))*((float) TIM1->ARR));
									switch(channelnum)
									{
										case 1:TIM1->CCR1=buffer;break;
										case 2:TIM1->CCR2=buffer;break;
										case 3:TIM1->CCR3=buffer;break;
										case 4:TIM1->CCR4=buffer;break;
										default:break;
									}
									break;
									
							case 2: buffer=(u16)((((float) single)/((float) 1000.0))*((float) TIM2->ARR));
									switch(channelnum)
									{
										case 1:TIM2->CCR1=buffer;break;
										case 2:TIM2->CCR2=buffer;break;
										case 3:TIM2->CCR3=buffer;break;
										case 4:TIM2->CCR4=buffer;break;
										default:break;
									}
									break;
									
							case 3: buffer=(u16)((((float) single)/((float) 1000.0))*((float) TIM3->ARR));
									switch(channelnum)
									{
										case 1:TIM3->CCR1=buffer;break;
										case 2:TIM3->CCR2=buffer;break;
										case 3:TIM3->CCR3=buffer;break;
										case 4:TIM3->CCR4=buffer;break;
										default:break;
									}
									break;
									
							case 4: buffer=(u16)((((float) single)/((float) 1000.0))*((float) TIM4->ARR));
									switch(channelnum)
									{
										case 1:TIM4->CCR1=buffer;break;
										case 2:TIM4->CCR2=buffer;break;
										case 3:TIM4->CCR3=buffer;break;
										case 4:TIM4->CCR4=buffer;break;
										default:break;
									}
									break;
									
							case 5: buffer=(u16)((((float) single)/((float) 1000.0))*((float) TIM5->ARR));
									switch(channelnum)
									{
										case 1:TIM5->CCR1=buffer;break;
										case 2:TIM5->CCR2=buffer;break;
										case 3:TIM5->CCR3=buffer;break;
										case 4:TIM5->CCR4=buffer;break;
										default:break;
									}
									break;
									
							case 8: buffer=(u16)((((float) single)/((float) 1000.0))*((float) TIM8->ARR));
									switch(channelnum)
									{
										case 1:TIM8->CCR1=buffer;break;
										case 2:TIM8->CCR2=buffer;break;
										case 3:TIM8->CCR3=buffer;break;
										case 4:TIM8->CCR4=buffer;break;
										default:break;
									}
									break;
								
							default:break;
						}
					}
				}
			}
		}
	}
	else
	{
		Sys_Error=8;
	}
}
