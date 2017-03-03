//Project: Robotic Board Template
//File: gpio.c
//Author: Jiamin Wang
//Revision: 0
//Date: 2017/1/10
//Introduction:
/*
//////////2017/1/10
The Groups of IO are shown below:
Group 1:E9 A15 B3 B10 B11
Group 2:E11 A0 A1 B0 B1
Group 3:E13 D12 D13 D14 D15
Group 4:E14 B4 B5 C6 C7
Group 5:B9 B8 D8 D9 E5 E6

Related Variable Types:
IO_PortSetup[5][6] - Layout of System Setup 
IO_Mode[5]
	Mode Definition:
	0: Not Used
	1: GPIO In
	2: GPIO Out
	3: ENCODER
	4: PWM
	... To be determined in the future
GPIO_Activation[5]  Determine Wheter IO should be activated
				  Setup Similar to Above
				  
IO_Buffer[5] Output Buffer, clear after outputed u8 0~1 for status
//////////
*/
#include "Gpio.h"

//Initialization of GPIO
void GPIO_Initialization(void)
{
	
//Write GPIO Ports from single number or setup buffer
	u8 cnt1;
	u8 cnt2;
	u8 activenum;
	u8	modenum;
	u8 portnum;
	u8 pinnum;
	for (cnt1=0;cnt1<5;cnt1++)
	{	
		for (cnt2=0;cnt2<6;cnt2++)
		{
			activenum=(GPIO_Activation[cnt1]&(0xF<<(cnt2*4)))>>(cnt2*4);
			modenum=(IO_Mode[cnt1]&(0xF<<((cnt2)*4)))>>((cnt2)*4);
			if ((activenum>0)&&(modenum==0)&&(activenum<3))
			{
				portnum=(IO_PortSetup[cnt1][cnt2]&0xF0)>>4;
				pinnum=((IO_PortSetup[cnt1][cnt2]&0x0F));
				switch (portnum)
				{
					case 0:RCC->AHB1ENR|=1<<0;
						   GPIO_Set(GPIOA,1<<pinnum,
									((activenum<2)? 0:1),((cnt1>3)? 0:1),
									GPIO_SPEED_2M,((cnt1>3)? 1:0));
									break;
					case 1:RCC->AHB1ENR|=1<<1;
						   GPIO_Set(GPIOB,1<<pinnum,
									((activenum<2)? 0:1),((cnt1>3)? 0:1),
									GPIO_SPEED_2M,((cnt1>3)? 1:0));
									break;
					case 2:RCC->AHB1ENR|=1<<2;
						   GPIO_Set(GPIOC,1<<pinnum,
									((activenum<2)? 0:1),((cnt1>3)? 0:1),
									GPIO_SPEED_2M,((cnt1>3)? 1:0));
									break;
					case 3:RCC->AHB1ENR|=1<<3;
						   GPIO_Set(GPIOD,1<<pinnum,
									((activenum<2)? 0:1),((cnt1>3)? 0:1),
									GPIO_SPEED_2M,((cnt1>3)? 1:0));
									break;
					case 4:RCC->AHB1ENR|=1<<4;
						   GPIO_Set(GPIOE,1<<pinnum,
									((activenum<2)? 0:1),((cnt1>3)? 0:1),
									GPIO_SPEED_2M,((cnt1>3)? 1:0));
									break;
					default:break;
				}
				IO_Mode[cnt1]|=activenum<<cnt2*4;
			}
		}
	}
}

void GPIO_Write(u8 group,u8 member,u8 single)
{
	u32 portnum;
	u32 pinnum;
	u8  modenum;
	u8  cnt1;
	u8  cnt2;
	u8  buffer;
	if ((group<6)&&(member<7))
	{
		if ((group>0)&&(member>0))
		{
			cnt1=group-1;
			cnt2=member-1;
			modenum=(IO_Mode[cnt1]&(0xF<<((cnt2)*4)))>>((cnt2)*4);
			if(modenum==2)
			{
				buffer=single;
				portnum=(IO_PortSetup[cnt1][cnt2]&0xF0)>>4;
				pinnum=((IO_PortSetup[cnt1][cnt2]&0x0F));
				switch (portnum)
				{
							case 0:PAout(pinnum)=(buffer>0);
								   break;
							case 1:PBout(pinnum)=(buffer>0);
								   break;
							case 2:PCout(pinnum)=(buffer>0);
								   break;
							case 3:PDout(pinnum)=(buffer>0);
								   break;
							case 4:PEout(pinnum)=(buffer>0);
								   break;
							default:break;
				}
				return;
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
				cnt1end=5;
			}
			else
			{
				cnt1start=group-1;
				cnt1end=group;
			}
			
			if (member==0)
			{
				cnt2start=0;
				cnt2end=6;
			}
			else
			{
				cnt2start=member-1;
				cnt2end=member;
			}
			
			for (cnt1=cnt1start;cnt1<cnt1end;cnt1++)
			{
				for (cnt2=cnt2start;cnt2<cnt2end;cnt2++)
				{
					modenum=(GPIO_Activation[cnt1]&(0xF<<(cnt2*4)))>>(cnt2*4);
					if(modenum==2)
					{
						portnum=(IO_PortSetup[cnt1][cnt2]&0xF0)>>4;
						pinnum=((IO_PortSetup[cnt1][cnt2]&0x0F));
						buffer=(GPIO_Buffer[cnt1]&(1<<cnt2));
						switch (portnum)
						{
							case 0:PAout(pinnum)=(buffer>0);
								   break;
							case 1:PBout(pinnum)=(buffer>0);
								   break;
							case 2:PCout(pinnum)=(buffer>0);
								   break;
							case 3:PDout(pinnum)=(buffer>0);
								   break;
							case 4:PEout(pinnum)=(buffer>0);
								   break;
							default:break;
						}
					}
				}
			}
		}
	}
}


//Read GPIO Ports
u8 GPIO_Read(u8 group,u8 member)
{
	u32 portnum;
	u32 pinnum;
	u8  modenum;
	u8 	cnt1;
	u8 	cnt2;
	cnt1=group-1;
	cnt2=member-1;
	modenum=(IO_Mode[cnt1]&(0xF<<((cnt2)*4)))>>((cnt2)*4);
	u8 buffer;
	if ((modenum<3)&&(modenum>0))
	{
		portnum=(IO_PortSetup[cnt1][cnt2]&0xF0)>>4;
		pinnum=((IO_PortSetup[cnt1][cnt2]&0x0F));
		switch (portnum)
			{
				case 0:buffer=PAin(pinnum);
					   break;
				case 1:buffer=PBin(pinnum);
					   break;
				case 2:buffer=PCin(pinnum);
					   break;
				case 3:buffer=PDin(pinnum);
					   break;
				case 4:buffer=PEin(pinnum);
					   break;
				default:break;
			}
		return buffer;
	}
	else
	{
		Sys_Error=8;
		return 0XFF;
	}
}
