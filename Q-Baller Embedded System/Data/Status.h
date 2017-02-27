#ifndef __STATUS_H
#define __STATUS_H

#include "sys.h"

//System Status
extern u8 Sys_Error;
extern u8 Sys_Mode;

//IO Status
extern u32 IO_Mode[5];
extern u32 GPIO_Activation[5];

//Timer Status
extern u16 Timer_ARR[7];
extern u16 Timer_PSC[7];

extern u32 Timer_ClockLow;
extern u32 Timer_ClockHigh;
extern float Timer_MiniStep;

//PWM Status
extern u8 PWM_Activation[4];
extern u8 PWM_Status;
	
//Encoder Status
extern u8 Encoder_Activation;
extern u8 Encoder_Status;
extern u16 Encoder_Start[6];
extern s32 Encoder_Cycle_Raw[6];

//Usart Status
extern u32 USART_Freq[3];
extern u32 USART_Baud[3];
extern u8 USART_Status[3];
extern u8 USART_Flag[3];
extern u8 USART_TxStarter[3][3];
extern u8 USART_TxEnder[3][3];
extern u8 USART_RxStarter[3][3];
extern u8 USART_RxEnder[3][3];
extern u8 USART_TxFull[3];
extern u8 USART_RxFull[3];
extern u8 USART_TxReady[3];
extern u8 USART_RxReady[3];

//Gyro Status
extern float Gyro_Acc_Final[3][5];
extern float Gyro_Ang_Final[3][5];
extern float Gyro_Att_Final[3][5];
extern s32 Gyro_AttLoop_Final[3][5];
extern float Gyro_Acc_Offset[3][1];
extern float Gyro_Ang_Offset[3][1];
extern float Gyro_Att_Offset[3][1];
extern float Gyro_Time[3][5];

#endif
