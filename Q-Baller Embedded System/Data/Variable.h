#ifndef __VARIABLE_H
#define __VARIABLE_H

#include "sys.h"

//Global Counter (These global counters are set to facilitate debugging)
extern s32 gcnt1;
extern s32 gcnt2;
extern s32 gcnt3;
extern s32 gcnt4;

//Timer
extern u16 Timer_FlowClick[10];//Part of Initialization
extern u16 Timer_FlowRemainder[10];//Part of Initialization
extern u8 Timer_FlowAct[10];
extern u8 Timer_Cnt; //Timer Counter for looping
extern u8 Timer_CntMax; //Timer Counter 

//Buffers
extern u8 GPIO_Buffer[5];
extern u16 PWM_Buffer[4][5];
extern u8 USART_RxBuffer[3];
extern u8 USART_TxBuffer[3];

//USART
extern u8 USART_TxCnt[3];
extern u8 USART_RxCnt[3];
extern u8 USART_TxCache[3][200];
extern u8 USART_RxCache[3][200];
extern u8 USART_TxBuffCnt[3];
extern u8 USART_RxBuffCnt[3];
extern u8 USART_TxCheck[3];
extern u8 USART_RxCheck[3];

//Gyro
extern u8 Gyro_RxType;
extern u8 Gyro_RxCheck;
extern u8 Gyro_Status[3];
extern u8 Gyro_Acc_Raw[6];
extern u8 Gyro_Ang_Raw[6];
extern u8 Gyro_Att_Raw[6];
extern u8 Gyro_Acc_Buffer[6];
extern u8 Gyro_Ang_Buffer[6];
extern u8 Gyro_Att_Buffer[6];
extern u32 Gyro_RecCnt[3];
extern float Gyro_Multiplier[3];

#endif
