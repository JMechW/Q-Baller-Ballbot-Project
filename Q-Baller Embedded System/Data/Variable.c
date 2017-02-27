#include "Variable.h"

//Global Counter (These global counters are set to facilitate debugging)
s32 gcnt1=0;
s32 gcnt2=0;
s32 gcnt3=0;
s32 gcnt4=0;

//Timer
u16 Timer_FlowClick[10]={0};//Part of Initialization
u16 Timer_FlowRemainder[10]={0};//Part of Initialization
u8 Timer_FlowAct[10]={0};
u8 Timer_Cnt=0; //Timer Counter for looping
u8 Timer_CntMax=0; //Cap of Timer Counter //Part of Initialization

//Buffers
u8 GPIO_Buffer[5]={0};
u16 PWM_Buffer[4][5]={0};
u8 USART_RxBuffer[3]={0};
u8 USART_TxBuffer[3]={0};

//USART
u8 USART_TxCnt[3]={0};
u8 USART_RxCnt[3]={0};
u8 USART_TxCache[3][200]={0};
u8 USART_RxCache[3][200]={0};
u8 USART_TxBuffCnt[3]={0};
u8 USART_RxBuffCnt[3]={0};
u8 USART_TxCheck[3]={0};//Part of Initialization
u8 USART_RxCheck[3]={0};//Part of Initialization

//Gyro
u8 Gyro_RxType=0;
u8 Gyro_RxCheck=0;
u8 Gyro_Status[3]={0};
u8 Gyro_Acc_Raw[6]={0};
u8 Gyro_Ang_Raw[6]={0};
u8 Gyro_Att_Raw[6]={0};
u8 Gyro_Acc_Buffer[6]={0};
u8 Gyro_Ang_Buffer[6]={0};
u8 Gyro_Att_Buffer[6]={0};
u32 Gyro_RecCnt[3]={0};
float Gyro_Multiplier[3]={4.7900390625e-3,1.331580545039619e-4,9.587379924285257e-5};//Part of Initialization
