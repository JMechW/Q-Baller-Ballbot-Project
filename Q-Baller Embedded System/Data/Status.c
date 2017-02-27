#include "Status.h"

//System Status
u8 Sys_Error=0;
u8 Sys_Mode=0;

//IO Status
u32 IO_Mode[5]={0};
u32 GPIO_Activation[5]={0};//Part of Initialization

//Timer Status
u16 Timer_ARR[7]={0};//Part of Initialization
u16 Timer_PSC[7]={0};//Part of Initialization

u32 Timer_ClockLow=0;
u32 Timer_ClockHigh=0;
float Timer_MiniStep=0;//Part of Initialization

//PWM Status
u8 PWM_Activation[4]={0};//Part of Initialization
u8 PWM_Status=0;
	
//Encoder Status
u8 Encoder_Activation=0;//Part of Initialization
u8 Encoder_Status=0;
u16 Encoder_Start[6]={0};//Part of Initialization
s32 Encoder_Cycle_Raw[6]={0};

//Usart Status
u32 USART_Freq[3]={0};
u32 USART_Baud[3]={0};
u8 USART_Status[3]={0};
u8 USART_Flag[3]={0};
u8 USART_TxStarter[3][3]={0};
u8 USART_TxEnder[3][3]={0};
u8 USART_RxStarter[3][3]={0};
u8 USART_RxEnder[3][3]={0};
u8 USART_TxFull[3]={0};
u8 USART_RxFull[3]={0};
u8 USART_TxReady[3]={0};
u8 USART_RxReady[3]={0};

//Gyro Status
float Gyro_Acc_Final[3][5]={0};
float Gyro_Ang_Final[3][5]={0};
float Gyro_Att_Final[3][5]={0};
s32 Gyro_AttLoop_Final[3][5]={0};
float Gyro_Acc_Offset[3][1]={0};
float Gyro_Ang_Offset[3][1]={0};
float Gyro_Att_Offset[3][1]={0};
float Gyro_Time[3][5]={0};
