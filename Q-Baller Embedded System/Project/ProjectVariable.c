#include "ProjectVariable.h"


//System Model Characteristics
const float Model_Ball_Radius=0.1;
const float Model_Wheel_Radius=0.024;

//System Event Timer Frequency
u32 TF_Sensor;
u32 TF_Control;

//Control Informations
u8 Control_Mode_XY;
u8 Control_Mode_C;
u8 Control_Completion_XY;//Indicating that the objective won't change for a while
u8 Control_Completion_C;//Indicating that the objective won't change for a while

float Control_Yaw_Pos[1][5]={0};
float Control_Yaw_Vel[1][5]={0};
float Control_Yaw_DefVel=0.25*3.1415927;
float Control_Yaw_MaxVel=0.25*3.1415927;

float Control_Gnd_Pos[3][5]={0};
float Control_Gnd_Vel[3][5]={0};//Recorded from the Command
float Control_Gnd_MaxAcc=0.0;
float Control_Gnd_MaxVel=0.5;
float Control_Gnd_DefAcc=0.0;
float Control_Gnd_DefVel=0.5;//Default Speed

float Control_ObjState_Pos[5][5]={0};
float Control_ObjState_Vel[5][5]={0};

float Control_PosAdj[3]={1};
float Control_VelAdj[3]={1};

u32 Control_Time[1][5]={0};
u32 Control_TimeGap[1][5]={0};

float Control_Gain_ZP_Pos[4][5]={0};
float Control_Gain_ZP_Vel[4][5]={0};
float Control_InitOutput_ZP[4][1]={0};

//Output Info
float Output_Voltage_Raw[4][5]={0};
float Output_Voltage_Final[4][5]={0};
float Output_Voltage_Medium[4][1]={0};
s8 		Output_Sign[4][5]={0};
u16 	Output_Value[4][5]={0};

float Output_RiseTime[4]={1.0};
float Output_Max[4]={11.1};
float Output_Deviation[4]={2.0};
float Output_DevSpot[4]={1.0};

//Encoder Variables 
u32 Encoder_Count_Final[4][5]={0};
s32 Encoder_Cycle_Final[4][5]={0};
u32 Encoder_Time[1][5]={0};
s32 Encoder_RecordGap[4][5]={0};
float Encoder_Sign[4]={1};

//Motor Variables
float Motor_Pos[4][5]={0};
float Motor_Vel[4][5]={0};
float Motor_Sign[4]={1};


//State Variables
float State_Pos[5][5]={0};
float State_Vel[5][5]={0};
u32 State_Time[1][5]={0};
u32 State_TimeGap[1][5]={0};

//Ground Position Variables
float Ground_Pos[3][5]={0};
float Ground_Vel[3][5]={0};
