#ifndef __PROJECTVARIABLE_H
#define __PROJECTVARIABLE_H

#include "UTILITY.h"

//System Model Characteristics
extern const float Model_Ball_Radius;
extern const float Model_Wheel_Radius;

//System Event Timer Frequency
extern u32 TF_Sensor;
extern u32 TF_Control;

//Control Informations
extern u8 Control_Mode_XY;
extern u8 Control_Mode_C;
extern u8 Control_Completion_XY;//Indicating that the objective won't change for a while
extern u8 Control_Completion_C;//Indicating that the objective won't change for a while

extern float Control_Yaw_Pos[1][5];
extern float Control_Yaw_Vel[1][5];
extern float Control_Yaw_DefVel;
extern float Control_Yaw_MaxVel;

extern float Control_Gnd_Pos[3][5];
extern float Control_Gnd_Vel[3][5];//Recorded from the Command
extern float Control_Gnd_MaxAcc;
extern float Control_Gnd_MaxVel;
extern float Control_Gnd_DefAcc;
extern float Control_Gnd_DefVel;//Default Speed

extern float Control_ObjState_Pos[5][5];
extern float Control_ObjState_Vel[5][5];

extern float Control_PosAdj[3];
extern float Control_VelAdj[3];

extern u32 Control_Time[1][5];
extern u32 Control_TimeGap[1][5];

extern float Control_Gain_ZP_Pos[4][5];
extern float Control_Gain_ZP_Vel[4][5];
extern float Control_InitOutput_ZP[4][1];

//Output Info
extern float Output_Voltage_Raw[4][5];
extern float Output_Voltage_Final[4][5];
extern float Output_Voltage_Medium[4][1];
extern s8 	Output_Sign[4][5];
extern u16 	Output_Value[4][5];

extern float Output_RiseTime[4];
extern float Output_Max[4];
extern float Output_Deviation[4];
extern float Output_DevSpot[4];

//Encoder Variables 
extern u32 Encoder_Count_Final[4][5];
extern s32 Encoder_Cycle_Final[4][5];
extern u32 Encoder_Time[1][5];
extern s32 Encoder_RecordGap[4][5];
extern float Encoder_Sign[4];

//Motor Variables
extern float Motor_Pos[4][5];
extern float Motor_Vel[4][5];
extern float Motor_Sign[4];


//State Variables
extern float State_Pos[5][5];
extern float State_Vel[5][5];
extern u32 State_Time[1][5];
extern u32 State_TimeGap[1][5];

//Ground Position Variables
extern float Ground_Pos[3][5];
extern float Ground_Vel[3][5];

#endif
