#ifndef __ATTITUDE_H

#define __ATTITUDE_H

#include "sys.h"

//Attitude in Euler Angle G->(Yaw)->O->(PITCH&ROLL)->L
extern float Euler_LtoG[3][3];	//PITCH ROLL YAW
extern float Euler_GtoL[3][3];	//PITCH ROLL YAW
extern float Euler_OtoG[3][3];	//YAW
extern float Euler_GtoO[3][3];	//YAW
extern float Euler_LtoO[3][3];	//PITCH ROLL
extern float Euler_OtoL[3][3];	//PITCH ROLL
extern float Euler_Jacobian[3][3];	//Jacobian Matrix
extern float Euler_InvJacobian[3][3];	//Jacobian Matrix Inverse

void Att_Euler(float RollA, float PitchB, float YawC, u8 Type);

#endif
