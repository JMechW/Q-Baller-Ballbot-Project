//Project: Robotic Board - Algorithm Pack - Attitude Functions
//File: Attitude.c
//Author: Jiamin Wang
//Revision: 1
//Date: 2017/2/25
//Introduction:
/*
//////////2017/2/25
Currently Attitude.c only contains the calculation of frequently used Euler Matrixes;

Explanations about the function are marked below.

In the future, more algorithms related to attitude control may be included in the collection.
//////////
*/

#include "Attitude.h"

float Euler_LtoG[3][3]={{1,0,0},{0,1,0},{0,0,1}};	//PITCH ROLL YAW
float Euler_GtoL[3][3]={{1,0,0},{0,1,0},{0,0,1}};	//PITCH ROLL YAW
float Euler_OtoG[3][3]={{1,0,0},{0,1,0},{0,0,1}};	//YAW
float Euler_GtoO[3][3]={{1,0,0},{0,1,0},{0,0,1}};	//YAW
float Euler_LtoO[3][3]={{1,0,0},{0,1,0},{0,0,1}};	//PITCH ROLL
float Euler_OtoL[3][3]={{1,0,0},{0,1,0},{0,0,1}};	//PITCH ROLL
float Euler_Jacobian[3][3]={{1,0,0},{0,1,0},{0,0,1}};	//Jacobian Matrix
float Euler_InvJacobian[3][3]={{1,0,0},{0,1,0},{0,0,1}};	//Jacobian Matrix Inverse


void Att_Euler(float RollA, float PitchB, float YawC, u8 Type)
{
	//Calculate a special Euler Matrix, Update Wanted Euler Matrixes or Update them All
	//Type Bit: 0-> G to L; 1-> L to G; 2-> G to O; 3 -> O to G; 4 -> O to L; 5 -> L to O; 6 -> Jacobian; 7 ->Jacobian Inverse;
	
	//float MatA[3][3]={{1,0,0},{0,cosf(RollA),(-sinf(RollA))},{0,sinf(RollA),cosf(RollA)}};
	if (Type==0)
	{
		return;
	}
	else
	{	
		u8 ii;
		u8 jj;
		if ((Type&(1<<0)))
		{
			float _Medium0[3][3]={{cosf(PitchB)*cosf(YawC), cosf(PitchB)*sinf(YawC), -sinf(PitchB)}, 
								 {cosf(YawC)*sinf(PitchB)*sinf(RollA) - cosf(RollA)*sinf(YawC), cosf(RollA)*cosf(YawC) + sinf(PitchB)*sinf(RollA)*sinf(YawC), cosf(PitchB)*sinf(RollA)}, 
								 {sinf(RollA)*sinf(YawC) + cosf(RollA)*cosf(YawC)*sinf(PitchB), cosf(RollA)*sinf(PitchB)*sinf(YawC) - cosf(YawC)*sinf(RollA), cosf(PitchB)*cosf(RollA)}};
									
			for (ii=0;ii<3;ii++)
			{
				for(jj=0;jj<3;jj++)
				{
					Euler_GtoL[ii][jj]=_Medium0[ii][jj];
				}
			}
			
		}
		if ((Type&(1<<1)))
		{
			float _Medium1[3][3]={{cosf(PitchB)*cosf(YawC), cosf(YawC)*sinf(PitchB)*sinf(RollA) - cosf(RollA)*sinf(YawC), sinf(RollA)*sinf(YawC) + cosf(RollA)*cosf(YawC)*sinf(PitchB)},
								 {cosf(PitchB)*sinf(YawC), cosf(RollA)*cosf(YawC) + sinf(PitchB)*sinf(RollA)*sinf(YawC), cosf(RollA)*sinf(PitchB)*sinf(YawC) - cosf(YawC)*sinf(RollA)}, 
								 {-sinf(PitchB), cosf(PitchB)*sinf(RollA), cosf(PitchB)*cosf(RollA)}};
			for (ii=0;ii<3;ii++)
			{
				for(jj=0;jj<3;jj++)
				{
					Euler_LtoG[ii][jj]=_Medium1[ii][jj];
				}
			}
		}
		if ((Type&(1<<2)))
		{
			float _Medium2[3][3]={{cosf(YawC),sinf(YawC),0.0},
								 {-sinf(YawC),cosf(YawC),0.0},
								 {0.0,0.0,1.0}};
				
			for (ii=0;ii<3;ii++)
			{
				for(jj=0;jj<3;jj++)
				{
					Euler_GtoO[ii][jj]=_Medium2[ii][jj];
				}
			}			
		}
		if ((Type&(1<<3)))
		{
			float _Medium3[3][3]={{cosf(YawC),-sinf(YawC),0.0},
								 {sinf(YawC),cosf(YawC),0.0},
								 {0.0,0.0,1.0}};
			for (ii=0;ii<3;ii++)
			{
				for(jj=0;jj<3;jj++)
				{
					Euler_OtoG[ii][jj]=_Medium3[ii][jj];
				}
			}			
		}
		if ((Type&(1<<4)))
		{	
			float _Medium4[3][3]={{cosf(PitchB), sinf(PitchB)*sinf(RollA), cosf(RollA)*sinf(PitchB)},
								 {0, cosf(RollA), -sinf(RollA)},
								 {-sinf(PitchB), cosf(PitchB)*sinf(RollA), cosf(PitchB)*cosf(RollA)}};
			for (ii=0;ii<3;ii++)
			{
				for(jj=0;jj<3;jj++)
				{
					Euler_OtoL[ii][jj]=_Medium4[ii][jj];
				}
			}			
		}
		if ((Type&(1<<5)))
		{
			float _Medium5[3][3]={{cosf(PitchB), 0.0, -sinf(PitchB)},
								 {sinf(PitchB)*sinf(RollA), cosf(RollA), cosf(PitchB)*sinf(RollA)},
								 {cosf(RollA)*sinf(PitchB), -sinf(RollA), cosf(PitchB)*cosf(RollA)}};
			for (ii=0;ii<3;ii++)
			{
				for(jj=0;jj<3;jj++)
				{
					Euler_LtoO[ii][jj]=_Medium5[ii][jj];
				}
			}			
		}
		if ((Type&(1<<6)))
		{
			float _Medium6[3][3]={{1.0, 0.0, -sinf(PitchB)},
								 {0.0, cosf(RollA), cosf(PitchB)*sinf(RollA)},
								 {0.0, -sinf(RollA), cosf(PitchB)*cosf(RollA)}};
			for (ii=0;ii<3;ii++)
			{
				for(jj=0;jj<3;jj++)
				{
					Euler_Jacobian[ii][jj]=_Medium6[ii][jj];
				}
			}			
		}
		if ((Type&(1<<7)))
		{
			float _Medium7[3][3]={{1.0, tanf(PitchB)*sinf(RollA), cosf(RollA)*tanf(PitchB)},
								 {0.0, cosf(RollA), -sinf(RollA)},
								 {0.0, sinf(RollA)/cosf(PitchB), cosf(RollA)/cosf(PitchB)}};
			for (ii=0;ii<3;ii++)
			{
				for(jj=0;jj<3;jj++)
				{
					Euler_InvJacobian[ii][jj]=_Medium7[ii][jj];
				}
			}			
		}
	}
	
}
