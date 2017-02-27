#ifndef __MATRIX_H

#define __MATRIX_H

#include "sys.h"

#define MRow(X) 	((u8)(sizeof(X)/sizeof(X[0]))) 	//Fast Calculations for Row Quantity (32 BitNum)
#define MCol(X)		((u8)(sizeof(X[0])/4))			//Fast Calculations for Column Quantity (32 BitNum)
#define MEle(X)		((u32)(sizeof(X))/4)			//Fast Calculation for Element Number (32 BitNum)


#define MAdrS(X)	((s32 *)(&X[0]))				//Float Address Extration
#define MAdrU(X)	((u32 *)(&X[0]))				//Signed 32 bit Address Extration
#define MAdrF(X)	((float *)(&X[0]))				//Unsigned 32 bit Address Extration

void MatF_Cut(float *Matrix, float *MatrixResult, u8 MatrixRow, u8 MatrixCol, u8 MatrixRowStr, u8 MatrixColStr, u8 MatrixResultRow, u8 MatrixResultCol);
void MatF_Comb(float *MatrixAAdd, float *MatrixBAdd, float *MatrixResult, u8 MatrixARow, u8 MatrixACol, u8 MatrixBRow, u8 MatrixBCol, u8 MatrixRow, u8 MatrixCol, u8 type);
void MatF_Wave(float *Matrix, u8 MatrixRow, u8 MatrixCol, u8 Wave, u8 WaveDirection);
void MatF_Mult(float *MatrixAAdd, float *MatrixBAdd, float *MatrixResult, float Const,u8 MatrixARow, u8 MatrixACol, u8 MatrixBRow, u8 MatrixBCol);
void MatF_Plus(float *MatrixAAdd, float *MatrixBAdd, float *MatrixResult, float ConstA, float ConstB,u8 MatrixARow, u8 MatrixACol, u8 MatrixBRow, u8 MatrixBCol);
void MatF_Time(float *Matrix, float Const, u8 MatrixRow, u8 MatrixCol);
void MatF_Diag(float *Matrix, float *MatrixResult, u8 MatrixLength, u8 MatrixRow, u8 MatrixCol);
void MatF_Sum(float *Matrix, float *MatrixResult, u8 MatrixRow, u8 MatrixCol, u8 Type);


void MatS_Cut(s32 *Matrix, s32 *MatrixResult, u8 MatrixRow, u8 MatrixCol, u8 MatrixRowStr, u8 MatrixColStr, u8 MatrixResultRow, u8 MatrixResultCol);
void MatS_Comb(s32 *MatrixAAdd, s32 *MatrixBAdd, s32 *MatrixResult, u8 MatrixARow, u8 MatrixACol, u8 MatrixBRow, u8 MatrixBCol, u8 MatrixRow, u8 MatrixCol, u8 type);
void MatS_Wave(s32 *Matrix, u8 MatrixRow, u8 MatrixCol, u8 Wave, u8 WaveDirection);
void MatS_Mult(s32 *MatrixAAdd, s32 *MatrixBAdd, s32 *MatrixResult, s32 Const,u8 MatrixARow, u8 MatrixACol, u8 MatrixBRow, u8 MatrixBCol);
void MatS_Plus(s32 *MatrixAAdd,s32 *MatrixBAdd, s32 *MatrixResult, s32 ConstA, s32 ConstB,u8 MatrixARow, u8 MatrixACol, u8 MatrixBRow, u8 MatrixBCol);
void MatS_Time(s32 *Matrix, s32 Const, u8 MatrixRow, u8 MatrixCol);
void MatS_Diag(s32 *Matrix, s32 *MatrixResult, u8 MatrixLength, u8 MatrixRow, u8 MatrixCol);
void MatS_Sum(s32 *Matrix, s32 *MatrixResult, u8 MatrixRow, u8 MatrixCol, u8 Type);

void MatU_Cut(u32 *Matrix, u32 *MatrixResult, u8 MatrixRow, u8 MatrixCol, u8 MatrixRowStr, u8 MatrixColStr, u8 MatrixResultRow, u8 MatrixResultCol);
void MatU_Comb(u32 *MatrixAAdd, u32 *MatrixBAdd, u32 *MatrixResult, u8 MatrixARow, u8 MatrixACol, u8 MatrixBRow, u8 MatrixBCol, u8 MatrixRow, u8 MatrixCol, u8 type);
void MatU_Wave(u32 *Matrix, u8 MatrixRow, u8 MatrixCol, u8 Wave, u8 WaveDirection);
void MatU_Mult(u32 *MatrixAAdd, u32 *MatrixBAdd, u32 *MatrixResult, u32 Const,u8 MatrixARow, u8 MatrixACol, u8 MatrixBRow, u8 MatrixBCol);
void MatU_Plus(u32 *MatrixAAdd,u32 *MatrixBAdd, u32 *MatrixResult, u32 ConstA, u32 ConstB,u8 MatrixARow, u8 MatrixACol, u8 MatrixBRow, u8 MatrixBCol);
void MatU_Time(u32 *Matrix, u32 Const, u8 MatrixRow, u8 MatrixCol);
void MatU_Diag(u32 *Matrix, u32 *MatrixResult, u8 MatrixLength, u8 MatrixRow, u8 MatrixCol);
void MatU_Sum(u32 *Matrix, u32 *MatrixResult, u8 MatrixRow, u8 MatrixCol, u8 Type);

#endif
