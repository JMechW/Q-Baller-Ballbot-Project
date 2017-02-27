//Project: Robotic Board - Algorithm Pack - Matrix Functions
//File: Matrix.c
//Author: Jiamin Wang
//Revision: 1
//Date: 2017/2/25
//Introduction:
/*
//////////2017/2/25
Matrix.c Contains Matrix Calculation Functions suitable for signed 32 bit integer, unsigned 32 bit integer and 32 bit float.

Mat$X$_... 	Where F in the place of  $X$ for 32 bit float
			Where S in the place of  $X$ for signed 32 bit integer
			Where S in the place of  $X$ for unsigned 32 bit integer
			
Calculation Macros Defined in Matrix.h:
MRow: The Row Quantity of the Matrix of at most 2 Dimensions; 
MCol: The Column Quantity of the Matrix of at most 2 Dimensions;
MEle: The Element Quantity of the Matrix of at most 2 Dimensions;

MAdrF: Float Initial Address of the matrix;
MAdrS: s32 Initial Address of the matrix;
MAdrU: u32 Initial Address of the matrix;

Functions:
Mat$X$_Cut: Cut out a matrix from the big matrix by providing the size of the big matrix, the size of the cutout and the starting point of the cutout in the big matrix
Mat$X$_Comb: Combine 2 Matrix of same row or column quantities
Mat$X$_Wave: Move the matrix element along row or column back to forward (bottom to upward) only, left the unwaved area unchanged
Mat$X$_Mult: Multiplication of 2 matrixes allowing a magnification real number => Result= Const*MatA*MatB
Mat$X$_Plus: Sum of 2 matrixes: Result=ConstA*MatA+ConstB*MatB
Mat$X$_Time: Multiply a matrix with a constant.;
Mat$X$_Diag: Diagonalize a row or column matrix;
Mat$X$_Sum: Sum of a matrix along row or column or as a whole.
//////////
*/

#include "Matrix.h"

//////////////////////////////////MatrixCalc float
void MatF_Cut(float *Matrix, float *MatrixResult, u8 MatrixRow, u8 MatrixCol, u8 MatrixRowStr, u8 MatrixColStr, u8 MatrixResultRow, u8 MatrixResultCol)
{
	//Matrix Cut Out, get part of the matrix out of a bigger one;
	u8 ii;
	u8 jj;
	
	if ((MatrixRowStr >= 1) && (MatrixColStr >= 1) && (MatrixResultRow >= 1) && (MatrixResultCol >= 1) && ((MatrixResultRow + MatrixRowStr) <= (MatrixRow+1)) && ((MatrixResultCol + MatrixColStr) <= (MatrixCol+1)))
	{
		for (ii = 0; ii<MatrixResultRow; ii++)
		{
			for (jj = 0; jj<MatrixResultCol; jj++)
			{
				*(MatrixResult + ii*MatrixResultCol + jj) = *(Matrix + (ii + MatrixRowStr - 1)*MatrixCol + (jj + MatrixColStr - 1));
			}
		}
	}	
}

	
void MatF_Comb(float *MatrixAAdd, float *MatrixBAdd, float *MatrixResult, u8 MatrixARow, u8 MatrixACol, u8 MatrixBRow, u8 MatrixBCol, u8 MatrixRow, u8 MatrixCol, u8 type)
{
	//Matrix Combine type either through row or through column
	u8 ii;
	u8 jj;

	if ((type==0)&&(MatrixACol==MatrixBCol))
	{
		for(ii=0;ii<(MatrixARow+MatrixBRow-1);ii++)
		{
			for(jj=0;jj<MatrixACol;jj++)
			{
				if(ii<MatrixARow)
				{
					*(MatrixResult+ii*MatrixACol+jj)=*(MatrixAAdd+ii*MatrixACol+jj);
				}
				else
				{
					*(MatrixResult+ii*MatrixACol+jj)=*(MatrixBAdd+(ii-MatrixARow)*MatrixBCol+jj);
				}
			}
		}			
	}
	else if (((type==1)&&(MatrixARow==MatrixBRow)))
	{
		for(ii=0;ii<MatrixARow;ii++)
		{
			for(jj=0;jj<(MatrixACol+MatrixBCol-1);jj++)
			{
				if(jj<MatrixACol)
				{
					*(MatrixResult+ii*(MatrixACol+MatrixBCol-1)+jj)=*(MatrixAAdd+ii*MatrixACol+jj);
				}
				else
				{
					*(MatrixResult+ii*(MatrixACol+MatrixBCol-1)+jj)=*(MatrixBAdd+ii*MatrixBCol+(jj-MatrixACol));
				}
			}
		}					
	}	
}

void MatF_Wave(float *Matrix, u8 MatrixRow, u8 MatrixCol, u8 Wave, u8 WaveDirection)
{
	//move matrix elements to a direction, Only leftward or upward
	u8 ii;
	u8 jj;
	if ((Wave<1)||((Wave>MatrixRow-1)&&(WaveDirection==1))||((Wave>MatrixCol-1)&&(WaveDirection==0)))
	{
		return;
	}
	
	if (WaveDirection==0)//Move Along Row
	{
		for (ii=0;ii<(MatrixCol-Wave);ii++)
		{
			for(jj=0;jj<MatrixRow;jj++)
			{
				*(Matrix+jj*MatrixCol+ii)=*(Matrix+jj*MatrixCol+ii+Wave);
			}
		}
	}
	
	else //Move Along Column
	{
		for (ii=0;ii<(MatrixRow-Wave);ii++)
		{
			for(jj=0;jj<MatrixCol;jj++)
			{
				*(Matrix+ii*MatrixCol+jj)=*(Matrix+ii*(MatrixCol+Wave)+jj);
			}
		}
	}	
}

void MatF_Mult(float *MatrixAAdd, float *MatrixBAdd, float *MatrixResult, float Const,u8 MatrixARow, u8 MatrixACol, u8 MatrixBRow, u8 MatrixBCol)
{
	
	//Matrix Multiplication A*B*(Constant)=Result;
	u8 ii;
	u8 jj;
	u8 kk;
	float Medium=0.0;
	
	if (MatrixACol==MatrixBRow)
	{
		for(ii=0;ii<MatrixARow;ii++)
		{
			for(jj=0;jj<MatrixBCol;jj++)
			{
				for(kk=0;kk<MatrixACol;kk++)
				{
					Medium+=(*(MatrixAAdd+ii*MatrixACol+kk))*(*(MatrixBAdd+kk*MatrixBCol+jj));
				}
				*(MatrixResult+ii*MatrixBCol+jj)=Const*Medium;
				Medium=0.0;
			}
		}
	}
}

void MatF_Plus(float *MatrixAAdd, float *MatrixBAdd, float *MatrixResult, float ConstA, float ConstB,u8 MatrixARow, u8 MatrixACol, u8 MatrixBRow, u8 MatrixBCol)
{
	
	//Matrix Plus or Minum (ca*A+cb*B)=Result;
	u8 ii;
	u8 jj;
	
	if ((MatrixACol==MatrixBCol)&&(MatrixARow==MatrixBRow))
	{
		for(ii=0;ii<MatrixARow;ii++)
		{
			for(jj=0;jj<MatrixACol;jj++)
			{
				*(MatrixResult+ii*MatrixACol+jj)=(ConstA*(*(MatrixAAdd+ii*MatrixACol+jj))+ConstB*(*(MatrixBAdd+ii*MatrixACol+jj)));
			}
		}
	}
}

void MatF_Time(float *Matrix, float Const, u8 MatrixRow, u8 MatrixCol)
{
	
	//Clear the value in a Matrix to 0
		u8 ii;
		u8 jj;
		for(ii=0;ii<MatrixRow;ii++)
		{
			for(jj=0;jj<MatrixCol;jj++)
			{
				*(Matrix+ii*MatrixCol+jj)=Const*(*(Matrix+ii*MatrixCol+jj));
			}
		}
		
}

void MatF_Diag(float *Matrix, float *MatrixResult, u8 MatrixLength, u8 MatrixRow, u8 MatrixCol)
{
	//Diagonize a vector to Matrix
	u8 ii;
	u8 jj;
	u8 kk=0;

	if (((MatrixLength==MatrixRow)&&(MatrixLength<=MatrixCol))||((MatrixLength==MatrixCol)&&(MatrixLength<=MatrixRow)))
	{
		for(ii=0;ii<MatrixRow;ii++)
		{
			for(jj=0;jj<MatrixCol;jj++)
			{
				if(ii==jj)
				{
					*(MatrixResult+ii*MatrixCol+jj)=*(Matrix+kk);
					kk++;
				}
				else
				{
					*(MatrixResult+ii*MatrixCol+jj)=0.0;
				}
			}
		}
	}
}

void MatF_Sum(float *Matrix, float *MatrixResult, u8 MatrixRow, u8 MatrixCol, u8 Type)
{
	

	//Calculate the Sum of a Matrix according to row, column or as a whole
	//Type: 1->Row; 2->Col; else->Whole
	u8 ii;
	u8 jj;
	float Medium=0.0;
	
	if (Type==0)
	{
		for(ii=0;ii<MatrixRow;ii++)
		{
			for(jj=0;jj<MatrixCol;jj++)
			{
				Medium+=*(Matrix+ii*MatrixCol+jj);
			}
			*(MatrixResult+ii)=Medium;
			Medium=0.0; //Turn into a column
		}
	}
	else if (Type==1)
	{
		for(jj=0;jj<MatrixCol;jj++)
		{
			for(ii=0;ii<MatrixRow;ii++)
			{
				Medium+=*(Matrix+ii*MatrixCol+jj);
			}
			*(MatrixResult+jj)=Medium;
			Medium=0.0;// Turn into a row
		}
	}
	else
	{
		for(ii=0;ii<MatrixRow;ii++)
		{
			for(jj=0;jj<MatrixCol;jj++)
			{
				Medium+=*(Matrix+ii*MatrixCol+jj);
			}
		}		
		*(MatrixResult)=Medium;
	}
}
/////////////////////////////////////////////////////////////////



//////////////////////////////////MatrixCalc s32
void MatS_Cut(s32 *Matrix, s32 *MatrixResult, u8 MatrixRow, u8 MatrixCol, u8 MatrixRowStr, u8 MatrixColStr, u8 MatrixResultRow, u8 MatrixResultCol)
{
	//Matrix Cut Out, get part of the matrix out of a bigger one;
	u8 ii;
	u8 jj;
	
	if ((MatrixRowStr >= 1) && (MatrixColStr >= 1) && (MatrixResultRow >= 1) && (MatrixResultCol >= 1) && ((MatrixResultRow + MatrixRowStr) <= (MatrixRow+1)) && ((MatrixResultCol + MatrixColStr) <= (MatrixCol+1)))
	{
		for (ii = 0; ii<MatrixResultRow; ii++)
		{
			for (jj = 0; jj<MatrixResultCol; jj++)
			{
				*(MatrixResult + ii*MatrixResultCol + jj) = *(Matrix + (ii + MatrixRowStr - 1)*MatrixCol + (jj + MatrixColStr - 1));
			}
		}
	}
}

	
void MatS_Comb(s32 *MatrixAAdd, s32 *MatrixBAdd, s32 *MatrixResult, u8 MatrixARow, u8 MatrixACol, u8 MatrixBRow, u8 MatrixBCol, u8 MatrixRow, u8 MatrixCol, u8 type)
{
	//Matrix Combine type either through row or through column
	u8 ii;
	u8 jj;

	if ((type==0)&&(MatrixACol==MatrixBCol))
	{
		for(ii=0;ii<(MatrixARow+MatrixBRow-1);ii++)
		{
			for(jj=0;jj<MatrixACol;jj++)
			{
				if(ii<MatrixARow)
				{
					*(MatrixResult+ii*MatrixACol+jj)=*(MatrixAAdd+ii*MatrixACol+jj);
				}
				else
				{
					*(MatrixResult+ii*MatrixACol+jj)=*(MatrixBAdd+(ii-MatrixARow)*MatrixBCol+jj);
				}
			}
		}			
	}
	else if (((type==1)&&(MatrixARow==MatrixBRow)))
	{
		for(ii=0;ii<MatrixARow;ii++)
		{
			for(jj=0;jj<(MatrixACol+MatrixBCol-1);jj++)
			{
				if(jj<MatrixACol)
				{
					*(MatrixResult+ii*(MatrixACol+MatrixBCol-1)+jj)=*(MatrixAAdd+ii*MatrixACol+jj);
				}
				else
				{
					*(MatrixResult+ii*(MatrixACol+MatrixBCol-1)+jj)=*(MatrixBAdd+ii*MatrixBCol+(jj-MatrixACol));
				}
			}
		}					
	}
}

void MatS_Wave(s32 *Matrix, u8 MatrixRow, u8 MatrixCol, u8 Wave, u8 WaveDirection)
{
	//move matrix elements to a direction, Only leftward or upward
	u8 ii;
	u8 jj;
	if ((Wave<1)||((Wave>MatrixRow-1)&&(WaveDirection==1))||((Wave>MatrixCol-1)&&(WaveDirection==0)))
	{
		return;
	}
	
	if (WaveDirection==0)//Move Along Row
	{
		for (ii=0;ii<(MatrixCol-Wave);ii++)
		{
			for(jj=0;jj<MatrixRow;jj++)
			{
				*(Matrix+jj*MatrixCol+ii)=*(Matrix+jj*MatrixCol+ii+Wave);
			}
		}
	}
	
	else //Move Along Column
	{
		for (ii=0;ii<(MatrixRow-Wave);ii++)
		{
			for(jj=0;jj<MatrixCol;jj++)
			{
				*(Matrix+ii*MatrixCol+jj)=*(Matrix+ii*(MatrixCol+Wave)+jj);
			}
		}
	}
	
}

void MatS_Mult(s32 *MatrixAAdd, s32 *MatrixBAdd, s32 *MatrixResult, s32 Const,u8 MatrixARow, u8 MatrixACol, u8 MatrixBRow, u8 MatrixBCol)
{
	
	//Matrix Multiplication A*B*(Constant)=Result;
	u8 ii;
	u8 jj;
	u8 kk;
	s32 Medium=0;
	
	if (MatrixACol==MatrixBRow)
	{
		for(ii=0;ii<MatrixARow;ii++)
		{
			for(jj=0;jj<MatrixBCol;jj++)
			{
				for(kk=0;kk<MatrixACol;kk++)
				{
					Medium+=(*(MatrixAAdd+ii*MatrixACol+kk))*(*(MatrixBAdd+kk*MatrixBCol+jj));
				}
				*(MatrixResult+ii*MatrixBCol+jj)=Const*Medium;
				Medium=0;
			}
		}
	}
}

void MatS_Plus(s32 *MatrixAAdd, s32 *MatrixBAdd, s32 *MatrixResult, s32 ConstA, s32 ConstB,u8 MatrixARow, u8 MatrixACol, u8 MatrixBRow, u8 MatrixBCol)
{
	
	//Matrix Plus or Minum (ca*A+cb*B)=Result;
	u8 ii;
	u8 jj;
	
	if ((MatrixACol==MatrixBCol)&&(MatrixARow==MatrixBRow))
	{
		for(ii=0;ii<MatrixARow;ii++)
		{
			for(jj=0;jj<MatrixACol;jj++)
			{
				*(MatrixResult+ii*MatrixACol+jj)=(ConstA*(*(MatrixAAdd+ii*MatrixACol+jj))+ConstB*(*(MatrixBAdd+ii*MatrixACol+jj)));
			}
		}
	}
}

void MatS_Time(s32 *Matrix, s32 Const, u8 MatrixRow, u8 MatrixCol)
{
	
	//Clear the value in a Matrix to 0
		u8 ii;
		u8 jj;
		for(ii=0;ii<MatrixRow;ii++)
		{
			for(jj=0;jj<MatrixCol;jj++)
			{
				*(Matrix+ii*MatrixCol+jj)=Const*(*(Matrix+ii*MatrixCol+jj));
			}
		}
		
}

void MatS_Diag(s32 *Matrix, s32 *MatrixResult, u8 MatrixLength, u8 MatrixRow, u8 MatrixCol)
{
	//Diagonize a vector to Matrix
	u8 ii;
	u8 jj;
	u8 kk=0;

	if (((MatrixLength==MatrixRow)&&(MatrixLength<=MatrixCol))||((MatrixLength==MatrixCol)&&(MatrixLength<=MatrixRow)))
	{
		for(ii=0;ii<MatrixRow;ii++)
		{
			for(jj=0;jj<MatrixCol;jj++)
			{
				if(ii==jj)
				{
					*(MatrixResult+ii*MatrixCol+jj)=*(Matrix+kk);
					kk++;
				}
				else
				{
					*(MatrixResult+ii*MatrixCol+jj)=0;
				}
			}
		}
	}
}

void MatS_Sum(s32 *Matrix, s32 *MatrixResult, u8 MatrixRow, u8 MatrixCol, u8 Type)
{
	

	//Calculate the Sum of a Matrix according to row, column or as a whole
	//Type: 1->Row; 2->Col; else->Whole
	u8 ii;
	u8 jj;
	s32 Medium=0;
	
	if (Type==0)
	{
		for(ii=0;ii<MatrixRow;ii++)
		{
			for(jj=0;jj<MatrixCol;jj++)
			{
				Medium+=*(Matrix+ii*MatrixCol+jj);
			}
			*(MatrixResult+ii)=Medium;
			Medium=0; //Turn into a column
		}
	}
	else if (Type==1)
	{
		for(jj=0;jj<MatrixCol;jj++)
		{
			for(ii=0;ii<MatrixRow;ii++)
			{
				Medium+=*(Matrix+ii*MatrixCol+jj);
			}
			*(MatrixResult+jj)=Medium;
			Medium=0;// Turn into a row
		}
	}
	else
	{
		for(ii=0;ii<MatrixRow;ii++)
		{
			for(jj=0;jj<MatrixCol;jj++)
			{
				Medium+=*(Matrix+ii*MatrixCol+jj);
			}
		}		
		*(MatrixResult)=Medium;
	}
}
/////////////////////////////////////////////////////////////////



//////////////////////////////////MatrixCalc u32
void MatU_Cut(u32 *Matrix, u32 *MatrixResult, u8 MatrixRow, u8 MatrixCol, u8 MatrixRowStr, u8 MatrixColStr, u8 MatrixResultRow, u8 MatrixResultCol)
{
	//Matrix Cut Out, get part of the matrix out of a bigger one;
	u8 ii;
	u8 jj;
	
	if ((MatrixRowStr >= 1) && (MatrixColStr >= 1) && (MatrixResultRow >= 1) && (MatrixResultCol >= 1) && ((MatrixResultRow + MatrixRowStr) <= (MatrixRow+1)) && ((MatrixResultCol + MatrixColStr) <= (MatrixCol+1)))
	{
		for (ii = 0; ii<MatrixResultRow; ii++)
		{
			for (jj = 0; jj<MatrixResultCol; jj++)
			{
				*(MatrixResult + ii*MatrixResultCol + jj) = *(Matrix + (ii + MatrixRowStr - 1)*MatrixCol + (jj + MatrixColStr - 1));
			}
		}
	}
}

	
void MatU_Comb(u32 *MatrixAAdd, u32 *MatrixBAdd, u32 *MatrixResult, u8 MatrixARow, u8 MatrixACol, u8 MatrixBRow, u8 MatrixBCol, u8 MatrixRow, u8 MatrixCol, u8 type)
{
	//Matrix Combine type either through row or through column
	u8 ii;
	u8 jj;

	if ((type==0)&&(MatrixACol==MatrixBCol))
	{
		for(ii=0;ii<(MatrixARow+MatrixBRow-1);ii++)
		{
			for(jj=0;jj<MatrixACol;jj++)
			{
				if(ii<MatrixARow)
				{
					*(MatrixResult+ii*MatrixACol+jj)=*(MatrixAAdd+ii*MatrixACol+jj);
				}
				else
				{
					*(MatrixResult+ii*MatrixACol+jj)=*(MatrixBAdd+(ii-MatrixARow)*MatrixBCol+jj);
				}
			}
		}			
	}
	else if (((type==1)&&(MatrixARow==MatrixBRow)))
	{
		for(ii=0;ii<MatrixARow;ii++)
		{
			for(jj=0;jj<(MatrixACol+MatrixBCol-1);jj++)
			{
				if(jj<MatrixACol)
				{
					*(MatrixResult+ii*(MatrixACol+MatrixBCol-1)+jj)=*(MatrixAAdd+ii*MatrixACol+jj);
				}
				else
				{
					*(MatrixResult+ii*(MatrixACol+MatrixBCol-1)+jj)=*(MatrixBAdd+ii*MatrixBCol+(jj-MatrixACol));
				}
			}
		}					
	}
}

void MatU_Wave(u32 *Matrix, u8 MatrixRow, u8 MatrixCol, u8 Wave, u8 WaveDirection)
{
	//move matrix elements to a direction, Only leftward or upward
	u8 ii;
	u8 jj;
	if ((Wave<1)||((Wave>MatrixRow-1)&&(WaveDirection==1))||((Wave>MatrixCol-1)&&(WaveDirection==0)))
	{
		return;
	}
	
	if (WaveDirection==0)//Move Along Row
	{
		for (ii=0;ii<(MatrixCol-Wave);ii++)
		{
			for(jj=0;jj<MatrixRow;jj++)
			{
				*(Matrix+jj*MatrixCol+ii)=*(Matrix+jj*MatrixCol+ii+Wave);
			}
		}
	}
	
	else //Move Along Column
	{
		for (ii=0;ii<(MatrixRow-Wave);ii++)
		{
			for(jj=0;jj<MatrixCol;jj++)
			{
				*(Matrix+ii*MatrixCol+jj)=*(Matrix+ii*(MatrixCol+Wave)+jj);
			}
		}
	}
	
}

void MatU_Mult(u32 *MatrixAAdd, u32 *MatrixBAdd, u32 *MatrixResult, u32 Const,u8 MatrixARow, u8 MatrixACol, u8 MatrixBRow, u8 MatrixBCol)
{
	
	//Matrix Multiplication A*B*(Constant)=Result;
	u8 ii;
	u8 jj;
	u8 kk;
	u32 Medium=0;
	
	if (MatrixACol==MatrixBRow)
	{
		for(ii=0;ii<MatrixARow;ii++)
		{
			for(jj=0;jj<MatrixBCol;jj++)
			{
				for(kk=0;kk<MatrixACol;kk++)
				{
					Medium+=(*(MatrixAAdd+ii*MatrixACol+kk))*(*(MatrixBAdd+kk*MatrixBCol+jj));
				}
				*(MatrixResult+ii*MatrixBCol+jj)=Const*Medium;
				Medium=0;
			}
		}
	}
}

void MatU_Plus(u32 *MatrixAAdd, u32 *MatrixBAdd, u32 *MatrixResult, u32 ConstA, u32 ConstB,u8 MatrixARow, u8 MatrixACol, u8 MatrixBRow, u8 MatrixBCol)
{
	
	//Matrix Plus or Minum (ca*A+cb*B)=Result;
	u8 ii;
	u8 jj;
	
	if ((MatrixACol==MatrixBCol)&&(MatrixARow==MatrixBRow))
	{
		for(ii=0;ii<MatrixARow;ii++)
		{
			for(jj=0;jj<MatrixACol;jj++)
			{
				*(MatrixResult+ii*MatrixACol+jj)=(ConstA*(*(MatrixAAdd+ii*MatrixACol+jj))+ConstB*(*(MatrixBAdd+ii*MatrixACol+jj)));
			}
		}
	}
}

void MatU_Time(u32 *Matrix, u32 Const, u8 MatrixRow, u8 MatrixCol)
{
	
	//Clear the value in a Matrix to 0
		u8 ii;
		u8 jj;
		for(ii=0;ii<MatrixRow;ii++)
		{
			for(jj=0;jj<MatrixCol;jj++)
			{
				*(Matrix+ii*MatrixCol+jj)=Const*(*(Matrix+ii*MatrixCol+jj));
			}
		}
		
}

void MatU_Diag(u32 *Matrix, u32 *MatrixResult, u8 MatrixLength, u8 MatrixRow, u8 MatrixCol)
{
	//Diagonize a vector to Matrix
	u8 ii;
	u8 jj;
	u8 kk=0;

	if (((MatrixLength==MatrixRow)&&(MatrixLength<=MatrixCol))||((MatrixLength==MatrixCol)&&(MatrixLength<=MatrixRow)))
	{
		for(ii=0;ii<MatrixRow;ii++)
		{
			for(jj=0;jj<MatrixCol;jj++)
			{
				if(ii==jj)
				{
					*(MatrixResult+ii*MatrixCol+jj)=*(Matrix+kk);
					kk++;
				}
				else
				{
					*(MatrixResult+ii*MatrixCol+jj)=0;
				}
			}
		}
	}
}

void MatU_Sum(u32 *Matrix, u32 *MatrixResult, u8 MatrixRow, u8 MatrixCol, u8 Type)
{
	

	//Calculate the Sum of a Matrix according to row, column or as a whole
	//Type: 1->Row; 2->Col; else->Whole
	u8 ii;
	u8 jj;
	u32 Medium=0;
	
	if (Type==0)
	{
		for(ii=0;ii<MatrixRow;ii++)
		{
			for(jj=0;jj<MatrixCol;jj++)
			{
				Medium+=*(Matrix+ii*MatrixCol+jj);
			}
			*(MatrixResult+ii)=Medium;
			Medium=0; //Turn into a column
		}
	}
	else if (Type==1)
	{
		for(jj=0;jj<MatrixCol;jj++)
		{
			for(ii=0;ii<MatrixRow;ii++)
			{
				Medium+=*(Matrix+ii*MatrixCol+jj);
			}
			*(MatrixResult+jj)=Medium;
			Medium=0;// Turn into a row
		}
	}
	else
	{
		for(ii=0;ii<MatrixRow;ii++)
		{
			for(jj=0;jj<MatrixCol;jj++)
			{
				Medium+=*(Matrix+ii*MatrixCol+jj);
			}
		}		
		*(MatrixResult)=Medium;
	}
}
/////////////////////////////////////////////////////////////////


