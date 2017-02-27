#ifndef __DATA_H
#define __DATA_H

#include "Static.h" //This contains all the static data, including mathematic numbers, control gains, system parameters
#include "Status.h" //This contains the status data of the system concerning safety, mode and sensor feedbacks
#include "Variable.h" //This contains the variable space for extra input storage
#include "Command.h" //This contains the commands related to control and communications

//Storage Type Define
typedef struct{
				char data[16384];//Different Data Share Same Address
				u32	Size[250][2]; 	//Chart indicate the size
				u32 Addr[250];		//Address of the data
				u8	Type[250];		//Type of the data 1~9 type shown above;
			  }Sys_StorageType;

//Data Storage
extern Sys_StorageType Data[2];
extern u32 Data_Cnt[2];		//Count the Data used
			  
#endif
