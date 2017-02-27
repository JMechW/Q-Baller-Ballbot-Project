#ifndef __GPIO_H
#define __GPIO_H

#include "DATA.H"
#include "ALGORITHM.H"

void GPIO_Initialization(void);
void GPIO_Write(u8 group,u8 member,u8 single);
u8 GPIO_Read(u8 group,u8 member);

#endif
