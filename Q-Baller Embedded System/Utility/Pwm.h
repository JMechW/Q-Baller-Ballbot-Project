#ifndef __PWM_H
#define __PWM_H

#include "DATA.H"
#include "ALGORITHM.H"

void PWM_Initialization(void);
void PWM_Write(u8 group, u8 port, u16 single);

#endif
