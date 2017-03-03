#ifndef __ENCODER_H
#define __ENCODER_H

#include "DATA.H"
#include "ALGORITHM.H"

void Encoder_Initialization(void);
u32 Encoder_Read(u8 num);
void Encoder_ManualRefill(u8 num, u16 refillamount);

#endif
