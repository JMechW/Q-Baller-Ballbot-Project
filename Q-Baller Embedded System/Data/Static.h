#ifndef __STATIC_H
#define __STATIC_H

#include <stm32f4xx.h>

extern const float PI;
extern const float E;

//OSC  (Unit MHz)
extern const u32 OSC_pllm;
extern const u32 OSC_plln;
extern const u32 OSC_pllp;
extern const u32 OSC_pllq;

//IO Port Setting
extern const u8 IO_PortSetup[5][6];
extern const u8 Timer_ChannelSetup[4][5];

#endif
