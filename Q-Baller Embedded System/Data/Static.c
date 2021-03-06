#include "Static.h"

const float PI=3.1415926;
const float E=2.7182818;

//Extern OSC
const u32 OSC_pllm=25; //MHz
const u32 OSC_plln=336; //MHz
const u32 OSC_pllp=2;
const u32 OSC_pllq=7;

const u8 IO_PortSetup[5][6]={{0x49,0x0F,0x13,0x1A,0x1B,0xFF},
							{0x4B,0x00,0x01,0x10,0x11,0xFF},
							{0x4D,0x3C,0x3D,0x3E,0x3F,0xFF},	
							{0x4E,0x14,0x15,0x26,0x27,0xFF},
							{0x19,0x18,0x38,0x39,0x45,0x46}};

const u8 Timer_ChannelSetup[4][5]=	{{0x11,0x21,0x22,0x23,0x24},
									{0x12,0x51,0x52,0x33,0x34},
									{0x13,0x41,0x42,0x43,0x44},	
									{0x14,0x31,0x32,0x81,0x82}};
