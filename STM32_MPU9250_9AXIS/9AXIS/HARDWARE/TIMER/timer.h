#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"
#include "stm32f10x_tim.h"

/************≈‰÷√≤Œ ˝**********/
#define SYS_TIMER2_SYSCLK_DIV 0//72/(0+1)=71MHZ
#define SYS_TIMER2_CLK_1MHZ   71//72/(71+1)=1MHZ
#define sys_timer2_period     1200//1000*1us=1ms
////////////////////////////////////////////////////////////////////////////////// 	  

//void SYS_Timer1_Configuration(unsigned short period);
void SYS_Timer2_Configuration(unsigned short period);
#endif
