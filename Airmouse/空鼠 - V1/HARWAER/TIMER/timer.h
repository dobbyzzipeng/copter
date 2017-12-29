#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"
#include "stm32f10x_tim.h"
/************≈‰÷√≤Œ ˝**********/
#define SYS_TIMER2_SYSCLK_DIV 0//72/(0+1)=72MHZ
#define SYS_TIMER2_CLK_1MHZ   71//72/(71+1)=1MHZ
#define sys_timer2_period     (1000-1)//1000*1us=1ms
//#define sys_timer2_period     0xFFFFFFFF//1000*1us=1ms
//////////////////////////////
typedef struct
{
	uint32_t systimems;
//	uint16_t systimercnt;
}SYSTIME;

typedef struct
{
	float thistime;
	float lasttime;
//	float dot;
}TIMEDOT;
//void SYS_Timer1_Configuration(unsigned short period);
void SYS_Timer2_Configuration(unsigned short period);
float Time_get(void);
float Timedot_get(TIMEDOT *timedot);
void Sys_Timer_RUN(void);

#define microms() Time_get()
#endif
