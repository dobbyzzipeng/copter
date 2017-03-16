#ifndef __CAPTURE_H
#define __CAPTURE_H
#include "sys.h"
#include "stm32f10x_tim.h"
/************配置参数**********/
#define SYS_TIMER1_SYSCLK_DIV 0//72/(0+1)=71MHZ
#define SYS_TIMER1_CLK_1MHZ   71//72/(71+1)=1MHZ
#define sys_timer1_period     0XFFFF//65535*1us=65.535ms

/******/
#define CHAOSHENGBOCAPTUREGPIORT RCC_APB2Periph_GPIOB
#define ECHO_PIN 		GPIO_Pin_13
#define ECHO_GPIORT		GPIOB
#define TRIG_PIN		GPIO_Pin_12
#define TRIG_GPIORT		GPIOB

#define Trig_h			GPIO_SetBits(TRIG_GPIORT,TRIG_PIN);
#define Trig_l			GPIO_ResetBits(TRIG_GPIORT,TRIG_PIN);
//PA0 TIM5输入捕获
#define TRIG_H			GPIO_SetBits(GPIOA,GPIO_Pin_0);
#define TRIG_L			GPIO_ResetBits(GPIOA,GPIO_Pin_0);
////////////////////////////////////////////////////////////////////////////////// 	  

//void SYS_Timer1_Configuration(unsigned short period);
void Chaoshengbo_Capture_Timer1_Configuration(unsigned short period);
void Chaoshengbo_Capture_Timer5_Configuration(unsigned short period);
void chaoshengbotrig(void);
void Chaoshengcapture(void);
void Chaoshengbomeasure(void);
#endif

