#ifndef __PWM_H
#define __PWM_H
#include "sys.h"


#define _MOTOR_PWM_M1			GPIO_Pin_9		 //PC9(M1)
#define _MOTOR_PWM_M2			GPIO_Pin_8		//PC7(M2)
#define _MOTOR_PWM_M3			GPIO_Pin_7		//PC6(M3)
#define _MOTOR_PWM_M4			GPIO_Pin_6		//PC8(M4)
#define _MOTOR_PWM_PORT		GPIOC
//
#define _MOTOR_PWM_SYSCLK_DIV	  0//72/(0+1)=72MHZ TIM8
#define _MOTOR_PWM_CLK_18MHZ  	3//72/(3+1)=18MHZ ¼ÆÊýÆµÂÊ
//
#define _MOTOR_PWM_PERIOD	(unsigned short)(1800-1)//10KHZ
#define _FLY_MAX_OUT 		  (unsigned short)(_MOTOR_PWM_PERIOD+1)
#define _FLY_MIN_OUT 		15
//

void Motor_PWM_Configuration(void);
void Motor_NVIC_Configuration(void);
void Motor_Interrupt(void);
void Motor_Reset(void);
void MOTOR_PWM_ON(void);
void MOTOR_PWM_OFF(void);
void Motor_PWMSET(signed short motorpwm1,signed short motorpwm2,signed short motorpwm3,signed short motorpwm4);
void Motor_PwmValSet(float motorout[]);
signed short Motor_Speed_Scale(float motor_speed_input);
void Motor_PwrAdd(float throttle,float controlout[],float motorout[]);
#endif
