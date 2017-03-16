#ifndef __PWM_H
#define __PWM_H
#include "sys.h"

/**********************DUOJI PWM***********************/
#define _DUOJI_PWM_M1			GPIO_Pin_4		 //PC9(M1)
#define _DUOJI_PWM_M2			GPIO_Pin_5		//PC7(M2)
#define _DUOJI_PWM_M3			GPIO_Pin_0		//PC6(M3)
#define _DUOJI_PWM_M4			GPIO_Pin_1		//PC8(M4)
#define _DUOJI_PWM_PORT			GPIOB

#define _DUOJI_PWM_SYSCLK_DIV	0//72/(0+1)=72MHZ TIM8
#define _DUOJI_PWM_CLK_1MHZ  	71//72/(71+1)=1MHZ 计数频率
#define _DUOJI_PWM_PERIOD       20000

#define DUOJI_PWM1		TIM3->CCR1
#define DUOJI_PWM2		TIM3->CCR2
#define DUOJI_PWM3		TIM3->CCR3
#define DUOJI_PWM4		TIM3->CCR4

void DUOJI_PWM_Configuration(void);
/*************************MOTOR PWM*****************************/
#define _MOTOR_PWM_M1			GPIO_Pin_9		 //PC9(M1)
#define _MOTOR_PWM_M2			GPIO_Pin_8		//PC7(M2)
#define _MOTOR_PWM_M3			GPIO_Pin_7		//PC6(M3)
#define _MOTOR_PWM_M4			GPIO_Pin_6		//PC8(M4)
#define _MOTOR_PWM_PORT			GPIOC
//
#define _MOTOR_PWM_SYSCLK_DIV	0//72/(0+1)=72MHZ TIM8
#define _MOTOR_PWM_CLK_18MHZ  	3//72/(3+1)=18MHZ 计数频率
//
#define _MOTOR_PWM_PERIOD	(unsigned short)(1800-1)//10KHZ
#define _FLY_MAX_OUT 		(unsigned short)(_MOTOR_PWM_PERIOD+1)
#define _FLY_MIN_OUT 		15
//

void _Motor_PWM_Configuration(void);
void _Motor_NVIC_Configuration(void);
void _Motor_Interrupt(void);
void Motor_Reset(void);
void Motor_PWMSET(signed short motorpwm1,signed short motorpwm2,signed short motorpwm3,signed short motorpwm4);
signed short _Motor_Speed_Scale(float motor_speed_input);
#endif
