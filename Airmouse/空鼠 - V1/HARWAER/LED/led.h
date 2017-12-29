#ifndef __LED_H
#define __LED_H	 
#include "sys.h"
/************½Ó¿ÚÅäÖÃ*************/
#define _LED1_					GPIO_Pin_8
#define _LED1_PORT_             GPIOA	
#define _MOTOR_LED_A				GPIO_Pin_15		//PA15
#define _MOTOR_LED_A_PORT		GPIOA
#define _MOTOR_LED_B				GPIO_Pin_9		//PB9
#define _MOTOR_LED_B_PORT		GPIOB
#define _MOTOR_LED_C				GPIO_Pin_5		//PB5
#define _MOTOR_LED_C_PORT		GPIOB
#define _MOTOR_LED_D				GPIO_Pin_2		//PD2
#define _MOTOR_LED_D_PORT		GPIOD

/***********************************/
#define LED1 PAout(8)
#define LED1_TOGGLE()   (_LED1_PORT_->ODR) ^= _LED1_//LED1

void LED_Init(void);
void LED1_ON(void);
void LED1_OFF(void);
void LED1_Flash(u8 i);
void _MOTOR_LED_ON(void);
void _MOTOR_LED_OFF(void);
void _MOTOR_LED_A_TOGGLE(void);
void _MOTOR_LED_B_TOGGLE(void);
void _MOTOR_LED_C_TOGGLE(void);
void _MOTOR_LED_D_TOGGLE(void);
void MOTOR_LED_TOGGLE(void);
#endif
