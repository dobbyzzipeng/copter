#include "main.h"
#include "globaldefine.h"



void Bsp_Init(void)
{
	/********各种初始化**********/
	NVIC_Configuration();//中断分组配置
	delay_init();//延时初始化
	uart_init(115200);//串口初始化 115200bps
	LED_Init();//LED配置初始化
	LED1_Flash(3);
	DUOJI_PWM_Configuration();
//	SYS_Timer2_Configuration(sys_timer2_period);//
	printf("sys ok\r\n");
	LED1_ON();
}



