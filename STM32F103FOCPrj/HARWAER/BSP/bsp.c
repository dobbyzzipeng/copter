#include "main.h"
#include "globaldefine.h"



void Bsp_Init(void)
{
	/********���ֳ�ʼ��**********/
	NVIC_Configuration();//�жϷ�������
	delay_init();//��ʱ��ʼ��
	uart_init(115200);//���ڳ�ʼ�� 115200bps
	LED_Init();//LED���ó�ʼ��
	LED1_Flash(3);
	DUOJI_PWM_Configuration();
//	SYS_Timer2_Configuration(sys_timer2_period);//
	printf("sys ok\r\n");
	LED1_ON();
}



