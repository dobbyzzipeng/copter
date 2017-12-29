#include "led.h"

//////////////////////////////////////////////////////////////////////////////////	   

//初始化PB5和PE5为输出口.并使能这两个口的时钟		    
//LED IO初始化
void LED_Init(void)
{
 
 GPIO_InitTypeDef  GPIO_InitStructure;
 	
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	 //使能PB端口时钟
	
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;				 //LED0-->PB0 端口配置
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
 GPIO_Init(GPIOC, &GPIO_InitStructure);					 //根据设定参数初始化GPIOB0
 GPIO_SetBits(GPIOC,GPIO_Pin_12);						 //PB0 输出高
	
}
 
void LED0_ON(void)
{
  LED0=0;
}

void LED0_OFF(void)
{
  LED0=1;
}

