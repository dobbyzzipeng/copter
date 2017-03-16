#include "led.h"
#include "delay.h"


////////////////////////////////////////////////////////////////////////////////// 	   

//INITCIAL FOR LED IO

void LED_Init(void)
{
	  GPIO_InitTypeDef  GPIO_InitStructure;
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOD, ENABLE);
	 
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//使能AFIO时钟		
	  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);	//使能SWD下载线
		
	 GPIO_InitStructure.GPIO_Pin = _MOTOR_LED_B|_MOTOR_LED_C;				
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
	 GPIO_Init(_MOTOR_LED_B_PORT, &GPIO_InitStructure);					 //根据设定参数初始化
	 GPIO_ResetBits(_MOTOR_LED_B_PORT,_MOTOR_LED_B|_MOTOR_LED_C);						 
		
	 GPIO_InitStructure.GPIO_Pin = _LED1_|_MOTOR_LED_A;				 //
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
	 GPIO_Init(_LED1_PORT_, &GPIO_InitStructure);					 //根据设定参数初始化
	 GPIO_ResetBits(_LED1_PORT_,_MOTOR_LED_A);				
	 GPIO_SetBits(_LED1_PORT_,_LED1_);
	
	 GPIO_InitStructure.GPIO_Pin = _MOTOR_LED_D;				 //
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
	 GPIO_Init(_MOTOR_LED_D_PORT, &GPIO_InitStructure);					 //根据设定参数初始化
	 GPIO_ResetBits(_MOTOR_LED_D_PORT,_MOTOR_LED_D);		
}

void DinPortInit(void)
{
	  GPIO_InitTypeDef  GPIO_InitStructure;
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	 
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//使能AFIO时钟		
	  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);	//使能SWD下载线
	
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;				 //
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
	 GPIO_Init(GPIOA, &GPIO_InitStructure);					 //根据设定参数初始化
	 GPIO_ResetBits(GPIOA,GPIO_Pin_1);	
}

void LED1_ON(void)
{
	LED1=0;
}

void LED1_OFF(void)
{
	LED1=1;
}

void LED1_Flash(u8 i)
{
  u8 j;
	for(j=i;j>0;j--)
	{
		LED1=0;//OFF
		delay_ms(100);
		LED1=1;//ON
		delay_ms(100);
  }
}

void _MOTOR_LED_ON(void)
{
	GPIO_SetBits(_MOTOR_LED_A_PORT, _MOTOR_LED_A);		//点亮LED
	GPIO_SetBits(_MOTOR_LED_B_PORT, _MOTOR_LED_B);		//点亮LED
	GPIO_SetBits(_MOTOR_LED_C_PORT, _MOTOR_LED_C);		//点亮LED
	GPIO_SetBits(_MOTOR_LED_D_PORT, _MOTOR_LED_D);		//点亮LED
}

void _MOTOR_LED_OFF(void)
{
	GPIO_ResetBits(_MOTOR_LED_A_PORT, _MOTOR_LED_A);	//熄灭LED
	GPIO_ResetBits(_MOTOR_LED_B_PORT, _MOTOR_LED_B);	//熄灭LED
	GPIO_ResetBits(_MOTOR_LED_C_PORT, _MOTOR_LED_C);	//熄灭LED
	GPIO_ResetBits(_MOTOR_LED_D_PORT, _MOTOR_LED_D);	//熄灭LED
}

void _MOTOR_LED_A_TOGGLE(void)
{
	 GPIOA->ODR ^= _MOTOR_LED_A;
}

void _MOTOR_LED_B_TOGGLE(void)
{
	 GPIOB->ODR ^= _MOTOR_LED_B;
}

void _MOTOR_LED_C_TOGGLE(void)
{
	 GPIOB->ODR ^= _MOTOR_LED_C;
}

void _MOTOR_LED_D_TOGGLE(void)
{
	 GPIOD->ODR ^= _MOTOR_LED_D;
}

void MOTOR_LED_TOGGLE(void)
{
	_MOTOR_LED_A_TOGGLE();
	_MOTOR_LED_B_TOGGLE();
	_MOTOR_LED_C_TOGGLE();
	_MOTOR_LED_D_TOGGLE();
}

 
