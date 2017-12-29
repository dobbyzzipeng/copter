#include "timer.h"
#include "led.h"
#include "delay.h"


/////***********************************************////// 	  
extern uint8_t system_tim_flag;

void SYS_Timer2_Configuration(unsigned short period)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef 		 NVIC_InitStructure;			//设置定时器2中断优先级
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	//
	TIM_TimeBaseStructure.TIM_ClockDivision = SYS_TIMER2_SYSCLK_DIV;//不分频时APB172MHZ
	TIM_TimeBaseStructure.TIM_Prescaler = SYS_TIMER2_CLK_1MHZ;
	TIM_TimeBaseStructure.TIM_Period = period;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);		//设置定时器1周期和频率:72*1K/72M=1mS
	//
	//
  	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	
	TIM_ClearITPendingBit(TIM2, TIM_FLAG_Update);
	TIM_ITConfig(TIM2,TIM_IT_Update, ENABLE);					//打开中断
	TIM_Cmd(TIM2, ENABLE);													  //打开定时器1
}

void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2,TIM_IT_Update)!= RESET) 
	{
		system_tim_flag++;
		TIM_ClearFlag(TIM2, TIM_FLAG_Update);	
    }
}



