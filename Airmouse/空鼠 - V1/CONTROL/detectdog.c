#include "detectdog.h"
static uint8_t i = 0;
void TIM6_Configuration(void)
{
    TIM_TimeBaseInitTypeDef  tim;
    NVIC_InitTypeDef         nvic;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);
    
    nvic.NVIC_IRQChannel = TIM6_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    tim.TIM_Prescaler = 72-1;        //84M internal clock
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_Period = 1000;  //1ms,1000Hz
    TIM_TimeBaseInit(TIM6,&tim);
}

void TIM6_IRQHandler(void)
{
   if(TIM_GetITStatus(TIM6,TIM_IT_Update)!= RESET) 
	{
	  TIM_ClearFlag(TIM6,TIM_FLAG_Update);
	  for(i=0;i<DETECTEVENTNUM;i++)
	  {
		DETECTDOGBUF[i]--;
	  }
    }
}

void DetectEventInit(void)
{
    TIM6_Configuration();
}

void DetectDogRest(uint32_t index)
{
	DETECTDOGBUF[index] = DOWNTHRESHOLD;
}

void DetectDogFeed(uint32_t index, uint32_t threshold)
{
	DETECTDOGBUF[index] = threshold;
}

uint8_t IsOverflowCheck(uint32_t index, uint32_t threshold)
{
	if(DETECTDOGBUF[index] < DOWNTHRESHOLD) return 1;
	return 0;
}
