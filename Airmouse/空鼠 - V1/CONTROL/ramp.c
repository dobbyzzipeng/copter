#include "ramp.h"
void RampInit(RampGen_t *ramp, int32_t XSCALE)
{
    TIM_TimeBaseInitTypeDef  tim;
	if(ramp->TIMx == TIM5)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
	}
	
//    NVIC_InitTypeDef         nvic;    
//    nvic.NVIC_IRQChannel = TIM5_IRQn;
//    nvic.NVIC_IRQChannelPreemptionPriority = 1;
//    nvic.NVIC_IRQChannelSubPriority = 0;
//    nvic.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&nvic);

    tim.TIM_Prescaler = 72-1;        //72M internal clock
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_Period = 0xffff/*1000*/;  //1ms,1000Hz
    TIM_TimeBaseInit(ramp->TIMx,&tim);
	
	ramp->XSCALE = XSCALE;
}

float RampCalc(RampGen_t *ramp)
{
//	ramp->slop = (float)(ramp->thisinput - ramp->lastoutput)/RamptimeGet(ramp);
//	if(ramp->slop >= SLOP)
//	{
		ramp->out = (++ramp->count/ramp->XSCALE);
//	}
//	else
//	{
//	    ramp->out = 0;
//	}
	return ramp->out;
}

void RampSetCounter(struct RampGen_t *ramp, int32_t count)
{
	ramp->count = count;
}

void RampResetCounter(struct RampGen_t *ramp)
{
	ramp->count = 0;
}

void RampSetScale(struct RampGen_t *ramp, int32_t scale)
{
	ramp->XSCALE = scale;
}

uint8_t RampIsOverflow(struct RampGen_t *ramp)
{
	if(ramp->count > ramp->XSCALE)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

//void TIM5_Configuration(void)
//{
//    TIM_TimeBaseInitTypeDef  tim;
////    NVIC_InitTypeDef         nvic;

//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
//    
////    nvic.NVIC_IRQChannel = TIM5_IRQn;
////    nvic.NVIC_IRQChannelPreemptionPriority = 1;
////    nvic.NVIC_IRQChannelSubPriority = 0;
////    nvic.NVIC_IRQChannelCmd = ENABLE;
////    NVIC_Init(&nvic);

//    tim.TIM_Prescaler = 72-1;        //72M internal clock
//    tim.TIM_CounterMode = TIM_CounterMode_Up;
//    tim.TIM_ClockDivision = TIM_CKD_DIV1;
//    tim.TIM_Period = 0xffff/*1000*/;  //1ms,1000Hz
//    TIM_TimeBaseInit(TIM5,&tim);
//}

void TIM5_IRQHandler(void)
{
   if(TIM_GetITStatus(TIM5,TIM_IT_Update)!= RESET) 
	{
	  TIM_ClearFlag(TIM5,TIM_FLAG_Update);
    }
}

int32_t RamptimeGet(RampGen_t *ramp)
{
	ramp->lasttime = ramp->thistime;
	ramp->thistime = ramp->TIMx->CNT;
	if(ramp->thistime >= ramp->lasttime)
	{
	  return (ramp->thistime - ramp->lasttime);
	}
	else
	{
	  return (0xffff + ramp->thistime - ramp->lasttime);
	}
}
