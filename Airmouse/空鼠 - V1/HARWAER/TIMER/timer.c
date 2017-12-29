#include "main.h"

SYSTIME Systime;
TIMEDOT ALTTimedot,InerTimedot,ContrTimedot,BaroTimedot,ThroTimedot;


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
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);		//设置定时器2周期和频率:72*1K/72M=1mS
	//
	//
  	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	
	TIM_ClearITPendingBit(TIM2, TIM_FLAG_Update);
	TIM_ITConfig(TIM2,TIM_IT_Update, ENABLE);					//打开中断
	TIM_Cmd(TIM2, DISABLE);
}

void Sys_Timer_RUN(void)
{
  TIM_Cmd(TIM2, ENABLE);//打开定时器
}

float Time_get(void)
{
  return Systime.systimems+(TIM2->CNT/1000.0f);//ms
}

float Timedot_get(TIMEDOT *timedot)
{
  timedot->lasttime=timedot->thistime;
  timedot->thistime=Time_get();

  return (timedot->thistime-timedot->lasttime)/1000.0f;//s
}

void TIM2_IRQHandler(void)
{
   if(TIM_GetITStatus(TIM2,TIM_IT_Update)!= RESET) 
	{
		Systime.systimems++;//1ms一次中断
		Flag.nrf_error_cnt++;
		Flag.system_led_cnt++;
		
			/**/
		if(Systime.systimems%1==0)
		{
			Imu_data_Prepare();
			AHRS_Update(Timedot_get(&ALTTimedot));
//			Interial_Update(Timedot_get(&InerTimedot));
			INS_Update(Timedot_get(&InerTimedot));
		}
		/*50HZ*/
		if(Systime.systimems%20==0)
		{
			Flag.system_usartrx_flag=0x01;
			Flag.system_usarttx_flag=0x01;			
		}

		TIM_ClearFlag(TIM2, TIM_FLAG_Update);	
    }
}



