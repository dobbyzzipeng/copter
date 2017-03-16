#include "pwm.h"
//#include "led.h"
#include "stm32f10x_it.h"
#include "stm32f10x_tim.h"
//////////////////////////////////////////////////////////////	  

void DUOJI_PWM_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
	//
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	//
  GPIO_InitStructure.GPIO_Pin = _DUOJI_PWM_M1;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //设置PWM口最大允许输出速度
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	  //设置PWM口为输出
  GPIO_Init(_DUOJI_PWM_PORT, &GPIO_InitStructure);
	//
	
  GPIO_InitStructure.GPIO_Pin = _DUOJI_PWM_M2;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //设置PWM口最大允许输出速度
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	  //设置PWM口为输出
  GPIO_Init(_DUOJI_PWM_PORT, &GPIO_InitStructure);
	//
     GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3,ENABLE);//映射
//     GPIO_PinRemapConfig(GPIO_Remap1_CAN1,ENABLE);//映射
	
  GPIO_InitStructure.GPIO_Pin = _DUOJI_PWM_M3;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //设置PWM口最大允许输出速度
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	  //设置PWM口为输出
  GPIO_Init(_DUOJI_PWM_PORT, &GPIO_InitStructure);
	//
  GPIO_InitStructure.GPIO_Pin = _DUOJI_PWM_M4;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //设置PWM口最大允许输出速度
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	  //设置PWM口为输出
  GPIO_Init(_DUOJI_PWM_PORT, &GPIO_InitStructure);
 
  GPIO_ResetBits(_DUOJI_PWM_PORT,_DUOJI_PWM_M1|_DUOJI_PWM_M2|_DUOJI_PWM_M3|_DUOJI_PWM_M4);
	 
  GPIO_InitStructure.GPIO_Pin = _DUOJI_PWM_M1;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //设置LED口最大允许输出速度
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	  	//设置LED口为输出
  GPIO_Init(_DUOJI_PWM_PORT, &GPIO_InitStructure);
	//
  GPIO_InitStructure.GPIO_Pin = _DUOJI_PWM_M2;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //设置LED口最大允许输出速度
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	  	//设置LED口为输出
  GPIO_Init(_DUOJI_PWM_PORT, &GPIO_InitStructure);
	//
  GPIO_InitStructure.GPIO_Pin = _DUOJI_PWM_M3;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //设置LED口最大允许输出速度
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	  	//设置LED口为输出
  GPIO_Init(_DUOJI_PWM_PORT, &GPIO_InitStructure);
	//
  GPIO_InitStructure.GPIO_Pin = _DUOJI_PWM_M4;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //设置LED口最大允许输出速度
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	  	//设置LED口为输出
  GPIO_Init(_DUOJI_PWM_PORT, &GPIO_InitStructure);
	//
	//PWM频率=BS004_MOTOR_PWM_CLK_36MHZ/（BS004_MOTOR_PWM_PERIOD+1)
	TIM_TimeBaseStructure.TIM_ClockDivision = _DUOJI_PWM_SYSCLK_DIV;//时钟分频
	TIM_TimeBaseStructure.TIM_Prescaler = _DUOJI_PWM_CLK_1MHZ;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = _DUOJI_PWM_PERIOD;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);								//设置PWM周期和频率
	//
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//比较输出使能
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;//正常通道输出极性高
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;//反通道输出高
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
  TIM_OC1Init(TIM3, &TIM_OCInitStructure);											//设置PWM占空比
	//
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC2Init(TIM3, &TIM_OCInitStructure);											//设置PWM占空比
	//
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC3Init(TIM3, &TIM_OCInitStructure);											//设置PWM占空比
	//
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC4Init(TIM3, &TIM_OCInitStructure);											//设置PWM占空比
	//

// 	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);   //CH1 预装载使能
// 	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);   //CH1 预装载使能
// 	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);   //CH1 预装载使能
// 	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);   //CH1 预装载使能
	
	TIM_ARRPreloadConfig(TIM3, ENABLE); //使能 TIMx 在 ARR 上的预装在寄存器

//	TIM_CtrlPWMOutputs(TIM3,ENABLE);          //允许PWM输出
	TIM_Cmd(TIM3, ENABLE);										//启动PWM
}


void _MOTOR_PWM_ON(void)
{
	GPIO_SetBits(_MOTOR_PWM_PORT, _MOTOR_PWM_M1);			//打开PWM输出
	GPIO_SetBits(_MOTOR_PWM_PORT, _MOTOR_PWM_M2);			//打开PWM输出
	GPIO_SetBits(_MOTOR_PWM_PORT, _MOTOR_PWM_M3);			//打开PWM输出
	GPIO_SetBits(_MOTOR_PWM_PORT, _MOTOR_PWM_M4);			//打开PWM输出
}
void _MOTOR_PWM_OFF(void)
{
	GPIO_ResetBits(_MOTOR_PWM_PORT, _MOTOR_PWM_M1);		//关闭PWM输出
	GPIO_ResetBits(_MOTOR_PWM_PORT, _MOTOR_PWM_M2);		//关闭PWM输出
	GPIO_ResetBits(_MOTOR_PWM_PORT, _MOTOR_PWM_M3);		//关闭PWM输出
	GPIO_ResetBits(_MOTOR_PWM_PORT, _MOTOR_PWM_M4);		//关闭PWM输出
}

//===============================================================
//TIM8高级定时器不分频72MHZ
void _Motor_PWM_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
	//
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
	//
  GPIO_InitStructure.GPIO_Pin = _MOTOR_PWM_M1;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //设置PWM口最大允许输出速度
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	  //设置PWM口为输出
  GPIO_Init(_MOTOR_PWM_PORT, &GPIO_InitStructure);
	//
  GPIO_InitStructure.GPIO_Pin = _MOTOR_PWM_M2;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //设置PWM口最大允许输出速度
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	  //设置PWM口为输出
  GPIO_Init(_MOTOR_PWM_PORT, &GPIO_InitStructure);
	//
  GPIO_InitStructure.GPIO_Pin = _MOTOR_PWM_M3;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //设置PWM口最大允许输出速度
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	  //设置PWM口为输出
  GPIO_Init(_MOTOR_PWM_PORT, &GPIO_InitStructure);
	//
  GPIO_InitStructure.GPIO_Pin = _MOTOR_PWM_M4;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //设置PWM口最大允许输出速度
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	  //设置PWM口为输出
  GPIO_Init(_MOTOR_PWM_PORT, &GPIO_InitStructure);
 
  GPIO_ResetBits(_MOTOR_PWM_PORT,_MOTOR_PWM_M1|_MOTOR_PWM_M2|_MOTOR_PWM_M3|_MOTOR_PWM_M4);
	 
  GPIO_InitStructure.GPIO_Pin = _MOTOR_PWM_M1;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //设置LED口最大允许输出速度
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	  	//设置LED口为输出
  GPIO_Init(_MOTOR_PWM_PORT, &GPIO_InitStructure);
	//
  GPIO_InitStructure.GPIO_Pin = _MOTOR_PWM_M2;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //设置LED口最大允许输出速度
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	  	//设置LED口为输出
  GPIO_Init(_MOTOR_PWM_PORT, &GPIO_InitStructure);
	//
  GPIO_InitStructure.GPIO_Pin = _MOTOR_PWM_M3;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //设置LED口最大允许输出速度
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	  	//设置LED口为输出
  GPIO_Init(_MOTOR_PWM_PORT, &GPIO_InitStructure);
	//
  GPIO_InitStructure.GPIO_Pin = _MOTOR_PWM_M4;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //设置LED口最大允许输出速度
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	  	//设置LED口为输出
  GPIO_Init(_MOTOR_PWM_PORT, &GPIO_InitStructure);
	//
	//PWM频率=BS004_MOTOR_PWM_CLK_36MHZ/（BS004_MOTOR_PWM_PERIOD+1)
	TIM_TimeBaseStructure.TIM_ClockDivision = _MOTOR_PWM_SYSCLK_DIV;//时钟分频
	TIM_TimeBaseStructure.TIM_Prescaler = _MOTOR_PWM_CLK_18MHZ;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = _MOTOR_PWM_PERIOD;
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);								//设置PWM周期和频率
	//
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//比较输出使能
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;//正常通道输出极性高
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;//反通道输出高
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
  TIM_OC1Init(TIM8, &TIM_OCInitStructure);											//设置PWM占空比
	//
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC2Init(TIM8, &TIM_OCInitStructure);											//设置PWM占空比
	//
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC3Init(TIM8, &TIM_OCInitStructure);											//设置PWM占空比
	//
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC4Init(TIM8, &TIM_OCInitStructure);											//设置PWM占空比
	//

// 	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);   //CH1 预装载使能
// 	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);   //CH1 预装载使能
// 	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);   //CH1 预装载使能
// 	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);   //CH1 预装载使能
	
	TIM_ARRPreloadConfig(TIM8, ENABLE); //使能 TIMx 在 ARR 上的预装在寄存器

	TIM_CtrlPWMOutputs(TIM8,ENABLE);          //允许PWM输出
	TIM_Cmd(TIM8, ENABLE);										//启动PWM
	Motor_Reset();
}
//===============================================================


signed short _Motor_Speed_Scale(float motor_speed_input)
{
	float motor_speed_output;
	//
	if(motor_speed_input>_FLY_MAX_OUT) motor_speed_output=_FLY_MAX_OUT;
	else if(motor_speed_input<_FLY_MIN_OUT) motor_speed_output=_FLY_MIN_OUT;
	else motor_speed_output=motor_speed_input;
	return motor_speed_output;
}

void Motor_Reset(void)
{	//
	TIM_SetCompare3(TIM8,0);//更新PWM占空比
	TIM_SetCompare4(TIM8,0);//更新PWM占空比
	TIM_SetCompare2(TIM8,0);//更新PWM占空比
	TIM_SetCompare1(TIM8,0);//更新PWM占空比
}

void Motor_PWMSET(signed short motorpwm1,signed short motorpwm2,signed short motorpwm3,signed short motorpwm4)
{
	if(motorpwm1<0) motorpwm1=0;
	if(motorpwm1>_MOTOR_PWM_PERIOD) motorpwm1=_MOTOR_PWM_PERIOD;//限幅
	if(motorpwm2<0) motorpwm2=0;
	if(motorpwm2>_MOTOR_PWM_PERIOD) motorpwm2=_MOTOR_PWM_PERIOD;//限幅	
	if(motorpwm3<0) motorpwm3=0;
	if(motorpwm3>_MOTOR_PWM_PERIOD) motorpwm3=_MOTOR_PWM_PERIOD;//限幅
	if(motorpwm4<0) motorpwm4=0;
	if(motorpwm4>_MOTOR_PWM_PERIOD) motorpwm4=_MOTOR_PWM_PERIOD;//限幅
	
	TIM_SetCompare4(TIM8,motorpwm1);//更新PWM占空比
	TIM_SetCompare3(TIM8,motorpwm2);//更新PWM占空比
	TIM_SetCompare2(TIM8,motorpwm3);//更新PWM占空比
	TIM_SetCompare1(TIM8,motorpwm4);//更新PWM占空比
}

