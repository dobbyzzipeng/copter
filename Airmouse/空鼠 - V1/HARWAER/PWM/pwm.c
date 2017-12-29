#include "pwm.h"
#include "led.h"
#include "stm32f10x_it.h"
//////////////////////////////////////////////////////////////	  

void MOTOR_PWM_ON(void)
{
	GPIO_SetBits(_MOTOR_PWM_PORT, _MOTOR_PWM_M1);			//打开PWM输出
	GPIO_SetBits(_MOTOR_PWM_PORT, _MOTOR_PWM_M2);			//打开PWM输出
	GPIO_SetBits(_MOTOR_PWM_PORT, _MOTOR_PWM_M3);			//打开PWM输出
	GPIO_SetBits(_MOTOR_PWM_PORT, _MOTOR_PWM_M4);			//打开PWM输出
}
void MOTOR_PWM_OFF(void)
{
	GPIO_ResetBits(_MOTOR_PWM_PORT, _MOTOR_PWM_M1);		//关闭PWM输出
	GPIO_ResetBits(_MOTOR_PWM_PORT, _MOTOR_PWM_M2);		//关闭PWM输出
	GPIO_ResetBits(_MOTOR_PWM_PORT, _MOTOR_PWM_M3);		//关闭PWM输出
	GPIO_ResetBits(_MOTOR_PWM_PORT, _MOTOR_PWM_M4);		//关闭PWM输出
}

//===============================================================
//TIM8高级定时器不分频72MHZ
void Motor_PWM_Configuration(void)
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


signed short Motor_Speed_Scale(float motor_speed_input)
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

void Motor_PwrAdd(float throttle,float controlout[],float motorout[])
{
	#if FLIGHT_MODE==QUAD_4AXIS_X
	{
		motorout[0]=throttle-controlout[0]+controlout[1]-controlout[2];
		motorout[1]=throttle+controlout[0]+controlout[1]+controlout[2];
		motorout[2]=throttle+controlout[0]-controlout[1]-controlout[2];
		motorout[3]=throttle-controlout[0]-controlout[1]+controlout[2];
		motorout[4]=0.0f;
		motorout[5]=0.0f;
	}
	#endif
//	
//	#if FLIGHT_MODE==QUAD_4AXIS_I
//	{
//		motorout[0]=throttle-  controlout[0]+0*controlout[1]-controlout[2];
//		motorout[1]=throttle+0*controlout[0]+  controlout[1]+controlout[2];
//		motorout[2]=throttle+  controlout[0]-0*controlout[1]-controlout[2];
//		motorout[3]=throttle-0*controlout[0]-  controlout[1]+controlout[2];
//		motorout[4]=0.0f;
//		motorout[5]=0.0f;	
//	}
//	#endif
//	
//	#if FLIGHT_MODE==QUAD_6AXIS_X
//	{
//		motorout[0]=throttle-0.866f*controlout[0]+0.500f*controlout[1]-controlout[2];
//		motorout[1]=throttle+0.866f*controlout[0]+0.500f*controlout[1]-controlout[2];
//		motorout[2]=throttle+0.866f*controlout[0]-0.500f*controlout[1]+controlout[2];
//		motorout[3]=throttle-0.866f*controlout[0]-0.500f*controlout[1]+controlout[2];
//		motorout[4]=throttle+0.000f*controlout[0]+1.000f*controlout[1]+controlout[2];
//		motorout[5]=throttle-0.000f*controlout[0]-1.000f*controlout[1]-controlout[2];
//	}
//	#endif
//	
//		#if FLIGHT_MODE==QUAD_6AXIS_I
//	{
//		motorout[0]=throttle-0.866f*controlout[0]+0.500f*controlout[1]-controlout[2];
//		motorout[1]=throttle+0.866f*controlout[0]+0.500f*controlout[1]-controlout[2];
//		motorout[2]=throttle+0.866f*controlout[0]-0.500f*controlout[1]+controlout[2];
//		motorout[3]=throttle-0.866f*controlout[0]-0.500f*controlout[1]+controlout[2];
//		motorout[4]=throttle+0.000f*controlout[0]+1.000f*controlout[1]+controlout[2];
//		motorout[5]=throttle-0.000f*controlout[0]-1.000f*controlout[1]-controlout[2];
//	}
//	#endif
}

void Motor_PwmValSet(float motorout[])
{
	unsigned char i=0;
	float thr[6]={0.0f};
	for(i=0;i<6;i++)
	{
		thr[i]=motorout[i]*_FLY_MAX_OUT;
		if(thr[i]>_FLY_MAX_OUT)	thr[i]=_FLY_MAX_OUT;
		if(thr[i]<0.0f)					thr[i]=0.0f;
	}
	
	TIM_SetCompare4(TIM8,thr[0]);//更新PWM占空比
	TIM_SetCompare3(TIM8,thr[1]);//更新PWM占空比
	TIM_SetCompare2(TIM8,thr[2]);//更新PWM占空比
	TIM_SetCompare1(TIM8,thr[3]);//更新PWM占空比
	//motor4 
	//motor5
}
