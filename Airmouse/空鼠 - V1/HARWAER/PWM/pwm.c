#include "pwm.h"
#include "led.h"
#include "stm32f10x_it.h"
//////////////////////////////////////////////////////////////	  

void MOTOR_PWM_ON(void)
{
	GPIO_SetBits(_MOTOR_PWM_PORT, _MOTOR_PWM_M1);			//��PWM���
	GPIO_SetBits(_MOTOR_PWM_PORT, _MOTOR_PWM_M2);			//��PWM���
	GPIO_SetBits(_MOTOR_PWM_PORT, _MOTOR_PWM_M3);			//��PWM���
	GPIO_SetBits(_MOTOR_PWM_PORT, _MOTOR_PWM_M4);			//��PWM���
}
void MOTOR_PWM_OFF(void)
{
	GPIO_ResetBits(_MOTOR_PWM_PORT, _MOTOR_PWM_M1);		//�ر�PWM���
	GPIO_ResetBits(_MOTOR_PWM_PORT, _MOTOR_PWM_M2);		//�ر�PWM���
	GPIO_ResetBits(_MOTOR_PWM_PORT, _MOTOR_PWM_M3);		//�ر�PWM���
	GPIO_ResetBits(_MOTOR_PWM_PORT, _MOTOR_PWM_M4);		//�ر�PWM���
}

//===============================================================
//TIM8�߼���ʱ������Ƶ72MHZ
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
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //����PWM�������������ٶ�
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	  //����PWM��Ϊ���
  GPIO_Init(_MOTOR_PWM_PORT, &GPIO_InitStructure);
	//
  GPIO_InitStructure.GPIO_Pin = _MOTOR_PWM_M2;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //����PWM�������������ٶ�
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	  //����PWM��Ϊ���
  GPIO_Init(_MOTOR_PWM_PORT, &GPIO_InitStructure);
	//
  GPIO_InitStructure.GPIO_Pin = _MOTOR_PWM_M3;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //����PWM�������������ٶ�
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	  //����PWM��Ϊ���
  GPIO_Init(_MOTOR_PWM_PORT, &GPIO_InitStructure);
	//
  GPIO_InitStructure.GPIO_Pin = _MOTOR_PWM_M4;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //����PWM�������������ٶ�
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	  //����PWM��Ϊ���
  GPIO_Init(_MOTOR_PWM_PORT, &GPIO_InitStructure);
 
  GPIO_ResetBits(_MOTOR_PWM_PORT,_MOTOR_PWM_M1|_MOTOR_PWM_M2|_MOTOR_PWM_M3|_MOTOR_PWM_M4);
	 
  GPIO_InitStructure.GPIO_Pin = _MOTOR_PWM_M1;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //����LED�������������ٶ�
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	  	//����LED��Ϊ���
  GPIO_Init(_MOTOR_PWM_PORT, &GPIO_InitStructure);
	//
  GPIO_InitStructure.GPIO_Pin = _MOTOR_PWM_M2;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //����LED�������������ٶ�
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	  	//����LED��Ϊ���
  GPIO_Init(_MOTOR_PWM_PORT, &GPIO_InitStructure);
	//
  GPIO_InitStructure.GPIO_Pin = _MOTOR_PWM_M3;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //����LED�������������ٶ�
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	  	//����LED��Ϊ���
  GPIO_Init(_MOTOR_PWM_PORT, &GPIO_InitStructure);
	//
  GPIO_InitStructure.GPIO_Pin = _MOTOR_PWM_M4;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //����LED�������������ٶ�
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	  	//����LED��Ϊ���
  GPIO_Init(_MOTOR_PWM_PORT, &GPIO_InitStructure);
	//
	//PWMƵ��=BS004_MOTOR_PWM_CLK_36MHZ/��BS004_MOTOR_PWM_PERIOD+1)
	TIM_TimeBaseStructure.TIM_ClockDivision = _MOTOR_PWM_SYSCLK_DIV;//ʱ�ӷ�Ƶ
	TIM_TimeBaseStructure.TIM_Prescaler = _MOTOR_PWM_CLK_18MHZ;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = _MOTOR_PWM_PERIOD;
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);								//����PWM���ں�Ƶ��
	//
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//�Ƚ����ʹ��
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;//����ͨ��������Ը�
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;//��ͨ�������
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
  TIM_OC1Init(TIM8, &TIM_OCInitStructure);											//����PWMռ�ձ�
	//
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC2Init(TIM8, &TIM_OCInitStructure);											//����PWMռ�ձ�
	//
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC3Init(TIM8, &TIM_OCInitStructure);											//����PWMռ�ձ�
	//
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC4Init(TIM8, &TIM_OCInitStructure);											//����PWMռ�ձ�
	//

// 	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);   //CH1 Ԥװ��ʹ��
// 	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);   //CH1 Ԥװ��ʹ��
// 	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);   //CH1 Ԥװ��ʹ��
// 	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);   //CH1 Ԥװ��ʹ��
	
	TIM_ARRPreloadConfig(TIM8, ENABLE); //ʹ�� TIMx �� ARR �ϵ�Ԥװ�ڼĴ���

	TIM_CtrlPWMOutputs(TIM8,ENABLE);          //����PWM���
	TIM_Cmd(TIM8, ENABLE);										//����PWM
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
	TIM_SetCompare3(TIM8,0);//����PWMռ�ձ�
	TIM_SetCompare4(TIM8,0);//����PWMռ�ձ�
	TIM_SetCompare2(TIM8,0);//����PWMռ�ձ�
	TIM_SetCompare1(TIM8,0);//����PWMռ�ձ�
}

void Motor_PWMSET(signed short motorpwm1,signed short motorpwm2,signed short motorpwm3,signed short motorpwm4)
{
	if(motorpwm1<0) motorpwm1=0;
	if(motorpwm1>_MOTOR_PWM_PERIOD) motorpwm1=_MOTOR_PWM_PERIOD;//�޷�
	if(motorpwm2<0) motorpwm2=0;
	if(motorpwm2>_MOTOR_PWM_PERIOD) motorpwm2=_MOTOR_PWM_PERIOD;//�޷�	
	if(motorpwm3<0) motorpwm3=0;
	if(motorpwm3>_MOTOR_PWM_PERIOD) motorpwm3=_MOTOR_PWM_PERIOD;//�޷�
	if(motorpwm4<0) motorpwm4=0;
	if(motorpwm4>_MOTOR_PWM_PERIOD) motorpwm4=_MOTOR_PWM_PERIOD;//�޷�
	
	TIM_SetCompare4(TIM8,motorpwm1);//����PWMռ�ձ�
	TIM_SetCompare3(TIM8,motorpwm2);//����PWMռ�ձ�
	TIM_SetCompare2(TIM8,motorpwm3);//����PWMռ�ձ�
	TIM_SetCompare1(TIM8,motorpwm4);//����PWMռ�ձ�
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
	
	TIM_SetCompare4(TIM8,thr[0]);//����PWMռ�ձ�
	TIM_SetCompare3(TIM8,thr[1]);//����PWMռ�ձ�
	TIM_SetCompare2(TIM8,thr[2]);//����PWMռ�ձ�
	TIM_SetCompare1(TIM8,thr[3]);//����PWMռ�ձ�
	//motor4 
	//motor5
}
