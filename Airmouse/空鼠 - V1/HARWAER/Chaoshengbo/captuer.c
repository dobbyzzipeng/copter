#include "capture.h"
#include "led.h"
#include "usart.h"	
//#include "MOTOR_CONTROL.h"

//#include "Data_Tramsfer.h"
//#include "adc.h"
#include "delay.h"


unsigned char  TIM1CH1_CAPTURE_STA=0;	//���벶��״̬		
//bit7 ������ɱ�־
//bit6 ���񵽸ߵ�ƽ��־
//bit5~0 ���񵽸ߵ�ƽ��ʱ���������
unsigned short TIM1CH1_CAPTURE_VAL=0;	//���벶��ֵ
//��¼�����½���ʱ,TIM1_CNT��ֵ
float cap_length=0.0f;

unsigned char  TIM5CH1_CAPTURE_STA=0;	//���벶��״̬		
//bit7 ������ɱ�־
//bit6 ���񵽸ߵ�ƽ��־
//bit5~0 ���񵽸ߵ�ƽ��ʱ���������
//unsigned short TIM5CH1_CAPTURE_VAL=0;	//���벶��ֵ
//��¼�����½���ʱ,TIM5_CNT��ֵ
//static u32 temp=0;
static u32 dat1_last=0;
static u32 dat1_now=0;
//static u16 cnt_times=0;
//static u8  TIM2CH1_CAPTURE_STAUS=0;
// u16 last_distance1=0,distance1=0;
float length1=0.0f,lastlength1=0.0f;


void Chaoshengbo_Capture_Timer1_Configuration(unsigned short period)
{
	GPIO_InitTypeDef 			GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  	TIM_TimeBaseStructure;
	TIM_ICInitTypeDef  			TIM1_ICInitStructure;
	NVIC_InitTypeDef 			NVIC_InitStructure;	//���ö�ʱ��1�ж����ȼ�
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
 	RCC_APB2PeriphClockCmd(CHAOSHENGBOCAPTUREGPIORT, ENABLE); //ʹ��GPIOBʱ��
	//ECHO PIN
	GPIO_InitStructure.GPIO_Pin  = ECHO_PIN;  // ���֮ǰ����  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //��������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//IO���ٶ�Ϊ50MHz
	GPIO_Init(ECHO_GPIORT, &GPIO_InitStructure);
	GPIO_ResetBits(ECHO_GPIORT,ECHO_PIN);
	//TRIG PIN
	GPIO_InitStructure.GPIO_Pin  = TRIG_PIN;  // ���֮ǰ����  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//IO���ٶ�Ϊ50MHz
	GPIO_Init(TRIG_GPIORT, &GPIO_InitStructure);
	GPIO_ResetBits(TRIG_GPIORT,TRIG_PIN);
	//timer1 base configuration
	TIM_TimeBaseStructure.TIM_ClockDivision = SYS_TIMER1_SYSCLK_DIV;
	TIM_TimeBaseStructure.TIM_Prescaler = SYS_TIMER1_CLK_1MHZ;
	TIM_TimeBaseStructure.TIM_Period = period;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);	//���ö�ʱ��1���ں�Ƶ��:72*1K/72M=1mS
	//
	//��ʼ��TIM1���벶�����
	TIM1_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
  	TIM1_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
  	TIM1_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
  	TIM1_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
  	TIM1_ICInitStructure.TIM_ICFilter = 0x03;//IC1F=0000 ���������˲��� ���˲�
  	TIM_ICInit(TIM1, &TIM1_ICInitStructure);
	//nvic configuration
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn|TIM1_CC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	
	TIM_ClearITPendingBit(TIM1, TIM_FLAG_Update);
	TIM_ITConfig(TIM1,TIM_IT_Update|TIM_IT_CC1, ENABLE);//�򿪸����жϺ����벶���ж�
	TIM_Cmd(TIM1, ENABLE);						//�򿪶�ʱ��1
}



void Chaoshengcapture(void)
{
//		chaoshengbotrig();	
	if(TIM1CH1_CAPTURE_STA&0X80)//�ɹ�������һ�ξ���
	{
		printf("HIGH:%f\r\n",cap_length);//��ӡ���� ��λcm	
		TIM1CH1_CAPTURE_STA=0;			//������һ�β���
		chaoshengbotrig();
 	}
	if(TIM1CH1_CAPTURE_STA&0x20)//��������� ���²���
	{
		TIM1CH1_CAPTURE_STA=0;			//������һ�β���
		chaoshengbotrig();	
	}
}

void chaoshengbotrig(void)
{
//	Trig_h;	
//	delay_us(20);
//	Trig_l;	
	
	TRIG_H;	
	delay_us(20);
	TRIG_L;		
}


void TIM1_UP_IRQHandler(void)
{ 

 	if((TIM1CH1_CAPTURE_STA&0X80)==0)//��δ�ɹ�����	
	{	  
		if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)//��������� 
		{	    
			if(TIM1CH1_CAPTURE_STA&0X40)//�Ѿ����񵽸ߵ�ƽ��
			{
				TIM1CH1_CAPTURE_STA|=0x20;	//�����־
// 				TIM1CH1_CAPTURE_VAL=0;//���ʱ�����			
			}	 
		}		     	    					   
 	}
    TIM_ClearITPendingBit(TIM1, TIM_IT_Update); //����жϱ�־λ
}

//TIM1���벶���ж�
void TIM1_CC_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM1, TIM_IT_CC1) != RESET)//����1���������¼�
	{	
		if(TIM1CH1_CAPTURE_STA&0X40)		//����Ѿ�����һ��������		
		{
			TIM1CH1_CAPTURE_VAL=TIM_GetCapture1(TIM1);//��ȡCNT����ֵ
			cap_length=0.017*TIM1CH1_CAPTURE_VAL;//��þ��� 0.034cm/us *TIM1CH1_CAPTURE_VAL /2  ��λcm				
			TIM1CH1_CAPTURE_STA|=0X80;		//�����½��� �Ѿ��ɹ���65.535ms�ڲ���һ��������
		   	TIM_OC1PolarityConfig(TIM1,TIM_ICPolarity_Rising); //CC1P=0 ����Ϊ�����ز���
		}
		else  								//��δ��ʼ,��һ�β���������
		{
			TIM1CH1_CAPTURE_STA=0;//���
			TIM1CH1_CAPTURE_STA=0;
	 		TIM_SetCounter(TIM1,0);//��ʼ����
			TIM1CH1_CAPTURE_STA|=0X40;//��ǲ�����������
		   	TIM_OC1PolarityConfig(TIM1,TIM_ICPolarity_Falling);	//CC1P=1 ����Ϊ�½��ز���
		}		    
	}		     	    					   
		
    TIM_ClearITPendingBit(TIM1, TIM_IT_CC1); //������벶���жϱ�־λ	
}


void Chaoshengbo_Capture_Timer5_Configuration(unsigned short period)
{
	GPIO_InitTypeDef 			GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  	TIM_TimeBaseStructure;
	TIM_ICInitTypeDef  			TIM5_ICInitStructure;
	NVIC_InitTypeDef 			NVIC_InitStructure;	//���ö�ʱ��1�ж����ȼ�
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,  ENABLE);
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //ʹ��GPIOBʱ��
	//ECHO PIN
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_2;  // ���֮ǰ����  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //��������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//IO���ٶ�Ϊ50MHz
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA,GPIO_Pin_2);
	//TRIG PIN
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0;  // ���֮ǰ����  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//IO���ٶ�Ϊ50MHz
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA,GPIO_Pin_0);
	//timer1 base configuration
	TIM_TimeBaseStructure.TIM_ClockDivision = SYS_TIMER1_SYSCLK_DIV;
	TIM_TimeBaseStructure.TIM_Prescaler = SYS_TIMER1_CLK_1MHZ;
	TIM_TimeBaseStructure.TIM_Period = period;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);	//���ö�ʱ��1���ں�Ƶ��:72*1K/72M=1mS
	//
	//��ʼ��TIM1���벶�����
	TIM5_ICInitStructure.TIM_Channel = 	TIM_Channel_3; //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
  	TIM5_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
  	TIM5_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ֱ��ӳ�䵽��ӦICX������ӳ��
  	TIM5_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
  	TIM5_ICInitStructure.TIM_ICFilter = 0x03;//IC1F=0000 ���������˲��� ���˲�
  	TIM_ICInit(TIM5, &TIM5_ICInitStructure);
	//nvic configuration
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	
	TIM_ClearITPendingBit(TIM5, TIM_FLAG_Update);
	TIM_ITConfig(TIM5,TIM_IT_Update|TIM_IT_CC3, ENABLE);//�򿪸����жϺ����벶���ж�
	TIM_Cmd(TIM5, ENABLE);						//�򿪶�ʱ��1
}


void Chaoshengbomeasure(void)
{
	chaoshengbotrig();
	lastlength1=length1;
	if(dat1_now>dat1_last)
    {
		length1=0.017*(dat1_now-dat1_last);//cm
		if(length1>2&&length1<450) //cm
		{
			printf("l%f\r\n",length1);
		}
	}
}

 void TIM5_IRQHandler(void)
 { 
    u16 tsr;
    u16 tccr;
    tsr=TIM5->SR;
 //     if(tsr&0X01)
 // 		{
 // 			cnt_times++;
 // 		}
// 	  if(tsr&0x02)
// 		{
// 			tccr=TIM5->CCR3;
// 			if(TIM5CH1_CAPTURE_STA)	 		
// 			{	  			
// 				TIM5CH1_CAPTURE_STA=0;		
// 				dat1_now=cnt_times*65535+tccr;
// 				TIM5->CCER&=~(1<<1); 		
// 			}
// 			else  								
// 			{
// 				TIM5CH1_CAPTURE_STA=1;		
// 				dat1_last=cnt_times*65535+tccr;
// 				TIM5->CCER|=1<<1;	
// 			}		    
// 		}
 	  if(tsr&0x08)//ch3
 		{
 			tccr=TIM5->CCR3;
 			if(TIM5CH1_CAPTURE_STA)	 		
 			{	  			
 				TIM5CH1_CAPTURE_STA=0;		
// 				dat1_now=cnt_times*65535+tccr;//
 				dat1_now=tccr;//
 				TIM5->CCER&=~(1<<9); //���������ش���		
 			}
 			else  								
 			{
 				TIM5CH1_CAPTURE_STA=1;//�Ѿ�����������		
// 				dat1_last=cnt_times*65535+tccr;
 				dat1_last=tccr;
 				TIM5->CCER|=1<<9;	//�����½��ش���
 			}		    
 		}
		TIM5->SR=0;
    TIM_ClearITPendingBit(TIM5, TIM_IT_Update|TIM_IT_CC3); //����жϱ�־λ
 }


