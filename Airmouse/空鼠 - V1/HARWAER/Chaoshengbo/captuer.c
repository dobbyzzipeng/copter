#include "capture.h"
#include "led.h"
#include "usart.h"	
//#include "MOTOR_CONTROL.h"

//#include "Data_Tramsfer.h"
//#include "adc.h"
#include "delay.h"


unsigned char  TIM1CH1_CAPTURE_STA=0;	//输入捕获状态		
//bit7 捕获完成标志
//bit6 捕获到高电平标志
//bit5~0 捕获到高电平后定时器溢出次数
unsigned short TIM1CH1_CAPTURE_VAL=0;	//输入捕获值
//记录捕获到下降沿时,TIM1_CNT的值
float cap_length=0.0f;

unsigned char  TIM5CH1_CAPTURE_STA=0;	//输入捕获状态		
//bit7 捕获完成标志
//bit6 捕获到高电平标志
//bit5~0 捕获到高电平后定时器溢出次数
//unsigned short TIM5CH1_CAPTURE_VAL=0;	//输入捕获值
//记录捕获到下降沿时,TIM5_CNT的值
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
	NVIC_InitTypeDef 			NVIC_InitStructure;	//设置定时器1中断优先级
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
 	RCC_APB2PeriphClockCmd(CHAOSHENGBOCAPTUREGPIORT, ENABLE); //使能GPIOB时钟
	//ECHO PIN
	GPIO_InitStructure.GPIO_Pin  = ECHO_PIN;  // 清除之前设置  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //输入下拉
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//IO口速度为50MHz
	GPIO_Init(ECHO_GPIORT, &GPIO_InitStructure);
	GPIO_ResetBits(ECHO_GPIORT,ECHO_PIN);
	//TRIG PIN
	GPIO_InitStructure.GPIO_Pin  = TRIG_PIN;  // 清除之前设置  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//IO口速度为50MHz
	GPIO_Init(TRIG_GPIORT, &GPIO_InitStructure);
	GPIO_ResetBits(TRIG_GPIORT,TRIG_PIN);
	//timer1 base configuration
	TIM_TimeBaseStructure.TIM_ClockDivision = SYS_TIMER1_SYSCLK_DIV;
	TIM_TimeBaseStructure.TIM_Prescaler = SYS_TIMER1_CLK_1MHZ;
	TIM_TimeBaseStructure.TIM_Period = period;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);	//设置定时器1周期和频率:72*1K/72M=1mS
	//
	//初始化TIM1输入捕获参数
	TIM1_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01 	选择输入端 IC1映射到TI1上
  	TIM1_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
  	TIM1_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
  	TIM1_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
  	TIM1_ICInitStructure.TIM_ICFilter = 0x03;//IC1F=0000 配置输入滤波器 不滤波
  	TIM_ICInit(TIM1, &TIM1_ICInitStructure);
	//nvic configuration
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn|TIM1_CC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	
	TIM_ClearITPendingBit(TIM1, TIM_FLAG_Update);
	TIM_ITConfig(TIM1,TIM_IT_Update|TIM_IT_CC1, ENABLE);//打开更新中断和输入捕获中断
	TIM_Cmd(TIM1, ENABLE);						//打开定时器1
}



void Chaoshengcapture(void)
{
//		chaoshengbotrig();	
	if(TIM1CH1_CAPTURE_STA&0X80)//成功测量了一次距离
	{
		printf("HIGH:%f\r\n",cap_length);//打印距离 单位cm	
		TIM1CH1_CAPTURE_STA=0;			//开启下一次捕获
		chaoshengbotrig();
 	}
	if(TIM1CH1_CAPTURE_STA&0x20)//计数器溢出 从新测量
	{
		TIM1CH1_CAPTURE_STA=0;			//开启下一次捕获
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

 	if((TIM1CH1_CAPTURE_STA&0X80)==0)//还未成功捕获	
	{	  
		if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)//计数器溢出 
		{	    
			if(TIM1CH1_CAPTURE_STA&0X40)//已经捕获到高电平了
			{
				TIM1CH1_CAPTURE_STA|=0x20;	//溢出标志
// 				TIM1CH1_CAPTURE_VAL=0;//溢出时间清空			
			}	 
		}		     	    					   
 	}
    TIM_ClearITPendingBit(TIM1, TIM_IT_Update); //清除中断标志位
}

//TIM1输入捕获中断
void TIM1_CC_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM1, TIM_IT_CC1) != RESET)//捕获1发生捕获事件
	{	
		if(TIM1CH1_CAPTURE_STA&0X40)		//如果已经捕获到一次上升沿		
		{
			TIM1CH1_CAPTURE_VAL=TIM_GetCapture1(TIM1);//获取CNT计数值
			cap_length=0.017*TIM1CH1_CAPTURE_VAL;//测得距离 0.034cm/us *TIM1CH1_CAPTURE_VAL /2  单位cm				
			TIM1CH1_CAPTURE_STA|=0X80;		//捕获到下降沿 已经成功在65.535ms内捕获到一次脉冲宽度
		   	TIM_OC1PolarityConfig(TIM1,TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
		}
		else  								//还未开始,第一次捕获上升沿
		{
			TIM1CH1_CAPTURE_STA=0;//清空
			TIM1CH1_CAPTURE_STA=0;
	 		TIM_SetCounter(TIM1,0);//开始计数
			TIM1CH1_CAPTURE_STA|=0X40;//标记捕获到了上升沿
		   	TIM_OC1PolarityConfig(TIM1,TIM_ICPolarity_Falling);	//CC1P=1 设置为下降沿捕获
		}		    
	}		     	    					   
		
    TIM_ClearITPendingBit(TIM1, TIM_IT_CC1); //清除输入捕获中断标志位	
}


void Chaoshengbo_Capture_Timer5_Configuration(unsigned short period)
{
	GPIO_InitTypeDef 			GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  	TIM_TimeBaseStructure;
	TIM_ICInitTypeDef  			TIM5_ICInitStructure;
	NVIC_InitTypeDef 			NVIC_InitStructure;	//设置定时器1中断优先级
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,  ENABLE);
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //使能GPIOB时钟
	//ECHO PIN
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_2;  // 清除之前设置  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //输入下拉
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//IO口速度为50MHz
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA,GPIO_Pin_2);
	//TRIG PIN
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0;  // 清除之前设置  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//IO口速度为50MHz
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA,GPIO_Pin_0);
	//timer1 base configuration
	TIM_TimeBaseStructure.TIM_ClockDivision = SYS_TIMER1_SYSCLK_DIV;
	TIM_TimeBaseStructure.TIM_Prescaler = SYS_TIMER1_CLK_1MHZ;
	TIM_TimeBaseStructure.TIM_Period = period;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);	//设置定时器1周期和频率:72*1K/72M=1mS
	//
	//初始化TIM1输入捕获参数
	TIM5_ICInitStructure.TIM_Channel = 	TIM_Channel_3; //CC1S=01 	选择输入端 IC1映射到TI1上
  	TIM5_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
  	TIM5_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //直接映射到相应ICX，不重映射
  	TIM5_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
  	TIM5_ICInitStructure.TIM_ICFilter = 0x03;//IC1F=0000 配置输入滤波器 不滤波
  	TIM_ICInit(TIM5, &TIM5_ICInitStructure);
	//nvic configuration
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	
	TIM_ClearITPendingBit(TIM5, TIM_FLAG_Update);
	TIM_ITConfig(TIM5,TIM_IT_Update|TIM_IT_CC3, ENABLE);//打开更新中断和输入捕获中断
	TIM_Cmd(TIM5, ENABLE);						//打开定时器1
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
 				TIM5->CCER&=~(1<<9); //设置上升沿触发		
 			}
 			else  								
 			{
 				TIM5CH1_CAPTURE_STA=1;//已经捕获到上升沿		
// 				dat1_last=cnt_times*65535+tccr;
 				dat1_last=tccr;
 				TIM5->CCER|=1<<9;	//设置下降沿触发
 			}		    
 		}
		TIM5->SR=0;
    TIM_ClearITPendingBit(TIM5, TIM_IT_Update|TIM_IT_CC3); //清除中断标志位
 }


