#include "sys.h"
#include "usart.h"	
#include "mpu6050.h"
#include "led.h"
#include "stm32f10x_dma.h"
#include "Data_Tramsfer.h"
//#include "MOTOR_CONTROL.h"
#include "globalvariable.h"
////////////////////////////////////////////////////////////////////////////////// 	 
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_UCOS
#include "includes.h"					//ucos 使用	  
#endif
//////////////////////////////////////////////////////////////////////////////////	 
////////////////////////////////////////////////////////////////////////////////// 	  
 

//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0){;}//循环发送,直到发送完毕   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 

/*使用microLib的方法*/
 /* 
int fputc(int ch, FILE *f)
{
	USART_SendData(USART1, (uint8_t) ch);

	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {}	
   
    return ch;
}
int GetKey (void)  { 

    while (!(USART1->SR & USART_FLAG_RXNE));

    return ((int)(USART1->DR & 0x1FF));
}
*/
//extern FLAGS  Flag;
//extern RC Rcdata;
//extern TARGET Target;

//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
unsigned char USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
unsigned char tx_buffer[DMA_TX_LENGTH]={0};
unsigned char rx_buffer[DMA_RX_LENGTH]={0};//DMA接收缓冲
unsigned short Usart_Flag=0x0000;//串口接受标志变量
//bit15:接收帧头标志
//bit14:功能字
//bit13:功能字
//bit12:功能字
//bit11:功能字
//bit10:接收到帧尾
//bit0~9:接收到有效数据长度
//初始化IO 串口1 
//bound:波特率
/****************usart1_init**********************/
//串口1DMA发送接收
//偶校验 一个字节需要9位长度
void uart_init(u32 bound)
{
    //GPIO端口设置
      GPIO_InitTypeDef  GPIO_InitStructure;
	  USART_InitTypeDef USART_InitStructure;
	  NVIC_InitTypeDef  NVIC_InitStructure;
      DMA_InitTypeDef   dma;
	
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//使能USART1，GPIOA时钟
	  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);//初始化DMA1时钟 	 
	  USART_DeInit(USART1);  //复位串口1
	 //USART1_TX   PA.9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
    GPIO_Init(GPIOA, &GPIO_InitStructure); //初始化PA9
   
    //USART1_RX	  PA.10
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);  //初始化PA10

   //Usart1_RX NVIC 配置
//    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
// 	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//抢占优先级3
// 	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//子优先级3
// 	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
// 	  NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
  
   //USART 初始化设置
	 USART_InitStructure.USART_BaudRate = bound;//一般设置为;
// 	 USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	 USART_InitStructure.USART_WordLength = USART_WordLength_9b;//字长为9位数据格式
	 USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	 USART_InitStructure.USART_Parity = USART_Parity_Even;//偶校验位
// 	 USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	 USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	 USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

    USART_Init(USART1, &USART_InitStructure); //初始化串口
//  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启中断
    USART_Cmd(USART1, ENABLE);                    //使能串口 
/*******************************USART接收DMA配置******************************************/	
    dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);//内存到USART1
    dma.DMA_MemoryBaseAddr = (uint32_t)rx_buffer;   
    dma.DMA_DIR = DMA_DIR_PeripheralSRC;//外设为传输源头
    dma.DMA_BufferSize = DMA_RX_LENGTH;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_Mode = DMA_Mode_Circular;
    dma.DMA_Priority = DMA_Priority_VeryHigh;
    dma.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel5,&dma);

	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);//使能USART1RX DMA1传输 
    DMA_ITConfig(DMA1_Channel5, DMA_IT_TC, ENABLE);//DMA1_Channel5传输完成中断
    DMA_Cmd(DMA1_Channel5, ENABLE);//使能DMA1_Channel5
	
/***********************DMA1_Channel5中断配置***************************/
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
  
	DMA_ITConfig(DMA1_Channel5, DMA_IT_TC, ENABLE);//DMA1_Channel5传输完成中断
/**************USART发送DMA配置****************************/    
    dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);//外设地址
    dma.DMA_MemoryBaseAddr = (uint32_t)tx_buffer;//内存地址   
    dma.DMA_DIR = DMA_DIR_PeripheralDST;//外设为传输目的地
    dma.DMA_BufferSize = DMA_TX_LENGTH;//传输长度
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址不可变
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;//内存地址可变
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_Mode = DMA_Mode_Normal;
    dma.DMA_Priority = DMA_Priority_VeryHigh;
    dma.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel4,&dma);
   	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);//使能USART1TX DMA1传输   
    DMA_Cmd(DMA1_Channel4, DISABLE);//禁止使能DMA1_Channel4
	
/***********************DMA1_Channel4中断配置***************************/
//     NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
//     NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//     NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//     NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//     NVIC_Init(&NVIC_InitStructure);
  
// 	DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);//DMA1_Channel4传输完成中断
/*********************************************************************/	
}

//帧格式：AB口100.00口200.00口300.00E
//帧格式：AC口100.00口200.00口300.00E
//帧格式：AD口100.00口200.00口300.00E
//帧格式：AE口100.00口200.00口300.00E
void USART1_IRQHandler(void) //串口1中断服务程序
{
	u8 Res=0;
	static u8 cnt=0,i=0;
	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
			Res =USART_ReceiveData(USART1);//(USART1->DR);	//读取接收到的数据
			cnt++;
			switch (cnt)
			{
				case 1:
				{
					if(Res=='A') 
					{
						Usart_Flag|=0x8000;//接收到帧头
						LED1=0;
					}
					else 
					{
						Usart_Flag=0;
					}
				}
				break;
				
				case 2:
				{
					if(Res=='B') 	
					{
						Usart_Flag|=0x4000;//位置环功能字
					}
					else if(Res=='C')
					{
						Usart_Flag|=0x2000;//速度环功能字
					}
					else if(Res=='D') 
					{
						Usart_Flag|=0x1000;//扭矩环功能字
					}
					else if(Res=='E')
					{
						Usart_Flag|=0x0800;//给定值功能字
					}
					else 
					{
						Usart_Flag=0;
					}
					
				}
				break;
				
				case 24:
				{
					if(Res!='E') 
					{
						Usart_Flag=0;//帧尾
					}
					else 
					{
						Usart_Flag|=0x0400;//接收完成
						cnt=0;//
						i=0;
					}
				}		
				break;
				
				default:
				{
					USART_RX_BUF[i]=Res;
					i++;
					Usart_Flag++;//有效字符个数
				}
				break;
					
			}

  } 

}

/***********DMA1_Channel5接收完成中断************/
void DMA1_Channel5_IRQHandler() 
{
	u8 k=0;
    if(DMA_GetITStatus(DMA1_IT_TC5) == SET)
    {
		if(rx_buffer[0]=='A'&&rx_buffer[DMA_RX_LENGTH-1]=='E')//
		{
			Usart_Flag|=0x8000;//接收到帧头
			Usart_Flag|=0x0400;//接收完成
			LED1_OFF();
			if(rx_buffer[1]=='B')//滤波参数功能字
			{
				Usart_Flag|=0x4000;//			
			}
			else if(rx_buffer[1]=='C')//pitch姿态环功能字	
			{
				Usart_Flag|=0x2000;//
			}
			else if(rx_buffer[1]=='D')//pitch速度环功能字
			{
				Usart_Flag|=0x1000;//
			}
			else if(rx_buffer[1]=='E')//ROLL姿态环环功能字
			{
				Usart_Flag|=0x0800;//
			}
//			else if(rx_buffer[1]=='F')//ROLL速度环功能字
//			{
//				Usart_Flag|=0x0400;//
//			}
//			else if(rx_buffer[1]=='G')//设定值功能字
//			{
//				Usart_Flag|=0x0200;//给定值功能字
//			}
//			else if(rx_buffer[1]=='H')//清楚PID参数功能字
//			{
//				Usart_Flag|=0x0100;//给定值功能字
//			}
			else 
			{
				Usart_Flag=0;
			}
			for(k=0;k<DMA_RX_LENGTH-3;k++)
			{
				USART_RX_BUF[k]=rx_buffer[k+2];
				Usart_Flag++;//有效字符个数
			}
			
		}
		else if(rx_buffer[0]==0xAA&&rx_buffer[17]==0XEE)//帧头帧尾
		{
			if(rx_buffer[1]==0xF1)//功能字
			{
//				//遥控器蓝牙通讯发过来的数据
//// 				Rcdata.pitch	=((int16_t)rx_buffer[2])<<8|(int16_t)rx_buffer[3];
//// 				Rcdata.roll		=((int16_t)rx_buffer[4])<<8|(int16_t)rx_buffer[5];
//// 				Rcdata.yaw		=((int16_t)rx_buffer[6])<<8|(int16_t)rx_buffer[7];
//// 				Rcdata.thottle  =((int16_t)rx_buffer[8])<<8|(int16_t)rx_buffer[9];

//				Rcdata.pitch	=(short)(((int16_t)rx_buffer[2])<<8|(int16_t)rx_buffer[3]);
//				Rcdata.pitch  =Rcdata.pitch/21.0f;
//				Rcdata.roll		=(short)(((int16_t)rx_buffer[4])<<8|(int16_t)rx_buffer[5]);
//				Rcdata.roll   =Rcdata.roll/21.0f;
//				Rcdata.yaw		=(short)(((int16_t)rx_buffer[6])<<8|(int16_t)rx_buffer[7]);
//				Rcdata.yaw    =Rcdata.yaw/32.0f;
//				Rcdata.throttle  =(short)(((int16_t)rx_buffer[8])<<8|(int16_t)rx_buffer[9]);
//				Rcdata.throttle  =Rcdata.throttle/200.0f;
//				
//				Rcdata.pitchtrim=rx_buffer[10];
//				Rcdata.rolltrim =rx_buffer[11];
//				
//				Rcdata.Jindou		  =rx_buffer[12];//翻筋斗
//				Rcdata.Lock			  =rx_buffer[13];//电机锁住标志
//				Rcdata.Selft_Test	=rx_buffer[14];//陀螺仪自检标志
//				Rcdata.batvol		  =rx_buffer[15];//遥控器电池电压标志
//	      Rcdata.Mode       =rx_buffer[16];

//				Flag.nrf_error_cnt=0;
//			  RCData_Check(&Rcdata,&Copter); 
			}
		}
		else
		{
				Usart_Flag=0;
				printf("dma error\r\n");//for dbug		
		}
		
        DMA_ClearFlag(DMA1_FLAG_TC5);
        DMA_ClearITPendingBit(DMA1_FLAG_TC5);
    }
}



//功能：发送数据
//输入 ：buf数据包首地址 len数据包长度
//返回：无
void USART_Send_BUF(uint8_t *buf,uint8_t len)
{
	u8 i=0;
   for(i=0;i<len;i++)
 {
 // USART_SendData(USART1, buf[i]);
	  USART1->DR=buf[i];
		while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);//等待发送结束
 }
}

/***********************/
//功能：发送解算出来的姿态数据给上位机
//输入:姿态数据
//返回：无
//void ANO_Data_Send(float *pitch,float *roll,float *yaw)
//{
//	u8 _cnt=0;
//	u8 sum =0;
//	u8 i=0;
//	float _temp=0.0f;//先清零
//	
//	
//	tx_buffer[_cnt++]=0xAA;//帧头
//	tx_buffer[_cnt++]=0xAA;//帧头
//	tx_buffer[_cnt++]=0xF1;//功能字
//	tx_buffer[_cnt++]=0;//数据长度

//// 	_temp = (int)(*pitch*100);
//	_temp = *pitch;
//	tx_buffer[_cnt++]=BYTE3(_temp);
//	tx_buffer[_cnt++]=BYTE2(_temp);
//	tx_buffer[_cnt++]=BYTE1(_temp);
//	tx_buffer[_cnt++]=BYTE0(_temp);

//// 	_temp = (int)(*roll*100);
//	_temp = *roll;
//	tx_buffer[_cnt++]=BYTE3(_temp);
//	tx_buffer[_cnt++]=BYTE2(_temp);
//	tx_buffer[_cnt++]=BYTE1(_temp);
//	tx_buffer[_cnt++]=BYTE0(_temp);

//// 	_temp = (int)(*yaw*100);
//	_temp = *yaw;
//	tx_buffer[_cnt++]=BYTE3(_temp);
//	tx_buffer[_cnt++]=BYTE2(_temp);
//	tx_buffer[_cnt++]=BYTE1(_temp);
//	tx_buffer[_cnt++]=BYTE0(_temp);
//	
//	tx_buffer[3] = _cnt-4;//数据长度 16-4=12

//	
//for(i=0;i<_cnt;i++) sum += tx_buffer[i];	//求取校验值
//	tx_buffer[_cnt++]=sum;//校验值

//	DMA_Cmd(DMA1_Channel4, DISABLE); 
//	DMA_SetCurrDataCounter(DMA1_Channel4,DMA_TX_LENGTH);        
//	DMA_Cmd(DMA1_Channel4, ENABLE);
//}


void ANO_Data_Send_Test_(float *angle,
						 float *temp1,float *temp2,float *temp3,
						 signed short * temp4,signed short * temp5)
{
	u8 _cnt=0;
	u8 sum =0;
	u8 i=0;
	float _temp=0.0f;//先清零
	signed short _temp1=0;
	
	tx_buffer[_cnt++]=0xAA;//帧头
	tx_buffer[_cnt++]=0xAA;//帧头
	tx_buffer[_cnt++]=0xF1;//功能字
	tx_buffer[_cnt++]=0;//数据长度

	
		_temp = *angle;
	tx_buffer[_cnt++]=BYTE3(_temp);
	tx_buffer[_cnt++]=BYTE2(_temp);
	tx_buffer[_cnt++]=BYTE1(_temp);
	tx_buffer[_cnt++]=BYTE0(_temp);
	
	_temp = *temp1;
	tx_buffer[_cnt++]=BYTE3(_temp);
	tx_buffer[_cnt++]=BYTE2(_temp);
	tx_buffer[_cnt++]=BYTE1(_temp);
	tx_buffer[_cnt++]=BYTE0(_temp);

	_temp = *temp2;
	tx_buffer[_cnt++]=BYTE3(_temp);
	tx_buffer[_cnt++]=BYTE2(_temp);
	tx_buffer[_cnt++]=BYTE1(_temp);
	tx_buffer[_cnt++]=BYTE0(_temp);

	_temp = *temp3;
	tx_buffer[_cnt++]=BYTE3(_temp);
	tx_buffer[_cnt++]=BYTE2(_temp);
	tx_buffer[_cnt++]=BYTE1(_temp);
	tx_buffer[_cnt++]=BYTE0(_temp);
	
		_temp1 = *temp4;
// 	tx_buffer[_cnt++]=BYTE3(_temp1);
// 	tx_buffer[_cnt++]=BYTE2(_temp1);
	tx_buffer[_cnt++]=BYTE1(_temp1);
	tx_buffer[_cnt++]=BYTE0(_temp1);
	
		_temp1 = *temp5;
// 	tx_buffer[_cnt++]=BYTE3(_temp1);
// 	tx_buffer[_cnt++]=BYTE2(_temp1);
	tx_buffer[_cnt++]=BYTE1(_temp1);
	tx_buffer[_cnt++]=BYTE0(_temp1);
	
	tx_buffer[3] = _cnt-4;//数据长度 16-4=12

	
for(i=0;i<_cnt;i++) sum += tx_buffer[i];	//求取校验值
	tx_buffer[_cnt++]=sum;//校验值

	DMA_Cmd(DMA1_Channel4, DISABLE); 
	DMA_SetCurrDataCounter(DMA1_Channel4,DMA_TX_LENGTH);        
	DMA_Cmd(DMA1_Channel4, ENABLE);
}


void ANO_Data_Send_ALTVEL_Test_(float *angle,
								float *temp1,float *temp2,float *temp3,
								float *temp4,float *temp5,float *temp6,
								float *thottle,
								signed short * temp7,
								signed short * temp8)
{
	u8 _cnt=0;
	u8 sum =0;
	u8 i=0;
	float _temp=0.0f;//先清零
	signed short _temp1=0;
	
	tx_buffer[_cnt++]=0xAA;//帧头
	tx_buffer[_cnt++]=0xAA;//帧头
	tx_buffer[_cnt++]=0xF1;//功能字
	tx_buffer[_cnt++]=0;//数据长度

	
		_temp = *angle;
	tx_buffer[_cnt++]=BYTE3(_temp);
	tx_buffer[_cnt++]=BYTE2(_temp);
	tx_buffer[_cnt++]=BYTE1(_temp);
	tx_buffer[_cnt++]=BYTE0(_temp);
	
	_temp = *temp1;
	tx_buffer[_cnt++]=BYTE3(_temp);
	tx_buffer[_cnt++]=BYTE2(_temp);
	tx_buffer[_cnt++]=BYTE1(_temp);
	tx_buffer[_cnt++]=BYTE0(_temp);

	_temp = *temp2;
	tx_buffer[_cnt++]=BYTE3(_temp);
	tx_buffer[_cnt++]=BYTE2(_temp);
	tx_buffer[_cnt++]=BYTE1(_temp);
	tx_buffer[_cnt++]=BYTE0(_temp);

	_temp = *temp3;
	tx_buffer[_cnt++]=BYTE3(_temp);
	tx_buffer[_cnt++]=BYTE2(_temp);
	tx_buffer[_cnt++]=BYTE1(_temp);
	tx_buffer[_cnt++]=BYTE0(_temp);


	_temp = *temp4;
	tx_buffer[_cnt++]=BYTE3(_temp);
	tx_buffer[_cnt++]=BYTE2(_temp);
	tx_buffer[_cnt++]=BYTE1(_temp);
	tx_buffer[_cnt++]=BYTE0(_temp);

	_temp = *temp5;
	tx_buffer[_cnt++]=BYTE3(_temp);
	tx_buffer[_cnt++]=BYTE2(_temp);
	tx_buffer[_cnt++]=BYTE1(_temp);
	tx_buffer[_cnt++]=BYTE0(_temp);

	_temp = *temp6;
	tx_buffer[_cnt++]=BYTE3(_temp);
	tx_buffer[_cnt++]=BYTE2(_temp);
	tx_buffer[_cnt++]=BYTE1(_temp);
	tx_buffer[_cnt++]=BYTE0(_temp);
	
	_temp = *thottle;
	tx_buffer[_cnt++]=BYTE3(_temp);
	tx_buffer[_cnt++]=BYTE2(_temp);
	tx_buffer[_cnt++]=BYTE1(_temp);
	tx_buffer[_cnt++]=BYTE0(_temp);
	
		_temp1 = *temp7;
// 	tx_buffer[_cnt++]=BYTE3(_temp1);
// 	tx_buffer[_cnt++]=BYTE2(_temp1);
	tx_buffer[_cnt++]=BYTE1(_temp1);
	tx_buffer[_cnt++]=BYTE0(_temp1);
	
		_temp1 = *temp8;
// 	tx_buffer[_cnt++]=BYTE3(_temp1);
// 	tx_buffer[_cnt++]=BYTE2(_temp1);
	tx_buffer[_cnt++]=BYTE1(_temp1);
	tx_buffer[_cnt++]=BYTE0(_temp1);
	
	tx_buffer[3] = _cnt-4;//数据长度 16-4=12

	
for(i=0;i<_cnt;i++) sum += tx_buffer[i];	//求取校验值
	tx_buffer[_cnt++]=sum;//校验值

	DMA_Cmd(DMA1_Channel4, DISABLE); 
	DMA_SetCurrDataCounter(DMA1_Channel4,DMA_TX_LENGTH);        
	DMA_Cmd(DMA1_Channel4, ENABLE);
}

void ANO_ImuDataReturn(float *angle,
								float *temp1,float *temp2,float *temp3,
								float *temp4,float *temp5,float *temp6,
								float *thottle,
								signed short  temp7,
								signed short  temp8)
{
	u8 _cnt=0;
	u8 sum =0;
	u8 i=0;
	float _temp=0.0f;//先清零
	signed short _temp1=0;
	
	tx_buffer[_cnt++]=0xAA;//帧头
	tx_buffer[_cnt++]=0xAA;//帧头
	tx_buffer[_cnt++]=0xF1;//功能字
	tx_buffer[_cnt++]=0;//数据长度

	
		_temp = *angle;
	tx_buffer[_cnt++]=BYTE3(_temp);
	tx_buffer[_cnt++]=BYTE2(_temp);
	tx_buffer[_cnt++]=BYTE1(_temp);
	tx_buffer[_cnt++]=BYTE0(_temp);
	
	_temp = *temp1;
	tx_buffer[_cnt++]=BYTE3(_temp);
	tx_buffer[_cnt++]=BYTE2(_temp);
	tx_buffer[_cnt++]=BYTE1(_temp);
	tx_buffer[_cnt++]=BYTE0(_temp);

	_temp = *temp2;
	tx_buffer[_cnt++]=BYTE3(_temp);
	tx_buffer[_cnt++]=BYTE2(_temp);
	tx_buffer[_cnt++]=BYTE1(_temp);
	tx_buffer[_cnt++]=BYTE0(_temp);

	_temp = *temp3;
	tx_buffer[_cnt++]=BYTE3(_temp);
	tx_buffer[_cnt++]=BYTE2(_temp);
	tx_buffer[_cnt++]=BYTE1(_temp);
	tx_buffer[_cnt++]=BYTE0(_temp);


	_temp = *temp4;
	tx_buffer[_cnt++]=BYTE3(_temp);
	tx_buffer[_cnt++]=BYTE2(_temp);
	tx_buffer[_cnt++]=BYTE1(_temp);
	tx_buffer[_cnt++]=BYTE0(_temp);

	_temp = *temp5;
	tx_buffer[_cnt++]=BYTE3(_temp);
	tx_buffer[_cnt++]=BYTE2(_temp);
	tx_buffer[_cnt++]=BYTE1(_temp);
	tx_buffer[_cnt++]=BYTE0(_temp);

	_temp = *temp6;
	tx_buffer[_cnt++]=BYTE3(_temp);
	tx_buffer[_cnt++]=BYTE2(_temp);
	tx_buffer[_cnt++]=BYTE1(_temp);
	tx_buffer[_cnt++]=BYTE0(_temp);
	
	_temp = *thottle;
	tx_buffer[_cnt++]=BYTE3(_temp);
	tx_buffer[_cnt++]=BYTE2(_temp);
	tx_buffer[_cnt++]=BYTE1(_temp);
	tx_buffer[_cnt++]=BYTE0(_temp);
	
		_temp1 = temp7;
// 	tx_buffer[_cnt++]=BYTE3(_temp1);
// 	tx_buffer[_cnt++]=BYTE2(_temp1);
	tx_buffer[_cnt++]=BYTE1(_temp1);
	tx_buffer[_cnt++]=BYTE0(_temp1);
	
		_temp1 = temp8;
// 	tx_buffer[_cnt++]=BYTE3(_temp1);
// 	tx_buffer[_cnt++]=BYTE2(_temp1);
	tx_buffer[_cnt++]=BYTE1(_temp1);
	tx_buffer[_cnt++]=BYTE0(_temp1);
	
	tx_buffer[3] = _cnt-4;//数据长度 16-4=12

	
for(i=0;i<_cnt;i++) sum += tx_buffer[i];	//求取校验值
	tx_buffer[_cnt++]=sum;//校验值

	DMA_Cmd(DMA1_Channel4, DISABLE); 
	DMA_SetCurrDataCounter(DMA1_Channel4,DMA_TX_LENGTH);        
	DMA_Cmd(DMA1_Channel4, ENABLE);
}

void ANO_Comunication_Check(unsigned char data)
{
	u8 _cnt=0;
	u8 sum =0;
	u8 i=0;

	
	tx_buffer[_cnt++]=0xAA;//帧头
	tx_buffer[_cnt++]=0xAA;//帧头
	tx_buffer[_cnt++]=0xF2;//功能字
	tx_buffer[_cnt++]=0;//数据长度
	
	tx_buffer[_cnt++]=data;	
	
for(i=0;i<_cnt;i++) sum += tx_buffer[i];	//求取校验值
	tx_buffer[_cnt++]=sum;//校验值

	DMA_Cmd(DMA1_Channel4, DISABLE); 
	DMA_SetCurrDataCounter(DMA1_Channel4,DMA_TX_LENGTH);        
	DMA_Cmd(DMA1_Channel4, ENABLE);
	
}


/***********
*环形缓冲FIFO
*
************/
#if 0
/**************FIFO Begin**************************************************/  
/*溢出标志:0-正常,-1-溢出*/    
#define FLAGS_OVERRUN 0x0001    
/*   
        buf- 缓冲区地址   
        size- 大小   
        free- 空余容量   
        putP- 下一个数据写入位置 
        getP- 下一个数据独处位置 
*/    
struct FIFO8{    
         char *buf;    
         int putP,getP,size,free,flags;    
         pthread_mutex_t mutex;  
};    
    
int fifo8_init(struct FIFO8 *fifo,int size);    
int fifo8_put(struct FIFO8 *fifo, char data);  //not mutiply thread safe  
int fifo8_get(struct FIFO8 *fifo);     //not mutiply thread safe  
void fifo8_status(struct FIFO8 *fifo,int *len);    
void fifo8_free(struct FIFO8 *fifo,int *len);    
int fifo8_write(struct FIFO8 *fifo,char *data,int len);  
int fifo8_read(struct FIFO8 *fifo,char *data,int len);  
  
/*初始化*/   
int  fifo8_init(struct FIFO8 *fifo,int size) {    
      
    int ret=0;  
  
    fifo->flags=0;              
    fifo->free=size;    
    fifo->size=size;    
    fifo->putP=0;                       
    fifo->getP=0;    
      
    fifo->buf=(char *)malloc(size);  
    if(fifo->buf==NULL){  
        LOGI("malloc fifo buffer failed!\n");  
        return -1;  
    }  
      
    ret = pthread_mutex_init(&(fifo->mutex),NULL);  
    if(ret <0){  
        LOGI("init mutex failed!\n");  
        return -1;  
    }  
  
    return 0;     
}    
  
/*向FIFO 中写入一个数据 */    
int fifo8_putPut(struct FIFO8 *fifo, char data)  {    
    if(fifo->free==0){    
        fifo->flags |= FLAGS_OVERRUN;    
        return -1;    
    }    
    fifo->buf[fifo->putP] = data;    
    fifo->putP++;    
    //循环队列缓冲区   
    if(fifo->putP == fifo->size){    
        fifo->putP = 0;    
    }    
    fifo->free--;    
  
    return 0;    
}    
/*从FIFO 中读取出一个数据 */  
int fifo8_get(struct FIFO8 *fifo)  {    
    int data;    
    if(fifo->free == fifo->size){    
        return -1;    
    }    
      
    data = fifo->buf[fifo->getP++];    
    //fifo->getP++;    
    if(fifo->getP == fifo->size){//用来实现循环   
        fifo->getP = 0;    
    }    
    fifo->free++;    
    return data;    
}    
/*写入len个字节,返回写入的字节数*/  
int fifo8_write(struct FIFO8 *fifo,char *data,int len){  
      
    int i=0;  
    pthread_mutex_lock(&(fifo->mutex));  
      
    if(fifo->free < len) {  
        pthread_mutex_unlock(&(fifo->mutex));      
        LOGI("the free size in not enough!\n");  
        return 0;  
    }  
    else {  
        LOGI("current fifo->putP =%d \n",fifo->putP);  
        for(i=0;i<len;i++){  
            fifo->buf[fifo->putP++] = *(data+i);    
            //循环队列缓冲区   
            if(fifo->putP == fifo->size){    
                fifo->putP = 0;    
            }    
            fifo->free--;    
        }  
    }  
  
    pthread_mutex_unlock(&(fifo->mutex));      
      
    return len;  
}  
/*读出len个字节,返回读出的字节数*/  
int fifo8_read(struct FIFO8 *fifo,char *data,int len){  
      
    int i=0;  
    pthread_mutex_lock(&(fifo->mutex));  
      
    if(fifo->size!=fifo->free){  
        LOGI("current fifo->getP =%d \n",fifo->getP);  
        for(i=0;  ; i++){  
            *(data+i) =fifo->buf[fifo->getP++];    
            if(fifo->getP == fifo->size){//用来实现循环   
                fifo->getP = 0;    
            }    
            fifo->free++;    
            if(fifo->size==fifo->free){  
                LOGI("the buffer is no data left!\n");  
                pthread_mutex_unlock(&(fifo->mutex));  
                return i+1;  
            }  
            if(i+1==len){  
                LOGI("read data finish!\n");  
                break;  
            }  
        }  
    }  
    else{  
        LOGI("the buffer is empty!\n");  
        pthread_mutex_unlock(&(fifo->mutex));      
        return 0;  
    }  
      
    pthread_mutex_unlock(&(fifo->mutex));      
      
    return len;  
}  
  
/*缓冲区被使用量*/    
void fifo8_status(struct FIFO8 *fifo,int *used)  {    
    pthread_mutex_lock(&(fifo->mutex));  
    *used = fifo->size - fifo->free;   
    pthread_mutex_unlock(&(fifo->mutex));      
}    
/*缓冲区剩余容量*/  
void fifo8_free(struct FIFO8 *fifo ,int *free)  {   
    pthread_mutex_lock(&(fifo->mutex));   
    *free = fifo->free;    
    pthread_mutex_unlock(&(fifo->mutex));  
}  

#endif
