#include "us100dataget.h"
#include "usart.h"

const unsigned char Usart4_DMARX_Length=2;
unsigned char 		usart4_rxbuffer[Usart4_DMARX_Length]={0};
const unsigned char Usart2_DMATX_Length=1;
unsigned char 		usart2_txbuffer[Usart2_DMATX_Length]={0X55};
extern signed short 		Us100_Distance;
/****************usartx_init**********************/
//串口DMA发送接收
//
void usart2_usart4_init(u32 bound)
{
    //GPIO端口设置
    GPIO_InitTypeDef  GPIO_InitStructure;
	  USART_InitTypeDef USART_InitStructure;
	  NVIC_InitTypeDef  NVIC_InitStructure;
    DMA_InitTypeDef   dma;
	
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC, ENABLE);//使能USARTX，GPIOA时钟
//	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2|RCC_APB1Periph_USART3,ENABLE);
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2|RCC_APB1Periph_UART4,ENABLE);
	  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1|RCC_AHBPeriph_DMA2,ENABLE);//初始化DMA1/DMA2时钟 	 
	  USART_DeInit(USART2);  //复位串口2
//	  USART_DeInit(USART3);  //复位串口3
	  USART_DeInit(UART4);  //复位串口4
	 //USART2_TX   PA.3
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA.2
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
    GPIO_Init(GPIOA, &GPIO_InitStructure); //初始化PA9
   
    //USART4_RX	  PC.11
	//AF USART3_RX 复用
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
    GPIO_Init(GPIOC, &GPIO_InitStructure);  //初始化PA10

   //Usart1_RX NVIC 配置
//    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
// 	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//抢占优先级3
// 	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//子优先级3
// 	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
// 	  NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
 /**************************************************************/ 
   //USART2 初始化设置
	 USART_InitStructure.USART_BaudRate = bound;//一般设置为;
 	 USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
//	 USART_InitStructure.USART_WordLength = USART_WordLength_9b;//字长为9位数据格式
	 USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
//	 USART_InitStructure.USART_Parity = USART_Parity_Even;//偶校验位
 	 USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	 USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
//	 USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	 USART_InitStructure.USART_Mode =  USART_Mode_Tx;	//仅仅发模式
    USART_Init(USART2, &USART_InitStructure); //初始化串口
//  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启中断
    USART_Cmd(USART2, ENABLE);                    //使能串口 
/********************************************************************/	
	   //USART4 初始化设置
	 USART_InitStructure.USART_BaudRate = bound;//一般设置为;
 	 USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
//	 USART_InitStructure.USART_WordLength = USART_WordLength_9b;//字长为9位数据格式
	 USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
//	 USART_InitStructure.USART_Parity = USART_Parity_Even;//偶校验位
 	 USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	 USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
//	 USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	 USART_InitStructure.USART_Mode = USART_Mode_Rx;	//收模式
    USART_Init(UART4, &USART_InitStructure); //初始化串口
//  USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//开启中断
    USART_Cmd(UART4, ENABLE);                    //使能串口 
/*******************************USART4接收DMA配置******************************************/	
    dma.DMA_PeripheralBaseAddr = (uint32_t)&(UART4->DR);//USART4到内存
    dma.DMA_MemoryBaseAddr = (uint32_t)usart4_rxbuffer; //接收缓冲  
    dma.DMA_DIR = DMA_DIR_PeripheralSRC;//外设为传输源头
    dma.DMA_BufferSize = Usart4_DMARX_Length;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_Mode = DMA_Mode_Circular;
    dma.DMA_Priority = DMA_Priority_VeryHigh;
    dma.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA2_Channel3,&dma);

		USART_DMACmd(UART4,USART_DMAReq_Rx,ENABLE);//使能USART4RX DMA2传输 
    DMA_ITConfig(DMA2_Channel3, DMA_IT_TC, ENABLE);//DMA2_Channel3传输完成中断
    DMA_Cmd(DMA2_Channel3, ENABLE);//使能DMA2_Channel3
	
/***********************DMA2_Channel3断配置***************************/
    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Channel3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
  
		DMA_ITConfig(DMA2_Channel3, DMA_IT_TC, ENABLE);//DMA2_Channel3传输完成中断
/**************USART2发送DMA配置****************************/    
    dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART2->DR);//外设地址
    dma.DMA_MemoryBaseAddr = (uint32_t)usart2_txbuffer;//内存地址   
    dma.DMA_DIR = DMA_DIR_PeripheralDST;//外设为传输目的地
    dma.DMA_BufferSize = Usart2_DMATX_Length;//传输长度
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址不可变
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;//内存地址可变
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_Mode = DMA_Mode_Normal;
    dma.DMA_Priority = DMA_Priority_VeryHigh;
    dma.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel7,&dma);
   	USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);//使能USART2TX DMA1传输   
    DMA_Cmd(DMA1_Channel7, DISABLE);//禁止使DMA1_Channel7
	
/***********************DMA1_Channel4中断配置***************************/
//     NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
//     NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//     NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//     NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//     NVIC_Init(&NVIC_InitStructure);
  
// 	DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);//DMA1_Channel4传输完成中断
/*********************************************************************/	
}

/***********DMA2_Channel3接收完成中断************/
void DMA2_Channel3_IRQHandler() 
{
    if(DMA_GetITStatus(DMA2_IT_TC3) == SET)
    {
		    Us100_Distance=usart4_rxbuffer[0]*256+usart4_rxbuffer[1];//获取距离，单位mm
//		printf("l:%d\r\n",Us100_Distance);
        DMA_ClearFlag(DMA2_FLAG_TC3);
        DMA_ClearITPendingBit(DMA2_FLAG_TC3);
    }
}


void Usart2_DMA_Tx_(unsigned char* temp)
{
	DMA_Cmd(DMA1_Channel7, DISABLE); 
	DMA_SetCurrDataCounter(DMA1_Channel7,Usart2_DMATX_Length);        
	DMA_Cmd(DMA1_Channel7, ENABLE);
}

void US100_DIATANCE_GET(void)
{
	DMA_Cmd(DMA1_Channel7, DISABLE); 
	DMA_SetCurrDataCounter(DMA1_Channel7,Usart2_DMATX_Length);        
	DMA_Cmd(DMA1_Channel7, ENABLE);
}

void US100_MODULE_Init(void)
{
	usart2_usart4_init(9600);//USART2 UART4 DMA用于US100模块测量距离
}

//功能：发送数据
//输入 ：buf数据包首地址 len数据包长度
//返回：无
//void USART_Send_BUF(uint8_t *buf,uint8_t len)
//{
//	u8 i=0;
//   for(i=0;i<len;i++)
// {
// // USART_SendData(USART1, buf[i]);
//	  USART1->DR=buf[i];
//		while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);//等待发送结束
// }
//}
