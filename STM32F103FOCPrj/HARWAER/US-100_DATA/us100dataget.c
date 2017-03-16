#include "us100dataget.h"
#include "usart.h"

const unsigned char Usart4_DMARX_Length=2;
unsigned char 		usart4_rxbuffer[Usart4_DMARX_Length]={0};
const unsigned char Usart2_DMATX_Length=1;
unsigned char 		usart2_txbuffer[Usart2_DMATX_Length]={0X55};
extern signed short 		Us100_Distance;
/****************usartx_init**********************/
//����DMA���ͽ���
//
void usart2_usart4_init(u32 bound)
{
    //GPIO�˿�����
    GPIO_InitTypeDef  GPIO_InitStructure;
	  USART_InitTypeDef USART_InitStructure;
	  NVIC_InitTypeDef  NVIC_InitStructure;
    DMA_InitTypeDef   dma;
	
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC, ENABLE);//ʹ��USARTX��GPIOAʱ��
//	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2|RCC_APB1Periph_USART3,ENABLE);
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2|RCC_APB1Periph_UART4,ENABLE);
	  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1|RCC_AHBPeriph_DMA2,ENABLE);//��ʼ��DMA1/DMA2ʱ�� 	 
	  USART_DeInit(USART2);  //��λ����2
//	  USART_DeInit(USART3);  //��λ����3
	  USART_DeInit(UART4);  //��λ����4
	 //USART2_TX   PA.3
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA.2
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
    GPIO_Init(GPIOA, &GPIO_InitStructure); //��ʼ��PA9
   
    //USART4_RX	  PC.11
	//AF USART3_RX ����
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
    GPIO_Init(GPIOC, &GPIO_InitStructure);  //��ʼ��PA10

   //Usart1_RX NVIC ����
//    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
// 	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//��ռ���ȼ�3
// 	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//�����ȼ�3
// 	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
// 	  NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
 /**************************************************************/ 
   //USART2 ��ʼ������
	 USART_InitStructure.USART_BaudRate = bound;//һ������Ϊ;
 	 USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
//	 USART_InitStructure.USART_WordLength = USART_WordLength_9b;//�ֳ�Ϊ9λ���ݸ�ʽ
	 USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
//	 USART_InitStructure.USART_Parity = USART_Parity_Even;//żУ��λ
 	 USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	 USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
//	 USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
	 USART_InitStructure.USART_Mode =  USART_Mode_Tx;	//������ģʽ
    USART_Init(USART2, &USART_InitStructure); //��ʼ������
//  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//�����ж�
    USART_Cmd(USART2, ENABLE);                    //ʹ�ܴ��� 
/********************************************************************/	
	   //USART4 ��ʼ������
	 USART_InitStructure.USART_BaudRate = bound;//һ������Ϊ;
 	 USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
//	 USART_InitStructure.USART_WordLength = USART_WordLength_9b;//�ֳ�Ϊ9λ���ݸ�ʽ
	 USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
//	 USART_InitStructure.USART_Parity = USART_Parity_Even;//żУ��λ
 	 USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	 USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
//	 USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
	 USART_InitStructure.USART_Mode = USART_Mode_Rx;	//��ģʽ
    USART_Init(UART4, &USART_InitStructure); //��ʼ������
//  USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//�����ж�
    USART_Cmd(UART4, ENABLE);                    //ʹ�ܴ��� 
/*******************************USART4����DMA����******************************************/	
    dma.DMA_PeripheralBaseAddr = (uint32_t)&(UART4->DR);//USART4���ڴ�
    dma.DMA_MemoryBaseAddr = (uint32_t)usart4_rxbuffer; //���ջ���  
    dma.DMA_DIR = DMA_DIR_PeripheralSRC;//����Ϊ����Դͷ
    dma.DMA_BufferSize = Usart4_DMARX_Length;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_Mode = DMA_Mode_Circular;
    dma.DMA_Priority = DMA_Priority_VeryHigh;
    dma.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA2_Channel3,&dma);

		USART_DMACmd(UART4,USART_DMAReq_Rx,ENABLE);//ʹ��USART4RX DMA2���� 
    DMA_ITConfig(DMA2_Channel3, DMA_IT_TC, ENABLE);//DMA2_Channel3��������ж�
    DMA_Cmd(DMA2_Channel3, ENABLE);//ʹ��DMA2_Channel3
	
/***********************DMA2_Channel3������***************************/
    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Channel3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
  
		DMA_ITConfig(DMA2_Channel3, DMA_IT_TC, ENABLE);//DMA2_Channel3��������ж�
/**************USART2����DMA����****************************/    
    dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART2->DR);//�����ַ
    dma.DMA_MemoryBaseAddr = (uint32_t)usart2_txbuffer;//�ڴ��ַ   
    dma.DMA_DIR = DMA_DIR_PeripheralDST;//����Ϊ����Ŀ�ĵ�
    dma.DMA_BufferSize = Usart2_DMATX_Length;//���䳤��
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//�����ַ���ɱ�
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;//�ڴ��ַ�ɱ�
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_Mode = DMA_Mode_Normal;
    dma.DMA_Priority = DMA_Priority_VeryHigh;
    dma.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel7,&dma);
   	USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);//ʹ��USART2TX DMA1����   
    DMA_Cmd(DMA1_Channel7, DISABLE);//��ֹʹDMA1_Channel7
	
/***********************DMA1_Channel4�ж�����***************************/
//     NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
//     NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//     NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//     NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//     NVIC_Init(&NVIC_InitStructure);
  
// 	DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);//DMA1_Channel4��������ж�
/*********************************************************************/	
}

/***********DMA2_Channel3��������ж�************/
void DMA2_Channel3_IRQHandler() 
{
    if(DMA_GetITStatus(DMA2_IT_TC3) == SET)
    {
		    Us100_Distance=usart4_rxbuffer[0]*256+usart4_rxbuffer[1];//��ȡ���룬��λmm
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
	usart2_usart4_init(9600);//USART2 UART4 DMA����US100ģ���������
}

//���ܣ���������
//���� ��buf���ݰ��׵�ַ len���ݰ�����
//���أ���
//void USART_Send_BUF(uint8_t *buf,uint8_t len)
//{
//	u8 i=0;
//   for(i=0;i<len;i++)
// {
// // USART_SendData(USART1, buf[i]);
//	  USART1->DR=buf[i];
//		while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);//�ȴ����ͽ���
// }
//}
