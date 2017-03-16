#include "main.h"
#include "stm32f10x_dma.h"
#include "globalvariable.h"
////////////////////////////////////////////////////////////////////////////////// 	 
//���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_UCOS
#include "includes.h"					//ucos ʹ��	  
#endif
//////////////////////////////////////////////////////////////////////////////////	 
////////////////////////////////////////////////////////////////////////////////// 	  
 

//////////////////////////////////////////////////////////////////
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0){;}//ѭ������,ֱ���������   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 

/*ʹ��microLib�ķ���*/
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

//����1�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	
unsigned char USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
unsigned char tx_buffer[DMA_TX_LENGTH]={0};
unsigned char rx_buffer[DMA_RX_LENGTH]={0};//DMA���ջ���
unsigned short Usart_Flag=0x0000;//���ڽ��ܱ�־����
//bit15:����֡ͷ��־
//bit14:������
//bit13:������
//bit12:������
//bit11:������
//bit10:���յ�֡β
//bit0~9:���յ���Ч���ݳ���
//��ʼ��IO ����1 
//bound:������
/****************usart1_init**********************/
//����1DMA���ͽ���
//żУ�� һ���ֽ���Ҫ9λ����
void uart_init(u32 bound)
{
    //GPIO�˿�����
      GPIO_InitTypeDef  GPIO_InitStructure;
	  USART_InitTypeDef USART_InitStructure;
	  NVIC_InitTypeDef  NVIC_InitStructure;
      DMA_InitTypeDef   dma;
	
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��USART1��GPIOAʱ��
	  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);//��ʼ��DMA1ʱ�� 	 
	  USART_DeInit(USART1);  //��λ����1
	 //USART1_TX   PA.9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
    GPIO_Init(GPIOA, &GPIO_InitStructure); //��ʼ��PA9
   
    //USART1_RX	  PA.10
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
    GPIO_Init(GPIOA, &GPIO_InitStructure);  //��ʼ��PA10

   //Usart1_RX NVIC ����
//    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
// 	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//��ռ���ȼ�3
// 	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//�����ȼ�3
// 	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
// 	  NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
  
   //USART ��ʼ������
	 USART_InitStructure.USART_BaudRate = bound;//һ������Ϊ;
// 	 USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	 USART_InitStructure.USART_WordLength = USART_WordLength_9b;//�ֳ�Ϊ9λ���ݸ�ʽ
	 USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	 USART_InitStructure.USART_Parity = USART_Parity_Even;//żУ��λ
// 	 USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	 USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	 USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

    USART_Init(USART1, &USART_InitStructure); //��ʼ������
//  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//�����ж�
    USART_Cmd(USART1, ENABLE);                    //ʹ�ܴ��� 
/*******************************USART����DMA����******************************************/	
    dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);//�ڴ浽USART1
    dma.DMA_MemoryBaseAddr = (uint32_t)rx_buffer;   
    dma.DMA_DIR = DMA_DIR_PeripheralSRC;//����Ϊ����Դͷ
    dma.DMA_BufferSize = DMA_RX_LENGTH;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_Mode = DMA_Mode_Circular;
    dma.DMA_Priority = DMA_Priority_VeryHigh;
    dma.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel5,&dma);

	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);//ʹ��USART1RX DMA1���� 
    DMA_ITConfig(DMA1_Channel5, DMA_IT_TC, ENABLE);//DMA1_Channel5��������ж�
    DMA_Cmd(DMA1_Channel5, ENABLE);//ʹ��DMA1_Channel5
	
/***********************DMA1_Channel5�ж�����***************************/
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
  
	DMA_ITConfig(DMA1_Channel5, DMA_IT_TC, ENABLE);//DMA1_Channel5��������ж�
/**************USART����DMA����****************************/    
    dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);//�����ַ
    dma.DMA_MemoryBaseAddr = (uint32_t)tx_buffer;//�ڴ��ַ   
    dma.DMA_DIR = DMA_DIR_PeripheralDST;//����Ϊ����Ŀ�ĵ�
    dma.DMA_BufferSize = DMA_TX_LENGTH;//���䳤��
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//�����ַ���ɱ�
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;//�ڴ��ַ�ɱ�
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_Mode = DMA_Mode_Normal;
    dma.DMA_Priority = DMA_Priority_VeryHigh;
    dma.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel4,&dma);
   	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);//ʹ��USART1TX DMA1����   
    DMA_Cmd(DMA1_Channel4, DISABLE);//��ֹʹ��DMA1_Channel4
	
/***********************DMA1_Channel4�ж�����***************************/
//     NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
//     NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//     NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//     NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//     NVIC_Init(&NVIC_InitStructure);
  
// 	DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);//DMA1_Channel4��������ж�
/*********************************************************************/	
}

//֡��ʽ��AB��100.00��200.00��300.00E
//֡��ʽ��AC��100.00��200.00��300.00E
//֡��ʽ��AD��100.00��200.00��300.00E
//֡��ʽ��AE��100.00��200.00��300.00E
void USART1_IRQHandler(void) //����1�жϷ������
{
	u8 Res=0;
	static u8 cnt=0,i=0;
	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
	{
			Res =USART_ReceiveData(USART1);//(USART1->DR);	//��ȡ���յ�������
			cnt++;
			switch (cnt)
			{
				case 1:
				{
					if(Res=='A') 
					{
						Usart_Flag|=0x8000;//���յ�֡ͷ
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
						Usart_Flag|=0x4000;//λ�û�������
					}
					else if(Res=='C')
					{
						Usart_Flag|=0x2000;//�ٶȻ�������
					}
					else if(Res=='D') 
					{
						Usart_Flag|=0x1000;//Ť�ػ�������
					}
					else if(Res=='E')
					{
						Usart_Flag|=0x0800;//����ֵ������
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
						Usart_Flag=0;//֡β
					}
					else 
					{
						Usart_Flag|=0x0400;//�������
						cnt=0;//
						i=0;
					}
				}		
				break;
				
				default:
				{
					USART_RX_BUF[i]=Res;
					i++;
					Usart_Flag++;//��Ч�ַ�����
				}
				break;
					
			}

  } 

}

/***********DMA1_Channel5��������ж�************/
void DMA1_Channel5_IRQHandler() 
{
	u8 k=0;
    if(DMA_GetITStatus(DMA1_IT_TC5) == SET)
    {
		if(rx_buffer[0]=='A'&&rx_buffer[DMA_RX_LENGTH-1]=='E')//
		{
			Usart_Flag|=0x8000;//���յ�֡ͷ
			Usart_Flag|=0x0400;//�������
			LED1_OFF();
			if(rx_buffer[1]=='B')//�˲�����������
			{
				Usart_Flag|=0x4000;//			
			}
			else if(rx_buffer[1]=='C')//pitch��̬��������	
			{
				Usart_Flag|=0x2000;//
			}
			else if(rx_buffer[1]=='D')//pitch�ٶȻ�������
			{
				Usart_Flag|=0x1000;//
			}
			else if(rx_buffer[1]=='E')//ROLL��̬����������
			{
				Usart_Flag|=0x0800;//
			}
//			else if(rx_buffer[1]=='F')//ROLL�ٶȻ�������
//			{
//				Usart_Flag|=0x0400;//
//			}
//			else if(rx_buffer[1]=='G')//�趨ֵ������
//			{
//				Usart_Flag|=0x0200;//����ֵ������
//			}
//			else if(rx_buffer[1]=='H')//���PID����������
//			{
//				Usart_Flag|=0x0100;//����ֵ������
//			}
			else 
			{
				Usart_Flag=0;
			}
			for(k=0;k<DMA_RX_LENGTH-3;k++)
			{
				USART_RX_BUF[k]=rx_buffer[k+2];
				Usart_Flag++;//��Ч�ַ�����
			}
			
		}
		else if(rx_buffer[0]==0xAA&&rx_buffer[17]==0XEE)//֡ͷ֡β
		{
			if(rx_buffer[1]==0xF1)//������
			{
				;
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



//���ܣ���������
//���� ��buf���ݰ��׵�ַ len���ݰ�����
//���أ���
void USART_Send_BUF(uint8_t *buf,uint8_t len)
{
	u8 i=0;
   for(i=0;i<len;i++)
 {
 // USART_SendData(USART1, buf[i]);
	  USART1->DR=buf[i];
	  while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);//�ȴ����ͽ���
 }
}


void ANO_Data_Send_Test_(float *angle,
						 float *temp1,float *temp2,float *temp3,
						 signed short * temp4,signed short * temp5)
{
	u8 _cnt=0;
	u8 sum =0;
	u8 i=0;
	float _temp=0.0f;//������
	signed short _temp1=0;
	
	tx_buffer[_cnt++]=0xAA;//֡ͷ
	tx_buffer[_cnt++]=0xAA;//֡ͷ
	tx_buffer[_cnt++]=0xF1;//������
	tx_buffer[_cnt++]=0;//���ݳ���

	
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
	
	tx_buffer[3] = _cnt-4;//���ݳ��� 16-4=12

	
for(i=0;i<_cnt;i++) sum += tx_buffer[i];	//��ȡУ��ֵ
	tx_buffer[_cnt++]=sum;//У��ֵ

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
	float _temp=0.0f;//������
	signed short _temp1=0;
	
	tx_buffer[_cnt++]=0xAA;//֡ͷ
	tx_buffer[_cnt++]=0xAA;//֡ͷ
	tx_buffer[_cnt++]=0xF1;//������
	tx_buffer[_cnt++]=0;//���ݳ���

	
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
	
	tx_buffer[3] = _cnt-4;//���ݳ��� 16-4=12

	
for(i=0;i<_cnt;i++) sum += tx_buffer[i];	//��ȡУ��ֵ
	tx_buffer[_cnt++]=sum;//У��ֵ

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
	float _temp=0.0f;//������
	signed short _temp1=0;
	
	tx_buffer[_cnt++]=0xAA;//֡ͷ
	tx_buffer[_cnt++]=0xAA;//֡ͷ
	tx_buffer[_cnt++]=0xF1;//������
	tx_buffer[_cnt++]=0;//���ݳ���

	
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
	
	tx_buffer[3] = _cnt-4;//���ݳ��� 16-4=12

	
for(i=0;i<_cnt;i++) sum += tx_buffer[i];	//��ȡУ��ֵ
	tx_buffer[_cnt++]=sum;//У��ֵ

	DMA_Cmd(DMA1_Channel4, DISABLE); 
	DMA_SetCurrDataCounter(DMA1_Channel4,DMA_TX_LENGTH);        
	DMA_Cmd(DMA1_Channel4, ENABLE);
}

void ANO_Comunication_Check(unsigned char data)
{
	u8 _cnt=0;
	u8 sum =0;
	u8 i=0;

	
	tx_buffer[_cnt++]=0xAA;//֡ͷ
	tx_buffer[_cnt++]=0xAA;//֡ͷ
	tx_buffer[_cnt++]=0xF2;//������
	tx_buffer[_cnt++]=0;//���ݳ���
	
	tx_buffer[_cnt++]=data;	
	
for(i=0;i<_cnt;i++) sum += tx_buffer[i];	//��ȡУ��ֵ
	tx_buffer[_cnt++]=sum;//У��ֵ

	DMA_Cmd(DMA1_Channel4, DISABLE); 
	DMA_SetCurrDataCounter(DMA1_Channel4,DMA_TX_LENGTH);        
	DMA_Cmd(DMA1_Channel4, ENABLE);
	
}


/***********
*���λ���FIFO
*
************/
#if 0
/**************FIFO Begin**************************************************/  
/*�����־:0-����,-1-���*/    
#define FLAGS_OVERRUN 0x0001    
/*   
        buf- ��������ַ   
        size- ��С   
        free- ��������   
        putP- ��һ������д��λ�� 
        getP- ��һ�����ݶ���λ�� 
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
  
/*��ʼ��*/   
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
  
/*��FIFO ��д��һ������ */    
int fifo8_putPut(struct FIFO8 *fifo, char data)  {    
    if(fifo->free==0){    
        fifo->flags |= FLAGS_OVERRUN;    
        return -1;    
    }    
    fifo->buf[fifo->putP] = data;    
    fifo->putP++;    
    //ѭ�����л�����   
    if(fifo->putP == fifo->size){    
        fifo->putP = 0;    
    }    
    fifo->free--;    
  
    return 0;    
}    
/*��FIFO �ж�ȡ��һ������ */  
int fifo8_get(struct FIFO8 *fifo)  {    
    int data;    
    if(fifo->free == fifo->size){    
        return -1;    
    }    
      
    data = fifo->buf[fifo->getP++];    
    //fifo->getP++;    
    if(fifo->getP == fifo->size){//����ʵ��ѭ��   
        fifo->getP = 0;    
    }    
    fifo->free++;    
    return data;    
}    
/*д��len���ֽ�,����д����ֽ���*/  
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
            //ѭ�����л�����   
            if(fifo->putP == fifo->size){    
                fifo->putP = 0;    
            }    
            fifo->free--;    
        }  
    }  
  
    pthread_mutex_unlock(&(fifo->mutex));      
      
    return len;  
}  
/*����len���ֽ�,���ض������ֽ���*/  
int fifo8_read(struct FIFO8 *fifo,char *data,int len){  
      
    int i=0;  
    pthread_mutex_lock(&(fifo->mutex));  
      
    if(fifo->size!=fifo->free){  
        LOGI("current fifo->getP =%d \n",fifo->getP);  
        for(i=0;  ; i++){  
            *(data+i) =fifo->buf[fifo->getP++];    
            if(fifo->getP == fifo->size){//����ʵ��ѭ��   
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
  
/*��������ʹ����*/    
void fifo8_status(struct FIFO8 *fifo,int *used)  {    
    pthread_mutex_lock(&(fifo->mutex));  
    *used = fifo->size - fifo->free;   
    pthread_mutex_unlock(&(fifo->mutex));      
}    
/*������ʣ������*/  
void fifo8_free(struct FIFO8 *fifo ,int *free)  {   
    pthread_mutex_lock(&(fifo->mutex));   
    *free = fifo->free;    
    pthread_mutex_unlock(&(fifo->mutex));  
}  

#endif
