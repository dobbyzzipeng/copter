//***************************************************************************************

//***************************************************************************************
#include "usart.h"
#include "delay.h"
#include "led.h"
#include "spi_nrf24l01.h"
#include "stm32f10x.h"
//#include "MOTOR_CONTROL.h"
#include "globalvariable.h"
/******************************************************************************
							IO�ӿں궨��
*******************************************************************************/ 
#define NRF_CE_GPIO		GPIOC		
#define NRF_CE_Pin		GPIO_Pin_4	
#define NRF_CSN_GPIO	GPIOA		
#define NRF_CSN_Pin		GPIO_Pin_4	
#define NRF_IRQ_GPIO	GPIOC
#define NRF_IRQ_Pin		GPIO_Pin_5

#define NRF_CE_H   		NRF_CE_GPIO ->BSRR = NRF_CE_Pin  //CE�ߵ�ƽ
#define NRF_CE_L   		NRF_CE_GPIO ->BRR  = NRF_CE_Pin  //CE�͵�ƽ
#define NRF_CSN_H  		NRF_CSN_GPIO->BSRR = NRF_CSN_Pin //CSN�ߵ�ƽ
#define NRF_CSN_L  		NRF_CSN_GPIO->BRR  = NRF_CSN_Pin //CSN�ߵ�ƽ
#define	NRF_IRQ_Read	NRF_IRQ_GPIO->IDR  & NRF_IRQ_Pin //IRQ������

/******************************************************************************
							��������
*******************************************************************************/ 

uint8_t NRF24L01_RXDATA[NRF_DATA_Length]={0};//nrf24l01���յ�������
uint8_t NRF24L01_TXDATA[NRF_DATA_Length]={0};//nrf24l01��Ҫ���͵�����
static uint8_t TX_ADDRESS[5]= {0x1A,0x3B,0x5C,0x7D,0x9E};//40λ���ص�ַ
static uint8_t RX_ADDRESS[5]= {0x1A,0x3B,0x5C,0x7D,0x9E};//40λ���յ�ַ
static uint16_t Nrf_Erro=0;//NRF����״̬���Ź�����


/******************************************************************************
����ԭ��:	void SPI1_Init(void)
��������:	��ʼ��SPI����
*******************************************************************************/ 
void SPI1_Init(void)
{
	SPI_InitTypeDef SPI_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
//	NVIC_InitTypeDef 		 NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	//����SCK,MISO,MOSI����  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //���츴��
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//����CE����
	GPIO_InitStructure.GPIO_Pin = NRF_CE_Pin; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //�������
	GPIO_Init(NRF_CE_GPIO, &GPIO_InitStructure);
	//����CSN����
	GPIO_InitStructure.GPIO_Pin = NRF_CSN_Pin; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //�������
	GPIO_Init(NRF_CSN_GPIO, &GPIO_InitStructure);	
	//����IRQ����

	GPIO_InitStructure.GPIO_Pin = NRF_IRQ_Pin; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //��������
	GPIO_Init(NRF_IRQ_GPIO, &GPIO_InitStructure);	
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource5);
    EXTI_InitStructure.EXTI_Line=EXTI_Line5;
    EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;//�ⲿ�ж�
    EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Falling;//�½��ش���
    EXTI_InitStructure.EXTI_LineCmd=ENABLE;
    EXTI_Init(&EXTI_InitStructure);
	
//	 NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure);
	
	
	NRF_CSN_H;	//��ֹNRF����
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; //˫��ȫ˫�� 
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master; //��ģʽ 
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b; //���ݴ�С8λ 
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low; //ʱ�Ӽ��ԣ�����ʱΪ�� 
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge; //��1��������Ч��������Ϊ����ʱ�� 
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;  //NSS�ź���������� 
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; //8��Ƶ��9MHz 
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB; //��λ��ǰ 
	SPI_InitStructure.SPI_CRCPolynomial = 7; 
	SPI_Init(SPI1, &SPI_InitStructure); 
	   
	SPI_Cmd(SPI1, ENABLE);//ʹ�� SPI1
	
// 	PrintString("\r\n SPI1     ��ʼ����ɣ�");
}
/******************************************************************************
����ԭ��:	uint8_t SPI_RW(uint8_t data) 
��������:	SPI���߶�д
�� �� ֵ��	����SPI���߶�ȡ����
*******************************************************************************/ 
uint8_t SPI_RW(uint8_t data) 
{
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);//��SPI���ͻ������ǿ�ʱ�ȴ�  
	SPI_I2S_SendData(SPI1, data);//ͨ��SPI���߷���һ�ֽ�����
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);//��SPI���ջ�����Ϊ��ʱ�ȴ�
	return SPI_I2S_ReceiveData(SPI1);
}

/******************************************************************************
����ԭ�ͣ�	uint8_t NRF_Write_Reg(uint8_t reg, uint8_t value)
��    �ܣ�	NRFд�Ĵ���
�� �� ֵ��	NRFд�Ĵ�������ֵ
*******************************************************************************/
uint8_t NRF_Write_Reg(uint8_t reg, uint8_t value)
{
	uint8_t status;
	NRF_CSN_L;		//ѡͨNRF����
	status = SPI_RW(reg);//д�Ĵ�����ַ
	SPI_RW(value);	//д����
	NRF_CSN_H;		//��ֹNRF����
	return 	status;
}

/******************************************************************************
����ԭ�ͣ�	uint8_t NRF_Read_Reg(uint8_t reg)
��    �ܣ�	NRF���Ĵ���
�� �� ֵ��	�Ĵ�������
*******************************************************************************/
uint8_t NRF_Read_Reg(uint8_t reg)
{
	uint8_t reg_val;
	NRF_CSN_L;		//ѡͨNRF���� 
	SPI_RW(reg);	//д�Ĵ�����ַ
	reg_val = SPI_RW(0);//��ȡ�üĴ�����������
	NRF_CSN_H;		//��ֹNRF���� 
    return 	reg_val;
}

/******************************************************************************
����ԭ�ͣ�	uint8_t NRF_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t uchars)
��    �ܣ�	NRFд������
�� �� ֵ��	NRFд����������ֵ
*******************************************************************************/
uint8_t NRF_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t uchars)
{
	uint8_t i;
	uint8_t status;
	NRF_CSN_L;	//ѡͨNRF���� 
	status = SPI_RW(reg);//д�Ĵ�����ַ 
	for(i=0; i<uchars; i++)
	{
		SPI_RW(pBuf[i]);//д���� 
	}
	NRF_CSN_H;	//��ֹNRF����
    return 	status;	
}

/******************************************************************************
����ԭ�ͣ�	uint8_t NRF_Read_Buff(uint8_t reg, uint8_t *pBuf, uint8_t uchars)
��    �ܣ�	NRF��������
�� �� ֵ��	����������
*******************************************************************************/
uint8_t NRF_Read_Buff(uint8_t reg, uint8_t *pBuf, uint8_t uchars)
{
	uint8_t i;
	uint8_t status;
	NRF_CSN_L;	//ѡͨNRF���� 
	status = SPI_RW(reg);//д�Ĵ�����ַ
	for(i=0; i<uchars; i++)
	{
		pBuf[i] = SPI_RW(0);//��ȡ��������	
	}
	NRF_CSN_H;	//��ֹNRF����
    return 	status;
}

/******************************************************************************
����ԭ�ͣ�	static void NRF24L01_Set_TX(void)
��    �ܣ�	��NRF24L01����Ϊ����ģʽ
*******************************************************************************/
static void NRF24L01_Set_TX(void)
{
	NRF_CE_L;
	NRF_Write_Reg(NRF_WRITE_REG + CONFIG,0x0E);//����
	NRF_CE_H;
}

/******************************************************************************
����ԭ�ͣ�	static void NRF24L01_Set_RX(void)
��    �ܣ�	��NRF24L01����Ϊ����ģʽ
*******************************************************************************/
void NRF24L01_Set_RX(void)
{
	NRF_CE_L;
	NRF_Write_Reg(NRF_WRITE_REG + CONFIG,0x0F);//����
	NRF_CE_H;
}

/******************************************************************************
����ԭ�ͣ�	void NRF_Send_TX(uint8_t * tx_buf, uint8_t len)
��    �ܣ�	NRF2401�������ݰ�
*******************************************************************************/
void NRF_Send_TX(uint8_t * tx_buf, uint8_t len)
{
	NRF24L01_Set_TX();
	NRF_CE_L;//�������ģʽ1	
	NRF_Write_Buf(WR_TX_PLOAD, tx_buf, len);//װ������
	NRF_CE_H;//����CEΪ�ߣ��������䡣CE�ߵ�ƽ����ʱ����СΪ10us
}

/******************************************************************************
����ԭ�ͣ�	void NRF24L01_Check(void)
��    �ܣ�	���NRF�����Ƿ�����
*******************************************************************************/
void NRF24L01_Check(void)
{
	uint8_t buf[5]; 
	uint8_t i; 
	//д��5���ֽڵĵ�ַ 
	NRF_Write_Buf(NRF_WRITE_REG+TX_ADDR,TX_ADDRESS,5); 
	//����д��ĵ�ַ 
	NRF_Read_Buff(TX_ADDR,buf,5); 
	//�Ƚ�
	for(i=0;i<5;i++) 
	{ 
		if(buf[i]!=TX_ADDRESS[i]) 
			break; 
	} 
	if(i==5)
	{
		printf("nrf ok!");
	}
	else
	{
		printf("nrf fail!");
		while(1)
		{
			LED1_TOGGLE();
			delay_ms(200);
		}
	}
}

/******************************************************************************
����ԭ�ͣ�	void NRF24L01_Init(uint8_t Chanal,uint8_t Mode)
��    �ܣ�	NRF24L01��ʼ��
��    ����	Chanal��RFͨ��  Mode:ģʽ RX���� TX����
*******************************************************************************/
void NRF24L01_Init(uint8_t Chanal,uint8_t Mode,uint8_t datalength)
{
	NRF_CE_L;
		
	NRF_Write_Reg(FLUSH_TX,0xff);//��շ��ͻ�����
	NRF_Write_Reg(FLUSH_RX,0xff);//��ս��ջ�����
	NRF_Write_Buf(NRF_WRITE_REG + TX_ADDR,   TX_ADDRESS,5); //дTX�ڵ��ַ  	
	NRF_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0,RX_ADDRESS,5);	//дRX�ڵ��ַ 

	NRF_Write_Reg(NRF_WRITE_REG + EN_AA,     0x01); //ʹ��ͨ��0���Զ�Ӧ�� 
	NRF_Write_Reg(NRF_WRITE_REG + EN_RXADDR, 0x01);	//ʹ��ͨ��0�Ľ��յ�ַ 
	NRF_Write_Reg(NRF_WRITE_REG + SETUP_RETR,0x1a);	//�����Զ��ط����ʱ��:500us;����Զ��ط�����:10�� 
	NRF_Write_Reg(NRF_WRITE_REG + RF_CH,   Chanal);	//����RFͨ��ΪCHANAL
	NRF_Write_Reg(NRF_WRITE_REG + RX_PW_P0,  datalength);	//����ͨ��0����Ч���ݿ��
	NRF_Write_Reg(NRF_WRITE_REG + RF_SETUP,  0x0f); //����TX�������,0db����,2Mbps,���������濪��
	
	if(Mode==TX)
		NRF_Write_Reg(NRF_WRITE_REG + CONFIG,0x0E);//����
	else if(Mode==RX)
		NRF_Write_Reg(NRF_WRITE_REG + CONFIG,0x0F);//����
	
	NRF_CE_H;
}

/******************************************************************************
����ԭ�ͣ�	static void NRF24L01_Analyse(void)
��    �ܣ�	����NRF24L01�յ�������֡
*******************************************************************************/
void NRF24L01_Analyse(void)
{
	
	if(NRF24L01_RXDATA[0]==0xAA || NRF24L01_RXDATA[17]==0XEE)
	{
		if(NRF24L01_RXDATA[1]==0XF1)//������
		{
//			Flag.nrf_error_cnt=0;//���Ź���������
//			Flag.nrf_flag=1;
//			Rcdata.pitch	  =(short)(((int16_t)NRF24L01_RXDATA[2])<<8|(int16_t)NRF24L01_RXDATA[3]);
////			Rcdata.pitch    =Rcdata.pitch/*/21.0f*/;
//			Rcdata.roll		  =(short)(((int16_t)NRF24L01_RXDATA[4])<<8|(int16_t)NRF24L01_RXDATA[5]);
////			Rcdata.roll     =Rcdata.roll/*/21.0f*/;
//			Rcdata.yaw		  =(short)(((int16_t)NRF24L01_RXDATA[6])<<8|(int16_t)NRF24L01_RXDATA[7]);
////			Rcdata.yaw      =Rcdata.yaw/*/32.0f*/;
//			Rcdata.throttle  =(short)(((int16_t)NRF24L01_RXDATA[8])<<8|(int16_t)NRF24L01_RXDATA[9]);
//			Rcdata.throttle  =Rcdata.throttle/200.0f;
//			
//			Rcdata.pitchtrim=NRF24L01_RXDATA[10];
//			Rcdata.rolltrim =NRF24L01_RXDATA[11];
//			
//			Rcdata.Jindou		  =NRF24L01_RXDATA[12];//���
//			Rcdata.Lock			  =NRF24L01_RXDATA[13];//�����ס��־
//			Rcdata.Selft_Test	=NRF24L01_RXDATA[14];//�������Լ��־
//			Rcdata.batvol		  =NRF24L01_RXDATA[15];//ң������ص�ѹ��־
//			Rcdata.Mode       =NRF24L01_RXDATA[16];
//// 			printf("NRF data get\r\n");
//			RCData_Check(&Rcdata,&Copter);
		}
		NRF24L01_RXDATA[0]=0;
		NRF24L01_RXDATA[17]=0;
//		printf("data get\r\n");
	}

}

///******************************************************************************
//����ԭ�ͣ�	void Nrf_Send_Data_Back(unsigned char *temp,unsigned char cnt)
//��    �ܣ�	��ң�ط�������
//*******************************************************************************/
//void Nrf_Send_Data_Back(unsigned char *temp,unsigned char cnt)
//{
//	unsigned char i=0,len=cnt;

//	*(temp+(i++))=0XAA;//֡ͷ
//	*(temp+(i++))=0XF1;//������
//	*(temp+(i++))='I';
//	*(temp+(i++))='A';
//	
//	*(temp+(i++))='M';  
//	*(temp+(i++))='O';
//	
//	*(temp+(i++))='K';
//	*(temp+(i++))='!';
//	
//	*(temp+(i++))='D';
//	*(temp+(i++))='A';

//	*(temp+(i++))='T';//
//	*(temp+(i++))='A';//
//	*(temp+(i++))='G';//
//	*(temp+(i++))='E';//
//	*(temp+(i++))='T';//

//	NRF_Send_TX(temp,len);
//}

/****************************************/
//�ⲿ�ж�
//
void EXTI9_5_IRQHandler(void)//Nrf2401�ж�
{	
	if(EXTI_GetITStatus(EXTI_Line5) != RESET) 
	{
		EXTI_ClearITPendingBit(EXTI_Line5);
		NRF24L01_IRQ();
	}
}
/******************************************************************************
����ԭ�ͣ�	void NRF24L01_IRQ(void)
��    �ܣ�	NRF24L01�ж�
*******************************************************************************/
void NRF24L01_IRQ(void)	
{
	uint8_t status = NRF_Read_Reg(NRF_READ_REG + NRFRegSTATUS);//��ȡ״̬�Ĵ�������
	
	if(status & (1<<RX_DR))//�����ж�
	{
		uint8_t rx_len = NRF_Read_Reg(R_RX_PL_WID);//��ȡ��Ч���ݳ���
		if(rx_len==NRF_DATA_Length)
		{
			NRF_Read_Buff(RD_RX_PLOAD,NRF24L01_RXDATA,rx_len);//��ȡ����FIFO���� ��������NRF24L01_RXDATA
// 			printf("Data get\r\n");
			Nrf_Erro = 0;//ʧ�ش����Ź���������
		}
		else
		{
			NRF_Write_Reg(FLUSH_RX,0xff);//��ս��ջ�����
		}
	}
	if(status & (1<<MAX_RT))//�ﵽ�����ط��ж�
	{
		if(status & (1<<TX_FULL))//TX FIFO ���
		{
			NRF_Write_Reg(FLUSH_TX,0xff);//��շ��ͻ�����
		}
	}
//	if(status&(1<<TX_DS))//�������
//	{	
// 		NRF24L01_Set_RX();//����Nrf2401Ϊ����ģʽ
//	}
	NRF_Write_Reg(NRF_WRITE_REG + NRFRegSTATUS, status);//����жϱ�־λ	
}

/******************************************************************************
����ԭ�ͣ�	void Nrf_Connect(void)
��    �ܣ�	NRF24L01���Ӻ���
*******************************************************************************/
void Nrf_Connect(void)//1KHZ
{
	Nrf_Erro ++;//ʧ�ش����Ź������ۼ�
// 	if(Nrf_Erro==1)
// 	{
		NRF24L01_Analyse();//����NRF24L01�յ�������֡
// 		Nrf_Send_Data_Back(NRF24L01_TXDATA,NRF_DATA_Length);
// 	}
		NRF24L01_IRQ();    //����жϱ�־λ
//	if(Nrf_Erro%100==0)  //0.1sδ����nrf���� ����ͼ����ң��
//	{	
//// 		NRF24L01_IRQ();    //����жϱ�־λ
//	}
//	if(Nrf_Erro>=1000)   //1sδ����nrf���� ,�������ᣬ���ͣת����ֹ���ⷢ��
//	{	

//	}
}
