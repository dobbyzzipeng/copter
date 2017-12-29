#include "spi.h"
//#include "usart.h"

SPI_InitTypeDef  SPI_InitStructure;

void SPI1_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA|RCC_APB2Periph_SPI1, ENABLE );	
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin  =  SPI1_CSN_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SPI1_CSN_PORT, &GPIO_InitStructure);
	GPIO_SetBits(SPI1_CSN_PORT,SPI1_CSN_PIN);
	
 	GPIO_SetBits(GPIOA,GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //����SPI�������˫�������ģʽ:SPI����Ϊ˫��˫��ȫ˫��
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//����SPI����ģʽ:����Ϊ��SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;		//ѡ���˴���ʱ�ӵ���̬:ʱ�����ո�
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	//���ݲ����ڵڶ���ʱ����
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;		//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ256
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
	SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRCֵ����Ķ���ʽ
	SPI_Init(SPI1, &SPI_InitStructure);  //����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���
 
	SPI_Cmd(SPI1, ENABLE); //ʹ��SPI����
	
	SPI1_ReadWriteByte(0xff);//��������		 
}
/*
SPI �ٶ����ú���
SpeedSet:
SPI_BaudRatePrescaler_2   2��Ƶ   (SPI 36M@sys 72M)
SPI_BaudRatePrescaler_8   8��Ƶ   (SPI 9M@sys 72M)
SPI_BaudRatePrescaler_16  16��Ƶ  (SPI 4.5M@sys 72M)
SPI_BaudRatePrescaler_256 256��Ƶ (SPI 281.25K@sys 72M)
*/
//void SPI1_SetSpeed(u8 SpeedSet)
//{
//	SPI_InitStructure.SPI_BaudRatePrescaler = SpeedSet ;
//  	SPI_Init(SPI1, &SPI_InitStructure);
//	SPI_Cmd(SPI1,ENABLE);
//} 

//SPIx ��дһ���ֽ�
//TxData:Ҫд����ֽ�
//����ֵ:��ȡ�����ֽ�
u8 SPI1_ReadWriteByte(u8 TxData)
{		
	u8 retry=0;				 	
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) //���ָ����SPI��־λ�������:���ͻ���ձ�־λ
	{
	  retry++;
	  if(retry>200)return 0;
	}			  
	SPI_I2S_SendData(SPI1, TxData); //ͨ������SPIx����һ������
	retry=0;

	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET)//���ָ����SPI��־λ�������:���ܻ���ǿձ�־λ
	{
	  retry++;
	  if(retry>200)return 0;
	}	  						    
	return SPI_I2S_ReceiveData(SPI1); //����ͨ��SPIx������յ�����					    
}
/******************************************************/
void SPI1_WriteOneByte(uint8_t SlaveAddr, uint8_t WriteAddr,uint8_t DataToWrite)
{
	uint8_t status;
	SPI1_CS=0;
	status=SPI1_ReadWriteByte(WriteAddr); //����д����+�Ĵ�����
	SPI1_ReadWriteByte(DataToWrite);//д��Ĵ���ֵ
	SPI1_CS=1;
}
/*******************************************************/
uint8_t SPI1_ReadOneByte(u8 SlaveAddr ,u8 ReadAddr)
{
	uint8_t reg_val;
	SPI1_CS=0;
	SPI1_ReadWriteByte(ReadAddr|0x80); //���Ͷ�����+�Ĵ�����
	reg_val=SPI1_ReadWriteByte(0xff);//��ȡ�Ĵ���ֵ
	SPI1_CS=1;
	return(reg_val);
}
/*******************************************************/
void SPI1_ReadmultiyBytes(u8 SlaveAddr,u8 RegAddr,u8 Len,u8 *Buf_Addr)
{
	uint8_t i=0;
	SPI1_CS=0;
	SPI1_ReadWriteByte(RegAddr|0x80); //���Ͷ�����+�Ĵ�����
	for(i=0;i<Len;i++)
	{
	  *(Buf_Addr+i)=SPI1_ReadWriteByte(0xff);//��ȡ�Ĵ���ֵ
	}
	SPI1_CS=1;
}
///*****��spi2�ĳ�ʼ��**********************************/
void SPI2_Init(void)
{
 	GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef  SPI_InitStructure;

	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB, ENABLE );//PORTBʱ��ʹ�� 
	RCC_APB1PeriphClockCmd(	RCC_APB1Periph_SPI2,  ENABLE );//SPI2ʱ��ʹ�� 	
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //PB13/14/15����������� 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

 	GPIO_SetBits(GPIOB,GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);  //PB13/14/15����

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //����SPI�������˫�������ģʽ:SPI����Ϊ˫��˫��ȫ˫��
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//����SPI����ģʽ:����Ϊ��SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;		//ѡ���˴���ʱ�ӵ���̬:ʱ�����ո�
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	//���ݲ����ڵڶ���ʱ����
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;		//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ16
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
	SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRCֵ����Ķ���ʽ
	SPI_Init(SPI2, &SPI_InitStructure);  //����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���
 
	SPI_Cmd(SPI2, ENABLE); //ʹ��SPI����
	
	SPI2_ReadWriteByte(0xff);//��������	 

}   
//SPI �ٶ����ú��� SPI2����ٶ�Ϊ36Mhz
//SpeedSet:
//SPI_BaudRatePrescaler_2   2��Ƶ   
//SPI_BaudRatePrescaler_8   8��Ƶ   
//SPI_BaudRatePrescaler_16  16��Ƶ  
//SPI_BaudRatePrescaler_256 256��Ƶ 
  
//void SPI2_SetSpeed(u8 SpeedSet)
//{
//	SPI2->CR1&=0XFFC7; 
//	SPI2->CR1|=SpeedSet;
//	SPI_Cmd(SPI2,ENABLE); 
//} 
//void SPI2_SetSpeed(u8 SpeedSet)
//{
//	SPI_InitStructure.SPI_BaudRatePrescaler = SpeedSet ;
//    SPI_Init(SPI2, &SPI_InitStructure);
//	SPI_Cmd(SPI2,ENABLE);
//}
//SPIx ��дһ���ֽ�
//TxData:Ҫд����ֽ�
//����ֵ:��ȡ�����ֽ�
u8 SPI2_ReadWriteByte(u8 TxData)
{		
	u8 retry=0;				 	
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET) //���ָ����SPI��־λ�������:���ͻ���ձ�־λ
	{
	  retry++;
	  if(retry>200)return 0;
	}			  
	SPI_I2S_SendData(SPI2, TxData); //ͨ������SPIx����һ������
	retry=0;

	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET) //���ָ����SPI��־λ�������:���ܻ���ǿձ�־λ
	{
	  retry++;
	  if(retry>200)return 0;
	}	  						    
	return SPI_I2S_ReceiveData(SPI2); //����ͨ��SPIx������յ�����					    
}


