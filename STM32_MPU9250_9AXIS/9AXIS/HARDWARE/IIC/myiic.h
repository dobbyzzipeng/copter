#ifndef __MYIIC_H
#define __MYIIC_H
#include "sys.h"
 
/**************************�ӿ�����*****************************/ 
#if 0
	#define _RCC_GPIOX_APB2PERIPH		RCC_APB2Periph_GPIOB
	#define _IIC_SCL_PIN 			    GPIO_Pin_6
	#define _IIC_SDA_PIN  				GPIO_Pin_7
	#define _IIC_PORT	  				GPIOB	

	#define _MPU6050_INT  				GPIO_Pin_3
	#define _MPU6050_PORT 				GPIOB	
	 //IO�������� 		 
	#define SDA_IN()  {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=0X80000000;}//��CRH12-15bit����
	#define SDA_OUT() {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=0X30000000;}
	 
	#define IIC_SCL    PBout(6) //SCL->PB6
	#define IIC_SDA    PBout(7) //SDA	->PB7
	#define READ_SDA   PBin(7)  //����SDA 
#else
	#define _RCC_GPIOX_APB2PERIPH		RCC_APB2Periph_GPIOA
	#define _IIC_SCL_PIN 			    GPIO_Pin_2
	#define _IIC_SDA_PIN  				GPIO_Pin_1
	#define _IIC_PORT	  				GPIOA	

	#define _MPU6050_INT  				GPIO_Pin_1
	#define _MPU6050_PORT 				GPIOB	
	 //IO�������� 		 
	#define SDA_IN()  {_IIC_PORT->CRL&=0XFFFFFF0F;_IIC_PORT->CRL|=0X00000080;}//
	#define SDA_OUT() {_IIC_PORT->CRL&=0XFFFFFF0F;_IIC_PORT->CRL|=0X00000030;}
	 
	#define IIC_SCL    PAout(2) //SCL->PB6
	#define IIC_SDA    PAout(1) //SDA	->PB7
	#define READ_SDA   PAin(1)  //����SDA 
#endif
/*****************************************************************/
//IIC���в�����������
void IIC_Init(void);                //��ʼ��IIC��IO��				 
void IIC_Start(void);				//����IIC��ʼ�ź�
void IIC_Stop(void);	  			//����IICֹͣ�ź�
void IIC_Send_Byte(uint8_t txd);			//IIC����һ���ֽ�
uint8_t  IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽڲ�����ack��ʾ�Ƿ����Ӧ���ź�
uint8_t  IIC_Wait_Ack(void); 			//IIC�ȴ�ACK�ź�
void IIC_Ack(void);					//IIC����ACK�ź�
void IIC_NAck(void);				//IIC������ACK�ź�

void MPU6050_Register_Configuration(void);
uint8_t IIC_ReadOneByte(uint8_t SlaveAddr ,uint8_t ReadAddr);							//ָ������ָ����ַ��ȡһ���ֽ�
void IIC_WriteOneByte(uint8_t SlaveAddr, uint8_t WriteAddr,uint8_t DataToWrite);		//ָ������ָ����ַд��һ���ֽ�
void IIC_ReadmultiyBytes(uint8_t SlaveAddr,uint8_t RegAddr,uint8_t Len,uint8_t *Buf_Addr);
/*void WriteLenByte(u16 WriteAddr,u32 DataToWrite,u8 Len);//ָ����ַ��ʼд��ָ�����ȵ�����
u32 ReadLenByte(u16 ReadAddr,u8 Len);					//ָ����ַ��ʼ��ȡָ����������
void Write(u16 WriteAddr,u8 *pBuffer,u16 NumToWrite);	//��ָ����ַ��ʼд��ָ�����ȵ�����
void Read(u16 ReadAddr,u8 *pBuffer,u16 NumToRead);   	//��ָ����ַ��ʼ����ָ�����ȵ�����
*/
#endif
















