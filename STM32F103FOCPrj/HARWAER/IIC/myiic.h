#ifndef __MYIIC_H
#define __MYIIC_H
#include "sys.h"
 
/**************************�ӿ�����*****************************/ 
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
/*****************************************************************/
//IIC���в�����������
void IIC_Init(void);                //��ʼ��IIC��IO��				 
void IIC_Start(void);				//����IIC��ʼ�ź�
void IIC_Stop(void);	  			//����IICֹͣ�ź�
void IIC_Send_Byte(u8 txd);			//IIC����һ���ֽ�
u8   IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽڲ�����ack��ʾ�Ƿ����Ӧ���ź�
u8   IIC_Wait_Ack(void); 			  //IIC�ȴ�ACK�ź�
void IIC_Ack(void);					  //IIC����ACK�ź�
void IIC_NAck(void);				  //IIC������ACK�ź�

void MPU6050_Register_Configuration(void);
u8 ReadOneByte(u8 SlaveAddr ,u8 ReadAddr);							//ָ������ָ����ַ��ȡһ���ֽ�
void WriteOneByte(u8 SlaveAddr, u8 WriteAddr,u8 DataToWrite);		//ָ������ָ����ַд��һ���ֽ�
void ReadmultiyBytes(u8 SlaveAddr,u8 RegAddr,u8 Len,u8 *Buf_Addr);
/*void WriteLenByte(u16 WriteAddr,u32 DataToWrite,u8 Len);//ָ����ַ��ʼд��ָ�����ȵ�����
u32 ReadLenByte(u16 ReadAddr,u8 Len);					//ָ����ַ��ʼ��ȡָ����������
void Write(u16 WriteAddr,u8 *pBuffer,u16 NumToWrite);	//��ָ����ַ��ʼд��ָ�����ȵ�����
void Read(u16 ReadAddr,u8 *pBuffer,u16 NumToRead);   	//��ָ����ַ��ʼ����ָ�����ȵ�����
*/
#endif


