#ifndef __MYIIC_H
#define __MYIIC_H
#include "sys.h"
 
/**************************接口配置*****************************/ 
#define _RCC_GPIOX_APB2PERIPH		RCC_APB2Periph_GPIOB
#define _IIC_SCL_PIN 			    GPIO_Pin_6
#define _IIC_SDA_PIN  				GPIO_Pin_7
#define _IIC_PORT	  				GPIOB	

#define _MPU6050_INT  				GPIO_Pin_3
#define _MPU6050_PORT 				GPIOB	
 //IO方向设置 		 
 #define SDA_IN()  {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=0X80000000;}//将CRH12-15bit清零
 #define SDA_OUT() {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=0X30000000;}
 
 #define IIC_SCL    PBout(6) //SCL->PB6
 #define IIC_SDA    PBout(7) //SDA	->PB7
 #define READ_SDA   PBin(7)  //输入SDA 
/*****************************************************************/
//IIC所有操作函数声明
void IIC_Init(void);                //初始化IIC的IO口				 
void IIC_Start(void);				//发送IIC开始信号
void IIC_Stop(void);	  			//发送IIC停止信号
void IIC_Send_Byte(u8 txd);			//IIC发送一个字节
u8   IIC_Read_Byte(unsigned char ack);//IIC读取一个字节并且用ack表示是否产生应答信号
u8   IIC_Wait_Ack(void); 			  //IIC等待ACK信号
void IIC_Ack(void);					  //IIC发送ACK信号
void IIC_NAck(void);				  //IIC不发送ACK信号

void MPU6050_Register_Configuration(void);
u8 ReadOneByte(u8 SlaveAddr ,u8 ReadAddr);							//指定器件指定地址读取一个字节
void WriteOneByte(u8 SlaveAddr, u8 WriteAddr,u8 DataToWrite);		//指定器件指定地址写入一个字节
void ReadmultiyBytes(u8 SlaveAddr,u8 RegAddr,u8 Len,u8 *Buf_Addr);
/*void WriteLenByte(u16 WriteAddr,u32 DataToWrite,u8 Len);//指定地址开始写入指定长度的数据
u32 ReadLenByte(u16 ReadAddr,u8 Len);					//指定地址开始读取指定长度数据
void Write(u16 WriteAddr,u8 *pBuffer,u16 NumToWrite);	//从指定地址开始写入指定长度的数据
void Read(u16 ReadAddr,u8 *pBuffer,u16 NumToRead);   	//从指定地址开始读出指定长度的数据
*/
#endif


