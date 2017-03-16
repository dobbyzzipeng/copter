#ifndef __BMP180_H
#define __BMP180_H
#include "sys.h"
#include "stm32f10x.h"
/*************Calibration coefficients registers address*****************/
#define _AC1_M	0XAA
#define _AC1_L  0XAB
#define _AC2_M	0XAC
#define _AC2_L	0XAD
#define _AC3_M	0XAE
#define _AC3_L	0XAF
#define _AC4_M	0XB0
#define _AC4_L	0XB1
#define _AC5_M	0XB2
#define _AC5_L	0XB3
#define _AC6_M	0XB4
#define _AC6_L	0XB5
#define _B1_M	0XB6
#define _B1_L	0XB7
#define _B2_M	0XB8
#define _B2_L	0XB9
#define _MB_M	0XBA
#define _MB_L	0XBB
#define _MC_M	0XBC
#define _MC_L	0XBD
#define _MD_M	0XBE
#define _MD_L	0XBF
/***************MODE******************/
#define ULTRA_LOW_POWER_MODE			0X00
#define STANDARD_MODE					0X01
#define HIGHT_RESOLUTION_MODE			0X02
#define ULTRA_HIGHT_RESOLUTION_MODE		0X03
/***************temprerature******************/
#define _TEMPERATURE_COMMAND_			0X2E//读温度数据指令
#define _TEMPERATURE_ADDRESS_			0XF4//温度数据存放地址
#define _TEMPERATURE_DATAMSB_			0XF6//数据高八位地址
#define _TEMPERATURE_DATALSB_			0XF7//数据低八位地址
/****************PRESSURE*******************/
#define _PRESSURE_COMMAND_				0X34
#define _PRESSURE_ADDRESS_				0XF4
#define _PRESSURE_DATAMSB_				0XF6
#define _PRESSURE_DATALSB_				0XF7
#define _PRESSURE_DATAXLSB_				0XF8
/******************BMP180_W/R ADDRESS***********************/
#define BMP180_CHIP_ID					0XD0//VALUE 0X55
#define BMP180_SOLFT_RESET				0XE0
#define BMP180_SOLFT_RESET_DATA			0XB6
#define BMP180_W						0XEE
#define BMP180_R						0XEF
/****************/
#define oss	3//超高精度模式
#define pressure0	1013.25f//sea level pressure 1013.25hPa=101325Pa
/**************************接口配置*****************************/ 
#define _BMP180_RCC_GPIOX_APB2PERIPH		RCC_APB2Periph_GPIOB
#define _BMP180_IIC_SCL_PIN 			    GPIO_Pin_10
#define _BMP180_IIC_SDA_PIN  				GPIO_Pin_11
#define _BMP180_IIC_PORT	  				GPIOB	

 //IO方向设置 		 
 #define _BMP180_SDA_IN()  {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=0X00008000;}//将CRH12-15bit清零
 #define _BMP180_SDA_OUT() {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=0X00003000;}
 
 #define _BMP180_IIC_SCL    PBout(10) //SCL->PB10
 #define _BMP180_IIC_SDA    PBout(11) //SDA	->PB11
 #define _BMP180_READ_SDA   PBin(11)  //输入SDA 
 /*************************************/
 typedef struct
 {
	signed short AC1;
	signed short AC2;
	signed short AC3;
	unsigned short AC4;
	unsigned short AC5;
	unsigned short AC6;
	signed short  B1;
	signed short  B2;
	signed short  MB;
	signed short  MC;
	signed short  MD;
	 
	long		  X1;
	long		  X2;
	signed short  X3;
	signed short  B3;
	signed short  B4;
	long	      B5;
	signed short  B6;
	signed short  B7;
	 
 }BMP180_CALDATA;
 
 typedef struct
 {
		unsigned char barostate;
	  float deadline;
 }BARO;
 
 extern BARO Baro;
 extern float TEMPERATURE,PRESSURE;
/*****************************function******************************/
float BMP180_Update(void);
void BMP180_INIT(void);
void BMP180_Registers_Init(void);
void BMP180_Check(void);
void BMP180_CalData_Read(BMP180_CALDATA* bmp_caldata);
void BMP180_Calibration(void);
void BMP180_Temperature_Command(void);
long BMP180_Temperature_Read(void);
long BMP180_Temperature_Cal(BMP180_CALDATA* bmp_caldata,long ut);
void BMP180_Pressure_Command(void);
long BMP180_Pressure_Read(void);
long BMP180_Pressure_Cal(BMP180_CALDATA* bmp_caldata,long up);
float BMP180_ALT_GET(long p);
 //IIC所有操作函数声明
void _BMP180_IIC_Init(void);                //初始化IIC的IO口				 
void _BMP180_IIC_Start(void);								//发送IIC开始信号
void _BMP180_IIC_Stop(void);	  						//发送IIC停止信号
void _BMP180_IIC_Send_Byte(u8 txd);					//IIC发送一个字节
u8 	_BMP180_IIC_Read_Byte(unsigned char ack);//IIC读取一个字节并且用ack表示是否产生应答信号
//u8 IIC_Read_Byte_1(void);           //IIC读取一个字节
u8 _BMP180_IIC_Wait_Ack(void); 							//IIC等待ACK信号
void _BMP180_IIC_Ack(void);									//IIC发送ACK信号
void _BMP180_IIC_NAck(void);								//IIC不发送ACK信号

//void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
// u8 _BMP180_IIC_Read_One_Byte(u8 slavedaddr,u8 addr);	
//void MPU6050_Register_Configuration(void);
u8   _BMP180_ReadOneByte(u8 SlaveAddr ,u8 ReadAddr);							//指定器件指定地址读取一个字节
void _BMP180_WriteOneByte(u8 SlaveAddr, u8 WriteAddr,u8 DataToWrite);		//指定器件指定地址写入一个字节
void _BMP180_ReadmultiyBytes(u8 SlaveAddr,u8 RegAddr,u8 Len,u8 *Buf_Addr);
/*void WriteLenByte(u16 WriteAddr,u32 DataToWrite,u8 Len);//指定地址开始写入指定长度的数据
u32 ReadLenByte(u16 ReadAddr,u8 Len);					//指定地址开始读取指定长度数据
void Write(u16 WriteAddr,u8 *pBuffer,u16 NumToWrite);	//从指定地址开始写入指定长度的数据
void Read(u16 ReadAddr,u8 *pBuffer,u16 NumToRead);   	//从指定地址开始读出指定长度的数据
*/
#endif


