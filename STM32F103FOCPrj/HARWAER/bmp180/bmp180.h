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
#define _TEMPERATURE_COMMAND_			0X2E//���¶�����ָ��
#define _TEMPERATURE_ADDRESS_			0XF4//�¶����ݴ�ŵ�ַ
#define _TEMPERATURE_DATAMSB_			0XF6//���ݸ߰�λ��ַ
#define _TEMPERATURE_DATALSB_			0XF7//���ݵͰ�λ��ַ
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
#define oss	3//���߾���ģʽ
#define pressure0	1013.25f//sea level pressure 1013.25hPa=101325Pa
/**************************�ӿ�����*****************************/ 
#define _BMP180_RCC_GPIOX_APB2PERIPH		RCC_APB2Periph_GPIOB
#define _BMP180_IIC_SCL_PIN 			    GPIO_Pin_10
#define _BMP180_IIC_SDA_PIN  				GPIO_Pin_11
#define _BMP180_IIC_PORT	  				GPIOB	

 //IO�������� 		 
 #define _BMP180_SDA_IN()  {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=0X00008000;}//��CRH12-15bit����
 #define _BMP180_SDA_OUT() {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=0X00003000;}
 
 #define _BMP180_IIC_SCL    PBout(10) //SCL->PB10
 #define _BMP180_IIC_SDA    PBout(11) //SDA	->PB11
 #define _BMP180_READ_SDA   PBin(11)  //����SDA 
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
 //IIC���в�����������
void _BMP180_IIC_Init(void);                //��ʼ��IIC��IO��				 
void _BMP180_IIC_Start(void);								//����IIC��ʼ�ź�
void _BMP180_IIC_Stop(void);	  						//����IICֹͣ�ź�
void _BMP180_IIC_Send_Byte(u8 txd);					//IIC����һ���ֽ�
u8 	_BMP180_IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽڲ�����ack��ʾ�Ƿ����Ӧ���ź�
//u8 IIC_Read_Byte_1(void);           //IIC��ȡһ���ֽ�
u8 _BMP180_IIC_Wait_Ack(void); 							//IIC�ȴ�ACK�ź�
void _BMP180_IIC_Ack(void);									//IIC����ACK�ź�
void _BMP180_IIC_NAck(void);								//IIC������ACK�ź�

//void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
// u8 _BMP180_IIC_Read_One_Byte(u8 slavedaddr,u8 addr);	
//void MPU6050_Register_Configuration(void);
u8   _BMP180_ReadOneByte(u8 SlaveAddr ,u8 ReadAddr);							//ָ������ָ����ַ��ȡһ���ֽ�
void _BMP180_WriteOneByte(u8 SlaveAddr, u8 WriteAddr,u8 DataToWrite);		//ָ������ָ����ַд��һ���ֽ�
void _BMP180_ReadmultiyBytes(u8 SlaveAddr,u8 RegAddr,u8 Len,u8 *Buf_Addr);
/*void WriteLenByte(u16 WriteAddr,u32 DataToWrite,u8 Len);//ָ����ַ��ʼд��ָ�����ȵ�����
u32 ReadLenByte(u16 ReadAddr,u8 Len);					//ָ����ַ��ʼ��ȡָ����������
void Write(u16 WriteAddr,u8 *pBuffer,u16 NumToWrite);	//��ָ����ַ��ʼд��ָ�����ȵ�����
void Read(u16 ReadAddr,u8 *pBuffer,u16 NumToRead);   	//��ָ����ַ��ʼ����ָ�����ȵ�����
*/
#endif


