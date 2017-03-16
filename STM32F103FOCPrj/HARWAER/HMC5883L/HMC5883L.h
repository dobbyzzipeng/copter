#ifndef _HMC5883L_H
#define _HMC5883L_H
#include "myiic.h"
#include "delay.h"
#include "math.h"
#include "usart.h"

//#define HMC5883L_Address 0x1E //高七位地址
#define HMC5883L_SlaveAddr 0X3C //0011 1100 方向写
#define ConfigurationRegisterA 0x00//配置寄存器A地址
#define ConfigurationRegisterB 0x01//配置寄存器B地址
#define ModeRegister 0x02//模式寄存器地址
#define DataRegisterBegin 0x03//数据寄存器首地址0X03-0X08
#define HMC58X3_R_IDA 10
#define HMC5883_DEVICE_ID_A                     0x48
// HMC58X3 register map. For details see HMC58X3 datasheet
#define HMC58X3_R_CONFA 0
#define HMC58X3_R_CONFB 1
#define HMC58X3_R_MODE 2

//#define Measurement_Continuous 0x00//连续测量模式
//#define Measurement_SingleShot 0x01//单次测量模式
//#define Measurement_Idle 0x03//闲置模式

//the max and min data of the mag
typedef __packed struct
{
	signed short MaxMagX;
	signed short MaxMagY;
	signed short MaxMagZ;
	signed short MinMagX;
	signed short MinMagY;
	signed short MinMagZ;
}MagMaxMinData_t;
extern MagMaxMinData_t MagMaxMinData;

void HMC5883L_Init(void);//初始化
void Multiple_ReadCal_HMC5883L(unsigned char buf[]);
float HMC5883L_EstimatedYawGet(void);//数据转换
void HMC58X3_getlastValues(int16_t *x,int16_t *y,int16_t *z);
#endif
