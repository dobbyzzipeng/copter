#ifndef _HMC5883L_H
#define _HMC5883L_H
#include "myiic.h"
#include "delay.h"
#include "math.h"
#include "usart.h"

//#define HMC5883L_Address 0x1E //����λ��ַ
#define HMC5883L_SlaveAddr 0X3C //0011 1100 ����д
#define ConfigurationRegisterA 0x00//���üĴ���A��ַ
#define ConfigurationRegisterB 0x01//���üĴ���B��ַ
#define ModeRegister 0x02//ģʽ�Ĵ�����ַ
#define DataRegisterBegin 0x03//���ݼĴ����׵�ַ0X03-0X08
#define HMC58X3_R_IDA 10
#define HMC5883_DEVICE_ID_A                     0x48
// HMC58X3 register map. For details see HMC58X3 datasheet
#define HMC58X3_R_CONFA 0
#define HMC58X3_R_CONFB 1
#define HMC58X3_R_MODE 2

//#define Measurement_Continuous 0x00//��������ģʽ
//#define Measurement_SingleShot 0x01//���β���ģʽ
//#define Measurement_Idle 0x03//����ģʽ

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

void HMC5883L_Init(void);//��ʼ��
void Multiple_ReadCal_HMC5883L(unsigned char buf[]);
float HMC5883L_EstimatedYawGet(void);//����ת��
void HMC58X3_getlastValues(int16_t *x,int16_t *y,int16_t *z);
#endif
