#ifndef Data_Tramsfer_h
#define  Data_Tramsfer_h
#include "stm32f10x.h"
#include "imu.h"

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

// void ANO_Euler_Data_Send(EULLA*eulla);
// void ANO_Mpu6050_Data_Send(Data_To_Imu*Acc,Data_To_Imu*Vec);
u8 ASII_To_Char(u8 *asii_code);
void Data_Receive_Anl(u8 *data_buf,uint16_t usart_flag);
// void ANO_Data_Send(float *pitch,float *roll,float *yaw);
#endif

