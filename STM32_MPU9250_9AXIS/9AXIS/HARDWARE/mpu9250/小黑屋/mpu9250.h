#ifndef _MPU9250_H
#define _MPU9250_H
#include "stm32f10x.h"

typedef struct
{
	int16_t X;
	int16_t Y;
	int16_t Z;
}S_INT16_XYZ;

extern uint8_t 	MPU9250_MAG_BUF[6];
extern S_INT16_XYZ GYRO_OFFSET;
extern S_INT16_XYZ ACC_OFFSET;		//零漂
extern S_INT16_XYZ MPU9250_ACC_LAST;
extern S_INT16_XYZ MPU9250_GYRO_LAST;
extern S_INT16_XYZ MPU9250_MAG_LAST;

//寄存器定义
#define SELF_TEST_X_GYRO		0X00
#define SELF_TEST_Y_GYRO		0X01
#define SELF_TEST_Z_GYRO		0X02

#define SELF_TEST_X_ACCEL		0X0D
#define SELF_TEST_Y_ACCEL		0X0E
#define SELF_TEST_Z_ACCEL		0X0F

#define XG_OFFSET_H					0X13
#define XG_OFFSET_L					0X14
#define YG_OFFSET_H					0X15
#define YG_OFFSET_L					0X16
#define ZG_OFFSET_H					0X17
#define ZG_OFFSET_L					0X18

#define SMPLRT_DIV					0X19 //陀螺仪采样率 典型值为0X07  1000/(1+7)=125HZ
#define CONFIG							0X1A //低通滤波器  典型值0x06 5hz
#define GYRO_CONFIG					0X1B //陀螺仪测量范围 0X18 正负2000度
#define ACCEL_CONFIG				0X1C //加速度计测量范围 0X18 正负16g
#define ACCEL_CONFIG2				0X1D //加速度计低通滤波器 0x06 5hz

#define LP_ACCEL_ODR				0X1E
#define WOM_THR							0X1F
#define FIFO_EN							0X23

#define ACCEL_XOUT_H				0X3B  //加速度计输出数据
#define ACCEL_XOUT_L				0X3C
#define ACCEL_YOUT_H				0X3D
#define ACCEL_YOUT_L				0X3E
#define ACCEL_ZOUT_H				0X3F
#define ACCEL_ZOUT_L				0X40

#define TEMP_OUT_H					0X41  //温度计输出数据
#define TEMP_OUT_L					0X42

#define GYRO_XOUT_H					0X43  //陀螺仪输出数据
#define GYRO_XOUT_L					0X44
#define GYRO_YOUT_H					0X45
#define GYRO_YOUT_L					0X46
#define GYRO_ZOUT_H					0X47
#define GYRO_ZOUT_L					0X48

#define MAG_XOUT_L          0X03
#define MAG_XOUT_H          0X04
#define MAG_YOUT_L          0X05
#define MAG_YOUT_H          0X06
#define MAG_ZOUT_L          0X07
#define MAG_ZOUT_H          0X08

#define PWR_MGMT_1					0X6B //电源管理1 典型值为0x00
#define PWR_MGMT_2					0X6C //电源管理2 典型值为0X00

#define WHO_AM_I						0X75 //器件ID MPU9250默认ID为0X71
#define WHO_AM_MAG					0X00 //器件ID MPU9250默认ID为0X71
#define USER_CTRL						0X6A //用户配置 当为0X10时使用SPI模式

#define MAG_I2C_SLV0_ADDR 	0x25
#define MAG_I2C_SLV0_REG		0x26
#define MAG_I2C_SLV0_CTRL		0x27
#define MAG_MODE_CTRL1      0X0A
#define MAG_I2C_MST_CTRL		0x24
#define MAG_I2C_MST_DELAY_CTRL 0x67
#define MAG_I2C_SLV0_DO 		0x63
#define MAG_INT_PIN_CFG 		0x37
#define MAG_EXT_SENS_DATA_00 0x49

#define MAG_I2C_ADDR        0x0C
#define MAG_WIA							0x00

// Read-only Reg
#define AK8963_WIA                  ((uint8_t)0x00)
#define AK8963_INFO                 ((uint8_t)0x01)
#define AK8963_ST1                  ((uint8_t)0x02)
#define AK8963_HXL                  ((uint8_t)0x03)
#define AK8963_HXH                  ((uint8_t)0x04)
#define AK8963_HYL                  ((uint8_t)0x05)
#define AK8963_HYH                  ((uint8_t)0x06)
#define AK8963_HZL                  ((uint8_t)0x07)
#define AK8963_HZH                  ((uint8_t)0x08)
#define AK8963_ST2                  ((uint8_t)0x09)
// Write/Read Reg
#define AK8963_CNTL1                ((uint8_t)0x0A)
#define AK8963_CNTL2                ((uint8_t)0x0B)
#define AK8963_ASTC                 ((uint8_t)0x0C)
#define AK8963_TS1                  ((uint8_t)0x0D)
#define AK8963_TS2                  ((uint8_t)0x0E)
#define AK8963_I2CDIS               ((uint8_t)0x0F)
// Read-only Reg ( ROM )
#define AK8963_ASAX                 ((uint8_t)0x10)
#define AK8963_ASAY                 ((uint8_t)0x11)
#define AK8963_ASAZ                 ((uint8_t)0x12)

#define MPU9250_CS			PAout(4) //MPU9250片选信号

extern void Clac_GYRO_OFFECT(void);
extern void MPU9250_IOAndSPI_Init(void);
extern void MPU9250_ReadValue(void);
extern uint8_t MPU9250_Write_Reg(uint8_t reg,uint8_t value);
extern uint8_t MPU9250_Read_Reg(uint8_t reg);
extern uint8_t MPU9250_Init(void);
extern uint8_t SPI1_ReadWriteByte(uint8_t TxData);//SPI总线读写一个字节
extern uint8_t MPU9250_Mag_Init(void);
extern void MPU9250_ReadMag(void);
extern void Init_AK8963(void);
#endif



