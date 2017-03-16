#ifndef _MPU6050_H
#define _MPU6050_H
#include "myiic.h"

//********MPU6050 Register Address************
#define	SMPLRT_DIV		0x19	//陀螺仪采样速率分频寄存器，典型值:0x07(125Hz)
#define	CONFIGUATION	0x1A	//低通滤波频率，典型值:0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//陀螺仪自检及测量范围，典型值:0x18(不自检,2000deg/s)
#define	ACCEL_CONFIG	0x1C	//加速度自检、测量范围以及高通滤波频率:0x01(不自检,2G,5Hz)
#define I2C_MST_CTRL    0x24    //I2C频率，典型值：0x0D(400Hz)

#define	ACCEL_XOUT_H	0x3B
// #define	ACCEL_XOUT_L	0x3C
// #define	ACCEL_YOUT_H	0x3D
// #define	ACCEL_YOUT_L	0x3E
// #define	ACCEL_ZOUT_H	0x3F
// #define	ACCEL_ZOUT_L	0x40
// //#define	TEMP_OUT_H		0x41
// //#define	TEMP_OUT_L		0x42

// #define	GYRO_XOUT_H		0x43
// #define	GYRO_XOUT_L		0x44	
// #define	GYRO_YOUT_H		0x45
// #define	GYRO_YOUT_L		0x46
// #define	GYRO_ZOUT_H		0x47
// #define	GYRO_ZOUT_L		0x48

#define	PWR_MGMT_1		0x6B	//电源管理，典型值:0x00(正常使用)
#define	WHO_AM_I		0x75	//IIC高七位地址寄存器(默认数值0x68,只读)
#define GYRO_RATE_1K    0X01
#define GYRO_RATE_8K    0x00
#define GYRO_SAMPRATE_	0X01
#define DLPF_CFG_				0X01
//****************************
//#define	MPU6050_Addr_W  0xD0	  //定义器件在IIC总线中的从地址,方向写
//#define	MPU6050_Addr_R  0xD1	  //定义器件在IIC总线中的从地址,方向读
#define	MPU6050_Addr      0xD0    //定义器件在IIC总线中的从地址,方向写
/*****declare of function*****/

//#define ACC_8G_SCALE  4096
#define ACC_4G_SCALE  8192

typedef struct
{
 	int16_t X;
 	int16_t Y;
 	int16_t Z;
}Origial_DATA;

typedef struct
{
	float X;
	float Y;
	float Z;
}Data_To_Imu;

typedef struct
{
 	int16_t AX_Offset;
 	int16_t AY_Offset;
 	int16_t AZ_Offset;
	
	int16_t GX_Offset;
	int16_t GY_Offset;
	int16_t GZ_Offset;
}OFFSET;

/***********************************/
//1维的卡尔曼滤波
typedef struct {
    float x;  // 系统的状态量
    float A;  // x(n)=A*x(n-1)+u(n),u(n)~N(0,q)
    float H;  // z(n)=H*x(n)+w(n),w(n)~N(0,r)
    float q;  // 预测过程噪声协方差
    float r;  // 测量过程噪声协方差
    float p;  // 估计误差协方差
    float gain;//卡尔曼增益
}kalman_struct;
/****************************************/
#define MOCEFILTERLEN 10
typedef struct
{
   int16_t rawbuffer[MOCEFILTERLEN];//buff 缓冲
   uint8_t index;//队列索引
   uint8_t fullflag;//
   int32_t rawsum;
}MOVELPFFILTER;

int16_t MoveLpfFilter(int16_t datain,MOVELPFFILTER *movefilter);
/**********************************************/
extern Origial_DATA Sensor_Acc,Sensor_Gyro;//传感器原始数据
extern Data_To_Imu  acc, velocity;
extern Data_To_Imu  ACC, VELOCITY;	
extern OFFSET OffSet;
extern kalman_struct KalmanfilterAccx,KalmanfilterAccy,KalmanfilterAccz;
/************************************************************************************************/
void  kalman_init(kalman_struct *kalman, float init_x, float init_p,float predict_q,float measure_r);
float kalman_filter(kalman_struct *kalman, float measure);
/********************************************/
void MPU6050_Init(void);
void MPU6050_Register_Configuration(void);
void MPU6050_CHECK(void);
/********************************************/
void MPU6050_Read_To_Use_(Origial_DATA*rawacc,Origial_DATA*rawgyro,OFFSET*offset);
void MPU6050_Read_To_Calculate(Origial_DATA *rawacc,Origial_DATA *rawgyro);
void MPU6050_Data_Check_(OFFSET *offset);
void MPU6050_Data_Read_Analys(Data_To_Imu *GYRO,Data_To_Imu *Acc,Origial_DATA *rawacc,Origial_DATA *rawgyro);
void MPU6050_Data_MoveLpffilter(Data_To_Imu *GYRO,Data_To_Imu *Acc,Origial_DATA *rawacc,Origial_DATA *rawgyro);
float Calculate_LpfFilteringCoefficient(float Time, float Cut_Off);
void ACC_LPF_Filter(Data_To_Imu *Accin,Data_To_Imu *Accout,float lpf_fator);
void MPU6050_Data_Exchange(Data_To_Imu*accin,Data_To_Imu*gyroin,Data_To_Imu*accout,Data_To_Imu*gyroout);
void MPU6050_Data_Read_Analys_Kalman(Data_To_Imu *GYRO,Data_To_Imu *Acc,Origial_DATA *rawacc,Origial_DATA *rawgyro);
float IIR_FILTER(float in[],float out[],float datain);
int16_t Windows_filter(int16_t *buffer,int16_t data,uint8_t *index,uint8_t size);
void Imu_data_Prepare(void);
void Mpu6050_IIR_Filter(Data_To_Imu *GYRO,Data_To_Imu *Acc,
												Origial_DATA*rawacc,Origial_DATA*rawgyro);
#endif /* _MPU6050_H_ */

