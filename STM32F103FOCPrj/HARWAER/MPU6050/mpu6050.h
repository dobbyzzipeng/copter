#ifndef _MPU6050_H
#define _MPU6050_H
#include "myiic.h"

//********MPU6050 Register Address************
#define	SMPLRT_DIV		0x19	//�����ǲ������ʷ�Ƶ�Ĵ���������ֵ:0x07(125Hz)
#define	CONFIGUATION	0x1A	//��ͨ�˲�Ƶ�ʣ�����ֵ:0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//�������Լ켰������Χ������ֵ:0x18(���Լ�,2000deg/s)
#define	ACCEL_CONFIG	0x1C	//���ٶ��Լ졢������Χ�Լ���ͨ�˲�Ƶ��:0x01(���Լ�,2G,5Hz)
#define I2C_MST_CTRL    0x24    //I2CƵ�ʣ�����ֵ��0x0D(400Hz)

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

#define	PWR_MGMT_1		0x6B	//��Դ��������ֵ:0x00(����ʹ��)
#define	WHO_AM_I		0x75	//IIC����λ��ַ�Ĵ���(Ĭ����ֵ0x68,ֻ��)
#define GYRO_RATE_1K    0X01
#define GYRO_RATE_8K    0x00
#define GYRO_SAMPRATE_	0X01
#define DLPF_CFG_				0X01
//****************************
//#define	MPU6050_Addr_W  0xD0	  //����������IIC�����еĴӵ�ַ,����д
//#define	MPU6050_Addr_R  0xD1	  //����������IIC�����еĴӵ�ַ,�����
#define	MPU6050_Addr      0xD0    //����������IIC�����еĴӵ�ַ,����д
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
//1ά�Ŀ������˲�
typedef struct {
    float x;  // ϵͳ��״̬��
    float A;  // x(n)=A*x(n-1)+u(n),u(n)~N(0,q)
    float H;  // z(n)=H*x(n)+w(n),w(n)~N(0,r)
    float q;  // Ԥ���������Э����
    float r;  // ������������Э����
    float p;  // �������Э����
    float gain;//����������
}kalman_struct;
/****************************************/
#define MOCEFILTERLEN 10
typedef struct
{
   int16_t rawbuffer[MOCEFILTERLEN];//buff ����
   uint8_t index;//��������
   uint8_t fullflag;//
   int32_t rawsum;
}MOVELPFFILTER;

int16_t MoveLpfFilter(int16_t datain,MOVELPFFILTER *movefilter);
/**********************************************/
extern Origial_DATA Sensor_Acc,Sensor_Gyro;//������ԭʼ����
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

