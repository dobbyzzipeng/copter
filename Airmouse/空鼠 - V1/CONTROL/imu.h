#ifndef _IMU_H_
#define _IMU_H_
#include "mpu6050.h"

typedef struct
{
	float pitch;
	float roll;
	float yaw;
	float pitchoffset;
	float rolloffset;
	float yawoffet;
	float thisyawangle;
	float lastyawangle;
	unsigned short runcnt;
	signed short turncnt;
}EULLA;

extern EULLA Eulla;//互补滤波后的欧拉角结构体
extern float Quat[4];//全局四元数

typedef struct
{
	float acclength;
	float magnitude;//大小 量级
	float acccorr;
	float accwithoutg;
}INERIAL;

extern INERIAL Inerialacc;

void imu_update(Data_To_Imu*velocity,Data_To_Imu*acc,EULLA*eulla,INERIAL*ineracc,float dt);
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az,float dt);
void AHRSReset(void);
void Quaternion_Init(float q[],EULLA eulla);
float invSqrt(float x);
void  AHRS_Update(float dt);
float Accwithoutg_Get(void);
float ACCangleCorrectFactorGet(void);
void RawEullaGetByAcc(Data_To_Imu*acc,EULLA*eulla);
float *QuaterionReturn(void);

#endif
