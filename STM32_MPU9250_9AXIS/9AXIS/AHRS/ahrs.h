#ifndef _AHRS_H_
#define _AHRS_H_

extern volatile float angle[3];

float invSqrt(float x);
void IMU_AHRSupdate(float gx,float gy,float gz,float ax,float ay,float az,float mx,float my,float mz);
void IMU_getQ(volatile float * q);
void IMU_getYawPitchRoll(volatile float * angles);
//void IMU_getValues(float *imudata);
float *QuaterionReturn(void);
void AcclerationReturn(float *ax,float *ay,float *az);
#endif

