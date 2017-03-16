#ifndef _PID_REGULATOR_H_
#define _PID_REGULATOR_H_
#include "stdint.h"
typedef struct PID_Regulator_t
{
	float ref;
	float fdb;
	float err[2];
	float kp;
	float ki;
	float kd;
	float componentKp;//补偿
	float componentKi;
	float componentKd;
	float componentKpMax;//补偿阈值
	float componentKiMax;
	float componentKdMax;
	float output;
	float outputMax;
	float kp_offset;//默认值
	float ki_offset;
	float kd_offset;
	void (*Calc)(struct PID_Regulator_t *pid);//函数指针
	void (*Reset)(struct PID_Regulator_t *pid);
}PID_Regulator_t;
void PID_Reset(PID_Regulator_t *pid);
void PID_Calc(PID_Regulator_t *pid);
#endif
