#include "ins.h"
#include "ahrs.h"

static float *Quat;
static float ax,ay,az;
static float Rot_matrix[3][3]={0};
INS_TYPEDEF INS_Estimate;
/*
*INS_Update
*传入参数:ax ay az    加速度 m/s^2
*         q0 q1 q2 q3 全局四元数
*return@ position vel accnog
*/
void INS_Update(float dt)
{
	Quat = QuaterionReturn();
	AcclerationReturn(&ax,&ay,&az);//m/s^2
	//Rot_matrix方向余弦矩阵
	Rot_matrix[0][0] = (*(Quat+0))*(*(Quat+0)) + (*(Quat+1))*(*(Quat+1)) - (*(Quat+2))*(*(Quat+2)) - (*(Quat+3))*(*(Quat+3));// 11
	Rot_matrix[0][1] = 2.0f*((*(Quat+1))*(*(Quat+2)) + (*(Quat+0))*(*(Quat+3)));	// 12
	Rot_matrix[0][2] = 2.0f*((*(Quat+1))*(*(Quat+3)) - (*(Quat+0))*(*(Quat+2)));	// 13
	Rot_matrix[1][0] = 2.0f*((*(Quat+1))*(*(Quat+2)) - (*(Quat+0))*(*(Quat+3)));	// 21
	Rot_matrix[1][1] = (*(Quat+0))*(*(Quat+0)) - (*(Quat+1))*(*(Quat+1)) + (*(Quat+2))*(*(Quat+2)) - (*(Quat+3))*(*(Quat+3));// 22
	Rot_matrix[1][2] = 2.0f*((*(Quat+2))*(*(Quat+3)) + (*(Quat+0))*(*(Quat+1)));	// 23
	Rot_matrix[2][0] = 2.0f*((*(Quat+1))*(*(Quat+3)) + (*(Quat+0))*(*(Quat+2)));	// 31
	Rot_matrix[2][1] = 2.0f*((*(Quat+2))*(*(Quat+3)) - (*(Quat+0))*(*(Quat+1)));	// 32
	Rot_matrix[2][2] = (*(Quat+0))*(*(Quat+0)) - (*(Quat+1))*(*(Quat+1)) - (*(Quat+2))*(*(Quat+2)) + (*(Quat+3))*(*(Quat+3));// 33

	INS_Estimate.Z.accZnog = ax * Rot_matrix[0][2] + ay * Rot_matrix[1][2] + az * Rot_matrix[2][2];
	INS_Estimate.Y.accZnog = ax * Rot_matrix[0][1] + ay * Rot_matrix[1][1] + az * Rot_matrix[2][1];
	INS_Estimate.X.accZnog = ax * Rot_matrix[0][0] + ay * Rot_matrix[1][0] + az * Rot_matrix[2][0];
	
	if(!INS_Estimate.calflag)
	{
		INS_Estimate.Z.accZnogSum += INS_Estimate.Z.accZnog;
		INS_Estimate.Y.accZnogSum += INS_Estimate.Y.accZnog;
		INS_Estimate.X.accZnogSum += INS_Estimate.X.accZnog;
		if(++INS_Estimate.runcnt>=100)
		{
			INS_Estimate.Z.accZnogoffset = INS_Estimate.Z.accZnogSum/100;
			INS_Estimate.Y.accZnogoffset = INS_Estimate.Y.accZnogSum/100;
			INS_Estimate.X.accZnogoffset = INS_Estimate.X.accZnogSum/100;
			INS_Estimate.calflag = 1;
		}
	}

	INS_Estimate.Z.accZnog = INS_Estimate.Z.accZnog - INS_Estimate.Z.accZnogoffset;
	INS_Estimate.Y.accZnog = INS_Estimate.Y.accZnog - INS_Estimate.Y.accZnogoffset;
	INS_Estimate.X.accZnog = INS_Estimate.X.accZnog - INS_Estimate.X.accZnogoffset;
	
	if(-0.5f<INS_Estimate.Z.accZnog && INS_Estimate.Z.accZnog<0.5f)  INS_Estimate.Z.accZnog=0;
	if(-2.0f<INS_Estimate.Y.accZnog && INS_Estimate.Y.accZnog<2.0f)  INS_Estimate.Y.accZnog=0;
	if(-2.0f<INS_Estimate.X.accZnog && INS_Estimate.X.accZnog<2.0f)  INS_Estimate.X.accZnog=0;
	
	INS_Estimate.Z.InsVel += INS_Estimate.Z.accZnog * dt;
	INS_Estimate.Y.InsVel += INS_Estimate.Y.accZnog * dt;
	INS_Estimate.X.InsVel += INS_Estimate.X.accZnog * dt;
	
	INS_Estimate.Z.InsPos = INS_Estimate.Z.InsVel * dt;
	INS_Estimate.Y.InsPos = INS_Estimate.Y.InsVel * dt;
	INS_Estimate.X.InsPos = INS_Estimate.X.InsVel * dt;
	
}

