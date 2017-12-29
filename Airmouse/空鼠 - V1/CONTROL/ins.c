#include "ins.h"
#include "IMU.h"
#include "mpu6050.h"

#define myabs(x) ((x)>0? (x):(-(x)))

static float Rot_matrix[3][3]={0};
INS_TYPEDEF INS_Estimate={0};
FILTERTYPDEF AccZFilter,AccYFilter,AccXFilter;

/*
*只限平飞+-10°
*INS_Update
*传入参数:ax ay az加速度 m/s^2
*         q0 q1 q2 q3当前已经规范化的四元数
*return@ position vel accnog
*/
void INS_Update(float dt)
{
	float *Q;
	float ax,ay,az;
	Q = QuaterionReturn();
	ax = acc.X;
	ay = acc.Y;
	az = acc.Z;
//	AcclerationReturn(&ax,&ay,&az);//m/s^2
//Rot_matrix
	Rot_matrix[0][0] = (*(Q+0))*(*(Q+0)) + (*(Q+1))*(*(Q+1)) - (*(Q+2))*(*(Q+2)) - (*(Q+3))*(*(Q+3));// 11
	Rot_matrix[0][1] = 2.0f*((*(Q+1))*(*(Q+2)) + (*(Q+0))*(*(Q+3)));	// 12
	Rot_matrix[0][2] = 2.0f*((*(Q+1))*(*(Q+3)) - (*(Q+0))*(*(Q+2)));	// 13
	Rot_matrix[1][0] = 2.0f*((*(Q+1))*(*(Q+2)) - (*(Q+0))*(*(Q+3)));	// 21
	Rot_matrix[1][1] = (*(Q+0))*(*(Q+0)) - (*(Q+1))*(*(Q+1)) + (*(Q+2))*(*(Q+2)) - (*(Q+3))*(*(Q+3));// 22
	Rot_matrix[1][2] = 2.0f*((*(Q+2))*(*(Q+3)) + (*(Q+0))*(*(Q+1)));	// 23
	Rot_matrix[2][0] = 2.0f*((*(Q+1))*(*(Q+3)) + (*(Q+0))*(*(Q+2)));	// 31
	Rot_matrix[2][1] = 2.0f*((*(Q+2))*(*(Q+3)) - (*(Q+0))*(*(Q+1)));	// 32
	Rot_matrix[2][2] = (*(Q+0))*(*(Q+0)) - (*(Q+1))*(*(Q+1)) - (*(Q+2))*(*(Q+2)) + (*(Q+3))*(*(Q+3));// 33

	INS_Estimate.Z.accraw = ax * Rot_matrix[0][2] + ay * Rot_matrix[1][2] + az * Rot_matrix[2][2];
	INS_Estimate.Y.accraw = ax * Rot_matrix[0][1] + ay * Rot_matrix[1][1] + az * Rot_matrix[2][1];
	INS_Estimate.X.accraw = ax * Rot_matrix[0][0] + ay * Rot_matrix[1][0] + az * Rot_matrix[2][0];
	
	INS_Estimate.Z.accnog = MoveFilter(INS_Estimate.Z.accraw,&AccZFilter);//FILTER ~
	INS_Estimate.Y.accnog = MoveFilter(INS_Estimate.Y.accraw,&AccYFilter);
	INS_Estimate.X.accnog = MoveFilter(INS_Estimate.X.accraw,&AccXFilter);
	
	if(!INS_Estimate.calflag)
	{
		INS_Estimate.Z.accnogSum += INS_Estimate.Z.accnog;
		INS_Estimate.Y.accnogSum += INS_Estimate.Y.accnog;
		INS_Estimate.X.accnogSum += INS_Estimate.X.accnog;
		if(++INS_Estimate.runcnt>=300)
		{
			INS_Estimate.Z.accnogoffset = INS_Estimate.Z.accnogSum/300;
			INS_Estimate.Y.accnogoffset = INS_Estimate.Y.accnogSum/300;
			INS_Estimate.X.accnogoffset = INS_Estimate.X.accnogSum/300;
			INS_Estimate.calflag = 1;
		}
		else
		{
			INS_Estimate.Z.accnogoffset = INS_Estimate.Z.accnog;
			INS_Estimate.Y.accnogoffset = INS_Estimate.Y.accnog;
			INS_Estimate.X.accnogoffset = INS_Estimate.X.accnog;		
		}
	}

//	INS_Estimate.Z.accnog  = az-INS_Estimate.Z.accnog;
//	INS_Estimate.Y.accnog  = ay-INS_Estimate.Y.accnog;
//	INS_Estimate.X.accnog  = ax-INS_Estimate.X.accnog;
	
	INS_Estimate.Z.accnog  -= INS_Estimate.Z.accnogoffset;
	INS_Estimate.Y.accnog  = ay-INS_Estimate.Y.accnog;
	INS_Estimate.X.accnog  = ax-INS_Estimate.X.accnog;//方向向上为正
	
	if(-0.35f<INS_Estimate.Z.accnog && INS_Estimate.Z.accnog<0.30f)
	{
		INS_Estimate.Z.accnog = 0;
		INS_Estimate.Z.InsVel = 0;
	}
	else
	{
		INS_Estimate.Z.InsVel += INS_Estimate.Z.accnog * dt;	
	}
	
	if(-5 <Eulla.pitch && Eulla.pitch<=5)
	{
		if(-0.35f<INS_Estimate.Y.accnog && INS_Estimate.Y.accnog<0.35f)
		{
			INS_Estimate.Y.accnog = 0;
			INS_Estimate.Y.InsVel = 0;
		}
		else
		{
			INS_Estimate.Y.InsVel += INS_Estimate.Y.accnog * dt;
		}
	}
	else
	{
			INS_Estimate.Y.accnog = 0;
			INS_Estimate.Y.InsVel = 0;	
	}
	
	if(-5 <Eulla.roll && Eulla.roll<=5)
	{
		if(-0.35f<INS_Estimate.X.accnog && INS_Estimate.X.accnog<0.35f)
		{
			INS_Estimate.X.accnog = 0;
			INS_Estimate.X.InsVel = 0;
		}
		else
		{
			INS_Estimate.X.InsVel += INS_Estimate.X.accnog * dt;
		}
	}
	else
	{
			INS_Estimate.X.accnog = 0;
			INS_Estimate.X.InsVel = 0;
	}
	
//	if(INS_Estimate.Z.InsVel >=0)
//	{
//	   INS_Estimate.Z.VelSlope = INS_Estimate.Z.InsVel - INS_Estimate.Z.LastInsVel;//计算斜率
//	   INS_Estimate.Z.LastSlopeState = INS_Estimate.Z.SlopeState;//更新上一次斜率状态
//	   if(INS_Estimate.Z.VelSlope > INS_Estimate.Z.LastVelSlope)//fall
//	   {
//			INS_Estimate.Z.SlopeState = PFALL;//
//	   }
//	   else if(INS_Estimate.Z.VelSlope < INS_Estimate.Z.LastVelSlope)
//	   {
//			INS_Estimate.Z.SlopeState = PRISE;//rise
//	   }
//	   else
//	   {
//			INS_Estimate.Z.SlopeState = STABLE;//	   
//	   }
//	   INS_Estimate.Z.LastVelSlope = INS_Estimate.Z.VelSlope;
//	   INS_Estimate.Z.LastInsVel   = INS_Estimate.Z.InsVel;   
//	}
//	else
//	{
//	   INS_Estimate.Z.VelSlope = INS_Estimate.Z.InsVel - INS_Estimate.Z.LastInsVel;//计算斜率
//	   INS_Estimate.Z.LastSlopeState = INS_Estimate.Z.SlopeState;//更新上一次斜率状态
//	   if(INS_Estimate.Z.VelSlope > INS_Estimate.Z.LastVelSlope)//fall
//	   {
//			INS_Estimate.Z.SlopeState = NFALL;
//	   }
//	   else if(INS_Estimate.Z.VelSlope < INS_Estimate.Z.LastVelSlope)//
//	   {
//			INS_Estimate.Z.SlopeState = NRISE;	   
//	   }
//	   else
//	   {
//			INS_Estimate.Z.SlopeState = STABLE;	   
//	   }
//	   INS_Estimate.Z.LastVelSlope = INS_Estimate.Z.VelSlope;
//	   INS_Estimate.Z.LastInsVel   = INS_Estimate.Z.InsVel;
//	}

//	if(INS_Estimate.Z.CaptureFlag == NONE)
//	{
//		if(INS_Estimate.Z.LastSlopeState == STABLE && INS_Estimate.Z.SlopeState == PRISE)
//		{
////			INS_Estimate.Z.SlopeGradientState = STOPUP;
//			INS_Estimate.Z.CaptureFlag = HEAD;
//		}
//		else if(INS_Estimate.Z.LastSlopeState == NRISE && INS_Estimate.Z.SlopeState == PRISE)
//		{
//			INS_Estimate.Z.CaptureFlag = HEAD;		
//		}
////		else if(INS_Estimate.Z.LastSlopeState == PRISE && INS_Estimate.Z.SlopeState == PFALL)
////		{
////			INS_Estimate.Z.SlopeGradientState = UPSTOP;
//////			INS_Estimate.Z.BusyFlag = 1;		
////		}
//		else if(INS_Estimate.Z.LastSlopeState == STABLE && INS_Estimate.Z.SlopeState == NFALL)
//		{
////			INS_Estimate.Z.SlopeGradientState = STOPDOWN;
//			INS_Estimate.Z.CaptureFlag = TAIL;			
//		}
//		else if(INS_Estimate.Z.LastSlopeState == PFALL && INS_Estimate.Z.SlopeState == NFALL)
//		{
//			INS_Estimate.Z.CaptureFlag = TAIL;		
//		}
////		else if(INS_Estimate.Z.LastSlopeState == NFALL && INS_Estimate.Z.SlopeState == NRISE)
////		{
////			INS_Estimate.Z.SlopeGradientState = DOWNSTOP;
//////			INS_Estimate.Z.BusyFlag = 1;		
////		}

//	}
//	else
//	{
//		if(INS_Estimate.Z.CaptureFlag == HEAD)
//		{
//			if(INS_Estimate.Z.InsVel>0)
//			{
//				INS_Estimate.Z.counter++;
//				INS_Estimate.Z.InsPos += (INS_Estimate.Z.InsVel * dt);
//			}
//			else
//			{
//				if(--INS_Estimate.Z.counter<0)
//				{
//					INS_Estimate.Z.counter=0;
//					INS_Estimate.Z.CaptureFlag = NONE;
//				}
//			}
//		}
//		else if(INS_Estimate.Z.CaptureFlag == TAIL)
//		{
//			if(INS_Estimate.Z.InsVel<0)
//			{
//				INS_Estimate.Z.counter++;
//				INS_Estimate.Z.InsPos += (INS_Estimate.Z.InsVel * dt);
//			}
//			else
//			{
//				if(--INS_Estimate.Z.counter<0)
//				{
//					INS_Estimate.Z.counter=0;
//					INS_Estimate.Z.CaptureFlag = NONE;
//				}
//			}
//		}
//		
//	}
	
	INS_Estimate.Z.InsPos += (-INS_Estimate.Z.InsVel * dt);
	INS_Estimate.Y.InsPos += (INS_Estimate.Y.InsVel * dt);
	INS_Estimate.X.InsPos += (INS_Estimate.X.InsVel * dt);
	
}

float MoveFilter(float datain,FILTERTYPDEF *movefilter)
{
	movefilter->rawsum -= movefilter->rawbuffer[movefilter->rawindex];//去掉tail
	movefilter->rawbuffer[movefilter->rawindex] = datain;
	movefilter->rawsum += movefilter->rawbuffer[movefilter->rawindex];//队列head
	
	if(++movefilter->rawindex>=FILTERLEN)
	{
		movefilter->rawindex=0;
		movefilter->firstfullflag=1;
	}
	if(movefilter->firstfullflag)
	{
		return movefilter->rawsum/FILTERLEN;
	}
	else
	{
		return movefilter->rawbuffer[movefilter->rawindex-1];
	}	
}

