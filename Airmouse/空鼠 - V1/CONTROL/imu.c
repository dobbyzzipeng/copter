#include "IMU.h"
#include "delay.h"
#include "timer.h"
#include "FastMath.h"
#include "globaldefine.h"


//#define PRESION (0.0015f)//精度 静态10min漂移4度
//#define PRESION (0.0010f)//精度 静态10min漂移5度
#define PRESION (0.0008f)//精度 静态10min漂移15度 16min 27
//#define PRESION (0.0005f)//精度 静态10min漂移20度

#define KP  (2.1f)//1.09f//2.0f
volatile float Kp = KP;//用于决定ACC HMC数据对GYRO补偿的大小   决定加速度计/磁力计收敛速度              
volatile float Ki = 0.01f; //0.01 用于消除陀螺仪漂移   

volatile static float exInt = 0.0f, eyInt = 0.0f, ezInt=0.0f;//积分环节变量
float Quat[4]={1.0f,0.0f,0.0f,0.0f};//全局四元数 quaternion

EULLA Eulla;//互补滤波后的欧拉角结构体
INERIAL Inerialacc;
/**************更新欧拉角****************/
//引自:S.O.H. Mahony 代码 On September 10th,2010
//功能:MPU6050数据互补滤波更新欧拉角数据
//输入：角速度(rad/s)、加速度(m/s^2)、电子罗盘数据(°).
//输出：四元数、欧拉角更新!
//返回：无
void imu_update(Data_To_Imu*velocity,Data_To_Imu*acc,EULLA*eulla,INERIAL*ineracc,float dt)
{
  float vx=0.0f, vy=0.0f, vz=0.0f,vw=0.0f;//当前的欧拉角(四元数)的机体坐标参照系上，换算出来的重力单位向量
  float ex=0.0f, ey=0.0f, ez=0.0f;//陀螺仪积分后的姿态和计算出来的姿态误差
  float norm=0.0f;//模向量变量
  float halft = dt/2.0f;

/*************为后面数据计算准备*********/
	float q0q0 = Quat[0]*Quat[0];
	float q0q1 = Quat[0]*Quat[1];
	float q0q2 = Quat[0]*Quat[2];
	//float q0q3 = q[0]*q[3];	
	float q1q1 = Quat[1]*Quat[1];
	//float q1q2 = q[1]*q[2];	
	float q1q3 = Quat[1]*Quat[3];
	float q2q2 = Quat[2]*Quat[2];
	float q2q3 = Quat[2]*Quat[3];
	float q3q3 = Quat[3]*Quat[3];

	float gz = velocity->Z;
	
	if(acc->X*acc->Y*acc->Z==0) return;	

	if(myabs(gz)<=PRESION)
	{
		gz = 0;//dead zone
	}
	/******************************修正Kp**************************************/
	ineracc->magnitude=(acc->X*acc->X + acc->Y*acc->Y + acc->Z*acc->Z)/(9.8f*9.8f);
	if(0.65f<ineracc->magnitude && ineracc->magnitude<1.21f)
	{
		Kp=KP;
	}
	else
	{
		Kp=0;//不采用加速度计补偿矫正陀螺仪积分算出的角度
	}
	/********************************************************************/
	norm=invSqrt(acc->X*acc->X + acc->Y*acc->Y + acc->Z*acc->Z);//
		
	vx = 2*(q1q3 - q0q2);	//陀螺仪积分后推算的姿态											
	vy = 2*(q0q1 + q2q3);//当前的欧拉角(四元数)的机体坐标参照系上，换算出来的重力单位向量
	vz = -0.5f+q0q0 + q3q3;//刚好是方向余弦中第三行的三个元素
	
/****************************accwithoutg****************************************/
	  vw = q0q0 - q1q1 - q2q2 + q3q3 ;//刚好是方向余弦中第三行的三个元素
	  ineracc->accwithoutg=((vx*acc->X+vy*acc->Y+vw*acc->Z)-9.80f)*100;//cm/s^2
//	ineracc->accwithoutg = acc->Z - 9.8f * FastCos(eulla->pitch)*FastCos(eulla->roll);
	/********************************************************************/	
	  acc->X = acc->X * norm;
	  acc->Y = acc->Y * norm;//把加计三位向量转换为单位向量
	  acc->Z = acc->Z * norm;//基于机体坐标所测得的重力向量
		
	  ex = (acc->Y*vz - acc->Z*vy); //陀螺仪积分后的推算姿态和加计测量后的姿态之间的误差                          					
	  ey = (acc->Z*vx - acc->X*vz);
	  ez = (acc->X*vy - acc->Y*vx);
	//向量之间的误差，可以用向量叉积(也叫向量外积，叉乘)来表示，ex,ey,ez就是两个重力向量vxyz 和acc->x y z的叉乘。
	//这个叉积向量仍旧是位于机体坐标系上的，而陀螺仪积分误差也是基于机体坐标系，而且叉乘的大小与陀螺仪积分误差成
	//正比，正好拿来纠正陀螺仪。
	/*********误差积分********/		
	  exInt = exInt + ex * Ki;							 
	  eyInt = eyInt + ey * Ki;
	  ezInt = ezInt + ez * Ki;
/*********用叉积误差来做PI修正陀螺仪零偏********/
	  velocity->X = velocity->X + Kp*ex + exInt;//P、I控制					   							
	  velocity->Y = velocity->Y + Kp*ey + eyInt;
	  velocity->Z = velocity->Z + Kp*ez + ezInt;//由于Z轴转动无法通过加计测量，所以不需要积分环节
	//这里的gz由于没有观测者进行矫正会产生漂移，表现出来的就是积分自增或自减
 		   							
/********龙格库塔法更新四元数，halfT为采样周期的一半**********/				   
	// integrate quaternion rate and normalise //四元数更新方程
	Quat[0] = Quat[0] + (-Quat[1]*velocity->X - Quat[2]*velocity->Y - Quat[3]*velocity->Z)*halft;
	Quat[1] = Quat[1] + ( Quat[0]*velocity->X + Quat[2]*velocity->Z - Quat[3]*velocity->Y)*halft;
	Quat[2] = Quat[2] + ( Quat[0]*velocity->Y - Quat[1]*velocity->Z + Quat[3]*velocity->X)*halft;
	Quat[3] = Quat[3] + ( Quat[0]*velocity->Z + Quat[1]*velocity->Y - Quat[2]*velocity->X)*halft;

	norm = invSqrt(Quat[0]*Quat[0] + Quat[1]*Quat[1] + Quat[2]*Quat[2] + Quat[3]*Quat[3]);//快速求方根倒数
	Quat[0] = Quat[0] * norm;//更新四元数
	Quat[1] = Quat[1] * norm;
	Quat[2] = Quat[2] * norm;
	Quat[3] = Quat[3] * norm;
	
    eulla->thisyawangle += gz * dt * 57.3f*2.0f;
//	eulla->thisyawangle = FastAtan2(2 * Quat[1] * Quat[2] + 2 * Quat[0] * Quat[3], -2 * Quat[2]*Quat[2] - 2 * Quat[3]* Quat[3] + 1)* 57.3f; // 绕 z轴旋转
 	eulla->pitch = FastAsin(-2 * Quat[1] * Quat[3] + 2 * Quat[0] * Quat[2])* 57.3f; //绕 y轴旋转 -90°~+90°
 	eulla->roll  = FastAtan2(2 * Quat[2] * Quat[3] + 2 * Quat[0] * Quat[1], -2 * Quat[1] * Quat[1] - 2 * Quat[2]* Quat[2] + 1) * 57.3f; // 绕 x轴旋转 -180°~+180°
	
	if(++eulla->runcnt>2500)
	{
		eulla->runcnt=2500;

//		eulla->thisyawangle -=eulla->yawoffet;
		eulla->pitch -=eulla->pitchoffset;
		eulla->roll  -=eulla->rolloffset;		
	}
	else
	{
		eulla->yawoffet   =eulla->thisyawangle/*eulla->yaw*/;
		eulla->pitchoffset=eulla->pitch;
		eulla->rolloffset =eulla->roll;	
	}
	
//	if((eulla->thisyawangle-eulla->lastyawangle)>180.0f)
//	{
//		eulla->turncnt--;
//	}
//	else if((eulla->thisyawangle-eulla->lastyawangle)<-180.0f)
//	{
//		eulla->turncnt++;
//	}
//	eulla->lastyawangle  = eulla->thisyawangle;
//	eulla->yaw=eulla->thisyawangle+eulla->turncnt*360.0f;
	eulla->yaw = eulla->thisyawangle;
}

// Definitions
#define sampleFreq	1000.0f		// sample frequency in Hz
#define betaDef		0.1f		// 2 * proportional gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float beta = betaDef;	
volatile float q0=1,q1=0,q2=0,q3=0;
//---------------------------------------------------------------------------------------------------
// IMU algorithm update  by Madgwick
//六轴梯度下降姿态解算算法
//静态5minYAW飘10度
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az,float dt) 
{
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	//四元数微分方程，更新四元数
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) 
	{

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step//计算步长
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * dt/*(1.0f / sampleFreq)*/;
	q1 += qDot2 * dt/*(1.0f / sampleFreq)*/;
	q2 += qDot3 * dt/*(1.0f / sampleFreq)*/;
	q3 += qDot4 * dt/*(1.0f / sampleFreq)*/;

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
	
	Eulla.yaw   = FastAtan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3* q3 + 1)* 57.3f; // 绕 z轴旋转
 	Eulla.pitch = FastAsin(-2 * q1 * q3 + 2 * q0 * q2)* 57.3f; //绕 y轴旋转 -90°~+90°
 	Eulla.roll  = FastAtan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1) * 57.3f; // 绕 x轴旋转 -180°~+180°

	if(++Eulla.runcnt>2500)
	{
		Eulla.runcnt=2500;

//		eulla->thisyawangle -=eulla->yawoffet;
		Eulla.pitch -= Eulla.pitchoffset;
		Eulla.roll  -= Eulla.rolloffset;		
	}
	else
	{
		Eulla.yawoffet    = Eulla.thisyawangle/*eulla->yaw*/;
		Eulla.pitchoffset = Eulla.pitch;
		Eulla.rolloffset  = Eulla.roll;	
	}
}

// Fast inverse square-root
/**************************实现函数********************************************
*函数原型:	   float invSqrt(float x)
*功　　能:	   快速计算 1/Sqrt(x) 	
输入参数： 要计算的值
输出参数： 结果
*******************************************************************************/
float invSqrt(float x) 
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}


/**************************/
void AHRS_Update(float dt)
{
	#if USE_Mahony_ALGORITHM
	imu_update(&VELOCITY,&ACC,&Eulla,&Inerialacc,dt);//姿态解算
	#endif
	
	#if USE_Madgwick_ALGORITHM
    MadgwickAHRSupdateIMU(VELOCITY.X,VELOCITY.Y, VELOCITY.Z, ACC.X, ACC.Y, ACC.Z,dt);//梯度下降
	#endif
//	Inerialacc.accwithoutg=AcczWithoutG_Get(q,ACC);
}

float Accwithoutg_Get(void)
{
	return Inerialacc.accwithoutg;
}

float ACCangleCorrectFactorGet(void)
{
	return Inerialacc.acccorr*0.9;
}

/************************/
//通过加速度计数据以及磁力计数据算出初始的roll pitch yaw角
void RawEullaGetByAcc(Data_To_Imu*acc,EULLA*eulla)
{
	eulla->roll=FastAtan2(acc->Y,acc->Z)*57.3f;
	eulla->pitch=-FastAsin(acc->X/9.8)*57.3f;
//	eulla->yaw=FastAtan2(hx,hy)*57.3f;//通过磁力计算出
}
/*******
*void AHRSReset(void)
*
*****/
void AHRSReset(void)
{
	Quat[0]=1.0f;
	Quat[1]=Quat[2]=Quat[3]=0.0f;
	Eulla.pitch=0.0f;
	Eulla.roll=0.0f;
	Eulla.yaw=0.0f;
	Eulla.runcnt=0;
	Eulla.lastyawangle=0.0f;
	Eulla.thisyawangle=0.0f;
	Eulla.turncnt=0;
	Eulla.pitchoffset=0.0f;
	Eulla.rolloffset=0.0f;
	Eulla.yawoffet=0.0f;
}
/************
*Quaternion_Init
*四元数初始化
*根据初始化的欧拉角初始化
***********/
void Quaternion_Init(float q[],EULLA eulla)
{
	q[0]=FastCos(eulla.yaw/2)*FastCos(eulla.pitch/2)*FastCos(eulla.roll/2) + FastSin(eulla.yaw/2)*FastSin(eulla.pitch/2)*FastSin(eulla.roll/2);
	q[1]=FastCos(eulla.yaw/2)*FastCos(eulla.pitch/2)*FastSin(eulla.roll/2) - FastSin(eulla.yaw/2)*FastSin(eulla.pitch/2)*FastCos(eulla.roll/2);
	q[2]=FastCos(eulla.yaw/2)*FastSin(eulla.pitch/2)*FastCos(eulla.roll/2) + FastSin(eulla.yaw/2)*FastCos(eulla.pitch/2)*FastSin(eulla.roll/2);
	q[3]=FastSin(eulla.yaw/2)*FastCos(eulla.pitch/2)*FastCos(eulla.roll/2) - FastCos(eulla.yaw/2)*FastSin(eulla.pitch/2)*FastSin(eulla.roll/2);
}
/************
*float *QuaterionReturn
*返回四元数指针
***********/
float *QuaterionReturn(void)
{
	return Quat;
}
