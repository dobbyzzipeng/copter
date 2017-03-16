#include "IMU.h"
#include "delay.h"
#include "timer.h"
#include "FastMath.h"
#include "globaldefine.h"


//#define PRESION (0.0015f)//���� ��̬10minƯ��4��
//#define PRESION (0.0010f)//���� ��̬10minƯ��5��
#define PRESION (0.0008f)//���� ��̬10minƯ��15�� 16min 27
//#define PRESION (0.0005f)//���� ��̬10minƯ��20��

#define KP  (2.1f)//1.09f//2.0f
volatile float Kp = KP;//���ھ���ACC HMC���ݶ�GYRO�����Ĵ�С   �������ٶȼ�/�����������ٶ�              
volatile float Ki = 0.01f; //0.01 ��������������Ư��   

volatile static float exInt = 0.0f, eyInt = 0.0f, ezInt=0.0f;//���ֻ��ڱ���
float Quat[4]={1.0f,0.0f,0.0f,0.0f};//ȫ����Ԫ�� quaternion

EULLA Eulla;//�����˲����ŷ���ǽṹ��
INERIAL Inerialacc;
/**************����ŷ����****************/
//����:S.O.H. Mahony ���� On September 10th,2010
//����:MPU6050���ݻ����˲�����ŷ��������
//���룺���ٶ�(rad/s)�����ٶ�(m/s^2)��������������(��).
//�������Ԫ����ŷ���Ǹ���!
//���أ���
void imu_update(Data_To_Imu*velocity,Data_To_Imu*acc,EULLA*eulla,INERIAL*ineracc,float dt)
{
  float vx=0.0f, vy=0.0f, vz=0.0f,vw=0.0f;//��ǰ��ŷ����(��Ԫ��)�Ļ����������ϵ�ϣ����������������λ����
  float ex=0.0f, ey=0.0f, ez=0.0f;//�����ǻ��ֺ����̬�ͼ����������̬���
  float norm=0.0f;//ģ��������
  float halft = dt/2.0f;

/*************Ϊ�������ݼ���׼��*********/
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
	/******************************����Kp**************************************/
	ineracc->magnitude=(acc->X*acc->X + acc->Y*acc->Y + acc->Z*acc->Z)/(9.8f*9.8f);
	if(0.65f<ineracc->magnitude && ineracc->magnitude<1.21f)
	{
		Kp=KP;
	}
	else
	{
		Kp=0;//�����ü��ٶȼƲ������������ǻ�������ĽǶ�
	}
	/********************************************************************/
	norm=invSqrt(acc->X*acc->X + acc->Y*acc->Y + acc->Z*acc->Z);//
		
	vx = 2*(q1q3 - q0q2);	//�����ǻ��ֺ��������̬											
	vy = 2*(q0q1 + q2q3);//��ǰ��ŷ����(��Ԫ��)�Ļ����������ϵ�ϣ����������������λ����
	vz = -0.5f+q0q0 + q3q3;//�պ��Ƿ��������е����е�����Ԫ��
	
/****************************accwithoutg****************************************/
	  vw = q0q0 - q1q1 - q2q2 + q3q3 ;//�պ��Ƿ��������е����е�����Ԫ��
	  ineracc->accwithoutg=((vx*acc->X+vy*acc->Y+vw*acc->Z)-9.80f)*100;//cm/s^2
//	ineracc->accwithoutg = acc->Z - 9.8f * FastCos(eulla->pitch)*FastCos(eulla->roll);
	/********************************************************************/	
	  acc->X = acc->X * norm;
	  acc->Y = acc->Y * norm;//�ѼӼ���λ����ת��Ϊ��λ����
	  acc->Z = acc->Z * norm;//���ڻ�����������õ���������
		
	  ex = (acc->Y*vz - acc->Z*vy); //�����ǻ��ֺ��������̬�ͼӼƲ��������̬֮������                          					
	  ey = (acc->Z*vx - acc->X*vz);
	  ez = (acc->X*vy - acc->Y*vx);
	//����֮������������������(Ҳ��������������)����ʾ��ex,ey,ez����������������vxyz ��acc->x y z�Ĳ�ˡ�
	//�����������Ծ���λ�ڻ�������ϵ�ϵģ��������ǻ������Ҳ�ǻ��ڻ�������ϵ�����Ҳ�˵Ĵ�С�������ǻ�������
	//���ȣ������������������ǡ�
	/*********������********/		
	  exInt = exInt + ex * Ki;							 
	  eyInt = eyInt + ey * Ki;
	  ezInt = ezInt + ez * Ki;
/*********�ò���������PI������������ƫ********/
	  velocity->X = velocity->X + Kp*ex + exInt;//P��I����					   							
	  velocity->Y = velocity->Y + Kp*ey + eyInt;
	  velocity->Z = velocity->Z + Kp*ez + ezInt;//����Z��ת���޷�ͨ���ӼƲ��������Բ���Ҫ���ֻ���
	//�����gz����û�й۲��߽��н��������Ư�ƣ����ֳ����ľ��ǻ����������Լ�
 		   							
/********���������������Ԫ����halfTΪ�������ڵ�һ��**********/				   
	// integrate quaternion rate and normalise //��Ԫ�����·���
	Quat[0] = Quat[0] + (-Quat[1]*velocity->X - Quat[2]*velocity->Y - Quat[3]*velocity->Z)*halft;
	Quat[1] = Quat[1] + ( Quat[0]*velocity->X + Quat[2]*velocity->Z - Quat[3]*velocity->Y)*halft;
	Quat[2] = Quat[2] + ( Quat[0]*velocity->Y - Quat[1]*velocity->Z + Quat[3]*velocity->X)*halft;
	Quat[3] = Quat[3] + ( Quat[0]*velocity->Z + Quat[1]*velocity->Y - Quat[2]*velocity->X)*halft;

	norm = invSqrt(Quat[0]*Quat[0] + Quat[1]*Quat[1] + Quat[2]*Quat[2] + Quat[3]*Quat[3]);//�����󷽸�����
	Quat[0] = Quat[0] * norm;//������Ԫ��
	Quat[1] = Quat[1] * norm;
	Quat[2] = Quat[2] * norm;
	Quat[3] = Quat[3] * norm;
	
    eulla->thisyawangle += gz * dt * 57.3f*2.0f;
//	eulla->thisyawangle = FastAtan2(2 * Quat[1] * Quat[2] + 2 * Quat[0] * Quat[3], -2 * Quat[2]*Quat[2] - 2 * Quat[3]* Quat[3] + 1)* 57.3f; // �� z����ת
 	eulla->pitch = FastAsin(-2 * Quat[1] * Quat[3] + 2 * Quat[0] * Quat[2])* 57.3f; //�� y����ת -90��~+90��
 	eulla->roll  = FastAtan2(2 * Quat[2] * Quat[3] + 2 * Quat[0] * Quat[1], -2 * Quat[1] * Quat[1] - 2 * Quat[2]* Quat[2] + 1) * 57.3f; // �� x����ת -180��~+180��
	
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
//�����ݶ��½���̬�����㷨
//��̬5minYAWƮ10��
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az,float dt) 
{
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	//��Ԫ��΢�ַ��̣�������Ԫ��
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

		// Gradient decent algorithm corrective step//���㲽��
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
	
	Eulla.yaw   = FastAtan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3* q3 + 1)* 57.3f; // �� z����ת
 	Eulla.pitch = FastAsin(-2 * q1 * q3 + 2 * q0 * q2)* 57.3f; //�� y����ת -90��~+90��
 	Eulla.roll  = FastAtan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1) * 57.3f; // �� x����ת -180��~+180��

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
/**************************ʵ�ֺ���********************************************
*����ԭ��:	   float invSqrt(float x)
*��������:	   ���ټ��� 1/Sqrt(x) 	
��������� Ҫ�����ֵ
��������� ���
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
	imu_update(&VELOCITY,&ACC,&Eulla,&Inerialacc,dt);//��̬����
	#endif
	
	#if USE_Madgwick_ALGORITHM
    MadgwickAHRSupdateIMU(VELOCITY.X,VELOCITY.Y, VELOCITY.Z, ACC.X, ACC.Y, ACC.Z,dt);//�ݶ��½�
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
//ͨ�����ٶȼ������Լ����������������ʼ��roll pitch yaw��
void RawEullaGetByAcc(Data_To_Imu*acc,EULLA*eulla)
{
	eulla->roll=FastAtan2(acc->Y,acc->Z)*57.3f;
	eulla->pitch=-FastAsin(acc->X/9.8)*57.3f;
//	eulla->yaw=FastAtan2(hx,hy)*57.3f;//ͨ�����������
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
*��Ԫ����ʼ��
*���ݳ�ʼ����ŷ���ǳ�ʼ��
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
*������Ԫ��ָ��
***********/
float *QuaterionReturn(void)
{
	return Quat;
}
