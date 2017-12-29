#include "ahrs.h"
#include "math.h"
#include "mpu92xx.h"

#define M_PI  ((float)3.1415926f)
#define RADTODEG ((float)57.2957805f)
// Fast inverse square-root
/**************************实现函数********************************************
*函数原型:	   float invSqrt(float x)
*功　　能:	   快速计算 1/Sqrt(x) 	
输入参数： 要计算的值
输出参数： 结果
*******************************************************************************/
float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

/**************************实现函数********************************************
*函数原型:	   void IMU_AHRSupdate
*功　　能:	 更新AHRS 更新四元数 
输入参数： 当前的测量值。
输出参数：没有
*******************************************************************************/
#define Kp 2.0f   // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.01f   // integral gain governs rate of convergence of gyroscope biases
#define halfT 0.001f//2ms period,half period is 1ms

volatile float exInt, eyInt, ezInt;  // 误差积分
volatile float q0 = 1.0f;
volatile float q1 = 0.0f;
volatile float q2 = 0.0f;
volatile float q3 = 0.0f;

/*static volatile*/ float q[4]; //　四元数
volatile float angle[3] = {0};
/*
*IMU_AHRSupdate 基于变形PI控制器的九轴融合互补滤波算法
*gx gy gz:三轴角速度，单位rad/s
*ax ay az:三轴加速度，单位无要求
*hx hy hz:三轴磁力计数据 单位无要求
*return:无
*更新全局四元数
*/
void IMU_AHRSupdate(float gx,float gy,float gz,float ax,float ay,float az,float mx,float my,float mz) 
{
    float norm;
    float hx, hy, hz, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez/*,halfT*/;
    float tempq0,tempq1,tempq2,tempq3;

    float q0q0 = q0*q0;
    float q0q1 = q0*q1;
    float q0q2 = q0*q2;
    float q0q3 = q0*q3;
    float q1q1 = q1*q1;
    float q1q2 = q1*q2;
    float q1q3 = q1*q3;
    float q2q2 = q2*q2;   
    float q2q3 = q2*q3;
    float q3q3 = q3*q3;   		

    //快速求平方根算法
    norm = invSqrt(ax*ax + ay*ay + az*az);       
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;
    //把加计的三维向量转成单位向量。
    norm = invSqrt(mx*mx + my*my + mz*mz);          
    mx = mx * norm;
    my = my * norm;
    mz = mz * norm; 
    // compute reference direction of flux
    hx = 2.0f*mx*(0.5f - q2q2 - q3q3) + 2.0f*my*(q1q2 - q0q3) + 2.0f*mz*(q1q3 + q0q2);
    hy = 2.0f*mx*(q1q2 + q0q3) + 2.0f*my*(0.5f - q1q1 - q3q3) + 2.0f*mz*(q2q3 - q0q1);
    hz = 2.0f*mx*(q1q3 - q0q2) + 2.0f*my*(q2q3 + q0q1) + 2.0f*mz*(0.5f - q1q1 - q2q2);         
    bx = sqrt((hx*hx) + (hy*hy));
    bz = hz; 
    // estimated direction of gravity and flux (v and w)
    vx = 2.0f*(q1q3 - q0q2);
    vy = 2.0f*(q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
    wx = 2.0f*bx*(0.5f - q2q2 - q3q3) + 2.0f*bz*(q1q3 - q0q2);
    wy = 2.0f*bx*(q1q2 - q0q3) + 2.0f*bz*(q0q1 + q2q3);
    wz = 2.0f*bx*(q0q2 + q1q3) + 2.0f*bz*(0.5f - q1q1 - q2q2);  
    // error is sum of cross product between reference direction of fields and direction measured by sensors
    ex = (ay*vz - az*vy) + (my*wz - mz*wy);
    ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
    ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

    if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
    {
        exInt = exInt + ex * Ki * halfT;
        eyInt = eyInt + ey * Ki * halfT;	
        ezInt = ezInt + ez * Ki * halfT;
        // 用叉积误差来做PI修正陀螺零偏
        gx = gx + Kp*ex + exInt;
        gy = gy + Kp*ey + eyInt;
        gz = gz + Kp*ez + ezInt;
    }
    // 四元数微分方程
    tempq0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    tempq1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    tempq2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    tempq3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  

    // 四元数规范化
    norm = invSqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
    q0 = tempq0 * norm;
    q1 = tempq1 * norm;
    q2 = tempq2 * norm;
    q3 = tempq3 * norm;

}

/**************************实现函数********************************************
*函数原型:	   void IMU_getQ(float * q)
*功　　能:	 更新四元数 返回当前的四元数组值
*输入参数： 将要存放四元数的数组首地址
*输出参数：没有
*******************************************************************************/
float GX=0,GY=0,GZ=0,AX=0,AY=0,AZ=0,MX=0,MY=0,MZ=0;
void IMU_getQ(volatile float * q) 
{
    MPU9250_DataSolve(&GX,&GY,&GZ,&AX,&AY,&AZ,&MX,&MY,&MZ);	 //获取原始数据,加速度计和磁力计是原始值，陀螺仪转换成了deg/s
//  IMU_AHRSupdate(GX,GY,GZ,AX,AY,AZ,MX,MY,MZ);
    IMU_AHRSupdate(GX,GY,GZ,AX,AY,AZ,MY,MX,MZ);//MY MX MZ排列与磁力计坐标有关
	                                           //MP9250磁力计坐标与陀螺仪加速度计坐标不一一对应
    q[0] = q0; //返回当前值
    q[1] = q1;
    q[2] = q2;
    q[3] = q3;
}

/**************************实现函数********************************************
*函数原型:	   void IMU_getYawPitchRoll(float * angles)
*功　　能:	 更新四元数 返回当前解算后的姿态数据
输入参数： 将要存放姿态角的数组首地址
输出参数：没有
*******************************************************************************/
void IMU_getYawPitchRoll(volatile float * angles) 
{
    IMU_getQ(q); //更新全局四元数
    //四元数转换成欧拉角，经过三角函数计算即可
    angles[2] = atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], -2 * q[2]*q[2] - 2 * q[3] * q[3] + 1) * RADTODEG; // yaw        -pi----pi
    angles[1] = asin(-2 * q[1] * q[3] + 2 * q[0] * q[2]) * RADTODEG; // pitch    -pi/2    --- pi/2 
    angles[0] = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2] * q[2] + 1) * RADTODEG; // roll       -pi-----pi  
}
/*
*返回四元数指针
*
*/
float *QuaterionReturn(void)
{
  return q;
}
/*
*返回加速度计数据指针
*
*/
void AcclerationReturn(float *ax,float *ay,float *az)
{
  *ax = AX;
  *ay = AY;
  *az = AZ;
}
