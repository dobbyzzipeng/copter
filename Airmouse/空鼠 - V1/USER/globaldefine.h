#ifndef _GLOBALDEFINE_H_
#define _GLOBALDEFINE_H_

#define myabs(x) ((x>0)? (x) : (-x))
/*******************************************************************/
#define FLIGHT_MODE				0//1:6axis x mode  0:4axis xmode
#define USE_DMA_DBUG  			1//1:use  0:no use 
#define USE_NRF24L01_   		1//1:use  0:no use 
#define USE_BLUETOOTH   		0//1:use  0:no use 
#define USE_KALMAN      		0//1:use  0:no use 
#define USE_IIR             	1//1:use  0:no use 
#define USE_LPF            	 	0//1:use  0:no use 
#define INTEGERTOYAW        	1//
/*******************************Sensor*********************************/
#define IMU_TYPE				0//0:MPU6050
								//1:MPU6000
								//2:MPU9150
								//3:MPU9250
								//4:MPU6500
#define USE_Mahony_ALGORITHM     0//PI变形互补滤波
#define USE_Madgwick_ALGORITHM   1//梯度下降算法
#define USE_BARO			     1//0:no use
#define USEMS5611                0//
#define	USEBMP180                0//1
/********************************************************************/
#define USE_US100			     0//1:use  0:no use
#define USE_IRAVOID              0//1:use  0:no use
/*********************************************************************/
#define ACTUALY_FLY	    		 0//1:use  0:no use 
#define IMU_CHIP_ROTATION		 3//0：以MPU6050+X轴为机头前方
                                  //1：以MPU6050-X轴为机头前方
                                  //2：以MPU6050+Y轴为机头前方
                                  //3：以MPU6050-Y轴为机头前方
/*********************************************************************/														 
#define SYS_VOL_WARMING   840 //7.4/11/3.28*4096
#define BAT_VOL_WARMING   3085//4.99/2/3.28*4096 //电池电压阈值
#define NRF_ERR_TIME      1000//系统时钟定时时间为2ms的情况下，1000表示2秒 持续2秒丢失遥控信号
#define LED_FAST_BLINK	  100
#define LED_LOW_BLINK	  500
/******************************************************************************************/
#define RiseThrottle    0.8f
#define RelastThrottle  0.8f
#define IdleThrottle    0.1f
/******************************************************************************************/
enum
{
	USEMPU6050=0,
	USEMPU6000=1,
	USEMPU9150=2,
	USEMPU9250=3,
	USEMPU6500=4,
};

#define	QUAD_4AXIS_X 0//机型 x4
#define	QUAD_4AXIS_I 1//+4
#define	QUAD_6AXIS_X 2//X6
#define	QUAD_6AXIS_I 3//I6
#define	QUAD_6AXIS_Y 4//Y6
#define	QUAD_6AXIS_IY 5//IY6
#define	QUAD_8AXIS_I 6//I8
#define	QUAD_8AXIS_X 7//X8
#define	QUAD_8AXIS_V8 8//V8
#define	QUAD_8AXIS_I8 9//I+8
//enum
//{
//	QUAD_4AXIS_X=0,//机型 x
//	QUAD_4AXIS_I=1,//+
//	QUAD_6AXIS_X=2,//x6
//	QUAD_6AXIS_I=3,//i6
//	QUAD_6AXIS_Y=4,
//};

#define LOCK    1
#define	RELEASE 2
//enum motorflag
//{
//	LOCK=1,
//	RELEASE=2,
//};


#define applyDeadband(value, deadband)  \
  if(abs(value) < deadband) {           \
    value = 0;                          \
  } else if(value > 0){                 \
    value -= deadband;                  \
  } else if(value < 0){                 \
    value += deadband;                  \
  }
	
#endif
