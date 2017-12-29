#ifndef _INS_H_
#define _INS_H_

typedef enum 
{
	NFALL=-2,
	PFALL=-1,
	STABLE=0,
	NRISE=1,
	PRISE=2,
}SLOPESTATE;

typedef enum
{
	STOPDOWN=-2,
	DOWNSTOP=-1,
	STOP=0,
	STOPUP=1,
	UPSTOP=2,
}SLOPEGRADIENTSTATE;

typedef enum
{
	TAIL=-1,
	NONE=0,
	HEAD=1,
}CAPTURESTATE;

typedef struct
{
  float InsPos;
  float InsVel;
  float LastInsVel;
  float VelSlope;//速度斜率
  float LastVelSlope;
  signed char SlopeState;//速度斜率状态
  signed char LastSlopeState;
  signed char SlopeGradientState;//速度斜率波形状态
  signed char LastSlopeGradientState;
  signed short CaptureFlag;//波形捕获状态
  signed short counter;
  float accraw;
  float accnog;
  float accnogoffset;
  double accnogSum;
}INERTIAL_TYPEDEF;

typedef struct
{
  INERTIAL_TYPEDEF X;
  INERTIAL_TYPEDEF Y;
  INERTIAL_TYPEDEF Z;
  unsigned short runcnt;
  unsigned short calflag;
}INS_TYPEDEF;

#define FILTERLEN 10
typedef struct
{
   float rawbuffer[FILTERLEN];//buff 缓冲
   unsigned short rawindex;//队列索引
   unsigned short firstfullflag;//
   double rawsum;
}FILTERTYPDEF;

extern INS_TYPEDEF INS_Estimate;

void INS_Update(float dt);
float MoveFilter(float datain,FILTERTYPDEF *movefilter);
#endif
