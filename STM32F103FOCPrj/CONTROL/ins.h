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
  float VelSlope;//�ٶ�б��
  float LastVelSlope;
  signed char SlopeState;//�ٶ�б��״̬
  signed char LastSlopeState;
  signed char SlopeGradientState;//�ٶ�б�ʲ���״̬
  signed char LastSlopeGradientState;
  signed short CaptureFlag;//���β���״̬
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
   float rawbuffer[FILTERLEN];//buff ����
   unsigned short rawindex;//��������
   unsigned short firstfullflag;//
   double rawsum;
}FILTERTYPDEF;

extern INS_TYPEDEF INS_Estimate;

void INS_Update(float dt);
float MoveFilter(float datain,FILTERTYPDEF *movefilter);
#endif
