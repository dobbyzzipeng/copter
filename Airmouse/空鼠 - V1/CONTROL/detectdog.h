#ifndef __DETECTDOG_H__
#define __DETECTDOG_H__
#include "stdint.h"
#include "stm32f10x_tim.h"
/*
*    events detect dog
*    by zzipeng@2017/02/14
*/
#define DOWNTHRESHOLD  0
typedef enum
{
	DR16=0,
    MPU6050,
    HMC5883L,
	YAWGYRO,
	PITCHMOTOR,
	YAWMOTOR,
	MOTOR1,
	MOTOR2,
	MOTOR3,
	MOTOR4,
	TURNPLATEMOTOR,
}EVENTLIST;//����¼��б�
#define DETECTEVENTNUM 11//����¼���Ŀ
int32_t DETECTDOGBUF[DETECTEVENTNUM] = {2000,1000,1000,1500,2000,2000,\
										2000,2000,2000,2000};//����¼���ʱʱ�� ��λ:ms


void DetectEventInit(void);
void DetectDogRest(uint32_t index);
void DetectDogFeed(uint32_t index, uint32_t threshold);
uint8_t IsOverflowCheck(uint32_t index, uint32_t threshold);

#endif 
