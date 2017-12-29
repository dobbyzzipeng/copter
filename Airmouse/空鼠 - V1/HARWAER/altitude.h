#ifndef _ALTITUDE_H_
#define _ALTITUDE_H_
#include "bmp180.h"
	
//#if USEMS5611
//	#include "ms5611.h"
//	#define Baro_Init()             Ms5611_init()
//	#define Baro_Calibration()      Ms5611_Calibration()
//	#define Baro_Update()           Ms5611_Update()
//#endif

//#if USEBMP180
//	#include "bmp180.h"
//	#define Baro_Init()             BMP180_INIT()
//	#define Baro_Calibration()      BMP180_Calibration()
//	#define Baro_Update()           BMP180_Update()
//#endif
#define LENGTH 10
#define HALENGTH 5
typedef struct
{
	float filter[LENGTH];
	float bias;
	unsigned char index;
	unsigned char flag;
}ALTFIFO;

typedef struct
{
	float altitude;
	float temperature;
	float pressure;
}ALTITUDE;

//extern ALTITUDE Altitude;
extern float ALTitude;
float getEstimatedAltitude(void);
#endif
