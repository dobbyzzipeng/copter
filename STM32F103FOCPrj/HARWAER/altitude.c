#include "altitude.h"

ALTFIFO Altfifo;
//ALTITUDE Altitude;
float ALTitude=0.0f;
float getEstimatedAltitude(void)
{
	/*
	unsigned char i;
	float sum=0;
	Altfifo.filter[Altfifo.index]=BMP180_Update();
	if(++Altfifo.index==LENGTH)	
	{
		Altfifo.index=0;
	}

	for(i=0;i<LENGTH;i++)
	{
		sum+=Altfifo.filter[i];
	}
	return sum/LENGTH;
	*/
	
	static float sum=0;
	static unsigned char cnt=0;
		sum-=Altfifo.filter[Altfifo.index];
		Altfifo.filter[Altfifo.index]=BMP180_Update()-Altfifo.bias;
		sum+=Altfifo.filter[Altfifo.index];

	if(++Altfifo.index==LENGTH)	
	{
		Altfifo.index=0;
	}
	
	if(Altfifo.flag==0)
	{
		cnt++;
		if(cnt>100)
		{
			cnt=100;
			Altfifo.bias=sum/LENGTH;
			Altfifo.flag=1;
		}
		return 0;
	}
	else if(Altfifo.flag==1)
	{
				cnt++;
		if(cnt>110)
		{
			cnt=110;
	  	return sum/LENGTH;
		}
		return 0;
	}
	else return 0;
}

//u8 getEstimatedAltitude()
//{
//  int32_t  BaroAlt;
//  static float baroGroundTemperatureScale,logBaroGroundPressureSum;
//  static float vel = 0.0f;
//  static uint16_t previousT;
//  uint16_t currentT = microms();
//  uint16_t dTime;

//  dTime = currentT - previousT;
//  if (dTime < UPDATE_INTERVAL) return 0;
//  previousT = currentT;

//  if(calibratingB > 0) {
//    logBaroGroundPressureSum = log(baroPressureSum);
//    baroGroundTemperatureScale = (baroTemperature + 27315) *  29.271267f;
//    calibratingB--;
//  }

//  // baroGroundPressureSum is not supposed to be 0 here
//  // see: https://code.google.com/p/ardupilot-mega/source/browse/libraries/AP_Baro/AP_Baro.cpp
//  BaroAlt = ( logBaroGroundPressureSum - log(baroPressureSum) ) * baroGroundTemperatureScale;

//  alt.EstAlt = (alt.EstAlt * 6 + BaroAlt * 2) >> 3; // additional LPF to reduce baro noise (faster by 30 µs)

//  #if (defined(VARIOMETER) && (VARIOMETER != 2)) || !defined(SUPPRESS_BARO_ALTHOLD)
//    //P
//    int16_t error16 = constrain(AltHold - alt.EstAlt, -300, 300);
//    applyDeadband(error16, 10); //remove small P parametr to reduce noise near zero position
//    BaroPID = constrain((conf.pid[PIDALT].P8 * error16 >>7), -150, +150);

//    //I
//    errorAltitudeI += conf.pid[PIDALT].I8 * error16 >>6;
//    errorAltitudeI = constrain(errorAltitudeI,-30000,30000);
//    BaroPID += errorAltitudeI>>9; //I in range +/-60
// 
//    // projection of ACC vector to global Z, with 1G subtructed
//    // Math: accZ = A * G / |G| - 1G
//    int16_t accZ = (imu.accSmooth[ROLL] * EstG32.V.X + imu.accSmooth[PITCH] * EstG32.V.Y + imu.accSmooth[YAW] * EstG32.V.Z) * invG;

//    static int16_t accZoffset = 0;
//    if (!f.ARMED) {
//      accZoffset -= accZoffset>>3;
//      accZoffset += accZ;
//    }  
//    accZ -= accZoffset>>3;
//    applyDeadband(accZ, ACC_Z_DEADBAND);

//    static int32_t lastBaroAlt;
//    //int16_t baroVel = (alt.EstAlt - lastBaroAlt) * 1000000.0f / dTime;
//    int16_t baroVel = (alt.EstAlt - lastBaroAlt) * (1000000 / UPDATE_INTERVAL);
//    lastBaroAlt = alt.EstAlt;

//    baroVel = constrain(baroVel, -300, 300); // constrain baro velocity +/- 300cm/s
//    applyDeadband(baroVel, 10); // to reduce noise near zero

//    // Integrator - velocity, cm/sec
//    vel += accZ * ACC_VelScale * dTime;

//    // apply Complimentary Filter to keep the calculated velocity based on baro velocity (i.e. near real velocity). 
//    // By using CF it's possible to correct the drift of integrated accZ (velocity) without loosing the phase, i.e without delay
//    vel = vel * 0.985f + baroVel * 0.015f;

//    //D
//    alt.vario = vel;
//    applyDeadband(alt.vario, 5);
//    BaroPID -= constrain(conf.pid[PIDALT].D8 * alt.vario >>4, -150, 150);
//  #endif
//  return 1;
//}


