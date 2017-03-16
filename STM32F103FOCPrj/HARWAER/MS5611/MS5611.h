#ifndef _MS5611_H
#define _MS5611_H
#include "myiic.h"

// addresses of the device
//高七位地址
//#define MS561101BA_ADDR_CSB_HIGH  0x76   //CBR=1 0x76 I2C address when CSB is connected to HIGH (VCC)
//#define MS561101BA_ADDR_CSB_LOW   0x77   //CBR=0 0x77 I2C address when CSB is connected to LOW (GND)
// registers of the device
//#define MS561101BA_D1 0x40
//#define MS561101BA_D2 0x50
#define MS561101BA_RESET 0x1E

// OSR (Over Sampling Ratio) constants
/*#define MS561101BA_OSR_256 0x00
#define MS561101BA_OSR_512 0x02
#define MS561101BA_OSR_1024 0x04
#define MS561101BA_OSR_2048 0x06
#define MS561101BA_OSR_4096 0x08
*/
#define  MS561101BA_D1_OSR_4096 0x48 
#define  MS561101BA_D2_OSR_4096 0x58

#define MS561101BA_PROM_BASE_ADDR 0xA0 //如果地址取值0XA0，则MS561101BA_PROM_REG_COUNT值为7或8.
//by adding ints from 0 to 6 we can read all the prom configuration values.
// C1 will be at 0xA2 and all the subsequent are multiples of 2
#define MS561101BA_PROM_REG_COUNT 7 // number of registers in the PROM
//#define MS561101BA_PROM_REG_SIZE 2 // size in bytes of a prom registry.

       /*declare of functions*/
    void Ms5611_init(void);
    void getPressure(u8 OSR);
    void getTemperature(uint8_t OSR);
		void getAltitude(void);
		void readPROM(void);
    void reset(void);
		unsigned long doConversion(u8 command);
		void Baro_Update(float dt);
#endif
