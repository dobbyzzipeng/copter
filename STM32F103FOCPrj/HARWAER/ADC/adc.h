#ifndef __ADC_H
#define __ADC_H	
#include "sys.h"
								  
////////////////////////////////////////////////////////////////////////////////// 

#define _VOLTAGE_CHA 		GPIO_Pin_0	 	//PB0
#define _VOLTAGE_CHB 		GPIO_Pin_1	  //PB1
#define _VOLTAGE_PORT   	GPIOB
//
#define	_ADC_Address	((u32)0x4001244C)
/************************/
void Adc_Init(void);
u16  Get_Adc(u8 ch); 
u16  Get_Adc_Average(u8 ch,u8 times); 
#endif 
