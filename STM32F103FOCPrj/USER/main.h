#ifndef _MAIN_H_
#define _MAIN_H_

#include "bsp.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "led.h"
#include "timer.h"
#include "pwm.h"
#include "adc.h"
#include "globalvariable.h"
#include "ins.h"

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

#endif

