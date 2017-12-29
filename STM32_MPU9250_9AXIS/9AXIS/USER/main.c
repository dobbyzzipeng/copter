#include "led.h"
#include "delay.h"
#include "sys.h"
#include "mpu92xx.h"
#include "myiic.h"
#include "global.h"
#include "timer.h"
#include "ahrs.h"
#include "ins.h"
/*
*Corporation:	www.makeblock.com
*Project:		Airblock
*brief:			AHRS 9 axis fusion algorighm
*Device:		STM32F103RCT6 
*MODULE:		MPU9250(MPU6500+AK8963)
*interface:     SPI
*IO:			SCK->PA5  MISO->PA6 MOSI->PA7 CS->PA4
*				LED0->PC12
*zzp@2016-11-24
*/
/*
2ms一次运算周期，算出三轴姿态以及X,Y,Z轴方向的速度。
磁力计设定每20S运行一次校准，在20S内可以对磁力计三个轴分别绕圈圈求出MAX和MIN。
*/
uint8_t system_tim_flag=0;
uint32_t system_runcnt=0;

 int main(void)
 {
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);//can use PA15|PB3|PB4	 
	delay_init();    //延时函数初始化
	delay_ms(1000);	 
	LED_Init();		 //初始化与LED连接的硬件接口
//	uart_init(115200);  //初始化串口2
	#if MPU9250_USE_SPI
	SPI1_Init();//SPI初始化
	#else
	IIC_Init();
	#endif
	delay_ms(100);
	MPU9250_Init();
	SYS_Timer2_Configuration(2000);//2ms period
	LED0_ON();
	while(1)
	{
		if(system_tim_flag)
		{
			IMU_getYawPitchRoll(angle);
			INS_Update(0.002f);
			system_tim_flag=0;
			if(++system_runcnt>10000)
			{
			  MPU92XXMAGCALFLAG=1;//每隔10000x0.002s进行一次磁力计校准
			  system_runcnt=0;
			}
		}
	}
 }

