#ifndef _GLOBALVARIABLE_H_
#define _GLOBALVARIABLE_H_


typedef struct
{
	unsigned char system_usartrx_flag:1;
	unsigned char system_usarttx_flag:1;
    unsigned char system_Cal_flag:1;//�Լ�
	unsigned char system_nrf_flag:1;//nrf24l01
	//	unsigned char system_velctr_flag:1;
	//	unsigned char system_altctr_flag:1;
    unsigned char  system_capture_flag:1;//us100 module
	unsigned char  nrf_flag;
	
    unsigned short system_led_cnt;
	unsigned short nrf_error_cnt;
}FLAGS;

typedef struct
{
	unsigned char controlmode;
	unsigned char throttlemode;
	unsigned char motorflag;
}COPTER;

enum
{
	_NORMODE=1,//��������ģʽ
	_3DMODE=2,//3D����ģʽ
};

enum
{
	STABLIZE=1,
	ALT     =2,
	AUTO    =3,
	LAND    =4,
	_3D     =5,
};




extern FLAGS  Flag;
extern COPTER Copter;
extern unsigned short ADC_Tab[2];//ADC ��ѹ��⻺��
extern signed short   Us100_Distance;//us100ģ���õľ���

#endif

