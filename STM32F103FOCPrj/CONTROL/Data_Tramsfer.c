#include "Data_Tramsfer.h"
#include "stm32f10x.h"
#include "usart.h"
#include "delay.h"
#include "imu.h"
#include "mpu6050.h"
//#include "24l01.h"
#include "led.h"


extern unsigned char tx_buffer[DMA_TX_LENGTH];

void Data_Receive_Anl(u8 *data_buf,uint16_t usart_flag)
{
	uint16_t i=0;
	uint16_t length=0;
	float buf_1=0.0,buf_2=0.0,buf_3=0.0;
// 	uint16_t buf_4=0,buf_5=0,buf_6=0;
	float buf_4=0,buf_5=0,buf_6=0;
	u8 buf[25]={0};

	length=usart_flag&0x03ff;//��ȡ��Ч���ݳ���
	for(i=0;i<length;i++)
	{
		buf[i]=ASII_To_Char(data_buf+i);
	}
	
	if((usart_flag&0x0800)==0)//������PID����
	{
		buf_1=buf[1]*100+buf[2]*10+buf[3]+buf[5]/(float)10.0+buf[6]/(float)100.0;//֡��ʽ��AB��100.00��200.00��300.00E
		buf_2=buf[8]*100+buf[9]*10+buf[10]+buf[12]/(float)10.0+buf[13]/(float)100.0;
		buf_3=buf[15]*100+buf[16]*10+buf[17]+buf[19]/(float)10.0+buf[20]/(float)100.0;
		
		if((usart_flag&(uint16_t)0x4000)!=0x00)//�����֣�PITCH
		{
		

			LED1_ON();
//  		printf("%7.3f\t%7.3f\t%7.3f\r\n",Pitchpid.kp,Pitchpid.ki,Pitchpid.kd);
			
		}
		else if((usart_flag&(uint16_t)0x2000)!=0x00)//�����֣�ROLL
		{
			
			LED1_ON();
// 			printf("%7.3f\t%7.3f\t%7.3f\r\n",Rollpid.kp,Rollpid.ki,Rollpid.kd);
		}
		else if((usart_flag&(uint16_t)0x1000)!=0x00)//�����֣�YAW
		{
			
			LED1_ON();
//  		printf("%7.3f\t%7.3f\t%7.3f\r\n",Yawpid.kp,Yawpid.ki,Yawpid.kd);
		}
		else
		{;}
	}
	else
	{
		buf_4=buf[1]*10000+buf[2]*1000+buf[3]*100+buf[4]*10+buf[5];//֡��ʽ��AB��100.00��200.00��300.00E
		buf_5=buf[8]*10000+buf[9]*1000+buf[10]*100+buf[11]*10+buf[12];
		buf_6=buf[15]*10000+buf[16]*1000+buf[17]*100+buf[18]*10+buf[19];

		//���� pitch roll
//		printf("%3.3f\t%3.3f\t%3.3f\r\n",Target.thottle,Target.pitch,Target.roll);
		LED1_ON();
	}
}


u8 ASII_To_Char(u8 *asii_code)
{
	u8 temp=0;
	switch (*asii_code)
	{
		case 0x30:
		temp= 0;	
		break;
					
		case 0x31:
		temp= 1;				
		break;
		
		case 0x32:
		temp=  2;				
		break;
		
		case 0x33:
		temp=  3;				
		break;
		
		case 0x34:
		temp=  4;				
		break;
		
		case 0x35:
		temp=  5;				
		break;
		
		case 0x36:
		temp=  6;				
		break;
		
		case 0x37:
		temp=  7;				
		break;
		
		case 0x38:
		temp=  8;				
		break;
				
		case 0x39:
		temp=  9;				
		break;
		
		default:
			break;
	}
	return temp;
}


