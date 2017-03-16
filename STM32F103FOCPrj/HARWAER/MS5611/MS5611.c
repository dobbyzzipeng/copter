#include "MS5611.h"
#include "myiic.h"
#include "delay.h"
#include "math.h"

//#define Device_ADDRESS 0x77 ����λ��ַ
#define Slave_ADDRESS 0XEE //1110 1110 ����д

#if USE_BARO==USEMS5611

    const float sea_press = 1013.25;
    unsigned long D1_Press,D2_Temp;
	  float Temp,Pressure,Altitude,dT;
	  double OFF,SENS;
	  float OFF2,SENS2,TEMP2,Aux;
    uint16_t _C[MS561101BA_PROM_REG_COUNT];
		
void Ms5611_init()
{
	reset(); // reset the device to populate its internal PROM registers
  delay_ms(1000); // some safety time
  readPROM(); // reads the PROM into object variables for later use
  delay_ms(1000); // some safety time
}
/*********detasheet page4*********/
void getTemperature(uint8_t OSR) 
{
	D2_Temp=doConversion(OSR);
	delay_ms(10);
	dT=D2_Temp - (((unsigned long)_C[5])<<8); //_C[4]ֵΪ��ȡ�Ļ�׼�¶�20��
  Temp=2000+dT*((unsigned long)_C[6])/8388608;
}
/**********detasheet page4**********/
void getPressure(u8 OSR) 
 {
	 D1_Press=doConversion(OSR);
	 delay_ms(10);
	 OFF=(unsigned long)_C[2]*65536+((unsigned long)_C[4]*dT)/128; 
   SENS=(unsigned long)_C[1]*32768+((unsigned long)_C[3]*dT)/256; 
	 if(Temp<2000)
	 {
   // T2 = (dT*dT) / 0x80000000; 
		Aux = Temp*Temp; 
		OFF2 = 2.5*Aux; 
		SENS2 = 1.25*Aux; 
		Temp = Temp - TEMP2; 
		OFF = OFF - OFF2; 
		SENS = SENS - SENS2;  
		 if(Temp<-1500)
		 {
       OFF2 =OFF2+7*(Temp+1500)*(Temp+1500);
			 SENS2 = SENS2+11*(Temp+1500)*(Temp+1500)/2;
     }
   }
	 Pressure=(D1_Press*SENS/2097152-OFF)/32768;
	 //return Pressure;
 }
/***********************************/
void getAltitude()
{
	//float P1;
	getPressure(MS561101BA_D1_OSR_4096);
	getTemperature(MS561101BA_D2_OSR_4096) ;
	Temp/=100;
	//P1=Pressure/100;
	Pressure/=100;
	Altitude=((pow((sea_press/Pressure),1/5.257)-1.0)*(Temp + 273.15)) / 0.0065;
}
/************AD Transiston***************/
unsigned long doConversion(u8 command) 
{
  unsigned long conversion = 0;
	u8 conv1,conv2,conv3;
  IIC_Start();//��ʼ�ź�
	IIC_Send_Byte(Slave_ADDRESS);//���ʹӻ���ַ������Ϊд
	IIC_Wait_Ack();
	IIC_Send_Byte(command);
	IIC_Wait_Ack();
	IIC_Stop();//ֹͣ
	delay_ms(100);
	
	IIC_Start();
	IIC_Send_Byte(Slave_ADDRESS);//ѡ�дӻ�
	IIC_Wait_Ack();
	IIC_Send_Byte(0);
	IIC_Wait_Ack();
	
	IIC_Start();
	IIC_Send_Byte(Slave_ADDRESS+1);//���ʹӻ���ַ������Ϊ��
	IIC_Wait_Ack();
	conv1=IIC_Read_Byte(1);//Ӧ��
	conv2=IIC_Read_Byte(1);//Ӧ��
	conv3=IIC_Read_Byte(0);//��Ӧ��
	IIC_Stop();//ֹͣ
	conversion=65536*conv1+256*conv2+conv3;//conversion = conv1 << 16 | conv2 << 8 | conv3;
  return conversion;	
}
 /* Reads factory calibration and store it into object variables.
*/
 void readPROM() 
{
	u8 i,d1,d2;
  for (i=0;i<MS561101BA_PROM_REG_COUNT;i++) 
	{
		IIC_Start();		
		IIC_Send_Byte(Slave_ADDRESS);//���ʹӻ���ַ������Ϊд
	  IIC_Wait_Ack();
		IIC_Send_Byte((MS561101BA_PROM_BASE_ADDR+2*i));
		IIC_Wait_Ack();
		
		IIC_Start();//�ظ���ʼ�ź�
		IIC_Send_Byte(Slave_ADDRESS+1);//���ʹӻ���ַ������Ϊ��
		IIC_Wait_Ack();
		d1=IIC_Read_Byte(1);//��ȡ�߰�λ����,Ӧ��
		d2=IIC_Read_Byte(0);//��ȡ�Ͱ�λ���ݣ���Ӧ��
		IIC_Stop();//ֹͣ
		delay_ms(5);
		_C[i]=((u16)d1<<8)|d2;//�ߵͰ�λ���ݺϲ�Ϊһ��ʮ��λ���ݣ������������
	}
}



 //* Send a reset command to the device. With the reset command the device
 //* populates its internal registers with the values read from the PROM.

void reset() 
{
	IIC_Start();
	IIC_Send_Byte(Slave_ADDRESS);//���ʹӻ���ַ������д
	IIC_Wait_Ack();
	IIC_Send_Byte(MS561101BA_RESET);//���͸�λָ��
	IIC_Wait_Ack();
	IIC_Stop();
}

#endif

