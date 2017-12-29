#include "bmp180.h"
#include "delay.h"
#include "led.h"
#include "usart.h"
#include "timer.h"
#include <math.h>
/*@bref
*temperature converse time 4.5ms
*ultra high  resolution in pressure time 25.5ms 25cm
*advanced resolution in pressure time 76.5ms    17cm
*/	  
//unsigned char  BMP180_BUF[22]={0};
BMP180_CALDATA Bmp180_CalData;
BARO Baro;
float TEMPERATURE=0.0f,PRESSURE=0.0f;
#define FATOR 0.190295f//1/5.255f
void BMP180_INIT(void)
{
	_BMP180_IIC_Init();
	BMP180_Check();
	BMP180_Registers_Init();
	BMP180_Calibration();
}
/**********************************/
//BMP180_CalData_Read
//У�����ݶ�ȡ
void BMP180_CalData_Read(BMP180_CALDATA* bmp_caldata)
{
	unsigned char  BMP180_BUF[22]={0};
	
	_BMP180_ReadmultiyBytes(BMP180_W,_AC1_M,22,BMP180_BUF);
	
	bmp_caldata->AC1=(short)(((short)BMP180_BUF[0]<<8)|BMP180_BUF[1]);
	bmp_caldata->AC2=(short)(((short)BMP180_BUF[2]<<8)|BMP180_BUF[3]);
	bmp_caldata->AC3=(short)(((short)BMP180_BUF[4]<<8)|BMP180_BUF[5]);
	bmp_caldata->AC4=(unsigned short)(((unsigned short)BMP180_BUF[6]<<8)|BMP180_BUF[7]);
	bmp_caldata->AC5=(unsigned short)(((unsigned short)BMP180_BUF[8]<<8)|BMP180_BUF[9]);
	bmp_caldata->AC6=(unsigned short)(((unsigned short)BMP180_BUF[10]<<8)|BMP180_BUF[11]);
	bmp_caldata->B1=(short)(((short)BMP180_BUF[12]<<8)|BMP180_BUF[13]);
	bmp_caldata->B2=(short)(((short)BMP180_BUF[14]<<8)|BMP180_BUF[15]);
	bmp_caldata->MB=(short)(((short)BMP180_BUF[16]<<8)|BMP180_BUF[17]);
	bmp_caldata->MC=(short)(((short)BMP180_BUF[18]<<8)|BMP180_BUF[19]);
	bmp_caldata->MD=(short)(((short)BMP180_BUF[20]<<8)|BMP180_BUF[21]);
}
/**************************************/
void BMP180_Calibration(void)
{
	BMP180_CalData_Read(&Bmp180_CalData);
}
//BMP180_Temperature_Command
//д�¶�ָ��
void BMP180_Temperature_Command(void)
{
	_BMP180_WriteOneByte(BMP180_W,_TEMPERATURE_ADDRESS_,_TEMPERATURE_COMMAND_);
}
//BMP180_Temperature_Read
//���¶����� δ����
//д�¶�ָ������4.5msת��ʱ��֮���ȡ
//when oss is 3,25ms later to read
long BMP180_Temperature_Read(void)
{
	unsigned char buffer[2]={0};
	_BMP180_ReadmultiyBytes(BMP180_W,_TEMPERATURE_DATAMSB_,2,buffer);
	return (long)((short)buffer[0]<<8|buffer[1]);//ut
}
//BMP180_Temperature_Cal
//�¶�����У׼���� ����
//��λ��0.1��
long BMP180_Temperature_Cal(BMP180_CALDATA* bmp_caldata,long ut)
{
	long Temperature=0;
	bmp_caldata->X1=((ut-(long)bmp_caldata->AC6)*(long)bmp_caldata->AC5)>>15;
	bmp_caldata->X2=((long)bmp_caldata->MC<<11)/(bmp_caldata->X1+bmp_caldata->MD);
	bmp_caldata->B5=bmp_caldata->X1+bmp_caldata->X2;
	Temperature=(bmp_caldata->B5+8)>>4;
	return Temperature/10;
}
//BMP180_Pressure_Command
//дָ��
void BMP180_Pressure_Command(void)
{
	_BMP180_WriteOneByte(BMP180_W,_PRESSURE_ADDRESS_,_PRESSURE_COMMAND_+(oss<<6));
}
//BMP180_Pressure_Read
//��ѹ���ݶ�ȡ 25.5ms later to read 25cm relosution
//δ��������
long BMP180_Pressure_Read(void)
{
	unsigned char buf[3]={0};
	_BMP180_ReadmultiyBytes(BMP180_W,_PRESSURE_DATAMSB_,3,buf);
	return ((long)((long)buf[0]<<16|(long)buf[1]<<8|buf[2]))>>(8-oss);//up
}
//��ѹ������У׼
long BMP180_Pressure_Cal(BMP180_CALDATA* bmp_caldata,long up)
{
	long pressure=0;
	bmp_caldata->B6=bmp_caldata->B5-4000;
	bmp_caldata->X1=((int32_t)bmp_caldata->B2*(bmp_caldata->B6*bmp_caldata->B6 >>12))>>11;
	bmp_caldata->X2=(int32_t)bmp_caldata->AC2*bmp_caldata->B6>>11;
	bmp_caldata->X3=bmp_caldata->X1+bmp_caldata->X2;
	bmp_caldata->B3=((((long)bmp_caldata->AC1*4+bmp_caldata->X3)<<oss)+2)>>2;//
	bmp_caldata->X1=(int32_t)bmp_caldata->AC3*bmp_caldata->B6>>13;
	bmp_caldata->X2=((int32_t)bmp_caldata->B1*(bmp_caldata->B6*bmp_caldata->B6>>12))>>16;
	bmp_caldata->X3=((bmp_caldata->X1+bmp_caldata->X2)+2)>>2;
	bmp_caldata->B4=(int32_t)bmp_caldata->AC4*(unsigned long)(bmp_caldata->X3+32768)>>15;
	bmp_caldata->B7=((unsigned long)up-bmp_caldata->B3)*(50000>>oss);
	if(bmp_caldata->B7<0x80000000)
	{
		pressure=(bmp_caldata->B7*2)/bmp_caldata->B4;
	}
	else
	{
		pressure=(bmp_caldata->B7/bmp_caldata->B4)*2;
	}
	bmp_caldata->X1=(pressure>>8)*(pressure>>8);
	bmp_caldata->X1=(bmp_caldata->X1*3038)>>16;
	bmp_caldata->X2=(-7357*pressure)>>16;
	pressure+=(bmp_caldata->X1+bmp_caldata->X2+3791)>>4;
	return pressure;//p
}

float BMP180_ALT_GET(long p)
{
	return 44330*(1-pow((p/pressure0),FATOR));
}

//return altitude m
float BMP180_Update(void)
{
//	long /*Tem=0,*/pre=0;
	static float lastaltitude=0.0f;
	float thisaltitude=0.0f;
	if(Baro.barostate==0)
	{
//			BMP180_Temperature_Command();
//		Baro.deadline=microms()+10;//temperature conversion is 5ms period
//		if(microms()>=Baro.deadline)
//		{
			BMP180_Pressure_Command();
		  TEMPERATURE/*Tem*/=BMP180_Temperature_Cal(&Bmp180_CalData,BMP180_Temperature_Read());
		  Baro.deadline=microms()+80;//80ms period
		  Baro.barostate=1;
//		}
	}
	else if(Baro.barostate==1)
	{
		if(microms()>Baro.deadline)
		{
			BMP180_Temperature_Command();
			PRESSURE/*pre*/=BMP180_Pressure_Cal(&Bmp180_CalData,BMP180_Pressure_Read());
			thisaltitude=BMP180_ALT_GET(PRESSURE);
			Baro.barostate=0;
			lastaltitude=thisaltitude;
			return thisaltitude;
		}
	}
		return lastaltitude;
}

void BMP180_Check(void)
{
	unsigned char id=0,bmpreadcnt=0;
	id=_BMP180_ReadOneByte(BMP180_W,BMP180_CHIP_ID);
	if(id!=0x55)
	{
		for(bmpreadcnt=0;bmpreadcnt<10;bmpreadcnt++)
		{
			if(_BMP180_ReadOneByte(BMP180_W,BMP180_CHIP_ID)!=0x55)
			{
				delay_ms(1);
				if(bmpreadcnt==9)
				{
					printf("bmp180 fail\r\n");
					while(1)
					{
						delay_ms(100);
						LED1_TOGGLE();
					}
				}
			}
			else
			{
				break;
			}
		}
	}
	else
	{
		printf("bmp180 ok\r\n");
	}
}

void BMP180_Registers_Init(void)
{
	_BMP180_WriteOneByte(BMP180_W,BMP180_SOLFT_RESET,BMP180_SOLFT_RESET_DATA);//�����λ
	
}

void _BMP180_IIC_Init(void)
{					     
	GPIO_InitTypeDef GPIO_InitStructure;
	//RCC->APB2ENR|=1<<4;//��ʹ������IO PORTBʱ�� 
	RCC_APB2PeriphClockCmd(_BMP180_RCC_GPIOX_APB2PERIPH, ENABLE );	
	   
	GPIO_InitStructure.GPIO_Pin = _BMP180_IIC_SCL_PIN|_BMP180_IIC_SDA_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(_BMP180_IIC_PORT, &GPIO_InitStructure);
 
	_BMP180_IIC_SCL=1;
	_BMP180_IIC_SDA=1;
}

//����IIC��ʼ�ź�
void _BMP180_IIC_Start(void)
{
	_BMP180_SDA_OUT();     //sda�����
	_BMP180_IIC_SDA=1;	  	  
	_BMP180_IIC_SCL=1;
	delay_us(4);
 	_BMP180_IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	_BMP180_IIC_SCL=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  
//����IICֹͣ�ź�
void _BMP180_IIC_Stop(void)
{
	_BMP180_SDA_OUT();//sda�����
	_BMP180_IIC_SCL=0;
	_BMP180_IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	_BMP180_IIC_SCL=1; 
	_BMP180_IIC_SDA=1;//����I2C���߽����ź�
	delay_us(4);							   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 _BMP180_IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	_BMP180_SDA_IN();      //SDA����Ϊ����  
	_BMP180_IIC_SDA=1;delay_us(1);	   
	_BMP180_IIC_SCL=1;delay_us(1);	 
	while(_BMP180_READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			_BMP180_IIC_Stop();
			return 1;
		}
	}
	_BMP180_IIC_SCL=0;//ʱ�����0 	   
	return 0;  
} 
//����ACKӦ��
void _BMP180_IIC_Ack(void)
{
	_BMP180_IIC_SCL=0;
	_BMP180_SDA_OUT();
	_BMP180_IIC_SDA=0;
	delay_us(2);
	_BMP180_IIC_SCL=1;
	delay_us(2);
	_BMP180_IIC_SCL=0;
}
//������ACKӦ��		    
void _BMP180_IIC_NAck(void)
{
	_BMP180_IIC_SCL=0;
	_BMP180_SDA_OUT();
	_BMP180_IIC_SDA=1;
	delay_us(2);
	_BMP180_IIC_SCL=1;
	delay_us(2);
	_BMP180_IIC_SCL=0;
}					 				     
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void _BMP180_IIC_Send_Byte(u8 txd)
{
    u8 t;   
	_BMP180_SDA_OUT(); //SDA����Ϊ���	    
    _BMP180_IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        _BMP180_IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(2);   //��TEA5767��������ʱ���Ǳ����
		_BMP180_IIC_SCL=1;
		delay_us(2); 
		_BMP180_IIC_SCL=0;	
		delay_us(2);
    }	 
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 _BMP180_IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	_BMP180_SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	 {
       _BMP180_IIC_SCL=0; 
        delay_us(2);
		    _BMP180_IIC_SCL=1;
        receive<<=1;//receive=receive<<1;
        if(_BMP180_READ_SDA)receive++;   
		    delay_us(1); 
    }					 
    if (!ack)
        _BMP180_IIC_NAck();//����nACK
    else
        _BMP180_IIC_Ack(); //����ACK   
    return receive;
}
//��1���ֽ�
// u8 IIC_Read_Byte_1()
// {
// 	unsigned char i,receive=0;
// 	SDA_IN();//SDA����Ϊ����
//     for(i=0;i<8;i++ )
// 	 {
//         IIC_SCL=0; 
//         delay_us(2);
// 		    IIC_SCL=1;
//         receive<<=1;//receive=receive<<1;
//         if(READ_SDA)receive++;   
// 		    delay_us(1); 
//     }					  
//     return receive;
// }
//ָ������ָ����ַ��ȡһ���ֽ�
//devices address slave
//ReadAddr ָ����ַ
//����ֵu8 ����������
u8 _BMP180_ReadOneByte(u8 SlaveAddr ,u8 ReadAddr)
{
   u8 temp=0;		  	    																 
   _BMP180_IIC_Start();//��ʼ
	 _BMP180_IIC_Send_Byte(SlaveAddr);   //����������ַ+дָ��
   _BMP180_IIC_Wait_Ack(); //�ȴ�Ӧ��
   _BMP180_IIC_Send_Byte(ReadAddr);   //���ͼĴ�����ַ
	 _BMP180_IIC_Wait_Ack();	//�ȴ�Ӧ��    
	 _BMP180_IIC_Start();  	 //�ظ���ʼ����	   
	 _BMP180_IIC_Send_Byte(SlaveAddr+1); //��������ָ��+��ָ��
	 _BMP180_IIC_Wait_Ack();	 
   temp=_BMP180_IIC_Read_Byte(0);	//��Ӧ��	   
   _BMP180_IIC_Stop();//����һ��ֹͣ����	    
	return temp;
}
//ָ������ָ����ַд��һ���ֽ�
//devices address slave
//ReadAddr ָ����ַ
//DataToWrite Ҫд�������
void _BMP180_WriteOneByte(u8 SlaveAddr, u8 WriteAddr,u8 DataToWrite)
{
  _BMP180_IIC_Start(); //����IIC
  _BMP180_IIC_Send_Byte(SlaveAddr);   //����������ַ+дָ��
  _BMP180_IIC_Wait_Ack();	   
  _BMP180_IIC_Send_Byte(WriteAddr);   //���ͼĴ�����ַ
	_BMP180_IIC_Wait_Ack(); 	 										  		   
	_BMP180_IIC_Send_Byte(DataToWrite);     //�����ֽ�����							   
	_BMP180_IIC_Wait_Ack();  		    	   
  _BMP180_IIC_Stop();//����һ��ֹͣ���� 
	delay_ms(10);	 
}
//���ܣ�ָ��������ַ��ָ����ʼ�Ĵ�������ȡָ�����ȵ����ݣ�װ������
//���룺SlaveAddr��������ַ RegAddr��ָ����ʵ�Ĵ��� Len��Ҫ��ȡ��������
//���������
//���أ���
void _BMP180_ReadmultiyBytes(u8 SlaveAddr,u8 RegAddr,u8 Len,u8 *Buf_Addr)
{
	 u8 i;
   _BMP180_IIC_Start();//��ʼ
	 _BMP180_IIC_Send_Byte(SlaveAddr);   //����������ַ+дָ��
   _BMP180_IIC_Wait_Ack(); //�ȴ�Ӧ��
   _BMP180_IIC_Send_Byte(RegAddr);   //���ͼĴ�����ַ
	_BMP180_IIC_Wait_Ack();	//�ȴ�Ӧ��    
	 _BMP180_IIC_Start();  	 //�ظ���ʼ����	   
	 _BMP180_IIC_Send_Byte(SlaveAddr+1); //��������ָ��+��ָ��
	 _BMP180_IIC_Wait_Ack();
	for(i=0;i<Len;i++)
	{
		if(i!=Len-1) 
			{
				Buf_Addr[i]=_BMP180_IIC_Read_Byte(1);//Ӧ��	
			}
		else
		Buf_Addr[i]=_BMP180_IIC_Read_Byte(0);	//
		
   }
	_BMP180_IIC_Stop();//IIC����ֹͣ	 
}


