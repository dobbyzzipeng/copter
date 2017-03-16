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
//校验数据读取
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
//写温度指令
void BMP180_Temperature_Command(void)
{
	_BMP180_WriteOneByte(BMP180_W,_TEMPERATURE_ADDRESS_,_TEMPERATURE_COMMAND_);
}
//BMP180_Temperature_Read
//读温度数据 未补偿
//写温度指令数据4.5ms转换时间之后读取
//when oss is 3,25ms later to read
long BMP180_Temperature_Read(void)
{
	unsigned char buffer[2]={0};
	_BMP180_ReadmultiyBytes(BMP180_W,_TEMPERATURE_DATAMSB_,2,buffer);
	return (long)((short)buffer[0]<<8|buffer[1]);//ut
}
//BMP180_Temperature_Cal
//温度数据校准计算 补偿
//单位：0.1℃
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
//写指令
void BMP180_Pressure_Command(void)
{
	_BMP180_WriteOneByte(BMP180_W,_PRESSURE_ADDRESS_,_PRESSURE_COMMAND_+(oss<<6));
}
//BMP180_Pressure_Read
//气压数据读取 25.5ms later to read 25cm relosution
//未补偿数据
long BMP180_Pressure_Read(void)
{
	unsigned char buf[3]={0};
	_BMP180_ReadmultiyBytes(BMP180_W,_PRESSURE_DATAMSB_,3,buf);
	return ((long)((long)buf[0]<<16|(long)buf[1]<<8|buf[2]))>>(8-oss);//up
}
//气压计数据校准
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
	_BMP180_WriteOneByte(BMP180_W,BMP180_SOLFT_RESET,BMP180_SOLFT_RESET_DATA);//软件复位
	
}

void _BMP180_IIC_Init(void)
{					     
	GPIO_InitTypeDef GPIO_InitStructure;
	//RCC->APB2ENR|=1<<4;//先使能外设IO PORTB时钟 
	RCC_APB2PeriphClockCmd(_BMP180_RCC_GPIOX_APB2PERIPH, ENABLE );	
	   
	GPIO_InitStructure.GPIO_Pin = _BMP180_IIC_SCL_PIN|_BMP180_IIC_SDA_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(_BMP180_IIC_PORT, &GPIO_InitStructure);
 
	_BMP180_IIC_SCL=1;
	_BMP180_IIC_SDA=1;
}

//产生IIC起始信号
void _BMP180_IIC_Start(void)
{
	_BMP180_SDA_OUT();     //sda线输出
	_BMP180_IIC_SDA=1;	  	  
	_BMP180_IIC_SCL=1;
	delay_us(4);
 	_BMP180_IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	_BMP180_IIC_SCL=0;//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void _BMP180_IIC_Stop(void)
{
	_BMP180_SDA_OUT();//sda线输出
	_BMP180_IIC_SCL=0;
	_BMP180_IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	_BMP180_IIC_SCL=1; 
	_BMP180_IIC_SDA=1;//发送I2C总线结束信号
	delay_us(4);							   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 _BMP180_IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	_BMP180_SDA_IN();      //SDA设置为输入  
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
	_BMP180_IIC_SCL=0;//时钟输出0 	   
	return 0;  
} 
//产生ACK应答
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
//不产生ACK应答		    
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
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void _BMP180_IIC_Send_Byte(u8 txd)
{
    u8 t;   
	_BMP180_SDA_OUT(); //SDA配置为输出	    
    _BMP180_IIC_SCL=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        _BMP180_IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(2);   //对TEA5767这三个延时都是必须的
		_BMP180_IIC_SCL=1;
		delay_us(2); 
		_BMP180_IIC_SCL=0;	
		delay_us(2);
    }	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 _BMP180_IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	_BMP180_SDA_IN();//SDA设置为输入
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
        _BMP180_IIC_NAck();//发送nACK
    else
        _BMP180_IIC_Ack(); //发送ACK   
    return receive;
}
//读1个字节
// u8 IIC_Read_Byte_1()
// {
// 	unsigned char i,receive=0;
// 	SDA_IN();//SDA设置为输入
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
//指定器件指定地址读取一个字节
//devices address slave
//ReadAddr 指定地址
//返回值u8 读到的数据
u8 _BMP180_ReadOneByte(u8 SlaveAddr ,u8 ReadAddr)
{
   u8 temp=0;		  	    																 
   _BMP180_IIC_Start();//起始
	 _BMP180_IIC_Send_Byte(SlaveAddr);   //发送器件地址+写指令
   _BMP180_IIC_Wait_Ack(); //等待应答
   _BMP180_IIC_Send_Byte(ReadAddr);   //发送寄存器地址
	 _BMP180_IIC_Wait_Ack();	//等待应答    
	 _BMP180_IIC_Start();  	 //重复起始条件	   
	 _BMP180_IIC_Send_Byte(SlaveAddr+1); //发送器件指令+读指令
	 _BMP180_IIC_Wait_Ack();	 
   temp=_BMP180_IIC_Read_Byte(0);	//不应答	   
   _BMP180_IIC_Stop();//产生一个停止条件	    
	return temp;
}
//指定器件指定地址写入一个字节
//devices address slave
//ReadAddr 指定地址
//DataToWrite 要写入的数据
void _BMP180_WriteOneByte(u8 SlaveAddr, u8 WriteAddr,u8 DataToWrite)
{
  _BMP180_IIC_Start(); //开启IIC
  _BMP180_IIC_Send_Byte(SlaveAddr);   //发送器件地址+写指令
  _BMP180_IIC_Wait_Ack();	   
  _BMP180_IIC_Send_Byte(WriteAddr);   //发送寄存器地址
	_BMP180_IIC_Wait_Ack(); 	 										  		   
	_BMP180_IIC_Send_Byte(DataToWrite);     //发送字节数据							   
	_BMP180_IIC_Wait_Ack();  		    	   
  _BMP180_IIC_Stop();//产生一个停止条件 
	delay_ms(10);	 
}
//功能：指定器件地址、指定起始寄存器、读取指定长度的数据，装入数组
//输入：SlaveAddr：器件地址 RegAddr：指定其实寄存器 Len：要读取的数据数
//输出：数组
//返回：无
void _BMP180_ReadmultiyBytes(u8 SlaveAddr,u8 RegAddr,u8 Len,u8 *Buf_Addr)
{
	 u8 i;
   _BMP180_IIC_Start();//起始
	 _BMP180_IIC_Send_Byte(SlaveAddr);   //发送器件地址+写指令
   _BMP180_IIC_Wait_Ack(); //等待应答
   _BMP180_IIC_Send_Byte(RegAddr);   //发送寄存器地址
	_BMP180_IIC_Wait_Ack();	//等待应答    
	 _BMP180_IIC_Start();  	 //重复起始条件	   
	 _BMP180_IIC_Send_Byte(SlaveAddr+1); //发送器件指令+读指令
	 _BMP180_IIC_Wait_Ack();
	for(i=0;i<Len;i++)
	{
		if(i!=Len-1) 
			{
				Buf_Addr[i]=_BMP180_IIC_Read_Byte(1);//应答	
			}
		else
		Buf_Addr[i]=_BMP180_IIC_Read_Byte(0);	//
		
   }
	_BMP180_IIC_Stop();//IIC总线停止	 
}


