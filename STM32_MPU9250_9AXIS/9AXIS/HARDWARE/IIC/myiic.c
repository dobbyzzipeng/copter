#include "myiic.h"
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////	  

void IIC_Init(void)
{					     
	GPIO_InitTypeDef GPIO_InitStructure;
	//RCC->APB2ENR|=1<<4;//先使能外设IO PORTB时钟 
	RCC_APB2PeriphClockCmd(_RCC_GPIOX_APB2PERIPH, ENABLE );	
	   
	GPIO_InitStructure.GPIO_Pin = _IIC_SCL_PIN|_IIC_SDA_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(_IIC_PORT, &GPIO_InitStructure);
 
	GPIO_InitStructure.GPIO_Pin = _MPU6050_INT;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU ;   //上拉输入
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(_MPU6050_PORT, &GPIO_InitStructure);
	IIC_SCL=1;
	IIC_SDA=1;
}

//产生IIC起始信号
void IIC_Start(void)
{
	SDA_OUT();     //sda线输出
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	delay_us(4);
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_SCL=0;//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void IIC_Stop(void)
{
	SDA_OUT();//sda线输出
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_SCL=1; 
	IIC_SDA=1;//发送I2C总线结束信号
	delay_us(4);							   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA设置为输入  
	IIC_SDA=1;delay_us(1);	   
	IIC_SCL=1;delay_us(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL=0;//时钟输出0 	   
	return 0;  
} 
//产生ACK应答
void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}
//不产生ACK应答		    
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}					 				     
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); //SDA配置为输出	    
    IIC_SCL=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(2);   //对TEA5767这三个延时都是必须的
		IIC_SCL=1;
		delay_us(2); 
		IIC_SCL=0;	
		delay_us(2);
    }	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA设置为输入
    for(i=0;i<8;i++ )
	 {
        IIC_SCL=0; 
        delay_us(2);
		    IIC_SCL=1;
        receive<<=1;//receive=receive<<1;
        if(READ_SDA)receive++;   
		    delay_us(1); 
    }					 
    if (!ack)
        IIC_NAck();//发送nACK
    else
        IIC_Ack(); //发送ACK   
    return receive;
}

//指定器件指定地址读取一个字节
//devices address slave
//ReadAddr 指定地址
//返回值u8 读到的数据
u8 IIC_ReadOneByte(u8 SlaveAddr ,u8 ReadAddr)
{
   u8 temp=0;		  	    																 
   IIC_Start();//起始
   IIC_Send_Byte(SlaveAddr);   //发送器件地址+写指令
   IIC_Wait_Ack(); //等待应答
   IIC_Send_Byte(ReadAddr);   //发送寄存器地址
   IIC_Wait_Ack();	//等待应答    
   IIC_Start();  	 //重复起始条件	   
   IIC_Send_Byte(SlaveAddr+1); //发送器件指令+读指令
   IIC_Wait_Ack();	 
   temp=IIC_Read_Byte(0);	//不应答	   
   IIC_Stop();//产生一个停止条件	    
	return temp;
}
//指定器件指定地址写入一个字节
//devices address slave
//ReadAddr 指定地址
//DataToWrite 要写入的数据
void IIC_WriteOneByte(u8 SlaveAddr, u8 WriteAddr,u8 DataToWrite)
{
  IIC_Start(); //开启IIC
  IIC_Send_Byte(SlaveAddr);   //发送器件地址+写指令
  IIC_Wait_Ack();	   
  IIC_Send_Byte(WriteAddr);   //发送寄存器地址
  IIC_Wait_Ack(); 	 										  		   
  IIC_Send_Byte(DataToWrite);     //发送字节数据							   
  IIC_Wait_Ack();  		    	   
  IIC_Stop();//产生一个停止条件 
  delay_ms(10);	 
}
//功能：指定器件地址、指定起始寄存器、读取指定长度的数据，装入数组
//输入：SlaveAddr：器件地址 RegAddr：指定其实寄存器 Len：要读取的数据数
//输出：数组
//返回：无
void IIC_ReadmultiyBytes(u8 SlaveAddr,u8 RegAddr,u8 Len,u8 *Buf_Addr)
{
   u8 i;
   IIC_Start();//起始
   IIC_Send_Byte(SlaveAddr);   //发送器件地址+写指令
   IIC_Wait_Ack(); //等待应答
   IIC_Send_Byte(RegAddr);   //发送寄存器地址
   IIC_Wait_Ack();	//等待应答    
   IIC_Start();  	 //重复起始条件	   
   IIC_Send_Byte(SlaveAddr+1); //发送器件指令+读指令
   IIC_Wait_Ack();
	for(i=0;i<Len;i++)
	{
		if(i!=Len-1) 
		{
			Buf_Addr[i]=IIC_Read_Byte(1);//应答	
		}
		else
		Buf_Addr[i]=IIC_Read_Byte(0);	//	
   }
	IIC_Stop();//IIC总线停止	 
}


