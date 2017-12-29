#include "myiic.h"
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////	  

void IIC_Init(void)
{					     
	GPIO_InitTypeDef GPIO_InitStructure;
	//RCC->APB2ENR|=1<<4;//��ʹ������IO PORTBʱ�� 
	RCC_APB2PeriphClockCmd(_RCC_GPIOX_APB2PERIPH, ENABLE );	
	   
	GPIO_InitStructure.GPIO_Pin = _IIC_SCL_PIN|_IIC_SDA_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(_IIC_PORT, &GPIO_InitStructure);
 
	GPIO_InitStructure.GPIO_Pin = _MPU6050_INT;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU ;   //��������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(_MPU6050_PORT, &GPIO_InitStructure);
	IIC_SCL=1;
	IIC_SDA=1;
}

//����IIC��ʼ�ź�
void IIC_Start(void)
{
	SDA_OUT();     //sda�����
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	delay_us(4);
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_SCL=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  
//����IICֹͣ�ź�
void IIC_Stop(void)
{
	SDA_OUT();//sda�����
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_SCL=1; 
	IIC_SDA=1;//����I2C���߽����ź�
	delay_us(4);							   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA����Ϊ����  
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
	IIC_SCL=0;//ʱ�����0 	   
	return 0;  
} 
//����ACKӦ��
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
//������ACKӦ��		    
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
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); //SDA����Ϊ���	    
    IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(2);   //��TEA5767��������ʱ���Ǳ����
		IIC_SCL=1;
		delay_us(2); 
		IIC_SCL=0;	
		delay_us(2);
    }	 
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA����Ϊ����
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
        IIC_NAck();//����nACK
    else
        IIC_Ack(); //����ACK   
    return receive;
}

//ָ������ָ����ַ��ȡһ���ֽ�
//devices address slave
//ReadAddr ָ����ַ
//����ֵu8 ����������
u8 IIC_ReadOneByte(u8 SlaveAddr ,u8 ReadAddr)
{
   u8 temp=0;		  	    																 
   IIC_Start();//��ʼ
   IIC_Send_Byte(SlaveAddr);   //����������ַ+дָ��
   IIC_Wait_Ack(); //�ȴ�Ӧ��
   IIC_Send_Byte(ReadAddr);   //���ͼĴ�����ַ
   IIC_Wait_Ack();	//�ȴ�Ӧ��    
   IIC_Start();  	 //�ظ���ʼ����	   
   IIC_Send_Byte(SlaveAddr+1); //��������ָ��+��ָ��
   IIC_Wait_Ack();	 
   temp=IIC_Read_Byte(0);	//��Ӧ��	   
   IIC_Stop();//����һ��ֹͣ����	    
	return temp;
}
//ָ������ָ����ַд��һ���ֽ�
//devices address slave
//ReadAddr ָ����ַ
//DataToWrite Ҫд�������
void IIC_WriteOneByte(u8 SlaveAddr, u8 WriteAddr,u8 DataToWrite)
{
  IIC_Start(); //����IIC
  IIC_Send_Byte(SlaveAddr);   //����������ַ+дָ��
  IIC_Wait_Ack();	   
  IIC_Send_Byte(WriteAddr);   //���ͼĴ�����ַ
  IIC_Wait_Ack(); 	 										  		   
  IIC_Send_Byte(DataToWrite);     //�����ֽ�����							   
  IIC_Wait_Ack();  		    	   
  IIC_Stop();//����һ��ֹͣ���� 
  delay_ms(10);	 
}
//���ܣ�ָ��������ַ��ָ����ʼ�Ĵ�������ȡָ�����ȵ����ݣ�װ������
//���룺SlaveAddr��������ַ RegAddr��ָ����ʵ�Ĵ��� Len��Ҫ��ȡ��������
//���������
//���أ���
void IIC_ReadmultiyBytes(u8 SlaveAddr,u8 RegAddr,u8 Len,u8 *Buf_Addr)
{
   u8 i;
   IIC_Start();//��ʼ
   IIC_Send_Byte(SlaveAddr);   //����������ַ+дָ��
   IIC_Wait_Ack(); //�ȴ�Ӧ��
   IIC_Send_Byte(RegAddr);   //���ͼĴ�����ַ
   IIC_Wait_Ack();	//�ȴ�Ӧ��    
   IIC_Start();  	 //�ظ���ʼ����	   
   IIC_Send_Byte(SlaveAddr+1); //��������ָ��+��ָ��
   IIC_Wait_Ack();
	for(i=0;i<Len;i++)
	{
		if(i!=Len-1) 
		{
			Buf_Addr[i]=IIC_Read_Byte(1);//Ӧ��	
		}
		else
		Buf_Addr[i]=IIC_Read_Byte(0);	//	
   }
	IIC_Stop();//IIC����ֹͣ	 
}


