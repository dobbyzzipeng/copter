///****************************************
//						极光V1.0 mpu9250驱动 
//						修改时间2015/07/29
//***************************************/
#include "mpu9250.h"
//#include "delay.h"
//#include "sys.h"
//uint8_t	 MPU9250_buf[22];					//spi读取后存放数据

//S_INT16_XYZ	GYRO_OFFSET={0,0,0},ACC_OFFSET={56,86,0};		//零漂

//S_INT16_XYZ MPU9250_ACC_LAST={0,0,0};
//S_INT16_XYZ MPU9250_GYRO_LAST={0,0,0};
//S_INT16_XYZ MPU9250_MAG_LAST={0,0,0};

////陀螺仪数据零偏校准(务必保持四轴静止进行上电校准)
//void Clac_GYRO_OFFECT()
//{
//		static int32_t	tempgx=0,tempgy=0,tempgz=0;
//		uint16_t cnt_g=0;	//计数值
//	
//		//初始数据清零
//		cnt_g = 0;
//		GYRO_OFFSET.X=0;
//		GYRO_OFFSET.Y=0;
//		GYRO_OFFSET.Z=0;
//		tempgx = 0;
//		tempgy = 0;
//		tempgz = 0;
//	
//		//读取数据并计算
//		while(cnt_g!=501)
//		{
//			MPU9250_ReadValue();
//			tempgx+= MPU9250_GYRO_LAST.X;
//			tempgy+= MPU9250_GYRO_LAST.Y;
//			tempgz+= MPU9250_GYRO_LAST.Z;
//			cnt_g++;
//			delay_ms(1);
//		}
//		GYRO_OFFSET.X=tempgx/cnt_g;
//		GYRO_OFFSET.Y=tempgy/cnt_g;
//		GYRO_OFFSET.Z=tempgz/cnt_g;
//}


////MPU9250初始化·
//uint8_t MPU9250_Init(void)
//{
//	uint8_t result=0;
//	result = MPU9250_Read_Reg(WHO_AM_I);
//	if(result==0x73 || result==0x70)	//正确读取到9250的地址
//	{
//		MPU9250_Write_Reg(PWR_MGMT_1,0X80);   //电源管理,复位MPU9250
//		delay_ms(1);
//		MPU9250_Write_Reg(PWR_MGMT_1,0X01);   //选择时钟源
//		MPU9250_Write_Reg(PWR_MGMT_2,0X00);   //使能加速度计和陀螺仪
//		MPU9250_Write_Reg(CONFIG,0X02);				//低通滤波器 0x06 92hz (3.9ms delay) fs=1khz
//		MPU9250_Write_Reg(SMPLRT_DIV,0x00);		//采样率1000/(1+0)=1000HZ
//		MPU9250_Write_Reg(GYRO_CONFIG,0X10);  //陀螺仪测量范围 0X18 正负1000度
//		MPU9250_Write_Reg(ACCEL_CONFIG,0x08); //加速度计测量范围 0X00 正负4g
//		MPU9250_Write_Reg(ACCEL_CONFIG2,0x00);//加速度计速率1khz 滤波器460hz (1.94ms delay)
//		MPU9250_Write_Reg(MAG_INT_PIN_CFG,0x30); //有操作就清空中断标志
//        MPU9250_Write_Reg(MAG_I2C_MST_CTRL,0x4D); // I2C Speed 400 kHz
//		MPU9250_Write_Reg(USER_CTRL,0X30);  //使能MPU9250 AUX
//		delay_ms(1);
//		
//		Init_AK8963();
//		
//		return 0;
//	}
//	return 1;
//}

////MPU9250初始化
//void MPU9250_IOAndSPI_Init(void)
//{
//	GPIO_InitTypeDef GPIO_InitStructure;
//    SPI_InitTypeDef  SPI_InitStructure;

//	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB,ENABLE );//PORTA时钟使能 
//	RCC_APB2PeriphClockCmd( RCC_APB2Periph_SPI1 , ENABLE );//SPI1时钟使能 	
// 	RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO,ENABLE );
//	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4; //MPU9250片选信号
// 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);	//初始化指定IO
// 	GPIO_SetBits(GPIOA,GPIO_Pin_4);//上拉  禁止SPI通信
//	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //PA567复用推挽输出 
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);

//	MPU9250_CS=1;  //MPU9250片选取消
//	
//	SPI_Cmd(SPI1,DISABLE);  //SPI2不使能
//	
//	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //SPI设置为双线双向全双工
//	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//SPI主机
//	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//发送接收8位帧结构
//	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;		//时钟悬空低
//	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	//数据捕获于第1个时钟沿
//	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS信号由软件控制
//	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;		//定义波特率预分频的值:波特率预分频值为16（72M/16=6M）
//	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//数据传输从MSB位开始
//	SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRC值计算的多项式
//	SPI_Init(SPI1, &SPI_InitStructure);  //根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器
//	
//	SPI_Cmd(SPI1,ENABLE);//使能SPI2
//	
//	MPU9250_CS=1;  //MPU9250片选取消
//}

//void Read_AK8963_Data()
//{
//	
//}
////配置AK8963的模式 连续测量模式2 （由于极限速度为138hz 这里使用mode2 可以达到100hz）
//void Init_AK8963()
//{
//	uint8_t temp=0;
//	
//	delay_ms(500);
//	
//	MPU9250_Write_Reg(MAG_I2C_SLV0_ADDR,0x0C);//write I2C addr 
//	MPU9250_Write_Reg(MAG_I2C_SLV0_DO,0x01);	//重启设备 
//	MPU9250_Write_Reg(MAG_I2C_SLV0_REG,AK8963_CNTL2);
//	MPU9250_Write_Reg(MAG_I2C_SLV0_CTRL,0X81);//写入一个数据
//	
//	delay_ms(100);
//	
//	//Set Slave to Read AK8963
//	MPU9250_Write_Reg(MAG_I2C_SLV0_ADDR,0x8c);
//	MPU9250_Write_Reg(MAG_I2C_SLV0_REG,0x00);
//	MPU9250_Write_Reg(MAG_I2C_SLV0_CTRL,0x81);
//	delay_us(1000); //必须加延时 否则出错 最小600us （与I2C通信速度有关） 	可以使用I2C_SLV4 来解决
//	temp = MPU9250_Read_Reg(MAG_EXT_SENS_DATA_00);
//		
//	if(temp == 0x48)
//	{
//			//成功
//			delay_ms(500);
//	}
//	else 
//	{
//			//失败
//			while(1);
//	}
//	
//	MPU9250_Write_Reg(MAG_I2C_SLV0_ADDR,0x0C);//write I2C addr 
//	MPU9250_Write_Reg(MAG_I2C_SLV0_DO,0x16); //选择连续测量模式 16bit输出
//	MPU9250_Write_Reg(MAG_I2C_SLV0_REG,AK8963_CNTL1);
//	MPU9250_Write_Reg(MAG_I2C_SLV0_CTRL,0X81);//写入一个数据
//	delay_us(1000);	//必须加延时 否则出错 最小200us （与I2C通信速度有关）
//	MPU9250_Write_Reg(MAG_I2C_SLV0_ADDR,0x8C);//read I2C addr 
//	MPU9250_Write_Reg(MAG_I2C_SLV0_REG,AK8963_CNTL1);
//	MPU9250_Write_Reg(MAG_I2C_SLV0_CTRL,0X81);//读取一个数据
//	delay_us(1000);	//必须加延时 否则出错 最小200us （与I2C通信速度有关）
//	temp = MPU9250_Read_Reg(MAG_EXT_SENS_DATA_00);
//	if(temp == 0x16)
//	{
//			//初始化成功
//			delay_ms(500);
//	}
//	else 
//	{
//			//初始化失败
//			while(1);
//	}
//	//OLED_Clear();
//	
//	MPU9250_Write_Reg(MAG_I2C_SLV0_ADDR, 0x8C);          // Set AK8963_I2C_ADDR = 7'b000_1100
//	delay_us(1000);
//	MPU9250_Write_Reg(MAG_I2C_SLV0_REG, AK8963_ST1);     // Set Read Reg
//	MPU9250_Write_Reg(MAG_I2C_SLV0_CTRL, 0x88);          // Start Read, 6 bytes	ST1读到ST2
//	delay_us(1000);
//	
//}
////SPI写寄存器
////reg:指定的寄存器地址
////value:写入的值
////返回：读取到的值
//uint8_t MPU9250_Write_Reg(uint8_t reg,uint8_t value)
//{
//	uint8_t status;
//	MPU9250_CS=0;  //使能SPI传输
//	status=SPI1_ReadWriteByte(reg); //发送写命令+寄存器号
//	SPI1_ReadWriteByte(value);//写入寄存器值
//	MPU9250_CS=1;  //禁止MPU9250
//	return(status);//返回状态值
//}

////SPI读取寄存器
////reg:指定的寄存器地址
//uint8_t MPU9250_Read_Reg(uint8_t reg)
//{
//	uint8_t reg_val;
//	MPU9250_CS=0;  //使能SPI传输
//	SPI1_ReadWriteByte(reg|0x80); //发送读命令+寄存器号
//	reg_val=SPI1_ReadWriteByte(0xff);//读取寄存器值
//	MPU9250_CS=1;  //禁止MPU9250
//	return(reg_val);
//}

////SPIx 读写一个字节
////TxData:要写入的字节
////返回值:读取到的字节
//uint8_t SPI1_ReadWriteByte(uint8_t TxData)
//{		
//	uint8_t retry=0;				 	
//	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) //检查指定的SPI标志位设置与否:发送缓存空标志位
//		{
//		retry++;
//		if(retry>200)return 0;
//		}			  
//	SPI_I2S_SendData(SPI1, TxData); //通过外设SPIx发送一个数据
//	retry=0;

//	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET) //检查指定的SPI标志位设置与否:接受缓存非空标志位
//	//while((SPI2->SR &SPI_I2S_FLAG_RXNE) == RESET);	
//		{
//		retry++;
//		if(retry>200)return 0;
//		}	  						    
//	return SPI_I2S_ReceiveData(SPI1); //返回通过SPIx最近接收的数据					    
//}
