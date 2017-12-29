#include "mpu92xx.h"
#include "delay.h"
#include "led.h"

#define DEGTORAD  1878.4203821f//32.768*(180/3.14)
#define ONE_G     (8192)
#define MSB_MS2   (835.91836734f)//8192/9.8

const uint8_t CALLENTH = 100;
/*static*/ uint8_t MPU92XXDATACALFLAG=0,MPU92XXMAGCALFLAG=0;

signed short MPU9250_AK8963_ASA[3]={0};
MPU92XXRAWDATA Mpu92xxRawData={{0,0,0},
								  0,
							   {0,0,0},
							   {0,0,0}};

MPU92XXCALDATA Mpu92xxCalData={{0,0,0},
							   {0,0,0},
							   {0,0,0},
							   {0,0,0}};
MOVEFILTER AccMovefilter[3],GyroMovefilter[3],MagMovefilter[3];
							   
//MPU9250初始化·
unsigned char MPU9250_Init(void)
{
	uint8_t result=0;
	result = MPU9250_Read_Reg(MPU9250_ADDR,WHO_AM_I);
	if(result==0x73 || result==0x70)	//正确读取到9250的地址
	{
		MPU9250_Write_Reg(MPU9250_ADDR,PWR_MGMT_1,0X80);   //电源管理,复位MPU9250
		delay_ms(1);
		MPU9250_Write_Reg(MPU9250_ADDR,PWR_MGMT_1,0X01);   //选择时钟源
		MPU9250_Write_Reg(MPU9250_ADDR,PWR_MGMT_2,0X00);   //使能加速度计和陀螺仪
		MPU9250_Write_Reg(MPU9250_ADDR,CONFIG,0X02);	//低通滤波器 0x06 92hz (3.9ms delay) fs=1khz
		MPU9250_Write_Reg(MPU9250_ADDR,SMPLRT_DIV,0x00);//采样率1000/(1+0)=1000HZ
		MPU9250_Write_Reg(MPU9250_ADDR,GYRO_CONFIG,0X10);  //陀螺仪测量范围 0X10 正负1000度 32.768LSB/°
		MPU9250_Write_Reg(MPU9250_ADDR,ACCEL_CONFIG,0x08); //加速度计测量范围 0X08 正负4g
		MPU9250_Write_Reg(MPU9250_ADDR,ACCEL_CONFIG2,0x00);//加速度计速率1khz 滤波器460hz (1.94ms delay)
		MPU9250_Write_Reg(MPU9250_ADDR,MAG_INT_PIN_CFG,0x30); //有操作就清空中断标志
        MPU9250_Write_Reg(MPU9250_ADDR,MAG_I2C_MST_CTRL,0x4D); // I2C Speed 400 kHz
		MPU9250_Write_Reg(MPU9250_ADDR,USER_CTRL,0X30);  //使能MPU9250 AUX
		delay_ms(1);
		
		Init_AK8963();
		return 0;
	}
	return 1;
}

//配置AK8963的模式 连续测量模式2 （由于极限速度为138hz 这里使用mode2 可以达到100hz）
void Init_AK8963(void)
{
	uint8_t temp=0;
	uint8_t response[3]={0};
	delay_ms(500);
	MPU9250_Write_Reg(MAG_I2C_ADDR,MAG_I2C_SLV0_ADDR,0x0C);//write I2C addr 
	MPU9250_Write_Reg(MAG_I2C_ADDR,MAG_I2C_SLV0_DO,0x01);	//重启设备 
	MPU9250_Write_Reg(MAG_I2C_ADDR,MAG_I2C_SLV0_REG,AK8963_CNTL2);
	MPU9250_Write_Reg(MAG_I2C_ADDR,MAG_I2C_SLV0_CTRL,0X81);//写入一个数据
	
	delay_ms(100);
	
	//Set Slave to Read AK8963
	MPU9250_Write_Reg(MAG_I2C_ADDR,MAG_I2C_SLV0_ADDR,0x8c);
	MPU9250_Write_Reg(MAG_I2C_ADDR,MAG_I2C_SLV0_REG,0x00);
	MPU9250_Write_Reg(MAG_I2C_ADDR,MAG_I2C_SLV0_CTRL,0x81);
	delay_us(1000); //必须加延时 否则出错 最小600us （与I2C通信速度有关） 	可以使用I2C_SLV4 来解决
	temp = MPU9250_Read_Reg(MAG_I2C_ADDR,MAG_EXT_SENS_DATA_00);
		
	if(temp == 0x48)
	{
			//成功
			delay_ms(500);
	}
	else 
	{
			//失败
			while(1);
	}
	
	MPU9250_Write_Reg(MAG_I2C_ADDR,MAG_I2C_SLV0_ADDR,0x0C);//write I2C addr 
	MPU9250_Write_Reg(MAG_I2C_ADDR,MAG_I2C_SLV0_DO,0x16); //选择连续测量模式 16bit输出
	MPU9250_Write_Reg(MAG_I2C_ADDR,MAG_I2C_SLV0_REG,AK8963_CNTL1);
	MPU9250_Write_Reg(MAG_I2C_ADDR,MAG_I2C_SLV0_CTRL,0X81);//写入一个数据
	delay_us(1000);	//必须加延时 否则出错 最小200us （与I2C通信速度有关）
	MPU9250_Write_Reg(MAG_I2C_ADDR,MAG_I2C_SLV0_ADDR,0x8C);//read I2C addr 
	MPU9250_Write_Reg(MAG_I2C_ADDR,MAG_I2C_SLV0_REG,AK8963_CNTL1);
	MPU9250_Write_Reg(MAG_I2C_ADDR,MAG_I2C_SLV0_CTRL,0X81);//读取一个数据
	delay_us(1000);	//必须加延时 否则出错 最小200us （与I2C通信速度有关）
	temp = MPU9250_Read_Reg(MAG_I2C_ADDR,MAG_EXT_SENS_DATA_00);
	if(temp == 0x16)
	{
			//初始化成功
			delay_ms(500);
	}
	else 
	{
			//初始化失败
			while(1);
	}
	//AK8963 get calibration data
	MPU9250_Read_MultiBytes(MAG_I2C_ADDR, MPU9250_AK8963_ASAX, 3, response);
	//AK8963_SENSITIVITY_SCALE_FACTOR
	//AK8963_ASA[i++] = (s16)((data - 128.0f) / 256.0f + 1.0f) ;
	MPU9250_AK8963_ASA[0] = (s16)(response[0]) + 128;
	MPU9250_AK8963_ASA[1] = (s16)(response[1]) + 128;
	MPU9250_AK8963_ASA[2] = (s16)(response[2]) + 128;
	
	MPU9250_Write_Reg(MAG_I2C_ADDR,MAG_I2C_SLV0_ADDR, 0x8C);          // Set AK8963_I2C_ADDR = 7'b000_1100
	delay_us(1000);
	MPU9250_Write_Reg(MAG_I2C_ADDR,MAG_I2C_SLV0_REG, AK8963_ST1);     // Set Read Reg
	MPU9250_Write_Reg(MAG_I2C_ADDR,MAG_I2C_SLV0_CTRL, 0x88);          // Start Read, 6 bytes	ST1读到ST2
	delay_us(1000);
	
}


void MPU9250_ReadValue(void)
{
    uint8_t MPU9250_buf[22]={0};
	
	MPU9250_Read_MultiBytes(MPU9250_ADDR,ACCEL_XOUT_H,22,MPU9250_buf);
	//从加速度计的寄存器开始进行读取陀螺仪和加速度计的值
	
	Mpu92xxRawData.Rawacc.X = (((int16_t)MPU9250_buf[0]<<8) | MPU9250_buf[1])-Mpu92xxCalData.AccCal.X;
	Mpu92xxRawData.Rawacc.Y = (((int16_t)MPU9250_buf[2]<<8) | MPU9250_buf[3])-Mpu92xxCalData.AccCal.Y;
	Mpu92xxRawData.Rawacc.Z = (((int16_t)MPU9250_buf[4]<<8) | MPU9250_buf[5])+Mpu92xxCalData.AccCal.Z;
	Mpu92xxRawData.temp     = ((int16_t)MPU9250_buf[6]<<8) | MPU9250_buf[7];
	Mpu92xxRawData.Rawgyro.X = (((int16_t)MPU9250_buf[8]<<8)  | MPU9250_buf[9]) -Mpu92xxCalData.GyroCal.X;
	Mpu92xxRawData.Rawgyro.Y = (((int16_t)MPU9250_buf[10]<<8) | MPU9250_buf[11])-Mpu92xxCalData.GyroCal.Y;
	Mpu92xxRawData.Rawgyro.Z = (((int16_t)MPU9250_buf[12]<<8) | MPU9250_buf[13])-Mpu92xxCalData.GyroCal.Z;

	if (!(MPU9250_buf[14] & MPU9250_AK8963_DATA_READY) || (MPU9250_buf[14] & MPU9250_AK8963_DATA_OVERRUN))
	{
		return;
	}
	if (MPU9250_buf[21] & MPU9250_AK8963_OVERFLOW)
	{
		return;
	}
	Mpu92xxRawData.Rawmag.X = (MPU9250_buf[16] << 8) | MPU9250_buf[15];
	Mpu92xxRawData.Rawmag.Y = (MPU9250_buf[18] << 8) | MPU9250_buf[17];
	Mpu92xxRawData.Rawmag.Z = (MPU9250_buf[20] << 8) | MPU9250_buf[19];
	//ned x,y,z
	Mpu92xxRawData.Rawmag.X = (((long)Mpu92xxRawData.Rawmag.X * MPU9250_AK8963_ASA[0]) >> 8)-Mpu92xxCalData.MagOffset.X;
	Mpu92xxRawData.Rawmag.Y = (((long)Mpu92xxRawData.Rawmag.Y * MPU9250_AK8963_ASA[1]) >> 8)-Mpu92xxCalData.MagOffset.Y;
	Mpu92xxRawData.Rawmag.Z = (((long)Mpu92xxRawData.Rawmag.Z * MPU9250_AK8963_ASA[2]) >> 8)-Mpu92xxCalData.MagOffset.Z;
	//以上全部为未校准数据
	
	//更新最大最小值  磁力校准需要用到
	if(Mpu92xxCalData.MagCalMin.X > Mpu92xxRawData.Rawmag.X)
	{
		Mpu92xxCalData.MagCalMin.X = Mpu92xxRawData.Rawmag.X;
	}
	if(Mpu92xxCalData.MagCalMin.Y > Mpu92xxRawData.Rawmag.Y)
	{
		Mpu92xxCalData.MagCalMin.Y = Mpu92xxRawData.Rawmag.Y;
	}
	if(Mpu92xxCalData.MagCalMin.Z> Mpu92xxRawData.Rawmag.Z)
	{
		Mpu92xxCalData.MagCalMin.Z= Mpu92xxRawData.Rawmag.Z;
	}

	if(Mpu92xxCalData.MagCalMax.X < Mpu92xxRawData.Rawmag.X)
	{
		Mpu92xxCalData.MagCalMax.X = Mpu92xxRawData.Rawmag.X;		
	}
	if(Mpu92xxCalData.MagCalMax.Y < Mpu92xxRawData.Rawmag.Y)
	{
		Mpu92xxCalData.MagCalMax.Y = Mpu92xxRawData.Rawmag.Y;
	}
	if(Mpu92xxCalData.MagCalMax.Z < Mpu92xxRawData.Rawmag.Z)
	{
	   Mpu92xxCalData.MagCalMax.Z = Mpu92xxRawData.Rawmag.Z;
	}
}
/*
*滑动滤波
*datain      输入变量
**movefilter 待滤波变量结构体
*/
int16_t MoveFilter(int16_t datain,MOVEFILTER *movefilter)
{
	movefilter->rawsum -= movefilter->rawbuffer[movefilter->rawindex];//去掉tail
	movefilter->rawbuffer[movefilter->rawindex] = datain;
	movefilter->rawsum += movefilter->rawbuffer[movefilter->rawindex];//队列head
	
	if(++movefilter->rawindex>=FILTERLEN)
	{
		movefilter->rawindex=0;
		movefilter->firstfullflag=1;
	}
	if(movefilter->firstfullflag)
	{
		return movefilter->rawsum/FILTERLEN;
	}
	else
	{
		return movefilter->rawbuffer[movefilter->rawindex-1];
	}	
}

void MPU9250_DataSolve(float *gx,float *gy,float *gz,float *ax,float *ay,float *az,float *mx,float *my,float *mz)
{
  static uint8_t runcnt=0;
  static int32_t tempbuf[6]={0};
	
  MPU9250_ReadValue();
	
  if(!MPU92XXDATACALFLAG)//开机对陀螺仪加速度计数据校准
  {
	LED0_OFF();
	tempbuf[0] += Mpu92xxRawData.Rawacc.X;
	tempbuf[1] += Mpu92xxRawData.Rawacc.Y;
	tempbuf[2] += Mpu92xxRawData.Rawacc.Z;
	  
	tempbuf[3] += Mpu92xxRawData.Rawgyro.X;
	tempbuf[4] += Mpu92xxRawData.Rawgyro.Y;
	tempbuf[5] += Mpu92xxRawData.Rawgyro.Z;
	delay_ms(20);
	if(++runcnt>=CALLENTH)
	{
		Mpu92xxCalData.AccCal.X = tempbuf[0]/CALLENTH;
		Mpu92xxCalData.AccCal.Y = tempbuf[1]/CALLENTH;
		Mpu92xxCalData.AccCal.Z = ONE_G-tempbuf[2]/CALLENTH;
		
		Mpu92xxCalData.GyroCal.X = tempbuf[3]/CALLENTH;
		Mpu92xxCalData.GyroCal.Y = tempbuf[4]/CALLENTH;
		Mpu92xxCalData.GyroCal.Z = tempbuf[5]/CALLENTH;
		runcnt = 0;
		MPU92XXDATACALFLAG = 1;
		LED0_ON();
		tempbuf[0]=0;
		tempbuf[1]=0;
		tempbuf[2]=0;
		tempbuf[3]=0;
		tempbuf[4]=0;
		tempbuf[5]=0;
	}
  }
  
  if(MPU92XXMAGCALFLAG)//每隔一段时间重新校准磁力计
  {
	Mpu92xxCalData.MagOffset.X = (Mpu92xxCalData.MagCalMax.X - Mpu92xxCalData.MagCalMin.X)/2;
	Mpu92xxCalData.MagOffset.Y = (Mpu92xxCalData.MagCalMax.Y - Mpu92xxCalData.MagCalMin.Y)/2;
	Mpu92xxCalData.MagOffset.Z = (Mpu92xxCalData.MagCalMax.Z - Mpu92xxCalData.MagCalMin.Z)/2;
	MPU92XXMAGCALFLAG = 0; 
  }
  //滑动滤波
  *ax=MoveFilter(Mpu92xxRawData.Rawacc.X,AccMovefilter)/MSB_MS2;
  *ay=MoveFilter(Mpu92xxRawData.Rawacc.Y,AccMovefilter+1)/MSB_MS2;
  *az=MoveFilter(Mpu92xxRawData.Rawacc.Z,AccMovefilter+2)/MSB_MS2;
  
  *gx=MoveFilter(Mpu92xxRawData.Rawgyro.X,GyroMovefilter)/DEGTORAD;//rad/s
  *gy=MoveFilter(Mpu92xxRawData.Rawgyro.Y,GyroMovefilter+1)/DEGTORAD;
  *gz=MoveFilter(Mpu92xxRawData.Rawgyro.Z,GyroMovefilter+2)/DEGTORAD;
  
  *mx=MoveFilter(Mpu92xxRawData.Rawmag.X,MagMovefilter);
  *my=MoveFilter(Mpu92xxRawData.Rawmag.Y,MagMovefilter+1);
  *mz=MoveFilter(Mpu92xxRawData.Rawmag.Z,MagMovefilter+2);   
}
