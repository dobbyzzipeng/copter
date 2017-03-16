#include "HMC5883L.h"

#define AngleFactor 57.30f//180/3.1415926

MagMaxMinData_t MagMaxMinData;
signed short HMC5883_FIFO[3][11];

/*********HMC5883L_Init**********/
void HMC5883L_Init(void)
{
    uint8_t tmp_ch = 0;
    ReadmultiyBytes(HMC5883L_SlaveAddr, HMC58X3_R_IDA,1, &tmp_ch);
    if(tmp_ch != HMC5883_DEVICE_ID_A)//check id if is ok
    {
       printf("error 2A\r\n");
    }

    WriteOneByte(HMC5883L_SlaveAddr, HMC58X3_R_CONFA,0x70);
    delay_ms(5);
    WriteOneByte(HMC5883L_SlaveAddr, HMC58X3_R_CONFB,0xA0);
    delay_ms(5);
    WriteOneByte(HMC5883L_SlaveAddr, HMC58X3_R_MODE,0x00);    //这里初始化为0x00 连续模式
    //wait the response of the hmc5883 stabalizes, 6 milliseconds  
    delay_ms(6);
    //set DOR
    WriteOneByte(HMC5883L_SlaveAddr, HMC58X3_R_CONFA,6<<2);   //75HZ更新
    delay_ms(6);
}
 /***************Multiple_Read_HMC5883L*******************/
void Multiple_ReadCal_HMC5883L(unsigned char buf[])
{
	uint8_t i = 0;
	int32_t sum=0;
	signed short x=0,y=0,z=0;
	
  ReadmultiyBytes(HMC5883L_SlaveAddr,DataRegisterBegin,6,buf);
  x=((signed short)buf[0] << 8) | buf[1];
  y=((signed short)buf[2] << 8) | buf[3];
  z=((signed short)buf[4] << 8) | buf[5];
	
	for(i=1;i<10;i++)
	{
		HMC5883_FIFO[0][i-1]=HMC5883_FIFO[0][i];
		HMC5883_FIFO[1][i-1]=HMC5883_FIFO[1][i];
		HMC5883_FIFO[2][i-1]=HMC5883_FIFO[2][i];
	}
	HMC5883_FIFO[0][9]= x;//将新的数据放置到 数据的最后面
	HMC5883_FIFO[1][9]= y;
	HMC5883_FIFO[2][9]= z;
	
	for(i=0;i<10;i++)//求当前数组的合，再取平均值
	{	
		 sum+=HMC5883_FIFO[0][i];
	}
	HMC5883_FIFO[0][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++)
	{
		 sum+=HMC5883_FIFO[1][i];
	}
	HMC5883_FIFO[1][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++)
	{
		 sum+=HMC5883_FIFO[2][i];
	}
	HMC5883_FIFO[2][10]=sum/10;
	//以上全部为未校准数据
	if(MagMaxMinData.MinMagX>HMC5883_FIFO[0][10])
	{
		MagMaxMinData.MinMagX=(int16_t)HMC5883_FIFO[0][10];
	}
	if(MagMaxMinData.MinMagY>HMC5883_FIFO[1][10])
	{
		MagMaxMinData.MinMagY=(int16_t)HMC5883_FIFO[1][10];
	}
	if(MagMaxMinData.MinMagZ>HMC5883_FIFO[2][10])
	{
		MagMaxMinData.MinMagZ=(int16_t)HMC5883_FIFO[2][10];
	}

	if(MagMaxMinData.MaxMagX<HMC5883_FIFO[0][10])
	{
		MagMaxMinData.MaxMagX=(int16_t)HMC5883_FIFO[0][10];		
	}
	if(MagMaxMinData.MaxMagY<HMC5883_FIFO[1][10])
	{
		MagMaxMinData.MaxMagY = HMC5883_FIFO[1][10];
	}
	if(MagMaxMinData.MaxMagZ<HMC5883_FIFO[2][10])
	{
		MagMaxMinData.MaxMagZ=(int16_t)HMC5883_FIFO[2][10];
	}			
}
/**********HMC5883L_EstimatedYawGet**********/
float HMC5883L_EstimatedYawGet(void)
{
	unsigned char HmcTemp[6]={0};
	signed short mx=0,my=0,mz=0;
	Multiple_ReadCal_HMC5883L(HmcTemp);
	HMC58X3_getlastValues(&mx,&my,&mz);
	
  return atan2((double)my,(double)mx) * AngleFactor + 180; // angle in degrees
//	angle*=1;
}

/**************************实现函数********************************************
*函数原型:	  void HMC58X3_getValues(int16_t *x,int16_t *y,int16_t *z)
*功　　能:	   读取 磁力计的当前ADC值
输入参数：    三个轴对应的输出指针	
输出参数：  无
*******************************************************************************/
void HMC58X3_getlastValues(int16_t *x,int16_t *y,int16_t *z) 
{
    *x = HMC5883_FIFO[0][10];
    *y = HMC5883_FIFO[1][10]; 
    *z = HMC5883_FIFO[2][10]; 
}

