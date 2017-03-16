/***********************************************************************************************************************
---Copyright(C)DXY 2015-2020                   
---
---All rights reserved	               www.DXYtech.com
----------------------------------------------------------------[文件信息]---
---文 件 名: FBM320.c
---功能描述: FBM320气压计驱动
---版本信息: 
---函数统计: 
---创建信息: Awen 2016.2.20
---备    注: 
----------------------------------------------------------------[修改记录]---
---修改人/时间:
---发布版本:
---修改内容:
**************************************************************************************/
#include <math.h>
#include "FBM320.H"
//#include "myiic.h"
#include "delay.h"
#include "bmp180.h"

//---------变量定义----------------
int32_t UP_S=0, UT_S=0, RP_S=0, RT_S=0, OffP_S=0;

int32_t UP_I=0,OffUP_I=0;//分别是未经补偿的气压值以及初始化offset未经补偿气压值。
int32_t RP_I=0, OffP_I=0;//分别是补偿之后到气压值以及初始化补偿之后到气压值。

int32_t UT_I=0,OffUT_I=0;//分别是未经补偿的温度值以及初始化offset未经补偿的温度值。
int32_t RT_I=0,OffT_I=0; //分别是补偿之后的温度值以及初始化补偿之后的温度值。

float H_S=0, H_I=0;		//计算相对海拔高度
int32_t H_absI = 0 , H_Icm;	//海拔绝对值

//校准系数
uint16_t C0_S, C1_S, C2_S, C3_S, C6_S, C8_S, C9_S, C10_S, C11_S, C12_S; 
uint32_t C4_S, C5_S, C7_S;
uint16_t C0_I, C1_I, C2_I, C3_I, C6_I, C8_I, C9_I, C10_I, C11_I, C12_I; 
uint32_t C4_I, C5_I, C7_I;
uint8_t BusDetect=0, Formula_Select=0;

float ft_alt1=0,ft_altspeed=0;
float ft_alt2;


/*******************************************************************************
---函数名：  FBM320_Read_I2C
---功能：    I2C寄存器读取函数
---入口参数：
---返回参数：
---作者：    Awen
*************************************************/
uint8_t FBM320_Read_I2C(uint8_t ReadAddr)
{
  uint8_t temp=0;
  temp = _BMP180_ReadOneByte(FBM320_I2ADDR , ReadAddr);
  return temp;
}

/*******************************************************************************
---函数名：  FBM320_Write_I2C
---功能：    I2C单字节数据写入函数
---入口参数：
---返回参数：
---作者：    Awen
*************************************************/
uint8_t FBM320_Write_I2C(uint8_t WriteAddr,uint8_t DataToWrite)
{
  _BMP180_WriteOneByte(FBM320_I2ADDR, WriteAddr,DataToWrite);
  return 0;
}

//读取系数R0-R9，计算参数:  第一步
/*******************************************************************************
---函数名：  FBM320_Write_I2C
---功能：    I2C单字节数据写入函数
---入口参数：
---返回参数：
---作者：    Awen
*************************************************/
void Coefficient(uint8_t BusType)//Receive Calibrate Coefficient
{
	uint8_t i;
	uint16_t R[10];
	uint16_t C0=0, C1=0, C2=0, C3=0, C6=0, C8=0, C9=0, C10=0, C11=0, C12=0; 
	uint32_t C4=0, C5=0, C7=0;

	for(i=0; i<9; i++)
	R[i]=((uint16_t)FBM320_Read_I2C(0xAA + (i*2))<<8) | FBM320_Read_I2C(0xAB + (i*2));
	R[9]=((uint16_t)FBM320_Read_I2C(0xA4)<<8) | FBM320_Read_I2C(0xF1);

	if((Formula_Select & 0x0F) == 0x01)//B2版本气压计
	{
		C0 = R[0] >> 4;
		C1 = ((R[1] & 0xFF00) >> 5) | (R[2] & 7);
		C2 = ((R[1] & 0xFF) << 1) | (R[4] & 1);
		C3 = R[2] >> 3;
		C4 = ((uint32_t)R[3] << 2) | (R[0] & 3);
		C5 = R[4] >> 1;
		C6 = R[5] >> 3;
		C7 = ((uint32_t)R[6] << 3) | (R[5] & 7);
		C8 = R[7] >> 3;
		C9 = R[8] >> 2;
		C10 = ((R[9] & 0xFF00) >> 6) | (R[8] & 3);
		C11 = R[9] & 0xFF;
		C12 = ((R[0] & 0x0C) << 1) | (R[7] & 7);
	}
	else
	{
		C0 = R[0] >> 4;
		C1 = ((R[1] & 0xFF00) >> 5) | (R[2] & 7);
		C2 = ((R[1] & 0xFF) << 1) | (R[4] & 1);
		C3 = R[2] >> 3;
		C4 = ((uint32_t)R[3] << 1) | (R[5] & 1);
		C5 = R[4] >> 1;
		C6 = R[5] >> 3;
		C7 = ((uint32_t)R[6] << 2) | ((R[0] >> 2) & 3);
		C8 = R[7] >> 3;
		C9 = R[8] >> 2;
		C10 = ((R[9] & 0xFF00) >> 6) | (R[8] & 3);
		C11 = R[9] & 0xFF;
		C12 = ((R[5] & 6) << 2) | (R[7] & 7);
	}

	if((BusType & 0xF0) == 0x10)
	{
		C0_S = C0;
		C1_S = C1;
		C2_S = C2;
		C3_S = C3;
		C4_S = C4;
		C5_S = C5;
		C6_S = C6;
		C7_S = C7;
		C8_S = C8;
		C9_S = C9;
		C10_S = C10;
		C11_S = C11;
		C12_S = C12;
	}
	else if((BusType & 0x0F) == 0x01)
	{
		C0_I = C0;
		C1_I = C1;
		C2_I = C2;
		C3_I = C3;
		C4_I = C4;
		C5_I = C5;
		C6_I = C6;
		C7_I = C7;
		C8_I = C8;
		C9_I = C9;
		C10_I = C10;
		C11_I = C11;
		C12_I = C12;
	}	
}


/*******************************************************************************
---函数名：  FBM320_Write_I2C
---功能：    读取传感器中的温度和气压值
---入口参数：
---返回参数：
---作者：    Awen
*************************************************/
uint32_t FBM320_ReadADC_I2C()
{
   uint8_t Msb,Csb,Lsb=0;
   Msb = FBM320_Read_I2C(0xF6);
   Csb = FBM320_Read_I2C(0xF7);
   Lsb = FBM320_Read_I2C(0xF8);
    
   return ((((uint32_t)Msb)<<16) + (((uint32_t)Csb)<<8) + Lsb);
}


/*******************************************************************************
---函数名：  FBM320_Write_I2C
---功能：    计算得到结果气压值和温度值
---入口参数：
---返回参数：
---作者：    Awen
*************************************************/
void Calculate(uint8_t BusType, int32_t UP, int32_t UT)		//Calculate Real Pressure & Temperautre
{
	int8_t C12=0;
	int16_t C0=0, C2=0, C3=0, C6=0, C8=0, C9=0, C10=0, C11=0; 
	int32_t C1=0, C4=0, C5=0, C7=0;
	int32_t RP=0, RT=0;
	int32_t DT, DT2, X01, X02, X03, X11, X12, X13, X21, X22, X23, X24, X25, X26, X31, X32, CF, PP1, PP2, PP3, PP4;

	if((BusType & 0xF0) == 0x10)
	{
		C0 = C0_S;
		C1 = C1_S;
		C2 = C2_S;
		C3 = C3_S;
		C4 = C4_S;
		C5 = C5_S;
		C6 = C6_S;
		C7 = C7_S;
		C8 = C8_S;
		C9 = C9_S;
		C10 = C10_S;
		C11 = C11_S;
		C12 = C12_S;
	}
	else if((BusType & 0x0F) == 0x01)
	{
		C0 = C0_I;
		C1 = C1_I;
		C2 = C2_I;
		C3 = C3_I;
		C4 = C4_I;
		C5 = C5_I;
		C6 = C6_I;
		C7 = C7_I;
		C8 = C8_I;
		C9 = C9_I;
		C10 = C10_I;
		C11 = C11_I;
		C12 = C12_I;
	}	
	
	if((Formula_Select & 0x0F) == 0x01)			//For FBM320-B2
	{
		DT	=	((UT - 8388608) >> 4) + (C0 << 4);
		X01	=	(C1 + 4459) * DT >> 1;
		X02	=	((((C2 - 256) * DT) >> 14) * DT) >> 4;
		X03	=	(((((C3 * DT) >> 18) * DT) >> 18) * DT);
		RT	=	((2500 << 15) - X01 - X02 - X03) >> 15;
					
		DT2	=	(X01 + X02 + X03) >> 12;
					
		X11	=	((C5 - 4443) * DT2);
		X12	=	(((C6 * DT2) >> 16) * DT2) >> 2;
		X13	=	((X11 + X12) >> 10) + ((C4 + 120586) << 4);
					
		X21	=	((C8 + 7180) * DT2) >> 10;
		X22	=	(((C9 * DT2) >> 17) * DT2) >> 12;
		if(X22 >= X21)
			X23	=	X22 - X21;
		else
			X23	=	X21 - X22;
		X24	=	(X23 >> 11) * (C7 + 166426);
		X25	=	((X23 & 0x7FF) * (C7 + 166426)) >> 11;
		if((X22 - X21) < 0)
			X26	=	((0 - X24 - X25) >> 11) + C7 + 166426;
		else	
			X26	=	((X24 + X25) >> 11) + C7 + 166426;
					
		PP1	=	((UP - 8388608) - X13) >> 3;
		PP2	=	(X26 >> 11) * PP1;
		PP3	=	((X26 & 0x7FF) * PP1) >> 11;
		PP4	=	(PP2 + PP3) >> 10;
					
		CF	=	(2097152 + C12 * DT2) >> 3;
		X31	=	(((CF * C10) >> 17) * PP4) >> 2;
		X32	=	(((((CF * C11) >> 15) * PP4) >> 18) * PP4);
		RP	=	((X31 + X32) >> 15) + PP4 + 99880;
	}
	else			//For FBM320
	{
		DT	=	((UT - 8388608) >> 4) + (C0 << 4);
		X01	=	(C1 + 4418) * DT >> 1;
		X02	=	((((C2 - 256) * DT) >> 14) * DT) >> 4;
		X03	=	(((((C3 * DT) >> 18) * DT) >> 18) * DT);
		RT = ((2500 << 15) - X01 - X02 - X03) >> 15;
				
		DT2	=	(X01 + X02 + X03) >>12;
				
		X11	=	(C5 * DT2);
		X12	=	(((C6 * DT2) >> 16) * DT2) >> 2;
		X13	=	((X11 + X12) >> 10) + ((C4 + 211288) << 4);
				
		X21	=	((C8 + 7209) * DT2) >> 10;
		X22	=	(((C9 * DT2) >> 17) * DT2) >> 12;
		if(X22 >= X21)
			X23	=	X22 - X21;
		else
			X23	=	X21 - X22;
		X24	=	(X23 >> 11) * (C7 + 285594);
		X25	=	((X23 & 0x7FF) * (C7 + 285594)) >> 11;
		if((X22 - X21) < 0) 
			X26	=	((0 - X24 - X25) >> 11) + C7 + 285594;
		else
			X26	=	((X24 + X25) >> 11) + C7 + 285594;
		PP1	=	((UP - 8388608) - X13) >> 3;
		PP2	=	(X26 >> 11) * PP1;
		PP3	=	((X26 & 0x7FF) * PP1) >> 11;
		PP4	=	(PP2 + PP3) >> 10;
				
		CF	=	(2097152 + C12 * DT2) >> 3;
		X31	=	(((CF * C10) >> 17) * PP4) >> 2;
		X32	=	(((((CF * C11) >> 15) * PP4) >> 18) * PP4);
		RP = ((X31 + X32) >> 15) + PP4 + 99880;
	}
	
	if((BusType & 0xF0) == 0x10)
	{
		RP_S = RP;
		RT_S = RT;
	}
	else if((BusType & 0x0F) == 0x01)
	{
		RP_I = RP;
		RT_I = RT;
	}
}

void FBM320Calculate(uint8_t BusType, int32_t UP, int32_t UT)
{
  static uint8_t fbmruncnt=0,fbmcalflag=0;
  static uint64_t fbmpressum=0;
	
  Calculate(BusType, UP, UT);
	
  if(!fbmcalflag)
  {
	fbmruncnt++;
	if(10<fbmruncnt && fbmruncnt<=40)//前10个数据丢弃
	{
		fbmpressum += RP_I;
		OffP_I = RP_I;
	}
	else if(fbmruncnt>40)
	{
		OffP_I=fbmpressum/30;//更新press offset
		fbmcalflag=1;
	}
  }
}
/*******************************************************************************
---函数名：  FBM320Initial
---功能：    初始化接口
---入口参数：
---返回参数：
---作者：    Awen
*************************************************/
void FBM320Initial(void)//Initial FBM320 SPI or I2C Bus Protocol
{
//  IIC1_Init();
	_BMP180_IIC_Init();
  if(FBM320_Read_I2C(0x6B) == 0x42)
  {
	BusDetect |= 0x01;

#ifdef  SET_FORMULA_VER	
    Formula_Select = FORMULA_VER_B2;    //执行旧版本吧
#else
	if((FBM320_Read_I2C(0xF4) & 0x40) == 0x40)
	Formula_Select |= 0x01; //执行新版本  B2
#endif
		
	Coefficient(BusDetect);			//获取系数和参数
		
	FBM320_Write_I2C(0xF4, READ_T_REG);
	delay_ms(T_DELAY_MS);
	UT_I = FBM320_ReadADC_I2C();    //读3字节传感器数据，温度信息
	OffUT_I = UT_I;//原始温度
	FBM320_Write_I2C(0xF4,READ_P_REG);// 0xF4);		
	delay_ms(P_DELAY_MS);
    //delay_ms(P_DELAY_MS); //等12-33.6ms
	UP_I = FBM320_ReadADC_I2C();    //读气压值
	OffUP_I = UP_I;//
	Calculate(BusDetect, UP_I, UT_I);   //计算参数
//	OffP_I = UP_I;
	OffT_I = RT_I;//offset t after compensation
	OffP_I = RP_I;//offset_p after compensation
  }
}


/*******************************************************************************
---函数名：  FBM320_Write_I2C
---功能：    计算相对高度，气压Pa，对应结果单位是米
---入口参数：
---返回参数：
---作者：    Awen
*************************************************/
float Rel_Altitude(long Press, long Ref_P)								//Calculate Relative Altitude
{
  return 44330 * (1 - pow((double)((float)Press / (float)Ref_P), (double)(1/5.255)));
}

/*******************************************************************************
---函数名：  FBM320_Write_I2C
---功能：    计算绝对海拔，气压Pa，对应结果单位是毫米
---入口参数：
---返回参数：
---作者：    Awen
*************************************************/
int32_t Abs_Altitude(int32_t Press)
{
	int8_t P0;			
	int16_t hs1, dP0;			
	int32_t h0, hs0, HP1, HP2;			
					
	if(Press >= 103000)
	{	
		P0	=	103;
		h0	=	-138507;
		hs0	=	-21007;
		hs1	=	311;
	}	
	else if(Press >= 98000)
	{	
		P0	=	98;
		h0	=	280531;
		hs0	=	-21869;
		hs1	=	338;
	}	
	else if(Press >= 93000)
	{	
		P0	=	93;
		h0	=	717253;
		hs0	=	-22813;
		hs1	=	370;
	}				
	else if(Press >= 88000)
	{	
		P0	=	88;
		h0	=	1173421;
		hs0	=	-23854;
		hs1	=	407;
	}	
	else if(Press >= 83000)
	{	
		P0	=	83;
		h0	=	1651084;
		hs0	=	-25007;
		hs1	=	450;
	}	
	else if(Press >= 78000)
	{	
		P0	=	78;
		h0	=	2152645;
		hs0	=	-26292;
		hs1	=	501;
	}	
	else if(Press >= 73000)
	{	
		P0	=	73;
		h0	=	2680954;
		hs0	=	-27735;
		hs1	=	560;
	}	
	else if(Press >= 68000)
	{	
		P0	=	68;
		h0	=	3239426;
		hs0	=	-29366;
		hs1	=	632;
	}	
	else if(Press >= 63000)
	{	
		P0	=	63;
		h0	=	3832204;
		hs0	=	-31229;
		hs1	=	719;
	}	
	else if(Press >= 58000)
	{	
		P0	=	58;
		h0	=	4464387;
		hs0	=	-33377;
		hs1	=	826;
	}	
	else if(Press >= 53000)
	{	
		P0	=	53;
		h0	=	5142359;
		hs0	=	-35885;
		hs1	=	960;
	}		
	else if(Press >= 48000)
	{
		P0	=	48;
		h0	=	5874268;
		hs0	=	-38855;
		hs1	=	1131;
	}	
	else if(Press >= 43000)
	{	
		P0	=	43;
		h0	=	6670762;
		hs0	=	-42434;
		hs1	=	1354;
	}	
	else if(Press >= 38000)
	{	
		P0	=	38;
		h0	=	7546157;
		hs0	=	-46841;
		hs1	=	1654;
	}	
	else if(Press >= 33000)
	{	
		P0	=	33;
		h0	=	8520395;
		hs0	=	-52412;
		hs1	=	2072;
	}	
	else
	{	
		P0	=	28;
		h0	=	9622536;
		hs0	=	-59704;
		hs1	=	2682;
	}
					
	dP0	=	Press - P0 * 1000;
				
	HP1	=	(hs0 * dP0) >> 2;
	HP2	=	(((hs1 * dP0) >> 10)* dP0) >> 4;			

	return	((h0 << 6) + HP1 + HP2) >> 6;
}

/*******************************************************************************
---函数名：  testFBM320
---功能：    启动测试
---入口参数：
---返回参数：
---作者：    Awen
*************************************************/
void testFBM320(void)
{
    FBM320Initial();

    FBM320_Write_I2C(0xF4, READ_T_REG);//0x2E);
    delay_ms(T_DELAY_MS);
    UT_I = FBM320_ReadADC_I2C();
    FBM320_Write_I2C(0xF4, READ_P_REG);//0xF4);  
    delay_ms(P_DELAY_MS);
    UP_I = FBM320_ReadADC_I2C();
    Calculate(BusDetect, UP_I, UT_I);

    H_I = Rel_Altitude(RP_I, OffP_I);
    H_absI = Abs_Altitude(RP_I);

}
/******************************************************************************/
#define ALT_FILTERLEN 10
static float   Alt_rawbuffer[ALT_FILTERLEN]={0};
static uint8_t Alt_rawindex=0,Altfirstfullflag=0;//队列索引
static double   Alt_rawsum=0;

//添加一个新的值到FBM320高度队列 进行滤波
float FBM320_RawAltFilter(float val)
{
	Alt_rawsum -= Alt_rawbuffer[Alt_rawindex];//去掉tail
	Alt_rawbuffer[Alt_rawindex] = val;
	Alt_rawsum += Alt_rawbuffer[Alt_rawindex];//队列head
	
	if(++Alt_rawindex>=ALT_FILTERLEN)
	{
		Alt_rawindex=0;
		Altfirstfullflag=1;
	}

	if(Altfirstfullflag)
	{
		return Alt_rawsum/ALT_FILTERLEN;
	}
	else
	{
		return Alt_rawbuffer[Alt_rawindex-1];
	}
}

#define TEM_FILTERLEN 10
static uint32_t   Tem_rawbuffer[TEM_FILTERLEN]={0};
static uint8_t Tem_rawindex=0,Temfirstfullflag=0;//队列索引
static uint64_t   Tem_rawsum=0;
//添加一个新的值到FBM320温度队列进行滤波
int32_t FBM320_RawTemFilter(int32_t val)
{
	Tem_rawsum -= Tem_rawbuffer[Tem_rawindex];//去掉tail
	Tem_rawbuffer[Tem_rawindex] = val;
	Tem_rawsum += Tem_rawbuffer[Tem_rawindex];//队列head
	
	if(++Tem_rawindex>=TEM_FILTERLEN)
	{
		Tem_rawindex=0;
		Temfirstfullflag=1;
	}

	if(Temfirstfullflag)
	{
		return Tem_rawsum/TEM_FILTERLEN;
	}
	else
	{
		return Tem_rawbuffer[Tem_rawindex-1];
	}
}
//添加一个新的值到FBM320气压队列进行滤波
#define Pres_FILTERLEN 10
static uint32_t   Pres_rawbuffer[Pres_FILTERLEN]={0};
static uint8_t Pres_rawindex=0,Presfirstfullflag=0;//队列索引
static uint64_t   Pres_rawsum=0;
//添加一个新的值到FBM320温度队列进行滤波
int32_t FBM320_RawPresFilter(int32_t val)
{
	Pres_rawsum -= Pres_rawbuffer[Pres_rawindex];//去掉tail
	Pres_rawbuffer[Pres_rawindex] = val;
	Pres_rawsum += Pres_rawbuffer[Pres_rawindex];//队列head
	
	if(++Pres_rawindex>=Pres_FILTERLEN)
	{
		Pres_rawindex=0;
		Presfirstfullflag=1;
	}

	if(Presfirstfullflag)
	{
		return Pres_rawsum/Pres_FILTERLEN;
	}
	else
	{
		return Pres_rawbuffer[Pres_rawindex-1];
	}
}
/*******************************************************************************
---函数名：  updateVal
---功能：    读取当前传感器数据:  温度和气压
---入口参数：
---返回参数：
---作者：    Awen
*************************************************/
void updateVal( void *pvParameters )
{
  while(1)
  {

    FBM320_Write_I2C(0xF4, READ_T_REG);
    delay_ms(T_DELAY_US); //等2.5ms
//    vTaskDelay( 3 / portTICK_PERIOD_MS );
    UT_I = FBM320_ReadADC_I2C();	//获取温度值

	  UT_I = FBM320_RawTemFilter(UT_I);//filter

    FBM320_Write_I2C(0xF4, READ_P_REG);
    delay_ms(P_DELAY_MS); //等30ms
//    vTaskDelay( 30 / portTICK_PERIOD_MS );
    UP_I = FBM320_ReadADC_I2C();	//获取气压值

	UP_I = FBM320_RawPresFilter(UP_I);//filter

//    Calculate(BusDetect, UP_I, UT_I);	//矫正温度和气压值
	FBM320Calculate(BusDetect, UP_I, UT_I);//add
    //RP_I为经过校准补偿之后到气压 RT_I为经过校准补偿之后到温度
	//Off_P为初始化时到气压
    H_I = Rel_Altitude(RP_I, OffP_I);	//计算相对高度，单位米
//    H_absI = Abs_Altitude(RP_I);// 	//计算海拔高度，单位毫米

    ft_alt1 = FBM320_RawAltFilter(H_I);//高度滤波输出
//	sys.state.air_altitude = ft_alt1;//
	ft_altspeed = (ft_alt1 - ft_alt2) * 100;//cm/s速度输出
	ft_alt2 = ft_alt1;

//    ft_alt2 = H_absI;
  }
}


/*******************************************************************************
---函数名：  val2Char
---功能：    数据值转换为字符串存入Buffer，该函数功能将被替换
---入口参数：
---返回参数：
---作者：    Awen
*************************************************/
const char Nx[10] = {'0','1','2','3','4','5','6','7','8','9'};
#define INDEX_OFF  48
void val2Char(uint8_t * buff,uint32_t val, uint8_t len)
{
	if(4==len)		//温度
	{
		buff[0] = ' ';
		buff[1] = (val/1000) + INDEX_OFF;
		buff[2] = (val%1000)/100;
		buff[2] += INDEX_OFF;
		buff[3] = '.';
		buff[4] = (val%100)/10;
		buff[4] += INDEX_OFF;
		buff[5] = val%10;
		buff[5] += INDEX_OFF;
	}
	else
	{
		uint8_t i = 0;
		buff[0] = val/100000;
		buff[1] = (val%100000)/10000;
		buff[2] = (val%10000)/1000;
		buff[3] = (val%1000)/100;
		buff[4] = (val%100)/10;
		buff[5] = val%10;
		for(i=0;i<6;i++)
			buff[i] += INDEX_OFF;
		
	}
	buff[6] = 0;
}


