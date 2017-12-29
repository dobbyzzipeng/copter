/***********************************************************************************************************************
---Copyright(C)DXY 2015-2020      深圳市鼎芯无限科技有限公司                     
---
---All rights reserved	               www.DXYtech.com
----------------------------------------------------------------[文件信息]---
---文 件 名: FBM320.h
---功能描述: FBM320驱动文件
---版本信息: 
---函数统计: 
---创建信息: Awen 2016.2.20
---备    注: 
----------------------------------------------------------------[修改记录]---
---修改人/时间:
---发布版本:
---修改内容:
**************************************************************************************/
#ifndef _FBM320_H_
#define _FBM320_H_
#include "stm32f10x.h"
#include <stdio.h>
#include <string.h>

//-----------I2C通信延时参数
#define T_DELAY_MS   3// 3      //2.2 -  2.5  ; 4
#define T_DELAY_US   2500// 3      //2.2 -  2.5  ; 4
#define P_DELAY_MS   8// 8      //2.2 -  10.7   ;30

//----------FBM320  I2C  地址
#define FBM320_I2ADDR_B0H   0xDA    //I2C Address悬空或拉高的地址，默认
#define FBM320_I2ADDR_B0L   0xD8    //I2C Address接地的地址

#if 1//FBM320_XIAOBAN_TEST
#define FBM320_I2ADDR   FBM320_I2ADDR_B0L   //外挂FBM320小板ADDR下拉10K到地
#else
#define FBM320_I2ADDR   FBM320_I2ADDR_B0H   //板载FBM320模块ADDR上拉
#endif

//----------解析度模式  
#define MODE_0   0x00
#define MODE_1   0x40
#define MODE_2   0x80
#define MODE_3   0xC0
#define MODE_CFG   MODE_0

//----------温度/ 压力读取模式
#define T_REG   0x2E
#define P_REG   0x34

#define READ_T_REG  (MODE_0|T_REG)
#define READ_P_REG  (MODE_3|P_REG)

#define SET_FORMULA_VER        
#define FORMULA_VER_B1         00   //旧版本代码
#define FORMULA_VER_B2         01   //新版本代码

extern int32_t UP_S, UT_S, RP_S, RT_S, OffP_S;
extern int32_t UP_I, UT_I, RP_I, RT_I, OffP_I;
extern float H_S, H_I;	//计算相对高度，单位米
extern int32_t H_absI, H_Icm; //海拔绝对值

extern uint16_t C0_S, C1_S, C2_S, C3_S, C6_S, C8_S, C9_S, C10_S, C11_S, C12_S; 
extern uint32_t C4_S, C5_S, C7_S;
extern uint16_t C0_I, C1_I, C2_I, C3_I, C6_I, C8_I, C9_I, C10_I, C11_I, C12_I; 
extern uint32_t C4_I, C5_I, C7_I;

extern uint8_t BusDetect, Formula_Select;



//--------------------------------------------函数----------------------
uint8_t FBM320_Read_I2C(uint8_t ReadAddr);

void FBM320Initial(void);
void Coefficient(uint8_t BusType);
void Calculate(uint8_t BusType, int32_t UP, int32_t UT);
void FBM320Calculate(uint8_t BusType, int32_t UP, int32_t UT);//add
float Rel_Altitude(long Press, long Ref_P);
int32_t Abs_Altitude(int32_t Press);

void testFBM320(void);
void updateVal( void *pvParameters );
void val2Char(uint8_t * buff,uint32_t val, uint8_t len);

float FBM320_RawAltFilter(float val);
int32_t FBM320_RawTemFilter(int32_t val);
int32_t FBM320_RawPresFilter(int32_t val);

//澄海林工
void BaroInit(void);
void BaroHandler(void);

#endif
