/*
*Corporation:	www.Makeblock.com
*Project:		Airblock
*brief:			Airmouse 
*Device:		STM32F103RCT6 
*MODULE:		MPU6050
*INTERFACE:     IIC
*IO:			SCL->PB6  SDA->PB7 INT->PB3
*				LED1->PA8
*zzp@2016-11-24
*/
//��������1ms�����ŷ����(��)           Eulla.pitch Eulla.roll Eulla.yaw(pitch roll����ϵͳ����2.5s֮���ȥ��ǰֵ,���¾���)
//             �Լ�������ٶȣ�rad/s�� velocity.X velocity.Y velocity.Z
//             ������ٶ� (m/s^2)      acc.x acc.y acc.z
//             �����ٶ� (m/s)          INS_Estimate.Z.InsVel INS_Estimate.Y.InsVel INS_Estimate.X.InsVel(X Y���������С�Ƕȷ�Χ��)
//             ����λ�� (m)            INS_Estimate.Z.InsPos INS_Estimate.Y.InsPos INS_Estimate.Z.InsPos(X Y���������С�Ƕȷ�Χ��,����Ư�ƱȽϴ󣬲�����)
//��̬�����㷨����ѡ����ڱ���PI�������Ļ����˲��㷨�����ݶ��½��㷨������ԭ���޸���־��
//ͨ��globaldefine.h   #define USE_Mahony_ALGORITHM     0//PI���λ����˲�
//                     #define USE_Madgwick_ALGORITHM   1//�ݶ��½��㷨
//ѡ���л�
#include "main.h"	


int main()
{
	Bsp_Init();
	Sys_Timer_RUN();
	while(1)
	{
		 if(Flag.system_usarttx_flag)//
		{
			Flag.system_usarttx_flag=0;	
			
//			    ANO_ImuDataReturn(&Eulla.pitch,&Eulla.roll,&Eulla.yaw,
//						&Interial.velocity,&Interial.position,&PRESSURE,&TEMPERATURE,&ALTitude,
//						MotorOut[0]*_FLY_MAX_OUT,MotorOut[1]*_FLY_MAX_OUT);

//				ANO_ImuDataReturn(&Eulla.pitch,&Eulla.roll,&Eulla.yaw,
//						&Interial.velocity,&Interial.position,&Interial.acc_nog,&INS_Estimate.Z.accnog,&INS_Estimate.Y.accnog,
//						100,100);

//				ANO_ImuDataReturn(&INS_Estimate.Z.accnog,&INS_Estimate.Y.accnog,&INS_Estimate.X.accnog,
//						&INS_Estimate.Z.InsVel,&INS_Estimate.Y.InsVel,&INS_Estimate.X.InsVel,&INS_Estimate.Y.InsPos,&INS_Estimate.X.InsPos,
//						INS_Estimate.Z.SlopeState,100);

				ANO_ImuDataReturn(&Eulla.pitch,&Eulla.roll,&Eulla.yaw,
						&INS_Estimate.Z.InsVel,&INS_Estimate.Y.InsVel,&INS_Estimate.X.InsVel,&INS_Estimate.Z.InsPos,&INS_Estimate.X.InsPos,
						INS_Estimate.Z.SlopeState,100);
			
//				ANO_ImuDataReturn(&INS_Estimate.Z.accraw,&INS_Estimate.Y.accraw,&INS_Estimate.X.accraw,
//						&INS_Estimate.Z.accnog,&INS_Estimate.Y.accnog,&INS_Estimate.X.accnog,&INS_Estimate.Z.InsPos,&acc.Z,
//						100,100);
			
//			    ANO_ImuDataReturn(&Eulla.pitch,&Eulla.roll,&Eulla.yaw,
//						&acc.X,&acc.Y,&acc.Z,&Interial.position,&Interial.first_accnog,
//						10,100);				
		}
		
		 if(Flag.system_usartrx_flag)//
		{
			Flag.system_usartrx_flag=0;
			if((Usart_Flag&0x0400)!=0x0000)//�Ƿ���յ���Ч����
			{
				Data_Receive_Anl(USART_RX_BUF,Usart_Flag);//
				Usart_Flag=0;
			}
		}
		
	}
	
}	


