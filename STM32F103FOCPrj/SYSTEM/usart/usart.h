#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 
//**********************************************************

////////////////////////////////////////////////////////////////////////////////// 	
#define USART_REC_LEN  			24	//�����������ֽ��� 4
#define EN_USART1_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����
#define DMA_TX_LENGTH   		(5+9*4)//DMA����x����Ч���� float
//#define DMA_TX_LENGTH   		(5+1)//DMA����x����Ч���� float
#define DMA_RX_LENGTH   		24
	  	
extern u8  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
//extern u16 USART_RX_STA;         		//����״̬���	

extern u8 USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
extern uint16_t Usart_Flag;


//����봮���жϽ��գ��벻Ҫע�����º궨��
void uart_init(u32 bound);
void USART_Send_BUF(uint8_t *buf,uint8_t len);
void DMA_Tx_(signed short Data1,signed short Data2,signed short Data3);
//void ANO_Data_Send(float *pitch,float *roll,float *yaw);


void ANO_Data_Send_ALTVEL_Test_(float *angle,
								float *temp1,float *temp2,float *temp3,
								float *temp4,float *temp5,float *temp6,
								float *thottle,
								signed short * temp7,signed short * temp8);
								
void ANO_ImuDataReturn(float *angle,
								float *temp1,float *temp2,float *temp3,
								float *temp4,float *temp5,float *temp6,
								float *thottle,
								signed short  temp7,
								signed short  temp8);
								
//void ANO_Data_Send_(float *pitch,float *roll,float *yaw,Data_To_Imu * velocity,float target);
//void ANO_Data_Send_Test(float *setpitch,float *pitch,float *vy,float *thottle);
void ANO_Data_Send_Test_(float *angle,float *temp1,float *temp2,float *temp3,signed short * temp4,signed short * temp5);
void ANO_Comunication_Check(unsigned char data);
								
#endif


