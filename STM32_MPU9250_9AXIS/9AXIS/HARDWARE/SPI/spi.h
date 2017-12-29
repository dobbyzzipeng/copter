#ifndef __SPI_H
#define __SPI_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 

////////////////////////////////////////////////////////////////////////////////// 	  
										  
void SPI2_Init(void);			 //��ʼ��SPI��
void SPI2_SetSpeed(u8 SpeedSet); //����SPI�ٶ�   
u8 SPI2_ReadWriteByte(u8 TxData);//SPI���߶�дһ���ֽ�


#define SPI1_CSN_PIN	 GPIO_Pin_4
#define SPI1_CSN_PORT 	 GPIOA
#define SPI1_CS          PAout(4)
	
void SPI1_Init(void);			 //��ʼ��SPI��
void SPI1_SetSpeed(u8 SpeedSet); //����SPI�ٶ�   
u8 SPI1_ReadWriteByte(u8 TxData);//SPI���߶�дһ���ֽ�

void SPI1_WriteOneByte(uint8_t SlaveAddr, uint8_t WriteAddr,uint8_t DataToWrite);
/*******************************************************/
uint8_t SPI1_ReadOneByte(u8 SlaveAddr ,u8 ReadAddr);
void SPI1_ReadmultiyBytes(u8 SlaveAddr,u8 RegAddr,u8 Len,u8 *Buf_Addr);
#endif

