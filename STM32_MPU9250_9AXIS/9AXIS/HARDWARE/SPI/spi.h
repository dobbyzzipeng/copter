#ifndef __SPI_H
#define __SPI_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 

////////////////////////////////////////////////////////////////////////////////// 	  
										  
void SPI2_Init(void);			 //初始化SPI口
void SPI2_SetSpeed(u8 SpeedSet); //设置SPI速度   
u8 SPI2_ReadWriteByte(u8 TxData);//SPI总线读写一个字节


#define SPI1_CSN_PIN	 GPIO_Pin_4
#define SPI1_CSN_PORT 	 GPIOA
#define SPI1_CS          PAout(4)
	
void SPI1_Init(void);			 //初始化SPI口
void SPI1_SetSpeed(u8 SpeedSet); //设置SPI速度   
u8 SPI1_ReadWriteByte(u8 TxData);//SPI总线读写一个字节

void SPI1_WriteOneByte(uint8_t SlaveAddr, uint8_t WriteAddr,uint8_t DataToWrite);
/*******************************************************/
uint8_t SPI1_ReadOneByte(u8 SlaveAddr ,u8 ReadAddr);
void SPI1_ReadmultiyBytes(u8 SlaveAddr,u8 RegAddr,u8 Len,u8 *Buf_Addr);
#endif

