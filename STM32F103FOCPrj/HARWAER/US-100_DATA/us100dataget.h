#ifndef __US100DATAGET_H
#define __US100DATAGET_H
#include "sys.h" 
//**********************************************************

////////////////////////////////////////////////////////////////////////////////// 	
void usart2_usart4_init(u32 bound);
void Usart2_DMA_Tx_(unsigned char* temp);
void US100_DIATANCE_GET(void);
void US100_MODULE_Init(void);
#endif

