//***************************************************************************************

//***************************************************************************************
#include "usart.h"
#include "delay.h"
#include "led.h"
#include "spi_nrf24l01.h"
#include "stm32f10x.h"
//#include "MOTOR_CONTROL.h"
#include "globalvariable.h"
/******************************************************************************
							IO接口宏定义
*******************************************************************************/ 
#define NRF_CE_GPIO		GPIOC		
#define NRF_CE_Pin		GPIO_Pin_4	
#define NRF_CSN_GPIO	GPIOA		
#define NRF_CSN_Pin		GPIO_Pin_4	
#define NRF_IRQ_GPIO	GPIOC
#define NRF_IRQ_Pin		GPIO_Pin_5

#define NRF_CE_H   		NRF_CE_GPIO ->BSRR = NRF_CE_Pin  //CE高电平
#define NRF_CE_L   		NRF_CE_GPIO ->BRR  = NRF_CE_Pin  //CE低电平
#define NRF_CSN_H  		NRF_CSN_GPIO->BSRR = NRF_CSN_Pin //CSN高电平
#define NRF_CSN_L  		NRF_CSN_GPIO->BRR  = NRF_CSN_Pin //CSN高电平
#define	NRF_IRQ_Read	NRF_IRQ_GPIO->IDR  & NRF_IRQ_Pin //IRQ读数据

/******************************************************************************
							变量定义
*******************************************************************************/ 

uint8_t NRF24L01_RXDATA[NRF_DATA_Length]={0};//nrf24l01接收到的数据
uint8_t NRF24L01_TXDATA[NRF_DATA_Length]={0};//nrf24l01需要发送的数据
static uint8_t TX_ADDRESS[5]= {0x1A,0x3B,0x5C,0x7D,0x9E};//本地地址
static uint8_t RX_ADDRESS[5]= {0x1A,0x3B,0x5C,0x7D,0x9E};//接收地址
static uint16_t Nrf_Erro=0;//NRF错误状态看门狗变量


/******************************************************************************
函数原型:	void SPI1_Init(void)
功　　能:	初始化SPI总线
*******************************************************************************/ 
void SPI1_Init(void)
{
	SPI_InitTypeDef SPI_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
//	NVIC_InitTypeDef 		 NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	//配置SCK,MISO,MOSI引脚  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //推挽复用
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//配置CE引脚
	GPIO_InitStructure.GPIO_Pin = NRF_CE_Pin; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //推挽输出
	GPIO_Init(NRF_CE_GPIO, &GPIO_InitStructure);
	//配置CSN引脚
	GPIO_InitStructure.GPIO_Pin = NRF_CSN_Pin; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //推挽输出
	GPIO_Init(NRF_CSN_GPIO, &GPIO_InitStructure);	
	//配置IRQ引脚

	GPIO_InitStructure.GPIO_Pin = NRF_IRQ_Pin; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //上拉输入
	GPIO_Init(NRF_IRQ_GPIO, &GPIO_InitStructure);	
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource5);
    EXTI_InitStructure.EXTI_Line=EXTI_Line5;
    EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;//外部中断
    EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Falling;//下降沿触发
    EXTI_InitStructure.EXTI_LineCmd=ENABLE;
    EXTI_Init(&EXTI_InitStructure);
	
//	 NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure);
	
	
	NRF_CSN_H;	//禁止NRF器件
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; //双线全双工 
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master; //主模式 
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b; //数据大小8位 
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low; //时钟极性，空闲时为低 
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge; //第1个边沿有效，上升沿为采样时刻 
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;  //NSS信号由软件产生 
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; //8分频，9MHz 
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB; //高位在前 
	SPI_InitStructure.SPI_CRCPolynomial = 7; 
	SPI_Init(SPI1, &SPI_InitStructure); 
	   
	SPI_Cmd(SPI1, ENABLE);//使能 SPI1
	
// 	PrintString("\r\n SPI1     初始化完成！");
}
/******************************************************************************
函数原型:	uint8_t SPI_RW(uint8_t data) 
功　　能:	SPI总线读写
返 回 值：	返回SPI总线读取数据
*******************************************************************************/ 
uint8_t SPI_RW(uint8_t data) 
{
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);//当SPI发送缓冲器非空时等待  
	SPI_I2S_SendData(SPI1, data);//通过SPI总线发送一字节数据
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);//当SPI接收缓冲器为空时等待
	return SPI_I2S_ReceiveData(SPI1);
}

/******************************************************************************
函数原型：	uint8_t NRF_Write_Reg(uint8_t reg, uint8_t value)
功    能：	NRF写寄存器
返 回 值：	NRF写寄存器返回值
*******************************************************************************/
uint8_t NRF_Write_Reg(uint8_t reg, uint8_t value)
{
	uint8_t status;
	NRF_CSN_L;		//选通NRF器件
	status = SPI_RW(reg);//写寄存器地址
	SPI_RW(value);	//写数据
	NRF_CSN_H;		//禁止NRF器件
	return 	status;
}

/******************************************************************************
函数原型：	uint8_t NRF_Read_Reg(uint8_t reg)
功    能：	NRF读寄存器
返 回 值：	寄存器数据
*******************************************************************************/
uint8_t NRF_Read_Reg(uint8_t reg)
{
	uint8_t reg_val;
	NRF_CSN_L;		//选通NRF器件 
	SPI_RW(reg);	//写寄存器地址
	reg_val = SPI_RW(0);//读取该寄存器返回数据
	NRF_CSN_H;		//禁止NRF器件 
    return 	reg_val;
}

/******************************************************************************
函数原型：	uint8_t NRF_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t uchars)
功    能：	NRF写缓冲区
返 回 值：	NRF写缓冲区返回值
*******************************************************************************/
uint8_t NRF_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t uchars)
{
	uint8_t i;
	uint8_t status;
	NRF_CSN_L;	//选通NRF器件 
	status = SPI_RW(reg);//写寄存器地址 
	for(i=0; i<uchars; i++)
	{
		SPI_RW(pBuf[i]);//写数据 
	}
	NRF_CSN_H;	//禁止NRF器件
    return 	status;	
}

/******************************************************************************
函数原型：	uint8_t NRF_Read_Buff(uint8_t reg, uint8_t *pBuf, uint8_t uchars)
功    能：	NRF读缓冲区
返 回 值：	缓冲区数据
*******************************************************************************/
uint8_t NRF_Read_Buff(uint8_t reg, uint8_t *pBuf, uint8_t uchars)
{
	uint8_t i;
	uint8_t status;
	NRF_CSN_L;	//选通NRF器件 
	status = SPI_RW(reg);//写寄存器地址
	for(i=0; i<uchars; i++)
	{
		pBuf[i] = SPI_RW(0);//读取返回数据	
	}
	NRF_CSN_H;	//禁止NRF器件
    return 	status;
}

/******************************************************************************
函数原型：	static void NRF24L01_Set_TX(void)
功    能：	将NRF24L01设置为发送模式
*******************************************************************************/
static void NRF24L01_Set_TX(void)
{
	NRF_CE_L;
	NRF_Write_Reg(NRF_WRITE_REG + CONFIG,0x0E);//发送
	NRF_CE_H;
}

/******************************************************************************
函数原型：	static void NRF24L01_Set_RX(void)
功    能：	将NRF24L01设置为接收模式
*******************************************************************************/
void NRF24L01_Set_RX(void)
{
	NRF_CE_L;
	NRF_Write_Reg(NRF_WRITE_REG + CONFIG,0x0F);//接收
	NRF_CE_H;
}

/******************************************************************************
函数原型：	void NRF_Send_TX(uint8_t * tx_buf, uint8_t len)
功    能：	NRF2401发送数据包
*******************************************************************************/
void NRF_Send_TX(uint8_t * tx_buf, uint8_t len)
{
	NRF24L01_Set_TX();
	NRF_CE_L;//进入待机模式1	
	NRF_Write_Buf(WR_TX_PLOAD, tx_buf, len);//装载数据
	NRF_CE_H;//设置CE为高，启动发射。CE高电平持续时间最小为10us
}

/******************************************************************************
函数原型：	void NRF24L01_Check(void)
功    能：	检查NRF器件是否正常
*******************************************************************************/
void NRF24L01_Check(void)
{
	uint8_t buf[5]; 
	uint8_t i; 
	//写入5个字节的地址 
	NRF_Write_Buf(NRF_WRITE_REG+TX_ADDR,TX_ADDRESS,5); 
	//读出写入的地址 
	NRF_Read_Buff(TX_ADDR,buf,5); 
	//比较
	for(i=0;i<5;i++) 
	{ 
		if(buf[i]!=TX_ADDRESS[i]) 
			break; 
	} 
	if(i==5)
	{
		printf("nrf ok!");
	}
	else
	{
		printf("nrf fail!");
		while(1)
		{
			LED1_TOGGLE();
			delay_ms(200);
		}
	}
}

/******************************************************************************
函数原型：	void NRF24L01_Init(uint8_t Chanal,uint8_t Mode)
功    能：	NRF24L01初始化
参    数：	Chanal，RF通道  Mode:模式 RX接收 TX发送
*******************************************************************************/
void NRF24L01_Init(uint8_t Chanal,uint8_t Mode,uint8_t datalength)
{
	NRF_CE_L;
		
	NRF_Write_Reg(FLUSH_TX,0xff);//清空发送缓冲区
	NRF_Write_Reg(FLUSH_RX,0xff);//清空接收缓冲区
	NRF_Write_Buf(NRF_WRITE_REG + TX_ADDR,   TX_ADDRESS,5); //写TX节点地址  	
	NRF_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0,RX_ADDRESS,5);	//写RX节点地址 

	NRF_Write_Reg(NRF_WRITE_REG + EN_AA,     0x01); //使能通道0的自动应答 
	NRF_Write_Reg(NRF_WRITE_REG + EN_RXADDR, 0x01);	//使能通道0的接收地址 
	NRF_Write_Reg(NRF_WRITE_REG + SETUP_RETR,0x1a);	//设置自动重发间隔时间:500us;最大自动重发次数:10次 
	NRF_Write_Reg(NRF_WRITE_REG + RF_CH,   Chanal);	//设置RF通道为CHANAL
	NRF_Write_Reg(NRF_WRITE_REG + RX_PW_P0,  datalength);	//设置通道0的有效数据宽度
	NRF_Write_Reg(NRF_WRITE_REG + RF_SETUP,  0x0f); //设置TX发射参数,0db增益,2Mbps,低噪声增益开启
	
	if(Mode==TX)
		NRF_Write_Reg(NRF_WRITE_REG + CONFIG,0x0E);//发送
	else if(Mode==RX)
		NRF_Write_Reg(NRF_WRITE_REG + CONFIG,0x0F);//接收
	
	NRF_CE_H;
}

/******************************************************************************
函数原型：	static void NRF24L01_Analyse(void)
功    能：	分析NRF24L01收到的数据帧
*******************************************************************************/
void NRF24L01_Analyse(void)
{
	
	if(NRF24L01_RXDATA[0]==0xAA || NRF24L01_RXDATA[17]==0XEE)
	{
		if(NRF24L01_RXDATA[1]==0XF1)//功能字
		{
//			Flag.nrf_error_cnt=0;//看门狗变量清零
//			Flag.nrf_flag=1;
//			Rcdata.pitch	  =(short)(((int16_t)NRF24L01_RXDATA[2])<<8|(int16_t)NRF24L01_RXDATA[3]);
////			Rcdata.pitch    =Rcdata.pitch/*/21.0f*/;
//			Rcdata.roll		  =(short)(((int16_t)NRF24L01_RXDATA[4])<<8|(int16_t)NRF24L01_RXDATA[5]);
////			Rcdata.roll     =Rcdata.roll/*/21.0f*/;
//			Rcdata.yaw		  =(short)(((int16_t)NRF24L01_RXDATA[6])<<8|(int16_t)NRF24L01_RXDATA[7]);
////			Rcdata.yaw      =Rcdata.yaw/*/32.0f*/;
//			Rcdata.throttle  =(short)(((int16_t)NRF24L01_RXDATA[8])<<8|(int16_t)NRF24L01_RXDATA[9]);
//			Rcdata.throttle  =Rcdata.throttle/200.0f;
//			
//			Rcdata.pitchtrim=NRF24L01_RXDATA[10];
//			Rcdata.rolltrim =NRF24L01_RXDATA[11];
//			
//			Rcdata.Jindou		  =NRF24L01_RXDATA[12];//翻筋斗
//			Rcdata.Lock			  =NRF24L01_RXDATA[13];//电机锁住标志
//			Rcdata.Selft_Test	=NRF24L01_RXDATA[14];//陀螺仪自检标志
//			Rcdata.batvol		  =NRF24L01_RXDATA[15];//遥控器电池电压标志
//			Rcdata.Mode       =NRF24L01_RXDATA[16];
//// 			printf("NRF data get\r\n");
//			RCData_Check(&Rcdata,&Copter);
		}
		NRF24L01_RXDATA[0]=0;
		NRF24L01_RXDATA[17]=0;
//		printf("data get\r\n");
	}

}

///******************************************************************************
//函数原型：	void Nrf_Send_Data_Back(unsigned char *temp,unsigned char cnt)
//功    能：	向遥控发送数据
//*******************************************************************************/
//void Nrf_Send_Data_Back(unsigned char *temp,unsigned char cnt)
//{
//	unsigned char i=0,len=cnt;

//	*(temp+(i++))=0XAA;//帧头
//	*(temp+(i++))=0XF1;//功能字
//	*(temp+(i++))='I';
//	*(temp+(i++))='A';
//	
//	*(temp+(i++))='M';  
//	*(temp+(i++))='O';
//	
//	*(temp+(i++))='K';
//	*(temp+(i++))='!';
//	
//	*(temp+(i++))='D';
//	*(temp+(i++))='A';

//	*(temp+(i++))='T';//
//	*(temp+(i++))='A';//
//	*(temp+(i++))='G';//
//	*(temp+(i++))='E';//
//	*(temp+(i++))='T';//

//	NRF_Send_TX(temp,len);
//}

/****************************************/
//外部中断
//
void EXTI9_5_IRQHandler(void)//Nrf2401中断
{	
	if(EXTI_GetITStatus(EXTI_Line5) != RESET) 
	{
		EXTI_ClearITPendingBit(EXTI_Line5);
		NRF24L01_IRQ();
	}
}
/******************************************************************************
函数原型：	void NRF24L01_IRQ(void)
功    能：	NRF24L01中断
*******************************************************************************/
void NRF24L01_IRQ(void)	
{
	uint8_t status = NRF_Read_Reg(NRF_READ_REG + NRFRegSTATUS);//读取状态寄存器数据
	
	if(status & (1<<RX_DR))//接收中断
	{
		uint8_t rx_len = NRF_Read_Reg(R_RX_PL_WID);//读取有效数据长度
		if(rx_len==NRF_DATA_Length)
		{
			NRF_Read_Buff(RD_RX_PLOAD,NRF24L01_RXDATA,rx_len);//读取接收FIFO数据 放入数组NRF24L01_RXDATA
// 			printf("Data get\r\n");
			Nrf_Erro = 0;//失控错误看门狗变量清零
		}
		else
		{
			NRF_Write_Reg(FLUSH_RX,0xff);//清空接收缓冲区
		}
	}
	if(status & (1<<MAX_RT))//达到最多次重发中断
	{
		if(status & (1<<TX_FULL))//TX FIFO 溢出
		{
			NRF_Write_Reg(FLUSH_TX,0xff);//清空发送缓冲区
		}
	}
//	if(status&(1<<TX_DS))//发送完成
//	{	
// 		NRF24L01_Set_RX();//设置Nrf2401为接收模式
//	}
	NRF_Write_Reg(NRF_WRITE_REG + NRFRegSTATUS, status);//清除中断标志位	
}

/******************************************************************************
函数原型：	void Nrf_Connect(void)
功    能：	NRF24L01连接函数
*******************************************************************************/
void Nrf_Connect(void)//1KHZ
{
	Nrf_Erro ++;//失控错误看门狗变量累加
// 	if(Nrf_Erro==1)
// 	{
		NRF24L01_Analyse();//分析NRF24L01收到的数据帧
// 		Nrf_Send_Data_Back(NRF24L01_TXDATA,NRF_DATA_Length);
// 	}
		NRF24L01_IRQ();    //清除中断标志位
//	if(Nrf_Erro%100==0)  //0.1s未接收nrf数据 ，试图连接遥控
//	{	
//// 		NRF24L01_IRQ();    //清除中断标志位
//	}
//	if(Nrf_Erro>=1000)   //1s未接收nrf数据 ,锁定四轴，电机停转，防止意外发生
//	{	

//	}
}
