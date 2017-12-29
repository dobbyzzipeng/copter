///****************************************
//						����V1.0 mpu9250���� 
//						�޸�ʱ��2015/07/29
//***************************************/
#include "mpu9250.h"
//#include "delay.h"
//#include "sys.h"
//uint8_t	 MPU9250_buf[22];					//spi��ȡ��������

//S_INT16_XYZ	GYRO_OFFSET={0,0,0},ACC_OFFSET={56,86,0};		//��Ư

//S_INT16_XYZ MPU9250_ACC_LAST={0,0,0};
//S_INT16_XYZ MPU9250_GYRO_LAST={0,0,0};
//S_INT16_XYZ MPU9250_MAG_LAST={0,0,0};

////������������ƫУ׼(��ر������ᾲֹ�����ϵ�У׼)
//void Clac_GYRO_OFFECT()
//{
//		static int32_t	tempgx=0,tempgy=0,tempgz=0;
//		uint16_t cnt_g=0;	//����ֵ
//	
//		//��ʼ��������
//		cnt_g = 0;
//		GYRO_OFFSET.X=0;
//		GYRO_OFFSET.Y=0;
//		GYRO_OFFSET.Z=0;
//		tempgx = 0;
//		tempgy = 0;
//		tempgz = 0;
//	
//		//��ȡ���ݲ�����
//		while(cnt_g!=501)
//		{
//			MPU9250_ReadValue();
//			tempgx+= MPU9250_GYRO_LAST.X;
//			tempgy+= MPU9250_GYRO_LAST.Y;
//			tempgz+= MPU9250_GYRO_LAST.Z;
//			cnt_g++;
//			delay_ms(1);
//		}
//		GYRO_OFFSET.X=tempgx/cnt_g;
//		GYRO_OFFSET.Y=tempgy/cnt_g;
//		GYRO_OFFSET.Z=tempgz/cnt_g;
//}


////MPU9250��ʼ����
//uint8_t MPU9250_Init(void)
//{
//	uint8_t result=0;
//	result = MPU9250_Read_Reg(WHO_AM_I);
//	if(result==0x73 || result==0x70)	//��ȷ��ȡ��9250�ĵ�ַ
//	{
//		MPU9250_Write_Reg(PWR_MGMT_1,0X80);   //��Դ����,��λMPU9250
//		delay_ms(1);
//		MPU9250_Write_Reg(PWR_MGMT_1,0X01);   //ѡ��ʱ��Դ
//		MPU9250_Write_Reg(PWR_MGMT_2,0X00);   //ʹ�ܼ��ٶȼƺ�������
//		MPU9250_Write_Reg(CONFIG,0X02);				//��ͨ�˲��� 0x06 92hz (3.9ms delay) fs=1khz
//		MPU9250_Write_Reg(SMPLRT_DIV,0x00);		//������1000/(1+0)=1000HZ
//		MPU9250_Write_Reg(GYRO_CONFIG,0X10);  //�����ǲ�����Χ 0X18 ����1000��
//		MPU9250_Write_Reg(ACCEL_CONFIG,0x08); //���ٶȼƲ�����Χ 0X00 ����4g
//		MPU9250_Write_Reg(ACCEL_CONFIG2,0x00);//���ٶȼ�����1khz �˲���460hz (1.94ms delay)
//		MPU9250_Write_Reg(MAG_INT_PIN_CFG,0x30); //�в���������жϱ�־
//        MPU9250_Write_Reg(MAG_I2C_MST_CTRL,0x4D); // I2C Speed 400 kHz
//		MPU9250_Write_Reg(USER_CTRL,0X30);  //ʹ��MPU9250 AUX
//		delay_ms(1);
//		
//		Init_AK8963();
//		
//		return 0;
//	}
//	return 1;
//}

////MPU9250��ʼ��
//void MPU9250_IOAndSPI_Init(void)
//{
//	GPIO_InitTypeDef GPIO_InitStructure;
//    SPI_InitTypeDef  SPI_InitStructure;

//	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB,ENABLE );//PORTAʱ��ʹ�� 
//	RCC_APB2PeriphClockCmd( RCC_APB2Periph_SPI1 , ENABLE );//SPI1ʱ��ʹ�� 	
// 	RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO,ENABLE );
//	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4; //MPU9250Ƭѡ�ź�
// 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);	//��ʼ��ָ��IO
// 	GPIO_SetBits(GPIOA,GPIO_Pin_4);//����  ��ֹSPIͨ��
//	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //PA567����������� 
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);

//	MPU9250_CS=1;  //MPU9250Ƭѡȡ��
//	
//	SPI_Cmd(SPI1,DISABLE);  //SPI2��ʹ��
//	
//	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //SPI����Ϊ˫��˫��ȫ˫��
//	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//SPI����
//	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//���ͽ���8λ֡�ṹ
//	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;		//ʱ�����յ�
//	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	//���ݲ����ڵ�1��ʱ����
//	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS�ź����������
//	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;		//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ16��72M/16=6M��
//	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//���ݴ����MSBλ��ʼ
//	SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRCֵ����Ķ���ʽ
//	SPI_Init(SPI1, &SPI_InitStructure);  //����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���
//	
//	SPI_Cmd(SPI1,ENABLE);//ʹ��SPI2
//	
//	MPU9250_CS=1;  //MPU9250Ƭѡȡ��
//}

//void Read_AK8963_Data()
//{
//	
//}
////����AK8963��ģʽ ��������ģʽ2 �����ڼ����ٶ�Ϊ138hz ����ʹ��mode2 ���Դﵽ100hz��
//void Init_AK8963()
//{
//	uint8_t temp=0;
//	
//	delay_ms(500);
//	
//	MPU9250_Write_Reg(MAG_I2C_SLV0_ADDR,0x0C);//write I2C addr 
//	MPU9250_Write_Reg(MAG_I2C_SLV0_DO,0x01);	//�����豸 
//	MPU9250_Write_Reg(MAG_I2C_SLV0_REG,AK8963_CNTL2);
//	MPU9250_Write_Reg(MAG_I2C_SLV0_CTRL,0X81);//д��һ������
//	
//	delay_ms(100);
//	
//	//Set Slave to Read AK8963
//	MPU9250_Write_Reg(MAG_I2C_SLV0_ADDR,0x8c);
//	MPU9250_Write_Reg(MAG_I2C_SLV0_REG,0x00);
//	MPU9250_Write_Reg(MAG_I2C_SLV0_CTRL,0x81);
//	delay_us(1000); //�������ʱ ������� ��С600us ����I2Cͨ���ٶ��йأ� 	����ʹ��I2C_SLV4 �����
//	temp = MPU9250_Read_Reg(MAG_EXT_SENS_DATA_00);
//		
//	if(temp == 0x48)
//	{
//			//�ɹ�
//			delay_ms(500);
//	}
//	else 
//	{
//			//ʧ��
//			while(1);
//	}
//	
//	MPU9250_Write_Reg(MAG_I2C_SLV0_ADDR,0x0C);//write I2C addr 
//	MPU9250_Write_Reg(MAG_I2C_SLV0_DO,0x16); //ѡ����������ģʽ 16bit���
//	MPU9250_Write_Reg(MAG_I2C_SLV0_REG,AK8963_CNTL1);
//	MPU9250_Write_Reg(MAG_I2C_SLV0_CTRL,0X81);//д��һ������
//	delay_us(1000);	//�������ʱ ������� ��С200us ����I2Cͨ���ٶ��йأ�
//	MPU9250_Write_Reg(MAG_I2C_SLV0_ADDR,0x8C);//read I2C addr 
//	MPU9250_Write_Reg(MAG_I2C_SLV0_REG,AK8963_CNTL1);
//	MPU9250_Write_Reg(MAG_I2C_SLV0_CTRL,0X81);//��ȡһ������
//	delay_us(1000);	//�������ʱ ������� ��С200us ����I2Cͨ���ٶ��йأ�
//	temp = MPU9250_Read_Reg(MAG_EXT_SENS_DATA_00);
//	if(temp == 0x16)
//	{
//			//��ʼ���ɹ�
//			delay_ms(500);
//	}
//	else 
//	{
//			//��ʼ��ʧ��
//			while(1);
//	}
//	//OLED_Clear();
//	
//	MPU9250_Write_Reg(MAG_I2C_SLV0_ADDR, 0x8C);          // Set AK8963_I2C_ADDR = 7'b000_1100
//	delay_us(1000);
//	MPU9250_Write_Reg(MAG_I2C_SLV0_REG, AK8963_ST1);     // Set Read Reg
//	MPU9250_Write_Reg(MAG_I2C_SLV0_CTRL, 0x88);          // Start Read, 6 bytes	ST1����ST2
//	delay_us(1000);
//	
//}
////SPIд�Ĵ���
////reg:ָ���ļĴ�����ַ
////value:д���ֵ
////���أ���ȡ����ֵ
//uint8_t MPU9250_Write_Reg(uint8_t reg,uint8_t value)
//{
//	uint8_t status;
//	MPU9250_CS=0;  //ʹ��SPI����
//	status=SPI1_ReadWriteByte(reg); //����д����+�Ĵ�����
//	SPI1_ReadWriteByte(value);//д��Ĵ���ֵ
//	MPU9250_CS=1;  //��ֹMPU9250
//	return(status);//����״ֵ̬
//}

////SPI��ȡ�Ĵ���
////reg:ָ���ļĴ�����ַ
//uint8_t MPU9250_Read_Reg(uint8_t reg)
//{
//	uint8_t reg_val;
//	MPU9250_CS=0;  //ʹ��SPI����
//	SPI1_ReadWriteByte(reg|0x80); //���Ͷ�����+�Ĵ�����
//	reg_val=SPI1_ReadWriteByte(0xff);//��ȡ�Ĵ���ֵ
//	MPU9250_CS=1;  //��ֹMPU9250
//	return(reg_val);
//}

////SPIx ��дһ���ֽ�
////TxData:Ҫд����ֽ�
////����ֵ:��ȡ�����ֽ�
//uint8_t SPI1_ReadWriteByte(uint8_t TxData)
//{		
//	uint8_t retry=0;				 	
//	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) //���ָ����SPI��־λ�������:���ͻ���ձ�־λ
//		{
//		retry++;
//		if(retry>200)return 0;
//		}			  
//	SPI_I2S_SendData(SPI1, TxData); //ͨ������SPIx����һ������
//	retry=0;

//	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET) //���ָ����SPI��־λ�������:���ܻ���ǿձ�־λ
//	//while((SPI2->SR &SPI_I2S_FLAG_RXNE) == RESET);	
//		{
//		retry++;
//		if(retry>200)return 0;
//		}	  						    
//	return SPI_I2S_ReceiveData(SPI1); //����ͨ��SPIx������յ�����					    
//}
