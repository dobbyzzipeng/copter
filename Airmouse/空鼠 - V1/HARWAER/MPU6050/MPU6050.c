#include "MPU6050.h"
#include "led.h"
#include "delay.h"
#include "usart.h"
#include "stmflash.h"
#include "globaldefine.h"

const float Pi = 3.1415927f;
const float gyro_denominator = 1877.2f; //������������ ת��Ϊrad/S
//const float gyro_denominator = 32.75f; //������������ ת��Ϊ��/S

const float acc_denominator = 835.9183673f; //�Ӽ������� 8192/9.8=835.9183673f ���ݵ����������ٶ�9.788m/s^2
//const float acc_denominator = 417.15f;//�Ӽ������� 4096/9.8=417.15f ���ݵ����������ٶ�9.788m/s^2


const unsigned char accfilter_Num = 6; //ƽ���˲����
const unsigned char gyrofilter_Num = 6; //ƽ���˲����

const float ACCDIVD = acc_denominator*accfilter_Num;
const float GYRODIVD = gyro_denominator*gyrofilter_Num;

const unsigned char offsetlength = 25;  //������ƫ�����������ݴ���
const float gyroscale=gyro_denominator*gyrofilter_Num;


const unsigned char IIR_FILTER_NUM=4;
float IIR_FILTER_PARA_B[IIR_FILTER_NUM+1]={0.0008f,0.0032f,0.0048f,0.0032f,0.0008f};
float IIR_FILTER_PARA_A[IIR_FILTER_NUM+1]={1.0000f,-3.0176f,3.5072f,-1.8476f,0.3708f};

Origial_DATA Sensor_Acc,Sensor_Gyro;//������ԭʼ����
Data_To_Imu  acc, velocity;
Data_To_Imu  ACC, VELOCITY;	
OFFSET OffSet;
kalman_struct KalmanfilterAccx,KalmanfilterAccy,KalmanfilterAccz;
//**********���ݶ�ȡ******************/
//���ݶ�ȡ���ڽ���ʹ��
void MPU6050_Read_To_Use_(Origial_DATA*rawacc,Origial_DATA*rawgyro,OFFSET*offset)
{
	uint8_t Mpu6050_Data_Buf[14]={0};//������ԭʼ���ݽ��ջ���
	int32_t G_X=0,G_Y=0,G_Z=0,A_X=0,A_Y=0,A_Z=0;
	int32_t temp=0;
	
	
//	if(Rcdata.Selft_Test)
//	{
//		MPU6050_Data_Check_(&OffSet);
//		AHRSReset();
//		Rcdata.Selft_Test=0;
//	}
//	else
//	{
	 ReadmultiyBytes(MPU6050_Addr, ACCEL_XOUT_H,14,Mpu6050_Data_Buf);//ģ��IIC��ȡ����
	
	 A_X=(((int16_t)Mpu6050_Data_Buf[0]<<8)|Mpu6050_Data_Buf[1])-offset->AX_Offset;
	 A_Y=(((int16_t)Mpu6050_Data_Buf[2]<<8)|Mpu6050_Data_Buf[3])-offset->AY_Offset;
	 A_Z=(((int16_t)Mpu6050_Data_Buf[4]<<8)|Mpu6050_Data_Buf[5])+offset->AZ_Offset;
	// Temperature=	(((int16_t)Mpu6050_Data_Buf[6]<<8)|Mpu6050_Data_Buf[7]);//�ڲ��¶�ADC
	 G_X=(((int16_t)Mpu6050_Data_Buf[8]<<8)|Mpu6050_Data_Buf[9])-offset->GX_Offset;
	 G_Y=(((int16_t)Mpu6050_Data_Buf[10]<<8)|Mpu6050_Data_Buf[11])-offset->GY_Offset;
	 G_Z=(((int16_t)Mpu6050_Data_Buf[12]<<8)|Mpu6050_Data_Buf[13])-offset->GZ_Offset;
	 /***�����޷�***/
// 	if(G_X>Data_Max) G_X=Data_Max;
// 	if(G_X<Data_Min) G_X=Data_Min;
// 	if(G_Y>Data_Max) G_Y=Data_Max;
// 	if(G_Y<Data_Min) G_Y=Data_Min;
// 	if(G_Z>Data_Max) G_Z=Data_Max;
// 	if(G_Z<Data_Min) G_Z=Data_Min;
// 	if(A_X>Data_Max) A_X=Data_Max;
// 	if(A_X<Data_Min) A_X=Data_Min;
// 	if(A_Y>Data_Max) A_Y=Data_Max;
// 	if(A_Y<Data_Min) A_Y=Data_Min;
//  if(A_Z>Data_Max) A_Z=Data_Max;
//  if(A_Z<Data_Min) A_Z=Data_Min;
	
	#if IMU_CHIP_ROTATION==0  //0����MPU6050+X��Ϊ��ͷǰ��
//	_nop();	
	
	#elif IMU_CHIP_ROTATION==1//1����MPU6050-X��Ϊ��ͷǰ��
	A_X=-A_X;
	A_Y=-A_Y;
	
	G_X=-G_X;
	G_Y=-G_Y;
	
	#elif IMU_CHIP_ROTATION==2//2����MPU6050+Y��Ϊ��ͷǰ��
	temp=A_X;
	A_X = A_Y;
	A_Y =-temp;
	
	temp=G_X;
	G_X = G_Y;
	G_Y = -temp;
	
	#elif IMU_CHIP_ROTATION==3//3����MPU6050-Y��Ϊ��ͷǰ��
	temp=A_X;
	A_X=-A_Y;
	A_Y= temp;

	temp=G_X;
	G_X =-G_Y;
	G_Y = temp;
	#endif
	
	rawacc->X=(int16_t)A_X;
	rawacc->Y=(int16_t)A_Y;
	rawacc->Z=(int16_t)A_Z;
	rawgyro->X=(int16_t)G_X;
	rawgyro->Y=(int16_t)G_Y;
	rawgyro->Z=(int16_t)G_Z;
	
// 	printf("%d\t %d\t %d\t %d\t %d\t %d\r\n",Origial_DATA__ACC_XYZ->X,Origial_DATA__ACC_XYZ->Y,
// 	Origial_DATA__ACC_XYZ->Z,Origial_DATA__GYRO_XYZ->X,Origial_DATA__GYRO_XYZ->Y,Origial_DATA__GYRO_XYZ->Z);
//	}
}

//**********���ݶ�ȡ******************/
//���ݶ�ȡ������ƫ����
void MPU6050_Read_To_Calculate(Origial_DATA *rawacc,Origial_DATA *rawgyro)
{
	uint8_t Mpu6050_Data_Buf[14]={0};//
	int32_t G_X=0,G_Y=0,G_Z=0,A_X=0,A_Y=0,A_Z=0;
	int32_t temp=0;
	
	ReadmultiyBytes(MPU6050_Addr, ACCEL_XOUT_H,14,Mpu6050_Data_Buf);//
	
	 A_X=	(((int16_t)Mpu6050_Data_Buf[0]<<8)|Mpu6050_Data_Buf[1]);
	 A_Y=	(((int16_t)Mpu6050_Data_Buf[2]<<8)|Mpu6050_Data_Buf[3]);
	 A_Z=	(((int16_t)Mpu6050_Data_Buf[4]<<8)|Mpu6050_Data_Buf[5]);
	// Temperature=	(((int16_t)Mpu6050_Data_Buf[6]<<8)|Mpu6050_Data_Buf[7]);//�ڲ��¶�ADC
	 G_X=	(((int16_t)Mpu6050_Data_Buf[8]<<8)|Mpu6050_Data_Buf[9]);
	 G_Y=	(((int16_t)Mpu6050_Data_Buf[10]<<8)|Mpu6050_Data_Buf[11]);
	 G_Z=	(((int16_t)Mpu6050_Data_Buf[12]<<8)|Mpu6050_Data_Buf[13]);

	#if IMU_CHIP_ROTATION==0  //0����MPU6050+X��Ϊ��ͷǰ��
//	_nop();

	#elif IMU_CHIP_ROTATION==1//1����MPU6050-X��Ϊ��ͷǰ��
	A_X=-A_X;
	A_Y=-A_Y;
	
	G_X=-G_X;
	G_Y=-G_Y;
	
	#elif IMU_CHIP_ROTATION==2//2����MPU6050+Y��Ϊ��ͷǰ��
	temp=A_X;
	A_X = A_Y;
	A_Y =-temp;
	
	temp=G_X;
	G_X = G_Y;
	G_Y = -temp;
	
	#elif IMU_CHIP_ROTATION==3//3����MPU6050-Y��Ϊ��ͷǰ��
	temp=A_X;
	A_X=-A_Y;
	A_Y= temp;

	temp=G_X;
	G_X =-G_Y;
	G_Y = temp;
	#endif
	
	rawacc->X=(int16_t)A_X;
	rawacc->Y=(int16_t)A_Y;
	rawacc->Z=(int16_t)A_Z;
	rawgyro->X=(int16_t)G_X;
	rawgyro->Y=(int16_t)G_Y;
	rawgyro->Z=(int16_t)G_Z;
}
/*********MPU6050��ʼ��*************/
//���ܣ���ʼ��MPU6050
//��ϸ��Datasheet
//���룺��
//�������
//���أ���
void MPU6050_Init(void)
{
	IIC_Init();//IIC��ʼ��	
	delay_ms(10);
	MPU6050_CHECK();
	MPU6050_Register_Configuration();
	printf("MPU6050 is OK!\r\n");
}
/*****************/
//MPU6050_Register_Configuration
//�����ڲ��Ĵ���
//�趨������Ϊ500HZ�������������Ϊ1K����SMPLRT_DIV=1000/500-1=1
//���üĴ���CONFIGUATION��3λר������DLPF,������Ϊ500HZʱ��DLPF��ֹƵ��Ӧ��Ϊ��һ�룬��250HZ���ҡ�
//�鿴�����ֲ�ã�DLPF_CFG[2��0]=001ʱ������������ʸպ�Ϊ1K��DLPF��ֹƵ��Ϊ188HZ
void MPU6050_Register_Configuration(void)
{
	WriteOneByte(MPU6050_Addr,PWR_MGMT_1, 0x00);	  //�������״̬
	WriteOneByte(MPU6050_Addr,SMPLRT_DIV, GYRO_SAMPRATE_);    //�����ǲ����� 500HZ
	WriteOneByte(MPU6050_Addr,CONFIGUATION,DLPF_CFG_);   //��ͨ�˲���Ƶ������188HZ ACC 184HZ GYRO �����������1K
	WriteOneByte(MPU6050_Addr,I2C_MST_CTRL,0x0D);        //I2C���� 400khz
	WriteOneByte(MPU6050_Addr,0x6A,0xC8);           //����MPU6050 ����AUXI2C(BIT5=0)
	WriteOneByte(MPU6050_Addr,0x37,0x32);           //����������I2C��MPU6050��AUXI2Cֱͨ������������ֱ�ӷ���HMC5883L(BIT1=1)
//	WriteOneByte(MPU6050_Addr,GYRO_CONFIG, 0x08);  //+-500��/s 65535/1000=65.5�ֱ��� 
	WriteOneByte(MPU6050_Addr,GYRO_CONFIG, 0x10);  //+-1000��/s 65535/2000=32.75�ֱ��� 

	#ifdef ACC_8G_SCALE
	{
		WriteOneByte(MPU6050_Addr,ACCEL_CONFIG, 0x10); //+-8g  65536/16=4096�ֱ���
	}
	#endif
	
	#ifdef ACC_4G_SCALE
	{
		WriteOneByte(MPU6050_Addr,ACCEL_CONFIG, 0x08); //+-4g  65536/8=8192�ֱ���	
	}
	#endif
}

/*******************/
//MPU6050_CHECK
//���MPU6050�Ƿ����
void MPU6050_CHECK()
{
	unsigned char chip_id=0,_cnt_=0;
	chip_id=ReadOneByte(MPU6050_Addr,WHO_AM_I);
	if(!chip_id)
	{
		for(_cnt_=0;_cnt_<10;_cnt_++)
		{
			if(!ReadOneByte(MPU6050_Addr,WHO_AM_I))
			{
				delay_ms(100);
				if(_cnt_==(10-1))
				{
					while(1)
					{
						printf("mpu6050 init error!/r/n");
						LED1_Flash(1);

					}
				}
			}
			else
			{
				break;
			}
		}
	}
}

/************���ݱ궨**************/
//���ܣ��ԼӼơ����������ݽ��б궨
//���룺�OOFF_SET *ACC_OFF,OFF_SET *GYRO_OFF
//�������
void MPU6050_Data_Check_(OFFSET *offset)
{
	u8 num=0;
	int32_t temp[6]={0};

	for(num=0;num<offsetlength;num++)
	{
	   MPU6050_Read_To_Calculate(&Sensor_Acc,&Sensor_Gyro);	
		
	     temp[0]+=Sensor_Acc.X;
		 temp[1]+=Sensor_Acc.Y;
		 temp[2]+=Sensor_Acc.Z;
		
		 temp[3]+=Sensor_Gyro.X;
		 temp[4]+=Sensor_Gyro.Y;
		 temp[5]+=Sensor_Gyro.Z;
		
		 LED1_TOGGLE();
		 MOTOR_LED_TOGGLE();
		 delay_ms(20);
	}
	
	offset->AX_Offset=temp[0]/offsetlength;
	offset->AY_Offset=temp[1]/offsetlength;
	offset->AZ_Offset=ACC_4G_SCALE-temp[2]/offsetlength;
	
	offset->GX_Offset=temp[3]/offsetlength;
	offset->GY_Offset=temp[4]/offsetlength;
	offset->GZ_Offset=temp[5]/offsetlength;
// 	STMFLASH_Write(FLASH_Read_Addr,(u16*)OffSet,6);//���ݱ���
//	delay_ms(20);
//	printf("offset:%d\t %d\t %d\t %d\t %d\t %d\r\n",off_set[0],off_set[1],off_set[2],off_set[3],off_set[4],off_set[5]);
	
}

/******************/
//˫�����˲�����
//����������˲������ת�������õ��ļ��ٶ�(g/m^2)�����ٶ�����(rad/s)
//���룺ԭʼ���� �궨�õ�����ƫ
//�˲���Ȳ��˹��󣬷���Ӵ�������ʱ�����Ͷ�̬ЧӦ
void MPU6050_Data_Read_Analys(Data_To_Imu *GYRO,Data_To_Imu *Acc,Origial_DATA *rawacc,Origial_DATA *rawgyro)
{
	u8 i=0,j=0;
	static u8 count=0,counter=0;//��������
	int32_t temp1=0,temp2=0,temp3=0,temp4=0,temp5=0,temp6=0;
	static int16_t BUF1[accfilter_Num]={0},BUF2[accfilter_Num]={0},BUF3[accfilter_Num]={0};
	static int16_t BUF4[gyrofilter_Num]={0},BUF5[gyrofilter_Num]={0},BUF6[gyrofilter_Num]={0};

     BUF1[count]=rawacc->X;
	 BUF2[count]=rawacc->Y;
	 BUF3[count]=rawacc->Z;
	
	 BUF4[counter]=rawgyro->X;
	 BUF5[counter]=rawgyro->Y;
	 BUF6[counter]=rawgyro->Z;

	for(i=0;i<accfilter_Num;i++)
	{
		temp1+= BUF1[i];
		temp2+= BUF2[i];
		temp3+= BUF3[i];
	}
//	
	for(j=0;j<gyrofilter_Num;j++)
	{
		temp4+= BUF4[j];
		temp5+= BUF5[j];
		temp6+= BUF6[j];
	}
	
	 Acc->X=temp1/ACCDIVD;//m/s^2
	 Acc->Y=temp2/ACCDIVD;
	 Acc->Z=temp3/ACCDIVD;
		
	 GYRO->X=temp4/GYRODIVD;//rad/s
	 GYRO->Y=temp5/GYRODIVD;	
	 GYRO->Z=temp6/GYRODIVD;

	if(++count==accfilter_Num) count=0;	
	if(++counter==gyrofilter_Num) counter=0;	
//	printf("%7.3f\t %7.3f\t %7.3f\t %7.3f\t %7.3f\t %7.3f\r\n", Acc->X, Acc->Y, Acc->Z, GYRO->X, GYRO->Y, GYRO->Z);
}


/*���ݽ���*/
//�����ǲ�õĽ��ٶ�������imuupdate���������ı�ֵ��������õ�ַ���ݸı����ֵ�Ļ�
void MPU6050_Data_Exchange(Data_To_Imu*accin,Data_To_Imu*gyroin,Data_To_Imu*accout,Data_To_Imu*gyroout)
{
	accout->X=accin->X;
	accout->Y=accin->Y;
	accout->Z=accin->Z;

	gyroout->X=gyroin->X;
	gyroout->Y=gyroin->Y;
	gyroout->Z=gyroin->Z;
}

/******************************************************************************
����ԭ�ͣ�	float Calculate_LpfFilteringCoefficient(float Time, float Cut_Off)
��    �ܣ�	lpf��ͨ�˲���������
*******************************************************************************/ 
float Calculate_LpfFilteringCoefficient(float Time, float Cut_Off)
{
	return  (Time /( Time + 1/(2.0f*Pi*Cut_Off)));
}
/******************************************************************************
����ԭ�ͣ�	void ACC_LPF_Filter(Data_To_Imu *Accin,Data_To_Imu *Accout,float lpf_fator)
��    �ܣ�	acc��ͨ�˲�
*******************************************************************************/ 
void ACC_LPF_Filter(Data_To_Imu *Accin,Data_To_Imu *Accout,float lpf_fator)
{
	Accout->X = Accout->X + lpf_fator*(Accin->X - Accout->X); 
	Accout->Y = Accout->Y + lpf_fator*(Accin->Y - Accout->Y); 
	Accout->Z = Accout->Z + lpf_fator*(Accin->Z - Accout->Z); 
}

/**
 *kalman_init - �������˲�����ʼ��
 *@kalman���������˲����ṹ��
 *@init_x���������ĳ�ʼֵ
 *@init_p������״̬����ֵ���ķ���ĳ�ʼֵ
 */
void kalman_init(kalman_struct *kalman, float init_x, float init_p,float predict_q,float measure_r)
{
    kalman->x = init_x;//�������ĳ�ʼֵ��������ֵһ�������ֵ
    kalman->p = init_p;//����״̬����ֵ���ķ���ĳ�ʼֵ  Ӱ�첻��
    kalman->A = 1;
    kalman->H = 1;
    kalman->q = predict_q;//Ԥ�⣨���̣��������� ʵ�鷢���޸����ֵ��Ӱ����������
    kalman->r = measure_r;//�������۲⣩�������� ��Ҫ���� ʵ���������ó�
    //����������������ؼ���
}

/**
 *kalman_filter - �������˲���
 *@kalman:�������ṹ��
 *@measure������ֵ
 *�����˲����ֵ
 */
float kalman_filter(kalman_struct *kalman, float measure)
{
    /* Predict */
    kalman->x = kalman->A * kalman->x;//%x�������������һ��ʱ���ĺ������ֵ��������Ϣ����
    kalman->p = kalman->A * kalman->A * kalman->p + kalman->q;  /*������������� p(n|n-1)=A^2*p(n-1|n-1)+q */

    /* Measurement */
    kalman->gain = kalman->p * kalman->H / (kalman->p * kalman->H * kalman->H + kalman->r);
    kalman->x = kalman->x + kalman->gain * (measure - kalman->H * kalman->x);//���ò������Ϣ���ƶ�x(t)�Ĺ��ƣ�����������ƣ����ֵҲ�������
    kalman->p = (1 - kalman->gain * kalman->H) * kalman->p;//%������������

    return kalman->x;
}

/******************/
//���������ݻ����˲����� �Ӽ�����Kalman�˲�
//����������˲������ת�������õ��ļ��ٶ�(g/m^2)�����ٶ�����(rad/s)
//���룺ԭʼ���� �궨�õ�����ƫ
//�˲���Ȳ��˹��󣬷���Ӵ�������ʱ�����Ͷ�̬ЧӦ
void MPU6050_Data_Read_Analys_Kalman(Data_To_Imu *GYRO,Data_To_Imu *Acc,Origial_DATA *rawacc,Origial_DATA *rawgyro)
{
	u8 j=0;
	float a=0.3f,b=0.7f;
	static u8 counter=0;//��������
	float temp4=0,temp5=0,temp6=0;
	static float ACC_TEMP[3]={0};
	static int16_t BUF4[gyrofilter_Num]={0},BUF5[gyrofilter_Num]={0},BUF6[gyrofilter_Num]={0};

/*******************�������˲�*****************************/
	Acc->X=kalman_filter(&KalmanfilterAccx,rawacc->X);
//	Acc->Y=kalman_filter(&KalmanfilterAccy,rawacc->Y);
	Acc->Z=kalman_filter(&KalmanfilterAccz,rawacc->Z);
	
//	Acc->X=a*Sensor_Acc.X+b*ACC_TEMP[0];
//	ACC_TEMP[0]=Acc->X;

	Acc->Y=a*Sensor_Acc.Y+b*ACC_TEMP[1];
//	ACC_TEMP[1]=Acc->Y;

//	Acc->Z=a*Sensor_Acc.Z+b*ACC_TEMP[2];
//	ACC_TEMP[2]=Acc->Z;

	 BUF4[counter]=rawgyro->X;
	 BUF5[counter]=rawgyro->Y;
	 BUF6[counter]=rawgyro->Z;
	
	for(j=0;j<gyrofilter_Num;j++)
	{
		temp4+= BUF4[j];
		temp5+= BUF5[j];
		temp6+= BUF6[j];
	}
	
	 Acc->X/=acc_denominator;//g/m^2
	 Acc->Y/=acc_denominator;
	 Acc->Z/=acc_denominator;

	 GYRO->X=temp4/gyroscale;//rad/s
	 GYRO->Y=temp5/gyroscale;
	 GYRO->Z=temp6/gyroscale;
	
	counter++;
	if(counter==gyrofilter_Num) counter=0;
	
//	printf("%7.3f\t %7.3f\t %7.3f\t %7.3f\t %7.3f\t %7.3f\r\n", Acc->X, Acc->Y, Acc->Z, GYRO->X, GYRO->Y, GYRO->Z);
}
/*IIR_FILTER*/
//���ܣ����������ݽ��д���IIR�˲� 
//������in[] ���봰������ out[]����������� datain ��������
//���أ��˲�ֵ
float IIR_FILTER(float in[],float out[],float datain)
{
	double tempf1=0.0f,tempf2=0.0f;
	unsigned char i=0;
	
	for(i=IIR_FILTER_NUM;i>=1;i--)
	{
		in[i]=in[i-1];
		out[i]=out[i-1];
	}
	
	in[0]=datain;
	
	for(i=0;i<IIR_FILTER_NUM;i++)
	{
		tempf1+=IIR_FILTER_PARA_B[i]*in[i];
		if(i>=1)
		{
			tempf2+=IIR_FILTER_PARA_A[i]*out[i];
		}
	}
	out[0]=tempf1-tempf2;
	return out[0];
}

//
void Imu_data_Prepare(void)
{
	#if USE_KALMAN
	{
		MPU6050_Read_To_Use_(&Sensor_Acc,&Sensor_Gyro,&OffSet);//��ȡ���ݣ��궨
		MPU6050_Data_Read_Analys_Kalman(&velocity,&acc,&Sensor_Acc,&Sensor_Gyro);//kalman�Ӵ��ڻ����˲�
	}
	#endif
			
	#if USE_IIR
	{
		MPU6050_Read_To_Use_(&Sensor_Acc,&Sensor_Gyro,&OffSet);//��ȡ���ݣ��궨
//		Mpu6050_IIR_Filter(&velocity,&acc,&Sensor_Acc,&Sensor_Gyro);
	}
	#endif
			
	#if USE_LPF
	{
		MPU6050_Read_To_Use_(&Sensor_Acc,&Sensor_Gyro,&OffSet);//��ȡ���ݣ��궨
//		MPU6050_Data_Read_Analys(&velocity,&acc,&Sensor_Acc,&Sensor_Gyro);
		MPU6050_Data_MoveLpffilter(&velocity,&acc,&Sensor_Acc,&Sensor_Gyro);	
	}
	#endif
			
	MPU6050_Data_Exchange(&acc,&velocity,&ACC,&VELOCITY);
}

//void Mpu6050_IIR_Filter(Data_To_Imu *GYRO,Data_To_Imu *Acc,
//												Origial_DATA*rawacc,Origial_DATA*rawgyro)
//{
//	static float acc_filter_buff1[IIR_FILTER_NUM+1]={0};
//	static float acc_filter_buff2[IIR_FILTER_NUM+1]={0};
//	static float acc_filter_buff3[IIR_FILTER_NUM+1]={0};
//	
//	static float acc_avg_buff1[IIR_FILTER_NUM+1]={0};
//	static float acc_avg_buff2[IIR_FILTER_NUM+1]={0};
//	static float acc_avg_buff3[IIR_FILTER_NUM+1]={0};
//	
//	static unsigned char i=0,j=0,k=0;
//	static int16_t acc_raw[3][accfilter_Num]={0},gyro_raw[3][gyrofilter_Num]={0};
//	static int32_t acc_sum[3]={0},gyro_sum[3]={0};
//	static float acc_avg[3]={0},gyro_avg[3]={0};

//	for(i=0;i<3;i++)
//	{
//		if(i==0)
//		{
//				acc_sum[i]-=acc_raw[i][j];
//				acc_raw[i][j]=rawacc->X;
//				acc_sum[i]+=acc_raw[i][j];
//				acc_avg[i]=acc_sum[i]/ACCDIVD;//m/s^2
//				
//				gyro_sum[i]-=gyro_raw[i][k];
//				gyro_raw[i][k]=rawgyro->X;
//				gyro_sum[i]+=gyro_raw[i][k];
//				gyro_avg[i]=gyro_sum[i]/GYRODIVD;
//				
//			Acc->X=acc_avg[i]/*IIR_FILTER(acc_avg_buff1,acc_filter_buff1,acc_avg[i])*/;
//			GYRO->X=gyro_avg[i];
//		}
//		else if(i==1)
//		{
//				acc_sum[i]-=acc_raw[i][j];
//				acc_raw[i][j]=rawacc->Y;
//				acc_sum[i]+=acc_raw[i][j];
//				acc_avg[i]=acc_sum[i]/ACCDIVD;
//				
//				gyro_sum[i]-=gyro_raw[i][k];
//				gyro_raw[i][k]=rawgyro->Y;
//				gyro_sum[i]+=gyro_raw[i][k];
//				gyro_avg[i]=gyro_sum[i]/GYRODIVD;
//				
//			Acc->Y=acc_avg[i]/*IIR_FILTER(acc_avg_buff2,acc_filter_buff2,acc_avg[i])*/;
//			GYRO->Y=gyro_avg[i];
//		}
//		else if(i==2)
//		{

//				acc_sum[i]-=acc_raw[i][j];
//				acc_raw[i][j]=rawacc->Z;
//				acc_sum[i]+=acc_raw[i][j];
//				acc_avg[i]=acc_sum[i]/ACCDIVD;
//				
//				gyro_sum[i]-=gyro_raw[i][k];
//				gyro_raw[i][k]=rawgyro->Z;
//				gyro_sum[i]+=gyro_raw[i][k];
//				gyro_avg[i]=gyro_sum[i]/GYRODIVD;
//				
//			Acc->Z=acc_avg[i]/*IIR_FILTER(acc_avg_buff3,acc_filter_buff3,acc_avg[i])*/;
//			GYRO->Z=gyro_avg[i];
//		
//		}
//	}
//	if(++j==accfilter_Num)	j=0;
//	if(++k==gyrofilter_Num)	k=0;
//}

/*���ڻ�����ֵ�˲�*/
int16_t MoveLpfFilter(int16_t datain,MOVELPFFILTER *filter)
{
	if(filter->fullflag)
	{
		filter->rawsum -= filter->rawbuffer[filter->index];//ȥ��tail
		if(myabs(datain) > (2*myabs(filter->rawbuffer[filter->index])))
		{
			datain = 2*myabs(filter->rawbuffer[filter->index]);
		}
		filter->rawbuffer[filter->index] = datain;
		filter->rawsum += filter->rawbuffer[filter->index];//����head
		
		if(++filter->index >= MOCEFILTERLEN)
	    {
		   filter->index=0;
		   filter->fullflag=1;
	    }
	   
	   return filter->rawsum/MOCEFILTERLEN;
	}
	else
	{
		filter->rawsum -= filter->rawbuffer[filter->index];//ȥ��tail
		filter->rawbuffer[filter->index] = datain;
		filter->rawsum += filter->rawbuffer[filter->index];//����head
		
		if(++filter->index>=MOCEFILTERLEN)
		{
			filter->index=0;
			filter->fullflag=1;
		}
		
		return filter->rawbuffer[filter->index-1];
	}
}

MOVELPFFILTER AXFilter,AYFilter,AZFilter,GyroXFilter,GyroYFilter,GyroZFilter;
/******************/
//˫������ֵ�˲�����
//����������˲������ת�������õ��ļ��ٶ�(g/m^2)�����ٶ�����(rad/s)
//���룺ԭʼ���� �궨�õ�����ƫ
//�˲���Ȳ��˹��󣬷���Ӵ�������ʱ�����Ͷ�̬ЧӦ
void MPU6050_Data_MoveLpffilter(Data_To_Imu *GYRO,Data_To_Imu *Acc,
								Origial_DATA *rawacc,Origial_DATA *rawgyro)
{
	Acc->X = MoveLpfFilter(rawacc->X,&AXFilter)/acc_denominator;
	Acc->Y = MoveLpfFilter(rawacc->Y,&AYFilter)/acc_denominator;
	Acc->Z = MoveLpfFilter(rawacc->Z,&AZFilter)/acc_denominator;
	
	GYRO->X = MoveLpfFilter(rawgyro->X,&GyroXFilter)/gyro_denominator;
	GYRO->Y = MoveLpfFilter(rawgyro->Y,&GyroYFilter)/gyro_denominator;
	GYRO->Z = MoveLpfFilter(rawgyro->Z,&GyroZFilter)/gyro_denominator;
}
