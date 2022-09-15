

#include "includes.h"
#include "iic_app.h" 
#include "i2c.h" 


float        g_temperature; // temperature [��C] 
//float        g_humidity;    // relative humidity [%RH]  
 
static uint8_t g_temp_control = 1;   //�¶ȿ��ƵƵ����ȣ�1��ʾ���¶ȿ��ƣ�0��ʾ�����¶ȿ���

 
void SHT30_Init(void)
{
//	uint8_t ret;
	IicApp_Init(IIC1_INDEX);	
	
//	Delay1ms(10);
	


}



void temp_control_enable(void)
{
	g_temp_control = 1;
	printf("temp_control_enable\r\n");
	
}



void temp_control_disable(void)
{
	g_temp_control = 0;
	printf("temp_control_disable\r\n");
}



/*******************************************************************
 ��ʪ�Ȼ�ȡ����               
����ԭ��: SHT30_read_result(uint8_t addr);
����: �������մ������ɼ����ϳ���ʪ��,ʹ�õ���7λ��ַ
********************************************************************/ 
void SHT30_read_result(uint8_t addr)
{
	uint8_t ret;
	uint16_t tem;//,hum;
	uint16_t buff[6];
//	float Temperature=0;
//	float Humidity=0;
	

//	Delay1ms(50);//DelayMs(50);
	
	
	IIC_Start(IIC1_INDEX);  //IIC_Start();
	//IIC_SendByte(addr<<1 | read);//д7λI2C�豸��ַ��0��Ϊдȡλ,1Ϊ��ȡλ
	//if(IIC_wait_ACK()==0)
	ret = I2c_WriteByte(IIC1_INDEX,(addr<<1)|1);
	if(ret == 0)
	{
		buff[0]=IIC_Read_Byte(IIC1_INDEX,1);
		//IIC_ACK();
		buff[1]=IIC_Read_Byte(IIC1_INDEX,0);//buff[1]=IIC_RcvByte();
		//IIC_ACK();
//		buff[2]=IIC_Read_Byte(IIC1_INDEX,1);//buff[2]=IIC_RcvByte();
//		//IIC_ACK();
//		buff[3]=IIC_Read_Byte(IIC1_INDEX,1);//buff[3]=IIC_RcvByte();
//		//IIC_ACK();
//		buff[4]=IIC_Read_Byte(IIC1_INDEX,1);//buff[4]=IIC_RcvByte();
//		//IIC_ACK();
//		buff[5]=IIC_Read_Byte(IIC1_INDEX,0);//buff[5]=IIC_RcvByte();
		//IIC_NACK();
		//IIC_Stop();
		IIC_Stop(IIC1_INDEX);
	
	
		tem = ((buff[0]<<8) | buff[1]);//�¶�ƴ��
//		hum = ((buff[3]<<8) | buff[4]);//ʪ��ƴ��
		
		/*ת��ʵ���¶�*/
		g_temperature= (175.0*(float)tem/65535.0-45.0) ;// T = -45 + 175 * tem / (2^16-1)
		//Humidity= (100.0*(float)hum/65535.0);// RH = hum*100 / (2^16-1)
		
//		if((Temperature>=-20)&&(Temperature<=125)&&(Humidity>=0)&&(Humidity<=100))//���˴�������
//		{
//			humiture[0]=Temperature;
//			humiture[2]=Humidity;
//			sprintf(humiture_buff1,"%6.2f*C %6.2f%%",Temperature,Humidity);//111.01*C 100.01%������2λС����
//		}
//		printf("��ʪ�ȣ�%6.2f*C\r\n",g_temperature);
	}
	
	IIC_Start(IIC1_INDEX);  //IIC_Start();
	ret = I2c_WriteByte(IIC1_INDEX,0x44<<1);  //��ַ
	ret = I2c_WriteByte(IIC1_INDEX,0x2C);  //��ַ	
	ret = I2c_WriteByte(IIC1_INDEX,0x06);  //��ַ	
	IIC_Stop(IIC1_INDEX);//IIC_Stop();	
	
	//printf("ERROR: ��ʪ��\n");
//	hum=0;
//	tem=0;
}


//��ȡ��ʪ�ȵ�����100ms��ȡ�����
void get_sht30_tmp_task(void)
{
//	etError   error;       // error code 
//	float        temperature; // temperature [��C] 
//	float        humidity;    // relative humidity [%RH] 
	// read measurment buffer 
	
//	error = SHT3X_ReadMeasurementBuffer(&g_temperature, &g_humidity); 

	SHT30_read_result(0x44);
	
//	return ;
//	if(error == NO_ERROR) 
	if(g_temp_control)  //���¶ȿ����𣿣�
	{ 
//		MY_PRINTF("%s %d temp = %f \r\n",__FUNCTION__,__LINE__,g_temperature);
	//new temperature and humidity values 
		if(g_temperature < 30.0)
		{
			pwm_all_change(40);
		}
		else if(g_temperature < 35.0)
		{
			pwm_all_change(35);		
		}
		else if(g_temperature < 40.0)
		{
			pwm_all_change(30);		
		}
		else if(g_temperature < 45.0)
		{
			pwm_all_change(25);		
		}
		else if(g_temperature < 48.0)  //50����
		{
			pwm_all_change(20);		
		}
		else if(g_temperature < 50.0)  //50����
		{
			pwm_all_change(10);		
		}
		else //50��������
			pwm_all_change(0);
	}
//	else
//	{
//		DBG_PRINTF("ERROR:SHT3X_ReadMeasurementBuffer\r\n");
//	}
}

