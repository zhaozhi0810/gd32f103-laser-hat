

#include "includes.h"
#include "iic_app.h" 
#include "i2c.h" 


float        g_temperature; // temperature [°C] 
//float        g_humidity;    // relative humidity [%RH]  
 
static uint8_t g_temp_control = 1;   //温度控制灯的亮度，1表示用温度控制，0表示不用温度控制

 
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
 温湿度获取函数               
函数原型: SHT30_read_result(uint8_t addr);
功能: 用来接收从器件采集并合成温湿度,使用的是7位地址
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
	//IIC_SendByte(addr<<1 | read);//写7位I2C设备地址加0作为写取位,1为读取位
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
	
	
		tem = ((buff[0]<<8) | buff[1]);//温度拼接
//		hum = ((buff[3]<<8) | buff[4]);//湿度拼接
		
		/*转换实际温度*/
		g_temperature= (175.0*(float)tem/65535.0-45.0) ;// T = -45 + 175 * tem / (2^16-1)
		//Humidity= (100.0*(float)hum/65535.0);// RH = hum*100 / (2^16-1)
		
//		if((Temperature>=-20)&&(Temperature<=125)&&(Humidity>=0)&&(Humidity<=100))//过滤错误数据
//		{
//			humiture[0]=Temperature;
//			humiture[2]=Humidity;
//			sprintf(humiture_buff1,"%6.2f*C %6.2f%%",Temperature,Humidity);//111.01*C 100.01%（保留2位小数）
//		}
//		printf("温湿度：%6.2f*C\r\n",g_temperature);
	}
	
	IIC_Start(IIC1_INDEX);  //IIC_Start();
	ret = I2c_WriteByte(IIC1_INDEX,0x44<<1);  //地址
	ret = I2c_WriteByte(IIC1_INDEX,0x2C);  //地址	
	ret = I2c_WriteByte(IIC1_INDEX,0x06);  //地址	
	IIC_Stop(IIC1_INDEX);//IIC_Stop();	
	
	//printf("ERROR: 温湿度\n");
//	hum=0;
//	tem=0;
}


//获取温湿度的任务，100ms读取进入次
void get_sht30_tmp_task(void)
{
//	etError   error;       // error code 
//	float        temperature; // temperature [°C] 
//	float        humidity;    // relative humidity [%RH] 
	// read measurment buffer 
	
//	error = SHT3X_ReadMeasurementBuffer(&g_temperature, &g_humidity); 

	SHT30_read_result(0x44);
	
//	return ;
//	if(error == NO_ERROR) 
	if(g_temp_control)  //受温度控制吗？？
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
		else if(g_temperature < 48.0)  //50度了
		{
			pwm_all_change(20);		
		}
		else if(g_temperature < 50.0)  //50度了
		{
			pwm_all_change(10);		
		}
		else //50度以上了
			pwm_all_change(0);
	}
//	else
//	{
//		DBG_PRINTF("ERROR:SHT3X_ReadMeasurementBuffer\r\n");
//	}
}

