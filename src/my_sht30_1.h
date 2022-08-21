
#ifndef __SHT30_H__
#define __SHT30_H__

#include <gd32f10x.h>

extern float        g_temperature; // temperature [°C] 
extern float        g_humidity;    // relative humidity [%RH] 
//extern uint8_t humiture_buff1[20];
//extern uint8_t humiture_buff2[20];
//extern uint8_t Refresh_SHT30_Data;
//extern uint8_t send_data_fleg;
//extern uint8_t Temperature_L;
//extern uint8_t Humidity_L;
//extern uint8_t Temperature_H;
//extern uint8_t Humidity_H;
 
void SHT30_Init(void);
//void IIC_ACK(void);
//void IIC_NACK(void);
//uint8_t IIC_wait_ACK(void);
//void IIC_Start(void);
//void IIC_Stop(void);
//void IIC_SendByte(uint8_t byte);
//uint8_t IIC_RcvByte(void);
void SHT30_read_result(uint8_t addr);


//获取温湿度的任务，1s读取1次
void get_sht30_tmp_task(void); 



//温度控制激光pwm
void temp_control_enable(void);

//温度不能控制激光pwm
void temp_control_disable(void);

#endif


