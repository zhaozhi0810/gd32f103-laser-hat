
#ifndef INCLUDES_H
#define INCLUDES_H

//�����ܵ�ͷ�ļ�����

#include <gd32f10x.h>
#include <stdio.h>


//���ܿ��ƺ궨�壺
//#define LCD_PWM   //lcd���ȿ��ƣ�ʹ��pwm�ķ�ʽ
//#define LCD_PWM_HEAT   //LCDʹ��pwm���ȣ�ע�͸ú��ʾ��ʹ��pwm
#define BTNS_USE_INT   //����ɨ��ʹ���жϷ�ʽ

#define LITTLE_ENDIAN

//����lcd����ʱ���м��ȴ���
// #define LCD_HEAT_ENABLE    //����Һ�������ȹ��ܣ�ע��֮���û�м��ȹ���

extern const char* g_build_time_str;
extern uint8_t dev_run_status;   //�豸����״̬����Ƭ�����������еģ���0��ʾ����δ������1��ʾ�����ѿ���



#define DEBUG_COM_NUM 0   //���Դ��ں�
#define TOCPU_COM_NUM 1   //��cpuͨ�ŵĴ���


//-- Enumerations ------------------------------------------------------------- 
// Error codes 
typedef enum{ 
  NO_ERROR       = 0x00, // no error 
  ACK_ERROR      = 0x01, // no acknowledgment error 
  CHECKSUM_ERROR = 0x02, // checksum mismatch error 
  TIMEOUT_ERROR  = 0x04, // timeout error 
  PARM_ERROR     = 0x80, // parameter out of range error 
}etError;




#include "iic_app.h"
#include "systick.h"     //��ʱ����
#include "uart.h"        //���ڴ���
//#include "gpios.h"       //�ߵ͵�ƽ���Ƶ�

#include "task_cfg.h"    //������صĺ궨��
#include "uart_conect_cpu_handler.h"   //��cpuͨ�ŵĴ��ڽ��շ��ʹ�����Ϊ������ģ��ͨ�ţ�
#include "uart_debug_handle.h"        //���Դ��ڵĽ��մ���


#include "pwm.h" 
#include "sht30.h"
#include "btns.h"
#include "dev_run_status.h"    //�豸����״̬����

#include "wrok_leds.h"
#include "adc_vol.h"
#endif

