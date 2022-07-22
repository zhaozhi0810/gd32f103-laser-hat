
#ifndef INCLUDES_H
#define INCLUDES_H

//用于总的头文件包含

#include <gd32f10x.h>
#include <stdio.h>


//功能控制宏定义：
//#define LCD_PWM   //lcd亮度控制，使用pwm的方式
//#define LCD_PWM_HEAT   //LCD使用pwm加热，注释该宏表示不使用pwm
#define BTNS_USE_INT   //按键扫描使用中断方式

#define LITTLE_ENDIAN

//允许lcd低温时进行加热处理
// #define LCD_HEAT_ENABLE    //开启液晶屏加热功能，注释之后就没有加热功能

extern const char* g_build_time_str;
extern uint8_t dev_run_status;   //设备运行状态，单片机总是在运行的！！0表示激光未开启，1表示激光已开启



#define DEBUG_COM_NUM 0   //调试串口号
#define TOCPU_COM_NUM 1   //与cpu通信的串口


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
#include "systick.h"     //延时函数
#include "uart.h"        //串口处理
//#include "gpios.h"       //高低电平控制的

#include "task_cfg.h"    //任务相关的宏定义
#include "uart_conect_cpu_handler.h"   //与cpu通信的串口接收发送处理（改为与蓝牙模块通信）
#include "uart_debug_handle.h"        //调试串口的接收处理


#include "pwm.h" 
#include "sht30.h"
#include "btns.h"
#include "dev_run_status.h"    //设备运行状态控制

#include "wrok_leds.h"
#include "adc_vol.h"
#endif

