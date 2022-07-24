

#ifndef __POWER_MANAGER_H__
#define __POWER_MANAGER_H__

void PowerManager_init(void);
//单片机断电
void PowerManager_Mcu_Poweroff(void);


//usb电源是否连接？  1表示连接，0表示没有连接
uint8_t get_usb_PowerStatus(void);


//3.3v电源是否输出？  1表示输出，0表示没有输出
uint8_t get_BT3V_PowerStatus(void);


//获得系统运行的状态
//返回值大于0表示充电中，0表示未充电
uint8_t get_bat_charge_status(void);

//锂电池的5v升压输出,激光需要5v
void output_5v_disable(void);

//锂电池的5v升压输出
void output_5v_enable(void);



//外设电源3.3v升压输出
void output_BT3V_enable(void);

//外设电源3.3v输出关闭
void output_BT3V_disable(void);


//外部电源连接，正在充电返回1，返回2，表示已充满，没充电则0
uint8_t is_power_charge(void);



//音频芯片省电控制
void set_MAX9700_enable(void);

//音频芯片省电控制
void set_MAX9700_shutdown(void);
#endif



