

#ifndef __DEV_RUN_STATUS_H__
#define __DEV_RUN_STATUS_H__

typedef enum {
	DEV_BUG  =  1 ,     //故障
	DEV_VOL_LE30  = 2,    //电压低于3.0	
	DEV_POWEROFF = 3,      //关机
	DEV_VOL_LE36  =  4,   //电压低于3.6	
	DEV_RUN_NORMAL = 5,  //正常运行
	DEV_CHARGE  =  6,   //充电
	DEV_CHARGE_OK  =  7   //充电完成
}system_run_status_t;



//修改系统运行的状态
void set_system_run_status(system_run_status_t status);

//获得系统运行的状态
system_run_status_t get_system_run_status(void);

//开机
void system_power_on(void);
//关机
void system_power_off(void);

/*
	设备运行状态的获取初始化
	1. 充电状态 PC6充电中，低电平有效， PB15 升压模式控制，高电平有效，低电平表示休眠模式
	2. 头盔佩戴的状态
	3. 电池电压的状态
*/
void dev_status_get_init(void);

//获得系统运行的状态
//返回值大于0表示充电中，0表示未充电
uint8_t get_bat_charge_status(void);

//锂电池的5v升压输出,激光需要5v
void output_5v_disable(void);

//锂电池的5v升压输出
void output_5v_enable(void);


#endif
 

