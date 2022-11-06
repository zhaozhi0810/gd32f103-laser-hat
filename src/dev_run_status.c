
#include "includes.h"
#include "flash_record.h"
/*
	主要是设备的运行状态，及状态监控

	1. 关机状态时，按下开关后（3s以上），准备开机
		1.1 准备开机后，检测是否佩戴（红外管），如果没有佩戴，则不点亮激光，否则点亮激光
		1.2 激光开启时，如果检测到未佩戴，需关闭，同时倒计时，2分钟内再次佩戴，即正常使用，2分钟内未佩戴，则关机
		1.3 激光开启时，计入计时阶段，最长使用时间不得超过28分钟
	2. 开机状态，按下开关后（3s以上），准备关机


电量报警：电压不足3.6V，红灯慢闪,  电压不足3.0V则关机
正常运行/充电完成：绿灯常亮
充电：绿灯慢闪
设备故障：红灯常亮

*/





static system_run_status_t g_run_status = DEV_POWEROFF;  //系统运行的状态

uint32_t start_count  = 0;


//修改系统运行的状态
void set_system_run_status(system_run_status_t status)
{
	g_run_status = status;
}


//获得系统运行的状态
system_run_status_t get_system_run_status(void)
{
	return g_run_status ;
}





//开机
void system_power_on(void)
{
	uint16_t vol;
	uint8_t ret = 0;
	flash_rcd_t config;
	MY_PRINTF("%s %d\r\n",__FUNCTION__,__LINE__);
	
	ret = read_flash_config(&config);
	if(ret == 255)
	{
		printf("ret == 255\r\n");
		config.start_count = 1;
		start_count = 1;
		write_flash_config(&config);
	}
	else{
		printf("ret == %d\r\n",ret);
		config.start_count++;
		start_count = config.start_count;
		write_flash_config(&config);
	}
	
	if(start_count > 3000)  //大于3000次不让启动
		return;
	
	
	if(is_power_charge() && !charging_enable_start_laser) //充电不允许开机
	{
		printf("power_charging, system disable to start!!!!!!\r\n");
		return;
	}
	
	vol = ADCgetBatVol();   //获得电压值
	MY_PRINTF("%s %d vol = %d\r\n",__FUNCTION__,__LINE__,vol);
	if(vol <= 30)  //电压太低了，不启动
	{
		DBG_PRINTF("ERROR:power is too low vol = %d\n",vol);
		return ;
	}
	
	// 2. 设置系统状态为正常启动状态
	set_system_run_status(DEV_RUN_NORMAL);
	
	// 3. 红外检测开关开始工作
//	ir_detect_init();   //红外检测初始化
	laser_enable(0);   //激光全部关闭，
	
	// 4. 外设3.3v电源开启
	output_BT3V_enable();
	
	//5. 5v电源开启，红外需要5v电源
	output_5v_enable();
	
	//5. 红外定时器开启
	IR_Recv_Timer_Control(1);
	
	//6.激光pwm开启
	Laser_Pwm_Timer_Control(1);
}


//关机
void system_power_off(void)
{
	PowerManager_Mcu_Poweroff();   //单片机断电
		
	//没有连接usb电源的话，后面就不会处理了。
	MY_PRINTF("%s %d\r\n",__FUNCTION__,__LINE__);
	set_system_run_status(DEV_POWEROFF);  //系统状态修改为关机
	
	pwm_all_change(0);
	laser_enable(0);   //激光全部关闭，5v的电源被关闭  包括output_5v_disable(void)
//	ir_detect_off();   //红外检测关闭
	
	//4. 外设3.3v电源关闭
	output_BT3V_disable();
	
	//5. 红外定时器关闭
	IR_Recv_Timer_Control(0);
	
	//6.激光pwm关闭
	Laser_Pwm_Timer_Control(0);
	
	//7.关机后清零照射时间
	clear_laser_light_times();
		
}










/*
	设备运行状态的获取初始化
	1. 充电状态 PC6充电中，低电平有效， PB15 升压模式控制，高电平有效，低电平表示休眠模式
	2. 头盔佩戴的状态
	3. 电池电压的状态
*/
void dev_status_get_init(void)
{
	
		
	//3. 红外对射管的初始化	
}




//设备运行时的状态切换 200ms进入一次吧
void dev_run_status_monitor_task(void)
{
	
}

