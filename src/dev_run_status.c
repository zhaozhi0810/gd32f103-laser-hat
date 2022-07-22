
#include "includes.h"

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





static system_run_status_t g_run_status;  //系统运行的状态

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
	set_system_run_status(DEV_RUN_NORMAL);
}


//关机
void system_power_off(void)
{
	set_system_run_status(DEV_POWEROFF);  //系统状态修改为关机
	laser_enable(0);   //激光全部关闭
}



//锂电池的5v升压输出
void output_5v_enable(void)
{
	gpio_bit_set(GPIOB, GPIO_PIN_15);  //5v输出
}

//锂电池的5v升压输出
void output_5v_disable(void)
{
	gpio_bit_reset(GPIOB, GPIO_PIN_15);  //5v不输出
}



//获得系统运行的状态
//返回值大于0表示充电中，0表示未充电
uint8_t get_bat_charge_status(void)
{
	return !gpio_input_bit_get(GPIOC, GPIO_PIN_6);  //充电时为低电平，所以取反一下
}


/*
	设备运行状态的获取初始化
	1. 充电状态 PC6充电中，低电平有效， PB15 升压模式控制，高电平有效，低电平表示休眠模式
	2. 头盔佩戴的状态
	3. 电池电压的状态
*/
void dev_status_get_init(void)
{
	//0. 用于电压采样的控制器初始化
	ADC_Init();
	
	//2. 充电状态的引脚初始化
	//2.1 充电状态读取
	rcu_periph_clock_enable(RCU_GPIOC);			
	gpio_init(GPIOC, GPIO_MODE_IPU, GPIO_OSPEED_2MHZ, GPIO_PIN_6);	  //PC6设置为输入模式
	
	//2.2 升压5v输出使能，时钟使能
	rcu_periph_clock_enable(RCU_GPIOB);			
	gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ, GPIO_PIN_15);	
	gpio_bit_reset(GPIOB, GPIO_PIN_15);  //5v不输出
	
	//3. 红外对射管的初始化
	
	
	
}




