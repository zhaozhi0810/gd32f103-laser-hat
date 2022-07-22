

#include "includes.h"


/*
	按键扫描程序
	
	PA15  按键 1个
*/



//配置所有的按键
void gd_all_keys_init(void)
{	
	//1. 时钟使能
	rcu_periph_clock_enable(RCU_GPIOA);
		
	//2. 设置为输入模式	
	gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_2MHZ, GPIO_PIN_15);

}


//获取按键的值，0-14位表示按键值，1表示按下，0表示松开
static uint16_t gd_all_keys_state_get(void)
{
	//printf("val = %#x\n\r",val);
	return !gpio_input_bit_get(GPIOA,GPIO_PIN_15);    //因为按下是低电平，松开是高电平，取一下反
}






/*
	开关机按键的扫描
	定时器中断触发扫描，10ms扫一次，30ms
	
	返回值：
		无
*/
void btns_scan(void) // 10ms 调用一次
{
	static int pressCnt = 0;
	
	system_run_status_t status;

	status = get_system_run_status();
	
	
	if(gd_all_keys_state_get()==  SET)  //被按下
	{
		pressCnt++;
		if(pressCnt>300)  //3s
		{
			if(status == DEV_POWEROFF)  //已经关机了，执行开机
			{
				system_power_on();
			}
			else //开机状态，则执行关机
			{
				system_power_off();
			}
			if(pressCnt > 10000)
				pressCnt = 10000; //防止长按越界
		}				
	}
	else{  //松开
		pressCnt = 0;
	}	
}











