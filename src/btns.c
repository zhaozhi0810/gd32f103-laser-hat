

#include "includes.h"


/*
	按键扫描程序
	
	PA15  按键 1个
	
	
	根据原理图，按键可以使单片机通电，在电池供电的情况下
	
*/








//配置所有的按键，其实只有一个按键
void gd_all_keys_init(void)
{	
	//1. 时钟使能
	rcu_periph_clock_enable(RCU_GPIOA);
		
	//2. 设置为输入模式	
	gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_2MHZ, GPIO_PIN_15);

	
	// 清中断标志
//    exti_interrupt_flag_clear(EXTI_15);
//	exti_interrupt_disable(EXTI_15);    //禁止外部中断
//	nvic_irq_disable(EXTI5_9_IRQn);
}



//void btn_init_irq(void)
//{
//	//1. 时钟使能
//	rcu_periph_clock_enable(RCU_GPIOA);
//		
//	//2. 设置为输入模式	
//	gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_2MHZ, GPIO_PIN_15);
//	
//	// 设置优先级
//    nvic_irq_enable(EXTI5_9_IRQn, 2U, 2U);
//    
//    // 设置EXTI触发源
//    gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOA, GPIO_PIN_SOURCE_15);

//    // 下降沿中断
//    exti_init(EXTI_15, EXTI_INTERRUPT, EXTI_TRIG_BOTH); //双边缘触发
//    // 清中断标志
//    exti_interrupt_flag_clear(EXTI_15);
//	exti_interrupt_enable(EXTI_15);
//	
//	
//}



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
	static uint16_t pressCnt = 0;
//	static uint8_t release = 0;   //松开了吗？ 0表示已经松开，1表示没有
	
	system_run_status_t status;

	status = get_system_run_status();
	
	
	if(gd_all_keys_state_get()==  SET)  //被按下
	{
		if(pressCnt<=300) //之前是松开的
		{
			pressCnt++;
			if(pressCnt>300)  //3s
			{
			//	release = 1;   //记录状态：  已经按下，防止重复检测
				if(status <= DEV_POWEROFF)  //已经关机了，执行开机
				{
					MY_PRINTF("%s %d\r\n",__FUNCTION__,__LINE__);
					system_power_on();
				}
				else //开机状态，则执行关机
				{
					MY_PRINTF("%s %d\r\n",__FUNCTION__,__LINE__);
					system_power_off();
					//cpu重启
				}
				
//				if(pressCnt > 10000)
//					pressCnt = 10000; //防止长按越界
			}
		}		
	}
	else{  //松开
		if(pressCnt)
		{
			MY_PRINTF("%s %d btn release\r\n",__FUNCTION__,__LINE__);		
			pressCnt = 0;
		}
	//	release = 0;   //记录状态：  已经松开
	}	
}











