

#include "includes.h"


//状态灯,橙绿灯的控制
//共8个led，一组4个 GPA0对应GREEN  GPA1对应ORANGE
//高电平点亮
/*
电量报警：电压不足3.6V，红灯慢闪
正常运行/充电完成：绿灯常亮
充电：绿灯慢闪
设备故障：红灯常亮
*/
//

//指示灯的初始化
void Led_Show_Work_init(void)
{
	//时钟使能
	rcu_periph_clock_enable(RCU_GPIOA);	
		
	gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ, GPIO_PIN_0 | GPIO_PIN_1);	
	//熄灭
	gpio_bit_reset(GPIOA, GPIO_PIN_1);	
	gpio_bit_reset(GPIOA, GPIO_PIN_0);   //绿灯
}

//绿色指示灯点亮
void GreenLed_Show_Work_On(void)
{
	gpio_bit_set(GPIOA, GPIO_PIN_0);
}

//绿色指示灯熄灭
void GreenLed_Show_Work_Off(void)
{
	gpio_bit_reset(GPIOA, GPIO_PIN_0);
}


//红色指示灯点亮
void OrangeLed_Show_Work_On(void)
{
	gpio_bit_set(GPIOA, GPIO_PIN_1);
}


//红色指示灯熄灭
void OrangeLed_Show_Work_Off(void)
{
	gpio_bit_reset(GPIOA, GPIO_PIN_1);
}


//这个值与任务的间隔有关！！！！！
//#define FLASH_FRQ  5   //50ms进入一次，则500ms改变一次亮灭，250ms一个状态
#define FLASH_FRQ  10   //100ms进入一次，则1000ms改变一次亮灭，500ms一个状态
//工作灯闪烁
/*
	10 ms进入一次
	按照控制时序要求来！！！
	用全局变量控制灯的颜色，和快闪慢闪
*/
void Task_Led_Show_Work(void)
{
	static uint8_t n = 0;
//	static system_run_status_t status_rec = DEV_POWEROFF;  //记录之前的状态
	system_run_status_t status;
	uint8_t ret;
	
	n++;
	
	ret = is_power_charge();
//	printf("Task_Led_Show_Work ret = %d\r\n",ret);
	
	if(ret > 0)   //充电中，或已充满
	{
		if(ret == 1)  //充电中
		{
			OrangeLed_Show_Work_Off(); //红灯肯定要关闭
			
			if(n<FLASH_FRQ)
			{
				GreenLed_Show_Work_On();
			}
			else if(n<(FLASH_FRQ*2))
			{
				GreenLed_Show_Work_Off();
			}
			else  //n的值
				n = 0;
		}
		else  //已充满
		{
			GreenLed_Show_Work_On();   //绿灯常亮
			OrangeLed_Show_Work_Off();
		}		
	}
	else  //无外接电源
	{
		status = get_system_run_status();
		
		//关机状态下不控制激光了
		if(status == DEV_POWEROFF)
		{
			n = 0;
			GreenLed_Show_Work_Off();   //确保激光全部关闭
			OrangeLed_Show_Work_Off();  //关机状态下，灯熄灭
			return;
		}
		else
		{
			//if(status_rec != status) //现在的状态与记录的状态不一样，说明状态发生了变化
	//		{
	//			if(status_rec != DEV_POWEROFF)  //已经记录到是开机后了
	//			{
	//				n = 0;   // 主要是从0开始一个新的状态，好像也没必要，感觉影响不大
	//			}
	//			status_rec = status;
	//		}
			
//			if(status == DEV_CHARGE)  //充电中，绿灯慢闪
//			{
//				OrangeLed_Show_Work_Off(); //红灯肯定要关闭
//				
//				if(n<FLASH_FRQ)
//				{
//					GreenLed_Show_Work_On();
//				}
//				else if(n<(FLASH_FRQ*2))
//				{
//					GreenLed_Show_Work_Off();
//				}
//				else  //n的值
//					n = 0;
//				
//			}
//			else if(status == DEV_CHARGE_OK)  //充电完成，绿灯常亮
//			{
//				GreenLed_Show_Work_On();   //绿灯常亮
//				OrangeLed_Show_Work_Off();
//			}
//			else 
			if(status == DEV_RUN_NORMAL)  //正常运行，绿灯常亮
			{
				GreenLed_Show_Work_On();   //绿灯常亮
				OrangeLed_Show_Work_Off();
			}
			else if(status == DEV_VOL_LE36)  //电压不足3.6V，红灯慢闪
			{
				GreenLed_Show_Work_Off(); //绿灯肯定要关闭
				
				if(n<FLASH_FRQ)
				{
					OrangeLed_Show_Work_On();
				}
				else if(n<(FLASH_FRQ*2))
				{
					OrangeLed_Show_Work_Off();
				}
				else  //n的值
					n = 0;
			}
			else if(status == DEV_BUG)  //红灯常亮
			{
				GreenLed_Show_Work_Off();
				OrangeLed_Show_Work_On();   //红灯常亮
			}
		}
	}
	
}

