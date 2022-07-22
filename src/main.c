




#include "includes.h"


const char* g_build_time_str = "Buildtime :"__DATE__" "__TIME__;   //获得编译时间
uint16_t g_task_id;   //每一个位对应一个任务，为1表示需要启动任务，在任务中清零该位



////800ms 看门狗
//static void iwdog_init(void)
//{
//	fwdgt_write_enable();
//	fwdgt_config(0xfff,FWDGT_PSC_DIV8);    //设置分配系数,最长819ms
//	
//	fwdgt_enable(); //使能看门狗
//}


//static  void iwdog_feed(void)
//{
////	if(mcu_reboot)  //设置mcu重启，不喂狗。2021-12-17增加
////		return ;
//	fwdgt_counter_reload();
//}




static void BoardInit(void)
{
	etError   error;       // error code 
	uint32_t      serialNumber;// serial number 
	float        temperature; // temperature [°C] 
	float        humidity;    // relative humidity [%RH] 

	
	//0. 中断分组初始化
	//NVIC_SetPriorityGrouping(4);  //均为4个等级
	nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);
	
	//0.1 复用功能模块通电
    rcu_periph_clock_enable(RCU_AF);
	
	//只保留sw接口，其他用于GPIO端口
	gpio_pin_remap_config(GPIO_SWJ_SWDPENABLE_REMAP, ENABLE);
	
	//1.串口初始化
	//#define DEBUG_COM_NUM 0   //调试串口号
	//#define TOCPU_COM_NUM 1   //与cpu通信的串口
	gd_eval_com_init(DEBUG_COM_NUM);  //用于调试
//	gd_eval_com_init(TOCPU_COM_NUM);  //用于与cpu数据通信,改到cpu上电后再初始化
	
	//2.systick 初始化
	SystickConfig();
	
	//sht30温度传感器,使用8位地址！！！
	SHT3X_Init(0x44<<1); // Address: 0x44 = Sensor on EvalBoard connector   pin2 接地
                    //          0x45 = Sensor on EvalBoard  pin2 接vcc
	
	//激光控制初始化
	laser_control_init();
	
	dev_status_get_init();
	
	error = SHT3x_ReadSerialNumber(&serialNumber); 
	if(error != NO_ERROR){} // do error handling here 
	   
	  // demonstrate a single shot measurement with clock-stretching 
	error = SHT3X_GetTempAndHumi(&temperature, &humidity, REPEATAB_HIGH, MODE_CLKSTRETCH, 50); 
	if(error != NO_ERROR){} // do error handling here  
	   
//	  // demonstrate a single shot measurement with polling and 50ms timeout 
//	error = SHT3X_GetTempAndHumi(&temperature, &humidity, REPEATAB_HIGH, MODE_POLLING, 50); 
//	if(error != NO_ERROR){} // do error handling here
}





int main(void)
{
	uint8_t i;
	const task_t task[TASK_MAX]={btns_scan    //任务1，上电按钮扫描								
							,[1] = laser_run_pwm_task       		//任务2，激光的pwm设置，10ms一次
						//	,[2] = Task_Check_CPU_Run_Status    //任务3，运行状态检测，关机重启控制，这个优先级可以低一点
							,[3] = get_sht30_tmp_task       //任务4，温湿度，电压监控读取任务，2000ms调用一次
							,[4] = Task_Led_Show_Work  //任务5，定时向cpu汇报，500ms一次 //2022-04-21不再主动发送	
					//		,[5] = com3_frame_handle    //无需要
						//	,[14] = iwdog_feed         //最后一个任务喂狗
					//	,0
//						,[15]=Task_Led_Show_Work       //任务16，最后一个任务，让工作led灯闪烁,1s调用一次
					//因为工作灯不能正常使用，所以删除该任务。2021-12-01
	};
	
	
	//1. 初始化
	BoardInit();

	printf("%s\n\r", g_build_time_str);
	printf("BoardInit done! 2022-07-01\n\r");
	
//	Delay1ms(2000);
//	for(i=0;i<32;i++)
//	{
//		printf("i = %d\n\r", i);
//		key_light_leds_control(i,SET);
//		Delay1ms(2000);
//	}

	
	while(1)
	{
		for(i=0;i<TASK_MAX && g_task_id;i++){
			if(g_task_id & (1<<i))   //定时时间到，要执行
			{
				g_task_id &= ~(1<<i);  //对应的位置被清零，等待定时器设置
			
				if(task[i])  //指针不能为空
				{	
					task[i](); //执行该任务
					break;    //一次只执行一个任务，任务靠前的优先级高，任务靠后的优先级低
				}				
			}
		}//end for		
	}
}


