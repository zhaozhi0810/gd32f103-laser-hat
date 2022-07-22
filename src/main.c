




#include "includes.h"


const char* g_build_time_str = "Buildtime :"__DATE__" "__TIME__;   //��ñ���ʱ��
uint16_t g_task_id;   //ÿһ��λ��Ӧһ������Ϊ1��ʾ��Ҫ���������������������λ



////800ms ���Ź�
//static void iwdog_init(void)
//{
//	fwdgt_write_enable();
//	fwdgt_config(0xfff,FWDGT_PSC_DIV8);    //���÷���ϵ��,�819ms
//	
//	fwdgt_enable(); //ʹ�ܿ��Ź�
//}


//static  void iwdog_feed(void)
//{
////	if(mcu_reboot)  //����mcu��������ι����2021-12-17����
////		return ;
//	fwdgt_counter_reload();
//}




static void BoardInit(void)
{
	etError   error;       // error code 
	uint32_t      serialNumber;// serial number 
	float        temperature; // temperature [��C] 
	float        humidity;    // relative humidity [%RH] 

	
	//0. �жϷ����ʼ��
	//NVIC_SetPriorityGrouping(4);  //��Ϊ4���ȼ�
	nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);
	
	//0.1 ���ù���ģ��ͨ��
    rcu_periph_clock_enable(RCU_AF);
	
	//ֻ����sw�ӿڣ���������GPIO�˿�
	gpio_pin_remap_config(GPIO_SWJ_SWDPENABLE_REMAP, ENABLE);
	
	//1.���ڳ�ʼ��
	//#define DEBUG_COM_NUM 0   //���Դ��ں�
	//#define TOCPU_COM_NUM 1   //��cpuͨ�ŵĴ���
	gd_eval_com_init(DEBUG_COM_NUM);  //���ڵ���
//	gd_eval_com_init(TOCPU_COM_NUM);  //������cpu����ͨ��,�ĵ�cpu�ϵ���ٳ�ʼ��
	
	//2.systick ��ʼ��
	SystickConfig();
	
	//sht30�¶ȴ�����,ʹ��8λ��ַ������
	SHT3X_Init(0x44<<1); // Address: 0x44 = Sensor on EvalBoard connector   pin2 �ӵ�
                    //          0x45 = Sensor on EvalBoard  pin2 ��vcc
	
	//������Ƴ�ʼ��
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
	const task_t task[TASK_MAX]={btns_scan    //����1���ϵ簴ťɨ��								
							,[1] = laser_run_pwm_task       		//����2�������pwm���ã�10msһ��
						//	,[2] = Task_Check_CPU_Run_Status    //����3������״̬��⣬�ػ��������ƣ�������ȼ����Ե�һ��
							,[3] = get_sht30_tmp_task       //����4����ʪ�ȣ���ѹ��ض�ȡ����2000ms����һ��
							,[4] = Task_Led_Show_Work  //����5����ʱ��cpu�㱨��500msһ�� //2022-04-21������������	
					//		,[5] = com3_frame_handle    //����Ҫ
						//	,[14] = iwdog_feed         //���һ������ι��
					//	,0
//						,[15]=Task_Led_Show_Work       //����16�����һ�������ù���led����˸,1s����һ��
					//��Ϊ�����Ʋ�������ʹ�ã�����ɾ��������2021-12-01
	};
	
	
	//1. ��ʼ��
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
			if(g_task_id & (1<<i))   //��ʱʱ�䵽��Ҫִ��
			{
				g_task_id &= ~(1<<i);  //��Ӧ��λ�ñ����㣬�ȴ���ʱ������
			
				if(task[i])  //ָ�벻��Ϊ��
				{	
					task[i](); //ִ�и�����
					break;    //һ��ִֻ��һ����������ǰ�����ȼ��ߣ����񿿺�����ȼ���
				}				
			}
		}//end for		
	}
}


