
#include "includes.h"
#include "flash_record.h"
/*
	��Ҫ���豸������״̬����״̬���

	1. �ػ�״̬ʱ�����¿��غ�3s���ϣ���׼������
		1.1 ׼�������󣬼���Ƿ����������ܣ������û��������򲻵������⣬�����������
		1.2 ���⿪��ʱ�������⵽δ�������رգ�ͬʱ����ʱ��2�������ٴ������������ʹ�ã�2������δ�������ػ�
		1.3 ���⿪��ʱ�������ʱ�׶Σ��ʹ��ʱ�䲻�ó���28����
	2. ����״̬�����¿��غ�3s���ϣ���׼���ػ�


������������ѹ����3.6V���������,  ��ѹ����3.0V��ػ�
��������/�����ɣ��̵Ƴ���
��磺�̵�����
�豸���ϣ���Ƴ���

*/





static system_run_status_t g_run_status = DEV_POWEROFF;  //ϵͳ���е�״̬

uint32_t start_count  = 0;


//�޸�ϵͳ���е�״̬
void set_system_run_status(system_run_status_t status)
{
	g_run_status = status;
}


//���ϵͳ���е�״̬
system_run_status_t get_system_run_status(void)
{
	return g_run_status ;
}





//����
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
	
	if(start_count > 3000)  //����3000�β�������
		return;
	
	
	if(is_power_charge() && !charging_enable_start_laser) //��粻������
	{
		printf("power_charging, system disable to start!!!!!!\r\n");
		return;
	}
	
	vol = ADCgetBatVol();   //��õ�ѹֵ
	MY_PRINTF("%s %d vol = %d\r\n",__FUNCTION__,__LINE__,vol);
	if(vol <= 30)  //��ѹ̫���ˣ�������
	{
		DBG_PRINTF("ERROR:power is too low vol = %d\n",vol);
		return ;
	}
	
	// 2. ����ϵͳ״̬Ϊ��������״̬
	set_system_run_status(DEV_RUN_NORMAL);
	
	// 3. �����⿪�ؿ�ʼ����
//	ir_detect_init();   //�������ʼ��
	laser_enable(0);   //����ȫ���رգ�
	
	// 4. ����3.3v��Դ����
	output_BT3V_enable();
	
	//5. 5v��Դ������������Ҫ5v��Դ
	output_5v_enable();
	
	//5. ���ⶨʱ������
	IR_Recv_Timer_Control(1);
	
	//6.����pwm����
	Laser_Pwm_Timer_Control(1);
}


//�ػ�
void system_power_off(void)
{
	PowerManager_Mcu_Poweroff();   //��Ƭ���ϵ�
		
	//û������usb��Դ�Ļ�������Ͳ��ᴦ���ˡ�
	MY_PRINTF("%s %d\r\n",__FUNCTION__,__LINE__);
	set_system_run_status(DEV_POWEROFF);  //ϵͳ״̬�޸�Ϊ�ػ�
	
	pwm_all_change(0);
	laser_enable(0);   //����ȫ���رգ�5v�ĵ�Դ���ر�  ����output_5v_disable(void)
//	ir_detect_off();   //������ر�
	
	//4. ����3.3v��Դ�ر�
	output_BT3V_disable();
	
	//5. ���ⶨʱ���ر�
	IR_Recv_Timer_Control(0);
	
	//6.����pwm�ر�
	Laser_Pwm_Timer_Control(0);
	
	//7.�ػ�����������ʱ��
	clear_laser_light_times();
		
}










/*
	�豸����״̬�Ļ�ȡ��ʼ��
	1. ���״̬ PC6����У��͵�ƽ��Ч�� PB15 ��ѹģʽ���ƣ��ߵ�ƽ��Ч���͵�ƽ��ʾ����ģʽ
	2. ͷ�������״̬
	3. ��ص�ѹ��״̬
*/
void dev_status_get_init(void)
{
	
		
	//3. �������ܵĳ�ʼ��	
}




//�豸����ʱ��״̬�л� 200ms����һ�ΰ�
void dev_run_status_monitor_task(void)
{
	
}

