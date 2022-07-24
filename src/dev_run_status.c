
#include "includes.h"

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
	MY_PRINTF("%s %d\r\n",__FUNCTION__,__LINE__);
	
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
	ir_detect_init();   //�������ʼ��
	
	
	// 4. ����3.3v��Դ����
	output_BT3V_enable();
}


//�ػ�
void system_power_off(void)
{
	PowerManager_Mcu_Poweroff();   //��Ƭ���ϵ�
		
	//û������usb��Դ�Ļ�������Ͳ��ᴦ���ˡ�
	MY_PRINTF("%s %d\r\n",__FUNCTION__,__LINE__);
	set_system_run_status(DEV_POWEROFF);  //ϵͳ״̬�޸�Ϊ�ػ�
	laser_enable(0);   //����ȫ���رգ�5v�ĵ�Դ���ر�  ����output_5v_disable(void)
	ir_detect_off();   //������ر�
	
	//4. ����3.3v��Դ�ر�
	output_BT3V_disable();
		
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

