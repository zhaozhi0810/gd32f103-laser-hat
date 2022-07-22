
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





static system_run_status_t g_run_status;  //ϵͳ���е�״̬

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
	set_system_run_status(DEV_RUN_NORMAL);
}


//�ػ�
void system_power_off(void)
{
	set_system_run_status(DEV_POWEROFF);  //ϵͳ״̬�޸�Ϊ�ػ�
	laser_enable(0);   //����ȫ���ر�
}



//﮵�ص�5v��ѹ���
void output_5v_enable(void)
{
	gpio_bit_set(GPIOB, GPIO_PIN_15);  //5v���
}

//﮵�ص�5v��ѹ���
void output_5v_disable(void)
{
	gpio_bit_reset(GPIOB, GPIO_PIN_15);  //5v�����
}



//���ϵͳ���е�״̬
//����ֵ����0��ʾ����У�0��ʾδ���
uint8_t get_bat_charge_status(void)
{
	return !gpio_input_bit_get(GPIOC, GPIO_PIN_6);  //���ʱΪ�͵�ƽ������ȡ��һ��
}


/*
	�豸����״̬�Ļ�ȡ��ʼ��
	1. ���״̬ PC6����У��͵�ƽ��Ч�� PB15 ��ѹģʽ���ƣ��ߵ�ƽ��Ч���͵�ƽ��ʾ����ģʽ
	2. ͷ�������״̬
	3. ��ص�ѹ��״̬
*/
void dev_status_get_init(void)
{
	//0. ���ڵ�ѹ�����Ŀ�������ʼ��
	ADC_Init();
	
	//2. ���״̬�����ų�ʼ��
	//2.1 ���״̬��ȡ
	rcu_periph_clock_enable(RCU_GPIOC);			
	gpio_init(GPIOC, GPIO_MODE_IPU, GPIO_OSPEED_2MHZ, GPIO_PIN_6);	  //PC6����Ϊ����ģʽ
	
	//2.2 ��ѹ5v���ʹ�ܣ�ʱ��ʹ��
	rcu_periph_clock_enable(RCU_GPIOB);			
	gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ, GPIO_PIN_15);	
	gpio_bit_reset(GPIOB, GPIO_PIN_15);  //5v�����
	
	//3. �������ܵĳ�ʼ��
	
	
	
}




