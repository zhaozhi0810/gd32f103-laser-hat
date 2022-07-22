

#ifndef __DEV_RUN_STATUS_H__
#define __DEV_RUN_STATUS_H__

typedef enum {
	DEV_POWEROFF = 1,      //�ػ�
	DEV_VOL_LE36  =  2,   //��ѹ����3.6
	DEV_VOL_LE30  = 3,    //��ѹ����3.0
	DEV_RUN_NORMAL = 4,  //��������
	DEV_CHARGE  =  5,   //���
	DEV_CHARGE_OK  =  6,   //������
	DEV_BUG  =  7      //����
}system_run_status_t;



//�޸�ϵͳ���е�״̬
void set_system_run_status(system_run_status_t status);

//���ϵͳ���е�״̬
system_run_status_t get_system_run_status(void);

//����
void system_power_on(void);
//�ػ�
void system_power_off(void);

/*
	�豸����״̬�Ļ�ȡ��ʼ��
	1. ���״̬ PC6����У��͵�ƽ��Ч�� PB15 ��ѹģʽ���ƣ��ߵ�ƽ��Ч���͵�ƽ��ʾ����ģʽ
	2. ͷ�������״̬
	3. ��ص�ѹ��״̬
*/
void dev_status_get_init(void);

//���ϵͳ���е�״̬
//����ֵ����0��ʾ����У�0��ʾδ���
uint8_t get_bat_charge_status(void);

//﮵�ص�5v��ѹ���,������Ҫ5v
void output_5v_disable(void);

//﮵�ص�5v��ѹ���
void output_5v_enable(void);


#endif
 

