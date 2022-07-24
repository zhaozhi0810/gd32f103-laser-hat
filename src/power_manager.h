

#ifndef __POWER_MANAGER_H__
#define __POWER_MANAGER_H__

void PowerManager_init(void);
//��Ƭ���ϵ�
void PowerManager_Mcu_Poweroff(void);


//usb��Դ�Ƿ����ӣ�  1��ʾ���ӣ�0��ʾû������
uint8_t get_usb_PowerStatus(void);


//3.3v��Դ�Ƿ������  1��ʾ�����0��ʾû�����
uint8_t get_BT3V_PowerStatus(void);


//���ϵͳ���е�״̬
//����ֵ����0��ʾ����У�0��ʾδ���
uint8_t get_bat_charge_status(void);

//﮵�ص�5v��ѹ���,������Ҫ5v
void output_5v_disable(void);

//﮵�ص�5v��ѹ���
void output_5v_enable(void);



//�����Դ3.3v��ѹ���
void output_BT3V_enable(void);

//�����Դ3.3v����ر�
void output_BT3V_disable(void);


//�ⲿ��Դ���ӣ����ڳ�緵��1������2����ʾ�ѳ�����û�����0
uint8_t is_power_charge(void);



//��ƵоƬʡ�����
void set_MAX9700_enable(void);

//��ƵоƬʡ�����
void set_MAX9700_shutdown(void);
#endif



