
#ifndef __PWM_H__
#define __PWM_H__

#include <gd32f10x.h>

//������ͨ��io�˿ڣ�ʹ�ö�ʱ��ȥģ��pwm
void laser_control_init(void);


// ����ʹ�ܣ�area8λ����7λÿһλ��ʾһ������1��ʾʹ�ܿ�����0��ʾ�ر�
void laser_enable(unsigned char area);


/*
����lcd����ռ�ձ�
//degree �޸�Ϊ0-100
ch ��ʾ�ĸ�����0-6 ��7������
*/
void pwm_out(uint8_t ch,uint8_t degree);



/*
ȫ��ͨ������Ϊĳһ��ֵ
//degree Ϊ��Ҫ������ֵ��0-100
*/
void pwm_all_change(uint8_t degree);


//100HZ��Ƶ�ʣ�10ms����һ��
void laser_run_pwm_task(void);

#endif
