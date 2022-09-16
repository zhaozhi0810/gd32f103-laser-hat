
#ifndef __PWM_H__
#define __PWM_H__

#include <gd32f10x.h>

extern uint8_t g_pwm[7];   //ÿһ��ͨ�����ò�ͬ��pwmֵ��pwm��Χ0-100��һ�������7������ͬʱ�仯��


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


//��ü�����������
uint8_t get_laser_area_val(void);
//���ü�������  2022-09-16
void set_laser_area_val(uint8_t val);

//����һ������ļ��� areaȡֵ0-6
void laser_add_a_area(uint8_t area);

//����һ������ļ��� areaȡֵ0-6
void laser_sub_a_area(uint8_t area);


//��ʱ���������߹ر� 1Ϊ������0Ϊ�ر�   2022-09-10����
void Laser_Pwm_Timer_Control(uint8_t enable);

//ֻ������laserpwm��ʱ����ʹ��tim1�ˣ��ܿ�TIMER2
void TIM1_Laser_Pwm_Init(uint16_t arr,uint16_t psc);
#endif

