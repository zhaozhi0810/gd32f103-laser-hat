
#ifndef __TASK_CFG_H__
#define __TASK_CFG_H__


#include <gd32f10x.h>

//������main�������á�

//ʹ�ú궨������ļ��ʱ�䣬����Ķ�ʱ��systick.c�д���

//������õĸ�ʽ void fun(void)

#define TASK_MAX 16   //Ŀǰ����������

#define TASK1_TICKS_INTERVAL 10   //����1���ϵ簴ťɨ�� 10ms��
#define TASK2_TICKS_INTERVAL 1   //����2 �����pwm���ã�10msһ��
//#define TASK3_TICKS_INTERVAL 10   //����3����
#define TASK4_TICKS_INTERVAL 1000   //����4����ʪ�ȶ�ȡ����1000ms����һ��
#define TASK5_TICKS_INTERVAL 50   //����5��ϵͳ״̬�ƿ��ƣ�50msһ��
#define TASK6_TICKS_INTERVAL 500   //����6�����⿪�ؼ�⣬500ms����һ��,�������ⷢ��
#define TASK7_TICKS_INTERVAL 500   //����7����ص�ѹ���,����в�����ѹ 500msһ��
//#define TASK15_TICKS_INTERVAL 533   //����5��ι�����ݶ�533ms
//#define TASK16_TICKS_INTERVAL 1000   //����16��1sɨ�裬����led��˸����.2021-12-01 ����ɾ��

extern uint16_t g_task_id;   //ÿһ��λ��Ӧһ������Ϊ1��ʾ��Ҫ���������������������λ
//2021-09-30����task_allow,���ƶ�ʱ����������������
//extern uint16_t task_allow; //ÿһ��λ��Ӧһ������Ϊ1��ʾ����ʱɨ������񣬹ػ��󣬿��Խ�����Ҫ����������Ϊ������

typedef void(* task_t)(void);
#endif
