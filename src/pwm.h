
#ifndef __PWM_H__
#define __PWM_H__

#include <gd32f10x.h>

extern uint8_t g_pwm[7];   //每一个通道设置不同的pwm值，pwm范围0-100。一般情况是7个区域同时变化。


//用于普通的io端口，使用定时器去模拟pwm
void laser_control_init(void);


// 激光使能，area8位，低7位每一位表示一个区域，1表示使能开启，0表示关闭
void laser_enable(unsigned char area);


/*
设置lcd亮度占空比
//degree 修改为0-100
ch 表示哪个区域，0-6 共7个区域
*/
void pwm_out(uint8_t ch,uint8_t degree);



/*
全部通道设置为某一个值
//degree 为需要调整的值，0-100
*/
void pwm_all_change(uint8_t degree);


//100HZ的频率，10ms进入一次
void laser_run_pwm_task(void);


//获得激光区域设置
uint8_t get_laser_area_val(void);
//设置激光区域  2022-09-16
void set_laser_area_val(uint8_t val);

//增加一个区域的激光 area取值0-6
void laser_add_a_area(uint8_t area);

//减少一个区域的激光 area取值0-6
void laser_sub_a_area(uint8_t area);


//定时器开启或者关闭 1为开启，0为关闭   2022-09-10增加
void Laser_Pwm_Timer_Control(uint8_t enable);

//只是用于laserpwm计时，就使用tim1了，避开TIMER2
void TIM1_Laser_Pwm_Init(uint16_t arr,uint16_t psc);
#endif

