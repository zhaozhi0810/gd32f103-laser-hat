
#ifndef __PWM_H__
#define __PWM_H__

#include <gd32f10x.h>

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

#endif

