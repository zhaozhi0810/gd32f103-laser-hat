
#ifndef __WORK_LEDS_H__
#define __WORK_LEDS_H__

void Led_Show_Work_init(void);

//工作灯闪烁
/*
	10 ms进入一次
	按照控制时序要求来！！！
	用全局变量控制灯的颜色，和快闪慢闪
*/
void Task_Led_Show_Work(void);
#endif




