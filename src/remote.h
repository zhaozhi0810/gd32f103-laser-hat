/*remote.h*/
#ifndef __RED_H
#define __RED_H 
//#include "sys.h"   

#define RDATA 	PCin(8)	 	//红外数据输入脚

//红外遥控识别码(ID),每款遥控器的该值基本都不一样,但也有一样的.
//我们选用的遥控器识别码为0
#define REMOTE_ID 0      		   
#define MAX_PULSE_LEN 400 //500  300

extern u8  RmtCnt;			//按键按下的次数
extern u16 PulseTab[MAX_PULSE_LEN];
extern u8  Flag_LearnState ;
extern u8  PulseTabCnt;//上升沿下降沿计数器的值

void Remote_Init(void);    	//红外传感器接收头引脚初始化
void TIM3_PWM_Init(u16 arr,u16 psc);
void Infrared_Send(u16 *irdata,u32 irlen);

#endif
