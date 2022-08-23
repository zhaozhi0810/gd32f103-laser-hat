/*remote.h*/
#ifndef __REMOTE_H__
#define __REMOTE_H__ 


#include <gd32f10x.h>  

// #define RDATA 	PCin(8)	 	//红外数据输入脚

//红外遥控识别码(ID),每款遥控器的该值基本都不一样,但也有一样的.
//我们选用的遥控器识别码为0
#define REMOTE_ID 0      		   
#define MAX_PULSE_LEN 400 //500  300


#define TOLERANCE 20

typedef enum {
	IR_NEC_NONE = 0,
	IR_NEC_NDEF,
	IR_NEC_FIRST_BURST,
	IR_NEC_SECOND_BURST,
	IR_NEC_SECOND_BURST_REPEAT,
	IR_NEC_1
} ir_nec_state;

//void ir_nec_init(u16 pin, GPIO_TypeDef *gpio);
//void ir_nec_state_machine(unsigned int time);
//void ir_nec_reset_transmission();




// extern u8  RmtCnt;			//按键按下的次数
// extern u16 PulseTab[MAX_PULSE_LEN];
// extern u8  Flag_LearnState ;
// extern u8  PulseTabCnt;//上升沿下降沿计数器的值

// void Remote_Init(void);    	//红外传感器接收头引脚初始化
// void TIM3_PWM_Init(u16 arr,u16 psc);
// void Infrared_Send(u16 *irdata,u32 irlen);

//TIMER7_CH2 发射  PC8  红外发射管
void TIM7_IR_SEND_Init(uint16_t arr,uint16_t psc);

/**
  *@name            void Infrared_Send_IR1(u16 *irdata,u32 irlen)
  *@description     红外信号发射函数
  *@param           irdata --   u16,红外数据
                    irlen  --   u32,红外数据长度							
  *@return		
  *@notice			
  */
void Infrared_Send(uint16_t *irdata,uint32_t irlen);
#endif
