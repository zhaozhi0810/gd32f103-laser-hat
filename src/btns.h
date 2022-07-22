/*
    Copyright (C) 2016 GigaDevice

    2016-10-19, V1.0.0, firmware for GD32F450I
*/

#ifndef __BTNS_LEDS_H__
#define __BTNS_LEDS_H__

//#ifdef cplusplus
// extern "C" {
//#endif

#include <gd32f10x.h>
#include <stdint.h>

/* 按键检测：10ms中断检测3次 */



#define BTN_CODE_START 0xE8     //根据技术文档修改的   2021-09-29



typedef struct btn_info{
	uint8_t  value;	     //值，0表示松开，1表示按下
	uint8_t  reportEn;   //1，消抖检测到了，0没有检测到按键
	uint16_t  pressCnt;     //长按区分
}BTN_INFO;




void task1_btn_scan(void);

//按键处理函数
void btn_handle(void);
//按键扫描程序
void btns_scan(void); // 10ms 调用一次

#endif 
