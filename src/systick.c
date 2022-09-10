/*!
    \file  systick.c
    \brief the systick configuration file
 
    \version 2015-11-16, V1.0.0, demo for GD32F10x
    \version 2017-06-30, V2.0.0, demo for GD32F10x
    \version 2018-07-31, V2.1.0, demo for GD32F10x
*/

/*
    Copyright (c) 2018, GigaDevice Semiconductor Inc.

    All rights reserved.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include "includes.h"

static uint32_t delay;
static uint32_t g_localtime;

/*!
    \brief      configure systick
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SystickConfig(void)
{
    /* setup systick timer for 1000Hz interrupts */
    if(SysTick_Config(SystemCoreClock / 1000U)){    //改为1ms中断一次
        /* capture error */
        while (1){
        }
    }
    /* configure the systick handler priority */
    NVIC_SetPriority(SysTick_IRQn, 0x00U);
}

/*!
    \brief      delay a time in milliseconds
    \param[in]  count: count in milliseconds
    \param[out] none
    \retval     none
*/
void Delay1ms(uint32_t count)
{
    delay = count;

    while(0U != delay){
    }
}

/*!
    \brief      delay decrement
    \param[in]  none
    \param[out] none
    \retval     none
*/
void DelayDecrement(void)
{
    if(delay){
        delay--;
    }
}



	//systick多少频率		108MHz 没有8分频			//us延时倍乘数
//59652323ns，59652ms，59s
void Delay1us(uint32_t nus)
{
	uint32_t ticks;
	uint32_t told,tnow,tcnt=0;
	uint32_t reload=SysTick->LOAD;				//LOAD的值
	uint8_t  fac_us= SystemCoreClock / 1000000U;
	ticks=nus*fac_us; 						//需要的节拍数
//	delay_osschedlock();					//阻止OS调度，防止打断us延时
	told=SysTick->VAL;        				//刚进入时的计数器值
	while(1)
	{
		tnow=SysTick->VAL;
		if(tnow!=told)
		{
			if(tnow<told)tcnt+=told-tnow;	//这里注意一下SYSTICK是一个递减的计数器就可以了.
			else tcnt+=reload-tnow+told;
			told=tnow;
			if(tcnt>=ticks)break;			//时间超过/等于要延迟的时间,则退出.
		}
	}
}


/*!
    \brief      updates the system local time
    \param[in]  none
    \param[out] none
    \retval     none
*/
void Systick_Int_Update(void)
{
//    static uint16_t times = 0;
	g_localtime ++;
	
	DelayDecrement();
	
	//通电就要检测按键的情况
	if(g_localtime % TASK1_TICKS_INTERVAL == 0)
	{
		g_task_id |= 1;  //任务1，上电开关扫描
	}
	
	
	//外接电源时需要亮灯
	if(g_localtime % TASK5_TICKS_INTERVAL == 15)
	{
		g_task_id |= 0x10;   //任务5，系统状态灯控制，500ms一次
	}
	
	if(get_system_run_status() > DEV_POWEROFF)  //开机之后才需要做的事情
	{		
	//	if(g_localtime % TASK2_TICKS_INTERVAL == 0)
		{
			g_task_id |= 2;   //任务2 激光的pwm设置，10ms一次
		}

		if(g_localtime % TASK4_TICKS_INTERVAL == 666)
		{
			g_task_id |= 8;   //任务4，温湿度读取任务，1000ms调用一次
		}
		
		if(g_localtime % TASK6_TICKS_INTERVAL == 177)
		{
			g_task_id |= 0x20;   //任务6，红外开关检测，500ms进入一次,包含红外发射
		}
		if(g_localtime % TASK7_TICKS_INTERVAL == 133)
		{
			g_task_id |= 0x40;   //任务7，电池电压监控,充电中不检测电压 500ms一次
		}
	}	 
}






