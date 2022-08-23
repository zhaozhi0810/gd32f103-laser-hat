
#include "includes.h"



//TIMER7_CH2 发射  PC8  红外发射管
void TIM7_IR_SEND_Init(uint16_t arr,uint16_t psc)
{
	timer_parameter_struct initpara;
	timer_oc_parameter_struct ocpara;

	//发射部分
	rcu_periph_clock_enable(RCU_TIMER7);  //定时器模块时钟使能
	rcu_periph_clock_enable(RCU_GPIOC);

	gpio_init(GPIOC, GPIO_MODE_AF_PP, GPIO_OSPEED_2MHZ, GPIO_PIN_8);   //复用功能
	
	//3. 初始化定时器的数据结构  /* initialize TIMER init parameter struct */
	timer_struct_para_init(&initpara);
	initpara.period = arr-1;  //重载的数字，频率20kHZ
	initpara.prescaler = psc-1;  //预分频数，得到是1Mhz的脉
	//4. 初始化定时器      /* initialize TIMER counter */
	timer_init(TIMER7, &initpara);
	
	//5. 初始化定时器通道的数据结构 /* initialize TIMER channel output parameter struct */
	timer_channel_output_struct_para_init(&ocpara);	
#ifndef PWM_TIMER_USE_CHN
	ocpara.outputstate  = TIMER_CCX_ENABLE;  //输出通道使能	
#else
    ocpara.outputnstate = TIMER_CCXN_ENABLE;//使能互补通道输出
#endif
	//6. 初始化定时器通道   /* configure TIMER channel output function */
	timer_channel_output_config(TIMER7, TIMER_CH_2, &ocpara);
			
	//7. 初始化定时器通道输出方式设置   /* configure TIMER channel output compare mode */
	timer_channel_output_mode_config(TIMER7, TIMER_CH_2, TIMER_OC_MODE_PWM1);
	/* configure TIMER channel output pulse value */
	
	//8. 初始化定时器通道输出脉冲宽带
	timer_channel_output_pulse_value_config(TIMER7, TIMER_CH_2, (arr)/3);

	//9. 初始化定时器通道输出使能
	//timer_channel_output_fast_config(TIMER2, TIMER_CH_0, TIMER_OC_FAST_ENABLE);
	timer_channel_output_shadow_config(TIMER7, TIMER_CH_2, TIMER_OC_SHADOW_DISABLE);	  //stm32似乎用的是这个0x8
	//10.初始化，定时器不使能 2022-04-18	
	
	
	/* enable a TIMER */
//	if(TIMER7 == TIMER0 || TIMER7 == TIMER7)
		timer_primary_output_config(TIMER7, ENABLE);
	timer_auto_reload_shadow_enable(TIMER7);

}




/**
  *@name            void Infrared_Send_IR1(u16 *irdata,u32 irlen)
  *@description     红外信号发射函数
  *@param           irdata --   u16,红外数据
                    irlen  --   u32,红外数据长度							
  *@return		
  *@notice			
  */
void Infrared_Send(uint16_t *irdata,uint32_t irlen)
{
	uint32_t i;                                      //用于下面的for循环

	for(i=0; i<irlen && irdata[i]!=0xffff; i++) //循环，从i=0开始，当i<irlen 并且 irdata[i] != 0xffff 时成立，当其中一个不成立，退出循环
	{
		if(i%2 == 0)                            //偶数的下标的数组成员延时拉高电平
		{
			timer_channel_output_state_config(TIMER7, TIMER_CH_2, TIMER_CCX_ENABLE);//TIM_Cmd(TIM3,ENABLE);
			
			Delay1us(irdata[i]);
			
			timer_channel_output_state_config(TIMER7, TIMER_CH_2, TIMER_CCX_DISABLE);//TIM_Cmd(TIM3,DISABLE);
			
			gpio_bit_set(GPIOC, GPIO_PIN_8);//GPIO_SetBits(GPIOA,GPIO_Pin_7);			
		}
		else
		{
			gpio_bit_set(GPIOC, GPIO_PIN_8);//GPIO_SetBits(GPIOA,GPIO_Pin_7);
			
			Delay1us(irdata[i]);
		}
	}
	
	Delay1us(555);
	
	gpio_bit_reset(GPIOC, GPIO_PIN_8);//GPIO_ResetBits(GPIOA,GPIO_Pin_7);
	
	return ;
}


