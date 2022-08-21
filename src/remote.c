/*
* @Author: dazhi
* @Date:   2022-08-21 15:59:09
* @Last Modified by:   dazhi
* @Last Modified time: 2022-08-21 15:59:33
*/
/*remote.c*/
#include "includes.h"
//#include "delay.h"
//#include "usart.h"

/**
参考程序
发射：PA7-TIM3-CH2
接收：PC8-TIM8-CH3

TIM8：
下面代码中捕获中断和定时中断分开，不在同一中断函数，所以需要配置两次中断优先级（应该可以合在一起配置）
*/


/*
	PC9  ir_out 红外接收段   timer2ch2 全映射，  TIMER7_CH3
	PC8  红外发射管  		timer2ch3 全映射，  TIMER7_CH2


	根据红外的原理，我只要发出38kHZ的信号，接收头就会默认为接收到高电平，经三极管后，输出低电平即可

	接收引脚为低电平，即可认为收到了红外发射的信号，没收到则为高电平

	PC8 设置为pwm 38KHZ(占空比50%)，持续发送  （为了省电可以100ms发送一次）使用定时器7
	PC9 输入，低有效（表示接收到了信号，表示未佩戴），高则无效（表示没有收到信号，表示已经佩戴）
*/


#define PWM_PIN GPIO_PIN_8
#define PWM_PORT GPIOC
#define PWM_PORT_RCU RCU_GPIOC
#define PWM_TIMER_RCU  RCU_TIMER7    //
#define PWM_TIMER  TIMER7
#define PWM_TIMER_CH TIMER_CH_2

uint16_t PWM_DEGREE_MAX = (uint16_t)(1000/38);   //PWM频率  26为38KHZ   1us计个数，4000计算的频率就是1000000/4000=250Hz 





/**
 *  @name			void Remote_Init(void) 
 *	@description	红外遥控初始化 设置IO以及定时器8的输入捕获
 *                  Infrared remote initializer sets IO and timer 8 input capture
 *  @notice			
 */
void Remote_Init(void)    			  
{  
#if 0
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef  TIM_ICInitStructure;  

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO,ENABLE); //使能PORTB时钟 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);                      //TIM8 时钟使能 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;                                //PC8 输入 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;                    //浮空输入 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_SetBits(GPIOC,GPIO_Pin_8);                                          //初始化GPIOC.8

	TIM_TimeBaseStructure.TIM_Period = 9999;                                 //设定计数器自动重装值 最大10ms溢出  
	TIM_TimeBaseStructure.TIM_Prescaler =(72-1);                             //预分频器,1M的计数频率,1us加1.	   	
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;                  //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;              //TIM向上计数模式
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);                          //根据指定的参数初始化TIMx

	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;                         // 选择输入端 IC4映射到TI4上
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;              //上升沿捕获
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;                    //配置输入分频,不分频 
	TIM_ICInitStructure.TIM_ICFilter = 0x03;                                 //IC4F=0011 配置输入滤波器 8个定时器时钟周期滤波
	TIM_ICInit(TIM8, &TIM_ICInitStructure);                                  //初始化定时器输入捕获通道
	TIM_Cmd(TIM8,ENABLE );                                                   //使能定时器4

	NVIC_InitStructure.NVIC_IRQChannel = TIM8_CC_IRQn;                       //TIM8TIM8捕获比较中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;                //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;                       //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                          //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);                                          //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器	
	TIM_ITConfig( TIM8,TIM_IT_CC3,ENABLE);                                   //允许更新中断 ,允许CC4IE捕获中断		
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM8_UP_IRQn;                       //TIM3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;                //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;                       //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                          //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);                                          //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器	
	TIM_ITConfig( TIM8,TIM_IT_Update,ENABLE);                                //允许更新中断 ,允许CC4IE捕获中断								 
#else
	uint16_t degree = 50;    //占空比50%
	//PB15 通道
	timer_parameter_struct initpara;
	timer_oc_parameter_struct ocpara;
	//1. io引脚设置复用功能	
	gpio_init(PWM_PORT, GPIO_MODE_AF_PP, GPIO_OSPEED_2MHZ, PWM_PIN);   //复用功能	
	//PB15 默认映射tim0 ch2的N通道（从0开始数）
	//gpio_pin_remap_config(GPIO_TIMER2_PARTIAL_REMAP, ENABLE);    //部分映射
	
	//2. 定时器时钟使能
	rcu_periph_clock_enable(PWM_TIMER_RCU);  //定时器模块时钟使能
		
	if(degree > 100)
	{
		degree = 100;
	}
	
	//3. 初始化定时器的数据结构  /* initialize TIMER init parameter struct */
	timer_struct_para_init(&initpara);
	initpara.period = PWM_DEGREE_MAX-1;  //重载的数字，频率20kHZ
	initpara.prescaler = (SystemCoreClock/1000000)-1;  //预分频数，得到是1Mhz的脉冲  
		
	//4. 初始化定时器      /* initialize TIMER counter */
	timer_init(PWM_TIMER, &initpara);
		
	//5. 初始化定时器通道的数据结构 /* initialize TIMER channel output parameter struct */
	timer_channel_output_struct_para_init(&ocpara);	
#ifndef PWM_TIMER_USE_CHN
	ocpara.outputstate  = TIMER_CCX_ENABLE;  //输出通道使能	
#else
    ocpara.outputnstate = TIMER_CCXN_ENABLE;//使能互补通道输出
#endif
	//6. 初始化定时器通道   /* configure TIMER channel output function */
	timer_channel_output_config(PWM_TIMER, PWM_TIMER_CH, &ocpara);
			
	//7. 初始化定时器通道输出方式设置   /* configure TIMER channel output compare mode */
	timer_channel_output_mode_config(PWM_TIMER, PWM_TIMER_CH, TIMER_OC_MODE_PWM1);
	/* configure TIMER channel output pulse value */
	
	//8. 初始化定时器通道输出脉冲宽带
	timer_channel_output_pulse_value_config(PWM_TIMER, PWM_TIMER_CH, (100-degree) * PWM_DEGREE_MAX/100);

	//9. 初始化定时器通道输出使能
	//timer_channel_output_fast_config(TIMER2, TIMER_CH_0, TIMER_OC_FAST_ENABLE);
	timer_channel_output_shadow_config(PWM_TIMER, PWM_TIMER_CH, TIMER_OC_SHADOW_DISABLE);	  //stm32似乎用的是这个0x8
	//10.初始化，定时器不使能 2022-04-18	
	
	
	/* enable a TIMER */
	if(PWM_TIMER == TIMER0 || PWM_TIMER == TIMER7)
		timer_primary_output_config(PWM_TIMER, ENABLE);
	timer_auto_reload_shadow_enable(PWM_TIMER);
#endif
}
/**
 *  @name			void TIM3_PWM_Init(u16 arr,u16 psc)
 *	@description	初始化定时器3的设置，将定时器3用于PWM调制，PWM输出口为 PA.7
 *	@param			arr --	u16,定时器重装值
					psc --	u16,定时器分频值							
 *	@return		
 *  @notice			PWM频率 = 72M/((arr+1)*(psc+1)),这里用作红外发射的载波，需要生成38kHz的方波，故取arr = 1895,psc = 0。
 */
void TIM3_PWM_Init(u16 arr,u16 psc)
{
	/* 初始化结构体定义 */
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef	TIM_TimeBaseInitStructure;                        //定时器基本设置
	TIM_OCInitTypeDef 	TIM_OCInitStructure;                                  //定时器比较输出配置

	/* 使能相应端口的时钟 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);                      //使能定时器2时钟
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE); //使能GPIO外设时钟
	
	/* GPIOA.7初始化 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;                                 // TIM3 CH2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;                           // PA.7 复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA,GPIO_Pin_7);

	/* TIM3 初始化*/
	TIM_TimeBaseInitStructure.TIM_Period = arr;                               //下一个更新事件装入活动的自动重装载寄存器周期的值
	TIM_TimeBaseInitStructure.TIM_Prescaler = psc;                            //作为TIMx时钟频率除数的预分频值 
	TIM_TimeBaseInitStructure.TIM_ClockDivision = 0;                          //时钟分割:TDTS = Tck_tim
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;           //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);

	/* 定时器TIM3 Ch2 PWM模式初始化 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;                         //选择定时器模式:TIM PWM1   TIM_OCMode_PWM1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;             //比较输出使能
	TIM_OCInitStructure.TIM_Pulse = (arr+1)/3;                                //占空比1:3 (arr+1)/10  3
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;                 //输出极性:TIM输出比较极性高   TIM_OCPolarity_High
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);

	/* 使能TIM3在CCR1上的预装载寄存器 */
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable); 
}


u8 	RmtSta=0;                    //红外学习上升或下降标志位
u16 Dval;                        //上升沿、下降沿的计数器的值
u8  RmtCnt=0;                    //定时器红外学习计数超时标志位	  
u8  PulseTabCnt=0;               //上升沿下降沿计数器的值
u16 PulseTab[MAX_PULSE_LEN]={0}; //红外学习存储数据
u8  Flag_LearnState = 0;         //红外学习标志位

/**
  *@name  void TIM8_UP_IRQHandler(void)
  *@brief TIM8的定时计数中断函数
  *       The timing count interrupt function of TIM8
  *@note
  */
void TIM8_UP_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM8,TIM_IT_Update)!=RESET)
	{
		if(RmtCnt++>50)
		{
			RmtCnt = 0;

			if(RmtSta)
			{
				RmtSta = 0;
				Flag_LearnState = 1;
			}
		}
	}
	TIM_ClearITPendingBit(TIM8,TIM_IT_Update);	 	    
}

/**
  *@name  void TIM8_CC_IRQHandler(void)
  *@brief 捕获红外载波的高低电平宽度，记录到数组PulseTab
  *       Capture the high and low level width of the infrared carrier and record it to the array PulseTab
  *@note
  */
void TIM8_CC_IRQHandler(void)
{ 		    	 
	if(TIM_GetITStatus(TIM8,TIM_IT_CC3)!=RESET)                 //获取上升沿或下降沿状态
	{	  
		if(RDATA)                                               //上升沿捕获
		{
			Dval=TIM_GetCapture3(TIM8);                         //读取CCR4也可以清CC4IF标志位
			
			TIM_OC3PolarityConfig(TIM8,TIM_ICPolarity_Falling); //CC4P=1	设置为下降沿捕获
			
			TIM_SetCounter(TIM8,0);                             //清空定时器值
			
			if(RmtSta&0X01)
			{
				PulseTab[PulseTabCnt++] = Dval;
				
				RmtSta = 0x10;
			}
			else
			{
				RmtSta = 0X10;                                  //标记上升沿已经被捕获
			}
		}
		else                                                    //下降沿捕获
		{
			Dval=TIM_GetCapture3(TIM8);                         //读取CCR4也可以清CC4IF标志位

			TIM_OC3PolarityConfig(TIM8,TIM_ICPolarity_Rising);  //CC4P=0	设置为上升沿捕获
			
			TIM_SetCounter(TIM8,0);                             //清空定时器值
			
			if(RmtSta&0X10)                                     //完成一次高电平捕获 
			{
				PulseTab[PulseTabCnt++] = Dval;
				
				RmtSta = 0x01;
			}
			else 
			{
				RmtSta = 0x01;
			}
		}				 		     	    					   
	}
	TIM_ClearITPendingBit(TIM8,TIM_IT_CC3);	 	    
}

/**
  *@name            void Infrared_Send_IR1(u16 *irdata,u32 irlen)
  *@description     红外信号发射函数
  *@param           irdata --   u16,红外数据
                    irlen  --   u32,红外数据长度							
  *@return		
  *@notice			
  */
void Infrared_Send(u16 *irdata,u32 irlen)
{
	u32 i;                                      //用于下面的for循环

	for(i=0; i<irlen && irdata[i]!=0xffff; i++) //循环，从i=0开始，当i<irlen 并且 irdata[i] != 0xffff 时成立，当其中一个不成立，退出循环
	{
		if(i%2 == 0)                            //偶数的下标的数组成员延时拉高电平
		{
			TIM_Cmd(TIM3,ENABLE);
			
			delay_us(irdata[i]);
			
			TIM_Cmd(TIM3,DISABLE);
			
			GPIO_SetBits(GPIOA,GPIO_Pin_7);			
		}
		else
		{
			GPIO_SetBits(GPIOA,GPIO_Pin_7);
			
			delay_us(irdata[i]);
		}
	}
	
	delay_us(555);
	
	GPIO_ResetBits(GPIOA,GPIO_Pin_7);
	
	return ;
}
