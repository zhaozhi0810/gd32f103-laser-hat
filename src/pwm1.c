

/*
	2022-07-19 
	pwm 初始化，看样子是不使用定时器自带的pwm
	
	PB4 3,5,PD2,PC12,11,10,  对应PWM1-7 
	PB4 timer2ch0，部分映射 10
	PB3 timer1ch1，部分映射 01
	PB5 timer2ch1，部分映射 10
	PD2  TIMER2_ETI
	PC12 无timer
	PC11  无timer
	PC10  无timer
	
	PA12  PWM_M
	
	2022-09-10 改进激光灯的pwm，使用一个定时器实现。
	定时器timer1，定时时间为1ms，50HZ的频率，周期为20ms
	
	分成20分，占空比为5，10，15，20 等共20种

*/

#include "includes.h"

	
//uint16_t PWM_DEGREE_MAX = 4000;   //PWM频率  1us计个数，4000计算的频率就是1000000/4000=250Hz 太高了影响电磁兼容实验？？？
uint8_t g_pwm[7] = {0};   //每一个通道设置不同的pwm值，pwm范围0-100。一般情况是7个区域同时变化。
static uint8_t g_pwm_status = 0;  //0-6位，1表示开启激光，0表示关闭激光

#define PWM_HZ 20   //设置pwm的频率，定时器每20ms进入一次，即为50HZ

static uint8_t laser_area_control = 0x7f;  //表示7个区域全开，可以设置对应的区域 


//用于普通的io端口，使用定时器去模拟pwm
void laser_control_init(void)
{
	//时钟使能
	rcu_periph_clock_enable(RCU_GPIOA);	
	rcu_periph_clock_enable(RCU_GPIOB);	
	rcu_periph_clock_enable(RCU_GPIOC);	
	rcu_periph_clock_enable(RCU_GPIOD);	
		
	gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ, GPIO_PIN_12);
	gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ, GPIO_PIN_3 | GPIO_PIN_4 |GPIO_PIN_5 );
	gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ, GPIO_PIN_12 | GPIO_PIN_10 |GPIO_PIN_11 );
	gpio_init(GPIOD, GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ, GPIO_PIN_2);
	
	//低电平,无效，高电平使能
	gpio_bit_reset(GPIOA, GPIO_PIN_12);  //PWM_M
	gpio_bit_reset(GPIOB, GPIO_PIN_3 | GPIO_PIN_4 |GPIO_PIN_5);
	gpio_bit_reset(GPIOC, GPIO_PIN_12 | GPIO_PIN_10 |GPIO_PIN_11);
	gpio_bit_reset(GPIOD, GPIO_PIN_2);
	
	MY_PRINTF("%s %d\r\n",__FUNCTION__,__LINE__);
}



//只是用于计时，就使用tim1了，避开TIMER2
void TIM1_Laser_Pwm_Init(uint16_t arr,uint16_t psc)
{
	timer_parameter_struct initpara;
	//接收部分
	rcu_periph_clock_enable(RCU_TIMER1);  //定时器模块时钟使能

	//3. 初始化定时器的数据结构  /* initialize TIMER init parameter struct */
	timer_struct_para_init(&initpara);
	initpara.period = arr;  //重载的数字，
	initpara.prescaler = psc;  //预分频数，得到是1Mhz的脉
	//4. 初始化定时器      /* initialize TIMER counter */
	timer_init(TIMER1, &initpara);

	nvic_irq_enable(TIMER1_IRQn, 1U, 1U);
	timer_interrupt_enable(TIMER1, TIMER_INT_UP);   //定时中断	
}




//定时器1的中断处理，目前是pwm
void TIMER1_IRQHandler(void)
{
	static uint8_t count = 0;
	uint8_t i;
	
	if(timer_interrupt_flag_get(TIMER1,TIMER_INT_FLAG_UP)!=RESET)
	{
		count++;  //1ms 已过去
		
		if(count < PWM_HZ)
		{
			g_pwm_status = laser_area_control;   //默认设置全部点亮，由laser_control控制哪些区域要点亮
			
			for(i=0;i<7;i++)
			{
				if(g_pwm[i] <= count) //计数值比设定值要大，关闭
				{
					g_pwm_status &= ~(1<<i);   //清零表示关闭
				}			
			}
			laser_enable(g_pwm_status);   //根据状态控制激光区域的显示
		}
		else
		{
			count = 0;   //一个周期结束，重新开始下一个周期
		//	return;   //刚刚清零就不用去加了
		}
	}
	timer_interrupt_flag_clear(TIMER1,TIMER_INT_FLAG_UP);	
}


//定时器开启或者关闭 1为开启，0为关闭
void Laser_Pwm_Timer_Control(uint8_t enable)
{
	if(enable)
	{
		//启动定时器1
		timer_enable(TIMER1);
	}
	else
		timer_disable(TIMER1);
}




void laser_disable(void);

// 激光使能，area8位，低7位每一位表示一个区域，1表示使能开启，0表示关闭
void laser_enable(unsigned char area)
{
	if(!area)
	{
		laser_disable();
//		MY_PRINTF("%s %d\r\n",__FUNCTION__,__LINE__);
		return;
	}
	
	if(get_system_run_status() == DEV_POWEROFF)
	{
		laser_disable();
//		MY_PRINTF("%s %d\r\n",__FUNCTION__,__LINE__);
		return;
	}
	
	//开启5v外设电源供电
	output_5v_enable();
	
	if(area & 1)
		gpio_bit_set(GPIOB, GPIO_PIN_4);
	else
		gpio_bit_reset(GPIOB, GPIO_PIN_4);
	
	if(area & 2)
		gpio_bit_set(GPIOB, GPIO_PIN_3);
	else
		gpio_bit_reset(GPIOB, GPIO_PIN_3);
	
	if(area & 4)
		gpio_bit_set(GPIOB, GPIO_PIN_5);
	else
		gpio_bit_reset(GPIOB, GPIO_PIN_5);
	
	if(area & 8)
		gpio_bit_set(GPIOD, GPIO_PIN_2);
	else
		gpio_bit_reset(GPIOD, GPIO_PIN_2);
	
	if(area & 0x10)
		gpio_bit_set(GPIOC, GPIO_PIN_12);
	else
		gpio_bit_reset(GPIOC, GPIO_PIN_12);
	
	if(area & 0x20)
		gpio_bit_set(GPIOC, GPIO_PIN_11);
	else
		gpio_bit_reset(GPIOC, GPIO_PIN_11);
	
	if(area & 0x40)
		gpio_bit_set(GPIOC, GPIO_PIN_10);
	else
		gpio_bit_reset(GPIOC, GPIO_PIN_10);
}

// 激光禁止，
static void laser_disable(void)
{	
	//关闭5v外设电源供电
//	output_5v_disable();  //调试时关闭09-08
	
//	gpio_bit_reset(GPIOA, GPIO_PIN_12);  //PWM_M
	gpio_bit_reset(GPIOB, GPIO_PIN_3 | GPIO_PIN_4 |GPIO_PIN_5);
	gpio_bit_reset(GPIOC, GPIO_PIN_12 | GPIO_PIN_10 |GPIO_PIN_11);
	gpio_bit_reset(GPIOD, GPIO_PIN_2);	
}




/*
设置lcd亮度占空比
//degree 修改为0-100
ch 表示哪个区域，0-6 共7个区域
*/
void pwm_out(uint8_t ch,uint8_t degree)
{
	if(ch > 6)
		return;
	
	if(degree>100)
		degree = 100;
	
	
	g_pwm[ch] = degree/5;
	
	MY_PRINTF("%s %d ch = %d degree = %d\r\n",__FUNCTION__,__LINE__,ch,degree);
}


/*
全部通道设置为某一个值
//degree 为需要调整的值，0-10
*/
void pwm_all_change(uint8_t degree)
{
	uint8_t i;
	if(degree>100)
		degree = 100;

	for(i=0;i<7;i++)
	{
		g_pwm[i] = degree/5;  //只有0-20 这几种，共20个等级   					
	}
	
//	MY_PRINTF("%s %d degree = %d\r\n",__FUNCTION__,__LINE__,degree);
}


//增加一个区域的激光 area取值0-6
void laser_add_a_area(uint8_t area)
{
	if(area < 7)
		laser_area_control |= 1<<area;
}

//减少一个区域的激光 area取值0-6
void laser_sub_a_area(uint8_t area)
{
	if(area < 7)
		laser_area_control &= ~(1<<area);
}



uint8_t get_laser_area_val(void)
{
	return laser_area_control;
}




//100HZ的频率，10ms进入一次
//void laser_run_pwm_task(void)
//{
//	static uint16_t count = 0;
//	uint8_t i;
//	
//	//关机状态下不控制激光了
//	if(get_system_run_status() <= DEV_POWEROFF)  //有几种状态是不开机的
//	{
//		count = 0;
//		laser_enable(0);   //确保激光全部关闭
//		return;
//	}
//	//调试时暂时关闭 2022-09-08
//	laser_enable(0x7f);   //根据状态控制激光区域的显示  //调试时暂时关闭 2022-09-08
//	return 0;   //调试时暂时关闭 2022-09-08
//	if(count < PWM_HZ)
//	{
//		g_pwm_status = laser_area_control;   //默认设置全部点亮，由laser_control控制哪些区域要点亮
//		
//		for(i=0;i<7;i++)
//		{
//			if(g_pwm[i] <= count) //计数值比设定值要大，关闭
//			{
//				g_pwm_status &= ~(1<<i);   //清零表示关闭
//			}			
//		}
//		laser_enable(g_pwm_status);   //根据状态控制激光区域的显示
//	}
//	else
//	{
//		count = 0;   //一个周期结束，重新开始下一个周期
//		return;   //刚刚清零就不用去加了
//	}
//	count++;
//}


