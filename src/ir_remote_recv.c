
#include "includes.h"

/* 红外的接收部分 */
//PC9  ir_out 红外接收引脚   TIMER7_CH3
//由于timer7 可能在某些单片机不可用，这个计时定时器改为timer4，2022-08-23

//uint16_t pin;
//GPIO_TypeDef *gpio;

#define IR_DETECT_USE_IRQ   //红外接收使用外部中断信号（必须使用）


static ir_nec_state ir_nec_current_state;
//uint8_t cnt = 0;

static unsigned int ir_recv_data, ir_recv_prev_data;
static uint16_t ir_nec_pulse_lengths[6];
static bool ir_nec_repeat_last_command = FALSE;

static uint16_t g_laser_light_timers = 0; //用于计时佩戴的时间



//数据结构初始化
static void ir_nec_init(void) {
//	pin = p;
//	gpio = g;
	ir_nec_current_state = IR_NEC_NONE;

	ir_nec_pulse_lengths[IR_NEC_NONE] = 0;
	ir_nec_pulse_lengths[IR_NEC_FIRST_BURST] = 9000;    //9ms
	ir_nec_pulse_lengths[IR_NEC_SECOND_BURST] = 4500;   //4.5ms
	ir_nec_pulse_lengths[IR_NEC_1] = 560;               //560us
	ir_nec_pulse_lengths[IR_NEC_NDEF] = 1;				//1us
	ir_nec_pulse_lengths[IR_NEC_SECOND_BURST_REPEAT] = 2250;  //2.25ms
}

static void ir_print_data(unsigned int p_data) {
	uint8_t dev_id;
//	static uint8_t dev_id2;
	uint8_t cmd_id;
	uint8_t cmd_id2;

	dev_id =  (p_data & 0xFF000000) >> 24;
//	dev_id2 = (p_data & 0x00FF0000) >> 16;
	cmd_id =  (p_data & 0x0000FF00) >> 8;
	cmd_id2 = (p_data & 0x000000FF);

	if (cmd_id + cmd_id2 == 0xFF) {
		printf("%u %u\r\n", dev_id, cmd_id);
	}
}

static void ir_nec_reset_transmission() {
	
	if (ir_nec_current_state != IR_NEC_NONE) {
		ir_nec_current_state = IR_NEC_NONE;

		if (ir_nec_repeat_last_command) {
			ir_print_data(ir_recv_prev_data);
		} else {
			ir_print_data(ir_recv_data);
			ir_recv_prev_data = ir_recv_data;
		}

		ir_recv_data = 0;
//		cnt = 0;
		ir_nec_repeat_last_command = FALSE;
	}
}



//识别脉冲的时间
static bool ir_nec_check_tolerance(unsigned int received, ir_nec_state current_state) {
	unsigned int expected = ir_nec_pulse_lengths[current_state];

	if (current_state == IR_NEC_NONE || current_state == IR_NEC_NDEF) {  //IR_NEC_NDEF 数据阶段，不判断时间，直接返回true
		return TRUE;
	} else if (current_state == IR_NEC_SECOND_BURST) {
		// We can receive long (4.5 ms) or short (2.25 ms) burst.
		if (received < 3000) {
			ir_nec_current_state = IR_NEC_SECOND_BURST_REPEAT;
			// ^ nasty hack, but we are can determine type of burst after receiving
			expected = ir_nec_pulse_lengths[IR_NEC_SECOND_BURST_REPEAT];
		} else {
			expected = ir_nec_pulse_lengths[IR_NEC_SECOND_BURST];
		}
	}

	uint32_t min = expected - (TOLERANCE / 100.0) * expected;   //比标准值小20%
	uint32_t max = expected + (TOLERANCE / 100.0) * expected;   //比标准值大20%
	if ((received >= min) && (received <= max)) {
		return TRUE;
	}
	return FALSE;
}

static void ir_nec_state_machine(unsigned int time) {
//	BitAction bit = 1 - GPIO_ReadInputDataBit(gpio, pin); // Invert received value
//	GPIO_WriteBit(GPIOC, GPIO_Pin_8, bit);
	static uint8_t cnt = 0;
//	printf("timer = %d\r\n",time);
	
	if (!ir_nec_check_tolerance(time, ir_nec_current_state)) {
		ir_nec_reset_transmission();
		return;
	}

	switch (ir_nec_current_state) {
		case IR_NEC_NONE:
			ir_nec_current_state = IR_NEC_FIRST_BURST;
			cnt = 0;
//			printf("0\r\n");
			break;
		case IR_NEC_FIRST_BURST:
			ir_nec_current_state = IR_NEC_SECOND_BURST;
//			printf("1\r\n");
			break;
		case IR_NEC_SECOND_BURST:
			ir_nec_current_state = IR_NEC_1;
//			printf("2\r\n");
			break;
		case IR_NEC_1:
			ir_nec_current_state = IR_NEC_NDEF; // we can receive either 0 or 1
//			printf("3\r\n");
			break;
		case IR_NEC_NDEF:
//			printf("4\r\n");
			ir_nec_current_state = IR_NEC_1;
			ir_recv_data >>= 1;   //最开始发送的是最低位
			//data = data << 1;
			if (time > 1000) {  //高电平是时间大于1.9ms ，低电平是0.5ms(发射管发射载波信号时（高电平），但是红外收的表现是低，对方发的是低，红外接收表现为高)
				ir_recv_data |= 0x80000000;   //最高位置1 
			//	data = data | 1;
			} 
//			else {
//				data = data << 1 | 0;
//			}
			++cnt;
//			if (cnt > 30)
//			{
//				printf("cnt = %d data = %#x\r\n",cnt,ir_recv_data);
//			}
			if (cnt >= 32) { //all data received
//				printf("cnt = %d data = %#x\r\n",cnt,ir_recv_data);
				cnt = 0;
				ir_recv_prev_data = ir_recv_data;
			}
			break;
		case IR_NEC_SECOND_BURST_REPEAT:
			// repeat last message
//			ir_nec_repeat_last_command = TRUE;
			ir_nec_current_state = IR_NEC_1;
			break;
		default:
			ir_nec_reset_transmission();
			break;
	}
}





//红外接收引脚，设置为中断方式。
static void ir_detect_pin_init(void)
{
	rcu_periph_clock_enable(RCU_GPIOC);	
	rcu_periph_clock_enable(RCU_AF);   //考虑到可移植性，需要添加	
	gpio_init(GPIOC, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_2MHZ, GPIO_PIN_9);	  //PC9设置为输入模式

//设置为中断方式。
#ifdef IR_DETECT_USE_IRQ
	// 设置优先级
    nvic_irq_enable(EXTI5_9_IRQn, 2U, 2U);
    
    // 设置EXTI触发源
    gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOC, GPIO_PIN_SOURCE_9);

    // 下降沿中断
    exti_init(EXTI_9, EXTI_INTERRUPT, EXTI_TRIG_BOTH); //双边缘触发
    // 清中断标志
    exti_interrupt_flag_clear(EXTI_9);
	exti_interrupt_enable(EXTI_9);
#endif	

}







//void IR_Init(void) {
//	/* Enable GPIO clock */
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

//	// Use PC6 as input from IR receiver
//	GPIO_InitTypeDef GPIO_InitStructure;

//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOC, &GPIO_InitStructure);

//	// Enable clock and its interrupts
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

//	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource6);

//    NVIC_InitTypeDef NVIC_InitStructure;
//    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure);

//	EXTI_InitTypeDef EXTI_InitStructure;
//    EXTI_InitStructure.EXTI_Line = EXTI_Line6;
//    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
//    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//    EXTI_Init(&EXTI_InitStructure);


//}

//只是用于计时，就不使用定时器7了！！！！就使用tim3了，避开TIMER2
static void TIM3_IR_RECIVER_Init(uint16_t arr,uint16_t psc)
{
//	timer_ic_parameter_struct icpara;
	timer_parameter_struct initpara;
	//接收部分
	rcu_periph_clock_enable(RCU_TIMER3);  //定时器模块时钟使能

	//3. 初始化定时器的数据结构  /* initialize TIMER init parameter struct */
	timer_struct_para_init(&initpara);
	initpara.period = arr;  //重载的数字，
	initpara.prescaler = psc;  //预分频数，得到是1Mhz的脉
	//4. 初始化定时器      /* initialize TIMER counter */
	timer_init(TIMER3, &initpara);

	//5.输入捕获初始化
//	timer_channel_input_struct_para_init(&icpara);
//	icpara.icfilter    = 3U;
//	timer_input_capture_config(TIMER7, TIMER_CH_3, &icpara);

//	nvic_irq_enable(TIMER7_Channel_IRQn, 1U, 1U);
//	timer_interrupt_enable(TIMER7, TIMER_INT_CH3);
	nvic_irq_enable(TIMER3_IRQn, 1U, 1U);
	timer_interrupt_enable(TIMER3, TIMER_INT_UP);

	
}


//初始化 ir接收部分初始化
void IR_Recv_Init(void)
{
	ir_detect_pin_init();   //引脚初始化
	ir_nec_init();    //nec数据结构初始化
	TIM3_IR_RECIVER_Init(10000-1,(SystemCoreClock/1000000)-1);   //10ms超时   1MHz == 1us
}




//接收定时器开启或者关闭 1为开启，0为关闭
void IR_Recv_Timer_Control(uint8_t enable)
{
	if(enable)
	{
		//启动定时器2
		timer_enable(TIMER3);
	}
	else
		timer_disable(TIMER3);
}




//void TIM_Init(void) {
//	// Enable clock and its interrupts
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

//    NVIC_InitTypeDef NVIC_InitStructure;
//    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure);

//	TIM_TimeBaseInitTypeDef TIM_InitStructure;

//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
//	TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
//	TIM_InitStructure.TIM_Prescaler = 24 - 1;
//	TIM_InitStructure.TIM_Period = 10000 - 1; // Update event every 10000 us / 10 ms
//	TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
//	TIM_InitStructure.TIM_RepetitionCounter = 0;
//	TIM_TimeBaseInit(TIM2, &TIM_InitStructure);
//	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
//	TIM_Cmd(TIM2, ENABLE);
//}

//int main(void)
//{
//	LED_Init();
//	IR_Init();
//	TIM_Init();
//	USART1_Init(921600);
//	ir_nec_init(GPIO_Pin_6, GPIOC);

//	printf("\r\nWorking\r\n ");

//    while(1)
//    {
//    }
//}


//定时器4的中断处理，目前是用于红外接收计时
void TIMER3_IRQHandler(void)
{
	if(timer_interrupt_flag_get(TIMER3,TIMER_INT_FLAG_UP)!=RESET)
	{
	//	printf("time3 .. timeout\r\n");
		ir_nec_reset_transmission();    //定时器超时复位接收
	}
	timer_interrupt_flag_clear(TIMER3,TIMER_INT_FLAG_UP);	
}


#ifdef IR_DETECT_USE_IRQ

//static volatile uint8_t ir_detect_time_out = 5;    //中断接收超时时间

void ir_irq9_handle(void)
{
	unsigned int counter;
	// Restart Timer
	counter = timer_counter_read(TIMER3);//TIM_GetCounter(TIM2);
	timer_counter_value_config(TIMER3, 0);//TIM_SetCounter(TIM2, 0);
	ir_nec_state_machine(counter);
}

#endif



//void EXTI9_5_IRQHandler(void)
//{
//    static unsigned int counter;

//    if (EXTI_GetITStatus(EXTI_Line6) != RESET)
//    {
//        uint8_t bit = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_6);
//        if (bit) {
//        	GPIO_ResetBits(GPIOC, GPIO_Pin_9);
//        } else {
//        	GPIO_SetBits(GPIOC, GPIO_Pin_9);
//        }

//        // Restart Timer
//        counter = TIM_GetCounter(TIM2);
//        TIM_SetCounter(TIM2, 0);
//        ir_nec_state_machine(counter);

//        EXTI_ClearITPendingBit(EXTI_Line6);
//    }
//}



//返回1表示受到红外数据，返回0表示没有收到红外数据
//返回2表示收到了错误的数据。
static uint8_t check_ir_recv_data(void)
{
	uint8_t data[4];
	
	if(ir_recv_prev_data == 0)
		return 0;

	data[3] =  (ir_recv_prev_data & 0xFF000000) >> 24;
	data[2] = (ir_recv_prev_data & 0x00FF0000) >> 16;
	data[1] =  (ir_recv_prev_data & 0x0000FF00) >> 8;
	data[0] = (ir_recv_prev_data & 0x000000FF);
	
	ir_recv_prev_data = 0; //接收数据清零
	
	if((ir_Send_DAT[0] == data[0]) &&
		(ir_Send_DAT[1] == data[1]) &&
		(ir_Send_DAT[2] == data[2]) &&
		(ir_Send_DAT[3] == data[3]))
	{
		return 1;   //发送的和接收的数据相同，返回1
	}
	else
	{
		printf("ERROR:org ir_Send_DAT[0] = %#x,ir_Send_DAT[1] = %#x,ir_Send_DAT[2] = %#x,ir_Send_DAT[3] = %#x\r\n",ir_Send_DAT[0],ir_Send_DAT[1],ir_Send_DAT[2],ir_Send_DAT[3]);
		printf("ERROR:recv data[0] = %#x,data[1] = %#x,data[2] = %#x,data[3] = %#x\r\n",data[0],data[1],data[2],data[3]);
		return 2;
	}
	
	return 0;
}




void print_laser_light_times(void)
{
	uint16_t ts = g_laser_light_timers/2;
	printf("laser_light_times : %d mins,%d secs\r\n",ts/60,ts%60);
}


//关机后要清零照射时间
void clear_laser_light_times(void)
{
	g_laser_light_timers = 0;
}



// 500ms 进入1次
//系统开机才检测，不开机就不用检测了！！！
// 红外发送也是在这的。
void ir_irq9_detect_task(void)
{
	static uint8_t cn = 0,cn1= 0;   //未佩戴检测去波动
	static uint8_t n = 0;   //未佩戴时间计时
	static uint8_t saved_laser_area_val = 0;   //保存区域值 
//	static uint16_t k = 0;
	uint8_t ret = 0;
	
	if (get_system_run_status() <= DEV_POWEROFF)  //关机模式下不发送和检测
	{
		g_laser_light_timers = 0;
		cn = 0;
		n = 0;
		return;
	}
	
	if(cn1++%2 == 0)
	{
		IR_NEC_Send_Code(ir_Send_DAT, 4);
	}
	else
	{
		ret = check_ir_recv_data();
		if(1 == ret)  //收到红外信号了
		{	
			//激光pwm关闭
		//	Laser_Pwm_Timer_Control(0);
			
			if(debug_ir_recv_mode)	//调试模式加打印		
				printf("@@@ir_recv_data\r\n");  //调试时暂时开启 2022-09-08
			
			if(get_laser_area_val())//关闭激光照射
			{
				saved_laser_area_val = get_laser_area_val();   //保存之前的值
				set_laser_area_val(0);   //关闭所有的区域
			}
			
			if(n>3)
			{				
				if(cn)
					cn = 0;   //延迟检测清零
			}
			
			//if(n == 0)
			//{
			//	pwm_all_change(0);  //
			//	DBG_PRINTF("ERROR : ir_irq9_detect_task detect device off ,and will poweroff laser\r\n");
			//}
			n++;
						
			if(debug_ir_recv_mode)	//调试模式加打印	
			{
				if(n%2)
					printf("detect device off ,2 minute countdown left time = %d s\r\n",(240-n)/2);
			}
			if(n > 240)   //2min = 120秒，1秒进入10次
			{
				DBG_PRINTF("ERROR : ir_irq9_detect_task detect device off 2 mins,and system  go to poweroff\r\n");
				g_laser_light_timers = 0;   //佩戴时间清零
				n = 0;
				system_power_off();   //关机
			}
		}
		else if(0 == ret)  //没有收到红外信号
		{
			if(saved_laser_area_val)  //重新点亮激光
				set_laser_area_val(saved_laser_area_val);
			
			if(cn < 10)
			{
				cn++;  //延迟清零未佩戴信号
				if(cn == 10)
				{
					if(n)  //清零计数值
						n = 0;					
				}
			}
			else //连续监测到了10次
			{
				//6.激光pwm开启
			//	Laser_Pwm_Timer_Control(1);
				
				if(debug_ir_recv_mode)  //调试模式加打印
					printf("no no no no ir_data\r\n");   //调试时暂时关闭 2022-09-08
								 
				 //开启激光
				 if(g_laser_light_timers == 3)   //第一次进来的时候，开启全部激光
				 {
					 pwm_all_change(40);
					 DBG_PRINTF("ir_irq9_detect_task detect someone ,and poweron laser\r\n");
				 }
				 //判断时间是否到了最长时间限制
				  g_laser_light_timers ++;   //佩戴时间计时开始
				 
	//			 if(g_laser_light_timers == 10)  //连续10次监测不到数据就清零
	//			 {
	//				
	//			 }
				 
				 
				 if(g_laser_light_timers > (60*28*2))   //49200 小于65535
				 {
					 DBG_PRINTF("ir_irq9_detect_task task is run 28 mins,and system poweroff\r\n");
					 system_power_off();   //时间到关机
					 g_laser_light_timers = 0;				 
				 }
			}
		}
		else // if(2 == ret)  //错误的码
		{
			//不操作
		}
	}
}

