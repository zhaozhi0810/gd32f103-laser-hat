

#include "includes.h"

/*
	PC9  ir_out 红外接收段   timer2ch2 全映射  TIMER7_CH3
	PC8  红外发射管  		timer2ch3 全映射  TIMER7_CH2


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



void ir_pwm_init(void)
{
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
}



//开启定时器则发送38Khz频率，此时接收器的数据为1，输出电平为0
void ir_send_high(void)
{
	timer_enable(PWM_TIMER);   //开启定时器}
}

//关闭定时器则不再发送38Khz频率，此时接收器的数据为0，输出电平为1
void ir_send_low(void)
{
	timer_disable(PWM_TIMER);   //开启定时器}
}



//红外接收引脚，设置为中断方式。
void ir_detect_init(void)
{
	rcu_periph_clock_enable(RCU_GPIOC);			
	gpio_init(GPIOC, GPIO_MODE_IPU, GPIO_OSPEED_2MHZ, GPIO_PIN_9);	  //PC9设置为输入模式

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

//关闭irq检测，关机时就不需要了。
void ir_detect_off(void)
{
	MY_PRINTF("%s %d\r\n",__FUNCTION__,__LINE__);
	ir_send_low(); //红外发射停止
	
	exti_interrupt_disable(EXTI_9); //中断禁止
	nvic_irq_disable(EXTI5_9_IRQn);   //中断禁止
}



#ifdef IR_DETECT_USE_IRQ

static uint16_t ir_detect_time_out = 5;    //中断接收超时时间

void ir_irq9_handle(void)
{
	ir_detect_time_out = 5;   // 5次就是500ms的时间
}


// 100ms 进入1次
//系统开机才检测，不开机就不用检测了！！！
// 红外发送也是在这的。
void ir_irq9_detect_task(void)
{
	static uint16_t n = 0;   //未佩戴时间计时
	static uint16_t count = 0; //用于计时佩戴的时间
	static uint16_t k = 0;
	
	if(get_system_run_status() == DEV_POWEROFF) //关机后，不再进行计时操作
	{
		n = 0;
		count = 0;
		k = 0;
		return ;
	}
	
	
	if(ir_detect_time_out)  //未佩戴的时候，激光关闭
	{
		ir_detect_time_out -- ;  //倒计时减少
		
		//关闭激光照射
		if(n == 0)
		{
			pwm_all_change(0);  //
			DBG_PRINTF("ERROR : ir_irq9_detect_task detect device off ,and poweroff laser\r\n");
		}
		n++;
		
		if(n > 1200)   //2min = 120秒，1秒进入10次
		{
			DBG_PRINTF("ERROR : ir_irq9_detect_task detect device off 2 mins,and system  go to poweroff\r\n");
			count = 0;   //佩戴时间清零
			system_power_off();   //关机
		}
		
	}
	else  //没有接收到信号了，表示佩戴了头盔
	{
		 if(n)  //清零计数值
			 n = 0;
		 		 
		 //开启激光
		 if(count == 0)   //第一次进来的时候，开启全部激光
		 {
			 pwm_all_change(100);
			 DBG_PRINTF("ir_irq9_detect_task detect someone ,and poweron laser\r\n");
		 }
		 //判断时间是否到了最长时间限制
		  count ++;   //佩戴时间计时开始
		 if(count > (60*28*10))   //49200 小于65535
		 {
			 DBG_PRINTF("ir_irq9_detect_task task is run 28 mins,and system poweroff\r\n");
			 system_power_off();   //时间到关机
			 count = 0;
			 
		 }		 
	}
	
	
	//k 是用于发送红外的，高低电平的翻转，不断的触发中断
	k++;
	if(k == 1)
		ir_send_high();   //发送高电平，100ms
	else if(k < 5)
		ir_send_low();   //发送低电平，300ms
	else	
		k = 0;
}



#endif

#if 0


/**
* @breaf 红外摇控器发射红外信息，红外接收传感器的接收信号，判断接收到的是0 还是 1,
* 破解摇器的控制码
* 
* detailed 使用定时器的捕获功能；读取红外接收传感器输出引脚脉冲宽度（us高电平）。
* 实现方法，将定时器初始化为捕获上升沿，捕获到上升沿，清空定时器计数值，将定时器
* 设置为捕获到下降沿，捕获到下降沿，读定时器计数器值，将定时器配置为捕获上升沿。将
* 断读取到的计数器值 450左右 为1， 1600左右为 0，4500左右为 引导码 ，不同的红外信号
* 编码方式不同，要根据不同的信号作出调整；
*/
__IO uint16_t readvalue1 = 0, readvalue2 = 0;
__IO uint16_t ccnumber = 0;
__IO uint32_t count = 0;
__IO float fre = 0;
static    timer_ic_parameter_struct timer_icinitpara;
static    timer_parameter_struct timer_initpara;



void ir_rx_init(void)
{
    timer_ic_parameter_struct timer_icinitpara;
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_AF);
    rcu_periph_clock_enable(RCU_TIMER3);

    /*configure PB8 (TIMER3 CH3) as alternate function*/
    gpio_init(GPIOB, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_8);

    timer_deinit(TIMER3);
    /* TIMER3 configuration */
    timer_initpara.prescaler         = 107;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 10000;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER3,&timer_initpara);

    /* TIMER3 CH3 input capture configuration */
    timer_icinitpara.icpolarity  = TIMER_IC_POLARITY_RISING;
    timer_icinitpara.icselection = TIMER_IC_SELECTION_DIRECTTI;
    timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;
    timer_icinitpara.icfilter    = 0;
    timer_input_capture_config(TIMER3,TIMER_CH_2,&timer_icinitpara);

    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER3);
    /* clear channel 3 interrupt bit */
    timer_interrupt_flag_clear(TIMER3,TIMER_INT_FLAG_CH2|TIMER_INT_FLAG_UP);
    timer_interrupt_enable(TIMER3,TIMER_INT_CH2|TIMER_INT_UP);
    /* channel 3 interrupt enable */
    nvic_irq_enable(TIMER3_IRQn, 1, 0);

    /* TIMER3 counter enable */
    timer_enable(TIMER3);


}

//遥控器接收状态
//[7]:收到了引导码标志 [6]:得到了一个按键的所有信息
//[5]:保留 [4]:标记上升沿是否已经被捕获 [3:0]:溢出计时器
static uint8_t RmtSta=0;
static uint16_t Dval; //下降沿时计数器的值
static uint8_t RmtRec=0; //红外接收到的数据
static uint8_t RmtCnt=0; //按键按下的次数
static uint8_t iri=0, irj=0;
uint8_t IrCodeSize = 6;
uint8_t ir_rx_data[20] = {0};
/**
  * @brief  This function handles TIMER3 interrupt request.
  * @param  None
  * @retval None
  */
void TIMER3_IRQHandler(void)
{
    if(SET == timer_interrupt_flag_get(TIMER3,TIMER_INT_FLAG_CH2))
    {
        if(gpio_input_bit_get(GPIOB, GPIO_PIN_8))        //捕获到上升没沿，读取GPIO PIN状态当为1时是上升沿
        {
            timer_counter_value_config(TIMER3, 0);
            timer_icinitpara.icpolarity  = TIMER_IC_POLARITY_FALLING;
            timer_icinitpara.icselection = TIMER_IC_SELECTION_DIRECTTI;
            timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;
            timer_icinitpara.icfilter    = 0;
            timer_input_capture_config(TIMER3,TIMER_CH_2,&timer_icinitpara);
        }
        else        ////捕获到下降沿，为0是下降沿
        {
            Dval = timer_counter_read(TIMER3);
            timer_icinitpara.icpolarity  = TIMER_IC_POLARITY_RISING;
            timer_icinitpara.icselection = TIMER_IC_SELECTION_DIRECTTI;
            timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;
            timer_icinitpara.icfilter    = 0;
            timer_input_capture_config(TIMER3,TIMER_CH_2,&timer_icinitpara);
            if (RmtSta & 1<<7)
            {
                if(Dval>300 && Dval<800)
                {
                    RmtRec>>=1; //右移一位
                    RmtRec|=0<<7; //接收到 0
                    iri++;
                }
                else if(Dval>1200 && Dval<1900)
                {
                    RmtRec>>=1; //右移一位
                    RmtRec|=1<<7; //接收到 1
                    iri++;
                }
                else if((Dval>4750 && Dval<5500))
                {
                    RmtSta &= ~(1<<7);
                }
                if(iri==8)
                {
                    iri=0;
                    ir_rx_data[irj]=RmtRec;
                    irj++;
                    RmtRec = 0;
                }
                if(irj==IrCodeSize)
                {
                    for (iri=0; iri<IrCodeSize; iri++)
                    {

                        usart_data_transmit(UART4, ir_rx_data[iri]);
                        while(!usart_flag_get(UART4,USART_FLAG_TBE));

                    }
                }
            }
            else if((Dval>4200 && Dval<4700)&& (!(RmtSta & 1<<7)))
            {
                RmtSta |= 1<<7;
                RmtRec = 0;
                iri = 0;
            }
        }
        /* clear channel 3 interrupt bit */
        timer_interrupt_flag_clear(TIMER3,TIMER_INT_FLAG_CH2);
    }
    if(SET == timer_interrupt_flag_get(TIMER3,TIMER_INT_FLAG_UP))     //定时器超时，没有获捕获到任何信号
    {
        timer_icinitpara.icpolarity  = TIMER_IC_POLARITY_RISING;
        timer_icinitpara.icselection = TIMER_IC_SELECTION_DIRECTTI;
        timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;
        timer_icinitpara.icfilter    = 0;
        timer_input_capture_config(TIMER3,TIMER_CH_2,&timer_icinitpara);
        timer_interrupt_flag_clear(TIMER3,TIMER_INT_FLAG_UP);
        irj = 0;
        iri = 0;
        RmtSta &= ~(1<<7);
    }
}





// A code block
#include "ir.h"
_IR ir={.count=0,.overflag=0};
int second=0;


void IR_Config(void)
{
  TIM2_Config(72,20000);  //20ms
  TIM1_Config(72,26,13);  //38KHz  50%
  sFLASH_ReadBuffer((u8 *)&ir,0x60000,sizeof(ir));
  if(*(uint8_t *)&ir!=0xff)
  {
    second=ir.frequency;
    IR_Display();
  }
}

void TIM2_Config(uint16_t psc,uint16_t arr)
{
 //PB3
 GPIO_InitTypeDef GPIO_InitStruct;
 TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
 TIM_ICInitTypeDef TIM_ICInitStruct;
 NVIC_InitTypeDef NVIC_InitStruct;
 
 
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO,ENABLE);
 RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
 
 GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);//关闭JATG，启用SWD
 GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2,ENABLE);//将TIM2_CH2映射到PB3
 
 GPIO_InitStruct.GPIO_Mode=GPIO_Mode_IN_FLOATING;
 GPIO_InitStruct.GPIO_Pin=GPIO_Pin_3;
 GPIO_Init(GPIOB,&GPIO_InitStruct);
 //定时器TIM2配置
 TIM_TimeBaseInitStruct.TIM_ClockDivision=TIM_CKD_DIV2;
 TIM_TimeBaseInitStruct.TIM_CounterMode=TIM_CounterMode_Up;
 TIM_TimeBaseInitStruct.TIM_Period=arr-1;
 TIM_TimeBaseInitStruct.TIM_Prescaler=psc-1;
 TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStruct);
 //输入捕获
 TIM_ICInitStruct.TIM_Channel=TIM_Channel_2;//通道2
 TIM_ICInitStruct.TIM_ICFilter=0x00;//不使用滤波器
 TIM_ICInitStruct.TIM_ICPolarity=TIM_ICPolarity_Falling;//下降沿捕获
 TIM_ICInitStruct.TIM_ICPrescaler=TIM_ICPSC_DIV1;//不分频
 TIM_ICInitStruct.TIM_ICSelection=TIM_ICSelection_DirectTI;// IC2
 TIM_ICInit(TIM2,&TIM_ICInitStruct);
 
 
 TIM_ITConfig(TIM2,TIM_IT_CC2|TIM_IT_Update,ENABLE);
 
 NVIC_InitStruct.NVIC_IRQChannel=TIM2_IRQn;
 NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;
 NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=0;
 NVIC_InitStruct.NVIC_IRQChannelSubPriority=1;
 NVIC_Init(&NVIC_InitStruct);
 
 TIM_Cmd(TIM2,ENABLE);
}

//PA8  -- TIM1_CH1  
//38KHz载波    -- 分频 72  -- 1M  --重装载值 -- 26 
void TIM1_Config(u16 psc,u16 arr,u16 ccr)
{ 
 GPIO_InitTypeDef GPIO_InitStruct;
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_TIM1,ENABLE);
 //配置PA8 为复用推挽输出
 GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
 GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 
 GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;
 GPIO_Init(GPIOA,&GPIO_InitStruct);  //GPIO初始化
 
 TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
 TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV2;
 TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;//向上计数
 TIM_TimeBaseInitStruct.TIM_Period = arr-1;//重装载值
 TIM_TimeBaseInitStruct.TIM_Prescaler = psc-1;//分频值
 TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
 TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitStruct);


 TIM_OCInitTypeDef TIM_OCInitStruct;
 TIM_OCInitStruct.TIM_Pulse = ccr;//比较值
 TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2;//PWM2模式
 TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High; //极性：高
 TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;//比较输出使能
 TIM_OCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Set; //空闲为高，对极性反转起作用
 
 TIM_OC1Init(TIM1,&TIM_OCInitStruct);
 
 TIM_CtrlPWMOutputs(TIM1,ENABLE);//主输出使能
 
 TIM_Cmd(TIM1,DISABLE);
}

uint16_t ir_buff[1024]; //存放捕获的计数器的值
uint16_t ir_count=0; //保存边沿的个数
void TIM2_IRQHandler(void)
{
 uint16_t i=0;
 static uint16_t count = 0;
 if(TIM_GetITStatus(TIM2,TIM_IT_Update))//更新中断
 {
  TIM_ClearFlag(TIM2,TIM_FLAG_Update); //清中断
  if(count>40)
  {
  
//   for(i=0;i<count;i++)
//   {
//    printf("%d\r\n",ir_buff[i]);
//   }
   ir_count=count;
   irrun[1] = 2000; //启动时间点
   count = 0;
   ir.overflag = 1;
  }
 }
 if(TIM_GetITStatus(TIM2,TIM_IT_CC2))//捕获中断
 {
  TIM_ClearFlag(TIM2,TIM_FLAG_CC2); //清中断
  ir_buff[count++]=TIM_GetCapture2(TIM2);
  TIM2->CCER ^= (1<<5);     //极性反转,输入/捕获通道2输出极性
  TIM_SetCounter(TIM2,0); //计数器清零
 }
}
/*
比较函数封装
time1 -- 要检测的数据
time2 -- 标准数据
range1 -- 下限
range2 -- 上限
*/
uint32_t guide=0;
uint8_t sign=0;
uint8_t Time_Judge(uint16_t time1,uint16_t time2,uint16_t range1,uint16_t range2)
{
 if(time1 > (time2-range1) && time1 < (time2+range2))
  return 1;
 else 
  return 0;
}

void IR_Bootcode(void)
{
 sign=0;
 for(++guide;guide<ir.count;guide++)
 {
  if(Time_Judge(ir_buff[guide],4500,2000,5000)) //2500~9500
  {
   ir.ir_boot.boot_time[second][ir.ir_boot.boot_len[second]]=ir_buff[guide];
   ir.ir_boot.boot_len[second]++;
   sign=1;
   printf("%d\r\n",ir_buff[guide]);
  }
  else
  {
   break;
  }
 }
}

void IR_Datacode(void)
{
 sign=0;
 for(guide+=1;guide<ir.count;guide+=2)
 {
  if(Time_Judge(ir_buff[guide],560,200,200))
  {
   ir.irdata[second][ir.datalen[second]/8] &=~ (1<<(7-ir.datalen[second]%8));
   ir.datalen[second]++;
   sign=3;
  }
  else if(Time_Judge(ir_buff[guide],1690,200,200))
  {
   ir.irdata[second][ir.datalen[second]/8] |= (1<<(7-ir.datalen[second]%8));
   ir.datalen[second]++;
   sign=2;
  }
  else
  {
   break;
  }
 }
}

void IR_Cutcode(void)
{
	if(Time_Judge(ir_buff[guide],5200,500,500))
	{
		ir.ir_boot.cut_time=ir_buff[guide];
		second=1;
		printf("%d\r\n",ir_buff[guide]);
	}
}

void IR_empty(void)
{
	memset(&ir,0,sizeof(ir));
	second=0;
	guide=0; 
	sign=0;
}

void IR_Transformation(uint8_t a)
{
	guide=0;
	second=0;
	irrun[1] = 0xffffffff;
	if((ir.overflag ==1)&&(a==1))
	{
		memset(&ir,0,sizeof(ir));
		ir.count = ir_count;
		for(u8 i=0;i<second+1;i++)
		{

			IR_Bootcode();
			IR_Datacode();
			IR_Cutcode();
			if((sign==0)||(ir.datalen[i]/8!=6))
			{
				IR_empty();
				return;
			}
			ir.frequency=second;
		}
		ir.count = 0;
		ir.overflag = 0;
		sign=0;
		IR_Display();
		sFLASH_EraseSector(0x060000);//擦除器擦除第6块
		sFLASH_WriteBuffer((uint8_t *)&ir,0x060000,sizeof(ir));//写缓存区
	}
	IR_empty();
}

void IR_Display(void)
{
	u32 i=0,j=0;
	for(j=0;j<second+1;j++)
	{
		//打印引导码
		printf("第%d次引导码\r\n",j+1);
		for(i=0;i<ir.ir_boot.boot_len[j];i++)
		{
			printf("%d\t",ir.ir_boot.boot_time[j][i]);
		}
		printf("\r\n");
		printf("第%d次数据\r\n",j+1);
		for(i=0;i<ir.datalen[j]/8;i++)
		{
			printf("%d\t",ir.irdata[j][i]);
		}
		printf("\r\n");
		if((second!=0)&&(sign==0))
		{
			printf("分割码\r\n");
			printf("%d\r\n",ir.ir_boot.cut_time);
			sign=1;
		}
		printf("\r\n");
	}
}

void IR_Bootsend(void)
{
	for(guide=0;guide<ir.ir_boot.boot_len[second];guide++)
	{
		IR_38KHz(sign);
		sign=!sign;
		Delay_nus(ir.ir_boot.boot_time[second][guide]);
	}
	for(guide=0;guide<ir.datalen[second]/8;guide++)
	{
		for(u32 i=0;i<8;i++)
		{
			IR_38KHz(sign);
			sign=!sign;
			Delay_nus(520);
			IR_38KHz(sign);
			sign=!sign;
			if(ir.irdata[second][guide]&(1<<(7-i)))
			{
				Delay_nus(1500);
			}
			else
				Delay_nus(520);
		}
	}
	IR_38KHz(sign);
	sign=!sign;
	Delay_nus(520);
	IR_38KHz(sign);
	sign=!sign;
	Delay_nus(ir.ir_boot.cut_time);
	second=1;
} 

void IR_Sendout(void)
{
	sign=1;
	second=0;
	for(u32 i=0;i<second+1;i++)
	{
		IR_Bootsend();
	}
	printf("发送完成\r\n");
	printf("\r\n");
}

#endif

