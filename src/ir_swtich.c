

#include "includes.h"

/*
	PC9  ir_out ������ն�   timer2ch2 ȫӳ��  TIMER7_CH3
	PC8  ���ⷢ���  		timer2ch3 ȫӳ��  TIMER7_CH2
*/



/**
* @breaf ����ҡ�������������Ϣ��������մ������Ľ����źţ��жϽ��յ�����0 ���� 1,
* �ƽ�ҡ���Ŀ�����
* 
* detailed ʹ�ö�ʱ���Ĳ����ܣ���ȡ������մ�����������������ȣ�us�ߵ�ƽ����
* ʵ�ַ���������ʱ����ʼ��Ϊ���������أ����������أ���ն�ʱ������ֵ������ʱ��
* ����Ϊ�����½��أ������½��أ�����ʱ��������ֵ������ʱ������Ϊ���������ء���
* �϶�ȡ���ļ�����ֵ 450���� Ϊ1�� 1600����Ϊ 0��4500����Ϊ ������ ����ͬ�ĺ����ź�
* ���뷽ʽ��ͬ��Ҫ���ݲ�ͬ���ź�����������
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

//ң��������״̬
//[7]:�յ����������־ [6]:�õ���һ��������������Ϣ
//[5]:���� [4]:����������Ƿ��Ѿ������� [3:0]:�����ʱ��
static uint8_t RmtSta=0;
static uint16_t Dval; //�½���ʱ��������ֵ
static uint8_t RmtRec=0; //������յ�������
static uint8_t RmtCnt=0; //�������µĴ���
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
        if(gpio_input_bit_get(GPIOB, GPIO_PIN_8))        //��������û�أ���ȡGPIO PIN״̬��Ϊ1ʱ��������
        {
            timer_counter_value_config(TIMER3, 0);
            timer_icinitpara.icpolarity  = TIMER_IC_POLARITY_FALLING;
            timer_icinitpara.icselection = TIMER_IC_SELECTION_DIRECTTI;
            timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;
            timer_icinitpara.icfilter    = 0;
            timer_input_capture_config(TIMER3,TIMER_CH_2,&timer_icinitpara);
        }
        else        ////�����½��أ�Ϊ0���½���
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
                    RmtRec>>=1; //����һλ
                    RmtRec|=0<<7; //���յ� 0
                    iri++;
                }
                else if(Dval>1200 && Dval<1900)
                {
                    RmtRec>>=1; //����һλ
                    RmtRec|=1<<7; //���յ� 1
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
    if(SET == timer_interrupt_flag_get(TIMER3,TIMER_INT_FLAG_UP))     //��ʱ����ʱ��û�л񲶻��κ��ź�
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



#if 0

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
 
 GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);//�ر�JATG������SWD
 GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2,ENABLE);//��TIM2_CH2ӳ�䵽PB3
 
 GPIO_InitStruct.GPIO_Mode=GPIO_Mode_IN_FLOATING;
 GPIO_InitStruct.GPIO_Pin=GPIO_Pin_3;
 GPIO_Init(GPIOB,&GPIO_InitStruct);
 //��ʱ��TIM2����
 TIM_TimeBaseInitStruct.TIM_ClockDivision=TIM_CKD_DIV2;
 TIM_TimeBaseInitStruct.TIM_CounterMode=TIM_CounterMode_Up;
 TIM_TimeBaseInitStruct.TIM_Period=arr-1;
 TIM_TimeBaseInitStruct.TIM_Prescaler=psc-1;
 TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStruct);
 //���벶��
 TIM_ICInitStruct.TIM_Channel=TIM_Channel_2;//ͨ��2
 TIM_ICInitStruct.TIM_ICFilter=0x00;//��ʹ���˲���
 TIM_ICInitStruct.TIM_ICPolarity=TIM_ICPolarity_Falling;//�½��ز���
 TIM_ICInitStruct.TIM_ICPrescaler=TIM_ICPSC_DIV1;//����Ƶ
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
//38KHz�ز�    -- ��Ƶ 72  -- 1M  --��װ��ֵ -- 26 
void TIM1_Config(u16 psc,u16 arr,u16 ccr)
{ 
 GPIO_InitTypeDef GPIO_InitStruct;
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_TIM1,ENABLE);
 //����PA8 Ϊ�����������
 GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
 GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 
 GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;
 GPIO_Init(GPIOA,&GPIO_InitStruct);  //GPIO��ʼ��
 
 TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
 TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV2;
 TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;//���ϼ���
 TIM_TimeBaseInitStruct.TIM_Period = arr-1;//��װ��ֵ
 TIM_TimeBaseInitStruct.TIM_Prescaler = psc-1;//��Ƶֵ
 TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
 TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitStruct);


 TIM_OCInitTypeDef TIM_OCInitStruct;
 TIM_OCInitStruct.TIM_Pulse = ccr;//�Ƚ�ֵ
 TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2;//PWM2ģʽ
 TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High; //���ԣ���
 TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;//�Ƚ����ʹ��
 TIM_OCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Set; //����Ϊ�ߣ��Լ��Է�ת������
 
 TIM_OC1Init(TIM1,&TIM_OCInitStruct);
 
 TIM_CtrlPWMOutputs(TIM1,ENABLE);//�����ʹ��
 
 TIM_Cmd(TIM1,DISABLE);
}

uint16_t ir_buff[1024]; //��Ų���ļ�������ֵ
uint16_t ir_count=0; //������صĸ���
void TIM2_IRQHandler(void)
{
 uint16_t i=0;
 static uint16_t count = 0;
 if(TIM_GetITStatus(TIM2,TIM_IT_Update))//�����ж�
 {
  TIM_ClearFlag(TIM2,TIM_FLAG_Update); //���ж�
  if(count>40)
  {
  
//   for(i=0;i<count;i++)
//   {
//    printf("%d\r\n",ir_buff[i]);
//   }
   ir_count=count;
   irrun[1] = 2000; //����ʱ���
   count = 0;
   ir.overflag = 1;
  }
 }
 if(TIM_GetITStatus(TIM2,TIM_IT_CC2))//�����ж�
 {
  TIM_ClearFlag(TIM2,TIM_FLAG_CC2); //���ж�
  ir_buff[count++]=TIM_GetCapture2(TIM2);
  TIM2->CCER ^= (1<<5);     //���Է�ת,����/����ͨ��2�������
  TIM_SetCounter(TIM2,0); //����������
 }
}
/*
�ȽϺ�����װ
time1 -- Ҫ��������
time2 -- ��׼����
range1 -- ����
range2 -- ����
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
		sFLASH_EraseSector(0x060000);//������������6��
		sFLASH_WriteBuffer((uint8_t *)&ir,0x060000,sizeof(ir));//д������
	}
	IR_empty();
}

void IR_Display(void)
{
	u32 i=0,j=0;
	for(j=0;j<second+1;j++)
	{
		//��ӡ������
		printf("��%d��������\r\n",j+1);
		for(i=0;i<ir.ir_boot.boot_len[j];i++)
		{
			printf("%d\t",ir.ir_boot.boot_time[j][i]);
		}
		printf("\r\n");
		printf("��%d������\r\n",j+1);
		for(i=0;i<ir.datalen[j]/8;i++)
		{
			printf("%d\t",ir.irdata[j][i]);
		}
		printf("\r\n");
		if((second!=0)&&(sign==0))
		{
			printf("�ָ���\r\n");
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
	printf("�������\r\n");
	printf("\r\n");
}

#endif

