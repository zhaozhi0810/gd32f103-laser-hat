

/*
	2022-07-19 
	pwm ��ʼ�����������ǲ�ʹ�ö�ʱ���Դ���pwm
	
	PB4 3,5,PD2,PC12,11,10,  ��ӦPWM1-7 
	PB4 timer2ch0������ӳ�� 10
	PB3 timer1ch1������ӳ�� 01
	PB5 timer2ch1������ӳ�� 10
	PD2  TIMER2_ETI
	PC12 ��timer
	PC11  ��timer
	PC10  ��timer
	
	PA12  PWM_M
	
	PA8��PA11 ���ӵ�оƬNC���Ͳ���Ҫ�����ˡ�
	
	PC9  ir_out ������ն�   timer2ch2 ȫӳ��
	PC8  ���ⷢ���  		timer2ch3 ȫӳ��
*/

#include "includes.h"

	
//uint16_t PWM_DEGREE_MAX = 4000;   //PWMƵ��  1us�Ƹ�����4000�����Ƶ�ʾ���1000000/4000=250Hz ̫����Ӱ���ż���ʵ�飿����
uint8_t g_pwm[7] = {0};   //ÿһ��ͨ�����ò�ͬ��pwmֵ��pwm��Χ0-100��һ�������7������ͬʱ�仯��
static uint8_t g_pwm_status = 0;  //0-6λ��1��ʾ�������⣬0��ʾ�رռ���
#define PWM_HZ 100   //����pwm��Ƶ�ʣ���ʱ��ÿ10ms����һ�Σ���Ϊ100HZ




//������ͨ��io�˿ڣ�ʹ�ö�ʱ��ȥģ��pwm
void laser_control_init(void)
{
			//ʱ��ʹ��
	rcu_periph_clock_enable(RCU_GPIOA);	
	rcu_periph_clock_enable(RCU_GPIOB);	
	rcu_periph_clock_enable(RCU_GPIOC);	
	rcu_periph_clock_enable(RCU_GPIOD);	
		
	gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ, GPIO_PIN_12);
	gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ, GPIO_PIN_3 | GPIO_PIN_4 |GPIO_PIN_5 );
	gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ, GPIO_PIN_12 | GPIO_PIN_10 |GPIO_PIN_11 );
	gpio_init(GPIOD, GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ, GPIO_PIN_2);
	
	//�͵�ƽ,��Ч���ߵ�ƽʹ��
	gpio_bit_reset(GPIOA, GPIO_PIN_12);  //PWM_M
	gpio_bit_reset(GPIOB, GPIO_PIN_3 | GPIO_PIN_4 |GPIO_PIN_5);
	gpio_bit_reset(GPIOC, GPIO_PIN_12 | GPIO_PIN_10 |GPIO_PIN_11);
	gpio_bit_reset(GPIOD, GPIO_PIN_2);
}

void laser_disable(void);

// ����ʹ�ܣ�area8λ����7λÿһλ��ʾһ������1��ʾʹ�ܿ�����0��ʾ�ر�
void laser_enable(unsigned char area)
{
	if(!area)
	{
		laser_disable();
		return;
	}
	
	if(get_system_run_status() == DEV_POWEROFF)
	{
		laser_disable();
		return;
	}
	
//	gpio_bit_set(GPIOA, GPIO_PIN_12);  //PWM_M
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
	
//	gpio_bit_set(GPIOB, GPIO_PIN_3 | GPIO_PIN_4 |GPIO_PIN_5);
//	gpio_bit_set(GPIOC, GPIO_PIN_12 | GPIO_PIN_10 |GPIO_PIN_11);
//	gpio_bit_set(GPIOD, GPIO_PIN_2);

}

// �����ֹ��
static void laser_disable(void)
{
	//1. �ر� pl2628
	//�͵�ƽ,��Ч���ߵ�ƽʹ��
//	gpio_bit_reset(GPIOA, GPIO_PIN_12);  //PWM_M
	gpio_bit_reset(GPIOB, GPIO_PIN_3 | GPIO_PIN_4 |GPIO_PIN_5);
	gpio_bit_reset(GPIOC, GPIO_PIN_12 | GPIO_PIN_10 |GPIO_PIN_11);
	gpio_bit_reset(GPIOD, GPIO_PIN_2);
}




/*
����lcd����ռ�ձ�
//degree �޸�Ϊ0-100
ch ��ʾ�ĸ�����0-6 ��7������
*/
void pwm_out(uint8_t ch,uint8_t degree)
{
	if(ch > 6)
		return;
	
	if(degree>100)
		degree = 100;
	
	
	g_pwm[ch] = degree;
}


/*
ȫ��ͨ������Ϊĳһ��ֵ
//degree Ϊ��Ҫ������ֵ��0-100
*/
void pwm_all_change(uint8_t degree)
{
	uint8_t i;
	if(degree>100)
		degree = 100;

	for(i=0;i<7;i++)
	{
		g_pwm[i] = degree; 
					
	}
}



//100HZ��Ƶ�ʣ�10ms����һ��
void laser_run_pwm_task(void)
{
	static uint16_t count = 0;
	uint8_t i;
	
	//�ػ�״̬�²����Ƽ�����
	if(get_system_run_status() == DEV_POWEROFF)
	{
		count = 0;
		laser_enable(0);   //ȷ������ȫ���ر�
		return;
	}
	
	
	if(count < PWM_HZ)
	{
		g_pwm_status = 0x7f;   //Ĭ������ȫ������
		
		for(i=0;i<7;i++)
		{
			if(g_pwm[i] <= count) //����ֵ���趨ֵҪ�󣬹ر�
			{
				g_pwm_status &= ~(1<<i);   //�����ʾ�ر�
			}			
		}
		laser_enable(g_pwm_status);   //����״̬���Ƽ����������ʾ
	}
	else
	{
		count = 0;   //һ�����ڽ��������¿�ʼ��һ������
		return;   //�ո�����Ͳ���ȥ����
	}
	count++;
}


