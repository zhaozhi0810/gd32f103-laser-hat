

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

#define PWM_HZ 10   //����pwm��Ƶ�ʣ���ʱ��ÿ10ms����һ�Σ���Ϊ100HZ

static uint8_t laser_area_control = 0x7f;  //��ʾ7������ȫ�����������ö�Ӧ������ 


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
	
	MY_PRINTF("%s %d\r\n",__FUNCTION__,__LINE__);
}

void laser_disable(void);

// ����ʹ�ܣ�area8λ����7λÿһλ��ʾһ������1��ʾʹ�ܿ�����0��ʾ�ر�
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
	
	//����5v�����Դ����
	output_5v_enable();
	
//	MY_PRINTF("%s %d area = %d\r\n",__FUNCTION__,__LINE__,area);
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
	//�ر�5v�����Դ����
//	output_5v_disable();  //����ʱ�ر�09-08
	
//	gpio_bit_reset(GPIOA, GPIO_PIN_12);  //PWM_M
	gpio_bit_reset(GPIOB, GPIO_PIN_3 | GPIO_PIN_4 |GPIO_PIN_5);
	gpio_bit_reset(GPIOC, GPIO_PIN_12 | GPIO_PIN_10 |GPIO_PIN_11);
	gpio_bit_reset(GPIOD, GPIO_PIN_2);
	
//	MY_PRINTF("%s %d\r\n",__FUNCTION__,__LINE__);
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
	
	MY_PRINTF("%s %d ch = %d degree = %d\r\n",__FUNCTION__,__LINE__,ch,degree);
}


/*
ȫ��ͨ������Ϊĳһ��ֵ
//degree Ϊ��Ҫ������ֵ��0-10
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
	
//	MY_PRINTF("%s %d degree = %d\r\n",__FUNCTION__,__LINE__,degree);
}


//����һ������ļ��� areaȡֵ0-6
void laser_add_a_area(uint8_t area)
{
	if(area < 7)
		laser_area_control |= 1<<area;
}

//����һ������ļ��� areaȡֵ0-6
void laser_sub_a_area(uint8_t area)
{
	if(area < 7)
		laser_area_control &= ~(1<<area);
}



uint8_t get_laser_area_val(void)
{
	return laser_area_control;
}




//100HZ��Ƶ�ʣ�10ms����һ��
void laser_run_pwm_task(void)
{
	static uint16_t count = 0;
	uint8_t i;
	
	//�ػ�״̬�²����Ƽ�����
	if(get_system_run_status() <= DEV_POWEROFF)  //�м���״̬�ǲ�������
	{
		count = 0;
		laser_enable(0);   //ȷ������ȫ���ر�
		return;
	}
	//����ʱ��ʱ�ر� 2022-09-08
	laser_enable(0x7f);   //����״̬���Ƽ����������ʾ  //����ʱ��ʱ�ر� 2022-09-08
	return 0;   //����ʱ��ʱ�ر� 2022-09-08
	if(count < PWM_HZ)
	{
		g_pwm_status = laser_area_control;   //Ĭ������ȫ����������laser_control������Щ����Ҫ����
		
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


