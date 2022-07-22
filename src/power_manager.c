

#include "includes.h"


/*
	1.�ڵ�ع��������£�����ԭ��ͼ����������ʹ��Ƭ��ͨ�磬
	2.��������ɵ�Ƭ�������Ƿ񹩵� BT3V   PB13����   PC7���Ի�ȡ�������
	3.����֪���ⲿ�Ƿ������ⲿ��Դ PB12  vcc 1/2��ѹ
	4.PC3��Ƭ���ɰ��������ϵ����Ҫ����Ϊ�ߣ�����Ƭ���ϵ�
	5.�����ⲿusb��Դ��ʱ�򣬵�Ƭ��һֱͨ��
*/



void PowerManager_init(void)
{
	//1.ʱ��ʹ��
	rcu_periph_clock_enable(RCU_GPIOB);	
	rcu_periph_clock_enable(RCU_GPIOC);	
	
	//2.����3.3v��������
	gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ, GPIO_PIN_13);	 //���蹩������
	gpio_bit_reset(GPIOB, GPIO_PIN_13);  //�����޵�
	
	//3.��Ƭ����Դ�������ţ�����Ч
	gpio_bit_set(GPIOC, GPIO_PIN_3);  //��Ƭ����Դ����ʹ�ܣ���Ƭ�����е�
	gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ, GPIO_PIN_3);	 //��Ƭ����Դ��������
	
	//4. ����Ϊ����ģʽ	
	gpio_init(GPIOB, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_2MHZ, GPIO_PIN_12);  //�ⲿ�Ƿ������ⲿ��Դ
	//5. ����Ϊ����ģʽ	
	gpio_init(GPIOC, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_2MHZ, GPIO_PIN_7);   //3.3���Ƿ����
	
	
	//6. ���״̬�����ų�ʼ��
	//6.1 ���״̬��ȡ
	rcu_periph_clock_enable(RCU_GPIOC);			
	gpio_init(GPIOC, GPIO_MODE_IPU, GPIO_OSPEED_2MHZ, GPIO_PIN_6);	  //PC6����Ϊ����ģʽ
	
	//7 ��ѹ5v���ʹ�ܣ�ʱ��ʹ��
	rcu_periph_clock_enable(RCU_GPIOB);			
	gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ, GPIO_PIN_15);	
	gpio_bit_reset(GPIOB, GPIO_PIN_15);  //5v�����
	
}

//��Ƭ���ϵ�
void PowerManager_Mcu_Poweroff(void)
{
	gpio_bit_reset(GPIOC, GPIO_PIN_3);  //��Ƭ���ϵ�
}



//usb��Դ�Ƿ����ӣ�  1��ʾ���ӣ�0��ʾû������
static uint8_t get_usb_PowerStatus(void)
{
	return gpio_input_bit_get(GPIOB, GPIO_PIN_12); 
}


//���ϵͳ���е�״̬
//����ֵ����0��ʾ����У�0��ʾδ���
static uint8_t get_bat_charge_status(void)
{
	return !gpio_input_bit_get(GPIOC, GPIO_PIN_6);  //���ʱΪ�͵�ƽ������ȡ��һ��
}


//�ⲿ��Դ���ӣ����ڳ�緵��1������2����ʾ�ѳ�����û�����0
uint8_t is_power_charge(void)
{
	if(get_usb_PowerStatus() == 0)
		return 0;
	
	if(get_bat_charge_status())  //��0��ʾ�ڳ��
		return 1;
	else
		return 2;    //�ѳ���
}





//usb��Դ�Ƿ����ӣ�  1��ʾ���ӣ�0��ʾû������
uint8_t get_BT3V_PowerStatus(void)
{
	return gpio_input_bit_get(GPIOC, GPIO_PIN_7); 
}



//�����Դ3.3v��ѹ���
void output_BT3V_enable(void)
{
	MY_PRINTF("%s %d\r\n",__FUNCTION__,__LINE__);
	gpio_bit_set(GPIOB, GPIO_PIN_13);  //5v���
}

//�����Դ3.3v����ر�
void output_BT3V_disable(void)
{
	MY_PRINTF("%s %d\r\n",__FUNCTION__,__LINE__);
	gpio_bit_reset(GPIOB, GPIO_PIN_13);  //5v�����
}


//﮵�ص�5v��ѹ���
void output_5v_enable(void)
{
	MY_PRINTF("%s %d\r\n",__FUNCTION__,__LINE__);
	gpio_bit_set(GPIOB, GPIO_PIN_15);  //5v���
}

//﮵�ص�5v��ѹ���
void output_5v_disable(void)
{
	MY_PRINTF("%s %d\r\n",__FUNCTION__,__LINE__);
	gpio_bit_reset(GPIOB, GPIO_PIN_15);  //5v�����
}






