

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
	//ʱ��ʹ��
	rcu_periph_clock_enable(RCU_GPIOB);	
	rcu_periph_clock_enable(RCU_GPIOC);	
	
	gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ, GPIO_PIN_13);	 //���蹩������
	gpio_bit_reset(GPIOB, GPIO_PIN_13);  //�����޵�
	
	gpio_bit_set(GPIOC, GPIO_PIN_3);  //��Ƭ����Դ����ʹ�ܣ���Ƭ�����е�
	gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ, GPIO_PIN_3);	 //��Ƭ����Դ��������
	
	//3. ����Ϊ����ģʽ	
	gpio_init(GPIOB, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_2MHZ, GPIO_PIN_12);  //�ⲿ�Ƿ������ⲿ��Դ
	//4. ����Ϊ����ģʽ	
	gpio_init(GPIOC, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_2MHZ, GPIO_PIN_7);
}

//��Ƭ���ϵ�
void PowerManager_Mcu_Poweroff(void)
{
	gpio_bit_reset(GPIOC, GPIO_PIN_3);  //��Ƭ���ϵ�
}



//usb��Դ�Ƿ����ӣ�  1��ʾ���ӣ�0��ʾû������
uint8_t get_usb_PowerStatus(void)
{
	return gpio_input_bit_get(GPIOB, GPIO_PIN_12); 
}

//usb��Դ�Ƿ����ӣ�  1��ʾ���ӣ�0��ʾû������
uint8_t get_BT3V_PowerStatus(void)
{
	return gpio_input_bit_get(GPIOC, GPIO_PIN_7); 
}



