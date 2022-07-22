

#include "includes.h"


/*
	1.在电池供电的情况下，根据原理图，按键可以使单片机通电，
	2.外设可以由单片机控制是否供电 BT3V   PB13控制   PC7可以获取供电情况
	3.可以知道外部是否连接外部电源 PB12  vcc 1/2分压
	4.PC3单片机由按键触发上电后，需要保持为高，低则单片机断电
	5.连接外部usb电源的时候，单片机一直通电
*/



void PowerManager_init(void)
{
	//时钟使能
	rcu_periph_clock_enable(RCU_GPIOB);	
	rcu_periph_clock_enable(RCU_GPIOC);	
	
	gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ, GPIO_PIN_13);	 //外设供电引脚
	gpio_bit_reset(GPIOB, GPIO_PIN_13);  //外设无电
	
	gpio_bit_set(GPIOC, GPIO_PIN_3);  //单片机电源引脚使能，单片机才有电
	gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ, GPIO_PIN_3);	 //单片机电源供电引脚
	
	//3. 设置为输入模式	
	gpio_init(GPIOB, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_2MHZ, GPIO_PIN_12);  //外部是否连接外部电源
	//4. 设置为输入模式	
	gpio_init(GPIOC, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_2MHZ, GPIO_PIN_7);
}

//单片机断电
void PowerManager_Mcu_Poweroff(void)
{
	gpio_bit_reset(GPIOC, GPIO_PIN_3);  //单片机断电
}



//usb电源是否连接？  1表示连接，0表示没有连接
uint8_t get_usb_PowerStatus(void)
{
	return gpio_input_bit_get(GPIOB, GPIO_PIN_12); 
}

//usb电源是否连接？  1表示连接，0表示没有连接
uint8_t get_BT3V_PowerStatus(void)
{
	return gpio_input_bit_get(GPIOC, GPIO_PIN_7); 
}



