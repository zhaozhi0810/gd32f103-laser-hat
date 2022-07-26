

#include "includes.h"


/*
	1.在电池供电的情况下，根据原理图，按键可以使单片机通电，
	2.外设可以由单片机控制是否供电 BT3V   PB13控制   PC7可以获取供电情况
	3.可以知道外部是否连接外部电源 PB12  vcc 1/2分压
	4.PC3单片机由按键触发上电后，需要保持为高，低则单片机断电
	5.连接外部usb电源的时候，单片机一直通电
	6.PB14 控制升压后5v的输出，低电平有效（P沟道：Ug<Us时导通）。
	7.PC4 MAX9700DEUB+T shutdown 低电平有效 

mos管区分
1）G极是比较好区分的，大家一眼就能区分。
不论是P沟道还是N沟道，两根线相交的就是S极。
不论是P沟道还是N沟道，单独引线的那边就是D极。
2）N、P沟道如何区分？
箭头指向G极的就是N沟道。
箭头背向G极的就是P沟道。
3）寄生二极管方向向
N沟道，由S极指向D极。
P沟道，由D极指向S极。
MOS管导通条件
N沟道：Ug>Us时导通。（简单认为）Ug=Us时截?。
P沟道：Ug<Us时导通。（简单认为）Ug=Us时截?
*/



void PowerManager_init(void)
{
	//1.时钟使能
	rcu_periph_clock_enable(RCU_GPIOB);	
	rcu_periph_clock_enable(RCU_GPIOC);	
	
	//2.外设3.3v控制引脚
	gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ, GPIO_PIN_13 | GPIO_PIN_14);	 //外设供电引脚
	gpio_bit_reset(GPIOB, GPIO_PIN_13);  //外设无电
	gpio_bit_set(GPIOB, GPIO_PIN_14);  //5v不输出
	
	//3.单片机电源控制引脚，高有效
	gpio_bit_set(GPIOC, GPIO_PIN_3);  //单片机电源引脚使能，单片机才有电
	gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ, GPIO_PIN_3);	 //单片机电源供电引脚
	
	//4. 设置为输入模式	
	gpio_init(GPIOB, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_2MHZ, GPIO_PIN_12);  //外部是否连接外部电源
	//5. 设置为输入模式	
	gpio_init(GPIOC, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_2MHZ, GPIO_PIN_7);   //3.3伏是否输出
	
	
	//6. 充电状态的引脚初始化
	//6.1 充电状态读取
//	rcu_periph_clock_enable(RCU_GPIOC);			
	gpio_init(GPIOC, GPIO_MODE_IPU, GPIO_OSPEED_2MHZ, GPIO_PIN_6);	  //PC6设置为输入模式
	
	//7 升压5v输出使能，时钟使能
//	rcu_periph_clock_enable(RCU_GPIOB);			
	gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ, GPIO_PIN_15);	
	gpio_bit_reset(GPIOB, GPIO_PIN_15);  //5v不输出
	
	//8. MAX9700DEUB+T shutdown 低电平有效 PC4
	gpio_bit_reset(GPIOC, GPIO_PIN_4);  //音频芯片待机模式
	gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ, GPIO_PIN_4);	 //音频芯片待机模式
}

//单片机断电
void PowerManager_Mcu_Poweroff(void)
{
	gpio_bit_reset(GPIOC, GPIO_PIN_3);  //单片机断电
}



//usb电源是否连接？  1表示连接，0表示没有连接
static uint8_t get_usb_PowerStatus(void)
{
	return gpio_input_bit_get(GPIOB, GPIO_PIN_12); 
}


//获得系统运行的状态
//返回值大于0表示充电中，0表示未充电
static uint8_t get_bat_charge_status(void)
{
	return !gpio_input_bit_get(GPIOC, GPIO_PIN_6);  //充电时为低电平，所以取反一下
}


//外部电源连接，正在充电返回1，返回2，表示已充满，没充电则0
uint8_t is_power_charge(void)
{
	if(get_usb_PowerStatus() == 0)
		return 0;
	
	if(get_bat_charge_status())  //非0表示在充电
		return 1;
	else
		return 2;    //已充满
}





//BT3V电源是否连接？  1表示连接，0表示没有连接
uint8_t get_BT3V_PowerStatus(void)
{
	return gpio_input_bit_get(GPIOC, GPIO_PIN_7); 
}



//外设电源3.3v升压输出
void output_BT3V_enable(void)
{
//	MY_PRINTF("%s %d\r\n",__FUNCTION__,__LINE__);
	gpio_bit_set(GPIOB, GPIO_PIN_13);  //5v输出
}

//外设电源3.3v输出关闭
void output_BT3V_disable(void)
{
//	MY_PRINTF("%s %d\r\n",__FUNCTION__,__LINE__);
	gpio_bit_reset(GPIOB, GPIO_PIN_13);  //5v不输出
}


//锂电池的5v升压输出
void output_5v_enable(void)
{
//	MY_PRINTF("%s %d\r\n",__FUNCTION__,__LINE__);
	gpio_bit_set(GPIOB, GPIO_PIN_15);  //5v输出
	gpio_bit_set(GPIOB, GPIO_PIN_14);  //5v mos管 输出
}

//锂电池的5v升压输出
void output_5v_disable(void)
{
//	MY_PRINTF("%s %d\r\n",__FUNCTION__,__LINE__);
	gpio_bit_reset(GPIOB, GPIO_PIN_15 );  //5v不输出
	gpio_bit_reset(GPIOB, GPIO_PIN_14);  //5v mos管 不输出
}


//音频芯片省电控制
void set_MAX9700_enable(void)
{
	gpio_bit_set(GPIOC, GPIO_PIN_4);  //音频芯片电源开启模式
}

//音频芯片省电控制
void set_MAX9700_shutdown(void)
{
	gpio_bit_reset(GPIOC, GPIO_PIN_4);  //音频芯片待机模式
}
