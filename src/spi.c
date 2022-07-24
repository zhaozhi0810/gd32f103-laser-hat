#include "gd32f10x.h"
#include "spi.h" 

/*
	硬件SPI 初始化
	
	软件片选
	
	spi的3个硬件接口是GPIOB 3，4，5号引脚
		
*/




static uint32_t spi_index[] = {SPI0,SPI1,SPI2};
static uint32_t rcu_spi_index[] = {RCU_SPI0,RCU_SPI1,RCU_SPI2};

static uint8_t spi_inited = 0 ; //bit0,bit1,bit2有效，为1表示初始化了，为0表示未初始化

void usr_spi_init(spi_index_t index,uint8_t isremap)
{
	static uint8_t ready = 0;	   //非0表示已经初始化完成
	spi_parameter_struct spi_struct;
	
	if(spi_inited & (1<<index))   //已经初始化了，就不用再来初始化了。
		return;
		
	/*0. 使能uart时钟 enable SPI clock */
	rcu_periph_clock_enable(rcu_spi_index[index]);
	
	if(index == SPI1_INDEX)  
	{		
		if(!isremap)   //没有映射的情况
		{
			/*1. 使能GPIO时钟 enable GPIO clock */
			rcu_periph_clock_enable(RCU_GPIOA);
		
			/*3. 引脚初始化为复用功能模式 connect port to SPI0_DO SCLK */
			gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_MODE_AF_OD,  GPIO_PIN_5 |GPIO_PIN_7);

			/*4. 引脚初始化为复用功能模式 connect port to SPI0_DI */
			gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_2MHZ, GPIO_PIN_6);
		}
		else
		{
			/*1. 使能GPIO时钟 enable GPIO clock */
			rcu_periph_clock_enable(RCU_GPIOB);
		
			/*3. 引脚初始化为复用功能模式 connect port to SPI0_DO SCLK */
			gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_MODE_AF_OD,  GPIO_PIN_5 |GPIO_PIN_3);

			/*4. 引脚初始化为复用功能模式 connect port to SPI0_DI */
			gpio_init(GPIOB, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_2MHZ, GPIO_PIN_4);
		}
		
		spi_inited |= 1<<index;
	}

	/* initialize the parameters of SPI struct with the default values */
	spi_struct_para_init(&spi_struct);
	spi_struct.device_mode = SPI_MASTER;   //单片机端是主机模式
	spi_struct.nss = SPI_NSS_SOFT;    //软件片选
//	spi_struct.prescale = SPI_PSC_2;  //时钟太快，可以选择调慢一点
	/* initialize SPI parameter */
	spi_init(spi_index[index], &spi_struct);
	
	i2s_enable(spi_index[index]);

}


uint8_t usr_spi_write_read(spi_index_t index,uint8_t dat)
{
	//是不是数据正在发送，如果有就等待
	while(spi_i2s_flag_get(spi_index[index], SPI_FLAG_TBE) == RESET);	
	spi_i2s_data_transmit(spi_index[index], dat);
	
	//是不是接收完了？没有就等待
	while(spi_i2s_flag_get(spi_index[index], SPI_FLAG_RBNE) == RESET);
	return spi_i2s_data_receive(spi_index[index]);
}
