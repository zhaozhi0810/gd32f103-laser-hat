#include "gd32f10x.h"
#include "spi.h" 

/*
	Ӳ��SPI ��ʼ��
	
	���Ƭѡ
	
	spi��3��Ӳ���ӿ���GPIOB 3��4��5������
		
*/




static uint32_t spi_index[] = {SPI0,SPI1,SPI2};
static uint32_t rcu_spi_index[] = {RCU_SPI0,RCU_SPI1,RCU_SPI2};

static uint8_t spi_inited = 0 ; //bit0,bit1,bit2��Ч��Ϊ1��ʾ��ʼ���ˣ�Ϊ0��ʾδ��ʼ��

void usr_spi_init(spi_index_t index,uint8_t isremap)
{
	static uint8_t ready = 0;	   //��0��ʾ�Ѿ���ʼ�����
	spi_parameter_struct spi_struct;
	
	if(spi_inited & (1<<index))   //�Ѿ���ʼ���ˣ��Ͳ���������ʼ���ˡ�
		return;
		
	/*0. ʹ��uartʱ�� enable SPI clock */
	rcu_periph_clock_enable(rcu_spi_index[index]);
	
	if(index == SPI1_INDEX)  
	{		
		if(!isremap)   //û��ӳ������
		{
			/*1. ʹ��GPIOʱ�� enable GPIO clock */
			rcu_periph_clock_enable(RCU_GPIOA);
		
			/*3. ���ų�ʼ��Ϊ���ù���ģʽ connect port to SPI0_DO SCLK */
			gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_MODE_AF_OD,  GPIO_PIN_5 |GPIO_PIN_7);

			/*4. ���ų�ʼ��Ϊ���ù���ģʽ connect port to SPI0_DI */
			gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_2MHZ, GPIO_PIN_6);
		}
		else
		{
			/*1. ʹ��GPIOʱ�� enable GPIO clock */
			rcu_periph_clock_enable(RCU_GPIOB);
		
			/*3. ���ų�ʼ��Ϊ���ù���ģʽ connect port to SPI0_DO SCLK */
			gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_MODE_AF_OD,  GPIO_PIN_5 |GPIO_PIN_3);

			/*4. ���ų�ʼ��Ϊ���ù���ģʽ connect port to SPI0_DI */
			gpio_init(GPIOB, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_2MHZ, GPIO_PIN_4);
		}
		
		spi_inited |= 1<<index;
	}

	/* initialize the parameters of SPI struct with the default values */
	spi_struct_para_init(&spi_struct);
	spi_struct.device_mode = SPI_MASTER;   //��Ƭ����������ģʽ
	spi_struct.nss = SPI_NSS_SOFT;    //���Ƭѡ
//	spi_struct.prescale = SPI_PSC_2;  //ʱ��̫�죬����ѡ�����һ��
	/* initialize SPI parameter */
	spi_init(spi_index[index], &spi_struct);
	
	i2s_enable(spi_index[index]);

}


uint8_t usr_spi_write_read(spi_index_t index,uint8_t dat)
{
	//�ǲ����������ڷ��ͣ�����о͵ȴ�
	while(spi_i2s_flag_get(spi_index[index], SPI_FLAG_TBE) == RESET);	
	spi_i2s_data_transmit(spi_index[index], dat);
	
	//�ǲ��ǽ������ˣ�û�о͵ȴ�
	while(spi_i2s_flag_get(spi_index[index], SPI_FLAG_RBNE) == RESET);
	return spi_i2s_data_receive(spi_index[index]);
}
