

/*
	��ͷ�ļ��ṩģ��iic����ӿڼ����ݽṹ
*/

#ifndef __IIC_SIM_INCLUDE_H__
#define __IIC_SIM_INCLUDE_H__


typedef struct dz_sim_iic_iostruct
{
	rcu_periph_enum periph_sda;    //sda_ioʱ��
	rcu_periph_enum periph_scl;    //scl_ioʱ��
	
	uint32_t gpio_periph_sda;     //sda_GPIO�� 
	uint32_t gpio_periph_scl;     //scl_GPIO��
	
	uint32_t pin_sda;   //sda_pin��
	uint32_t pin_scl;	//scl_pin��
}dz_sim_iic_iostruct_t;
	

//��Ҫ�ṩ��ʱ��1us��������ʵ�֣�����������û��ʵ�ָú���
extern void Delay1us(uint32_t nus);




#endif
