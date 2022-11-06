

/*
	本头文件提供模拟iic对外接口及数据结构
*/

#ifndef __IIC_SIM_INCLUDE_H__
#define __IIC_SIM_INCLUDE_H__


typedef struct dz_sim_iic_iostruct
{
	rcu_periph_enum periph_sda;    //sda_io时钟
	rcu_periph_enum periph_scl;    //scl_io时钟
	
	uint32_t gpio_periph_sda;     //sda_GPIO组 
	uint32_t gpio_periph_scl;     //scl_GPIO组
	
	uint32_t pin_sda;   //sda_pin脚
	uint32_t pin_scl;	//scl_pin脚
}dz_sim_iic_iostruct_t;
	

//需要提供延时（1us）函数的实现！！！！库中没有实现该函数
extern void Delay1us(uint32_t nus);




#endif
