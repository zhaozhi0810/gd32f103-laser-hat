

#include "includes.h"


/*
	获取电池电压
	
	PC0
*/


/*********************************************************************
 * PUBLIC FUNCTIONS
 */
/**
 @brief ADC驱动初始化
 @param 无
 @return 无
*/
void ADC_Init(void)
{
    /*------------------时钟配置------------------*/
    // GPIO时钟使能
    rcu_periph_clock_enable(RCU_GPIOC);
    // ADC时钟使能
    rcu_periph_clock_enable(RCU_ADC1);
    // ADC时钟8分频，最大14MHz
    rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV8);
    
    /*------------------ADC GPIO配置------------------*/
    // 必须为模拟输入
    gpio_init(GPIOC, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_0);
//    gpio_init(GPIOC, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_4);
    
    /*------------------ADC工作模式配置------------------*/
    // 只使用一个ADC,属于独立模式
    adc_mode_config(ADC_MODE_FREE);
    // 多通道用扫描模式
    adc_special_function_config(ADC1, ADC_SCAN_MODE, ENABLE);
    // 单通道用连续转换模式
//    adc_special_function_config(ADC1, ADC_CONTINUOUS_MODE, ENABLE);
    
    // 结果转换右对齐
    adc_data_alignment_config(ADC1, ADC_DATAALIGN_RIGHT);
    // 转换通道1个
    adc_channel_length_config(ADC1, ADC_REGULAR_CHANNEL, 1);
    
    // 不用外部触发转换，软件开启即可
    adc_external_trigger_source_config(ADC1, ADC_REGULAR_CHANNEL, ADC0_1_2_EXTTRIG_REGULAR_NONE);
    adc_external_trigger_config(ADC1, ADC_REGULAR_CHANNEL, ENABLE);
    
    // 使能ADC
    adc_enable(ADC1);
    Delay1ms(1);                                                   // 等待1ms
    // 使能ADC校准
    adc_calibration_enable(ADC1);
}

/**
 @brief ADC读取
 @param channel -[in] ADC通道
 @return ADC采样值
*/
uint16_t ADC_Read(uint8_t channel)
{
    uint16_t adcValue = 0;
    
    // 配置ADC通道转换顺序，采样时间为55.5个时钟周期
    adc_regular_channel_config(ADC1, 0, channel, ADC_SAMPLETIME_55POINT5);
    // 由于没有采用外部触发，所以使用软件触发ADC转换
    adc_software_trigger_enable(ADC1, ADC_REGULAR_CHANNEL);   
    
    while(!adc_flag_get(ADC1, ADC_FLAG_EOC));                       // 等待采样完成
    adc_flag_clear(ADC1, ADC_FLAG_EOC);                             // 清除结束标志
    
    adcValue = adc_regular_data_read(ADC1);                         // 读取ADC数据
    return adcValue;
}



//获取通道ch的转换值，取times次,然后平均 
//ch:通道编号
//times:获取次数
//返回值:通道ch的times次转换结果平均值
static uint16_t getAdcAverage(uint8_t ch,uint8_t times)
{
	uint32_t temp_val=0;
	uint8_t t;
	for(t=0;t<times;t++)
	{
		temp_val+=ADC_Read(ch);
		Delay1ms(5);
		//delay_1ms(5);
	}
	return temp_val/times;
} 
	 


/*
	电池电压，正常4.2v，但是进行了1/2分压，调试再看看吧
	电压值的计算 3.3*val /1024 就得到计算出来的电压
*/
uint16_t ADCgetBatVol(void)
{	
	return getAdcAverage(ADC_CHANNEL_0,3)* 66 /1024;   //因为2分压了，所以乘3.3*2,扩大10倍，就不再使用小数
}



