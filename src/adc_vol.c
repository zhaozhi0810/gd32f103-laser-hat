

#include "includes.h"


/*
	获取电池电压
	
	PC0
*/


static uint8_t g_Battery_voltage = 40;    //电池电压，放大了10倍，则保留一位小数


#if 0

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
#if 1
    /*------------------时钟配置------------------*/
    // GPIO时钟使能
//    rcu_periph_clock_enable(RCU_GPIOC);
    // ADC时钟使能
    rcu_periph_clock_enable(RCU_ADC0);
    // ADC时钟8分频，最大14MHz
    rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV12);

    /*------------------ADC GPIO配置------------------*/
    /* ADC SCAN function enable */
    adc_special_function_config(ADC0, ADC_SCAN_MODE,ENABLE);  
    /* ADC trigger config */
    adc_external_trigger_source_config(ADC0, ADC_INSERTED_CHANNEL, ADC0_1_2_EXTTRIG_INSERTED_NONE);
    /* ADC data alignment config */
    adc_data_alignment_config(ADC0, ADC_DATAALIGN_RIGHT);
    /* ADC mode config */
    adc_mode_config(ADC_MODE_FREE);  
    /* ADC channel length config */
    adc_channel_length_config(ADC0, ADC_INSERTED_CHANNEL, 1);

    /* ADC temperature sensor channel config */
    adc_inserted_channel_config(ADC0, 0, ADC_CHANNEL_0, ADC_SAMPLETIME_71POINT5);//ADC_SAMPLETIME_239POINT5);
    /* ADC internal reference voltage channel config */
    //adc_inserted_channel_config(ADC0, 1, ADC_CHANNEL_17, ADC_SAMPLETIME_239POINT5);

    /* ADC external trigger enable */
    adc_external_trigger_config(ADC0, ADC_INSERTED_CHANNEL,ENABLE);

    /* ADC temperature and Vrefint enable */
    adc_tempsensor_vrefint_enable();
    
    /* enable ADC interface */
    adc_enable(ADC0);
    Delay1ms(1);    
    /* ADC calibration and reset calibration */
    adc_calibration_enable(ADC0);
	
	adc_software_trigger_enable(ADC0, ADC_INSERTED_CHANNEL);  /* ADC software trigger enable */	
#else	
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
#endif	
}

/**
 @brief ADC读取
 @param channel -[in] ADC通道
 @return ADC采样值
*/
uint16_t ADC_Read(uint8_t channel)
{
#if 1
	return ADC_IDATA0(ADC0);
#else
	
    uint16_t adcValue = 0;
    
    // 配置ADC通道转换顺序，采样时间为55.5个时钟周期
    adc_regular_channel_config(ADC0, 0, channel, ADC_SAMPLETIME_55POINT5);
    // 由于没有采用外部触发，所以使用软件触发ADC转换
    adc_software_trigger_enable(ADC0, ADC_REGULAR_CHANNEL);   
    
    while(!adc_flag_get(ADC0, ADC_FLAG_EOC));                       // 等待采样完成
    adc_flag_clear(ADC0, ADC_FLAG_EOC);                             // 清除结束标志
    
    adcValue = adc_regular_data_read(ADC0);                         // 读取ADC数据
    return adcValue;
#endif
}

#else
//PC0 --- ADC12_IN10
static void pc0_adc_test_init(void)
{
	rcu_periph_clock_enable(RCU_GPIOC);
	gpio_init(GPIOC, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_0);
}



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
    // ADC时钟使能
    rcu_periph_clock_enable(RCU_ADC1);
    // ADC时钟8分频，最大14MHz
    rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV8);
        
    /*------------------ADC工作模式配置------------------*/
    // 只使用一个ADC,属于独立模式
    adc_mode_config(ADC_MODE_FREE);
    // 多通道用扫描模式
    adc_special_function_config(ADC1, ADC_SCAN_MODE, DISABLE);
    // 单通道用连续转换模式
    adc_special_function_config(ADC1, ADC_CONTINUOUS_MODE, DISABLE);
    
	adc_discontinuous_mode_config(ADC1, ADC_REGULAR_CHANNEL, 1);   //规则模式下1个通道
	
    // 结果转换右对齐
    adc_data_alignment_config(ADC1, ADC_DATAALIGN_RIGHT);
    // 转换通道1个
    adc_channel_length_config(ADC1, ADC_REGULAR_CHANNEL, 1);
    
//	adc_tempsensor_vrefint_enable();  //温度通道开启
//	adc_regular_channel_config(ADC1, 0, ADC_CHANNEL_16, ADC_SAMPLETIME_55POINT5);
	pc0_adc_test_init();
	// 配置ADC通道转换顺序，采样时间为55.5个时钟周期
	adc_regular_channel_config(ADC1, 0, ADC_CHANNEL_10, ADC_SAMPLETIME_55POINT5);
	
    // 不用外部触发转换，软件开启即可
    adc_external_trigger_source_config(ADC1, ADC_REGULAR_CHANNEL, ADC0_1_2_EXTTRIG_REGULAR_NONE);
    adc_external_trigger_config(ADC1, ADC_REGULAR_CHANNEL, ENABLE);
    
    // 使能ADC
    adc_enable(ADC1);
    Delay1ms(1);                                                   // 等待1ms
    // 使能ADC校准
    adc_calibration_enable(ADC1);

    
    // 由于没有采用外部触发，所以使用软件触发ADC转换
    adc_software_trigger_enable(ADC1, ADC_REGULAR_CHANNEL); 	
}




/**
 @brief ADC读取
 @param channel -[in] ADC通道
 @return ADC采样值

//PC0 --- ADC12_IN10
*/
uint16_t ADC_Read(uint8_t channel)
{
    uint16_t adcValue = 0;
	uint16_t  n = 0;	
    
    while(!adc_flag_get(ADC1, ADC_FLAG_EOC))//;                       // 等待采样完成
    {
		n++;
		Delay1ms(1);
		if(n>10)
		{	
			printf("ERROR: adc time out\r\n");
			break;  //return 0;
		}
	}
	adc_flag_clear(ADC1, ADC_FLAG_EOC);                             // 清除结束标志
    
    adcValue = adc_regular_data_read(ADC1);                         // 读取ADC数据

    // 配置ADC通道转换顺序，采样时间为55.5个时钟周期
    adc_regular_channel_config(ADC1, 0, channel, ADC_SAMPLETIME_55POINT5);
    // 由于没有采用外部触发，所以使用软件触发ADC转换
    adc_software_trigger_enable(ADC1, ADC_REGULAR_CHANNEL);   

	return adcValue;
}
#endif


//获取通道ch的转换值，取times次,然后平均 
//ch:通道编号
//times:获取次数
//返回值:通道ch的times次转换结果平均值
//static uint16_t getAdcAverage(uint8_t ch,uint8_t times)
//{
//	uint32_t temp_val=0;
//	uint8_t t;
//	for(t=0;t<times;t++)
//	{
//		temp_val+=ADC_Read(ch);
//		Delay1ms(5);
//		//delay_1ms(5);
//	}
//	return temp_val/times;
//} 
	 


/*
	电池电压，正常4.2v，但是进行了1/2分压，调试再看看吧
	电压值的计算 3.3*val /1024 就得到计算出来的电压

//PC0 --- ADC12_IN10
*/
uint16_t ADCgetBatVol(void)
{
//    uint32_t adcx;
    uint16_t result;
    //float temperature;
    result = (ADC_Read(ADC_CHANNEL_10)* 66.0 / 4096);//(ADC_CHANNEL_16,5);  //读取通道16,5次取平均
    //MY_PRINTF("ADCgetBatVol = %d\r\n",result);
	//temperature = (1.43 - adcx*3.3/4096) * 1000 / 4.3 + 10;     //25 --> 10
//    result = temperature;                  //扩大100倍.
    return result;

	
//	return getAdcAverage(ADC_CHANNEL_0,3)* 66 /1024;   //因为2分压了，所以乘3.3*2,扩大10倍，就不再使用小数
}


//打印电池电压
void print_Battery_voltage(void)
{
	printf("Battery_voltage = %.1f\r\n",g_Battery_voltage/10.0);
}



//电池电压检测任务,500ms进入一次
void bat_vol_task(void)
{
	uint16_t vol;
	
	if(is_power_charge())//get_system_run_status() == DEV_CHARGE)  //充电时不检测电压
	{
		if(!charging_enable_start_laser) //充电模式不允许开机
		{
			printf("power_charging, system poweroff!!!!!!\r\n");
			system_power_off();     //调试时暂时关闭 2022-09-08
			return;
		}
	}
	
	vol  = ADCgetBatVol();   //获得电压值
	if(is_power_charge())
		g_Battery_voltage = vol;
	else if(vol < g_Battery_voltage)  //没有充电的情况下，去获取的最小值
		g_Battery_voltage = vol;
		
//	MY_PRINTF("%s %d vol = %d\r\n",__FUNCTION__,__LINE__,vol);
	if(g_Battery_voltage > 34)   //电压放大了10倍  3.6伏
	{
	//	MY_PRINTF("%s %d vol>36\r\n",__FUNCTION__,__LINE__);
		//nothing to do
		set_system_run_status(DEV_RUN_NORMAL);
	}
	else if(g_Battery_voltage > 30)  //3.0-3.6 需要报警
	{
	//	MY_PRINTF("%s %d 3.0< vol <=3.6\r\n",__FUNCTION__,__LINE__);
		set_system_run_status(DEV_VOL_LE36);
	}
	else //3.0v以下了
	{
	//	MY_PRINTF("%s %d vol <=3.0\r\n",__FUNCTION__,__LINE__);
		system_power_off();
	//	set_system_run_status(DEV_VOL_LE30);
	}
	
}
