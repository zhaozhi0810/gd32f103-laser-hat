

#include "includes.h"


/*
	��ȡ��ص�ѹ
	
	PC0
*/


static uint8_t g_Battery_voltage = 40;    //��ص�ѹ���Ŵ���10��������һλС��


#if 0

/*********************************************************************
 * PUBLIC FUNCTIONS
 */
/**
 @brief ADC������ʼ��
 @param ��
 @return ��
*/
void ADC_Init(void)
{
#if 1
    /*------------------ʱ������------------------*/
    // GPIOʱ��ʹ��
//    rcu_periph_clock_enable(RCU_GPIOC);
    // ADCʱ��ʹ��
    rcu_periph_clock_enable(RCU_ADC0);
    // ADCʱ��8��Ƶ�����14MHz
    rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV12);

    /*------------------ADC GPIO����------------------*/
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
	/*------------------ʱ������------------------*/
    // GPIOʱ��ʹ��
    rcu_periph_clock_enable(RCU_GPIOC);
    // ADCʱ��ʹ��
    rcu_periph_clock_enable(RCU_ADC1);
    // ADCʱ��8��Ƶ�����14MHz
    rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV8);
    
    /*------------------ADC GPIO����------------------*/
    // ����Ϊģ������
    gpio_init(GPIOC, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_0);
//    gpio_init(GPIOC, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_4);
    
    /*------------------ADC����ģʽ����------------------*/
    // ֻʹ��һ��ADC,���ڶ���ģʽ
    adc_mode_config(ADC_MODE_FREE);
    // ��ͨ����ɨ��ģʽ
    adc_special_function_config(ADC1, ADC_SCAN_MODE, ENABLE);
    // ��ͨ��������ת��ģʽ
//    adc_special_function_config(ADC1, ADC_CONTINUOUS_MODE, ENABLE);
    
    // ���ת���Ҷ���
    adc_data_alignment_config(ADC1, ADC_DATAALIGN_RIGHT);
    // ת��ͨ��1��
    adc_channel_length_config(ADC1, ADC_REGULAR_CHANNEL, 1);
    
    // �����ⲿ����ת���������������
    adc_external_trigger_source_config(ADC1, ADC_REGULAR_CHANNEL, ADC0_1_2_EXTTRIG_REGULAR_NONE);
    adc_external_trigger_config(ADC1, ADC_REGULAR_CHANNEL, ENABLE);
    
    // ʹ��ADC
    adc_enable(ADC1);
    Delay1ms(1);                                                   // �ȴ�1ms
    // ʹ��ADCУ׼
    adc_calibration_enable(ADC1);
#endif	
}

/**
 @brief ADC��ȡ
 @param channel -[in] ADCͨ��
 @return ADC����ֵ
*/
uint16_t ADC_Read(uint8_t channel)
{
#if 1
	return ADC_IDATA0(ADC0);
#else
	
    uint16_t adcValue = 0;
    
    // ����ADCͨ��ת��˳�򣬲���ʱ��Ϊ55.5��ʱ������
    adc_regular_channel_config(ADC0, 0, channel, ADC_SAMPLETIME_55POINT5);
    // ����û�в����ⲿ����������ʹ���������ADCת��
    adc_software_trigger_enable(ADC0, ADC_REGULAR_CHANNEL);   
    
    while(!adc_flag_get(ADC0, ADC_FLAG_EOC));                       // �ȴ��������
    adc_flag_clear(ADC0, ADC_FLAG_EOC);                             // ���������־
    
    adcValue = adc_regular_data_read(ADC0);                         // ��ȡADC����
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
 @brief ADC������ʼ��
 @param ��
 @return ��
*/
void ADC_Init(void)
{
    // ADCʱ��ʹ��
    rcu_periph_clock_enable(RCU_ADC1);
    // ADCʱ��8��Ƶ�����14MHz
    rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV8);
        
    /*------------------ADC����ģʽ����------------------*/
    // ֻʹ��һ��ADC,���ڶ���ģʽ
    adc_mode_config(ADC_MODE_FREE);
    // ��ͨ����ɨ��ģʽ
    adc_special_function_config(ADC1, ADC_SCAN_MODE, DISABLE);
    // ��ͨ��������ת��ģʽ
    adc_special_function_config(ADC1, ADC_CONTINUOUS_MODE, DISABLE);
    
	adc_discontinuous_mode_config(ADC1, ADC_REGULAR_CHANNEL, 1);   //����ģʽ��1��ͨ��
	
    // ���ת���Ҷ���
    adc_data_alignment_config(ADC1, ADC_DATAALIGN_RIGHT);
    // ת��ͨ��1��
    adc_channel_length_config(ADC1, ADC_REGULAR_CHANNEL, 1);
    
//	adc_tempsensor_vrefint_enable();  //�¶�ͨ������
//	adc_regular_channel_config(ADC1, 0, ADC_CHANNEL_16, ADC_SAMPLETIME_55POINT5);
	pc0_adc_test_init();
	// ����ADCͨ��ת��˳�򣬲���ʱ��Ϊ55.5��ʱ������
	adc_regular_channel_config(ADC1, 0, ADC_CHANNEL_10, ADC_SAMPLETIME_55POINT5);
	
    // �����ⲿ����ת���������������
    adc_external_trigger_source_config(ADC1, ADC_REGULAR_CHANNEL, ADC0_1_2_EXTTRIG_REGULAR_NONE);
    adc_external_trigger_config(ADC1, ADC_REGULAR_CHANNEL, ENABLE);
    
    // ʹ��ADC
    adc_enable(ADC1);
    Delay1ms(1);                                                   // �ȴ�1ms
    // ʹ��ADCУ׼
    adc_calibration_enable(ADC1);

    
    // ����û�в����ⲿ����������ʹ���������ADCת��
    adc_software_trigger_enable(ADC1, ADC_REGULAR_CHANNEL); 	
}




/**
 @brief ADC��ȡ
 @param channel -[in] ADCͨ��
 @return ADC����ֵ

//PC0 --- ADC12_IN10
*/
uint16_t ADC_Read(uint8_t channel)
{
    uint16_t adcValue = 0;
	uint16_t  n = 0;	
    
    while(!adc_flag_get(ADC1, ADC_FLAG_EOC))//;                       // �ȴ��������
    {
		n++;
		Delay1ms(1);
		if(n>10)
		{	
			printf("ERROR: adc time out\r\n");
			break;  //return 0;
		}
	}
	adc_flag_clear(ADC1, ADC_FLAG_EOC);                             // ���������־
    
    adcValue = adc_regular_data_read(ADC1);                         // ��ȡADC����

    // ����ADCͨ��ת��˳�򣬲���ʱ��Ϊ55.5��ʱ������
    adc_regular_channel_config(ADC1, 0, channel, ADC_SAMPLETIME_55POINT5);
    // ����û�в����ⲿ����������ʹ���������ADCת��
    adc_software_trigger_enable(ADC1, ADC_REGULAR_CHANNEL);   

	return adcValue;
}
#endif


//��ȡͨ��ch��ת��ֵ��ȡtimes��,Ȼ��ƽ�� 
//ch:ͨ�����
//times:��ȡ����
//����ֵ:ͨ��ch��times��ת�����ƽ��ֵ
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
	��ص�ѹ������4.2v�����ǽ�����1/2��ѹ�������ٿ�����
	��ѹֵ�ļ��� 3.3*val /1024 �͵õ���������ĵ�ѹ

//PC0 --- ADC12_IN10
*/
uint16_t ADCgetBatVol(void)
{
//    uint32_t adcx;
    uint16_t result;
    //float temperature;
    result = (ADC_Read(ADC_CHANNEL_10)* 66.0 / 4096);//(ADC_CHANNEL_16,5);  //��ȡͨ��16,5��ȡƽ��
    //MY_PRINTF("ADCgetBatVol = %d\r\n",result);
	//temperature = (1.43 - adcx*3.3/4096) * 1000 / 4.3 + 10;     //25 --> 10
//    result = temperature;                  //����100��.
    return result;

	
//	return getAdcAverage(ADC_CHANNEL_0,3)* 66 /1024;   //��Ϊ2��ѹ�ˣ����Գ�3.3*2,����10�����Ͳ���ʹ��С��
}


//��ӡ��ص�ѹ
void print_Battery_voltage(void)
{
	printf("Battery_voltage = %.1f\r\n",g_Battery_voltage/10.0);
}



//��ص�ѹ�������,500ms����һ��
void bat_vol_task(void)
{
	uint16_t vol;
	
	if(is_power_charge())//get_system_run_status() == DEV_CHARGE)  //���ʱ������ѹ
	{
		if(!charging_enable_start_laser) //���ģʽ��������
		{
			printf("power_charging, system poweroff!!!!!!\r\n");
			system_power_off();     //����ʱ��ʱ�ر� 2022-09-08
			return;
		}
	}
	
	vol  = ADCgetBatVol();   //��õ�ѹֵ
	if(is_power_charge())
		g_Battery_voltage = vol;
	else if(vol < g_Battery_voltage)  //û�г�������£�ȥ��ȡ����Сֵ
		g_Battery_voltage = vol;
		
//	MY_PRINTF("%s %d vol = %d\r\n",__FUNCTION__,__LINE__,vol);
	if(g_Battery_voltage > 34)   //��ѹ�Ŵ���10��  3.6��
	{
	//	MY_PRINTF("%s %d vol>36\r\n",__FUNCTION__,__LINE__);
		//nothing to do
		set_system_run_status(DEV_RUN_NORMAL);
	}
	else if(g_Battery_voltage > 30)  //3.0-3.6 ��Ҫ����
	{
	//	MY_PRINTF("%s %d 3.0< vol <=3.6\r\n",__FUNCTION__,__LINE__);
		set_system_run_status(DEV_VOL_LE36);
	}
	else //3.0v������
	{
	//	MY_PRINTF("%s %d vol <=3.0\r\n",__FUNCTION__,__LINE__);
		system_power_off();
	//	set_system_run_status(DEV_VOL_LE30);
	}
	
}
