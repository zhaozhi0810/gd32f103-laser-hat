

#include "includes.h"


/*
	��ȡ��ص�ѹ
	
	PC0
*/


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
}

/**
 @brief ADC��ȡ
 @param channel -[in] ADCͨ��
 @return ADC����ֵ
*/
uint16_t ADC_Read(uint8_t channel)
{
    uint16_t adcValue = 0;
    
    // ����ADCͨ��ת��˳�򣬲���ʱ��Ϊ55.5��ʱ������
    adc_regular_channel_config(ADC1, 0, channel, ADC_SAMPLETIME_55POINT5);
    // ����û�в����ⲿ����������ʹ���������ADCת��
    adc_software_trigger_enable(ADC1, ADC_REGULAR_CHANNEL);   
    
    while(!adc_flag_get(ADC1, ADC_FLAG_EOC));                       // �ȴ��������
    adc_flag_clear(ADC1, ADC_FLAG_EOC);                             // ���������־
    
    adcValue = adc_regular_data_read(ADC1);                         // ��ȡADC����
    return adcValue;
}



//��ȡͨ��ch��ת��ֵ��ȡtimes��,Ȼ��ƽ�� 
//ch:ͨ�����
//times:��ȡ����
//����ֵ:ͨ��ch��times��ת�����ƽ��ֵ
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
	��ص�ѹ������4.2v�����ǽ�����1/2��ѹ�������ٿ�����
	��ѹֵ�ļ��� 3.3*val /1024 �͵õ���������ĵ�ѹ
*/
uint16_t ADCgetBatVol(void)
{	
	return getAdcAverage(ADC_CHANNEL_0,3)* 66 /1024;   //��Ϊ2��ѹ�ˣ����Գ�3.3*2,����10�����Ͳ���ʹ��С��
}



