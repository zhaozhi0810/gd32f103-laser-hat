

#ifndef __ADC_VOL_H__
#define __ADC_VOL_H__

#include <gd32f10x.h>


void ADC_Init(void);
uint16_t ADC_Read(uint8_t channel);


/*
	��ص�ѹ������4.2v�����ǽ�����1/2��ѹ�������ٿ�����
*/
uint16_t ADCgetBatVol(void);

#endif

