

#ifndef __ADC_VOL_H__
#define __ADC_VOL_H__

#include <gd32f10x.h>


void ADC_Init(void);
uint16_t ADC_Read(uint8_t channel);


/*
	电池电压，正常4.2v，但是进行了1/2分压，调试再看看吧
*/
uint16_t ADCgetBatVol(void);

#endif

