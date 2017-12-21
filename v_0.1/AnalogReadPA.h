#include "stm32f10x_conf.h"

GPIO_InitTypeDef GPIO_InitStructure;
ADC_InitTypeDef ADC_InitStructure;

void initAnalogPA();
u32 analogReadPA(u8 canal);
