//AnalogRead
//Projeto AVANT
//Térsis Natan Boff da Silva
//10/05/2016
//fornece uma interface simples de leitura do ADC no stm32f1 (INCOMPLETA!)

#include "AnalogReadPA.h"

/*
 * configura pinos da porta a para leitura de dado analogico
 * pin: pino utilizado = 0:7!
 * retorna nada
 */
void initAnalogPA(){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOA, &GPIO_InitStructure);


	RCC_ADCCLKConfig(RCC_PCLK2_Div2);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC1, &ADC_InitStructure);



	ADC_Cmd(ADC1, ENABLE);

	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1));

	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));
}

/*
 * Le dado analógico na porta A
 * pin: pino utilizado = 0:7!
 * retorna variável u32
 */
u32 analogReadPA(u8 canal){
	ADC_RegularChannelConfig(ADC1, canal, 1, ADC_SampleTime_13Cycles5);
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	while(ADC_GetSoftwareStartConvStatus(ADC1));
	return   ADC_GetConversionValue(ADC1);

}


