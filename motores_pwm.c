//PWM Motores AVANT UFSM
//Társis Natan Boff da Silva
//29-03-2016

#include "stm32f10x_tim.h"
#include "stm32f10x_gpio.h"
#include "motores_pwm.h"

void motores_pwminit(int canal){

	GPIO_InitTypeDef config_gpio;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	// ativa timer clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	// configuracao timer
	// frequencia PWM = 100 hz with 24,000,000 hz system clock
	// 24,000,000/240 = 100,000
	// 100,000/4420 = 226Hz
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = SystemCoreClock/1000000 - 1; // 0..239
	TIM_TimeBaseStructure.TIM_Period = 4420 - 1; // 0..4420 ms
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);


	// Edge-aligned; not single pulse mode
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;

	// ativa clock na porta IO
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	// CONFIGURACAO DOS PINOS IO
	GPIO_StructInit(&config_gpio);
	config_gpio.GPIO_Mode=GPIO_Mode_AF_PP;
	config_gpio.GPIO_Speed=GPIO_Speed_50MHz;

	if(canal == 1){
		TIM_OC1Init(TIM4, &TIM_OCInitStructure);
		config_gpio.GPIO_Pin= GPIO_Pin_6;
	}
	if(canal == 2){
		TIM_OC2Init(TIM4, &TIM_OCInitStructure);
		config_gpio.GPIO_Pin= GPIO_Pin_7;
		}
	if(canal == 3){
		TIM_OC3Init(TIM4, &TIM_OCInitStructure);
		config_gpio.GPIO_Pin= GPIO_Pin_8;
		}
	if(canal == 4){
		TIM_OC4Init(TIM4, &TIM_OCInitStructure);
		config_gpio.GPIO_Pin= GPIO_Pin_9;
		}
	//flag de erro = acender led PC13
	if(canal !=1&&canal !=2&&canal !=3&&canal !=4){
		error_flag();
	}
	// Enable Timer
	TIM_Cmd(TIM4, ENABLE);
	// Enable GPIO
	GPIO_Init(GPIOB, &config_gpio);
}
void motores_pwm(int duty_cycle, int canal){
	int t_pwm = min_t; // ms

	//if(duty_cycle> 100) duty_cycle = 100;
	if(duty_cycle> 830) duty_cycle = 830;
	if(duty_cycle<0)duty_cycle = 0;
	t_pwm = duty_cycle + min_t;
	//t_pwm = ((duty_cycle*(max_t-min_t))/100)+min_t; //mapear escala de % no periodo pwm
	if(canal == 1){
		TIM_SetCompare1(TIM4, t_pwm);
	}
	if(canal == 2){
		TIM_SetCompare2(TIM4, t_pwm);
		}
	if(canal == 3){
		TIM_SetCompare3(TIM4, t_pwm);
		}
	if(canal == 4){
		TIM_SetCompare4(TIM4, t_pwm);
		}
	if(canal !=1&&canal !=2&&canal !=3&&canal !=4){
		error_flag();
	}

}
//rotina flag de erro = acender led PC13
void error_flag(){
	uint8_t flag_tim_config[4];

	GPIO_InitTypeDef config_gpio;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_StructInit(&config_gpio);
	config_gpio.GPIO_Mode=GPIO_Mode_Out_PP;
	config_gpio.GPIO_Pin= GPIO_Pin_13;
	config_gpio.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &config_gpio);
	GPIO_WriteBit(GPIOC,GPIO_Pin_13, Bit_RESET);
}
