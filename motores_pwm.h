//PWM Motores AVANT UFSM
//Társis Natan Boff da Silva
//29-03-2016

#include "stm32f10x_tim.h"
#include "stm32f10x_gpio.h"

#define min_t 840 //ms
#define max_t 1670 //ms



void motores_pwminit(int canal);
void motores_pwm(int duty_cycle, int canal);
