//software avant
//última atualização: 13/04/17

#include "AnalogReadPA.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_gpio.h"
#include "motores_pwm.h"
#include "MPU6050.h"


#define min_duty_cycle 20
#define max_duty_cycle 50
#define contagem 50
//ref = 25; salva como vetor: d, pwm, angulo
//depois de 5 segundos preencher vetor


FlagStatus  inicio= RESET, gatilho = RESET, gatilhoCL = RESET;
float x, y, angulo = 0, pwm1, pwm2, tensao , p2 = 280, erro, erro_ant, d=0, pwmalt = 240, ref = 0;
int canal1=1,canal2=2, duty_cycle1=0,duty_cycle2=0,tempoms=0,tempos=0;
bool mpu_status;
uint8_t  status, flaglpf;


//vetores para plotar grafico de d e do angulo
float ang[500], v_d[500];
int i = 0;


int main(void)
{
	SystemInit();
	//inicia conversor ADC1
	initAnalogPA();	//inicia ADC1
	//inicia canais de PWM
	motores_pwminit(canal1); //inicia pwm no canal 1
	motores_pwminit(canal2); //inicia pwm no canal 2
	//Iicia comunicação com módulo IMU via I2C
	MPU6050_I2C_Init();
	MPU6050_Initialize();
	status = MPU6050_GetDeviceID();
	mpu_status = MPU6050_TestConnection();
	set_LPF(4); // filtro passa baixa IMU
	//timer
	SysTick_Config(SystemCoreClock/contagem ); // interrupcoes com T = 1/contagem t ms


	while(tempos<=3){			//esperar 3 seguntos
			motores_pwm(0,canal1);	//inicialização de motores
			motores_pwm(0,canal2);
			step_open_loop();
		}
	//inicio= SET;
	pwm1 = 0;
	pwm2 = 0;
    while(1)
    {
    	get_xy_f(&x,&y,0.84,0.2);
    	motores_pwm(pwm1, canal1);
    	motores_pwm(pwm2, canal2);

    }
}
void controle_xy(){ // função do controle
    float p_gain, i_gain;
	// trecho longo! IMU/////////////////

	//angulo = x;
    /////////////////////////////////
    //amostrar_pot(); //recebe amostra do ângulo do sensor
    //p_gain = analogReadPA(3);
    i_gain = analogReadPA(4);
    p_gain =  p_gain/100000;
    i_gain =  i_gain/100000;
	erro_ant = erro;
	erro = ref - angulo;
	// controle proporcional integral: próximo d = d_anterior + erro pela proporcional p1 - o acúmulo de erro anterior
	d = d +(erro* p_gain)+ i_gain*erro_ant ;
	// saturação dos atuadores
	if(d < -9) d = -9;
	if(d > 9) d = 9;
	pwm2 = pwmalt + d;
	pwm1 = pwmalt - d;

}
void alt_control(){
		float ganho_alt = 0, p=0.05, P=0.95;
		ganho_alt = analogReadPA(2);

		pwmalt = ganho_alt/5;
		controle_xy();
		step_close_loop();
}

void amostrar_pot(){
		tensao = analogReadPA(1);
		angulo = (tensao - 1727)/14.92; // tensão para angulo

		}

void step_open_loop(){  //*
	float d_step = 8.3; // 1% * 8.3 ms de pulso no pwm
	if(gatilho == SET){
		pwm2 = pwmalt + d_step;
		if(i<250){
			amostrar_pot();
			ang[i] = angulo;
			i++;
			}
		else gatilho = RESET;
		}
	else{
		pwm2 = pwmalt;
		i = 0;
	}
	pwm1 = pwmalt  ;
}

void step_close_loop(){
	int d_inicio = 0, d_final = 10;
	if(gatilhoCL == SET){
		ref = d_final;
			if(i<250){
				amostrar_pot();
				ang[i] = angulo;
				v_d[i] = d;
				i++;
				}
			else gatilhoCL = RESET;
			}
		else i = 0;
}

void segundoCont(){
	if(tempoms<contagem){
		tempoms++;
	}
	else{
		tempoms = 0;
		if(tempos<50) tempos++;
		else tempos = 0;
	}
}

void com_radio(){

}
void SysTick_Handler(void){ // a cada 2ms:
	if(inicio == SET){
		alt_control();
	}
	else segundoCont();
	//step_open_loop();
}



