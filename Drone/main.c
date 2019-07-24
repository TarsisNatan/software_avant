/*
leitura dos sensores inerciais e implementação de rotinas para controle de vôo (controle PID)
*/
#include "stm32f10x.h"
#include "string.h"
#include "MPU6050.h"
#include "math.h"
//#include "AnalogReadPA.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_gpio.h"
#include "motores_pwm.h"



#define BUF_SIZE 28
#define contagem 1000 // 1ms

#define Rad2Dree       57.295779513082320876798154814105
#define PI	3.1415926535897932384626433832795

struct structPackage{			// Size should not exceed 32Bytes
	float xAxis;
	float yAxis;
	float zAxis;
	float text[28];
	
} txPackage;


float ganho_filtro = 0.7, p_gain = 0, i_gain = 0, d_gain = 0;
float angulo = 0, pwm1 = 0, pwm2 = 0, pwm3 = 0, pwm4 = 0, tensao , p2 = 280, erro, erro_ant, d=0, pwmalt = 240, ref = 0;
int canal1=1,canal3=3, duty_cycle1=0,duty_cycle2=0,tempoms=0,tempos=0;


float mag_data[3];
uint8_t floatb[4], *f2cptr;
char caracter;
char buffer_tx[28], buffconf[100], buf[BUF_SIZE];
char *ptx = &buffer_tx[0], *ptc = &buffconf[0] ; 	
int i;
int us2_cont = 0, i2c2_cont = 0;
int t_uart = 0;
int16_t x_int, y_int, z_int;
uint8_t x_byte[2], *xb2int = (uint8_t*)&x_int, 
				y_byte[2], *yb2int = (uint8_t*)&y_int, 
				z_byte[2], *zb2int = (uint8_t*)&z_int;

int test_i; 
float test_f1, test_f2;
//FLAGS//
bool mpu_status = 0, FTX_USART2 = 0, tel = 0, FRX_I2C2 = 0, FMG_ON = 0; 

float magCalibration[3] = {0, 0, 0}, fMag[3] = {0, 0, 0};


void USART2_Init(void);
void USARTx_PutChar(char c, USART_TypeDef *USARTx);
void USARTx_PutString(char *s, USART_TypeDef *USARTx);
uint16_t USARTx_GetChar(USART_TypeDef *USARTx);
void USARTx_PutByte(uint8_t b, USART_TypeDef *USARTx);
void USARTx_PutBytes(uint8_t *bytes, USART_TypeDef *USARTx);
void LoraInit();
void enviaRF();
void fusaoSens();
void controle_xy();
void alt_control();
void controle_z();
void batStatus();


uint8_t kk ;



int main(){ 
	
	//init sis
 	SystemInit();
	LoraInit();
	// mpu i2c2
	MPU6050_I2C_Init();
 /* Initialize USART1 */
	USART2_Init();
	mpu_status = MPU6050_TestConnection();
	SysTick_Config(SystemCoreClock/contagem ); 
	if(mpu_status == SET){ 
		USARTx_PutString("imu ok\n", USART2);
		MPU6050_Initialize();
	}
	else USARTx_PutString("imu off\n", USART2);
	set_LPF(4);
  kk = get_LPF();
	
	//mag
  kk = MPU9250_OnMag();
	Initialize_AK8963(magCalibration);
	
	motores_pwminit(3);
	while(1)
	{ 

		if(FRX_I2C2 == 1){
			fusaoSens();
			alt_control();
		}
		motores_pwm(pwm1, canal3);
   // motores_pwm(pwm2, canal2);
		if(FTX_USART2 == 1 && tel == 1){ 
				enviaRF();
		}
	}
}

void LoraInit(){
	GPIO_InitTypeDef GPIO_InitStruct;
	//config = pb12 e reset = pb13 pins:
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	GPIO_ResetBits(GPIOB, GPIO_Pin_13);
	delay(500000);
	snprintf(&buffer_tx[0], sizeof(buffer_tx),"AT\r\n");
	USARTx_PutString(&buffer_tx[0], USART2);
	delay(500000);
	//buffer_tx++ = (char)USARTx_GetChar(USART2);
			GPIO_SetBits(GPIOB, GPIO_Pin_13);
}

void USART2_Init(void)
{
    /* USART configuration structure for USART1 */
    USART_InitTypeDef usart2_init_struct;
    /* Bit configuration structure for GPIOA PIN9 and PIN10 */
    GPIO_InitTypeDef gpioa_init_struct;
	
	   
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA, ENABLE);
    /* Enalbe clock for USART1, AFIO and GPIOA */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 ,ENABLE);
                            
    /* GPIOA PIN9 alternative function Tx */
    gpioa_init_struct.GPIO_Pin = GPIO_Pin_2;
    gpioa_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
    gpioa_init_struct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpioa_init_struct);
    /* GPIOA PIN9 alternative function Rx */
    gpioa_init_struct.GPIO_Pin = GPIO_Pin_3;
    gpioa_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
    gpioa_init_struct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpioa_init_struct);

    /* Enable USART1 */
    USART_Cmd(USART2, ENABLE);  
    /* Baud rate 9600, 8-bit data, One stop bit
     * No parity, Do both Rx and Tx, No HW flow control
     */
    usart2_init_struct.USART_BaudRate = 9600;   
    usart2_init_struct.USART_WordLength = USART_WordLength_8b;  
    usart2_init_struct.USART_StopBits = USART_StopBits_1;   
    usart2_init_struct.USART_Parity = USART_Parity_No ;
    usart2_init_struct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    usart2_init_struct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    /* Configure USART1 */
    USART_Init(USART2, &usart2_init_struct);
    /* Enable RXNE interrupt */
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    /* Enable USART1 global interrupt */
    NVIC_EnableIRQ(USART2_IRQn);
}

void USARTx_PutChar(char c, USART_TypeDef *USARTx)
{
	while (!USART_GetFlagStatus(USARTx, USART_FLAG_TXE));
	USART_SendData(USARTx, c);
}

void USARTx_PutString(char *s, USART_TypeDef *USARTx)
{
	while (*s)
	{
		USARTx_PutChar(*s++, USARTx);
	}
}

uint16_t USARTx_GetChar(USART_TypeDef *USARTx)
{
	while (!USART_GetFlagStatus(USARTx, USART_FLAG_RXNE));
	return USART_ReceiveData(USARTx);
}
   


void USARTx_PutByte(uint8_t b, USART_TypeDef *USARTx)
{
	while (!USART_GetFlagStatus(USARTx, USART_FLAG_TXE));
	USART_SendData(USARTx, b);
}

void USARTx_PutBytes(uint8_t *bytes, USART_TypeDef *USARTx)
{
	while (*bytes)
	{
		USARTx_PutChar(*bytes++, USARTx);
	}
}



void enviaRF(){
	  x_int = (int16_t)(txPackage.xAxis *10);
		
		y_int = (int16_t)(txPackage.yAxis *10);
		
		z_int = (int16_t)(txPackage.zAxis *10);
		
		x_byte[0] =  (uint8_t)*xb2int++;
		x_byte[1] =  (uint8_t)*xb2int;
		xb2int = (uint8_t*)&x_int;
		
		y_byte[0] =  *yb2int++;
		y_byte[1] =  *yb2int;
		yb2int = (uint8_t*)&y_int;
		
		z_byte[0] =  *zb2int++;
		z_byte[1] =  *zb2int;
		zb2int = (uint8_t*)&z_int;
		
		if(x_byte[0] == 0x00) x_byte[0] = 0x01;
			USARTx_PutChar('$', USART2);	
			USARTx_PutChar('X', USART2);
			USARTx_PutByte(x_byte[0], USART2);
			USARTx_PutByte(x_byte[1], USART2);	
			USARTx_PutChar('Y', USART2);
			USARTx_PutByte(y_byte[0], USART2);
			USARTx_PutByte(y_byte[1], USART2);			
			USARTx_PutChar('Z', USART2);
			USARTx_PutByte(z_byte[0], USART2);
			USARTx_PutByte(z_byte[1], USART2);	
			USARTx_PutChar('#', USART2);		
			USARTx_PutChar('\n', USART2);	
}

void fusaoSens(){ 
	Get_Mag(&mag_data);
			get_xyz_f(&txPackage.xAxis,&txPackage.yAxis, &txPackage.zAxis, ganho_filtro,0.002);
		  fMag[0] = mag_data[0] * 0.5 + (fMag[0] * (1.0 - 0.5));
			fMag[1] = mag_data[1] * 0.5 + (fMag[1] * (1.0 - 0.5));
			fMag[2] = mag_data[2] * 0.5 + (fMag[2] * (1.0 - 0.5));
			
			test_f1 = (atan2(fMag[1],fMag[0]) )  ;
			test_f1 += 0.259763; 
		  if (test_f1 < 0) test_f1 += 2*PI;
		  if (test_f1 > 2*PI) test_f1 -= 2*PI;
			test_f2 = test_f1*Rad2Dree;

		
			
			//test_f = test_f * 180/
			txPackage.zAxis = mag_data[0]*0.8 + txPackage.zAxis * 0.2;
}

void controle_xy(){ // função do controle
    float p_gain, i_gain;
	// trecho longo! IMU/////////////////

	//angulo = x;
    /////////////////////////////////
    //amostrar_pot(); //recebe amostra do ângulo do sensor
    //p_gain = analogReadPA(3);
    //i_gain = analogReadPA(4);
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
}

void controle_z(){ 
	
}

void batStatus(){ 
	//verifica se tensao da bateria é menor que 11V
}

void sinc_vars(){
	USARTx_PutChar('$', USART2);
	//ganho dos atuadores
	USARTx_PutByte((int8_t)pwm1, USART2);
	USARTx_PutByte((int8_t)pwm2, USART2);
	USARTx_PutByte((int8_t)pwm3, USART2);
	USARTx_PutByte((int8_t)pwm4, USART2);
	//ganhos do pid
	USARTx_PutByte((int8_t)p_gain, USART2);
	USARTx_PutByte((int8_t)i_gain, USART2);
	USARTx_PutByte((int8_t)d_gain, USART2);
	//ganho do filtro complementar para roll e pitch
	USARTx_PutByte((int8_t)ganho_filtro, USART2);
	USARTx_PutChar('#', USART2);
}

void USART2_IRQHandler(void)
{
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
		{
			USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
			
		  caracter = (char)USART_ReceiveData(USART2);
			if(caracter == 't'){ 
				USARTx_PutChar('T', USART2); 
				tel = !tel;
			}
			if(caracter == 'p') pwm1 = pwm1 + 100;
			else if(caracter == 'l') pwm1 = pwm1 -100;
			
			if(caracter == 'S'){ // inicia sincronização de variaveis
				
			}
			USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
		}
}  

void SysTick_Handler(void){	// a cada (1/contagem)s:
	if(us2_cont < 10) us2_cont ++; 
	else{
		FTX_USART2 = !FTX_USART2; // habilita flag de transmissão rf 
		us2_cont = 0;
	}
	if(i2c2_cont < 1) i2c2_cont ++; 
	else{
		FRX_I2C2 = !FRX_I2C2;
		i2c2_cont = 0;
	}


}