/*
Codigo referente a estação de solo do projeto AVANT
Recebe leitura dos sensores via rádio LoRa e apresenta dados em máquina ocal via serial 
Envia comandos para os motores (PWM) da aeronave via rádio LoRa 
*/
#include "stm32f10x.h"
#include "string.h"

//const t_ser_text = 200;
#define tpack 28
#define BUF_SIZE	16
#define contagem 200 // 10ms

char buf[BUF_SIZE],caracter;
char iRX2 = 0, iRX1 = 0;

float x ,y, temp;
uint8_t status; 
bool mpu_status;
char text[100]; 

char buffer1[tpack],buffer2[tpack];
char *pb1 = &buffer1[0],*pb2 = &buffer2[0];
int cont = 0;
char f2c[4];
	
uint8_t t_string;

void USART1_Init(void);
void USART2_Init(void);
void USARTx_PutChar(char c, USART_TypeDef *USARTx);
void USARTx_PutString(char *s, USART_TypeDef *USARTx);
uint16_t USARTx_GetChar(USART_TypeDef *USARTx);
void getUSARTx_putUSARTy(USART_TypeDef *USARTx, USART_TypeDef *USARTy, char *pbuff);
void LoraInit(); 



int main(){ 
 	SystemInit();
    /* Initialize USART1 */
	USART2_Init();
  USART1_Init();
	SysTick_Config(SystemCoreClock/contagem ); 
	LoraInit();
	while(1){ 
		//recebe pacote uart por LoRa e envia para porta serial de máquina local usando IRQ
	}
}

void LoraInit(){
	GPIO_InitTypeDef GPIO_InitStruct;
	//config = pb12 e reset = pb13 pins:
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	GPIO_WriteBit(GPIOB, GPIO_Pin_13, Bit_RESET); //reset modulo
	GPIO_WriteBit(GPIOB, GPIO_Pin_13, Bit_SET);
	GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_RESET); // config mode
	snprintf(&buf[0], sizeof(buf),"AT\r\n");
	USARTx_PutString(&buf[0], USART2);
	snprintf(&buf[0], sizeof(buf),"AT+SPR=3\r\n");
	USARTx_PutString(&buf[0], USART2);
	
	snprintf(&buf[0], sizeof(buf),"lora ok\r\n");
	USARTx_PutString(&buf[0], USART1);
	
	GPIO_WriteBit(GPIOB, GPIO_Pin_13, Bit_SET);
}

void USART1_Init(void)
{
    /* USART configuration structure for USART1 */
    USART_InitTypeDef usart1_init_struct;
    /* Bit configuration structure for GPIOA PIN9 and PIN10 */
    GPIO_InitTypeDef gpioa_init_struct;
     
    /* Enalbe clock for USART1, AFIO and GPIOA */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO | 
                           RCC_APB2Periph_GPIOA, ENABLE);
                            
    /* GPIOA PIN9 alternative function Tx */
    gpioa_init_struct.GPIO_Pin = GPIO_Pin_9;
    gpioa_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
    gpioa_init_struct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpioa_init_struct);
    /* GPIOA PIN9 alternative function Rx */
    gpioa_init_struct.GPIO_Pin = GPIO_Pin_10;
    gpioa_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
    gpioa_init_struct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpioa_init_struct);
 
    /* Enable USART1 */
    USART_Cmd(USART1, ENABLE);  
    /* Baud rate 9600, 8-bit data, One stop bit
     * No parity, Do both Rx and Tx, No HW flow control
     */
    usart1_init_struct.USART_BaudRate = 9600;   
    usart1_init_struct.USART_WordLength = USART_WordLength_8b;  
    usart1_init_struct.USART_StopBits = USART_StopBits_1;   
    usart1_init_struct.USART_Parity = USART_Parity_No ;
    usart1_init_struct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    usart1_init_struct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    /* Configure USART1 */
    USART_Init(USART1, &usart1_init_struct);
    /* Enable RXNE interrupt */
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    /* Enable USART1 global interrupt */
    NVIC_EnableIRQ(USART1_IRQn);
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

void getUSARTx_putUSARTy(USART_TypeDef *USARTx, USART_TypeDef *USARTy, char *pbuff)
	{
	char *ptemp;
	ptemp = pbuff;
		*pbuff++ = USART_ReceiveData(USARTx);
	USARTx_PutString(ptemp,USARTy);
	pbuff = ptemp;
}

void USART1_IRQHandler(void)
{		
	if (USART_GetITStatus(USART1, USART_IT_RXNE))
	{
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
		caracter = (char)USARTx_GetChar(USART1);
	  USARTx_PutChar(caracter,USART2);
		
		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	}	
}   


void USART2_IRQHandler(void)
{	
	if (USART_GetITStatus(USART2, USART_IT_RXNE))
	{
		USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
		getUSARTx_putUSARTy(USART2, USART1, pb1);
		USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	}
}   
void SysTick_Handler(void){	// a cada (1/contagem)s:
	
	

	
	
//    
}