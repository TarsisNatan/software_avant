//Biblioteca para ler e tratar dados do sensor MPU9250 com filtro complementar
//Projeto AVANT
//Adaptada e modificada do MPU6050 por:
//Társis Natan Boff da Silva
//28/04/2016

/* Includes */
#include "MPU6050.h"
#include "HAL_MPU6050.h"
#include "stm32f10x_i2c.h"
#include "math.h"

char currentGyroRange = 3,
	   currentAcceleroRange = 3,
		 mscale = 1;
/** @defgroup MPU6050_Library
 * @{
 */

/** Power on and prepare for general usage.
 * This will activate the device and take it out of sleep mode (which must be done
 * after start-up). This function also sets both the accelerometer and the gyroscope
 * to their most sensitive settings, namely +/- 2g and +/- 250 degrees/sec, and sets
 * the clock source to use the X Gyro for reference, which is slightly better than
 * the default internal clock source.
 */
void MPU6050_Initialize()
{
    MPU6050_SetClockSource(MPU6050_CLOCK_PLL_XGYRO);
    MPU6050_SetFullScaleGyroRange(MPU6050_GYRO_FS_250);
    MPU6050_SetFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    MPU6050_SetSleepModeStatus(DISABLE);
}

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, FALSE otherwise
 */
bool MPU6050_TestConnection()
{
    return MPU6050_GetDeviceID() == 0x38 ? TRUE : FALSE; //0b110100; 8-bit representation in hex = 0x34
}
// WHO_AM_I register

/** Get Device ID.
 * This register is used to verify the identity of the device (0b110100).
 * @return Device ID (should be 0x68, 104 dec, 150 oct)
 * @see MPU6050_RA_WHO_AM_I
 * @see MPU6050_WHO_AM_I_BIT
 * @see MPU6050_WHO_AM_I_LENGTH
 */
uint8_t MPU6050_GetDeviceID()
{
    uint8_t tmp;
    MPU6050_ReadBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH, &tmp);
    return tmp;
}



/** Set clock source setting.
 * An internal 8MHz oscillator, gyroscope based clock, or external sources can
 * be selected as the MPU-60X0 clock source. When the internal 8 MHz oscillator
 * or an external source is chosen as the clock source, the MPU-60X0 can operate
 * in low power modes with the gyroscopes disabled.
 *
 * Upon power up, the MPU-60X0 clock source defaults to the internal oscillator.
 * However, it is highly recommended that the device be configured to use one of
 * the gyroscopes (or an external clock source) as the clock reference for
 * improved stability. The clock source can be selected according to the following table:
 *
 * <pre>
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
 * </pre>
 *
 * @param source New clock source setting
 * @see MPU6050_GetClockSource()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_CLKSEL_BIT
 * @see MPU6050_PWR1_CLKSEL_LENGTH
 */
void MPU6050_SetClockSource(uint8_t source)
{
    MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}

/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see MPU6050_GetFullScaleGyroRange()
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
void MPU6050_SetFullScaleGyroRange(uint8_t range)
{
    MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

// GYRO_CONFIG register

/** Get full-scale gyroscope range.
 * The FS_SEL parameter allows setting the full-scale range of the gyro sensors,
 * as described in the table below.
 *
 * <pre>
 * 0 = +/- 250 degrees/sec
 * 1 = +/- 500 degrees/sec
 * 2 = +/- 1000 degrees/sec
 * 3 = +/- 2000 degrees/sec
 * </pre>
 *
 * @return Current full-scale gyroscope range setting
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
uint8_t MPU6050_GetFullScaleGyroRange()
{
    uint8_t tmp;
    MPU6050_ReadBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, &tmp);
    return tmp;
}

/** Get full-scale accelerometer range.
 * The FS_SEL parameter allows setting the full-scale range of the accelerometer
 * sensors, as described in the table below.
 *
 * <pre>
 * 0 = +/- 2g
 * 1 = +/- 4g
 * 2 = +/- 8g
 * 3 = +/- 16g
 * </pre>
 *
 * @return Current full-scale accelerometer range setting
 * @see MPU6050_ACCEL_FS_2
 * @see MPU6050_RA_ACCEL_CONFIG
 * @see MPU6050_ACONFIG_AFS_SEL_BIT
 * @see MPU6050_ACONFIG_AFS_SEL_LENGTH
 */
uint8_t MPU6050_GetFullScaleAccelRange()
{
    uint8_t tmp;
    MPU6050_ReadBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, &tmp);
    return tmp;
}

/** Set full-scale accelerometer range.
 * @param range New full-scale accelerometer range setting
 * @see MPU6050_GetFullScaleAccelRange()
 */
void MPU6050_SetFullScaleAccelRange(uint8_t range)
{
    MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

/** Get sleep mode status.
 * Setting the SLEEP bit in the register puts the device into very low power
 * sleep mode. In this mode, only the serial interface and internal registers
 * remain active, allowing for a very low standby current. Clearing this bit
 * puts the device back into normal mode. To save power, the individual standby
 * selections for each of the gyros should be used if any gyro axis is not used
 * by the application.
 * @return Current sleep mode enabled status
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_SLEEP_BIT
 */
bool MPU6050_GetSleepModeStatus()
{
    uint8_t tmp;
    MPU6050_ReadBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, &tmp);
    return tmp == 0x00 ? FALSE : TRUE;
}

/** Set sleep mode status.
 * @param enabled New sleep mode enabled status
 * @see MPU6050_GetSleepModeStatus()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_SLEEP_BIT
 */
void MPU6050_SetSleepModeStatus(FunctionalState NewState)
{
    MPU6050_WriteBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, NewState);
}

/** Get raw 6-axis motion sensor readings (accel/gyro).
 * Retrieves all currently available motion sensor values.
 * @param AccelGyro 16-bit signed integer array of length 6
 * @see MPU6050_RA_ACCEL_XOUT_H
 */
void MPU6050_GetRawAccelGyro(s16* AccelGyro)
{
    u8 tmpBuffer[14];
    MPU6050_I2C_BufferRead(MPU6050_DEFAULT_ADDRESS, tmpBuffer, MPU6050_RA_ACCEL_XOUT_H, 14);
    /* Get acceleration */
    for (int i = 0; i < 3; i++)
        AccelGyro[i] = ((s16) ((u16) tmpBuffer[2 * i] << 8) + tmpBuffer[2 * i + 1]);
    /* Get Angular rate */
    for (int i = 4; i < 7; i++)
        AccelGyro[i - 1] = ((s16) ((u16) tmpBuffer[2 * i] << 8) + tmpBuffer[2 * i + 1]);

}

/** Write multiple bits in an 8-bit device register.
 * @param slaveAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-7)
 * @param length Number of bits to write (not more than 8)
 * @param data Right-aligned value to write
 */
void MPU6050_WriteBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data)
{
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    uint8_t tmp;
    MPU6050_I2C_BufferRead(slaveAddr, &tmp, regAddr, 1);
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    data <<= (bitStart - length + 1); // shift data into correct position
    data &= mask; // zero all non-important bits in data
    tmp &= ~(mask); // zero all important bits in existing byte
    tmp |= data; // combine data with existing byte
    MPU6050_I2C_ByteWrite(slaveAddr, &tmp, regAddr);
}

/** write a single bit in an 8-bit device register.
 * @param slaveAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-7)
 * @param value New bit value to write
 */
void MPU6050_WriteBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data)
{
    uint8_t tmp;
    MPU6050_I2C_BufferRead(slaveAddr, &tmp, regAddr, 1);
    tmp = (data != 0) ? (tmp | (1 << bitNum)) : (tmp & ~(1 << bitNum));
    MPU6050_I2C_ByteWrite(slaveAddr, &tmp, regAddr);
}

/** Read multiple bits from an 8-bit device register.
 * @param slaveAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-7)
 * @param length Number of bits to read (not more than 8)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in readTimeout)
 */
void MPU6050_ReadBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data)
{
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    uint8_t tmp;
    MPU6050_I2C_BufferRead(slaveAddr, &tmp, regAddr, 1);
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    tmp &= mask;
    tmp >>= (bitStart - length + 1);
    *data = tmp;
}

/** Read a single bit from an 8-bit device register.
 * @param slaveAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-7)
 * @param data Container for single bit value
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in readTimeout)
 */
void MPU6050_ReadBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data)
{
    uint8_t tmp;
    MPU6050_I2C_BufferRead(slaveAddr, &tmp, regAddr, 1);
    *data = tmp & (1 << bitNum);
}

/**
 * @brief  Initializes the I2C peripheral used to drive the MPU6050
 * @param  None
 * @return None
 */
void MPU6050_I2C_Init()
{
    I2C_InitTypeDef I2C_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable I2C and GPIO clocks */
    RCC_APB1PeriphClockCmd(MPU6050_I2C_RCC_Periph, ENABLE);
    RCC_APB2PeriphClockCmd(MPU6050_I2C_RCC_Port, ENABLE);

    /* Configure I2C pins: SCL and SDA */
    GPIO_InitStructure.GPIO_Pin = MPU6050_I2C_SCL_Pin | MPU6050_I2C_SDA_Pin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(MPU6050_I2C_Port, &GPIO_InitStructure);

    /* I2C configuration */
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00; // MPU6050 7-bit adress = 0x68, 8-bit adress = 0xD0;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = MPU6050_I2C_Speed;

    /* Apply I2C configuration after enabling it */
    I2C_Init(MPU6050_I2C, &I2C_InitStructure);
    /* I2C Peripheral Enable */
    I2C_Cmd(MPU6050_I2C, ENABLE);
}

/**
 * @brief  Writes one byte to the  MPU6050.
 * @param  slaveAddr : slave address MPU6050_DEFAULT_ADDRESS
 * @param  pBuffer : pointer to the buffer  containing the data to be written to the MPU6050.
 * @param  writeAddr : address of the register in which the data will be written
 * @return None
 */
void MPU6050_I2C_ByteWrite(u8 slaveAddr, u8* pBuffer, u8 writeAddr)
{
    // ENTR_CRT_SECTION();

    /* Send START condition */
    I2C_GenerateSTART(MPU6050_I2C, ENABLE);

    /* Test on EV5 and clear it */
    while (!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_MODE_SELECT));

    /* Send MPU6050 address for write */
    I2C_Send7bitAddress(MPU6050_I2C, slaveAddr, I2C_Direction_Transmitter);

    /* Test on EV6 and clear it */
    while (!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    /* Send the MPU6050's internal address to write to */
    I2C_SendData(MPU6050_I2C, writeAddr);

    /* Test on EV8 and clear it */
    while (!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    /* Send the byte to be written */
    I2C_SendData(MPU6050_I2C, *pBuffer);

    /* Test on EV8 and clear it */
    while (!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    /* Send STOP condition */
    I2C_GenerateSTOP(MPU6050_I2C, ENABLE);

    // EXT_CRT_SECTION();
}

/**
 * @brief  Reads a block of data from the MPU6050.
 * @param  slaveAddr  : slave address MPU6050_DEFAULT_ADDRESS
 * @param  pBuffer : pointer to the buffer that receives the data read from the MPU6050.
 * @param  readAddr : MPU6050's internal address to read from.
 * @param  NumByteToRead : number of bytes to read from the MPU6050 ( NumByteToRead >1  only for the Mgnetometer readinf).
 * @return None
 */
void MPU6050_I2C_BufferRead(u8 slaveAddr, u8* pBuffer, u8 readAddr, u16 NumByteToRead)
{
    // ENTR_CRT_SECTION();

    /* While the bus is busy */
    while (I2C_GetFlagStatus(MPU6050_I2C, I2C_FLAG_BUSY));

    /* Send START condition */
    I2C_GenerateSTART(MPU6050_I2C, ENABLE);

    /* Test on EV5 and clear it */
    while (!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_MODE_SELECT));

    /* Send MPU6050 address for write */
    I2C_Send7bitAddress(MPU6050_I2C, slaveAddr, I2C_Direction_Transmitter);

    /* Test on EV6 and clear it */
    while (!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    /* Clear EV6 by setting again the PE bit */
    I2C_Cmd(MPU6050_I2C, ENABLE);

    /* Send the MPU6050's internal address to write to */
    I2C_SendData(MPU6050_I2C, readAddr);

    /* Test on EV8 and clear it */
    while (!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    /* Send STRAT condition a second time */
    I2C_GenerateSTART(MPU6050_I2C, ENABLE);

    /* Test on EV5 and clear it */
    while (!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_MODE_SELECT));

    /* Send MPU6050 address for read */
    I2C_Send7bitAddress(MPU6050_I2C, slaveAddr, I2C_Direction_Receiver);

    /* Test on EV6 and clear it */
    while (!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

    /* While there is data to be read */
    while (NumByteToRead)
    {
        if (NumByteToRead == 1)
        {
            /* Disable Acknowledgement */
            I2C_AcknowledgeConfig(MPU6050_I2C, DISABLE);

            /* Send STOP Condition */
            I2C_GenerateSTOP(MPU6050_I2C, ENABLE);
        }

        /* Test on EV7 and clear it */
        if (I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED))
        {
            /* Read a byte from the MPU6050 */
            *pBuffer = I2C_ReceiveData(MPU6050_I2C);

            /* Point to the next location where the byte read will be saved */
            pBuffer++;

            /* Decrement the read bytes counter */
            NumByteToRead--;
        }
    }

    /* Enable Acknowledgement to be ready for another reception */
    I2C_AcknowledgeConfig(MPU6050_I2C, ENABLE);
    // EXT_CRT_SECTION();
}


/* @brief função pra ativar/desativar e alterar valor da BW do LPF
 * @param peso: frequencia de corte :
 * 		BW(Hz)		peso(reg)		delay(ms)
 * 		xxxx		xxxx			0.75
 * 		256         0x00			1.94
 *		188         0x01			5.80
 *		98          0x02			7.80
 *		42          0x03			11.80
 *		20          0x04			19.80
 *		10          0x05			35.70
 *		5           0x06			66.96
 * @return None
 */
//AINDA NÃO FUNCIONA!!!!!
void set_LPF(u8 peso){
	if(peso>6)return;
	MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU9250_RA_ACCEL_CONFIG2,2,3,peso);
}


/* @brief função pra retornar o valor no reg de LPF
 * @return byte
 */
uint8_t  get_LPF(){
	uint8_t temp;
	MPU6050_ReadBits(MPU6050_DEFAULT_ADDRESS, MPU9250_RA_ACCEL_CONFIG2, 2,3, &temp);
	return temp;
}


/* @brief função pra receber os angulos de orintação do mpu filtrados (filtro complementar)
 * @param xtetaf: contém valor do angulo X filtrado
 * @param ytetaf: contém valor do angulo Y filtrado

* @param P: peso do filtro complementar (FREQUENCIA DE CORTE)
 * @param T: periodo de amostragem
 * @return None
 */
void get_xyz_f(float *xtetaf, float *ytetaf, float *ztetaf, float P,float T){
	float acx, acy,acz,gyx, gyy, gyz,teta_acelx, teta_acely, wxgy, wygy, teta_acelz, wzgy;
	s16 bytes[6], *temp;
	//temp =* &bytes;

	MPU6050_GetRawAccelGyro(&bytes[0]);
	acx = bytes[0]/A_R;
	acy = bytes[1]/A_R;
	acz = bytes[2]/A_R;

	gyx = bytes[3]/A_R;
	gyy = bytes[4]/A_R;
	gyz = bytes[5]/A_R;
	// Calcula o angulo de X com acelerometro em DG.
	teta_acelx = atanf(acx / A_R / sqrtf(powf((acy/A_R),2) + powf((acz/A_R),2)))*RAD_TO_DEG;
	// Calcula o angulo de Y com acelerometro em DG.
	teta_acely = atanf(acy / A_R / sqrtf(powf((acx/A_R),2) + powf((acz/A_R),2)))*RAD_TO_DEG;
	// teste 
	teta_acelz = (acz / A_R / sqrtf(powf((acx/A_R),2) + powf((acy/A_R),2)))*RAD_TO_DEG;
	wzgy = gyz * G_R;
	// Calcula o vel ang de X com giroscopio em DG/s.
	wxgy =gyx * G_R;
	// Calcula o vel ang de Y com giroscopio em DG/s.
	wygy =gyy * G_R;

	//filtragem dos tetasX
	*xtetaf   = P*(*xtetaf - teta_acelx) + teta_acelx + wxgy*T; //filtro complementar
	//tetaX_complementar_f = (0.95*tetaX_complementar_f + 0.05*tetaX_complementar);  // filtro passa baixas opcional
	//filtragem dos tetasY
	*ytetaf   = P*(*ytetaf - teta_acely) + teta_acely + wygy*T; //filtro complementar
	//tetaY_complementar_f = (0.8*tetaY_complementar_f + 0.2*tetaY_complementar);    // filtro passa baixas opcional
	// teste ang z
	*ztetaf   = wzgy*T;
	
}

// WHO_AM_I register

/** MPU9250_OnMag.
 * liga mag.
 * @return mag ID (0x48)

 */
uint8_t MPU9250_OnMag()
{
    uint8_t tmp, byte = 0x22;
	
  MPU6050_I2C_BufferRead(MPU6050_DEFAULT_ADDRESS, &tmp, 0x37, 1);
	 tmp |= byte;
	MPU6050_I2C_ByteWrite(MPU6050_DEFAULT_ADDRESS, &tmp, 0x37);
  
  MPU6050_I2C_BufferRead(AK8963_ADDRESS_DEFAULT, &tmp, 0x00, 1);
    return tmp;
}

void Initialize_AK8963(float *destination)
{
	u8 rawData[3], byte = 0x00;
	MPU6050_I2C_ByteWrite(AK8963_ADDRESS_DEFAULT, &byte, AK8963_CNTL);
	delay(10000);
	byte = 0x0F;
	MPU6050_I2C_ByteWrite(AK8963_ADDRESS_DEFAULT,  &byte, AK8963_CNTL);
  delay(10000);
	MPU6050_I2C_BufferRead(AK8963_ADDRESS_DEFAULT, rawData, AK8963_ASAX,  3);
	destination[0] =  (float)(rawData[0] - 128)/256.0f + 1.0f;   // Return x-axis sensitivity adjustment values, etc.
  destination[1] =  (float)(rawData[1] - 128)/256.0f + 1.0f;  
  destination[2] =  (float)(rawData[2] - 128)/256.0f + 1.0f; 
	byte = 0x00;
	MPU6050_I2C_ByteWrite(AK8963_ADDRESS_DEFAULT,  &byte, AK8963_CNTL);
	delay(10000);
	byte = (mscale << 4) | 0x02;  // Output bit setting  = 16 bits | Operation mode setting = Single measurement mode
	MPU6050_I2C_ByteWrite(AK8963_ADDRESS_DEFAULT, &byte , AK8963_CNTL);
	delay(1000);
}

void AK8963_Calibrate(float *MCali)
{
	u8 data[3], byte = 0x1F;
	MPU6050_I2C_ByteWrite(AK8963_ADDRESS_DEFAULT,  &byte, AK8963_CNTL);
	delay(10000);
	MPU6050_I2C_BufferRead(AK8963_ADDRESS_DEFAULT, data, AK8963_ASAX,  3);
	MCali[0] = (float)(data[0] - 128)/256. +1.;
	MCali[1] = (float)(data[1] - 128)/256. +1.;
	MCali[2] = (float)(data[2] - 128)/256. +1.;
	//delete data;
}


void delay(u32 k)
{
	while (k != 0)
	{
		k--;
	}
}

void Get_Mag(float *Mag)
{
	u8 data[7];
	//u16 temp;
	//Write_Byte(AK8963_ADDRESS_DEFAULT, AK8963_CNTL, 0x01);
	MPU6050_I2C_BufferRead(AK8963_ADDRESS_DEFAULT,  data, AK8963_ST1,  1);
	if (data[0] & 0x01)
	{
		MPU6050_I2C_BufferRead(AK8963_ADDRESS_DEFAULT,  data, AK8963_XOUT_L, 7);
		if (!(data[6] & 0x08))
		{
			Mag[0] = (float)((int16_t)(data[1] << 8) | data[0]);
			Mag[1] = (float)((int16_t)(data[3] << 8) | data[2]);
			Mag[2] = (float)((int16_t)(data[5] << 8) | data[4]);
		}
		
				Mag[0] *= 10.*4912./32760.0;
				Mag[1] *= 10.*4912./32760.0;
				Mag[2] *= 10.*4912./32760.0;
		
		
	}
}