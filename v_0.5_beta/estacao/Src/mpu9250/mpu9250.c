
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_i2c.h"
#include "mpu9250.h"
#include "math.h"

uint8_t readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data, uint16_t timeout, I2C_HandleTypeDef hi2c)
{
    uint16_t tout = timeout > 0 ? timeout : I2CDEV_DEFAULT_READ_TIMEOUT;

    HAL_I2C_Master_Transmit(&hi2c, devAddr << 1, &regAddr, 1, tout);
    if (HAL_I2C_Master_Receive(&hi2c, devAddr << 1, data, length, tout) == HAL_OK) return length;
    return -1;
}

void MPU6050_getMotion(imu imu,I2C_HandleTypeDef hi2c) {
    int16_t teta_ax, teta_ay, omega_gx, omega_gy;
	
	readBytes(Adr_imu, MPU6050_RA_ACCEL_XOUT_H, 14, imu.buffer, 0,hi2c);
/*   
	  ax = (((int16_t)imu.buffer[0]) << 8) | imu.buffer[1];
    ay = (((int16_t)imu.buffer[2]) << 8) | imu.buffer[3];
    az = (((int16_t)imu.buffer[4]) << 8) | imu.buffer[5];
    gx = (((int16_t)imu.buffer[8]) << 8) | imu.buffer[9];
    gy = (((int16_t)imu.buffer[10]) << 8) | imu.buffer[11];
    gz = (((int16_t)imu.buffer[12]) << 8) | imu.buffer[13];
	*/
	
	// Calcula o angulo de X com acelerometro em DG.
	teta_ax = atanf(((((int16_t)imu.buffer[0]) << 8) | imu.buffer[1]) / A_R / sqrtf(powf((((((int16_t)imu.buffer[2]) << 8) |
									imu.buffer[3])/A_R),2) + powf((((((int16_t)imu.buffer[4]) << 8) | imu.buffer[5])/A_R),2)))*RAD_TO_DEG;
	// Calcula o angulo de Y com acelerometro em DG.
	teta_ay = atanf(((((int16_t)imu.buffer[2]) << 8) | imu.buffer[3]) / A_R / sqrtf(powf((((((int16_t)imu.buffer[0]) << 8) 
								| imu.buffer[1])/A_R),2) + powf((((((int16_t)imu.buffer[4]) << 8) | imu.buffer[5])/A_R),2)))*RAD_TO_DEG;
	// Calcula o vel ang de X com giroscopio em DG/s.
	omega_gx =((((int16_t)imu.buffer[8]) << 8) | imu.buffer[9]) * G_R;
	// Calcula o vel ang de Y com giroscopio em DG/s.
	omega_gy=((((int16_t)imu.buffer[10]) << 8) | imu.buffer[11]) * G_R;

	//filtragem dos tetasX
	imu.pich   = P*(imu.pich - teta_ax) + teta_ax + omega_gy*T; //filtro complementar
	//tetaX_complementar_f = (0.95*tetaX_complementar_f + 0.05*tetaX_complementar);  // filtro passa baixas opcional
	//filtragem dos tetasY
	imu.roll   = P*(imu.roll - teta_ay) + teta_ay + omega_gx*T; //filtro complementar
	//tetaY_complementar_f = (0.8*tetaY_complementar_f + 0.2*tetaY_complementar);    // filtro passa baixas opcional
}






