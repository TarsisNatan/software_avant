//codigo originalmente feito para mpu6050
//configuraçoes da porta i2c do stm32

/* Define to prevent recursive inclusion*/
#ifndef __HAL_MPU6050_H
#define __HAL_MPU6050_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes */
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"


/**
 * @addtogroup  MPU6050_I2C_Define
 * @{
 */

#define MPU6050_I2C                  I2C1
#define MPU6050_I2C_RCC_Periph       RCC_APB1Periph_I2C1
#define MPU6050_I2C_Port             GPIOB
#define MPU6050_I2C_SCL_Pin          GPIO_Pin_6
#define MPU6050_I2C_SDA_Pin          GPIO_Pin_7
#define MPU6050_I2C_RCC_Port         RCC_APB2Periph_GPIOB
#define MPU6050_I2C_Speed            100000 // 100kHz standard mode

/**
 *@}
 *//* end of group MPU6050_I2C_Define */

#ifdef __cplusplus
}
#endif

#endif /* __HAL___MPU6050_H */

/******************* (C) COPYRIGHT 2012 Harinadha Reddy Chintalapalli *****END OF FILE****/
