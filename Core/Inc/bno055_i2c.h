/*
 * bno055_i2c.h
 *
 *  Created on: Nov 22, 2024
 *      Author: Victor Kalenda
 *
 *      All credit given to https://github.com/ivyknob/bno055_stm32/tree/master
 *      where the code was found and used for this stm32 application
 *
 */

#ifndef INC_BNO055_I2C_H_
#define INC_BNO055_I2C_H_

#ifdef __cplusplus
  extern "C" {
#endif

#include "main.h"

#ifdef FREERTOS_ENABLED
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#endif

#include "bno055.h"

extern I2C_HandleTypeDef hi2c1;

I2C_HandleTypeDef *_bno055_i2c_port;

void bno055_delay(int time)
{
#ifdef FREERTOS_ENABLED
  osDelay(time);
#else
  HAL_Delay(time);
#endif
}

uint8_t bno055_writeData(uint8_t reg, uint8_t data)
{
	uint8_t txdata[2] = {reg, data};
	return HAL_I2C_Master_Transmit(&hi2c1, BNO055_I2C_ADDR << 1, txdata, sizeof(txdata), 10);
}

uint8_t bno055_readData(uint8_t reg, uint8_t *data, uint8_t len)
{
	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_I2C_Master_Transmit(&hi2c1, BNO055_I2C_ADDR << 1, &reg, 1, 100);
	if(status != HAL_OK)
	{
		return status;
	}
	status = HAL_I2C_Master_Receive(&hi2c1, BNO055_I2C_ADDR << 1, data, len, 100);

	return status;
}

uint8_t bno055_read_data_dma(uint8_t reg, uint8_t *data, uint8_t len)
{
	return HAL_OK;
}

#ifdef __cplusplus
  }
#endif

#endif /* INC_BNO055_I2C_H_ */
