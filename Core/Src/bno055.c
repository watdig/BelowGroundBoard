/*
 * bno055.c
 *
 *  Created on: Nov 22, 2024
 *      Author: Victor Kalenda
 *
 *      Credit given to https://github.com/ivyknob/bno055_stm32/tree/master
 *      where the framework of this code was found for STM32 applications
 */

#include "main.h"
#include "bno055.h"
#include <string.h>


typedef struct mem_read_s
{
	bno055_vector_type_t reg;
	uint8_t reg_len;
} mem_read_t;

mem_read_t mem_read_map[NUM_VECTORS] =
{
		{BNO055_VECTOR_ACCELEROMETER, 6},
		{BNO055_VECTOR_MAGNETOMETER, 6},
		{BNO055_VECTOR_GYROSCOPE, 6},
		{BNO055_VECTOR_EULER, 6},
		{BNO055_VECTOR_LINEARACCEL, 6},
		{BNO055_VECTOR_GRAVITY, 6},
		{BNO055_VECTOR_QUATERNION, 8},
};

uint8_t read_index;
uint32_t i2c_time;
volatile uint8_t i2c_tx_int;
volatile uint8_t i2c_rx_int;

extern I2C_HandleTypeDef hi2c1;
extern uint16_t holding_register_database[NUM_HOLDING_REGISTERS];


void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	i2c_tx_int = 1;
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	i2c_rx_int = 1;
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	// Do something
}









uint16_t accelScale = 100;
uint16_t tempScale = 1;
uint16_t angularRateScale = 16;
uint16_t eulerScale = 16;
uint16_t magScale = 16;
uint16_t quaScale = (1<<14);    // 2^14


void bno055_setPage(uint8_t page) { bno055_writeData(BNO055_PAGE_ID, page); }

uint8_t poll_transaction();

bno055_opmode_t bno055_getOperationMode()
{
	bno055_opmode_t mode;
	bno055_readData(BNO055_OPR_MODE, &mode, 1);
	return mode;
}

void bno055_setOperationMode(bno055_opmode_t mode)
{
	bno055_writeData(BNO055_OPR_MODE, mode);
	if (mode == BNO055_OPERATION_MODE_CONFIG)
	{
		bno055_delay(19);
	}
	else
	{
		bno055_delay(7);
	}
}

void bno055_setOperationModeConfig()
{
	bno055_setOperationMode(BNO055_OPERATION_MODE_CONFIG);
}

void bno055_setOperationModeNDOF()
{
	bno055_setOperationMode(BNO055_OPERATION_MODE_NDOF);
}

void bno055_setExternalCrystalUse(bool state)
{
	bno055_setPage(0);
	uint8_t tmp = 0;
	bno055_readData(BNO055_SYS_TRIGGER, &tmp, 1);
	tmp |= (state == true) ? 0x80 : 0x0;
	bno055_writeData(BNO055_SYS_TRIGGER, tmp);
	bno055_delay(700);
}

void bno055_enableExternalCrystal() { bno055_setExternalCrystalUse(true); }
void bno055_disableExternalCrystal() { bno055_setExternalCrystalUse(false); }

uint8_t poll_transaction()
{
	i2c_time = HAL_GetTick();
	while(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
	{
		if(HAL_GetTick() - i2c_time >= I2C_TIMEOUT_MS)
		{
			return HAL_ERROR;
		}
	}
	return HAL_OK;
}

uint8_t bno055_reset()
{
	uint8_t status = HAL_OK;
	status = bno055_writeData(BNO055_SYS_TRIGGER, 0x20);
	if(status != HAL_OK)
	{
	  return status;
	}
	bno055_delay(700);
	return status;
}

int8_t bno055_getTemp()
{
  bno055_setPage(0);
  uint8_t t;
  bno055_readData(BNO055_TEMP, &t, 1);
  return t;
}

uint8_t bno055_setup()
{
	uint8_t status = HAL_OK;
	read_index = 0;
	status = bno055_reset();
	if(status != HAL_OK)
	{
		return status;
	}

	uint8_t id = 0;
	status = bno055_readData(BNO055_CHIP_ID, &id, 1);
	if(status != HAL_OK)
	{
		return status;
	}
	if (id != BNO055_ID)
	{
		return HAL_ERROR;
	}
	bno055_setPage(0);
	status = bno055_writeData(BNO055_SYS_TRIGGER, 0x0); // TODO: change to external oscillator

	if(status != HAL_OK)
	{
		return status;
	}
	// Select BNO055 config mode
	bno055_setOperationModeConfig();
	bno055_delay(10);
	return status;
}

int16_t bno055_getSWRevision()
{
  bno055_setPage(0);
  uint8_t buffer[2];
  bno055_readData(BNO055_SW_REV_ID_LSB, buffer, 2);
  return (int16_t)((buffer[1] << 8) | buffer[0]);
}

uint8_t bno055_getBootloaderRevision()
{
  bno055_setPage(0);
  uint8_t tmp;
  bno055_readData(BNO055_BL_REV_ID, &tmp, 1);
  return tmp;
}

uint8_t bno055_getSystemStatus()
{
  bno055_setPage(0);
  uint8_t tmp;
  bno055_readData(BNO055_SYS_STATUS, &tmp, 1);
  return tmp;
}

bno055_self_test_result_t bno055_getSelfTestResult()
{
  bno055_setPage(0);
  uint8_t tmp;
  bno055_self_test_result_t res = {
      .mcuState = 0, .gyrState = 0, .magState = 0, .accState = 0};
  bno055_readData(BNO055_ST_RESULT, &tmp, 1);
  res.mcuState = (tmp >> 3) & 0x01;
  res.gyrState = (tmp >> 2) & 0x01;
  res.magState = (tmp >> 1) & 0x01;
  res.accState = (tmp >> 0) & 0x01;
  return res;
}

uint8_t bno055_getSystemError()
{
  bno055_setPage(0);
  uint8_t tmp;
  bno055_readData(BNO055_SYS_ERR, &tmp, 1);
  return tmp;
}

bno055_calibration_state_t bno055_getCalibrationState()
{
  bno055_setPage(0);
  bno055_calibration_state_t cal = {.sys = 0, .gyro = 0, .mag = 0, .accel = 0};
  uint8_t calState = 0;
  bno055_readData(BNO055_CALIB_STAT, &calState, 1);
  cal.sys = (calState >> 6) & 0x03;
  cal.gyro = (calState >> 4) & 0x03;
  cal.accel = (calState >> 2) & 0x03;
  cal.mag = calState & 0x03;
  return cal;
}


bno055_calibration_data_t bno055_getCalibrationData()
{
  bno055_calibration_data_t calData;
  uint8_t buffer[22];
  bno055_opmode_t operationMode = bno055_getOperationMode();
  bno055_setOperationModeConfig();
  bno055_setPage(0);

  bno055_readData(BNO055_ACC_OFFSET_X_LSB, buffer, 22);

  // Assumes little endian processor
  memcpy(&calData.offset.accel, buffer, 6);
  memcpy(&calData.offset.mag, buffer + 6, 6);
  memcpy(&calData.offset.gyro, buffer + 12, 6);
  memcpy(&calData.radius.accel, buffer + 18, 2);
  memcpy(&calData.radius.mag, buffer + 20, 2);

  bno055_setOperationMode(operationMode);

  return calData;
}

void bno055_setCalibrationData(bno055_calibration_data_t calData)
{
  uint8_t buffer[22];
  bno055_opmode_t operationMode = bno055_getOperationMode();
  bno055_setOperationModeConfig();
  bno055_setPage(0);

  // Assumes litle endian processor
  memcpy(buffer, &calData.offset.accel, 6);
  memcpy(buffer + 6, &calData.offset.mag, 6);
  memcpy(buffer + 12, &calData.offset.gyro, 6);
  memcpy(buffer + 18, &calData.radius.accel, 2);
  memcpy(buffer + 20, &calData.radius.mag, 2);

  for (uint8_t i=0; i < 22; i++)
  {
    // TODO(oliv4945): create multibytes write
    bno055_writeData(BNO055_ACC_OFFSET_X_LSB+i, buffer[i]);
  }

  bno055_setOperationMode(operationMode);
}

void bno055_get_all_values()
{
	uint8_t* buffer = (uint8_t*)(&holding_register_database[12]);
	for(uint8_t i = 0; i < NUM_VECTORS; i++)
	{
		bno055_readData(mem_read_map[i].reg, &buffer[6 * i], mem_read_map[i].reg_len);
	}
}
uint8_t bno055_rx()
{
	if(i2c_rx_int)
	{
		i2c_rx_int = 0;
		return 1;
	}
	return i2c_rx_int;
}
uint8_t bno055_queue_transaction()
{
	uint8_t status = HAL_OK;
	uint8_t* buffer = (uint8_t*)(&holding_register_database[12]);
	status = HAL_I2C_Mem_Read_IT(&hi2c1, BNO055_I2C_ADDR << 1, mem_read_map[read_index].reg,
			I2C_MEMADD_SIZE_8BIT, &buffer[6 * read_index], mem_read_map[read_index].reg_len);
	if(status != HAL_OK)
	{
		return status;
	}
	read_index = (read_index == NUM_VECTORS - 1)? 0 : read_index + 1;
	return status;
}

void bno055_setAxisMap(bno055_axis_map_t axis)
{
  uint8_t axisRemap = (axis.z << 4) | (axis.y << 2) | (axis.x);
  uint8_t axisMapSign = (axis.x_sign << 2) | (axis.y_sign << 1) | (axis.z_sign);
  bno055_writeData(BNO055_AXIS_MAP_CONFIG, axisRemap);
  bno055_writeData(BNO055_AXIS_MAP_SIGN, axisMapSign);
}


