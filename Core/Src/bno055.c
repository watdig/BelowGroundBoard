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
#include "error_codes.h"
#include <string.h>

#ifdef FREERTOS_ENABLED
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#endif

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

uint8_t i2c_tx_buffer[I2C_TX_BUFFER_SIZE];
uint8_t i2c_rx_buffer[I2C_RX_BUFFER_SIZE];

uint8_t read_index;
uint8_t bno055_address;
uint32_t i2c_tx_time = 0;
uint32_t i2c_rx_time = 0;
volatile uint8_t i2c_tx_int = 1;
volatile uint8_t i2c_rx_int = 1;
volatile uint8_t i2c_err_int = 0;

extern uint16_t holding_register_database[NUM_HOLDING_REGISTERS];
extern I2C_HandleTypeDef hi2c1;
extern DMA_HandleTypeDef hdma_i2c1_rx;
extern DMA_HandleTypeDef hdma_i2c1_tx;

void bno055_delay(uint32_t time);
int8_t bno055_write(uint8_t *data, uint8_t len);
int8_t bno055_read(uint8_t reg, uint8_t *data, uint8_t len);

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	i2c_tx_int = 1;
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	i2c_rx_int = 1;
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	i2c_tx_int = 1;
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	i2c_rx_int = 1;
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	// Do something
	i2c_err_int = 1;
	__HAL_I2C_DISABLE_IT(&hi2c1, I2C_IT_ERRI | I2C_IT_TCI | I2C_IT_STOPI |
										 I2C_IT_NACKI | I2C_IT_ADDRI | I2C_IT_RXI | I2C_IT_TXI);
}


void bno055_delay(uint32_t time)
{
#ifdef FREERTOS_ENABLED
  osDelay(time);
#else
  HAL_Delay(time);
#endif
}

int8_t bno055_init()
{
	int8_t status = HAL_OK;

	// Set up the local IMU
	bno055_address = BNO055_I2C_ADDR_LO << 1;
	status = bno055_setup();
	if(status != HAL_OK){return status;}

	status = bno055_setOperationModeNDOF();
	if(status != HAL_OK){return status;}

	// Setup the remote IMU
	bno055_address = BNO055_I2C_ADDR_HI << 1;
	status = bno055_setup();
	if(status != HAL_OK){return status;}

	status = bno055_setOperationModeNDOF();
	return status;
}

int8_t bno055_setPage(uint8_t page)
{
	i2c_tx_buffer[0] = BNO055_PAGE_ID;
	i2c_tx_buffer[1] = page;
	return bno055_write(i2c_tx_buffer, 2);
}


int8_t bno055_getOperationMode(bno055_opmode_t *mode)
{
	return bno055_read(BNO055_OPR_MODE, mode, 1);
}

int8_t bno055_setOperationMode(bno055_opmode_t mode)
{
	int8_t status = HAL_OK;
	i2c_tx_buffer[0] = BNO055_OPR_MODE;
	i2c_tx_buffer[1] = mode;
	status = bno055_write(i2c_tx_buffer, 2);
	if(status != HAL_OK){return status;}

	status = bno055_poll_transaction();
//	bno055_writeData(BNO055_OPR_MODE, mode);
	if(status != HAL_OK){return status;}

	if (mode == BNO055_OPERATION_MODE_CONFIG)
	{
		bno055_delay(19);
	}
	else
	{
		bno055_delay(7);
	}
	return status;
}

int8_t bno055_setOperationModeConfig()
{
	return bno055_setOperationMode(BNO055_OPERATION_MODE_CONFIG);
}

int8_t bno055_setOperationModeNDOF()
{
	return bno055_setOperationMode(BNO055_OPERATION_MODE_NDOF);
}

int8_t bno055_setExternalCrystalUse(uint8_t state)
{
	int8_t status = HAL_OK;
	status = bno055_setPage(0);
	if(status != HAL_OK){return status;}

	status = bno055_poll_transaction();
	if(status != HAL_OK){return status;}

	status = bno055_read(BNO055_SYS_TRIGGER, i2c_rx_buffer, 1);
	if(status != HAL_OK){return status;}

	status = bno055_poll_transaction();
	if(status != HAL_OK){return status;}

	i2c_rx_buffer[0] |= (state == 1) ? 0x80 : 0x0;
	i2c_tx_buffer[0] = BNO055_SYS_TRIGGER;
	i2c_tx_buffer[1] = i2c_rx_buffer[0];
	status = bno055_write(i2c_tx_buffer, 2);
	if(status != HAL_OK){return status;}

	status = bno055_poll_transaction();
	bno055_delay(700);
	return status;
}

int8_t bno055_enableExternalCrystal() { return bno055_setExternalCrystalUse(1); }
int8_t bno055_disableExternalCrystal() { return bno055_setExternalCrystalUse(0); }

int8_t bno055_poll_transaction()
{
	int8_t status = HAL_BUSY;

	while(status == HAL_BUSY)
	{
		status = monitor_i2c();
	}

	return status;
}

int8_t bno055_reset()
{
	int8_t status = HAL_OK;
	i2c_tx_buffer[0] = BNO055_SYS_TRIGGER;
	i2c_tx_buffer[1] = 0x20;
	status = bno055_write(i2c_tx_buffer, 2);
	if(status != HAL_OK){return status;}

	status = bno055_poll_transaction();
	if(status != HAL_OK){return status;}

	bno055_delay(700);
	return status;
}

int8_t bno055_getTemp(int8_t *temp)
{
	int8_t status = HAL_OK;
	status = bno055_setPage(0);
	if(status != HAL_OK){return status;}

	status = bno055_poll_transaction();
	if(status != HAL_OK){return status;}

	return bno055_read(BNO055_TEMP, (uint8_t*) temp, 1);
}

int8_t bno055_setup()
{
	int8_t status = HAL_OK;
	read_index = 0;
	status = bno055_reset();
	if(status != HAL_OK){return status;}

	status = bno055_read(BNO055_CHIP_ID, i2c_rx_buffer, 1);
	if(status != HAL_OK){return status;}

	status = bno055_poll_transaction();
	if(status != HAL_OK){return status;}

	if(i2c_rx_buffer[0] != BNO055_ID)
	{
		return HAL_ERROR;
	}
	status = bno055_setPage(0);
	if(status != HAL_OK){return status;}

	status = bno055_poll_transaction();
	if(status != HAL_OK){return status;}

	status = bno055_setExternalCrystalUse(0);
	if(status != HAL_OK){return status;}

	// Select BNO055 config mode
	status = bno055_setOperationModeConfig();
	bno055_delay(10);
	return status;
}

int8_t bno055_getSWRevision(int16_t *sw_revision)
{
	int8_t status = HAL_OK;
	status = bno055_setPage(0);
	if(status != HAL_OK){return status;}

	status = bno055_poll_transaction();
	if(status != HAL_OK){return status;}

	status = bno055_read(BNO055_SW_REV_ID_LSB, i2c_rx_buffer, 2);
	if(status != HAL_OK){return status;}

	status = bno055_poll_transaction();
	if(status == HAL_OK)
	{
		(*sw_revision) = (i2c_rx_buffer[1] << 8 | i2c_rx_buffer[0]);
	}
	return status;
//  uint8_t buffer[2];
//  bno055_readData(BNO055_SW_REV_ID_LSB, buffer, 2);
//  return (int16_t)((buffer[1] << 8) | buffer[0]);
}

int8_t bno055_getBootloaderRevision(uint8_t *bootloader_revision)
{
	int8_t status = HAL_OK;
	status = bno055_setPage(0);
	if(status != HAL_OK){return status;}

	status = bno055_poll_transaction();
	if(status != HAL_OK){return status;}

	status = bno055_read(BNO055_BL_REV_ID, bootloader_revision, 1);
	if(status != HAL_OK){return status;}

	status = bno055_poll_transaction();
	return status;
}

int8_t bno055_getSystemStatus(uint8_t *system_status)
{
	int8_t status = HAL_OK;
	status = bno055_setPage(0);
	if(status != HAL_OK){return status;}

	status = bno055_poll_transaction();
	if(status != HAL_OK){return status;}

	status = bno055_read(BNO055_SYS_STATUS, system_status, 1);
	if(status != HAL_OK){return status;}

	status = bno055_poll_transaction();
	return status;
}

int8_t bno055_getSelfTestResult(bno055_self_test_result_t *st_result)
{
	int8_t status = HAL_OK;
	status = bno055_setPage(0);
	if(status != HAL_OK){return status;}

	status = bno055_poll_transaction();
	if(status != HAL_OK){return status;}

	st_result->mcuState = 0;
	st_result->magState = 0;
	st_result->gyrState = 0;
	st_result->accState = 0;
	status = bno055_read(BNO055_ST_RESULT, i2c_rx_buffer, 1);
	if(status != HAL_OK){return status;}

	status = bno055_poll_transaction();

//	bno055_readData(BNO055_ST_RESULT, &tmp, 1);
	st_result->mcuState = (i2c_rx_buffer[0] >> 3) & 0x01;
	st_result->gyrState = (i2c_rx_buffer[0] >> 2) & 0x01;
	st_result->magState = (i2c_rx_buffer[0] >> 1) & 0x01;
	st_result->accState = (i2c_rx_buffer[0] >> 0) & 0x01;
	return status;
}

int8_t bno055_getSystemError(uint8_t *system_error)
{
	int8_t status = HAL_OK;
	status = bno055_setPage(0);
	if(status != HAL_OK){return status;}

	status = bno055_poll_transaction();
	if(status != HAL_OK){return status;}

	status = bno055_read(BNO055_SYS_ERR, system_error, 1);
	if(status != HAL_OK){return status;}

	status = bno055_poll_transaction();
	return status;
}

int8_t bno055_getCalibrationState(bno055_calibration_state_t *cal_state)
{
	int8_t status = HAL_OK;
	status = bno055_setPage(0);
	if(status != HAL_OK){return status;}

	status = bno055_poll_transaction();
	if(status != HAL_OK){return status;}

	cal_state->sys = 0;
	cal_state->gyro = 0;
	cal_state->mag = 0;
	cal_state->accel = 0;
	status = bno055_read(BNO055_CALIB_STAT, i2c_rx_buffer, 1);
	if(status != HAL_OK){return status;}

	status = bno055_poll_transaction();

	cal_state->sys = (i2c_rx_buffer[0] >> 6) & 0x03;
	cal_state->gyro = (i2c_rx_buffer[0] >> 4) & 0x03;
	cal_state->accel = (i2c_rx_buffer[0] >> 2) & 0x03;
	cal_state->mag = (i2c_rx_buffer[0] >> 0) & 0x03;
	return status;
}

int8_t bno055_getCalibrationData(bno055_calibration_data_t *cal_data)
{
	int8_t status = HAL_OK;
	bno055_opmode_t operationMode;
	status = bno055_getOperationMode(&operationMode);
	if(status != HAL_OK){return status;}

	status = bno055_poll_transaction();
	if(status != HAL_OK){return status;}

	status = bno055_setOperationModeConfig();
	if(status != HAL_OK){return status;}

	status = bno055_setPage(0);
	if(status != HAL_OK){return status;}

	status = bno055_poll_transaction();
	if(status != HAL_OK){return status;}

	status = bno055_read(BNO055_ACC_OFFSET_X_LSB, i2c_rx_buffer, 22);
	if(status != HAL_OK){return status;}

	status |= bno055_poll_transaction();
	if(status != HAL_OK){return status;}

//	bno055_readData(BNO055_ACC_OFFSET_X_LSB, buffer, 22);

	cal_data->offset.accel.x = (int16_t)((i2c_rx_buffer[1] << 8) | i2c_rx_buffer[0]);
	cal_data->offset.accel.y = (int16_t)((i2c_rx_buffer[3] << 8) | i2c_rx_buffer[2]);
	cal_data->offset.accel.z = (int16_t)((i2c_rx_buffer[5] << 8) | i2c_rx_buffer[4]);

	cal_data->offset.mag.x = (int16_t)((i2c_rx_buffer[7] << 8) | i2c_rx_buffer[6]);
	cal_data->offset.mag.y = (int16_t)((i2c_rx_buffer[9] << 8) | i2c_rx_buffer[8]);
	cal_data->offset.mag.z = (int16_t)((i2c_rx_buffer[11] << 8) | i2c_rx_buffer[10]);

	cal_data->offset.gyro.x = (int16_t)((i2c_rx_buffer[13] << 8) | i2c_rx_buffer[12]);
	cal_data->offset.gyro.y = (int16_t)((i2c_rx_buffer[15] << 8) | i2c_rx_buffer[14]);
	cal_data->offset.gyro.z = (int16_t)((i2c_rx_buffer[17] << 8) | i2c_rx_buffer[16]);

	cal_data->radius.accel = (int16_t)((i2c_rx_buffer[19] << 8) | i2c_rx_buffer[18]);
	cal_data->radius.mag = (int16_t)((i2c_rx_buffer[21] << 8) | i2c_rx_buffer[20]);

	// Assumes little endian processor
//	memcpy(&calData.offset.accel, i2c_rx_buffer, 6);
//	memcpy(&calData.offset.mag, &i2c_rx_buffer[6], 6);
//	memcpy(&calData.offset.gyro, &i2c_rx_buffer[12], 6);
//	memcpy(&calData.radius.accel, &i2c_rx_buffer[18], 2);
//	memcpy(&calData.radius.mag, &i2c_rx_buffer[20], 2);

	status = bno055_setOperationMode(operationMode);

	return status;
}

int8_t bno055_setCalibrationData(bno055_calibration_data_t *cal_data)
{
	int8_t status = HAL_OK;
	bno055_opmode_t operationMode;
	status = bno055_getOperationMode(&operationMode);
	if(status != HAL_OK){return status;}

	status = bno055_poll_transaction();
	if(status != HAL_OK){return status;}

	status = bno055_setOperationModeConfig();
	if(status != HAL_OK){return status;}


	status = bno055_setPage(0);
	if(status != HAL_OK){return status;}

	status = bno055_poll_transaction();
	if(status != HAL_OK){return status;}


	i2c_tx_buffer[0] = BNO055_ACC_OFFSET_X_LSB;
	i2c_tx_buffer[1] = (cal_data->offset.accel.x >> 0) & 0xFF;
	i2c_tx_buffer[2] = BNO055_ACC_OFFSET_X_MSB;
	i2c_tx_buffer[3] = (cal_data->offset.accel.x >> 8) & 0xFF;
	i2c_tx_buffer[4] = BNO055_ACC_OFFSET_Y_LSB;
	i2c_tx_buffer[5] = (cal_data->offset.accel.y >> 0) & 0xFF;
	i2c_tx_buffer[6] = BNO055_ACC_OFFSET_Y_MSB;
	i2c_tx_buffer[7] = (cal_data->offset.accel.y >> 8) & 0xFF;
	i2c_tx_buffer[8] = BNO055_ACC_OFFSET_Z_LSB;
	i2c_tx_buffer[9] = (cal_data->offset.accel.z >> 0) & 0xFF;
	i2c_tx_buffer[10] = BNO055_ACC_OFFSET_Z_MSB;
	i2c_tx_buffer[11] = (cal_data->offset.accel.z >> 8) & 0xFF;

	i2c_tx_buffer[12] = BNO055_MAG_OFFSET_X_LSB;
	i2c_tx_buffer[13] = (cal_data->offset.mag.x >> 0) & 0xFF;
	i2c_tx_buffer[14] = BNO055_MAG_OFFSET_X_MSB;
	i2c_tx_buffer[15] = (cal_data->offset.mag.x >> 8) & 0xFF;
	i2c_tx_buffer[16] = BNO055_MAG_OFFSET_Y_LSB;
	i2c_tx_buffer[17] = (cal_data->offset.mag.y >> 0) & 0xFF;
	i2c_tx_buffer[18] = BNO055_MAG_OFFSET_Y_MSB;
	i2c_tx_buffer[19] = (cal_data->offset.mag.y >> 8) & 0xFF;
	i2c_tx_buffer[20] = BNO055_MAG_OFFSET_Z_LSB;
	i2c_tx_buffer[21] = (cal_data->offset.mag.z >> 0) & 0xFF;
	i2c_tx_buffer[22] = BNO055_MAG_OFFSET_Z_MSB;
	i2c_tx_buffer[23] = (cal_data->offset.mag.z >> 8) & 0xFF;

	i2c_tx_buffer[24] = BNO055_GYR_OFFSET_X_LSB;
	i2c_tx_buffer[25] = (cal_data->offset.gyro.x >> 0) & 0xFF;
	i2c_tx_buffer[26] = BNO055_GYR_OFFSET_X_MSB;
	i2c_tx_buffer[27] = (cal_data->offset.gyro.x >> 8) & 0xFF;
	i2c_tx_buffer[28] = BNO055_GYR_OFFSET_Y_LSB;
	i2c_tx_buffer[29] = (cal_data->offset.gyro.y >> 0) & 0xFF;
	i2c_tx_buffer[30] = BNO055_GYR_OFFSET_Y_MSB;
	i2c_tx_buffer[31] = (cal_data->offset.gyro.y >> 8) & 0xFF;
	i2c_tx_buffer[32] = BNO055_GYR_OFFSET_Z_LSB;
	i2c_tx_buffer[33] = (cal_data->offset.gyro.z >> 0) & 0xFF;
	i2c_tx_buffer[34] = BNO055_GYR_OFFSET_Z_MSB;
	i2c_tx_buffer[35] = (cal_data->offset.gyro.z >> 8) & 0xFF;

	i2c_tx_buffer[36] = BNO055_ACC_RADIUS_LSB;
	i2c_tx_buffer[37] = (cal_data->radius.accel >> 0) & 0xFF;
	i2c_tx_buffer[38] = BNO055_ACC_RADIUS_MSB;
	i2c_tx_buffer[39] = (cal_data->radius.accel >> 8) & 0xFF;
	i2c_tx_buffer[40] = BNO055_MAG_RADIUS_LSB;
	i2c_tx_buffer[41] = (cal_data->radius.mag >> 0) & 0xFF;
	i2c_tx_buffer[42] = BNO055_MAG_RADIUS_MSB;
	i2c_tx_buffer[43] = (cal_data->radius.mag >> 8) & 0xFF;

	for(uint8_t i = 0; i < 22; i++)
	{
		status = bno055_write(&i2c_tx_buffer[i * 2], 2);
		if(status != HAL_OK){return status;}

		status = bno055_poll_transaction();
		if(status != HAL_OK){return status;}
	}


//	for(uint8_t i = 0; i < 3; i++)
//	{
//		for(uint8_t j = 0; j < 3; j++)
//		{
//			i2c_tx_buffer[(i * 6) + (j * 2)] = BNO055_ACC_OFFSET_X_LSB + (i * 6) + j;
//			i2c_tx_buffer[(i * 6) + j] = cal_data->offset.accel
//		}
//	}
//	memcpy(i2c_tx_buffer, &calData.offset.accel, 6);
//	memcpy(&i2c_tx_buffer[6], &calData.offset.mag, 6);
//	memcpy(&i2c_tx_buffer[12], &calData.offset.gyro, 6);
//	memcpy(&i2c_tx_buffer[18], &calData.radius.accel, 2);
//	memcpy(&i2c_tx_buffer[20], &calData.radius.mag, 2);
//
//	for (uint8_t i=0; i < 22; i++)
//	{
//		bno055_writeData(BNO055_ACC_OFFSET_X_LSB+i, i2c_tx_buffer[i]);
//	}

	status = bno055_setOperationMode(operationMode);
	return status;
}

void bno055_get_all_values()
{
	uint8_t* buffer = (uint8_t*)(&holding_register_database[12]);
	for(uint8_t i = 0; i < NUM_VECTORS; i++)
	{
		bno055_read(mem_read_map[i].reg, &buffer[6 * i], mem_read_map[i].reg_len);
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

int8_t bno055_queue_transaction()
{
	uint8_t status = HAL_OK;
	uint8_t* buffer = (uint8_t*)(&holding_register_database[12]);
	i2c_rx_int = 0;
	i2c_rx_time = HAL_GetTick();
	status = HAL_I2C_Mem_Read_DMA(&hi2c1, bno055_address, mem_read_map[read_index].reg,
			I2C_MEMADD_SIZE_8BIT, &buffer[6 * read_index], mem_read_map[read_index].reg_len);
	__HAL_DMA_DISABLE_IT(&hdma_i2c1_rx, DMA_IT_HT);

	if(read_index == NUM_VECTORS - 1)
	{
		// Reset the read index
		read_index = 0;
		// Switch to the other IMU
		bno055_address = (bno055_address == (BNO055_I2C_ADDR_LO << 1))? (BNO055_I2C_ADDR_HI << 1): (BNO055_I2C_ADDR_LO << 1);
	}
	else
	{
		// Increment the read index to read the next vector on the next run
		read_index = read_index + 1;
	}
	return status;
}

int8_t bno055_setAxisMap(bno055_axis_map_t *axis)
{
	int8_t status = HAL_OK;

	i2c_tx_buffer[0] = BNO055_AXIS_MAP_CONFIG;
	i2c_tx_buffer[1] = (axis->z << 4) | (axis->y << 2) | (axis->x);

	status = bno055_write(i2c_tx_buffer, 2);
	if(status != HAL_OK){return status;}
	status = bno055_poll_transaction();

	i2c_tx_buffer[0] = BNO055_AXIS_MAP_SIGN;
	i2c_tx_buffer[1] = (axis->x_sign << 2) | (axis->y_sign << 1) | (axis->z_sign);
	status = bno055_write(i2c_tx_buffer, 2);
	return status;
//	i2c_rx_buffer[0] = BNO055_AXIS_MAP_CONFIG;
//	i2c_rx_buffer[1] = (axis.z << 4) | (axis.y << 2) | (axis.x);
//	status = bno055_write(i2c_rx_buffer, 2);
//	status |= bno055_poll_transaction();
//
//	i2c_rx_buffer[0] = BNO055_AXIS_MAP_SIGN;
//	i2c_rx_buffer[1] = (axis.x_sign << 2) | (axis.y_sign << 1) | (axis.z_sign);
//	status = bno055_write(i2c_rx_buffer, 2);
//	status |= bno055_poll_transaction();
}

int8_t monitor_i2c()
{
	int8_t status = HAL_OK;

	// I2C error handling
	if(i2c_err_int)
	{
		i2c_err_int = 0;
		status = i2c_reset();
		if(status != HAL_OK)
		{
			return status;
		}
		return handle_i2c_error(I2C_ERROR);
	}

	// TX timeout handling
	if(!i2c_tx_int)
	{
		if(HAL_GetTick() - i2c_tx_time >= 100)
		{
			i2c_tx_int = 1;
			return handle_i2c_error(I2C_TX_TIMEOUT);
		}
		status = HAL_BUSY;
	}

	// RX timeout handling
	if(!i2c_rx_int)
	{
		if(HAL_GetTick() - i2c_rx_time >= 100)
		{
			i2c_rx_int = 1;
			return handle_i2c_error(I2C_RX_TIMEOUT);
		}
		status = HAL_BUSY;
	}

	return status;
}

int8_t bno055_write(uint8_t *data, uint8_t len)
{
	int8_t status = HAL_OK;
	i2c_tx_int = 0;
	i2c_tx_time = HAL_GetTick();
	status = HAL_I2C_Master_Transmit_DMA(&hi2c1, bno055_address, data, len);
	__HAL_DMA_DISABLE_IT(&hdma_i2c1_tx, DMA_IT_HT);

	return status;
}

int8_t bno055_read(uint8_t reg, uint8_t *data, uint8_t len)
{
	int8_t status = HAL_OK;
	i2c_tx_int = 0;
	i2c_tx_time = HAL_GetTick();
	status = HAL_I2C_Master_Transmit_DMA(&hi2c1, bno055_address, &reg, 1);
	__HAL_DMA_DISABLE_IT(&hdma_i2c1_tx, DMA_IT_HT);
	if(status != HAL_OK){return status;}
	status = bno055_poll_transaction();
	if(status != HAL_OK){return status;}

	i2c_rx_int = 0;
	i2c_rx_time = HAL_GetTick();
	status = HAL_I2C_Master_Receive_DMA(&hi2c1, bno055_address, data, len);
	__HAL_DMA_DISABLE_IT(&hdma_i2c1_rx, DMA_IT_HT);

	return status;
}

int8_t i2c_reset()
{
	int8_t status = HAL_OK;
	status = bno055_poll_transaction();
	status |= HAL_I2C_DeInit(&hi2c1);
	__I2C1_FORCE_RESET();
	HAL_Delay(100);
	__I2C1_RELEASE_RESET();
	status = HAL_I2C_Init(&hi2c1);
	status |= HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE);
	status |= HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0);
	if(status != HAL_OK)
	{
		return handle_i2c_error(I2C_FATAL_ERROR);
	}
	return status;
}

int8_t handle_i2c_error(int8_t error_code)
{
	holding_register_database[I2C_ERRORS] |= 1U << (error_code - I2C_TX_TIMEOUT);
	return error_code;
}
