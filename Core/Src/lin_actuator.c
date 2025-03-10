/*
 * lin_actuator.c
 *
 *  Created on: Dec 20, 2024
 *      Author: Victor Kalenda
 *
 *      References
 *      Pid control algorithm: https://simonebertonilab.com/pid-controller-in-c/
 */

#include "main.h"
#include "lin_actuator.h"


uint8_t selected_actuator = 0;
uint8_t drv_on = 0;
uint32_t time_ms = 0;

extern uint16_t pin_map[NUM_ACTUATORS];
extern GPIO_TypeDef* port_map[NUM_ACTUATORS];
extern pid_t pid_constraints;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim1;

uint8_t tx_data[2];
uint8_t rx_data[2];

float pid_step(pid_t *pid, float measurement, float setpoint);

int8_t actuate_spi(uint8_t actuator, uint16_t current, uint16_t target)
{
	if(selected_actuator != actuator)
	{
		int8_t status = HAL_OK;
		drv_command_t command = DEFAULT_DRV_COMMAND;
		drv_spi_in_t spi_in = DEFAULT_DRV_SPI_IN;
		if(drv_on)
		{
			// Unlock the SPI_IN register. Refer to section 8.6.1.5
			command.spi_in_lock = SPI_IN_UNLOCK;
			status = DRV_SetCommand(&command);
			if(status != HAL_OK){return status;}

			// Turn off the DRV8244
			spi_in.s_drv_off = 0;
			spi_in.s_drv_off2 = 0;
			status = DRV_SetSpiIn(&spi_in);
			if(status != HAL_OK){return status;}

			// Lock the SPI_IN register. Refer to section 8.6.1.5
			command.spi_in_lock = SPI_IN_LOCK;
			status = DRV_SetCommand(&command);
			if(status != HAL_OK){return status;}

			drv_on = 0;
			time_ms = HAL_GetTick();
		}
		if(HAL_GetTick() - time_ms >= ACTUATOR_TRANSIENT_DELAY)
		{
			// Deactivate the old actuator
			HAL_GPIO_TogglePin(GPIOB, pin_map[selected_actuator]);
			// Activate the new actuator
			HAL_GPIO_TogglePin(GPIOB, pin_map[actuator]);

			// Unlock the SPI_IN register. Refer to section 8.6.1.5
			command.spi_in_lock = SPI_IN_UNLOCK;
			status = DRV_SetCommand(&command);
			if(status != HAL_OK){return status;}

			// Turn on the DRV8244
			if(target > current)
			{
				// Forwards
				spi_in.s_en_in1 = 1;
				status = DRV_SetSpiIn(&spi_in);
			}
			else
			{
				// Backwards
				spi_in.s_ph_in2 = 1;
				status = DRV_SetSpiIn(&spi_in);
			}
			if(status != HAL_OK){return status;}

			// Lock the SPI_IN register. Refer to section 8.6.1.5
			command.spi_in_lock = SPI_IN_LOCK;
			status = DRV_SetCommand(&command);

			drv_on = 1;
			selected_actuator = actuator;
		}
		return status;
	}
	return HAL_OK;
}

void actuate_pwm(uint8_t actuator, uint16_t current, uint16_t target)
{
	if(selected_actuator != actuator)
	{
		// Wait an amount of time for electrical safety
		if(drv_on)
		{
			// Shut off the PWM pin
			time_ms = HAL_GetTick();
			drv_on = 0;
		}
		if(HAL_GetTick() - time_ms >= ACTUATOR_TRANSIENT_DELAY)
		{

			// Deactivate the old actuator
			HAL_GPIO_TogglePin(GPIOB, pin_map[selected_actuator]);

			// Activate the new actuator
			HAL_GPIO_TogglePin(GPIOB, pin_map[actuator]);

			// Set the duty cycle
			TIM1->CCR1 = pid_step(&pid_constraints, current, target);

			drv_on = 1;
			// Exit this code section by switching the actuator
			selected_actuator = actuator;
		}

	}
	else
	{
		// Just set the duty cycle
		TIM1->CCR1 = pid_step(&pid_constraints, current, target);
	}

}

float pid_step(pid_t *pid, float measurement, float setpoint)
{
    /* This function implements a PID controller.
     *
     * Inputs:
     *   measurement: current measurement of the process variable
     *   setpoint: desired value of the process variable
     *   pid: a pointer to a PID struct containing the controller parameters
     *
     * Returns:
     *   command_sat: the control output of the PID controller (saturated based on max. min, max_rate)
     */

    float err;
    float command;
    float command_sat;
    float deriv_filt;

    /* Error calculation */
    err = setpoint - measurement;

    /* Integral term calculation - including anti-windup */
    pid->integral += pid->Ki*err*pid->T + pid->Kaw*(pid->command_sat_prev - pid->command_prev)*pid->T;

    /* Derivative term calculation using filtered derivative method */
    deriv_filt = (err - pid->err_prev + pid->T_C*pid->deriv_prev)/(pid->T + pid->T_C);
    pid->err_prev = err;
    pid->deriv_prev = deriv_filt;

    /* Summing the 3 terms */
    command = pid->Kp*err + pid->integral + pid->Kd*deriv_filt;

    /* Remember command at previous step */
    pid->command_prev = command;

    /* Saturate command */
    if (command > pid->max)
    {
        command_sat = pid->max;
    }
    else if (command < pid->min)
    {
        command_sat = pid->min;
    }
    else
    {
        command_sat = command;
    }

    /* Apply rate limiter */
    if (command_sat > pid->command_sat_prev + pid->max_rate*pid->T)
    {
        command_sat = pid->command_sat_prev + pid->max_rate*pid->T;
    }
    else if (command_sat < pid->command_sat_prev - pid->max_rate*pid->T)
    {
        command_sat = pid->command_sat_prev - pid->max_rate*pid->T;
    }
    else
    {
        /* No action */
    }

    /* Remember saturated command at previous step */
    pid->command_sat_prev = command_sat;

    return command_sat;
}

int8_t DRV_Init(uint8_t device_id)
{
	int8_t status = HAL_OK;

	// Initialize the PWM signal
	TIM1->CCR1 = 0;
	status = HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	if(status != HAL_OK){return status;}

	// Test communication by reading the Device ID
	uint8_t dev_id = 0;
	status = DRV_GetDeviceId(&dev_id);
	if(status != HAL_OK){return status;}

	if(dev_id == device_id)
	{
		// Unlock the Config Registers, clear the fault register. Refer to section 8.6.1.5
		drv_command_t command;
		command.clr_flt = 1;
		command.reg_lock = REG_UNLOCK;
		status = DRV_SetCommand(&command);
		if(status != HAL_OK){return status;}

		// Configure independent mode
		drv_config_3_t config_3;
		config_3.s_mode = S_MODE_INDEPENDENT;
		config_3.toff = TOFF_40US;
		status = DRV_SetConfig3(&config_3);
		if(status != HAL_OK){return status;}

		// Lock the Config Registers. Refer to section 8.6.1.5
		command.reg_lock = REG_LOCK;
		status = DRV_SetCommand(&command);
	}
	else
	{
		return HAL_ERROR;
	}
	return status;
}

int8_t DRV_GetDeviceId(uint8_t* device_id)
{
	int8_t status = HAL_OK;
	tx_data[0] = (DEVICE_ID | READ_MASK);
	tx_data[1] = 0;
	rx_data[0] = 0;
	rx_data[0] = 0;

	HAL_GPIO_WritePin(Actuator_CS_GPIO_Port, Actuator_CS_Pin, GPIO_PIN_RESET);
	status = HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, 100);
	HAL_GPIO_WritePin(Actuator_CS_GPIO_Port, Actuator_CS_Pin, GPIO_PIN_SET);
	if(status != HAL_OK){return status;}

	(*device_id) = rx_data[1];
	return status;
}

int8_t DRV_GetFaultSummary(drv_fault_summary_t* fault_summary)
{
	int8_t status = HAL_OK;
	tx_data[0] = (FAULT_SUMMARY | READ_MASK);
	tx_data[1] = 0;
	rx_data[0] = 0;
	rx_data[1] = 0;

	HAL_GPIO_WritePin(Actuator_CS_GPIO_Port, Actuator_CS_Pin, GPIO_PIN_RESET);
	status = HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, 100);
	HAL_GPIO_WritePin(Actuator_CS_GPIO_Port, Actuator_CS_Pin, GPIO_PIN_SET);
	if(status != HAL_OK){return status;}

	fault_summary->raw_data = rx_data[1];
	return status;
}

int8_t DRV_GetStatus1(drv_status_1_t* status_1)
{
	int8_t status = HAL_OK;
	tx_data[0] = (STATUS_1 | READ_MASK);
	tx_data[1] = 0;
	rx_data[0] = 0;
	rx_data[1] = 0;

	HAL_GPIO_WritePin(Actuator_CS_GPIO_Port, Actuator_CS_Pin, GPIO_PIN_RESET);
	status = HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, 100);
	HAL_GPIO_WritePin(Actuator_CS_GPIO_Port, Actuator_CS_Pin, GPIO_PIN_SET);
	if(status != HAL_OK){return status;}

	status_1->raw_data = rx_data[1];
	return status;
}

int8_t DRV_GetStatus2(drv_status_2_t* status_2)
{
	int8_t status = HAL_OK;
	tx_data[0] = (STATUS_2 | READ_MASK);
	tx_data[1] = 0;
	rx_data[0] = 0;
	rx_data[1] = 0;

	HAL_GPIO_WritePin(Actuator_CS_GPIO_Port, Actuator_CS_Pin, GPIO_PIN_RESET);
	status = HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, 100);
	HAL_GPIO_WritePin(Actuator_CS_GPIO_Port, Actuator_CS_Pin, GPIO_PIN_SET);
	if(status != HAL_OK){return status;}

	status_2->raw_data = rx_data[1];
	return status;
}

int8_t DRV_GetCommand(drv_command_t* command)
{
	int8_t status = HAL_OK;
	tx_data[0] = (COMMAND | READ_MASK);
	tx_data[1] = 0;
	rx_data[0] = 0;
	rx_data[1] = 0;

	HAL_GPIO_WritePin(Actuator_CS_GPIO_Port, Actuator_CS_Pin, GPIO_PIN_RESET);
	status = HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, 100);
	HAL_GPIO_WritePin(Actuator_CS_GPIO_Port, Actuator_CS_Pin, GPIO_PIN_SET);
	if(status != HAL_OK){return status;}

	command->raw_data = rx_data[1];
	return status;
}

int8_t DRV_SetCommand(drv_command_t* command)
{
	int8_t status = HAL_OK;

	tx_data[0] = COMMAND;
	tx_data[1] = command->raw_data;

	HAL_GPIO_WritePin(Actuator_CS_GPIO_Port, Actuator_CS_Pin, GPIO_PIN_RESET);
	status = HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, 100);
	HAL_GPIO_WritePin(Actuator_CS_GPIO_Port, Actuator_CS_Pin, GPIO_PIN_SET);
	return status;
}

int8_t DRV_GetSpiIn(drv_spi_in_t* spi_in)
{
	int8_t status = HAL_OK;

	tx_data[0] = (SPI_IN | READ_MASK);
	tx_data[1] = 0;
	rx_data[0] = 0;
	rx_data[1] = 0;

	HAL_GPIO_WritePin(Actuator_CS_GPIO_Port, Actuator_CS_Pin, GPIO_PIN_RESET);
	status = HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, 100);
	HAL_GPIO_WritePin(Actuator_CS_GPIO_Port, Actuator_CS_Pin, GPIO_PIN_SET);

	spi_in->raw_data = rx_data[1];
	return status;
}

int8_t DRV_SetSpiIn(drv_spi_in_t* spi_in)
{
	int8_t status = HAL_OK;

	tx_data[0] = SPI_IN;
	tx_data[1] = spi_in->raw_data;

	HAL_GPIO_WritePin(Actuator_CS_GPIO_Port, Actuator_CS_Pin, GPIO_PIN_RESET);
	status = HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, 100);
	HAL_GPIO_WritePin(Actuator_CS_GPIO_Port, Actuator_CS_Pin, GPIO_PIN_SET);
	return status;
}

int8_t DRV_GetConfig1(drv_config_1_t* config_1)
{
	int8_t status = HAL_OK;

	tx_data[0] = (CONFIG_1 | READ_MASK);
	tx_data[1] = 0;
	rx_data[0] = 0;
	rx_data[1] = 0;

	HAL_GPIO_WritePin(Actuator_CS_GPIO_Port, Actuator_CS_Pin, GPIO_PIN_RESET);
	status = HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, 100);
	HAL_GPIO_WritePin(Actuator_CS_GPIO_Port, Actuator_CS_Pin, GPIO_PIN_SET);

	config_1->raw_data = rx_data[1];
	return status;
}

int8_t DRV_SetConfig1(drv_config_1_t* config_1)
{
	int8_t status = HAL_OK;

	tx_data[0] = CONFIG_1;
	tx_data[1] = config_1->raw_data;

	HAL_GPIO_WritePin(Actuator_CS_GPIO_Port, Actuator_CS_Pin, GPIO_PIN_RESET);
	status = HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, 100);
	HAL_GPIO_WritePin(Actuator_CS_GPIO_Port, Actuator_CS_Pin, GPIO_PIN_SET);
	return status;
}

int8_t DRV_GetConfig2(drv_config_2_t* config_2)
{
	int8_t status = HAL_OK;

	tx_data[0] = (CONFIG_2 | READ_MASK);
	tx_data[1] = 0;
	rx_data[0] = 0;
	rx_data[1] = 0;

	HAL_GPIO_WritePin(Actuator_CS_GPIO_Port, Actuator_CS_Pin, GPIO_PIN_RESET);
	status = HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, 100);
	HAL_GPIO_WritePin(Actuator_CS_GPIO_Port, Actuator_CS_Pin, GPIO_PIN_SET);

	config_2->raw_data = rx_data[1];
	return status;
}

int8_t DRV_SetConfig2(drv_config_2_t* config_2)
{
	int8_t status = HAL_OK;

	tx_data[0] = CONFIG_2;
	tx_data[1] = config_2->raw_data;

	HAL_GPIO_WritePin(Actuator_CS_GPIO_Port, Actuator_CS_Pin, GPIO_PIN_RESET);
	status = HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, 100);
	HAL_GPIO_WritePin(Actuator_CS_GPIO_Port, Actuator_CS_Pin, GPIO_PIN_SET);
	return status;
}

int8_t DRV_GetConfig3(drv_config_3_t* config_3)
{
	int8_t status = HAL_OK;

	tx_data[0] = (CONFIG_3 | READ_MASK);
	tx_data[1] = 0;
	rx_data[0] = 0;
	rx_data[1] = 0;

	HAL_GPIO_WritePin(Actuator_CS_GPIO_Port, Actuator_CS_Pin, GPIO_PIN_RESET);
	status = HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, 100);
	HAL_GPIO_WritePin(Actuator_CS_GPIO_Port, Actuator_CS_Pin, GPIO_PIN_SET);

	config_3->raw_data = rx_data[1];
	return status;
}

int8_t DRV_SetConfig3(drv_config_3_t* config_3)
{
	int8_t status = HAL_OK;

	tx_data[0] = CONFIG_3;
	tx_data[1] = config_3->raw_data;

	HAL_GPIO_WritePin(Actuator_CS_GPIO_Port, Actuator_CS_Pin, GPIO_PIN_RESET);
	status = HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, 100);
	HAL_GPIO_WritePin(Actuator_CS_GPIO_Port, Actuator_CS_Pin, GPIO_PIN_SET);
	return status;
}

int8_t DRV_GetConfig4(drv_config_4_t* config_4)
{
	int8_t status = HAL_OK;

	tx_data[0] = (CONFIG_4 | READ_MASK);
	tx_data[1] = 0;
	rx_data[0] = 0;
	rx_data[1] = 0;

	HAL_GPIO_WritePin(Actuator_CS_GPIO_Port, Actuator_CS_Pin, GPIO_PIN_RESET);
	status = HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, 100);
	HAL_GPIO_WritePin(Actuator_CS_GPIO_Port, Actuator_CS_Pin, GPIO_PIN_SET);

	config_4->raw_data = rx_data[1];
	return status;
}

int8_t DRV_SetConfig4(drv_config_4_t* config_4)
{
	int8_t status = HAL_OK;

	tx_data[0] = CONFIG_4;
	tx_data[1] = config_4->raw_data;

	HAL_GPIO_WritePin(Actuator_CS_GPIO_Port, Actuator_CS_Pin, GPIO_PIN_RESET);
	status = HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, 100);
	HAL_GPIO_WritePin(Actuator_CS_GPIO_Port, Actuator_CS_Pin, GPIO_PIN_SET);
	return status;
}
