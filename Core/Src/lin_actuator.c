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


HAL_StatusTypeDef init_lin_actuator()
{
	HAL_StatusTypeDef status = HAL_OK;

	rx_data[0] = 0;
	rx_data[1] = 0;

	// Initialize the PWM signal
	TIM1->CCR1 = 0;
	status = HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	if(status != HAL_OK)
	{
		return status;
	}

	// Test communication by reading the Device ID
	tx_data[0] = (DEVICE_ID | READ_MASK);
	tx_data[1] = DUMMY_DATA;
	HAL_GPIO_WritePin(Actuator_CS_GPIO_Port, Actuator_CS_Pin, GPIO_PIN_RESET);
	status = HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, 100);
	HAL_GPIO_WritePin(Actuator_CS_GPIO_Port, Actuator_CS_Pin, GPIO_PIN_SET);
	if(status != HAL_OK)
	{
		return status;
	}

	if(rx_data[1] == DRV8244_Q1_ID)
	{
		// Unlock the Config Registers. Refer to section 8.6.1.5
		tx_data[0] = COMMAND; // WRITE MASK = 0
		tx_data[1] = REG_UNLOCK;
		status = HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, 100);
		if(status != HAL_OK)
		{
			return status;
		}

		// Configure independent mode
		tx_data[0] = CONFIG_3;
		tx_data[1] = TOFF_40U | S_MODE_INDEPENDENT;
		status = HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, 100);
		if(status != HAL_OK)
		{
			return status;
		}

		// Lock the Config Registers. Refer to section 8.6.1.5
		tx_data[0] = COMMAND; // WRITE MASK = 0
		tx_data[1] = REG_LOCK;
		status = HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, 100);
		if(status != HAL_OK)
		{
			return status;
		}
	}
	else
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

void actuate_pwm(uint8_t actuator, uint16_t current, uint16_t target)
{
	if(selected_actuator != actuator)
	{
		if(drv_on)
		{
			// Unlock the SPI_IN register. Refer to section 8.6.1.5
			tx_data[0] = COMMAND; // WRITE MASK = 0
			tx_data[1] = SPI_IN_UNLOCK;
			HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, 100);

			// Turn off the DRV8244
			tx_data[0] = SPI_IN; // WRITE MASK = 0
			tx_data[1] = 0;
			HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, 100);

			// Lock the SPI_IN register. Refer to section 8.6.1.5
			tx_data[0] = COMMAND; // WRITE MASK = 0
			tx_data[1] = SPI_IN_LOCK;
			HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, 100);
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
			tx_data[0] = COMMAND; // WRITE MASK = 0
			tx_data[1] = SPI_IN_UNLOCK;
			HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, 100);

			// Turn on the DRV8244
			if(target > current)
			{
				// Forwards
				tx_data[0] = SPI_IN; // WRITE MASK = 0
				tx_data[1] = S_EN_IN1;
				HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, 100);
			}
			else
			{
				// Backwards
				tx_data[0] = SPI_IN; // WRITE MASK = 0
				tx_data[1] = S_PH_IN2;
				HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, 100);
			}
			// Lock the SPI_IN register. Refer to section 8.6.1.5
			tx_data[0] = COMMAND; // WRITE MASK = 0
			tx_data[1] = SPI_IN_LOCK;
			HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, 100);
			drv_on = 1;
			selected_actuator = actuator;
		}
	}
}

void actuate_spi(uint8_t actuator, uint16_t current, uint16_t target)
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

