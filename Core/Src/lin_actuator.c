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


uint8_t selected_actuator = NUM_ACTUATORS;
uint8_t drv_on = 0;
uint32_t time_ms = 0;
uint8_t drv_shutoff = 0;
uint8_t pin_off[NUM_ACTUATORS];

extern uint16_t pin_map[NUM_ACTUATORS];
extern GPIO_TypeDef* port_map[NUM_ACTUATORS];
extern pid_t pid_constraints;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim1;

uint8_t tx_data[2];
uint8_t rx_data[2];

float pid_step(pid_t *pid, float measurement, float setpoint);

int8_t actuate(uint8_t actuator, uint16_t current, uint16_t target, uint32_t* actuator_time)
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
			TIM1->CCR1 = 0;
			spi_in.s_drv_off = 1;
			spi_in.s_drv_off2 = 1;
			status = DRV_SetSpiIn(&spi_in);
			if(status != HAL_OK){return status;}
			drv_on = 0;

			// Lock the SPI_IN register. Refer to section 8.6.1.5
			command.spi_in_lock = SPI_IN_LOCK;
			status = DRV_SetCommand(&command);
			if(status != HAL_OK){return status;}

			time_ms = HAL_GetTick();
		}
		if(HAL_GetTick() - time_ms >= ACTUATOR_TRANSIENT_DELAY)
		{
			if(!pin_off[selected_actuator])
			{
				pin_off[selected_actuator] = 1;
				// Deactivate the old actuator
				HAL_GPIO_WritePin(port_map[selected_actuator], pin_map[selected_actuator], GPIO_PIN_RESET);
			}

			if(!drv_shutoff)
			{
				status = DRV_Activate(actuator, current, target);
				if(status != HAL_OK){return status;}
				(*actuator_time) = HAL_GetTick();
			}
			selected_actuator = actuator;
		}
		return status;
	}
	if(!drv_shutoff)
	{
		TIM1->CCR1 = (uint8_t) pid_step(&pid_constraints, (float)current, (float)target);
	}
	return HAL_OK;
}

void DRV_Shutoff()
{
	drv_shutoff = 1;
}

uint8_t DRV_GetShutoff()
{
	return drv_shutoff;
}

int8_t DRV_Activate(uint8_t actuator, uint16_t current, uint16_t target)
{
	int8_t status = HAL_OK;
	drv_command_t command = DEFAULT_DRV_COMMAND;
	drv_spi_in_t spi_in = DEFAULT_DRV_SPI_IN;

	drv_shutoff = 0;

	// Activate the new actuator
	HAL_GPIO_WritePin(port_map[actuator], pin_map[actuator], GPIO_PIN_SET);
	pin_off[actuator] = 0;

	// Unlock the SPI_IN register. Refer to section 8.6.1.5
	command.spi_in_lock = SPI_IN_UNLOCK;
	status = DRV_SetCommand(&command);
	if(status != HAL_OK){return status;}

	// Turn on the DRV8244
	spi_in.s_en_in1 = 0;
	spi_in.s_drv_off = 0;
	spi_in.s_drv_off2 = 0;
	if(target > current)
	{
		// Extend
		spi_in.s_ph_in2 = 1;
	}
	else
	{
		// Retract
		spi_in.s_ph_in2 = 0;
	}
	status = DRV_SetSpiIn(&spi_in);
	if(status != HAL_OK){return status;}

	// Lock the SPI_IN register. Refer to section 8.6.1.5
	command.spi_in_lock = SPI_IN_LOCK;
	status = DRV_SetCommand(&command);

	// Set the PWM Frequency
	TIM1->CCR1 = (uint8_t) pid_step(&pid_constraints, (float)current, (float)target);

	drv_on = 1;
	selected_actuator = actuator;

	return status;
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
	pin_off[0] = 1;
	pin_off[1] = 1;
	pin_off[2] = 1;

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
		command.spi_in_lock = SPI_IN_LOCK;
		status = DRV_SetCommand(&command);
		if(status != HAL_OK){return status;}

		drv_config_1_t w_config_1;
		w_config_1.raw_data = 0;
		w_config_1.en_ola = 1;
		w_config_1.ocp_retry = 1;
		w_config_1.ola_retry = 1;
		w_config_1.ssc_dis = 1;
		w_config_1.tsd_retry = 1;
		w_config_1.vmov_retry = 1;
		w_config_1.vmov_sel = VMOV_SEL_18V;
		status = DRV_SetConfig1(&w_config_1);
		if(status != HAL_OK){return status;}

		drv_config_2_t w_config_2;
		w_config_2.raw_data = 0;
		w_config_2.pwm_extend = 0;
		w_config_2.s_diag = S_DIAG_MODE_3;
		w_config_2.s_itrip = S_ITRIP_DISABLE;
		status = DRV_SetConfig2(&w_config_2);
		if(status != HAL_OK){return status;}

		// Configure PH/EN Mode
		drv_config_3_t w_config_3;
		w_config_3.raw_data = 0;
		w_config_3.s_mode = S_MODE_PH_EN;
		w_config_3.toff = TOFF_40US;
		w_config_3.s_sr = 0;
		status = DRV_SetConfig3(&w_config_3);
		if(status != HAL_OK){return status;}

		// Configure the DRV to allow the SPI bit configuration to control the h-bridge
		drv_config_4_t w_config_4;
		w_config_4.raw_data = 0;
		w_config_4.drvoff_sel = DRVOFF_SEL_OR;
		w_config_4.en_in1_sel = EN_IN1_SEL_OR;
		w_config_4.ph_in2_sel = PH_IN2_SEL_OR;
		w_config_4.tocp_sel = TOCP_SEL_6US;
		w_config_4.ocp_sel = OCP_SEL_100;
		status = DRV_SetConfig4(&w_config_4);
		if(status != HAL_OK){return status;}

		// Lock the Config Registers. Refer to section 8.6.1.5
		command.reg_lock = REG_LOCK;
		status = DRV_SetCommand(&command);
		if(status != HAL_OK){return status;}

		// Sanity check the configuration
		drv_config_1_t r_config_1;
		drv_config_2_t r_config_2;
		drv_config_3_t r_config_3;
		drv_config_4_t r_config_4;

		status = DRV_GetConfig1(&r_config_1);
		status |= DRV_GetConfig2(&r_config_2);
		status |= DRV_GetConfig3(&r_config_3);
		status |= DRV_GetConfig4(&r_config_4);
		if(status != HAL_OK){return status;}

		if(r_config_1.raw_data != w_config_1.raw_data){return HAL_ERROR;}
		if(r_config_2.raw_data != w_config_2.raw_data){return HAL_ERROR;}
		if(r_config_3.raw_data != w_config_3.raw_data){return HAL_ERROR;}
		if(r_config_4.raw_data != w_config_4.raw_data){return HAL_ERROR;}
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
