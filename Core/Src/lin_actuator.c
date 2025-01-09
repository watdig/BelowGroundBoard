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

extern SPI_HandleTypeDef hspi1;
uint8_t selected_actuator = 0;

uint8_t time_stamped = 0;
uint32_t time_ms = 0;
extern uint16_t pin_map[NUM_ACTUATORS];

extern pid_t pid_constraints;

float pid_step(pid_t *pid, float measurement, float setpoint);

void actuate(uint8_t actuator, uint16_t current, uint16_t target)
{
	if(selected_actuator != actuator)
	{
		// Wait an amount of time for electrical safety
		if(!time_stamped)
		{
			// Shut off the PWM pin
			TIM1->CCR1 = 0;
			time_ms = HAL_GetTick();
			time_stamped = 1;
		}

		if(HAL_GetTick() - time_ms >= ACTUATOR_TRANSIENT_DELAY)
		{
			// Reset the transient protection timer
			time_stamped = 0;

			// Deactivate the old actuator
			HAL_GPIO_TogglePin(GPIOB, pin_map[selected_actuator]);

			// Activate the new actuator
			HAL_GPIO_TogglePin(GPIOB, pin_map[actuator]);

			// Exit this code section by switching the actuator
			selected_actuator = actuator;

			// Set the duty cycle
			TIM1->CCR1 = pid_step(&pid_constraints, current, target);
		}
	}
	else
	{
		// Just set the duty cycle

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

