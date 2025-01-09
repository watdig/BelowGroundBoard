/*
 * lin_actuator.h
 *
 *  Created on: Dec 20, 2024
 *      Author: Victor Kalenda
 */

#ifndef INC_LIN_ACTUATOR_H_
#define INC_LIN_ACTUATOR_H_

typedef struct
{
    float Kp;              // Proportional gain constant
    float Ki;              // Integral gain constant
    float Kd;              // Derivative gain constant
    float Kaw;             // Anti-windup gain constant
    float T_C;             // Time constant for derivative filtering
    float T;               // Time step
    float max;             // Max command
    float min;             // Min command
    float max_rate;        // Max rate of change of the command
    float integral;        // Integral term
    float err_prev;        // Previous error
    float deriv_prev;      // Previous derivative
    float command_sat_prev;// Previous saturated command
    float command_prev;    // Previous command
}pid_t;

void actuate(uint8_t actuator, uint16_t current, uint16_t target);

#endif /* INC_LIN_ACTUATOR_H_ */
