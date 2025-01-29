/*
 * lin_actuator.h
 *
 *  Created on: Dec 20, 2024
 *      Author: Victor Kalenda
 */

#ifndef INC_LIN_ACTUATOR_H_
#define INC_LIN_ACTUATOR_H_

// DRV8244 Mask Definitions
#define READ_MASK 0b01000000
#define DUMMY_DATA 0x00

// DEVICE_ID
#define DRV8244_ID 			0x42

// FAULT_SUMMARY
#define SPI_ERR				0b10000000
#define POR					0b01000000
#define FAULT				0b00100000
#define VMOV				0b00010000
#define VMUV				0b00001000
#define OCP					0b00000100
#define TSD					0b00000010
#define OLA					0b00000001

// STATUS_1
#define OLA1				0b10000000
#define OLA2				0b01000000
#define ITRIP_CMP			0b00100000
#define ACTIVE				0b00010000
#define OCP_H1				0b00001000
#define OCP_L1				0b00000100
#define OCP_H2				0b00000010
#define OCP_L2				0b00000001

// STATUS_2
#define DRV_OFF_STAT		0b10000000
// ACTIVE bit 4 for STATUS_2 as well
#define OLP_CMP				0b00000001

// COMMAND
#define CLR_FLT 			0b10000000
#define SPI_IN_LOCK 		0b00001000 // Locks the SPI_IN register (default)
#define SPI_IN_UNLOCK 		0b00010000 // Unlocks the SPI_IN register
#define REG_LOCK 			0b00000010 // Locks the CONFIG registers
#define REG_UNLOCK 			0b00000001 // Unlocks the CONFIG registers (default)

// SPI_IN
#define S_DRVOFF			0b00001000 // Independent mode - shuts off half bridge 1
#define S_DRVOFF2			0b00000100 // Independent mode - shuts off half bridge 2
#define S_EN_IN1			0b00000010
#define S_PH_IN2			0b00000001

// CONFIG_1
#define EN_OLA 				0b10000000 // Enable open load detection in active state
// VM over voltage diagnostic thresholds
#define VMOV_SEL_35V 		0b00000000 // (default)
#define VMOV_SEL_28V		0b00100000
#define VMOV_SEL_18V		0b01000000
#define VMOV_SEL_DISABLE	0b01100000
#define SSC_DIS 			0b00010000 // Write 0 to enable spread spectrum clocking feature
#define OCP_RETRY 			0b00001000 // Retry setting on the over current detection
#define TSD_RETRY 			0b00000100 // Retry setting on the over temperature detection
#define VMOV_RETRY 			0b00000010 // Retry setting on the over voltage detection
#define OLA_RETRY			0b00000001 // Retry setting on open load detection in active state

// CONFIG_2
#define PWM_EXTEND			0b10000000
#define S_DIAG				0b01100000 // Load Type indication
#define S_ITRIP 			0b00000111 // ITRIP level configuration

// CONFIG_3
// Time used for ITRIP current regulation
#define TOFF_20U			0b00000000
#define TOFF_30U			0b01000000
#define TOFF_40U			0b10000000 // (default)
#define TOFF_50U			0b11000000
#define S_SR				0b00011100 // Slew Rate Configuration
// Device Mode Configuration
#define S_MODE_PH_EN		0b00000000 // PH/IN2 = Direction, EN/IN1 = PWM (default)
#define S_MODE_INDEPENDENT	0b00000001 // SPI shuts each half bridge either on or off
#define S_MODE_PWM			0b00000011 // 2 PWM pins dictate direction

// CONFIG_4
// Filter time for over current detection configuration
#define TOCP_SEL_6U			0b00000000
#define TOCP_SEL_3U			0b01000000
#define TOCP_SEL_1U5		0b10000000
#define TOCP_SEL_MIN		0b11000000
// Threshold for over current detection configuration
#define OCP_SEL_100			0b00000000
#define OCP_SEL_50			0b00001000
#define OCP_SEL_25			0b00010000
#define DRV_OFF_SEL			0b00000100
#define EN_IN1_SEL			0b00000010 // 1 = AND, 0 = OR (default)
#define PH_IN2_SEL			0b00000001 // 1 = AND, 0 = OR (default)

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

typedef enum drv_reg_e
{
	// Read Only
	DEVICE_ID,
	FAULT_SUMMARY,
	STATUS_1,
	STATUS_2,
	// Read Write
	COMMAND = 0x08,
	SPI_IN,
	CONFIG_1,
	CONFIG_2,
	CONFIG_3,
	CONFIG_4
}drv_reg_t;

HAL_StatusTypeDef init_lin_actuator();
void actuate_pwm(uint8_t actuator, uint16_t current, uint16_t target);
void actuate_spi(uint8_t actuator, uint16_t current, uint16_t target);

#endif /* INC_LIN_ACTUATOR_H_ */
