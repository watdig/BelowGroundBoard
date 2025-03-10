/*
 * lin_actuator.h
 *
 *  Created on: Dec 20, 2024
 *      Author: Victor Kalenda
 */

#ifndef INC_LIN_ACTUATOR_H_
#define INC_LIN_ACTUATOR_H_

// DRV Mask Definitions
#define READ_MASK 0b01000000

// DEVICE_ID
#define DRV8243S_Q1	0x32
#define DRV8244S_Q1	0x42
#define DRV8245S_Q1	0x52
#define DRV8243P_Q1 0x36
#define DRV8244P_Q1 0x46
#define DRV8245P_Q1 0x56

typedef union drv_fault_summary_u
{
	uint8_t raw_data;
	struct
	{
		uint8_t ola  	: 1;
		uint8_t tsd  	: 1;
		uint8_t ocp  	: 1;
		uint8_t vmuv 	: 1;
		uint8_t vmov 	: 1;
		uint8_t fault 	: 1;
		uint8_t por	  	: 1;
		uint8_t spi_err : 1;
	};
}drv_fault_summary_t;

typedef union drv_status_1_u
{
	uint8_t raw_data;
	struct
	{
		uint8_t ocp_l2 	  : 1;
		uint8_t ocp_h2 	  : 1;
		uint8_t ocp_l1 	  : 1;
		uint8_t ocp_h1 	  : 1;
		uint8_t active 	  : 1;
		uint8_t itrip_cmp : 1;
		uint8_t ola2 	  : 1;
		uint8_t ola1 	  : 1;
	};
}drv_status_1_t;

typedef union drv_status_2_u
{
	uint8_t raw_data;
	struct
	{
		uint8_t olp_cmp 	 : 1;
		uint8_t reserved_1 	 : 3;
		uint8_t active 		 : 1;
		uint8_t reserved_2 	 : 2;
		uint8_t drv_off_stat : 1;
	};
}drv_status_2_t;


#define SPI_IN_UNLOCK 	0b10
#define SPI_IN_LOCK 	0b01

#define REG_UNLOCK		0b01
#define REG_LOCK		0b10

#define DEFAULT_DRV_COMMAND ((drv_command_t){ .reg_lock = REG_UNLOCK,   \
				  .spi_in_lock = SPI_IN_LOCK })
typedef union drv_command_u
{
	uint8_t raw_data;
	struct
	{
		uint8_t reg_lock 	: 2;
		uint8_t reserved_1	: 1;
		uint8_t spi_in_lock : 2;
		uint8_t reserved_2 	: 2;
		uint8_t clr_flt 	: 1;
	};
}drv_command_t;

#define DEFAULT_DRV_SPI_IN ((drv_spi_in_t){ .s_ph_in2 = 0,   \
				 .s_en_in1 = 0,   \
				 .s_drv_off2 = 1, \
				 .s_drv_off = 1,  })
typedef union drv_spi_in_u
{
	uint8_t raw_data;
	struct
	{
		uint8_t s_ph_in2 	: 1;
		uint8_t s_en_in1 	: 1;
		uint8_t s_drv_off2 	: 1;
		uint8_t s_drv_off 	: 1;
	};
}drv_spi_in_t;

#define VMOV_SEL_35V 		0b00
#define VMOV_SEL_28V 		0b01
#define VMOV_SEL_18V 		0b10
#define VMOV_SEL_DISABLE 	0b11

#define DEFAULT_DRV_CONFIG_1 ((drv_config_1_t){ .ssc_dis = 1			 \
				 .vmov_sel = VMOV_SEL_35V \
				 .en_ola = 0			  })
typedef union drv_config_1_u
{
	uint8_t raw_data;
	struct
	{
		uint8_t ola_retry 	: 1;
		uint8_t vmov_retry 	: 1;
		uint8_t tsd_retry 	: 1;
		uint8_t ocp_retry 	: 1;
		uint8_t ssc_dis 	: 1;
		uint8_t vmov_sel 	: 2;
		uint8_t en_ola 		: 1;

	};
}drv_config_1_t;

#define S_ITRIP_DISABLE	0b000
#define S_ITRIP_1V18	0b001
#define S_ITRIP_1V41	0b010 // Not Available
#define S_ITRIP_1V65	0b011 // Not Available
#define S_ITRIP_1V98	0b100
#define S_ITRIP_2V31	0b101
#define S_ITRIP_2V64	0b110
#define S_ITRIP_2V97	0b111

/*
 * Diagnostics Mode Configuration
 * SPI Variable PH/EN or PWM mode
 * 					|	STANDBY State		| 	ACTIVE State
 * Diagnostics Mode | Off State Diagnostics | On State Diagnostics
 * 		Mode 0		|		Disabled		|	Available
 * 		Mode 1		|		Enabled			|	Available
 * 		Mode 2		|		Enabled			|	Available
 * 		Mode 3		|		Enabled			|	Available
 *
 * SPI Variant Independent Mode
 * 					|	STANDBY State		| 							ACTIVE State
 * Diagnostics Mode | Off State Diagnostics | Load Configuration  | On State Diagnostics | IPROPI / ITRIP
 * 		Mode 0		|		Disabled		|	Low-side Load	  |	Disabled			 | Available
 * 		Mode 1		|		Enabled			|	Low-side Load	  |	Disabled			 | Available
 * 		Mode 2		|		Disabled		|	High-side Load	  |	Available			 | Disabled
 * 		Mode 3		|		Enabled			|	High-side Load	  |	Available			 | Disabled
 */
#define S_DIAG_MODE_0 	0b00
#define S_DIAG_MODE_1	0b01
#define S_DIAG_MODE_2 	0b10
#define S_DIAG_MODE_3	0b11

/*
 * If PWM Extend bit is enabled = 0b1, the following Hi-Z states are possible
 * Previous State | 	Current State 	 	 | Device State Transition
 * OUT1  | OUT2	  | OUT1 | OUT2 | IPROPI 	 | Remains in STANDBY, No change
 * Hi-Z	 | Hi-Z	  | Hi-Z | Hi-Z | No Current | ACTIVE to STANDBY
 * H	 | H	  | Hi-Z | Hi-Z | No Current | ACTIVE to STANDBY
 * L	 | H	  | Hi-Z | H	| 	ISNS2	 | ACTIVE to STANDBY
 * H	 | L	  | H	 | Hi-Z |  	ISNS1 	 | ACTIVE to STANDBY
 */

#define DEFAULT_DRV_CONFIG_2 ((drv_config_2_t){ .s_itrip = S_ITRIP_DISABLE, \
				 .s_diag = S_DIAG_MODE_0,	\
				 .pwm_extend = 0			})
typedef union drv_config_2_u
{
	uint8_t raw_data;
	struct
	{
		uint8_t s_itrip 	: 3;
		uint8_t reserved 	: 2;
		uint8_t s_diag 		: 2;
		uint8_t pwm_extend 	: 1;
	};
}drv_config_2_t;

#define S_MODE_PH_EN		0b00
#define S_MODE_INDEPENDENT	0b01
#define S_MODE_PWM			0b10

/*
 * Slew Rate Configuration
 *
 */

#define TOFF_20US	0b00
#define TOFF_30US	0b01
#define TOFF_40US	0b10
#define TOFF_50US	0b11

#define DEFAULT_DRV_CONFIG_3 ((drv_config_3_t){ .s_mode = S_MODE_PH_EN, \
				 .s_sr = 0,				\
				 .toff = TOFF_30US		})
typedef union drv_config_3_u
{
	uint8_t raw_data;
	struct
	{
		uint8_t s_mode 	 : 2;
		uint8_t s_sr 	 : 3;
		uint8_t reserved : 1;
		uint8_t toff 	 : 2;
	};
}drv_config_3_t;

#define PH_IN2_SEL_OR	0b0
#define PH_IN2_SEL_AND	0b1

#define EN_IN1_SEL_OR	0b0
#define EN_IN1_SEL_AND	0b1

#define DRVOFF_SEL_OR	0b0
#define DRVOFF_SEL_AND	0b1

#define OCP_SEL_100	0b00
#define OCP_SEL_75	0b10
#define OCP_SEL_50	0b01

#define TOCP_SEL_6US	0b00
#define TOCP_SEL_3US	0b01
#define TOCP_SEL_1US5	0b10
#define TOCP_SEL_MIN	0b11

#define DEFAULT_DRV_CONFIG_4 ((drv_config_4_t){ .ph_in2_sel = PH_IN2_SEL_OR,  \
				 .en_in1_sel = EN_IN1_SEL_OR,  \
				 .drvoff_sel = DRVOFF_SEL_AND, \
				 .ocp_sel 	= OCP_SEL_100	   \
				 .tocp_sel 	= TOCP_SEL_6US     })
typedef union drv_config_4_u
{
	uint8_t raw_data;
	struct
	{
		uint8_t ph_in2_sel : 1;
		uint8_t en_in1_sel : 1;
		uint8_t drvoff_sel : 1;
		uint8_t ocp_sel    : 2;
		uint8_t reserved   : 1;
		uint8_t tocp_sel   : 2;
	};
}drv_config_4_t;

typedef enum drv_reg_e
{
	// Read Only
	DEVICE_ID,
	FAULT_SUMMARY,
	STATUS_1,
	STATUS_2,
	// Read-Write
	COMMAND = 0x08,
	SPI_IN,
	CONFIG_1,
	CONFIG_2,
	CONFIG_3,
	CONFIG_4
}drv_reg_t;

typedef struct pid_s
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

void actuate_pwm(uint8_t actuator, uint16_t current, uint16_t target);
int8_t actuate_spi(uint8_t actuator, uint16_t current, uint16_t target);

int8_t DRV_Init(uint8_t device_id);
int8_t DRV_GetDeviceId(uint8_t* device_id);
int8_t DRV_GetFaultSummary(drv_fault_summary_t* fault_summary);
int8_t DRV_GetStatus1(drv_status_1_t* status_1);
int8_t DRV_GetStatus2(drv_status_2_t* status_2);
int8_t DRV_GetCommand(drv_command_t* command);
int8_t DRV_SetCommand(drv_command_t* command);
int8_t DRV_GetSpiIn(drv_spi_in_t* spi_in);
int8_t DRV_SetSpiIn(drv_spi_in_t* spi_in);
int8_t DRV_GetConfig1(drv_config_1_t* config_1);
int8_t DRV_SetConfig1(drv_config_1_t* config_1);
int8_t DRV_GetConfig2(drv_config_2_t* config_2);
int8_t DRV_SetConfig2(drv_config_2_t* config_2);
int8_t DRV_GetConfig3(drv_config_3_t* config_3);
int8_t DRV_SetConfig3(drv_config_3_t* config_3);
int8_t DRV_GetConfig4(drv_config_4_t* config_4);
int8_t DRV_SetConfig4(drv_config_4_t* config_4);

#endif /* INC_LIN_ACTUATOR_H_ */
