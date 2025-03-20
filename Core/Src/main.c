/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "modbus.h"
#include "bno055.h"
#include "lin_actuator.h"
#include "error_codes.h"
#include "sensor_adc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

pid_t pid_constraints;
uint16_t pin_map[NUM_ACTUATORS];
GPIO_TypeDef* port_map[NUM_ACTUATORS];
volatile int32_t encoder_pulse;

uint16_t holding_register_database[NUM_HOLDING_REGISTERS] = {
		0x0005,	// MODBUS_ID
		0x0007, // MB_BAUD_RATE
		   100, // MB_TRANSMIT_TIMEOUT
		   	 2, // MB_TRANSMIT_RETRIES
		0x0000, // MB_ERRORS
		0x0000, // I2C_ERRORS
		0x0000, // I2C_SHUTDOWN
		0x0000, // ADC_ERRORS
		0x0000, // AUTOPILOT

		0xFFFF, // ADC 0
		0xFFFF, // ADC 1
		0xFFFF, // ADC 2
		0xFFFF, // ADC 3
		0xFFFF, // ADC 4
		0xFFFF, // ADC 5
		0xFFFF, // ADC 6
		0xFFFF, // ADC 7
		0xFFFF, // ADC 8

		0xFFFF, // Accelerometer X
		0xFFFF, // Accelerometer Y
		0xFFFF, // Accelerometer Z
		0xFFFF, // Magnetometer X
		0xFFFF, // Magnetometer Y
		0xFFFF, // Magnetometer Z
		0xFFFF, // Gyroscope X
		0xFFFF, // Gyroscope Y
		0xFFFF, // Gyroscope Z

		0xFFFF, // Euler Heading
		0xFFFF, // Euler Roll
		0xFFFF, // Euler Pitch
		0xFFFF, // Linear Acceleration X
		0xFFFF, // Linear Acceleration Y
		0xFFFF, // Linear Acceleration Z
		0xFFFF, // Gravity X
		0xFFFF, // Gravity Y
		0xFFFF, // Gravity Z
		0xFFFF, // Quarternion W
		0xFFFF, // Quarternion X
		0xFFFF, // Quarternion Y
		0xFFFF, // Quarternion Z

		0xFFFF, // Remote Accelerometer X
		0xFFFF, // Remote Accelerometer Y
		0xFFFF, // Remote Accelerometer Z
		0xFFFF, // Remote Magnetometer X
		0xFFFF, // Remote Magnetometer Y
		0xFFFF, // Remote Magnetometer Z
		0xFFFF, // Remote Gyroscope X
		0xFFFF, // Remote Gyroscope Y
		0xFFFF, // Remote Gyroscope Z

		0xFFFF, // Euler Heading
		0xFFFF, // Euler Roll
		0xFFFF, // Euler Pitch
		0xFFFF, // Linear Acceleration X
		0xFFFF, // Linear Acceleration Y
		0xFFFF, // Linear Acceleration Z
		0xFFFF, // Gravity X
		0xFFFF, // Gravity Y
		0xFFFF, // Gravity Z
		0xFFFF, // Quarternion W
		0xFFFF, // Quarternion X
		0xFFFF, // Quarternion Y
		0xFFFF, // Quarternion Z

		0x0000, // Encoder Speed
		0x2710, // Encoder Refresh Rate

		400, // Actuator A Target
		400, // Actuator B Target
		400, // Actuator C Target
		5000, // Actuator Time
		0xFFFF, 0xFFFF, // Proportional Gain
		0xFFFF, 0xFFFF, // Integral Gain
		0xFFFF, 0xFFFF, // Derivative Gain
		0xFFFF, 0xFFFF, // Time for Derivative Filtering
		0xFFFF, 0xFFFF, // Time Step
		0xFFFF, 0xFFFF, // Maximum Rate of Change
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM14_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// Refresh the Encoder Speed (pulses / 10 minutes)
	holding_register_database[ENCODER_SPEED] = (int16_t)((((uint32_t)encoder_pulse)*600UL*10000UL) / (htim14.Init.Period + 1UL));
	encoder_pulse = 0;
}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
	GPIO_PinState pulse_b = HAL_GPIO_ReadPin(Encoder_Pulse_B_GPIO_Port, Encoder_Pulse_B_Pin);
	if(pulse_b == GPIO_PIN_SET)
	{
		encoder_pulse++;
	}
	else
	{
		encoder_pulse--;
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	int8_t modbus_status = HAL_OK;
	int8_t i2c_status = HAL_OK;
	int8_t adc_status = HAL_OK;
	uint8_t modbus_tx_len = 0;
	uint16_t encoder_refresh = 10000;
	uint8_t actuate_complete[NUM_ACTUATORS];
	uint8_t macro_consistency_count[NUM_ACTUATORS];
	uint8_t micro_consistency_count[NUM_ACTUATORS];
	uint32_t actuator_time = 0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM14_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  // Initialize the pin mapping table
  pin_map[0] = Actuator_A_EN_Pin;
  pin_map[1] = Actuator_B_EN_Pin;
  pin_map[2] = Actuator_C_EN_Pin;

  port_map[0] = Actuator_A_EN_GPIO_Port;
  port_map[1] = Actuator_B_EN_GPIO_Port;
  port_map[2] = Actuator_C_EN_GPIO_Port;

  // Initialize the PID constraints to defaults
  pid_constraints.Kp = 1;              // Proportional gain constant
  pid_constraints.Ki = 0.1;            // Integral gain constant
  pid_constraints.Kd = 5;              // Derivative gain constant
  pid_constraints.Kaw = 0.1;           // Anti-windup gain constant
  pid_constraints.T_C = 1;             // Time constant for derivative filtering
  pid_constraints.T = 100;             // Time step
  pid_constraints.max = 100;           // Max command
  pid_constraints.min = 15;            // Min command
  pid_constraints.max_rate = 40;       // Max rate of change of the command
  pid_constraints.integral = 0;        // Integral term
  pid_constraints.err_prev = 0;        // Previous error
  pid_constraints.deriv_prev = 0;      // Previous derivative
  pid_constraints.command_sat_prev = 0;// Previous saturated command
  pid_constraints.command_prev = 0;    // Previous command

  if(modbus_set_rx() != HAL_OK)
  {
	  Error_Handler();
  }

  /* Perform ADC activation procedure to make it ready to convert. */
  ADC_Activate();
  if ((LL_ADC_IsEnabled(ADC1) == 1)               &&
	  (LL_ADC_IsDisableOngoing(ADC1) == 0)        &&
	  (LL_ADC_REG_IsConversionOngoing(ADC1) == 0))
  {
	  LL_ADC_REG_StartConversion(ADC1);
  }

  bno055_init();

  	if(DRV_Init(DRV8244P_Q1) != HAL_OK)
  	{
  		Error_Handler();
  	}
	/*
	* target_actuator
	* 0: Actuator A
	* 1: Actuator B
	* 2: Actuator C
	*/
  	uint8_t target_actuator = 0;
  	actuate_complete[0] = 0;
  	actuate_complete[1] = 0;
  	actuate_complete[2] = 0;

  	HAL_TIM_Base_Start_IT(&htim14);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {
	  if(modbus_rx())
	  {
		  if(get_rx_buffer(0) == holding_register_database[MODBUS_ID]) // Check Slave ID
		  {
			  switch(get_rx_buffer(1))
			  {
				  case 0x03:
				  {
					  // Return holding registers
					  modbus_status = return_holding_registers(&modbus_tx_len);
					  break;
				  }
				  case 0x10:
				  {
					  // Write holding registers
					  modbus_status = edit_multiple_registers(&modbus_tx_len);
					  break;
				  }
				  default:
				  {
					  modbus_status = modbus_exception(MB_ILLEGAL_FUNCTION);
					  break;
				  }
			  }
			  if(modbus_status != 0)
			  {
				  holding_register_database[MB_ERRORS] |= 1U << ((modbus_status) + (MB_FATAL_ERROR - RANGE_ERROR));
			  }
		  }
		  // Special case where you retrieve the modbus ID
		  else if((get_rx_buffer(0) == 0xFF) && // modbus_id = 0xFF = 255
			(get_rx_buffer(1) == 0x03) && // Function code = read_holding_registers
			(((get_rx_buffer(2) << 8) | get_rx_buffer(3)) == 0x00) && // Address to read = 0
			(((get_rx_buffer(4) << 8) | get_rx_buffer(5)) == 1)) // # of registers to read = 1
		  {

			  modbus_status = return_holding_registers(&modbus_tx_len);
			  if(modbus_status != 0)
			  {
				  holding_register_database[MB_ERRORS] |= 1U << ((modbus_status) + (MB_FATAL_ERROR - RANGE_ERROR));
			  }
		  }
	  }
	  modbus_status = monitor_modbus();
	  if(modbus_status != HAL_OK && modbus_status != HAL_BUSY)
	  {
		  switch(modbus_status)
		  {
			  case MB_TX_TIMEOUT:
			  {
				  for(uint8_t i = 0; i < holding_register_database[MB_TRANSMIT_RETRIES]; i++)
				  {
					  modbus_status = modbus_send(modbus_tx_len);
					  if(modbus_status != HAL_OK)
					  {
						  holding_register_database[MB_ERRORS] |= 1U << ((modbus_status) + (MB_FATAL_ERROR - RANGE_ERROR));
					  }
				  }
				  break;
			  }
			  case MB_RX_TIMEOUT:
			  {
				  // Error only relates to Modbus Master Nodes
				  break;
			  }
			  case MB_UART_ERROR:
			  {
				  modbus_status = modbus_set_rx();
				  if(modbus_status != 0)
				  {
					  holding_register_database[MB_ERRORS] |= 1U << ((modbus_status) + (MB_FATAL_ERROR - RANGE_ERROR));
				  }
				  break;
			  }
			  case MB_FATAL_ERROR:
			  {
				  while(modbus_status != HAL_OK)
				  {
					  modbus_status = modbus_reset();
				  }
				  modbus_status = modbus_set_rx();
				  if(modbus_status != 0)
				  {
					  holding_register_database[MB_ERRORS] |= 1U << ((modbus_status) + (MB_FATAL_ERROR - RANGE_ERROR));
				  }
				  break;
			  }
			  default:
			  {
				  // Unknown error
			  }
		  }
	  }

	  if(!holding_register_database[I2C_SHUTDOWN])
	  {
		  if(bno055_rx())
		  {
			  i2c_status = bno055_queue_transaction();
			  if(i2c_status != 0)
			  {
				  holding_register_database[I2C_ERRORS] |= 1U << ((i2c_status) + (I2C_FATAL_ERROR - I2C_TX_TIMEOUT));
			  }
		  }

		  i2c_status = monitor_i2c();
		  if(i2c_status != HAL_OK && i2c_status != HAL_BUSY)
		  {
			  switch(i2c_status)
			  {
				  case I2C_TX_TIMEOUT:
				  {
					  break;
				  }
				  case I2C_RX_TIMEOUT:
				  {
					  break;
				  }
				  case I2C_ERROR:
				  {
					  break;
				  }
				  case I2C_FATAL_ERROR:
				  {
					  // Disable the I2C peripheral
					  holding_register_database[I2C_SHUTDOWN] = 1;
					  break;
				  }
				  default:
				  {
					  // Unknown error
				  }
			  }
		  }
	  }

	  // If the actuator is in the Micro range
	  if(((holding_register_database[ADC_6 + target_actuator]) >= (holding_register_database[ACTUATOR_A_TARGET + target_actuator] - ACTUATOR_TOLERANCE)) &&
		 ((holding_register_database[ADC_6 + target_actuator]) <= (holding_register_database[ACTUATOR_A_TARGET + target_actuator] + ACTUATOR_TOLERANCE)))
	  {
		  micro_consistency_count[target_actuator]++;
		  if(micro_consistency_count[target_actuator] >= 5)
		  {
			  micro_consistency_count[target_actuator] = 5;
			  actuate_complete[target_actuator] = 1;
			  if(actuate_complete[0] == 1 &&
				 actuate_complete[1] == 1 &&
				 actuate_complete[2] == 1 &&
				 !DRV_GetShutoff())
			  {
				  DRV_Shutoff();
			  }
			  else if(!DRV_GetShutoff())
			  {
				  // Move to the next actuator and force it to run if it isn't in the requested range
				  actuator_time = HAL_GetTick() - ((uint32_t)holding_register_database[ACTUATOR_TIME]);
				  target_actuator = ((target_actuator + 1) == NUM_ACTUATORS)? 0: target_actuator + 1;
			  }
		  }
	  }
	  else
	  {
		  micro_consistency_count[target_actuator] = 0;
		  // if the DRV is shutoff
		  if(DRV_GetShutoff())
		  {
			  // If the ADC is outside of the Macro range
			  if(((holding_register_database[ADC_6 + target_actuator]) <= (holding_register_database[ACTUATOR_A_TARGET + target_actuator] - ACTUATOR_TOLERANCE_MACRO)) ||
				 ((holding_register_database[ADC_6 + target_actuator]) >= (holding_register_database[ACTUATOR_A_TARGET + target_actuator] + ACTUATOR_TOLERANCE_MACRO)))
			  {
				  macro_consistency_count[target_actuator]++;
				  if(macro_consistency_count[target_actuator] >= 10)
				  {
					  macro_consistency_count[target_actuator] = 10;
					  DRV_Activate(target_actuator, holding_register_database[ADC_6 + target_actuator], holding_register_database[ACTUATOR_A_TARGET + target_actuator]);
					  actuator_time = HAL_GetTick();
					  actuate_complete[0] = 0;
					  actuate_complete[1] = 0;
					  actuate_complete[2] = 0;
				  }
			  }
			  else
			  {
				 // Check a different actuator
				  macro_consistency_count[target_actuator] = 0;
				  target_actuator = ((target_actuator + 1) == NUM_ACTUATORS)? 0: target_actuator + 1;
			  }
		  }
		  else
		  {
			  // If the actuator has been running for a given amount of time
			  if((HAL_GetTick() - actuator_time) >= ((uint32_t)holding_register_database[ACTUATOR_TIME]))
			  {
				  actuator_time = HAL_GetTick();
				  target_actuator = ((target_actuator + 1) == NUM_ACTUATORS)? 0: target_actuator + 1;
			  }
		  }
	  }
	  actuate(target_actuator, holding_register_database[ADC_6 + target_actuator], holding_register_database[ACTUATOR_A_TARGET + target_actuator], &actuator_time);

	  // Monitor any ADC Errors
	  adc_status = monitor_adc();
	  if(adc_status != HAL_OK)
	  {
		  // An Overrun has occurred as shown in ADC1_IRQHandler in stm32c0xx_it.c. Log the error
		  holding_register_database[ADC_ERRORS] = 1;
	  }

	  // Handle the User changing the encoder refresh rate (does not log potential errors)
	  if(encoder_refresh != holding_register_database[ENCODER_REFRESH])
	  {
		  HAL_TIM_Base_Stop_IT(&htim14);
		  HAL_TIM_Base_DeInit(&htim14);
		  __HAL_RCC_TIM14_FORCE_RESET();
		  HAL_Delay(10);
		  __HAL_RCC_TIM14_RELEASE_RESET();
		  HAL_Delay(10);
		  htim14.Init.Period = holding_register_database[ENCODER_REFRESH] - 1;
		  HAL_TIM_Base_Init(&htim14);
		  HAL_TIM_Base_Start_IT(&htim14);
		  encoder_refresh = holding_register_database[ENCODER_REFRESH];
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC);

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  /**ADC1 GPIO Configuration
  PA0   ------> ADC1_IN0
  PA1   ------> ADC1_IN1
  PA2   ------> ADC1_IN2
  PA3   ------> ADC1_IN3
  PA4   ------> ADC1_IN4
  PA5   ------> ADC1_IN5
  PA6   ------> ADC1_IN6
  PA7   ------> ADC1_IN7
  PA8   ------> ADC1_IN8
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* ADC1 DMA Init */

  /* ADC1 Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_1, LL_DMAMUX_REQ_ADC1);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_CIRCULAR);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_HALFWORD);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_HALFWORD);

  /* ADC1 interrupt Init */
  NVIC_SetPriority(ADC1_IRQn, 0);
  NVIC_EnableIRQ(ADC1_IRQn);

  /* USER CODE BEGIN ADC1_Init 1 */

  /* Set DMA transfer addresses of source and destination */
  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_1, LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA),
		  (uint32_t)&holding_register_database[ADC_0],
		  LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  /* Set DMA transfer size */
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, 9);

  /* Enable DMA transfer interruption: transfer complete */
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);

  /* Enable DMA transfer interruption: transfer half complete */
  LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_1);

  /* Enable DMA transfer interruption: transfer error */
  LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_1);

  /* Enable the DMA transfer */
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */

   #define ADC_CHANNEL_CONF_RDY_TIMEOUT_MS ( 1U)
   #if (USE_TIMEOUT == 1)
   uint32_t Timeout ; /* Variable used for Timeout management */
   #endif /* USE_TIMEOUT */

  ADC_InitStruct.Clock = LL_ADC_CLOCK_SYNC_PCLK_DIV2;
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);
  LL_ADC_REG_SetSequencerConfigurable(ADC1, LL_ADC_REG_SEQ_FIXED);
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
  ADC_REG_InitStruct.SequencerLength = 0;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_CONTINUOUS;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_UNLIMITED;
  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
  LL_ADC_REG_SetSequencerScanDirection(ADC1, LL_ADC_REG_SEQ_SCAN_DIR_FORWARD);
  LL_ADC_SetOverSamplingScope(ADC1, LL_ADC_OVS_DISABLE);
  LL_ADC_SetTriggerFrequencyMode(ADC1, LL_ADC_CLOCK_FREQ_MODE_HIGH);
  LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_0|LL_ADC_CHANNEL_1
                              |LL_ADC_CHANNEL_2|LL_ADC_CHANNEL_3
                              |LL_ADC_CHANNEL_4|LL_ADC_CHANNEL_5
                              |LL_ADC_CHANNEL_6|LL_ADC_CHANNEL_7
                              |LL_ADC_CHANNEL_8);

   /* Poll for ADC channel configuration ready */
   #if (USE_TIMEOUT == 1)
   Timeout = ADC_CHANNEL_CONF_RDY_TIMEOUT_MS;
   #endif /* USE_TIMEOUT */
   while (LL_ADC_IsActiveFlag_CCRDY(ADC1) == 0)
     {
   #if (USE_TIMEOUT == 1)
   /* Check Systick counter flag to decrement the time-out value */
   if (LL_SYSTICK_IsActiveCounterFlag())
     {
   if(Timeout-- == 0)
         {
   Error_Handler();
         }
     }
   #endif /* USE_TIMEOUT */
     }
   /* Clear flag ADC channel configuration ready */
   LL_ADC_ClearFlag_CCRDY(ADC1);
  LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_COMMON_1, LL_ADC_SAMPLINGTIME_160CYCLES_5);
  LL_ADC_DisableIT_EOC(ADC1);
  LL_ADC_DisableIT_EOS(ADC1);

   /* Enable ADC internal voltage regulator */
   LL_ADC_EnableInternalRegulator(ADC1);
   /* Delay for ADC internal voltage regulator stabilization. */
   /* Compute number of CPU cycles to wait for, from delay in us. */
   /* Note: Variable divided by 2 to compensate partially */
   /* CPU processing cycles (depends on compilation optimization). */
   /* Note: If system core clock frequency is below 200kHz, wait time */
   /* is only a few CPU processing cycles. */
   uint32_t wait_loop_index;
   wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
   while(wait_loop_index != 0)
     {
   wait_loop_index--;
     }
  /* USER CODE BEGIN ADC1_Init 2 */

   LL_ADC_EnableIT_OVR(ADC1);

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x0010020B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 80-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 800 - 1 ;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 10000 - 1;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart1, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel1_IRQn, 0);
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
  /* DMAMUX1_DMA1_CH4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMAMUX1_DMA1_CH4_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMAMUX1_DMA1_CH4_5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Actuator_C_EN_GPIO_Port, Actuator_C_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Actuator_A_EN_Pin|Actuator_B_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Actuator_CS_GPIO_Port, Actuator_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : Actuator_C_EN_Pin */
  GPIO_InitStruct.Pin = Actuator_C_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Actuator_C_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Encoder_Pulse_A_Pin */
  GPIO_InitStruct.Pin = Encoder_Pulse_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Encoder_Pulse_A_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Encoder_Pulse_B_Pin */
  GPIO_InitStruct.Pin = Encoder_Pulse_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Encoder_Pulse_B_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Actuator_A_EN_Pin Actuator_B_EN_Pin */
  GPIO_InitStruct.Pin = Actuator_A_EN_Pin|Actuator_B_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Actuator_CS_Pin */
  GPIO_InitStruct.Pin = Actuator_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(Actuator_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IMU_Reset_Pin */
  GPIO_InitStruct.Pin = IMU_Reset_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IMU_Reset_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
