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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

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
volatile uint8_t low_half_safe;

uint16_t holding_register_database[NUM_HOLDING_REGISTERS] = {
		0x0001,	// MODBUS_ID
		0x0003, // MB_BAUD_RATE
		   100, // MB_TRANSMIT_TIMEOUT
		   	 2, // MB_TRANSMIT_RETRIES
		0x0000, // MB_ERRORS
		0x0000, // I2C_ERRORS
		0x0000, // I2C_SHUTDOWN
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

		0xFFFF, // Actuator A Target
		0xFFFF, // Actuator B Target
		0xFFFF, // Actuator C Target
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



void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	low_half_safe = 0;
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)(&holding_register_database[ADC_0]), 9);
	//HAL_ADC_Stop_DMA(&hadc1);

//	for(uint8_t i = 0; i < 9; i++)
//	{
//		holding_register_database[i + 3] = (uint16_t)raw_data[i];
//	}
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
	low_half_safe = 1;
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
	uint8_t modbus_tx_len = 0;
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
  pid_constraints.min = 0;             // Min command
  pid_constraints.max_rate = 40;       // Max rate of change of the command
  pid_constraints.integral = 0;        // Integral term
  pid_constraints.err_prev = 0;        // Previous error
  pid_constraints.deriv_prev = 0;      // Previous derivative
  pid_constraints.command_sat_prev = 0;// Previous saturated command
  pid_constraints.command_prev = 0;    // Previous command

//  if(modbus_set_rx() != HAL_OK)
//  {
//	  Error_Handler();
//  }

  low_half_safe = 0;
//  if(HAL_ADC_Start_DMA(&hadc1, (uint32_t*)(&holding_register_database[ADC_0]), 9) != HAL_OK)
//  {
//	  Error_Handler();
//  }

  bno055_setup();
  bno055_setOperationModeNDOF();


//  	if(init_lin_actuator() != HAL_OK)
//  	{
//  		Error_Handler();
//  	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /*
   * target_actuator
   * 0: Actuator A
   * 1: Actuator B
   * 2: Actuator C
   */
//  uint8_t target_actuator = 0;

  // Start the retrieval process for the bno055 (i2c in interrupt mode)
  //bno055_queue_transaction();

  while (1)
  {
	  if(modbus_rx())
	  {
		  if(get_rx_buffer(0) == holding_register_database[0]) // Check Slave ID
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
				  holding_register_database[MB_ERRORS] |= 1U << (modbus_status + (MB_FATAL_ERROR - 1));
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
				  holding_register_database[MB_ERRORS] |= 1U << ((modbus_status - 1) + MB_FATAL_ERROR);
			  }
		  }
		  modbus_status = modbus_set_rx();
		  if(modbus_status != 0)
		  {
			  holding_register_database[MB_ERRORS] |= 1U << ((modbus_status - 1) + MB_FATAL_ERROR);
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
						  holding_register_database[MB_ERRORS] |= 1U << ((modbus_status - 1) + MB_FATAL_ERROR);
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
					  holding_register_database[MB_ERRORS] |= 1U << ((modbus_status - 1) + MB_FATAL_ERROR);
				  }
				  break;
			  }
			  case MB_FATAL_ERROR:
			  {
				  while(modbus_status != HAL_OK)
				  {
					  modbus_status = modbus_reset();
				  }
				  break;
			  }
			  default:
			  {
				  // Unknown error
			  }
		  }
		  // Handle a MB_TX_TIMEOUT
	  }

	  if(!holding_register_database[I2C_SHUTDOWN])
	  {
		  if(bno055_rx())
		  {
			  i2c_status = bno055_queue_transaction();
			  if(i2c_status != 0)
			  {
				  holding_register_database[I2C_ERRORS] |= 1U << ((i2c_status - 1) + I2C_FATAL_ERROR);
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

//	  bno055_get_all_values();

//	  if(holding_register_database[9 + target_actuator] >= holding_register_database[56 + target_actuator] - ACTUATOR_TOLERANCE &&
//		 holding_register_database[9 + target_actuator] <= holding_register_database[56 + target_actuator] + ACTUATOR_TOLERANCE)
//	  {
//		  actuate_spi(target_actuator, holding_register_database[9 + target_actuator], holding_register_database[56 + target_actuator]);
//	  }
//	  else
//	  {
//		  target_actuator = ((target_actuator + 1) == NUM_ACTUATORS)? 0: target_actuator + 1;
//	  }

	  // 15 adc values relates to x cm of the linear actuator
//	  if(holding_register_database[9 + target_actuator] >= holding_register_database[56 + target_actuator] - ACTUATOR_TOLERANCE &&
//		 holding_register_database[9 + target_actuator] <= holding_register_database[56 + target_actuator] + ACTUATOR_TOLERANCE)
//	  {
//		  actuate_pwm(target_actuator, holding_register_database[9 + target_actuator], holding_register_database[56 + target_actuator]);
//	  }
//	  else
//	  {
//		  target_actuator = ((target_actuator + 1) == NUM_ACTUATORS)? 0: target_actuator + 1;
//	  }

	  // TEST CODE START

	  // DRV8244 Testing
	  // PWM Actuator Test
//	  HAL_GPIO_WritePin(Actuator_A_EN_GPIO_Port, Actuator_A_EN_Pin, GPIO_PIN_SET);
//	  TIM1->CCR1 = 10;
//	  HAL_Delay(1000);
//	  TIM1->CCR1 = 0;
//	  HAL_GPIO_WritePin(Actuator_A_EN_GPIO_Port, Actuator_A_EN_Pin, GPIO_PIN_RESET);

//	  uint8_t tx_data[2];
//	  uint8_t rx_data[2];
//
//	  // Independent Mode Test
//	  // Unlock the SPI_IN register. Refer to section 8.6.1.5
//		tx_data[0] = COMMAND; // WRITE MASK = 0
//		tx_data[1] = SPI_IN_UNLOCK;
//		HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, 100);
//
//
//		// Forwards
//		tx_data[0] = SPI_IN; // WRITE MASK = 0
//		tx_data[1] = S_EN_IN1;
//		HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, 100);
//
//		HAL_Delay(1000);
//
//		// Turn off the DRV8244
//		tx_data[0] = SPI_IN; // WRITE MASK = 0
//		tx_data[1] = 0;
//		HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, 100);
//
//		// Lock the SPI_IN register. Refer to section 8.6.1.5
//		tx_data[0] = COMMAND; // WRITE MASK = 0
//		tx_data[1] = SPI_IN_LOCK;
//		HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, 100);

	  // TEST CODE END

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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */
  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_SEQ_FIXED;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 0;
  hadc1.Init.DiscontinuousConvMode = ENABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_160CYCLES_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_LOW;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  LL_ADC_REG_SetSequencerConfigurable(ADC1, LL_ADC_REG_SEQ_FIXED);
  LL_ADC_REG_SetSequencerScanDirection(ADC1, LL_ADC_REG_SEQ_SCAN_DIR_FORWARD);

  LL_ADC_REG_SetSequencerChannels(ADC1, LL_ADC_CHANNEL_0 | LL_ADC_CHANNEL_1 |
		  	  	  	  	  	  	  	  	LL_ADC_CHANNEL_2 | LL_ADC_CHANNEL_3 |
										LL_ADC_CHANNEL_4 | LL_ADC_CHANNEL_5 |
										LL_ADC_CHANNEL_6 | LL_ADC_CHANNEL_7 |
										LL_ADC_CHANNEL_8);
  uint32_t setup_adc_time = HAL_GetTick();

  while(LL_ADC_IsActiveFlag_CCRDY(ADC1) && HAL_GetTick() - setup_adc_time <= 100);
  if(!LL_ADC_IsActiveFlag_CCRDY(ADC1))
  {
	  Error_Handler();
  }
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
  hi2c1.Init.Timing = 0x00201D2C;
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  htim1.Init.Prescaler = 10-1;
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
  htim14.Init.Prescaler = 12 - 1 ;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 65535;
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
  huart1.Init.BaudRate = 9600;
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
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
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
  HAL_GPIO_WritePin(GPIOC, Actuator_A_EN_Pin|Actuator_B_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Actuator_CS_GPIO_Port, Actuator_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : Actuator_C_EN_Pin Encoder_Pulse_B_Pin */
  GPIO_InitStruct.Pin = Actuator_C_EN_Pin|Encoder_Pulse_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Encoder_Pulse_A_Pin */
  GPIO_InitStruct.Pin = Encoder_Pulse_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Encoder_Pulse_A_GPIO_Port, &GPIO_InitStruct);

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
