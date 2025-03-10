/*
 * sensor_adc.c
 *
 *  Created on: Mar 8, 2025
 *      Author: Victor Kalenda
 */

/**
  * @brief  Perform ADC activation procedure to make it ready to convert
  *         (ADC instance: ADC1).
  * @param  None
  * @retval None
  */

#include "main.h"
#include "sensor_adc.h"

/* Definitions of ADC hardware constraints delays */
/* Note: Only ADC IP HW delays are defined in ADC LL driver driver,           */
/*       not timeout values:                                                  */
/*       Timeout values for ADC operations are dependent to device clock      */
/*       configuration (system clock versus ADC clock),                       */
/*       and therefore must be defined in user application.                   */
/*       Refer to @ref ADC_LL_EC_HW_DELAYS for description of ADC timeout     */
/*       values definition.                                                   */

/* Timeout values for ADC operations. */
/* (calibration, enable settling time, disable settling time, ...)          */
/* Values defined to be higher than worst cases: low clock frequency,       */
/* maximum prescalers.                                                      */
/* Note: ADC channel configuration ready (ADC_CHANNEL_CONF_RDY_TIMEOUT_MS)  */
/*       is added in CubeMx code section.                                   */
/* Unit: ms                                                                 */
#define ADC_CALIBRATION_TIMEOUT_MS       (   1UL)
#define ADC_ENABLE_TIMEOUT_MS            (   1UL)
#define ADC_DISABLE_TIMEOUT_MS           (   1UL)
#define ADC_STOP_CONVERSION_TIMEOUT_MS   (   1UL)
#define ADC_CONVERSION_TIMEOUT_MS        (4000UL)


/* Delay between ADC end of calibration and ADC enable.                     */
/* Delay estimation in CPU cycles: Case of ADC enable done                  */
/* immediately after ADC calibration, ADC clock setting slow                */
/* (LL_ADC_CLOCK_ASYNC_DIV32). Use a higher delay if ratio                  */
/* (CPU clock / ADC clock) is above 32.                                     */
#define ADC_DELAY_CALIB_ENABLE_CPU_CYCLES  (LL_ADC_DELAY_CALIB_ENABLE_ADC_CYCLES * 32)

/* Definitions of environment analog values */
/* Value of analog reference voltage (Vref+), connected to analog voltage   */
/* supply Vdda (unit: mV).                                                  */
#define VDDA_APPLI                       (3300UL)

/* Definitions of data related to this example */
/* Definition of ADCx conversions data table size */
#define ADC_CONVERTED_DATA_BUFFER_SIZE   (   4UL)

/* Init variable out of expected ADC conversion data range */
#define VAR_CONVERTED_DATA_INIT_VALUE    (__LL_ADC_DIGITAL_SCALE(LL_ADC_RESOLUTION_12B) + 1)

volatile uint8_t low_half_safe;

void ADC_ConvCpltCallback()
{
	low_half_safe = 0;
	//HAL_ADC_Stop_DMA(&hadc1);

//	for(uint8_t i = 0; i < 9; i++)
//	{
//		holding_register_database[i + 3] = (uint16_t)raw_data[i];
//	}
}

void ADC_ConvHalfCpltCallback()
{
	low_half_safe = 1;
}

void ADC_ErrorCallback()
{

}

void ADC_Activate()
{
low_half_safe = 0;
	__IO uint32_t wait_loop_index = 0U;
	__IO uint32_t backup_setting_adc_dma_transfer = 0U;
	#if (USE_TIMEOUT == 1)
	uint32_t Timeout = 0U; /* Variable used for timeout management */
	#endif /* USE_TIMEOUT */

	/*## Operation on ADC hierarchical scope: ADC instance #####################*/

	/* Note: Hardware constraint (refer to description of the functions         */
	/*       below):                                                            */
	/*       On this STM32 series, setting of these features is conditioned to  */
	/*       ADC state:                                                         */
	/*       ADC must be disabled.                                              */
	/* Note: In this example, all these checks are not necessary but are        */
	/*       implemented anyway to show the best practice usages                */
	/*       corresponding to reference manual procedure.                       */
	/*       Software can be optimized by removing some of these checks, if     */
	/*       they are not relevant considering previous settings and actions    */
	/*       in user application.                                               */
	if (LL_ADC_IsEnabled(ADC1) == 0)
	{
		/* Enable ADC internal voltage regulator */
		LL_ADC_EnableInternalRegulator(ADC1);

		/* Delay for ADC internal voltage regulator stabilization.                */
		/* Compute number of CPU cycles to wait for, from delay in us.            */
		/* Note: Variable divided by 2 to compensate partially                    */
		/*       CPU processing cycles (depends on compilation optimization).     */
		/* Note: If system core clock frequency is below 200kHz, wait time        */
		/*       is only a few CPU processing cycles.                             */
		wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
		while(wait_loop_index != 0)
		{
			wait_loop_index--;
		}

		/* Disable ADC DMA transfer request during calibration */
		/* Note: Specificity of this STM32 series: Calibration factor is           */
		/*       available in data register and also transferred by DMA.           */
		/*       To not insert ADC calibration factor among ADC conversion data   */
		/*       in DMA destination address, DMA transfer must be disabled during */
		/*       calibration.                                                     */
		backup_setting_adc_dma_transfer = LL_ADC_REG_GetDMATransfer(ADC1);
		LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_NONE);

		/* Run ADC self calibration */
		LL_ADC_StartCalibration(ADC1);

		/* Poll for ADC effectively calibrated */
#if (USE_TIMEOUT == 1)
		Timeout = ADC_CALIBRATION_TIMEOUT_MS;
#endif /* USE_TIMEOUT */

		while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0)
		{
#if (USE_TIMEOUT == 1)
			/* Check Systick counter flag to decrement the time-out value */
			if (LL_SYSTICK_IsActiveCounterFlag())
			{
				if(Timeout-- == 0)
				{
					/* Error: Time-out */
					Error_Handler();
				}
			}
#endif /* USE_TIMEOUT */
		}

		/* Restore ADC DMA transfer request after calibration */
		LL_ADC_REG_SetDMATransfer(ADC1, backup_setting_adc_dma_transfer);

		/* Delay between ADC end of calibration and ADC enable.                   */
		/* Note: Variable divided by 2 to compensate partially                    */
		/*       CPU processing cycles (depends on compilation optimization).     */
		wait_loop_index = (ADC_DELAY_CALIB_ENABLE_CPU_CYCLES >> 1);
		while(wait_loop_index != 0)
		{
			wait_loop_index--;
		}

		/* Enable ADC */
		LL_ADC_Enable(ADC1);


		/* Poll for ADC ready to convert */
#if (USE_TIMEOUT == 1)
		Timeout = ADC_ENABLE_TIMEOUT_MS;
#endif /* USE_TIMEOUT */

		while (LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0)
		{
#if (USE_TIMEOUT == 1)
			/* Check Systick counter flag to decrement the time-out value */
			if (LL_SYSTICK_IsActiveCounterFlag())
			{
				if(Timeout-- == 0)
				{
					/* Error: Time-out */
					Error_Handler();
				}
			}
#endif /* USE_TIMEOUT */
		}

		/* Note: ADC flag ADRDY is not cleared here to be able to check ADC       */
		/*       status afterwards.                                               */
		/*       This flag should be cleared at ADC Deactivation, before a new    */
		/*       ADC activation, using function "LL_ADC_ClearFlag_ADRDY()".       */
	}

	/*## Operation on ADC hierarchical scope: ADC group regular ################*/
	/* Note: No operation on ADC group regular performed here.                  */
	/*       ADC group regular conversions to be performed after this function  */
	/*       using function:                                                    */
	/*       "LL_ADC_REG_StartConversion();"                                    */

	/*## Operation on ADC hierarchical scope: ADC group injected ###############*/
	/* Note: Feature not available on this STM32 series */
}
