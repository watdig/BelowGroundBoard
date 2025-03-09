/*
 * sensor_adc.h
 *
 *  Created on: Mar 8, 2025
 *      Author: Victor Kalenda
 */

#ifndef INC_SENSOR_ADC_H_
#define INC_SENSOR_ADC_H_

void ADC_Activate();
void ADC_ConvCpltCallback();
void ADC_ConvHalfCpltCallback();
void ADC_ErrorCallback();

#endif /* INC_SENSOR_ADC_H_ */
