/*
 * adc_handler.c
 *
 *  Created on: 26 wrz 2021
 *      Author: Marek Jaromin
 */


#include "adc_handler.h"

volatile uint32_t temperature_status = CORRECT_TEMPERATURE;


/**
  * @brief CONV_ADC_VOLTAGE_TO_TEMPERATURE
  * @param VALUE READ FROM ADC CONVERTER
  * @retval TEMPERATURE VALUE IN CELSIUS DEGREES
  */
float CONV_ADC_VOLTAGE_TO_TEMPERATURE(uint16_t adc_read_value){

	float voltage = (ADC_REFERENCE_VOLTAGE*adc_read_value)/ADC_RESOLUTION;
	float temperature = voltage/VOLTAGE_RISE;
	return temperature;
}
