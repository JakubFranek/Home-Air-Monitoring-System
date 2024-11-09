#ifndef INC_ADC_CUSTOM_H_
#define INC_ADC_CUSTOM_H_

#include <stdint.h>

#define ADC_OSR					32
#define ADC_SAMPLING_CYCLES		13		// actually 12.5, for sampling VREFINT should be typically 10 us (STM32L031K6 datasheet p.55)
#define ADC_CONVERSION_CYCLES	13		// actually 12.5
#define ADC_PRESCALER			4
#define ADC_CLOCK_MHZ			8
#define ADC_CONVERSION_TIME_US	ADC_OSR * (ADC_SAMPLING_CYCLES + ADC_CONVERSION_CYCLES - 1) / (ADC_CLOCK_MHZ / ADC_PRESCALER)


int8_t ADC_setup();
int8_t ADC_measure_vdda(uint16_t* vdda_mv);

#endif /* INC_ADC_CUSTOM_H_ */
