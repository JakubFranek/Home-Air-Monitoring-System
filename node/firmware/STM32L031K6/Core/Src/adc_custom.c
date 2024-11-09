#include "adc_custom.h"
#include "tim_custom.h"
#include "adc.h"

#define ADC_TIMEOUT_US 		1000
#define ADC_TIMEOUT_TIMER 	TIM2

uint32_t vrefint_cal_coefficient;

static uint16_t adc_output;

int8_t ADC_setup()
{
	uint16_t timer_count;

	vrefint_cal_coefficient = VREFINT_CAL_VREF * (*VREFINT_CAL_ADDR);

	TIMx_restart(ADC_TIMEOUT_TIMER);

	LL_ADC_StartCalibration(ADC1);
	while (LL_ADC_IsCalibrationOnGoing(ADC1))	// Wait until calibration is complete
	{
		TIMx_get_count(ADC_TIMEOUT_TIMER, &timer_count);
		if (timer_count > ADC_TIMEOUT_US)
			return -1;
	}

	LL_ADC_SetLowPowerMode(ADC1, LL_ADC_LP_AUTOPOWEROFF);	// ADC will automatically power-off after conversion

	return 0;
}

int8_t ADC_measure_vdda(uint16_t* vdda_mv)
{
	uint16_t timer_count;

	LL_ADC_EnableInternalRegulator(ADC1);

	TIMx_restart(ADC_TIMEOUT_TIMER);
	LL_ADC_REG_StartConversion(ADC1);		// Start ADC conversion
	while (!LL_ADC_IsActiveFlag_EOC(ADC1))	// Poll for conversion completion
	{
		TIMx_get_count(ADC_TIMEOUT_TIMER, &timer_count);
		if (timer_count > ADC_TIMEOUT_US)
			return -1;
	}

	adc_output = LL_ADC_REG_ReadConversionData12(ADC1);

	LL_ADC_ClearFlag_EOC(ADC1);
	LL_ADC_DisableInternalRegulator(ADC1);

	*vdda_mv = (uint16_t) (vrefint_cal_coefficient / adc_output);

	return 0;
}

