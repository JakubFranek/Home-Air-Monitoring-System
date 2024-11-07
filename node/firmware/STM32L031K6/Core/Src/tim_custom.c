#include <stdint.h>
#include <stddef.h>
#include "main.h"
#include "tim_custom.h"

static VoidCallback tim2_callback = NULL;
static VoidCallback tim21_callback = NULL;
static VoidCallback tim22_callback = NULL;

int8_t TIMx_delay_us(TIM_TypeDef *TIMx, uint16_t us)
{
	uint16_t ticks = us;

	LL_TIM_DisableCounter(TIMx);
	LL_TIM_SetCounter(TIMx, 0);
	LL_TIM_EnableCounter(TIMx);
	while (LL_TIM_GetCounter(TIMx) < ticks)
		;
	LL_TIM_DisableCounter(TIMx);

	return 0;
}

int8_t TIMx_restart(TIM_TypeDef *TIMx)
{
	LL_TIM_DisableCounter(TIMx);
	LL_TIM_SetCounter(TIMx, 0);
	LL_TIM_EnableCounter(TIMx);

	return 0;
}

int8_t TIMx_stop(TIM_TypeDef *TIMx)
{
	LL_TIM_DisableCounter(TIMx);

	return 0;
}

int8_t TIMx_get_count(TIM_TypeDef *TIMx, uint16_t* count)
{
	*count = LL_TIM_GetCounter(TIMx);
	return 0;
}

int8_t TIMx_schedule_interrupt(TIM_TypeDef *TIMx, uint16_t us, VoidCallback callback)
{
	LL_TIM_DisableCounter(TIMx);
	LL_TIM_SetCounter(TIMx, 0);
	LL_TIM_SetAutoReload(TIMx, us);

	if (TIMx == TIM2)
		tim2_callback = callback;
	else if (TIMx == TIM21)
		tim21_callback = callback;
	else if (TIMx == TIM22)
		tim22_callback = callback;
	else
		return -1;

	LL_TIM_ClearFlag_UPDATE(TIMx); // this is needed for unknown reason (unclear why flag is re-set)
	LL_TIM_EnableIT_UPDATE(TIMx);
	LL_TIM_EnableCounter(TIMx);

	return 0;
}

int8_t TIMx_disable_scheduled_interrupt(TIM_TypeDef *TIMx)
{
	LL_TIM_DisableCounter(TIMx);
	LL_TIM_DisableIT_UPDATE(TIMx);
	LL_TIM_ClearFlag_UPDATE(TIMx);

	if (TIMx == TIM2)
		tim2_callback = NULL;
	else if (TIMx == TIM21)
		tim21_callback = NULL;
	else if (TIMx == TIM22)
		tim22_callback = NULL;
	else
		return -1;

	return 0;
}

void TIM_IRQ_callback(TIM_TypeDef *TIMx)
{
	if (TIMx == TIM2 && tim2_callback != NULL)
		tim2_callback();
	else if (TIMx == TIM21 && tim21_callback != NULL)
		tim21_callback();
	else if (TIMx == TIM22 && tim22_callback != NULL)
		tim22_callback();

	TIMx_disable_scheduled_interrupt(TIMx);
}
