#ifndef INC_TIM_CUSTOM_H_
#define INC_TIM_CUSTOM_H_

#include "tim.h"

typedef void (*VoidCallback)(void);

void TIMx_delay_us(TIM_TypeDef *TIMx, uint16_t us);
void TIMx_restart(TIM_TypeDef *TIMx);
void TIMx_stop(TIM_TypeDef *TIMx);
void TIMx_get_count(TIM_TypeDef *TIMx, uint16_t* count);
int8_t TIMx_schedule_interrupt(TIM_TypeDef *TIMx, uint16_t us, VoidCallback callback);
int8_t TIMx_disable_scheduled_interrupt(TIM_TypeDef *TIMx);
void TIM_IRQ_callback(TIM_TypeDef *TIMx);

#endif
