#ifndef INC_TIM_CUSTOM_H_
#define INC_TIM_CUSTOM_H_

typedef void (*VoidCallback)(void);

int8_t TIMx_delay_us(TIM_TypeDef *TIMx, uint16_t us);
int8_t TIMx_restart(TIM_TypeDef *TIMx);
int8_t TIMx_stop(TIM_TypeDef *TIMx);
int8_t TIMx_get_count(TIM_TypeDef *TIMx, uint16_t* count);
int8_t TIMx_schedule_interrupt(TIM_TypeDef *TIMx, uint16_t us, VoidCallback callback);
int8_t TIMx_disable_scheduled_interrupt(TIM_TypeDef *TIMx);
void TIM_IRQ_callback(TIM_TypeDef *TIMx);

#endif
