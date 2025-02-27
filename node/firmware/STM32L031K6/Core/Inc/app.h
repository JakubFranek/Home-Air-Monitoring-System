#ifndef INC_APP_H_
#define INC_APP_H_

#define NRF24_DATA_LENGTH 10		// bytes
#define NODE_ID 0
#define NRF24_AUTO_RETRANSMIT_DELAY 2*NODE_ID
#define RTC_WAKE_UP_PERIOD_S 10		// min. 1 second

void app_setup(void);
void app_loop(void);
void GPIO_EXTI15_IRQ_callback(void);
void GPIO_EXTI4_IRQ_callback(void);
void RTC_WAKEUP_IRQ_callback(void);

#endif /* INC_APP_H_ */
