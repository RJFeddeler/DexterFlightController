#ifndef _ONBOARD_LED_H
#define _ONBOARD_LED_H


#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "clocks.h"


#define ONBOARD_LED_RCC_GPIO			RCC_AHB1Periph_GPIOD
#define ONBOARD_LED_PORT			GPIOD
#define ONBOARD_LED_ERROR_CODE			0x400

#define ONBOARD_LED_GREEN			GPIO_Pin_12
#define ONBOARD_LED_ORANGE			GPIO_Pin_13
#define ONBOARD_LED_RED				GPIO_Pin_14
#define ONBOARD_LED_BLUE			GPIO_Pin_15

#define ONBOARD_LED_DELAY_SINGLE		250
#define ONBOARD_LED_DELAY_FOREVER		100


uint16_t 	onboard_led_config();
void 		onboard_led_toggle(uint16_t color);
void		onboard_led_set(uint16_t color);
bool		onboard_led_get(uint16_t color);
void		onboard_led_reset(uint16_t color);
void		onboard_led_reset_all();
void 		onboard_led_blink(uint16_t color);
void 		onboard_led_blink_forever();


#endif
