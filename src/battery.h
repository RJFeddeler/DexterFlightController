#ifndef _BATTERY_H
#define _BATTERY_H


#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_adc.h"
#include "misc.h"

#include "state.h"
#include "serialtx.h"
#include "other_stuff.h"


#define BATTERY_SAMPLE_RATE				400
#define	BATTERY_COUNT					2

#define	BATTERY_THRESHOLD_WARNING		0		// Set this
#define BATTERY_THRESHOLD_ALARM			0		// Set this


uint8_t				battery_last_state;
uint16_t			battery_dma_buffer[2];
float				battery_voltage;
float				battery_amperage;


uint16_t	battery_config();
uint16_t	battery_init();
int8_t		battery_update();
void		battery_change_state(uint8_t new_state);


#endif
