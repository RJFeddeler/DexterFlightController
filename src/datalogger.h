#ifndef _DATALOGGER_H
#define _DATALOGGER_H


#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "misc.h"

#include "state.h"
#include "sdcard.h"
#include "other_stuff.h"


#define DATALOGGER_SAMPLE_RATE		100		// 100 Hz


uint8_t				datalogger_last_state;


uint16_t	datalogger_config();
uint16_t	datalogger_init();
int8_t		datalogger_update();
void		datalogger_change_state(uint8_t new_state);


#endif
