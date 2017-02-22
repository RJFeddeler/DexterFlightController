#ifndef _HCSR05_H
#define _HCSR05_H


#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "misc.h"

#include "state.h"
#include "serialtx.h"
#include "other_stuff.h"


#define HCSR05_SAMPLE_RATE					25			// 25 Hz
#define HCSR05_FILTER_COUNT					9			// Median filter window size


uint8_t				hcsr05_last_state;
uint32_t			hcsr05_counter;

bool				hcsr05_listening;
bool				hcsr05_data_ready;

float				hcsr05_offset;
float				hcsr05_avg_sum;

uint32_t			hcsr05_raw;
uint32_t			hcsr05_raw_history[HCSR05_FILTER_COUNT];
float				hcsr05_cooked;

uint16_t	hcsr05_config();
uint16_t	hcsr05_init();
int8_t		hcsr05_update();
void		hcsr05_change_state(uint8_t new_state);
uint32_t	hcsr05_median_filter(uint32_t value);
int8_t		hcsr05_send_pulse();
int8_t 		hcsr05_reset_echo();


#endif
