#ifndef _CLOCKS_H
#define _CLOCKS_H


#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_flash.h"
#include "misc.h"

#include "other_stuff.h"


#define CLOCKS_ERROR_CODE		0x200


static volatile uint32_t	sys_ticks;


void 	 	SysTick_Handler(void);
uint16_t	clocks_systick_config();
uint16_t  	clocks_clock_config();
uint32_t 	clocks_get_ticks();
uint32_t 	clocks_get_millis();
int8_t 		clocks_check_sample_rate(uint32_t *last_update, uint16_t rate);
void	 	delay(uint32_t delay_ms);
void 	 	delay_us(uint32_t delay_ticks);
void		timed_out_init(uint32_t *start_tick);
bool		timed_out(uint32_t start_tick, uint32_t time_limit);
bool		timed_out_us(uint32_t start_tick, uint32_t time_limit);


#endif
