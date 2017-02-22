#ifndef _NEOM8N_H
#define _NEOM8N_H


#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "misc.h"

#include "state.h"
#include "clocks.h"
#include "other_stuff.h"


uint8_t				neom8n_last_state;


uint16_t	neom8n_config();
uint16_t	neom8n_init();
int8_t		neom8n_update();
void		neom8n_change_state(uint8_t new_state);


#endif
