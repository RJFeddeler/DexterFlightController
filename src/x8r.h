#ifndef _X8R_H
#define _X8R_H


#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_dma.h"
#include "misc.h"

#include "state.h"
#include "other_stuff.h"


#define X8R_FRAME_SIZE			25
#define X8R_NUM_CHANNELS		16

#define X8R_VALUE_MIN			STATE_RX_VALUE_MIN
#define X8R_VALUE_MAX			STATE_RX_VALUE_MAX

#define X8R_FLAG_BYTE			23
#define X8R_START_BYTE			0x0F
#define X8R_END_BYTE			0x00

#define X8R_FLAG_BIT_DIGITAL1		0x01
#define X8R_FLAG_BIT_DIGITAL2		0x02
#define X8R_FLAG_BIT_LOST		0x04
#define X8R_FLAG_BIT_FAILSAFE		0x08

#define X8R_CHANNEL_THROTTLE		STATE_RX_CHANNEL_THROTTLE
#define X8R_CHANNEL_AILERON		STATE_RX_CHANNEL_AILERON
#define X8R_CHANNEL_ELEVATOR		STATE_RX_CHANNEL_ELEVATOR
#define X8R_CHANNEL_RUDDER		STATE_RX_CHANNEL_RUDDER
#define X8R_CHANNEL_TOGGLE_L_3POS_1	STATE_RX_CHANNEL_TOGGLE_L_3POS_1
#define X8R_CHANNEL_TOGGLE_L_3POS_2 	STATE_RX_CHANNEL_TOGGLE_L_3POS_2
#define X8R_CHANNEL_TOGGLE_L_3POS_3 	STATE_RX_CHANNEL_TOGGLE_L_3POS_3
#define X8R_CHANNEL_TOGGLE_R_3POS_1 	STATE_RX_CHANNEL_TOGGLE_R_3POS_1
#define X8R_CHANNEL_TOGGLE_R_3POS_2 	STATE_RX_CHANNEL_TOGGLE_R_3POS_2
#define X8R_CHANNEL_TOGGLE_R_3POS_3	STATE_RX_CHANNEL_TOGGLE_R_3POS_3
#define X8R_CHANNEL_TOGGLE_L_2POS	STATE_RX_CHANNEL_TOGGLE_L_2POS
#define X8R_CHANNEL_TOGGLE_R_2POS	STATE_RX_CHANNEL_TOGGLE_R_2POS
#define X8R_CHANNEL_TOGGLE_L_KNOB	STATE_RX_CHANNEL_TOGGLE_L_KNOB
#define X8R_CHANNEL_TOGGLE_R_KNOB	STATE_RX_CHANNEL_TOGGLE_R_KNOB
#define X8R_CHANNEL_TOGGLE_L_SLIDER 	STATE_RX_CHANNEL_TOGGLE_L_SLIDER
#define X8R_CHANNEL_TOGGLE_R_SLIDER 	STATE_RX_CHANNEL_TOGGLE_R_SLIDER

#define X8R_SIGNAL_OK			0
#define X8R_SIGNAL_LOST			1
#define X8R_SIGNAL_FAILSAFE		2


uint8_t			x8r_last_state;
bool 			x8r_frame_ready;
bool			x8r_dma_synced;
uint8_t			x8r_dma_buffer[X8R_FRAME_SIZE];
uint32_t		x8r_timedout_start;
uint8_t			x8r_flags;
uint8_t			x8r_digital1;
uint8_t			x8r_digital2;
uint8_t			x8r_status;

volatile uint16_t	x8r_reload_value;
volatile uint32_t	x8r_last_update;


uint8_t		x8r_config();
uint8_t		x8r_init();
int8_t		x8r_update();
void		x8r_change_state(uint8_t new_state);
int8_t		x8r_parse_frame();


#endif
