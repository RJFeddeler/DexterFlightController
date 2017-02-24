#ifndef _SERIALTX_H
#define _SERIALTX_H


#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_dma.h"
#include "misc.h"

#include "onboard_led.h"
#include "other_stuff.h"


#define SERIALTX_BUFFER_SIZE		500
#define SERIALTX_ERROR_CODE		0x800


uint8_t 		serialtx_dma_buffer[SERIALTX_BUFFER_SIZE];
bool			serialtx_busy;


uint16_t 	serialtx_config();
bool 		serialtx_get_busy();
int8_t		serialtx_send(uint16_t num_bytes);
int8_t 		serialtx_send_dec_int32(int32_t data);
int8_t 		serialtx_send_dec_x8r(uint16_t *channels, uint8_t num_chans);
int8_t		serialtx_send_processing(uint8_t status, uint16_t *x8r, int32_t *bno, uint16_t *motor);
int8_t		serialtx_send_processing_with_leds(uint8_t status, uint16_t *x8r, int32_t *bno, uint16_t *motor, uint8_t *ws2812);
int8_t 		serialtx_clear_buffer();


#endif
