#ifndef _I2C_HELPER_H
#define _I2C_HELPER_H


#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_i2c.h"

#include "clocks.h"


int8_t	i2c_helper_read_byte	(I2C_TypeDef *I2Cx, uint8_t device, uint8_t reg, uint8_t  *data);
int8_t 	i2c_helper_read_word	(I2C_TypeDef *I2Cx, uint8_t device, uint8_t reg, uint16_t  *data);
int8_t 	i2c_helper_read_bytes	(I2C_TypeDef *I2Cx, uint8_t device, uint8_t reg, uint8_t length, uint8_t  *data);
int8_t 	i2c_helper_write_byte	(I2C_TypeDef *I2Cx, uint8_t device, uint8_t reg, uint8_t  data);
int8_t 	i2c_helper_write_word	(I2C_TypeDef *I2Cx, uint8_t device, uint8_t reg, uint16_t  data);
int8_t 	i2c_helper_write_bytes	(I2C_TypeDef *I2Cx, uint8_t device, uint8_t reg, uint8_t length, const uint8_t  *data);
int8_t 	i2c_helper_write_bit	(I2C_TypeDef *I2Cx, uint8_t device, uint8_t reg, uint8_t loc, uint8_t data);
int8_t 	i2c_helper_write_bits	(I2C_TypeDef *I2Cx, uint8_t device, uint8_t reg, uint8_t start, uint8_t length, uint8_t data);
int8_t 	i2c_helper_unstick	(GPIO_TypeDef *port, uint16_t scl_pin, uint16_t sda_pin);


#endif
