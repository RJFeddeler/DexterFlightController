#ifndef _OTHER_STUFF_H
#define _OTHER_STUFF_H


#include <stdbool.h>
#include "stm32f4xx.h"


struct Color {
	uint8_t red;
	uint8_t green;
	uint8_t blue;
	uint8_t brightness;
};


uint32_t 	power(uint32_t base, uint32_t exp);
int32_t		abs(int32_t value);
float		abs_f(float value);
uint16_t 	min(int16_t value1, int16_t value2);
int16_t		min_array(int16_t *values, uint16_t length);
uint16_t 	max(int16_t value1, int16_t value2);
int16_t		max_array(int16_t *values, uint16_t length);
int32_t		scale(int32_t start_scale_value, int32_t start_scale_min, int32_t start_scale_max, int32_t end_scale_min, int32_t end_scale_max);
float		scalef(float start_scale_value, float start_scale_min, float start_scale_max, float end_scale_min, float end_scale_max);
void		scale_array(int16_t *values, uint16_t length, float scale);
int32_t	 	clamp(int32_t value, int32_t min, int32_t max);
float	 	clampf(float value, float min, float max);
void 	 	mem_copy(uint8_t *dest, const uint8_t *source, uint16_t length);
bool	 	mem_cmp(const uint8_t *dest, const uint8_t *source, uint16_t length);
void		low_pass_filter(float lpf_beta, float raw_value, float *filtered_value);
float 	 	c_to_f(float c);
float	 	f_to_c(float f);
float	 	m_to_ft(float m);
float	 	ft_to_m(float ft);


#endif
