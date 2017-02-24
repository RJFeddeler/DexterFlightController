#ifndef _MOTOR_H
#define _MOTOR_H


#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "misc.h"

#include "state.h"


#define MOTOR_SAMPLE_RATE		400

#define MOTOR_COUNT			6
#define MOTOR_1				STATE_MOTOR_1
#define MOTOR_2				STATE_MOTOR_2
#define MOTOR_3				STATE_MOTOR_3
#define MOTOR_4				STATE_MOTOR_4
#define MOTOR_5				STATE_MOTOR_5
#define MOTOR_6				STATE_MOTOR_6

#define MOTOR_TIM1_FREQ			168
#define MOTOR_TIM3_FREQ			84
#define MOTOR_RESOLUTION_DIV		21
#define MOTOR_PERIOD			2500
#define MOTOR_PULSE_ARM			1000
#define MOTOR_PULSE_MIN			1068
#define MOTOR_PULSE_MAX			1868


uint8_t			motor_last_state;
static const float 	motor_mixer_Hex6X[][3] = {	{ 0.866025f,	-0.5f,		-1.0f	},	// FRONT_R
							{ 0.866025f, 	0.5f,		1.0f	},	// FRONT_L
							{ 0.0f,  	1.0f,		-1.0f	},	// LEFT
							{ -0.866025f, 	0.5f, 		1.0f	},	// REAR_L
							{ -0.866025f, 	-0.5f, 		-1.0f	},	// REAR_R
							{ 0.0f, 	-1.0f,     	1.0f	}};	// RIGHT


uint16_t	motor_config();
uint16_t	motor_init();
int8_t		motor_update();
void		motor_change_state(uint8_t new_state);
int8_t		motor_mixer(const float mix[][3]);
int8_t 		motor_set(uint8_t motor, uint16_t pulse);
int8_t 		motor_set_all(uint16_t pulse);
int8_t 		motor_reset(uint8_t motor);
int8_t 		motor_reset_all();
int8_t 		motor_arm(uint8_t motor);
int8_t 		motor_arm_all();


#endif
