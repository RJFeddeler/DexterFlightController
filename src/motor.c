#include "motor.h"

/** Interrupt handler sets the duty cycle for the next PWM signal
  * sent to the ESCs for motors 1 to 4.
  *
  * @return Void.
***/
void TIM1_CC_IRQHandler(void) {
	if (TIM_GetITStatus(TIM1, TIM_IT_CC1)) {
		TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);
		TIM_SetCompare1(TIM1, state.motors[MOTOR_1] * MOTOR_RESOLUTION_DIV);
	}

	if (TIM_GetITStatus(TIM1, TIM_IT_CC2)) {
		TIM_ClearITPendingBit(TIM1, TIM_IT_CC2);
		TIM_SetCompare2(TIM1, state.motors[MOTOR_2] * MOTOR_RESOLUTION_DIV);
	}
	if (TIM_GetITStatus(TIM1, TIM_IT_CC3)) {
		TIM_ClearITPendingBit(TIM1, TIM_IT_CC3);
		TIM_SetCompare3(TIM1, state.motors[MOTOR_3] * MOTOR_RESOLUTION_DIV);
	}

	if (TIM_GetITStatus(TIM1, TIM_IT_CC4)) {
		TIM_ClearITPendingBit(TIM1, TIM_IT_CC4);
		TIM_SetCompare4(TIM1, state.motors[MOTOR_4] * MOTOR_RESOLUTION_DIV);
	}
}

/** Interrupt handler sets the duty cycle for the next PWM signal
  * sent to the ESCs for motors 5 to 6.
  *
  * @return Void.
***/
void TIM3_IRQHandler(void) {
	if (TIM_GetITStatus(TIM3, TIM_IT_CC3)) {
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);
		TIM_SetCompare3(TIM3, state.motors[MOTOR_5] * MOTOR_RESOLUTION_DIV);
	}

	if (TIM_GetITStatus(TIM3, TIM_IT_CC4)) {
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC4);
		TIM_SetCompare4(TIM3, state.motors[MOTOR_6] * MOTOR_RESOLUTION_DIV);
	}
}

/** Enable clocks, configure GPIO pins E9, E11, E13, and E14 as Alternate Function (TIM1),
  * configure GPIO pins B0 and B1 as Alternate Function (TIM3),
  * enable capture/compare interrupts, and configure two timers for PWM.
  *
  * @return The status of configuration.
***/
uint16_t motor_config() {
	GPIO_InitTypeDef 	GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef	TIM_TimeBaseStruct;
	TIM_OCInitTypeDef  	TIM_OCInitStruct;
	NVIC_InitTypeDef 	NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	GPIO_PinAFConfig(GPIOE, GPIO_PinSource9,  GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3);

	GPIO_InitStructure.GPIO_Pin = 	GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = 	GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed =	GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType =	GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = 	GPIO_PuPd_DOWN;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = 	GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_Init(&NVIC_InitStructure);

	TIM_DeInit(TIM1);
	TIM_TimeBaseStruct.TIM_Prescaler = (MOTOR_TIM1_FREQ / MOTOR_RESOLUTION_DIV) - 1;
	TIM_TimeBaseStruct.TIM_Period = (MOTOR_PERIOD * MOTOR_RESOLUTION_DIV) - 1;
	TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStruct);

	TIM_DeInit(TIM3);
	TIM_TimeBaseStruct.TIM_Prescaler = (MOTOR_TIM3_FREQ / MOTOR_RESOLUTION_DIV) - 1;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStruct);

	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_Pulse = 0;
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OC1Init(TIM1, &TIM_OCInitStruct);
	TIM_OC2Init(TIM1, &TIM_OCInitStruct);
	TIM_OC3Init(TIM1, &TIM_OCInitStruct);
	TIM_OC4Init(TIM1, &TIM_OCInitStruct);
	TIM_OC3Init(TIM3, &TIM_OCInitStruct);
	TIM_OC4Init(TIM3, &TIM_OCInitStruct);

	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM1, ENABLE);
	TIM_ARRPreloadConfig(TIM3, ENABLE);

	TIM_ITConfig(TIM1, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4, ENABLE);
	TIM_ITConfig(TIM3, TIM_IT_CC3 | TIM_IT_CC4, ENABLE);

	// Enable main output (Needed for advanced timers)
	TIM_CtrlPWMOutputs(TIM1, ENABLE);

	return 0;
}

/** Initialize variables and prepare to begin normal operation.
  *
  * @return The status of initialization.
***/
uint16_t motor_init() {
	motor_last_state = 0;
	pid_reset();

	return 0;
}

/** Update loop to handle state switching and control implementation.
  *
  * @return The status of current update iteration.
***/
int8_t motor_update() {
	static uint32_t last_update = 0;

	if (clocks_check_sample_rate(&last_update, MOTOR_SAMPLE_RATE)) {
		if (state.status != motor_last_state) {
			motor_change_state(state.status);
			motor_last_state = state.status;
		}

		if (state.status == STATE_SYS_ARMED) {
			pid_update();
			motor_mixer(motor_mixer_Hex6X);

			return 1;
		}
	}

	return 0;
}

/** Used to change settings, enable/disable features, or trigger actions when
  * switching between two different states of operation.
  *
  * @param new_state:	The state being switched to.
  *
  * @return Void.
***/
void motor_change_state(uint8_t new_state) {
	switch (new_state) {
	case STATE_SYS_CALIBRATING:
		motor_reset_all();
		state_set_calibrated(STATE_PERIPH_MOTOR);

		TIM_Cmd(TIM1, DISABLE);
		TIM_Cmd(TIM3, DISABLE);

		break;
	case STATE_SYS_IDLE:
		motor_reset_all();

		TIM_Cmd(TIM1, DISABLE);
		TIM_Cmd(TIM3, DISABLE);

		break;
	case STATE_SYS_ARMING:
		motor_arm_all();

		TIM_Cmd(TIM1, ENABLE);
		TIM_Cmd(TIM3, ENABLE);

		break;
	case STATE_SYS_ARMED:
		motor_set_all(MOTOR_PULSE_MIN);

		break;
	}
}

/** Applies the correct adjustments to each motor to reach desired attitude and position.
  *
  * @param mix:		2D array of pitch, roll, and yaw ratios for each motor
  * 			(each unique multicopter configuration needs its own mixer values).
  *
  * @return The status of the function.
***/
int8_t motor_mixer(const float mix[][3]) {
	uint16_t motor_output[MOTOR_COUNT];
	int32_t throttle = state_get_throttle();

	for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
		motor_output[i] = (uint16_t)((float)throttle +
				((mix[i][STATE_AXIS_PITCH] * state.pid[STATE_AXIS_PITCH].value) +
				(mix[i][STATE_AXIS_ROLL] * state.pid[STATE_AXIS_ROLL].value) +
				(mix[i][STATE_AXIS_YAW] * state.pid[STATE_AXIS_YAW].value)));

		motor_output[i] = clamp(motor_output[i], MOTOR_PULSE_MIN, MOTOR_PULSE_MAX);
	}

	for (uint8_t i = 0; i < MOTOR_COUNT; i++)
		motor_set(i, motor_output[i]);

	return 0;
}

/** Set the speed of a given motor.
  *
  * @param motor:	The motor number to change the speed of.
  * @param pulse:	The length of time, in uS, that the PWM pulse for the motor is high.
  *
  * @return The status of the function.
***/
int8_t motor_set(uint8_t motor, uint16_t pulse) {
	if ((pulse != MOTOR_PULSE_ARM) && (pulse != 0))
		pulse = (uint16_t)clamp((int32_t)pulse, MOTOR_PULSE_MIN, MOTOR_PULSE_MAX);

	state.motors[motor] = pulse;

	return 0;
}

/** Set the pulse of every motor to the same given value.
  *
  * @param pulse:	The length of time, in uS, that the PWM pulses for the motors are high.
  *
  * @return The status of the function.
***/
int8_t motor_set_all(uint16_t pulse) {
	for (uint8_t i = 0; i < MOTOR_COUNT; i++)
		motor_set(i, pulse);

	return 0;
}

/** Set the pulse of a given motor to 0 (off).
  *
  * @param motor:	The motor number to turn off.
  *
  * @return The status of the function.
***/
int8_t motor_reset(uint8_t motor) {
	return motor_set(motor, 0);
}

/** Set the pulse of every motor to 0 (off).
  *
  * @return The status of the function.
***/
int8_t motor_reset_all() {
	return motor_set_all(0);
}

/** Set the pulse of a given motor to the arming pulse (off but non-zero).
  *
  * @param motor:	The motor number to change the pulse of.
  *
  * @return The status of the function.
***/
int8_t motor_arm(uint8_t motor) {
	return motor_set(motor, MOTOR_PULSE_ARM);
}

/** Set the pulse of every motor to the arming pulse (off but non-zero).
  *
  * @return The status of the function.
***/
int8_t motor_arm_all() {
	return motor_set_all(MOTOR_PULSE_ARM);
}
