#include "hcsr05.h"
// WORK IN PROGRESS!!!

// NOTE TO SELF: Set starting offset inside library and whenever setting or returning altitude/distance apply offset first.

/** Interrupt handler reads the timer count, resets listening status, and
  * is triggered when the peripheral receives the sent out signal back.
  *
  * @return Void.
***/
void TIM2_IRQHandler(void) {
	if (TIM_GetITStatus(TIM2, TIM_IT_CC2)) {
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);

		hcsr05_raw = TIM_GetCapture1(TIM2);
		hcsr05_listening = false;
		hcsr05_data_ready = true;
	}
}

/** Enable clocks, configure GPIO pin B3 (echo) as Alternate Function (TIM2_CH2)
  * and pin B4 (trigger) as Output, configure Timer 2, and enable capture/compare interrupt.
  *
  * @return The status of configuration.
***/
uint16_t hcsr05_config() {
	GPIO_InitTypeDef	GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef	TIM_TimeBaseStruct;
	TIM_ICInitTypeDef	TIM_ICInitStruct;
	NVIC_InitTypeDef 	NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_TIM2);

	GPIO_InitStructure.GPIO_Pin = 	GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = 	GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed =	GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType =	GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = 	GPIO_PuPd_DOWN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = 	GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = 	GPIO_Mode_OUT;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_TimeBaseStruct.TIM_Prescaler = 83;			// 1 MHz (tick every microsecond)
	TIM_TimeBaseStruct.TIM_Period = 0xFFFFFFFF;		// TIMER2 is 32-bit
	TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStruct);

	TIM_ICInitStruct.TIM_Channel = TIM_Channel_2;
	TIM_ICInitStruct.TIM_ICFilter = 0;
	TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_PWMIConfig(TIM2, &TIM_ICInitStruct);

	TIM_SelectInputTrigger(TIM2, TIM_TS_TI2FP2);
	TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset);
	TIM_SelectMasterSlaveMode(TIM2, TIM_MasterSlaveMode_Enable);

	TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);

	return 0;
}

/** Initialize variables and prepare to begin normal operation.
  *
  * @return The status of initialization.
***/
uint16_t hcsr05_init() {
	hcsr05_offset = 0;
	hcsr05_avg_sum = 0;
	hcsr05_data_ready = false;
	hcsr05_listening = false;

	for (uint8_t i = 0; i < HCSR05_FILTER_COUNT; i++)
		hcsr05_raw_history[i] = 0;

	hcsr05_cooked = 0.0f;

	TIM_Cmd(TIM2, ENABLE);

	hcsr05_last_state = 0;
	hcsr05_counter = 0;

	return 0;
}

/** Update loop to handle state switching, starting and checking the signal,
  * and conversion from raw to cooked data.
  *
  * @return The status of current update iteration.
***/
int8_t hcsr05_update() {
	static uint32_t last_update = 0;
	uint32_t filtered_val = 0;

	if (state.status != hcsr05_last_state) {
		hcsr05_change_state(state.status);
		hcsr05_last_state = state.status;
	}

	if (hcsr05_data_ready) {
		hcsr05_data_ready = false;

		filtered_val = hcsr05_median_filter(hcsr05_raw);
		low_pass_filter(0.25f, (filtered_val * 0.000171821f), &hcsr05_cooked);

		if (state.status == STATE_SYS_ARMED) {
			//hcsr05_cooked -= hcsr05_offset; ///////////// DISABLING OFFSET FOR TESTING
		}

		state.alt_us = hcsr05_cooked;

		if (state.status == STATE_SYS_ARMING)
			hcsr05_avg_sum += (filtered_val * 0.000171821f);

		if (state.status >= STATE_SYS_ARMING)
			hcsr05_counter++;

		return 1;
	}

	if (clocks_check_sample_rate(&last_update, HCSR05_SAMPLE_RATE)) {
		if (hcsr05_listening)
			hcsr05_reset_echo();

		hcsr05_send_pulse();
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
void hcsr05_change_state(uint8_t new_state) {
	switch (new_state) {
	case STATE_SYS_CALIBRATING:
		state_set_calibrated(STATE_PERIPH_HCSR05);	// Add a check here
		break;
	case STATE_SYS_IDLE:
		break;
	case STATE_SYS_ARMING:
		break;
	case STATE_SYS_ARMED:
		hcsr05_offset = hcsr05_avg_sum / hcsr05_counter;
		break;
	}
}

/** Use a median filter with a window of 9 to filter noise.
  *
  * @param value:	The most recent value read from the peripheral,
  * 			used to replace the oldest value in the window.
  *
  * @return The median value of the current window.
***/
uint32_t hcsr05_median_filter(uint32_t value) {
	static uint32_t sorted[HCSR05_FILTER_COUNT];

	for (int8_t i = HCSR05_FILTER_COUNT - 2; i >= 0; i--) {
		hcsr05_raw_history[i + 1] = hcsr05_raw_history[i];
	}

	hcsr05_raw_history[0] = value;

	for (uint8_t i = 0; i < HCSR05_FILTER_COUNT; i++) // SWITCH TO MEMCOPY?
		sorted[i] = hcsr05_raw_history[i];

	uint8_t num_swaps = 1;
	uint32_t temp;
	while (num_swaps > 0) {
		num_swaps = 0;
		for (uint8_t i = 0; i < HCSR05_FILTER_COUNT - 1; i++) {
			if (sorted[i] > sorted[i + 1]) {
				temp = sorted[i];
				sorted[i] = sorted[i + 1];
				sorted[i + 1] = temp;

				num_swaps++;
			}

		}
	}

	return sorted[(uint8_t)(HCSR05_FILTER_COUNT / 2)];
}

/** Send a high pulse for 10 uS to trigger the sensor to start.
  *
  * @return The status of the function.
***/
int8_t hcsr05_send_pulse() {
	if (!hcsr05_listening) {
		hcsr05_listening = true;

		GPIO_ResetBits(GPIOB, GPIO_Pin_4);
		delay_us(5);
		GPIO_SetBits(GPIOB, GPIO_Pin_4);
		delay_us(10);
		GPIO_ResetBits(GPIOB, GPIO_Pin_4);

		return 0;
	}
	else
		return -2;
}

/** Reset GPIO pin B3 (echo) if an echo is never received and the line never resets for some reason.
  *
  * @return The status of the function.
***/
int8_t hcsr05_reset_echo() {
	GPIO_InitTypeDef 			GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = 	GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = 	GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed =	GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType =	GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = 	GPIO_PuPd_DOWN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_ResetBits(GPIOB, GPIO_Pin_3);
	delay(100);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_TIM2);

	GPIO_InitStructure.GPIO_Mode = 	GPIO_Mode_AF;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	return 0;
}
