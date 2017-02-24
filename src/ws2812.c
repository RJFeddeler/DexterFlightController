#include "ws2812.h"

/** The interrupt handler triggers when a frame of data has been
  * fully transmitted to the LED strip.
  *
  * @return Void.
***/
void DMA1_Stream0_IRQHandler(void) {
	if (DMA_GetITStatus(DMA1_Stream0, DMA_IT_TCIF0) != RESET) {
		DMA_ClearITPendingBit(DMA1_Stream0, DMA_IT_TCIF0);
		DMA_Cmd(DMA1_Stream0, DISABLE);

		uint32_t timedout_start;
		timed_out_init(&timedout_start);
		while (DMA_GetCmdStatus(DMA1_Stream0) == ENABLE)
			if (timed_out(timedout_start, 100))
				return; //// TRY TO RECOVER

		ws2812_dma_running = false;
	}
}

/** Enable clocks, configure GPIO pin B6 as Alternate Function (TIM4), set timer to 800 kHz,
  * configure DMA1_Stream0_Channel2, and enable transfer complete interrupt.
  *
  * @return The status of configuration.
***/
uint8_t ws2812_config() {
	// TIM4 CHANNEL 1 - Pin_B6 - DMA1_Stream0_Channel2

	GPIO_InitTypeDef 	GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef	TIM_TimeBaseStruct;
	TIM_OCInitTypeDef  	TIM_OCInitStruct;
	DMA_InitTypeDef 	DMA_InitStruct;
	NVIC_InitTypeDef 	NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);

	GPIO_InitStructure.GPIO_Pin = 	GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = 	GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed =	GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType =	GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = 	GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_TimeBaseStruct.TIM_Prescaler = 0;			//  84 MHz
	TIM_TimeBaseStruct.TIM_Period = 104;			// 800 kHz
	TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStruct);

	DMA_DeInit(DMA1_Stream0);
	DMA_InitStruct.DMA_Channel = DMA_Channel_2;
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&TIM4->CCR1;
	DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)ws2812_dma_buffer;
	DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStruct.DMA_BufferSize =  WS2812_BUFFER_SIZE;
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStruct.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_Init(DMA1_Stream0, &DMA_InitStruct);
	DMA_ITConfig(DMA1_Stream0, DMA_IT_TC, ENABLE);

	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_Pulse = 0;
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OC1Init(TIM4, &TIM_OCInitStruct);
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM4, TIM_OCPreload_Enable);

	TIM_DMACmd(TIM4, TIM_DMA_CC1, ENABLE);
	TIM_Cmd(TIM4, ENABLE);

	return 0;
}

/** Initialize variables and prepare to begin normal operation.
  *
  * @return The status of initialization.
***/
uint8_t ws2812_init() {
	ws2812_last_state = 0;

	ws2812_dma_running = false;
	for (uint16_t i = 0; i < WS2812_BUFFER_SIZE; i++)
		ws2812_dma_buffer[i] = 0;

	ws2812_reset_all_colors();

	ws2812_set_global_brightness(WS2812_DEFAULT_BRIGHTNESS);

	ws2812_reset_animation(&ws2812_animation_tube);
	ws2812_reset_animation(&ws2812_animation_mount);

	return 0;
}

/** Update loop to handle state switching and animation updates at set intervals.
  *
  * @return The status of current update iteration.
***/
int8_t ws2812_update() {
	static uint32_t last_update = 0, last_update_tube = 0, last_update_mount = 0;
	static uint16_t last_brightness = 0;
	static bool refresh_needed = false;

	if (state.status != ws2812_last_state) {
		ws2812_change_state(state.status);
		ws2812_last_state = state.status;
		refresh_needed = true;
	}

	if (state.status == STATE_SYS_CALIBRATING) {
		ws2812_animation_tube.ao_layer1 &= ~opt_layer_length_15;
		uint16_t llength = scale(state.imu_calibration_percent, 0, 100, 1, WS2812_LEDS_PER_ARM_TUBE);
		llength <<= 12;
		ws2812_animation_tube.ao_layer1 |= llength;
	}

	if (ws2812_animation_tube.sample_rate == 0 && last_update_tube == 0) {
		ws2812_animate(ss_Tube);
		last_update_tube = clocks_get_ticks();
		refresh_needed = true;
	}

	if (ws2812_animation_mount.sample_rate == 0 && last_update_mount == 0) {
		ws2812_animate(ss_Mount);
		last_update_mount = clocks_get_ticks();
		refresh_needed = true;
	}

	if (abs(last_brightness - state.rx_channels[STATE_RX_CHANNEL_TOGGLE_L_KNOB]) > STATE_RX_DEADZONE && state.rx_channels[STATE_RX_CHANNEL_TOGGLE_L_KNOB] > 0) {  ///////////////// THE CHECK AFTER THE && SHOULDN'T BE NECESSARY!! STOP UPDATING STATE VALUES WHEN DATA IS INVALID!
		last_brightness = state.rx_channels[STATE_RX_CHANNEL_TOGGLE_L_KNOB];
		refresh_needed = true;
	}

	if (clocks_check_sample_rate(&last_update_mount, ws2812_animation_mount.sample_rate)) {
		ws2812_animate(ss_Mount);
		refresh_needed = true;
	}

	if (clocks_check_sample_rate(&last_update_tube, ws2812_animation_tube.sample_rate)) {
		ws2812_animate(ss_Tube);
		refresh_needed = true;
	}

	if (refresh_needed && clocks_check_sample_rate(&last_update, WS2812_SAMPLE_RATE)) {
		ws2812_global_brightness = scale(state.rx_channels[STATE_RX_CHANNEL_TOGGLE_L_KNOB], STATE_RX_VALUE_MIN, STATE_RX_VALUE_MAX, 0, 100);

		if (!ws2812_dma_running) {
			if (ws2812_convert_colors()) return -1;
			ws2812_push_colors();
			refresh_needed = false;
		}

		return 1;
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
void ws2812_change_state(uint8_t new_state) {
	struct ColorSet cs_background_tube, cs_layer1_tube;
	struct ColorSet cs_background_mount, cs_layer1_mount;
	enum AnimationOptions ao_background_tube, ao_layer1_tube;
	enum AnimationOptions ao_background_mount, ao_layer1_mount;

	uint16_t rate_tube = 0;
	uint16_t rate_mount = 0;

	switch (new_state) {
	case STATE_SYS_CALIBRATING:
		state_set_calibrated(STATE_PERIPH_WS2812);

		ws2812_fill_colorset(&cs_background_tube, 0, cp_white, 10, cp_white, 10);
		ws2812_fill_colorset(&cs_background_tube, 1, cp_white, 10, cp_white, 10);
		ws2812_fill_colorset(&cs_background_tube, 2, cp_white, 10, cp_white, 10);
		ws2812_fill_colorset(&cs_background_tube, 3, cp_white, 10, cp_white, 10);
		ws2812_fill_colorset(&cs_background_tube, 4, cp_white, 10, cp_white, 10);
		ws2812_fill_colorset(&cs_background_tube, 5, cp_white, 10, cp_white, 10);

		ws2812_fill_colorset(&cs_layer1_tube, 0, cp_red, 100, cp_orange, 100);
		ws2812_fill_colorset(&cs_layer1_tube, 1, cp_red, 0, cp_orange, 0);
		ws2812_fill_colorset(&cs_layer1_tube, 2, cp_red, 5, cp_orange, 5);
		ws2812_fill_colorset(&cs_layer1_tube, 3, cp_red, 20, cp_orange, 20);
		ws2812_fill_colorset(&cs_layer1_tube, 4, cp_red, 40, cp_orange, 40);
		ws2812_fill_colorset(&cs_layer1_tube, 5, cp_red, 60, cp_orange, 60);

		ws2812_fill_colorset(&cs_background_mount, 0, cp_red, 100, cp_red, 100);
		ws2812_fill_colorset(&cs_background_mount, 1, cp_red, 100, cp_red, 100);
		ws2812_fill_colorset(&cs_background_mount, 2, cp_red, 100, cp_red, 100);
		ws2812_fill_colorset(&cs_background_mount, 3, cp_red, 100, cp_red, 100);
		ws2812_fill_colorset(&cs_background_mount, 4, cp_red, 100, cp_red, 100);
		ws2812_fill_colorset(&cs_background_mount, 5, cp_red, 100, cp_red, 100);

		ws2812_fill_colorset(&cs_layer1_mount, 0, cp_red, 100, cp_red, 100);
		ws2812_fill_colorset(&cs_layer1_mount, 1, cp_red, 100, cp_red, 100);
		ws2812_fill_colorset(&cs_layer1_mount, 2, cp_white, 100, cp_white, 100);
		ws2812_fill_colorset(&cs_layer1_mount, 3, cp_red, 100, cp_red, 100);
		ws2812_fill_colorset(&cs_layer1_mount, 4, cp_red, 100, cp_red, 100);
		ws2812_fill_colorset(&cs_layer1_mount, 5, cp_white, 100, cp_white, 100);

		rate_tube = 18;
		rate_mount = 54;

		ao_background_tube = opt_none;
		ao_layer1_tube = opt_gradient_linear | opt_layer_length_1 | opt_pass_to_next_arm;
		ao_background_mount = opt_none;
		ao_layer1_mount = opt_layer_length_1 | opt_slide_out | opt_pass_to_next_arm;

		break;
	case STATE_SYS_IDLE:
		ws2812_fill_colorset(&cs_background_tube, 0, cp_red, 100, cp_red, 100);
		ws2812_fill_colorset(&cs_background_tube, 1, cp_red, 100, cp_red, 100);
		ws2812_fill_colorset(&cs_background_tube, 2, cp_green, 100, cp_green, 100);
		ws2812_fill_colorset(&cs_background_tube, 3, cp_blue, 100, cp_blue, 100);
		ws2812_fill_colorset(&cs_background_tube, 4, cp_blue, 100, cp_blue, 100);
		ws2812_fill_colorset(&cs_background_tube, 5, cp_green, 100, cp_green, 100);

		ws2812_fill_colorset(&cs_layer1_tube, 0, cp_white, 100, cp_white, 100);
		ws2812_fill_colorset(&cs_layer1_tube, 1, cp_white, 100, cp_white, 100);
		ws2812_fill_colorset(&cs_layer1_tube, 2, cp_white, 100, cp_white, 100);
		ws2812_fill_colorset(&cs_layer1_tube, 3, cp_white, 100, cp_white, 100);
		ws2812_fill_colorset(&cs_layer1_tube, 4, cp_white, 100, cp_white, 100);
		ws2812_fill_colorset(&cs_layer1_tube, 5, cp_white, 100, cp_white, 100);

		ws2812_fill_colorset(&cs_background_mount, 0, cp_red, 100, cp_red, 100);
		ws2812_fill_colorset(&cs_background_mount, 1, cp_red, 100, cp_red, 100);
		ws2812_fill_colorset(&cs_background_mount, 2, cp_green, 100, cp_green, 100);
		ws2812_fill_colorset(&cs_background_mount, 3, cp_blue, 100, cp_blue, 100);
		ws2812_fill_colorset(&cs_background_mount, 4, cp_blue, 100, cp_blue, 100);
		ws2812_fill_colorset(&cs_background_mount, 5, cp_green, 100, cp_green, 100);

		ws2812_fill_colorset(&cs_layer1_mount, 0, cp_white, 100, cp_white, 100);
		ws2812_fill_colorset(&cs_layer1_mount, 1, cp_white, 100, cp_white, 100);
		ws2812_fill_colorset(&cs_layer1_mount, 2, cp_white, 100, cp_white, 100);
		ws2812_fill_colorset(&cs_layer1_mount, 3, cp_white, 100, cp_white, 100);
		ws2812_fill_colorset(&cs_layer1_mount, 4, cp_white, 100, cp_white, 100);
		ws2812_fill_colorset(&cs_layer1_mount, 5, cp_white, 100, cp_white, 100);

		rate_tube = WS2812_LEDS_PER_ARM_TUBE * 2;
		rate_mount = WS2812_LEDS_PER_ARM_MOUNT;

		ao_background_tube = opt_none;
		ao_layer1_tube = opt_layer_length_2 | opt_slide_bounce;
		ao_background_mount = opt_none;
		ao_layer1_mount = opt_layer_length_1 | opt_slide_out | opt_pass_to_next_arm;

		break;
	case STATE_SYS_ARMING:
		ws2812_fill_colorset(&cs_background_tube, 0, cp_red, 100, cp_red, 100);
		ws2812_fill_colorset(&cs_background_tube, 1, cp_white, 20, cp_white, 20);
		ws2812_fill_colorset(&cs_background_tube, 2, cp_red, 100, cp_red, 100);
		ws2812_fill_colorset(&cs_background_tube, 3, cp_white, 20, cp_white, 20);
		ws2812_fill_colorset(&cs_background_tube, 4, cp_red, 100, cp_red, 100);
		ws2812_fill_colorset(&cs_background_tube, 5, cp_white, 20, cp_white, 20);

		ws2812_fill_colorset(&cs_layer1_tube, 0, cp_off, 100, cp_off, 100);
		ws2812_fill_colorset(&cs_layer1_tube, 1, cp_off, 100, cp_off, 100);
		ws2812_fill_colorset(&cs_layer1_tube, 2, cp_off, 100, cp_off, 100);
		ws2812_fill_colorset(&cs_layer1_tube, 3, cp_off, 100, cp_off, 100);
		ws2812_fill_colorset(&cs_layer1_tube, 4, cp_off, 100, cp_off, 100);
		ws2812_fill_colorset(&cs_layer1_tube, 5, cp_off, 100, cp_off, 100);

		ws2812_fill_colorset(&cs_background_mount, 0, cp_red, 100, cp_red, 100);
		ws2812_fill_colorset(&cs_background_mount, 1, cp_off, 100, cp_off, 100);
		ws2812_fill_colorset(&cs_background_mount, 2, cp_off, 100, cp_off, 100);
		ws2812_fill_colorset(&cs_background_mount, 3, cp_white, 100, cp_white, 100);
		ws2812_fill_colorset(&cs_background_mount, 4, cp_off, 100, cp_off, 100);
		ws2812_fill_colorset(&cs_background_mount, 5, cp_off, 100, cp_off, 100);

		ws2812_fill_colorset(&cs_layer1_mount, 0, cp_off, 100, cp_off, 100);
		ws2812_fill_colorset(&cs_layer1_mount, 1, cp_off, 100, cp_off, 100);
		ws2812_fill_colorset(&cs_layer1_mount, 2, cp_off, 100, cp_off, 100);
		ws2812_fill_colorset(&cs_layer1_mount, 3, cp_off, 100, cp_off, 100);
		ws2812_fill_colorset(&cs_layer1_mount, 4, cp_off, 100, cp_off, 100);
		ws2812_fill_colorset(&cs_layer1_mount, 5, cp_off, 100, cp_off, 100);

		rate_tube = 4;
		rate_mount = 12;

		ao_background_tube = opt_pass_to_next_arm;
		ao_layer1_tube = opt_none;
		ao_background_mount = opt_pass_to_next_arm;
		ao_layer1_mount = opt_none;

		break;
	case STATE_SYS_ARMED:
		ws2812_fill_colorset(&cs_background_tube, 0, cp_red, 100, cp_red, 100);
		ws2812_fill_colorset(&cs_background_tube, 1, cp_red, 100, cp_red, 100);
		ws2812_fill_colorset(&cs_background_tube, 2, cp_green, 100, cp_green, 100);
		ws2812_fill_colorset(&cs_background_tube, 3, cp_blue, 100, cp_blue, 100);
		ws2812_fill_colorset(&cs_background_tube, 4, cp_blue, 100, cp_blue, 100);
		ws2812_fill_colorset(&cs_background_tube, 5, cp_green, 100, cp_green, 100);

		ws2812_fill_colorset(&cs_layer1_tube, 0, cp_white, 100, cp_white, 100);
		ws2812_fill_colorset(&cs_layer1_tube, 1, cp_white, 100, cp_white, 100);
		ws2812_fill_colorset(&cs_layer1_tube, 2, cp_white, 100, cp_white, 100);
		ws2812_fill_colorset(&cs_layer1_tube, 3, cp_white, 100, cp_white, 100);
		ws2812_fill_colorset(&cs_layer1_tube, 4, cp_white, 100, cp_white, 100);
		ws2812_fill_colorset(&cs_layer1_tube, 5, cp_white, 100, cp_white, 100);

		ws2812_fill_colorset(&cs_background_mount, 0, cp_red, 100, cp_red, 100);
		ws2812_fill_colorset(&cs_background_mount, 1, cp_red, 100, cp_red, 100);
		ws2812_fill_colorset(&cs_background_mount, 2, cp_green, 100, cp_green, 100);
		ws2812_fill_colorset(&cs_background_mount, 3, cp_blue, 100, cp_blue, 100);
		ws2812_fill_colorset(&cs_background_mount, 4, cp_blue, 100, cp_blue, 100);
		ws2812_fill_colorset(&cs_background_mount, 5, cp_green, 100, cp_green, 100);

		ws2812_fill_colorset(&cs_layer1_mount, 0, cp_white, 100, cp_white, 100);
		ws2812_fill_colorset(&cs_layer1_mount, 1, cp_white, 100, cp_white, 100);
		ws2812_fill_colorset(&cs_layer1_mount, 2, cp_white, 100, cp_white, 100);
		ws2812_fill_colorset(&cs_layer1_mount, 3, cp_white, 100, cp_white, 100);
		ws2812_fill_colorset(&cs_layer1_mount, 4, cp_white, 100, cp_white, 100);
		ws2812_fill_colorset(&cs_layer1_mount, 5, cp_white, 100, cp_white, 100);

		rate_tube = WS2812_LEDS_PER_ARM_TUBE;
		rate_mount = WS2812_LEDS_PER_ARM_MOUNT;

		ao_background_tube = opt_none;
		ao_layer1_tube = opt_layer_length_1 | opt_slide_out;
		ao_background_mount = opt_none;
		ao_layer1_mount = opt_none;

		break;
	default:
		ws2812_fill_colorset(&cs_background_tube, 0, cp_off, 100, cp_off, 100);
		ws2812_fill_colorset(&cs_background_tube, 1, cp_off, 100, cp_off, 100);
		ws2812_fill_colorset(&cs_background_tube, 2, cp_off, 100, cp_off, 100);
		ws2812_fill_colorset(&cs_background_tube, 3, cp_off, 100, cp_off, 100);
		ws2812_fill_colorset(&cs_background_tube, 4, cp_off, 100, cp_off, 100);
		ws2812_fill_colorset(&cs_background_tube, 5, cp_off, 100, cp_off, 100);

		ws2812_fill_colorset(&cs_layer1_tube, 0, cp_off, 100, cp_off, 100);
		ws2812_fill_colorset(&cs_layer1_tube, 1, cp_off, 100, cp_off, 100);
		ws2812_fill_colorset(&cs_layer1_tube, 2, cp_off, 100, cp_off, 100);
		ws2812_fill_colorset(&cs_layer1_tube, 3, cp_off, 100, cp_off, 100);
		ws2812_fill_colorset(&cs_layer1_tube, 4, cp_off, 100, cp_off, 100);
		ws2812_fill_colorset(&cs_layer1_tube, 5, cp_off, 100, cp_off, 100);

		ws2812_fill_colorset(&cs_background_mount, 0, cp_off, 100, cp_off, 100);
		ws2812_fill_colorset(&cs_background_mount, 1, cp_off, 100, cp_off, 100);
		ws2812_fill_colorset(&cs_background_mount, 2, cp_off, 100, cp_off, 100);
		ws2812_fill_colorset(&cs_background_mount, 3, cp_off, 100, cp_off, 100);
		ws2812_fill_colorset(&cs_background_mount, 4, cp_off, 100, cp_off, 100);
		ws2812_fill_colorset(&cs_background_mount, 5, cp_off, 100, cp_off, 100);

		ws2812_fill_colorset(&cs_layer1_mount, 0, cp_off, 100, cp_off, 100);
		ws2812_fill_colorset(&cs_layer1_mount, 1, cp_off, 100, cp_off, 100);
		ws2812_fill_colorset(&cs_layer1_mount, 2, cp_off, 100, cp_off, 100);
		ws2812_fill_colorset(&cs_layer1_mount, 3, cp_off, 100, cp_off, 100);
		ws2812_fill_colorset(&cs_layer1_mount, 4, cp_off, 100, cp_off, 100);
		ws2812_fill_colorset(&cs_layer1_mount, 5, cp_off, 100, cp_off, 100);

		rate_tube = 0;
		rate_mount = 0;

		ao_background_tube = opt_none;
		ao_layer1_tube = opt_none;
		ao_background_mount = opt_none;
		ao_layer1_mount = opt_none;

		break;
	}

	ws2812_set_colorset(ss_Tube, al_background, cs_background_tube, true);
	ws2812_set_colorset(ss_Tube, al_layer1, cs_layer1_tube, true);
	ws2812_set_colorset(ss_Mount, al_background, cs_background_mount, true);
	ws2812_set_colorset(ss_Mount, al_layer1, cs_layer1_mount, true);

	ws2812_set_animation_options(ss_Tube, al_background, rate_tube, ao_background_tube, true);
	ws2812_set_animation_options(ss_Tube, al_layer1, rate_tube, ao_layer1_tube, true);
	ws2812_set_animation_options(ss_Mount, al_background, rate_mount, ao_background_mount, true);
	ws2812_set_animation_options(ss_Mount, al_layer1, rate_mount, ao_layer1_mount, true);
}

/** Fill up the DMA buffer by converting color data from the color array.
  *
  * @return The status of the function.
***/
int8_t ws2812_convert_colors() {
	uint8_t temp, color[4];

	if (!ws2812_dma_running) {
		for (uint8_t i = 0; i < WS2812_SIGNAL_START_DELAY; i++)
			ws2812_dma_buffer[i] = WS2812_BIT_Z;

		for (uint16_t i = 0; i < WS2812_LED_COUNT; i++) {
			color[WS2812_RED] = ws2812_colors[i].red;
			color[WS2812_GREEN] = ws2812_colors[i].green;
			color[WS2812_BLUE] = ws2812_colors[i].blue;
			color[WS2812_BRIGHTNESS] = clamp(ws2812_colors[i].brightness, 0, 100);

			for (uint8_t j = 0; j < 3; j++) {
				temp = (uint8_t)(color[j] * ((float)color[WS2812_BRIGHTNESS] / 100.0f) * ((float)ws2812_global_brightness / 100.0f));

				for (uint8_t k = 0; k < 8; k++) {
					if (temp & 0x80) {
						ws2812_dma_buffer[i * 24 + j * 8 + k + WS2812_SIGNAL_START_DELAY] = WS2812_BIT_1;
					}
					else {
						ws2812_dma_buffer[i * 24 + j * 8 + k + WS2812_SIGNAL_START_DELAY] = WS2812_BIT_0;
					}

					temp <<= 1;
				}
			}
		}

		for (int i = WS2812_LED_COUNT * 24 + WS2812_SIGNAL_START_DELAY; i < WS2812_BUFFER_SIZE; i++)
			ws2812_dma_buffer[i] = WS2812_BIT_Z;

		return 0;
	}

	return -1;
}

/** Start the DMA to transmit the color data to the LED strip.
  *
  * @return The status of the function.
***/
int8_t ws2812_push_colors() {
	if (!ws2812_dma_running) {
		DMA_SetCurrDataCounter(DMA1_Stream0, WS2812_BUFFER_SIZE);
		ws2812_dma_running = true;
		DMA_Cmd(DMA1_Stream0, ENABLE);
		uint32_t timedout_start;
		timed_out_init(&timedout_start);
		while (DMA_GetCmdStatus(DMA1_Stream0) != ENABLE) {
			if (timed_out(timedout_start, 100)) {
				DMA_Cmd(DMA1_Stream0, DISABLE);
				ws2812_dma_running = false;

				return -2;
			}
		}

		return 0;
	}

	return -1;
}

/** Update the specified section of the LED strip based on the animation options.
  *
  * @param ss:	The section of the strip to animate (ss_Tube or ss_Mount).
  *
  * @return The status of the function.
***/
int8_t ws2812_animate(enum StripSection ss) {
	static bool alternator = true;
	static bool alternator2 = true;
	uint8_t layer_length = 0;
	uint16_t frame_count = 0;
	int16_t slide_offset = 0;
	int16_t arm = 0, led = 0;
	int16_t repeat = 0;
	float progress = 0.0f;
	uint16_t *frame_ptr, fp_default = 0;
	frame_ptr = &fp_default;
	uint16_t frame = 0;
	bool *first_run, fr_default = true;
	first_run = &fr_default;
	struct Animation *anim;
	struct ColorSet *color_set;
	enum AnimationOptions *anim_options, ao_default = opt_none;
	anim_options = &ao_default;
	struct Color color, start, end;

	anim = (ss == ss_Tube) ? &ws2812_animation_tube : &ws2812_animation_mount;

	uint32_t ticks = clocks_get_ticks();
	uint32_t delta_t = ticks;
	if (delta_t >= anim->last_update)
		delta_t -= anim->last_update;
	else
		delta_t += (0xFFFFFFFF - anim->last_update);

	if (delta_t != 0) {
		uint16_t freq = (uint16_t)(1000000.0f / delta_t);
		if (freq != 0)
			if (anim->last_update > 0)
				if ((anim->sample_rate / freq) >= 1)
					repeat = (anim->sample_rate / freq) - 1;
	}
	anim->last_update = ticks;

	for (uint8_t j = 0; j < WS2812_LAYER_COUNT; j++) {
		for (uint8_t k = 0; k < WS2812_ARM_COUNT; k++) {
			for (uint8_t m = 0; m < ((ss == ss_Tube) ? WS2812_LEDS_PER_ARM_TUBE : WS2812_LEDS_PER_ARM_MOUNT); m++) {
				switch (j) {
				case 1:
					color_set = &anim->cs_layer1;
					anim_options = &anim->ao_layer1;
					layer_length = clamp((*anim_options & WS2812_LAYER_LENGTH_MASK) >> 12, 0, ((ss == ss_Tube) ? WS2812_LEDS_PER_ARM_TUBE : WS2812_LEDS_PER_ARM_MOUNT));
					frame_ptr = &anim->frame_layer1;
					first_run = &anim->first_run_layer1;

					break;
				case 0:
				default:
					color_set = &anim->cs_background;
					anim_options = &anim->ao_background;
					layer_length = ((ss == ss_Tube) ? WS2812_LEDS_PER_ARM_TUBE : WS2812_LEDS_PER_ARM_MOUNT);
					frame_ptr = &anim->frame_background;
					first_run = &anim->first_run_background;

					break;
				}

				frame = *frame_ptr + repeat;

				ws2812_copy_color(&start, color_set->cs_start[k]);
				ws2812_copy_color(&end, color_set->cs_end[k]);

				if (layer_length > 1)
					progress = m / (float)(layer_length - 1);
				else if (layer_length == 1) {
					if (m > layer_length - 1)
						progress = 2.0f;
					else
						progress = (float)m;
				}
				else
					progress = 2.0f;

				if (progress > 1.0f)
					continue;

				if (*anim_options & opt_gradient_linear) {
				}
				else if (*anim_options & opt_gradient_center) {
					if (progress < 0.5f)
						progress *= 2.0f;
					else
						progress = (1.0f - progress) * 2.0f;
				}
				else {
					if (progress < 0.5f)
						progress = 0.0f;
					else
						progress = 1.0f;
				}

				color = ws2812_gradient(progress, start, end);

				if (!(*anim_options & opt_anim_blend_slow) && !(*anim_options & opt_anim_blend_medium)) {
					if (*anim_options & opt_max_blend_100)
						color = ws2812_get_color(ss, k, m);
					else if (*anim_options & opt_max_blend_66)
						color = ws2812_gradient(0.66f, color, ws2812_get_color(ss, k, m));
					else if (*anim_options & opt_max_blend_33)
						color = ws2812_gradient(0.33f, color, ws2812_get_color(ss, k, m));
				}

				arm = k;
				led = m;

				frame_count = 0;
				if (ws2812_is_animation(*anim_options, layer_length)) {
					frame_count = ((ss == ss_Tube) ? WS2812_LEDS_PER_ARM_TUBE : WS2812_LEDS_PER_ARM_MOUNT) - layer_length + 1;

					if (*anim_options & opt_pass_to_next_arm)
						frame_count *= WS2812_ARM_COUNT;

					if ((*anim_options & opt_slide_bounce) == opt_slide_bounce) {
						frame_count = (frame_count * 2) - 2;
						uint16_t midpoint = ((ss == ss_Tube) ? WS2812_LEDS_PER_ARM_TUBE : WS2812_LEDS_PER_ARM_MOUNT) - layer_length;
						if (frame <= midpoint)
							slide_offset = frame;
						else
							slide_offset = midpoint - (frame - midpoint);
					}
					else {
						if (*anim_options & opt_slide_out)
							slide_offset = frame;
						else if (*anim_options & opt_slide_in)
							slide_offset = (((ss == ss_Tube) ? WS2812_LEDS_PER_ARM_TUBE : WS2812_LEDS_PER_ARM_MOUNT) - 1) - frame - layer_length + 1;
						else if (*anim_options & opt_pass_to_next_arm) {
							frame_count = WS2812_ARM_COUNT;
							slide_offset = (((ss == ss_Tube) ? WS2812_LEDS_PER_ARM_TUBE : WS2812_LEDS_PER_ARM_MOUNT) * (frame % layer_length));
						}
					}

					float max_blend = 0.0f;
					if (*anim_options & opt_max_blend_100)
						max_blend = 1.0f;
					else if (*anim_options & opt_max_blend_66)
						max_blend = 0.66f;
					else if (*anim_options & opt_max_blend_33)
						max_blend = 0.33f;

					float multiplier = 0.0f;
					if (*anim_options & opt_anim_blend_fast)
						multiplier = 6.0f;
					else if (*anim_options & opt_anim_blend_medium)
						multiplier = 4.0f;
					else if (*anim_options & opt_anim_blend_slow)
						multiplier = 2.0f;

					float blend_progress = (frame % ((ss == ss_Tube) ? WS2812_LEDS_PER_ARM_TUBE : WS2812_LEDS_PER_ARM_MOUNT)) / ((float)((ss == ss_Tube) ? WS2812_LEDS_PER_ARM_TUBE : WS2812_LEDS_PER_ARM_MOUNT) - 1.0f);
					blend_progress = clampf(blend_progress * max_blend * multiplier, 0.0f, multiplier);

					while (blend_progress > 1.0f)
						blend_progress -= 1.0f;

					if (blend_progress > 0.5f)
						blend_progress = 1.0f - blend_progress;

					color = ws2812_gradient(blend_progress, color, ws2812_get_color(ss, k, m));

					int16_t offset_arm = abs(slide_offset) / ((ss == ss_Tube) ? WS2812_LEDS_PER_ARM_TUBE : WS2812_LEDS_PER_ARM_MOUNT);
					int16_t offset_led = abs(slide_offset) % ((ss == ss_Tube) ? WS2812_LEDS_PER_ARM_TUBE : WS2812_LEDS_PER_ARM_MOUNT);

					if (slide_offset < 0) {
						offset_arm *= -1;
						offset_led *= -1;
					}

					arm = (k + offset_arm) % WS2812_ARM_COUNT;
					while (arm < 0)
						arm += WS2812_ARM_COUNT;

					led = (m + offset_led) % ((ss == ss_Tube) ? WS2812_LEDS_PER_ARM_TUBE : WS2812_LEDS_PER_ARM_MOUNT);
					while (led < 0) {
						led += ((ss == ss_Tube) ? WS2812_LEDS_PER_ARM_TUBE : WS2812_LEDS_PER_ARM_MOUNT);

						if (--arm < 0)
							arm += WS2812_ARM_COUNT;
					}
				}

				//////////////////////// GET BLINKING WORKING!!!!!!!!!!! SOOO TIRED :(
				if ((*anim_options & opt_blink_on) && alternator) { }
				else if ((*anim_options & opt_blink_offset) && alternator) {
					if (!alternator2) {
						ws2812_set_color(ss, arm, led, color);
						alternator2 = !alternator2;
					}
				}
				else {
					ws2812_set_color(ss, arm, led, color);
				}
			}
		}

		if (ws2812_is_animation(*anim_options, layer_length)) {
			*frame_ptr += (1 + repeat);
			if (*frame_ptr >= frame_count) {
				if (*first_run)
					*first_run = false;

				*frame_ptr %= frame_count;
			}
		}

		alternator = !alternator;
	}


	return 0;
}

/** Set the color of a specified LED on the strip.
  *
  * @param section:	The section of the strip the LED is in (ss_Tube or ss_Mount).
  * @param arm:		The arm of the copter that the LED is on.
  * @param led:		The number of the LED on the specified arm.
  * @param color:	The color to set the LED to.
  *
  * @return The status of the function.
***/
int8_t ws2812_set_color(enum StripSection section, uint8_t arm, uint8_t led, struct Color color) {
	if (arm != clamp(arm, 0, WS2812_ARM_COUNT - 1))
		return -1;

	if (led != clamp(led, 0, (section == ss_Tube ? WS2812_LEDS_PER_ARM_TUBE : WS2812_LEDS_PER_ARM_MOUNT) - 1))
		return -2;

	uint16_t offset = arm * WS2812_LEDS_PER_ARM_ALL + (section == ss_Tube ? 0 : WS2812_LEDS_PER_ARM_TUBE) + led;

	ws2812_copy_color(&ws2812_colors[offset], color);

	return 0;
}

/** Get the color of a specified LED on the strip.
  *
  * @param section:	The section of the strip the LED is in (ss_Tube or ss_Mount).
  * @param arm:		The arm of the multicopter that the LED is on.
  * @param led:		The number of the LED on the specified arm.
  *
  * @return The color of the LED.
***/
struct Color ws2812_get_color(enum StripSection section, uint8_t arm, uint8_t led) {
	struct Color color = ws2812_fill_color_from_palette(WS2812_DEFAULT_COLOR, WS2812_DEFAULT_BRIGHTNESS);

	if (arm != clamp(arm, 0, WS2812_ARM_COUNT - 1))
		return color;

	if (led != clamp(led, 0, (section == ss_Tube ? WS2812_LEDS_PER_ARM_TUBE : WS2812_LEDS_PER_ARM_MOUNT) - 1))
		return color;

	uint16_t offset = arm * WS2812_LEDS_PER_ARM_ALL + (section == ss_Tube ? 0 : WS2812_LEDS_PER_ARM_TUBE) + led;
	ws2812_copy_color(&color, ws2812_colors[offset]);

	return color;
}

/** Set all the LEDs on the strip to the default color and brightness.
  *
  * @return The status of the function.
***/
int8_t ws2812_reset_all_colors() {
	struct Color color = ws2812_fill_color_from_palette(WS2812_DEFAULT_COLOR, WS2812_DEFAULT_BRIGHTNESS);

	for (uint16_t i = 0; i < WS2812_LED_COUNT; i++)
		ws2812_copy_color(&ws2812_colors[i], color);

	return 0;
}

/** Copy the color data from one color structure to another.
  *
  * @param *dest:	Pointer to the color data to be set.
  * @param source:	The color data to be copied.
  *
  * @return The status of the function.
***/
int8_t ws2812_copy_color(struct Color *dest, struct Color source) {
	dest->red = source.red;
	dest->green = source.green;
	dest->blue = source.blue;
	dest->brightness = source.brightness;

	return 0;
}

/** Set the overall maximum brightness level for all LEDs.
  *
  * @param brightness:	The brightness to set (0 to 100)
  *
  * @return The status of the function.
***/
int8_t ws2812_set_global_brightness(uint8_t brightness) {
	ws2812_global_brightness = clamp(brightness, 0, 100);

	return 0;
}

/** Fill a color structure with data based on a color name from the palette.
  *
  * @param source:	The name of the color.
  * @param brightness:	The brightness level.
  *
  * @return The color structure.
***/
struct Color ws2812_fill_color_from_palette(enum ColorPalette source, uint8_t brightness) {
	struct Color color;
	uint8_t red = 0, green = 0, blue = 0;

	switch (source) {
	case cp_off:
		break;
	case cp_white:
		red = 255;
		green = 255;
		blue = 255;
		break;
	case cp_red:
		red = 255;
		break;
	case cp_green:
		green = 255;
		break;
	case cp_blue:
		blue = 255;
		break;
	case cp_magenta:
		red = 255;
		blue = 255;
		break;
	case cp_orange:
		red = 255;
		green = 70;
		break;
	case cp_yellow:
		red = 255;
		green = 255;
		break;
	case cp_cyan:
		blue = 255;
		green = 255;
		break;
	case cp_purple:
		red = 170;
		blue = 255;
		break;
	}

	color.red = red;
	color.green = green;
	color.blue = blue;
	color.brightness = brightness;

	return color;
}

/** Fill a color structure based on given red, green, and blue values, along with brightness.
  *
  * @param red:		Red value (0 to 255).
  * @param green:	Green value (0 to 255).
  * @param blue:	Blue value (0 to 255).
  * @param brightness:	Brightness value (0 to 100)
  *
  * @return The color structure.
***/
struct Color ws2812_fill_color_from_rgb(uint8_t red, uint8_t green, uint8_t blue, uint8_t brightness) {
	struct Color color;

	color.red = red;
	color.green = green;
	color.blue = blue;
	color.brightness = brightness;

	return color;
}

/** Fill in the specified ColorSet data for the arm with the given color information.
  *
  * @param *set:		Pointer to the ColorSet being filled.
  * @param arm:			The arm being set.
  * @param start_color:		The starting color for the specified arm.
  * @param start_brightness:	The starting brightness for the specified arm.
  * @param end_color:		The ending color for the specified arm.
  * @param end_brightness:	The ending brightness for the specified arm.
  *
  * @return The status of the function.
***/
int8_t ws2812_fill_colorset(struct ColorSet *set, uint8_t arm, enum ColorPalette start_color, uint8_t start_brightness, enum ColorPalette end_color, uint8_t end_brightness) {
	ws2812_copy_color(&set->cs_start[arm], ws2812_fill_color_from_palette(start_color, start_brightness));
	ws2812_copy_color(&set->cs_end[arm], ws2812_fill_color_from_palette(end_color, end_brightness));

	return 0;
}

/** Set the ColorSet of a specified strip section and animation layer to
  * a previously filled in ColorSet structure.
  *
  * @param ss:		The strip section being set (ss_Tube or ss_Mount).
  * @param al:		The animation layer being set.
  * @param colorset:	The filled in ColorSet being used.
  * @param reset:	True if you want to reset the animation, otherwise False.
  *
  * @return The status of the function.
***/
int8_t ws2812_set_colorset(enum StripSection ss, enum AnimationLayer al, struct ColorSet colorset, bool reset) {
	struct Animation *anim;
	struct ColorSet *set;
	uint16_t *frame;

	if (ss == ss_Tube)
		anim = &ws2812_animation_tube;
	else if (ss == ss_Mount)
		anim = &ws2812_animation_mount;
	else
		return -1;

	if (al == al_background) {
		set = &anim->cs_background;
		frame = &anim->frame_background;
	}
	else if (al == al_layer1) {
		set = &anim->cs_layer1;
		frame = &anim->frame_layer1;
	}
	else
		return -2;

	if (reset)
		*frame = 0;

	ws2812_copy_colorset(set, colorset);

	return 0;
}

/** Copy the data from one ColorSet to another.
  *
  * @param *dest:	Pointer to the ColorSet to be filled.
  * @param source:	The ColorSet to be copied from.
  *
  * @return The status of the function.
***/
int8_t ws2812_copy_colorset(struct ColorSet *dest, struct ColorSet source) {
	for (uint8_t i = 0; i < WS2812_ARM_COUNT; i++) {
		dest->cs_start[i] = source.cs_start[i];
		dest->cs_end[i] = source.cs_end[i];
	}

	return 0;
}

/** Reset the data in a specified ColorSet to default color and brightness values.
  *
  * @param *set:	Pointer to the ColorSet to be reset.
  *
  * @return The status of the function.
***/
int8_t ws2812_reset_colorset(struct ColorSet *set) {
	struct Color color = ws2812_fill_color_from_palette(WS2812_DEFAULT_COLOR, WS2812_BRIGHTNESS);

	for (uint8_t i = 0; i < WS2812_ARM_COUNT; i++) {
		ws2812_copy_color(&set->cs_start[i], color);
		ws2812_copy_color(&set->cs_end[i], color);
	}

	return 0;
}

/** Set the animation options for a given strip section and animation layer.
  *
  * @param ss:		The strip section to set (ss_Tube or ss_Mount).
  * @param al:		The animation layer to set.
  * @param rate:	The refresh rate for the new animation.
  * @param options:	The options which define the animation.
  * @param reset:	True if you want to reset the animation to the beginning, otherwise False.
  *
  * @return The status of the function.
***/
int8_t ws2812_set_animation_options(enum StripSection ss, enum AnimationLayer al, uint16_t rate, enum AnimationOptions options, bool reset) {
	struct Animation *anim;
	enum AnimationOptions *ao;

	if (ss == ss_Tube)
		anim = &ws2812_animation_tube;
	else if (ss == ss_Mount)
		anim = &ws2812_animation_mount;
	else
		return -1;

	if (al == al_background)
		ao = &anim->ao_background;
	else if (al == al_layer1)
		ao = &anim->ao_layer1;
	else
		return -2;

	anim->sample_rate = clamp(rate, 0, WS2812_SAMPLE_RATE * ((ss == ss_Tube) ? WS2812_LEDS_PER_ARM_TUBE : WS2812_LEDS_PER_ARM_MOUNT));

	if (reset) {
		bool *first_run;
		uint16_t *frame;

		first_run = (al == al_background) ? &anim->first_run_background : &anim->first_run_layer1;
		*first_run = true;

		frame = (al == al_background) ? &anim->frame_background : &anim->frame_layer1;
		*frame = 0;
	}

	ws2812_copy_animation_options(ao, options);

	return 0;
}

/** Copy the animation options from one location to another.
  *
  * @param *dest:	Pointer to the AnimationOptions that you want to copy to.
  * @param source:	The AnimationOptions to be copied from.
  *
  * @return The status of the function.
***/
int8_t ws2812_copy_animation_options(enum AnimationOptions *dest, enum AnimationOptions source) {
	*dest = source;

	return 0;
}

/** Reset an animation to default settings (no animation).
  *
  * @param *animation:	Pointer to the animation to be reset.
  *
  * @return The status of the function.
***/
int8_t ws2812_reset_animation(struct Animation *animation) {
	animation->sample_rate = WS2812_DEFAULT_SAMPLE_RATE;
	animation->first_run_background = true;
	animation->first_run_layer1 = true;
	animation->frame_background = 0;
	animation->frame_layer1 = 0;
	ws2812_reset_colorset(&animation->cs_background);
	ws2812_reset_colorset(&animation->cs_layer1);
	animation->ao_background = opt_gradient_none;
	animation->ao_layer1 = opt_gradient_none;

	return 0;
}

/** Get the color at a specified point blended between two other colors.
  *
  * @param progress:	How far from start and close to end the color is
  * 			(0.0 = start, 1.0 = end).
  * @param start:	The starting color.
  * @param end:		The ending color.
  *
  * @return The Color somewhere between the start color and end color.
***/
struct Color ws2812_gradient(float progress, struct Color start, struct Color end) {
	if (progress == 0.0f)
		return start;

	if (progress == 1.0f)
		return end;

	struct Color result;

	result.red = start.red + ((end.red - start.red) * progress);
	result.green = start.green + ((end.green - start.green) * progress);
	result.blue = start.blue + ((end.blue - start.blue) * progress);
	result.brightness = start.brightness + ((end.brightness - start.brightness) * progress);

	return result;
}

/** Determine if given AnimationOptions and layer length include an animation or not.
  *
  * @param ao:			The AnimationOptions to check.
  * @param layer_length:	The length of the layer.
  *
  * @return True if specified AnimationOptions include something animated, otherwise False.
***/
bool ws2812_is_animation(enum AnimationOptions ao, uint16_t layer_length) {
	if (layer_length == 0)
		return false;

	if (ao & opt_slide_in || ao & opt_slide_out)
		return true;

	if (ao & opt_blink_on || ao & opt_blink_offset)
		return true;

	if (ao & opt_anim_blend_slow || ao & opt_anim_blend_medium)
		return true;

	if (ao & opt_pass_to_next_arm)
		return true;

	return false;
}

/** Fill in the array color data for the Drone Dashboard Processing packet.
  *
  * @param *ws2812:	Pointer to the array of color data.
  *
  * @return Void.
***/
void ws2812_get_processing(uint8_t *ws2812) {
	struct Color color;

	for (uint8_t i = 0; i < WS2812_LED_COUNT; i++) {
		ws2812_copy_color(&color, ws2812_colors[i]);

		ws2812[(i * 3)] = color.red * (color.brightness / 100.0f);
		ws2812[(i * 3) + 1] = color.green * (color.brightness / 100.0f);
		ws2812[(i * 3) + 2] = color.blue * (color.brightness / 100.0f);
	}
}
