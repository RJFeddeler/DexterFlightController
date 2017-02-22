#include "x8r.h"

/** The interrupt handler triggers when it receives data and
  * handles syncing the data frame and checking for errors.
  *
  * @return Void.
***/
void DMA2_Stream2_IRQHandler(void) {
	static volatile uint16_t unsynced_fail_count = 0, synced_fail_count = 0;

	if (DMA_GetITStatus(DMA2_Stream2, DMA_IT_TCIF2) == SET) {
		DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TCIF2);

		if (x8r_dma_synced == false) {
			if (x8r_reload_value > 1) {
				x8r_reload_value = X8R_FRAME_SIZE;
				x8r_dma_synced = true;
				if (state.status == STATE_SYS_CALIBRATING)
					state_set_calibrated(STATE_PERIPH_X8R);
			}
			else {
				if (x8r_last_update == 0)
					x8r_last_update = clocks_get_ticks();

				if (x8r_dma_buffer[0] == X8R_START_BYTE) {
					uint32_t ticks = clocks_get_ticks();
					if (ticks < x8r_last_update) {
						if ((ticks + (0xFFFFFFFF - x8r_last_update)) > 3000)
							x8r_reload_value = X8R_FRAME_SIZE - 1;
					}
					else {
						if ((ticks - x8r_last_update) > 3000)
							x8r_reload_value = X8R_FRAME_SIZE - 1;
					}
				}

				// Instead of incrementing state fail-safe counter with each single byte, read a full frames worth of bytes before incrementing
				if (x8r_reload_value == 1) {
					if (++unsynced_fail_count >= X8R_FRAME_SIZE) {
						unsynced_fail_count = 0;
						state.rx_failsafe_count++;
					}
				}
				else
					unsynced_fail_count = 0;

				x8r_last_update = clocks_get_ticks();
			}
		}
		else {
			if ((x8r_dma_buffer[0] == X8R_START_BYTE) && (x8r_dma_buffer[X8R_FRAME_SIZE - 1] == X8R_END_BYTE)) {
				synced_fail_count = 0;
				timed_out_init(&x8r_timedout_start);
				x8r_frame_ready = true;
			}
			else {
				//	Three frames out of sync in a row triggers a re-sync
				if (++synced_fail_count >= 3) {
					synced_fail_count = 0;
					x8r_dma_synced = false;
					x8r_reload_value = 1;
					x8r_last_update = 0;
					state.rx_failsafe_count++;
					if (state.status == STATE_SYS_CALIBRATING)
						state_set_uncalibrated(STATE_PERIPH_X8R);
				}
			}
		}

		DMA_SetCurrDataCounter(DMA2_Stream2, x8r_reload_value);
		DMA_Cmd(DMA2_Stream2, ENABLE);
	}
}

/** Enable clocks, configure GPIO pin A10 as Alternate Function (USART1),
  * configure DMA2_Stream2_Channel4, and enable transfer complete interrupt.
  *
  * @return The status of configuration.
***/
uint8_t x8r_config() {
	GPIO_InitTypeDef  		GPIO_InitStruct;
	USART_InitTypeDef 		USART_InitStruct;
	DMA_InitTypeDef			DMA_InitStruct;
	NVIC_InitTypeDef  		NVIC_InitStruct;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	NVIC_InitStruct.NVIC_IRQChannel = DMA2_Stream2_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);

	USART_DeInit(USART1);
	USART_InitStruct.USART_BaudRate = 100000;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_InitStruct.USART_StopBits = USART_StopBits_2;
	USART_InitStruct.USART_Parity = USART_Parity_Even;
	USART_InitStruct.USART_Mode = USART_Mode_Rx;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART1, &USART_InitStruct);

	DMA_DeInit(DMA2_Stream2);
	DMA_InitStruct.DMA_Channel = DMA_Channel_4;
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
	DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)x8r_dma_buffer;
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStruct.DMA_BufferSize =  1;
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_Init(DMA2_Stream2, &DMA_InitStruct);
	DMA_ITConfig(DMA2_Stream2, DMA_IT_TC, ENABLE);
	DMA2_Stream2->CR &= ~(0x4000); // DISABLE MEM2MEM MODE (PROBABLY DON'T NEED THIS, TEST ON WS2812)

	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);

	return 0;
}

/** Initialize variables and prepare to begin normal operation.
  *
  * @return The status of initialization.
***/
uint8_t x8r_init() {
	x8r_digital1 = 0;
	x8r_digital2 = 0;
	x8r_status = X8R_SIGNAL_OK;

	x8r_reload_value = 1;
	x8r_last_update = 0;

	x8r_frame_ready = false;
	x8r_dma_synced = false;
	x8r_timedout_start = 0;

	state.rx_failsafe_count = 0;

	x8r_last_state = 0;

	USART_Cmd(USART1, ENABLE);
	DMA_Cmd(DMA2_Stream2, ENABLE);

	return 0;
}

/** Update loop to handle state switching and conversion from raw to cooked data.
  *
  * @return The status of current update iteration.
***/
int8_t x8r_update() {

	if (state.status != x8r_last_state) {
		x8r_change_state(state.status);
		x8r_last_state = state.status;
	}

	if (x8r_frame_ready)
		return x8r_parse_frame();
	else if (state.status == STATE_SYS_ARMED) {
		if (timed_out(x8r_timedout_start, 250)) {
			state.rx_failsafe_count++;
			timed_out_init(&x8r_timedout_start);
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
void x8r_change_state(uint8_t new_state) {
	switch (new_state) {
	case STATE_SYS_CALIBRATING:
		x8r_frame_ready = false;
		x8r_dma_synced = false;
		x8r_reload_value = 1;

		break;
	case STATE_SYS_IDLE:
		state.rx_failsafe_count = 0;

		if (x8r_last_state == STATE_SYS_ARMED) {
			x8r_frame_ready = false;
			x8r_dma_synced = false;
			x8r_reload_value = 1;
			x8r_last_update = 0;
			x8r_timedout_start = 0;
			x8r_digital1 = 0;
			x8r_digital2 = 0;
			x8r_status = X8R_SIGNAL_OK;

			for (uint8_t i = 0; i < X8R_NUM_CHANNELS; i++)
				state.rx_channels[i] = 0;
		}

		break;
	case STATE_SYS_ARMING:
		break;
	case STATE_SYS_ARMED:
		break;
	}
}

/** Parse the frame buffer and store values in the channel array.
  *
  * @return The status of the function.
***/
int8_t x8r_parse_frame() {
	uint8_t pos = 0;
	uint8_t *channel_buffer = &x8r_dma_buffer[1];

	if (!x8r_frame_ready)
		return -1;

	x8r_frame_ready = false;

	x8r_flags = x8r_dma_buffer[X8R_FLAG_BYTE];
	x8r_status = X8R_SIGNAL_OK;

	if (x8r_flags & X8R_FLAG_BIT_LOST)
		x8r_status = X8R_SIGNAL_LOST;

	if (x8r_flags & X8R_FLAG_BIT_FAILSAFE)
		x8r_status = X8R_SIGNAL_FAILSAFE;

	if (x8r_status == X8R_SIGNAL_OK) {
		state.rx_failsafe_count = 0;

		for (uint8_t i = 0; i < X8R_NUM_CHANNELS; i++)
			state.rx_channels[i] = 0;

		while (pos++ < X8R_NUM_CHANNELS * 11) {
			if (channel_buffer[pos / 8] & 1<<(pos % 8))
				state.rx_channels[pos / 11] |= 1<<(pos % 11);
		}
	}
	else {
		state.rx_failsafe_count++;
		return -1;
	}

	return 1;
}
