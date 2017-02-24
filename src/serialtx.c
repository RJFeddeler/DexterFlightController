#include "serialtx.h"
// NOTE TO SELF: Fix up this library, IT IS TERRIBLE!!!

/** Interrupt handler is triggered when serial data has finished
  * transmitting and ensures the DMA is disabled.
  *
  * @return Void.
***/
void DMA2_Stream6_IRQHandler(void) {
	if (DMA_GetITStatus(DMA2_Stream6, DMA_IT_TCIF6) == SET) {
		DMA_ClearITPendingBit(DMA2_Stream6, DMA_IT_TCIF6);

		DMA_Cmd(DMA2_Stream6, DISABLE);

		uint32_t timedout_start;
		timed_out_init(&timedout_start);
		while (DMA_GetCmdStatus(DMA2_Stream6) == ENABLE)
			if (timed_out(timedout_start, 100))
				return;		// TRY TO RECOVER??

		serialtx_busy = false;
	}
}

/** Enable clocks, configure GPIO pin C6 (Tx) as Alternate Function (USART6),
  * configure DMA2_Stream6_Channel5, and enable interrupt.
  *
  * @return The status of configuration.
***/
uint16_t serialtx_config() {
	GPIO_InitTypeDef  	GPIO_InitStruct;
	USART_InitTypeDef 	USART_InitStruct;
	DMA_InitTypeDef		DMA_InitStruct;
	NVIC_InitTypeDef  	NVIC_InitStruct;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStruct);

	NVIC_InitStruct.NVIC_IRQChannel = DMA2_Stream6_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);

	USART_DeInit(USART6);
	USART_InitStruct.USART_BaudRate = 230400;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_Mode = USART_Mode_Tx;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART6, &USART_InitStruct);

	DMA_DeInit(DMA2_Stream6);
	DMA_InitStruct.DMA_Channel = DMA_Channel_5;
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&USART6->DR;
	DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)serialtx_dma_buffer;
	DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStruct.DMA_BufferSize =  SERIALTX_BUFFER_SIZE;
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStruct.DMA_Priority = DMA_Priority_Low;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_Init(DMA2_Stream6, &DMA_InitStruct);

	DMA_ITConfig(DMA2_Stream6, DMA_IT_TC, ENABLE);
	DMA2_Stream6->CR &= ~(0x4000);	 // DISABLE MEM2MEM MODE (PROBABLY DON'T NEED THIS, TEST ON WS2812)

	USART_DMACmd(USART6, USART_DMAReq_Tx, ENABLE);
	USART_Cmd(USART6, ENABLE);

	serialtx_busy = false;

	return 0;
}

/** Determine if serial transmission is currently in use.
  *
  * @return True if serial transmission is already in progress, otherwise False.
***/
bool serialtx_get_busy() { return serialtx_busy; }

/** Send the data in the buffer across the serial transmitting line.
  *
  * @param num_bytes:	The number of bytes in the transmission.
  *
  * @return The status of the function.
***/
int8_t serialtx_send(uint16_t num_bytes) {
	delay(30);			/// TESTING
	if (serialtx_busy == false) {
		if (USART_GetFlagStatus(USART6, USART_FLAG_TXE) == SET) {
			serialtx_busy = true;

			DMA_SetCurrDataCounter(DMA2_Stream6, num_bytes);
			DMA_Cmd(DMA2_Stream6, ENABLE);
		}
	}

	return 0;
}

/** Send a 32-bit integer as string across the serial line.
  *
  * @param data:	The integer representation of the number to send in string format.
  *
  * @return The status of the function.
***/
int8_t serialtx_send_dec_int32(int32_t data) {
	uint8_t offset = 0;
	uint8_t num_digits = 1;
	uint32_t divisor;
	uint32_t dummy;

	dummy = (data < 0 ? -data : data);
	while (dummy / 10 > 0) {
		num_digits++;
		dummy /= 10;
	}

	serialtx_clear_buffer();

	if (data < 0) {
		serialtx_dma_buffer[0] = '-';
		offset = 1;
	}

	dummy = (data < 0 ? -data : data);
	for (uint8_t i = 0; i < num_digits; i++) {
		divisor = power(10, num_digits - (i + 1));
		serialtx_dma_buffer[i + offset] = dummy / divisor;
		dummy -= serialtx_dma_buffer[i + offset] * divisor;
		serialtx_dma_buffer[i + offset] += 0x30;
	}
	serialtx_dma_buffer[num_digits + offset] = 0x0A;

	serialtx_send(num_digits + 1 + offset);

	return 0;
}

/** Send the values of the radio transmitter channels over the serial line.
  *
  * @param *channels:	Pointer to the radio channel data.
  * @param num_chans:	The number of channels for the radio transmitter.
  *
  * @return The status of the function.
***/
int8_t serialtx_send_dec_x8r(uint16_t *channels, uint8_t num_chans) {
	uint8_t num_digits;
	uint16_t offset = 2;
	uint32_t dummy;

	serialtx_clear_buffer();

	serialtx_dma_buffer[0] = 0x23;
	serialtx_dma_buffer[1] = 0x0A;

	for (uint8_t j = 0; j < num_chans; j++) {
		num_digits = 1;
		dummy = channels[j];
		while (dummy / 10 > 0) {
			num_digits++;
			dummy /= 10;
		}

		dummy = channels[j];
		uint32_t divisor;
		for (uint8_t i = 0; i < num_digits; i++) {
			divisor = power(10, num_digits - (i + 1));
			serialtx_dma_buffer[i + offset] = dummy / divisor;
			dummy -= serialtx_dma_buffer[i + offset] * divisor;
			serialtx_dma_buffer[i + offset] += 0x30;
		}
		serialtx_dma_buffer[num_digits + offset] = 0x0A;
		offset += num_digits + 1;
	}

	serialtx_send(offset);

	return 0;
}

/** Send the Processing packet over the serial line for the Drone Dashboard.
  *
  * @param status:	Value of the 4 LEDs (used for status signals).
  * @param *x8r:	Pointer to the radio transmitter channels data.
  * @param *bno:	Pointer to the BNO055 sensor data.
  * @param *motor:	Pointer to the motor data.
  *
  * @return The status of the function
***/
int8_t serialtx_send_processing(uint8_t status, uint16_t *x8r, int32_t *bno, uint16_t *motor) {
	serialtx_clear_buffer();

	serialtx_dma_buffer[0] = 0xFE;
	serialtx_dma_buffer[1] = 0xDD;

	uint8_t header_status_byte = (status << 4);
	if (onboard_led_get(ONBOARD_LED_RED))
		header_status_byte |= 0x01;
	if (onboard_led_get(ONBOARD_LED_ORANGE))
		header_status_byte |= 0x02;
	if (onboard_led_get(ONBOARD_LED_GREEN))
		header_status_byte |= 0x04;
	if (onboard_led_get(ONBOARD_LED_BLUE))
		header_status_byte |= 0x08;

	serialtx_dma_buffer[2] = header_status_byte;

	for (uint8_t i = 1; i <= 16; i++) {
		serialtx_dma_buffer[i * 2 + 1] = (uint8_t)(x8r[i - 1] >> 8);
		serialtx_dma_buffer[i * 2 + 2] = (uint8_t)(x8r[i - 1] & 0xFF);
	}

	for (uint8_t i = 35; i < (35 + 52); i += 4) {
		serialtx_dma_buffer[i] = (uint8_t)(bno[(i-34)/4] >> 24);
		serialtx_dma_buffer[i + 1] = (uint8_t)((bno[(i-34)/4] >> 16) & 0xFF);
		serialtx_dma_buffer[i + 2] = (uint8_t)((bno[(i-34)/4] >> 8) & 0xFF);
		serialtx_dma_buffer[i + 3] = (uint8_t)(bno[(i-34)/4] & 0xFF);
	}

	for (uint16_t i = 87; i < (87 + 12); i += 2) {
		serialtx_dma_buffer[i] = (uint8_t)(motor[(i - 87) / 2] >> 8);
		serialtx_dma_buffer[i + 1] = (uint8_t)(motor[(i - 87) / 2] & 0xFF);
	}

	serialtx_dma_buffer[99] = 0x0A;
	serialtx_send(100);
	//delay(50);

	return 0;
}

/** Send the Processing packet (including LED strip data) over the
  * serial line for the Drone Dashboard.
  *
  * @param status:	Value of the 4 LEDs (used for status signals).
  * @param *x8r:	Pointer to the radio transmitter channels data.
  * @param *bno:	Pointer to the BNO055 sensor data.
  * @param *motor:	Pointer to the motor data.
  * @param *ws2812:	Pointer to the LED strip data.
  *
  * @return The status of the function
***/
int8_t serialtx_send_processing_with_leds(uint8_t status, uint16_t *x8r, int32_t *bno, uint16_t *motor, uint8_t *ws2812) {
	serialtx_clear_buffer();

	serialtx_dma_buffer[0] = 0xFE;
	serialtx_dma_buffer[1] = 0xDD;

	uint8_t header_status_byte = (status << 4);
	if (onboard_led_get(ONBOARD_LED_RED))
		header_status_byte |= 0x01;
	if (onboard_led_get(ONBOARD_LED_ORANGE))
		header_status_byte |= 0x02;
	if (onboard_led_get(ONBOARD_LED_GREEN))
		header_status_byte |= 0x04;
	if (onboard_led_get(ONBOARD_LED_BLUE))
		header_status_byte |= 0x08;

	serialtx_dma_buffer[2] = header_status_byte;

	for (uint8_t i = 1; i <= 16; i++) {
		serialtx_dma_buffer[i * 2 + 1] = (uint8_t)(x8r[i - 1] >> 8);
		serialtx_dma_buffer[i * 2 + 2] = (uint8_t)(x8r[i - 1] & 0xFF);
	}

	for (uint8_t i = 35; i < (35 + 52); i += 4) {
		serialtx_dma_buffer[i] = (uint8_t)(bno[(i-34)/4] >> 24);
		serialtx_dma_buffer[i + 1] = (uint8_t)((bno[(i-34)/4] >> 16) & 0xFF);
		serialtx_dma_buffer[i + 2] = (uint8_t)((bno[(i-34)/4] >> 8) & 0xFF);
		serialtx_dma_buffer[i + 3] = (uint8_t)(bno[(i-34)/4] & 0xFF);
	}

	for (uint16_t i = 0; i < 252; i++)
		serialtx_dma_buffer[i + 87] = ws2812[i];

	for (uint16_t i = 339; i < (339 + 12); i += 2) {
		serialtx_dma_buffer[i] = (uint8_t)(motor[(i - 339) / 2] >> 8);
		serialtx_dma_buffer[i + 1] = (uint8_t)(motor[(i - 339) / 2] & 0xFF);
	}

	serialtx_dma_buffer[351] = 0x0A;
	serialtx_send(352);
	//delay(50);

	return 0;
}

/** Clear out the data transmission buffer.
  *
  * @return The status of the function.
***/
int8_t serialtx_clear_buffer() {
	for (uint16_t i = 0; i < SERIALTX_BUFFER_SIZE; i++)
		serialtx_dma_buffer[i] = 0;

	return 0;
}
