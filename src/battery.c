#include "battery.h"
// WORK IN PROGRESS!!!

/** Enable clocks, configure GPIO pin C4 (voltage) and pin C5 (current) as
  * Analog Read (ADC1) using DMA2_Stream0_Channel0.
  *
  * @return The status of configuration.
***/
uint16_t battery_config() {
	GPIO_InitTypeDef		GPIO_InitStructure;
	ADC_InitTypeDef			ADC_InitStructure;
	ADC_CommonInitTypeDef	ADC_CommonInitStructure;
	DMA_InitTypeDef			DMA_InitStruct;

	// Enable Clocks
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,  ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,  ENABLE);

	// Configure and Enable Pins C4 & C5 as Analog
	GPIO_InitStructure.GPIO_Pin = 	GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = 	GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_Speed =	GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = 	GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// Configure ADC (Analog to Digital Converter)
	ADC_DeInit();
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 2;
	ADC_Init(ADC1, &ADC_InitStructure);

	// Configure ADC clock and sampling values
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInit(&ADC_CommonInitStructure);

	// Configure DMA to read the ADC's data register
	DMA_DeInit(DMA2_Stream0);
	DMA_InitStruct.DMA_Channel = DMA_Channel_0;
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
	DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)battery_dma_buffer;
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStruct.DMA_BufferSize =  2;
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStruct.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_Init(DMA2_Stream0, &DMA_InitStruct);

	// Configure ADC Channels
	ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 1, ADC_SampleTime_480Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 2, ADC_SampleTime_480Cycles);

	// Trigger DMA to restart after reading both voltage and current
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

	// Enable DMA for ADC and start it
	ADC_DMACmd(ADC1, ENABLE);
	DMA_Cmd(DMA2_Stream0, ENABLE);

	return 0;
}

/** Initialize variables and prepare to begin normal operation.
  *
  * @return The status of initialization.
***/
uint16_t battery_init() {
	// Reset DMA buffer to zeroes
	for (uint8_t i = 0; i < 2; i++)
		battery_dma_buffer[i] = 0;

	// Enable and begin ADC readings
	ADC_Cmd(ADC1, ENABLE);
	ADC_SoftwareStartConv(ADC1);

	// Set state parameters
	state_set_calibrated(STATE_PERIPH_BATTERY);
	battery_last_state = 0;

	return 0;
}

/** Update loop to handle state switching and conversion from raw to cooked data.
  *
  * @return The status of current update iteration.
***/
int8_t battery_update() {
	static uint32_t last_update = 0;

	if (clocks_check_sample_rate(&last_update, BATTERY_SAMPLE_RATE)) {
		// Check for state change
		if (state.status != battery_last_state)
			battery_change_state(state.status);

		// Calculate voltage and current from raw sensor readings
		state.voltage = (battery_dma_buffer[0] / 63.69f) * 0.805664f;
		state.current = (battery_dma_buffer[1] / 18.3f) * 0.805664f;

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
void battery_change_state(uint8_t new_state) {
	// There are no changes made between states for the battery (for now)
	switch (new_state) {
	case STATE_SYS_CALIBRATING:
		break;
	case STATE_SYS_IDLE:
		break;
	case STATE_SYS_ARMING:
		break;
	case STATE_SYS_ARMED:
		break;
	}

	// Update state variable used to check for changes
	battery_last_state = new_state;
}
