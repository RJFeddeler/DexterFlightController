#include "neom8n.h"
// WORK IN PROGRESS!!!

/** The interrupt handler ............
  *
  * @return Void.
***/
void USART2_IRQHandler(void) {
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
		// DO STUFF
	}
}

/** Enable clocks, configure GPIO pin A2 (Tx) and pin A3 (Rx) as
  * Alternate Function (USART2), and enable interrupt.
  *
  * @return The status of configuration.
***/
uint16_t neom8n_config() {
	GPIO_InitTypeDef 	GPIO_InitStructure;
	USART_InitTypeDef 	USART_InitStructure;
	NVIC_InitTypeDef 	NVIC_InitStructure;

	// Enable GPIO-A on AHB1
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	// Enable USART2 on APB1
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	// Set up GPIO pins A2 (Tx) & A3 (Rx)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// Connect A2 (Tx) & A3 (Rx) AF to USART2
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

	// Set up USART2
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART2, &USART_InitStructure);

	// Set up interrupt for USART2
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// Enable USART2
	USART_Cmd(USART2, ENABLE);

	return 0;
}

/** Initialize variables and prepare to begin normal operation.
  *
  * @return The status of initialization.
***/
uint16_t neom8n_init() {
	neom8n_last_state = 0;

	return 0;
}

/** Update loop to handle state switching.
  *
  * @return The status of current update iteration.
***/
int8_t neom8n_update() {
	// Check for state change
	if (state.status != neom8n_last_state)
		neom8n_change_state(state.status);

	return 0;
}

/** Used to change settings, enable/disable features, or trigger actions when
  * switching between two different states of operation.
  *
  * @param new_state:	The state being switched to.
  *
  * @return Void.
***/
void neom8n_change_state(uint8_t new_state) {
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
}
