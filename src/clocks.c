#include "clocks.h"

/** SysTick interrupt handler that is set to interrupt every 1 uS
  * and increment the counting variable.
  *
  * @return Void.
***/
void SysTick_Handler(void) { sys_ticks++; }

/** Configure SysTick to interrupt every 1 uS.
  *
  * @return The status of configuration.
***/
uint16_t clocks_systick_config() {
	// Set system tick to every 1 uS
	if (SysTick_Config(SystemCoreClock / 1000000))
		return CLOCKS_ERROR_CODE;

	return 0;
}

/** Configure system clocks and set main clock to 168 MHz.
  *
  * @return The status of configuration.
***/
uint16_t clocks_clock_config() {
	// Reset clocks to default
	RCC_DeInit();

	// Enable external clock
	RCC_HSEConfig(RCC_HSE_ON);

	// Wait for clock to be ready
	ErrorStatus err = RCC_WaitForHSEStartUp();

	if (err == SUCCESS) {
		// Set system clock to 168 MHz
		RCC_PLLConfig(RCC_PLLSource_HSE, 8, 336, 2, 7);
		// Enable main PLL
		RCC_PLLCmd(ENABLE);
		// Wait for PLL ready
		uint32_t timedout_start;
		timed_out_init(&timedout_start);
		while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
			if (timed_out(timedout_start, 1000))
				return CLOCKS_ERROR_CODE;

		// Set flash latency
		FLASH_SetLatency(FLASH_Latency_5);

		// Set AHB clock to 168 MHz
		RCC_HCLKConfig(RCC_SYSCLK_Div1);
		// Set APB1 to 42 MHz
		RCC_PCLK1Config(RCC_HCLK_Div4);
		// Set APB2 to 84 MHz
		RCC_PCLK2Config(RCC_HCLK_Div2);
		// Set system clock to PLL
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

		return clocks_systick_config();
	}

	return CLOCKS_ERROR_CODE;
}

/** Get the current tick count, with each tick representing a microsecond.
  *
  * @return The current tick count.
***/
uint32_t clocks_get_ticks() { return sys_ticks; }

/** Get the current tick count divided by 1000 which represents milliseconds.
  *
  * @return The current tick count converted to milliSeconds.
***/
uint32_t clocks_get_millis() { return (sys_ticks / 1000); }

/** Determine if a specified amount of time has passed, which is useful
  * for timing an event to execute at a specific interval.
  *
  * @param *last_update:	Pointer to the tick count of the last time the event was executed
  * 				and it's value is automatically updated when the time interval is reached.
  * @param rate:		The rate, in Hz, at which the event should be executed (1-10000000 Hz).
  *
  * @return 1 when the desired time has expired, otherwise 0.
***/
int8_t clocks_check_sample_rate(uint32_t *last_update, uint16_t rate) {
	if (rate == 0)
		return 0;

	rate = clamp(rate, 1, 1000000);

	uint32_t ticks = sys_ticks;
	if (ticks < *last_update) {
		if ((0xFFFFFFFF - *last_update) + ticks > (1000000 / rate)) {
			*last_update += (1000000 / rate);
			return 1;
		}
	}
	else if (ticks > *last_update + (uint32_t)(1000000 / rate)) {
		*last_update += (1000000 / rate);
		return 1;
	}

	return 0;
}

/** Pauses code execution for a given amount of time in milliseconds.
  *
  * @param delay_ms:	The number of milliseconds to delay before resuming code execution.
  *
  * @return Void.
***/
void delay(uint32_t delay_ms) { delay_us(delay_ms * 1000); }

/** Pauses code execution for a given amount of time in microseconds.
  *
  * @param delay_ticks:	The number of ticks/microseconds to delay before resuming code execution.
  *
  * @return Void.
***/
void delay_us(uint32_t delay_ticks) {
	uint32_t timedout_start;
	timed_out_init(&timedout_start);
	while (!timed_out_us(timedout_start, delay_ticks));
}

/** Initializes the timed_out function by setting the start time.
  *
  * @param *start_tick:	Pointer to the start time.
  *
  * @return Void.
***/
void timed_out_init(uint32_t *start_tick) { *start_tick = sys_ticks; }

/** Determines if a specified amount of time has passed in order to detect if a code sequence is taking too long.
  *
  * @param start_tick:	The tick count of when the timer was started.
  * @param time_limit:	How long to allow (in milliseconds) before a time out is triggered.
  *
  * @return True if the amount of time since calling timed_out_init has passed the time
  * limit, otherwise false.
***/
bool timed_out(uint32_t start_tick, uint32_t time_limit) {
	return timed_out_us(start_tick, time_limit * 1000);
}

/** Determines if a specified amount of time has passed in order to detect if a code sequence is taking too long.
  *
  * @param start_tick:	The tick count of when the timer was started.
  * @param time_limit:	How long to allow (in microseconds) before a time out is triggered.
  *
  * @return True if the amount of time since calling timed_out_init has passed the time
  * limit, otherwise false.
***/
bool timed_out_us(uint32_t start_tick, uint32_t time_limit) {
	uint32_t current_time = sys_ticks;
	if (current_time < start_tick) {
		if (((0xFFFFFFFF - start_tick) + current_time) >= time_limit)
			return true;
	}
	else if (current_time >= (start_tick + time_limit))
		return true;

	return false;
}
