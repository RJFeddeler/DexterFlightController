#include "onboard_led.h"

/** Enable clock and configure GPIO pin D12 (Green LED), pin D13 (Orange LED),
  * pin D14 (Red LED), and pin D15 (Blue LED) as outputs.
  *
  * @return The status of configuration.
***/
uint16_t onboard_led_config() {
	GPIO_InitTypeDef 	GPIO_InitStructure;

	// Enable GPIO-D on AHB1
	RCC_AHB1PeriphClockCmd(ONBOARD_LED_RCC_GPIO, ENABLE);

	// Set up GPIO pins D12 (GREEN), D13 (ORANGE), D14 (RED), and D15 (BLUE)
	GPIO_InitStructure.GPIO_Pin = 	ONBOARD_LED_GREEN | ONBOARD_LED_ORANGE | ONBOARD_LED_RED | ONBOARD_LED_BLUE;
	GPIO_InitStructure.GPIO_Mode = 	GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed =	GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType =	GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = 	GPIO_PuPd_NOPULL;
	GPIO_Init(ONBOARD_LED_PORT, &GPIO_InitStructure);

	return 0;
}

/** Toggle an LED on or off based on its current state.
  *
  * @param color:	The color of the LED to toggle.
  *
  * @return Void.
***/
void onboard_led_toggle(uint16_t color) {
	GPIO_ToggleBits(ONBOARD_LED_PORT, color);
}

/** Turn on the LED of a given color.
  *
  * @param color:	The color of the LED to turn on.
  *
  * @return Void.
***/
void onboard_led_set(uint16_t color) {
	GPIO_SetBits(ONBOARD_LED_PORT, color);
}

/** Get the current state of a given color's LED.
  *
  * @param color:	The LED color to check.
  *
  * @return The value of whether the chosen LED is on.
***/
bool onboard_led_get(uint16_t color) {
	return GPIO_ReadOutputDataBit(ONBOARD_LED_PORT, color);
}

/** Turn off the LED of a given color.
  *
  * @param color:	The color of the LED to turn off.
  *
  * @return Void.
***/
void onboard_led_reset(uint16_t color) {
	GPIO_ResetBits(ONBOARD_LED_PORT, color);
}

/** Turn off all four LEDs.
  *
  * @return Void.
***/
void onboard_led_reset_all() {
	GPIO_ResetBits(ONBOARD_LED_PORT, ONBOARD_LED_GREEN);
	GPIO_ResetBits(ONBOARD_LED_PORT, ONBOARD_LED_ORANGE);
	GPIO_ResetBits(ONBOARD_LED_PORT, ONBOARD_LED_RED);
	GPIO_ResetBits(ONBOARD_LED_PORT, ONBOARD_LED_BLUE);
}

/** Turn on the LED of a given color for some time before turning it off.
  *
  * @param color:	The color of the LED to blink.
  *
  * @return Void.
***/
void onboard_led_blink(uint16_t color) {
	GPIO_SetBits(ONBOARD_LED_PORT, color);
	delay(ONBOARD_LED_DELAY_SINGLE);
	GPIO_ResetBits(ONBOARD_LED_PORT, color);
	delay(ONBOARD_LED_DELAY_SINGLE);
}

/** Cycle through an animation of the four LEDs in an infinite loop.
  *
  * @return Void.
***/
void onboard_led_blink_forever() {
	uint16_t led_all = ONBOARD_LED_GREEN | ONBOARD_LED_ORANGE | ONBOARD_LED_RED | ONBOARD_LED_BLUE;

	GPIO_ResetBits(ONBOARD_LED_PORT, led_all);

	while (1) {
		GPIO_ToggleBits(ONBOARD_LED_PORT, ONBOARD_LED_GREEN);
		delay(ONBOARD_LED_DELAY_FOREVER);
		GPIO_ToggleBits(ONBOARD_LED_PORT, ONBOARD_LED_ORANGE);
		delay(ONBOARD_LED_DELAY_FOREVER);
		GPIO_ToggleBits(ONBOARD_LED_PORT, ONBOARD_LED_RED);
		delay(ONBOARD_LED_DELAY_FOREVER);
		GPIO_ToggleBits(ONBOARD_LED_PORT, ONBOARD_LED_BLUE);
		delay(ONBOARD_LED_DELAY_FOREVER);
	}
}
