#include "state.h"
#include "clocks.h"
#include "onboard_led.h"
#include "other_stuff.h"
#include "serialtx.h"
#include "datalogger.h"
#include "battery.h"
#include "x8r.h"
#include "bno055.h"
#include "neom8n.h"
#include "bmp280.h"
#include "hcsr05.h"
#include "ws2812.h"
#include "motor.h"

void error_state(uint16_t status);

int main(void) {
	uint16_t error_status = 0;
	struct PID pid[3];
	uint16_t rx_arming_rules[STATE_RX_CHANNEL_COUNT][2];
	uint16_t periphs = 0;

	pid_set(	&pid[STATE_AXIS_PITCH], 	4.1f,  0.0f,  0.4f		);	//	   4 / 1 / 0.5		Good
	pid_set(	&pid[STATE_AXIS_ROLL], 		4.1f,  0.0f,  0.4f		);	//	   4 / 1 / 0.5		Starting
	pid_set(	&pid[STATE_AXIS_YAW], 		9.0f,  0.0f,  0.9f		);	//	   9 / 1 / 1.5		Point

	state_rx_reset_arming_rules(rx_arming_rules);
	state_rx_add_arming_rule(rx_arming_rules, STATE_RX_CHANNEL_THROTTLE, STATE_RX_RULE_EQUAL_TO, STATE_RX_VALUE_MIN);
	state_rx_add_arming_rule(rx_arming_rules, STATE_RX_CHANNEL_AILERON, STATE_RX_RULE_EQUAL_TO, STATE_RX_VALUE_MIDPOINT);
	state_rx_add_arming_rule(rx_arming_rules, STATE_RX_CHANNEL_ELEVATOR, STATE_RX_RULE_EQUAL_TO, STATE_RX_VALUE_MIDPOINT);
	state_rx_add_arming_rule(rx_arming_rules, STATE_RX_CHANNEL_RUDDER, STATE_RX_RULE_EQUAL_TO, STATE_RX_VALUE_MAX);
	state_rx_add_arming_rule(rx_arming_rules, STATE_RX_CHANNEL_TOGGLE_R_2POS, STATE_RX_RULE_EQUAL_TO, STATE_RX_POSITION_0);
	state_rx_add_arming_rule(rx_arming_rules, STATE_RX_CHANNEL_TOGGLE_L_3POS_2, STATE_RX_RULE_EQUAL_TO, STATE_RX_POSITION_0);
	state_rx_add_arming_rule(rx_arming_rules, STATE_RX_CHANNEL_TOGGLE_L_2POS, STATE_RX_RULE_EQUAL_TO, STATE_RX_POSITION_0);

	periphs = STATE_PERIPH_MOTOR | STATE_PERIPH_BATTERY | STATE_PERIPH_X8R | STATE_PERIPH_BNO055 | STATE_PERIPH_WS2812;


	//   CONFIGURATION   //
	error_status |= clocks_clock_config();
	error_status |= onboard_led_config();
	error_status |= serialtx_config();
	error_status |= state_init(periphs, pid, rx_arming_rules);
	if (error_status != 0x00)
		error_state(error_status);

	if (state_periph_enabled(STATE_PERIPH_BATTERY))
		error_status |= battery_config();
	if (state_periph_enabled(STATE_PERIPH_X8R))
		error_status |= x8r_config();
	if (state_periph_enabled(STATE_PERIPH_BNO055))
		error_status |= bno055_config();
	if (state_periph_enabled(STATE_PERIPH_NEOM8N))
		error_status |= neom8n_config();
	if (state_periph_enabled(STATE_PERIPH_BMP280))
		error_status |= bmp280_config();
	if (state_periph_enabled(STATE_PERIPH_HCSR05))
		error_status |= hcsr05_config();
	if (state_periph_enabled(STATE_PERIPH_WS2812))
		error_status |= ws2812_config();
	if (state_periph_enabled(STATE_PERIPH_MOTOR))
		error_status |= motor_config();
	if (state_periph_enabled(STATE_PERIPH_DATALOGGER))
		error_status |= datalogger_config();
	if (error_status != 0x00)
		error_state(error_status);

	//   INITIALIZATION   //
	if (state_periph_enabled(STATE_PERIPH_BATTERY))
		error_status |= battery_init();
	if (state_periph_enabled(STATE_PERIPH_X8R))
		error_status |= x8r_init();
	if (state_periph_enabled(STATE_PERIPH_BNO055))
		error_status |= bno055_init(false);
	if (state_periph_enabled(STATE_PERIPH_NEOM8N))
		error_status |= neom8n_init();
	if (state_periph_enabled(STATE_PERIPH_BMP280))
		error_status |= bmp280_init();
	if (state_periph_enabled(STATE_PERIPH_HCSR05))
		error_status |= hcsr05_init();
	if (state_periph_enabled(STATE_PERIPH_WS2812))
		error_status |= ws2812_init();
	if (state_periph_enabled(STATE_PERIPH_MOTOR))
		error_status |= motor_init();
	if (state_periph_enabled(STATE_PERIPH_DATALOGGER))
		error_status |= datalogger_init();
	if (error_status != 0x00)
		error_state(error_status);

	//	MAIN LOOP	//
	while(1) {
		//   STATE   //
		state_update();

		//   BATTERY   //
		if (state_periph_enabled(STATE_PERIPH_BATTERY))
			if (battery_update() == 1) {}

		//   X8R (RADIO RECEIVER)   //
		if (state_periph_enabled(STATE_PERIPH_X8R))
			if (x8r_update() == 1) {}

		//   BNO055 (IMU)   //
		int8_t imu_result = 0;
		if (state_periph_enabled(STATE_PERIPH_BNO055)) {
			imu_result = bno055_update();
			if (imu_result == 1) {
				//bmp280_integrate_accz(state.linear_acc_notilt);

				if (state.status == STATE_SYS_CALIBRATING) {
					onboard_led_reset_all();
					if (state.imu_calibration_percent > 45)
						onboard_led_set(ONBOARD_LED_RED);
					if (state.imu_calibration_percent > 60)
						onboard_led_set(ONBOARD_LED_ORANGE);
					if (state.imu_calibration_percent > 75)
						onboard_led_set(ONBOARD_LED_GREEN);
					if (state.imu_calibration_percent == 100)
						onboard_led_set(ONBOARD_LED_BLUE);
				}
			}
		}

		//   BMP280 (PRESSURE)   //
		if (state_periph_enabled(STATE_PERIPH_BMP280))
			if (bmp280_update() == 1) {}

		//   HCSR05 (ULTRASONIC)   //
		if (state_periph_enabled(STATE_PERIPH_HCSR05))
			if (hcsr05_update() == 1) {}

		//   NEO-M8N (GPS)   //
		if (state_periph_enabled(STATE_PERIPH_NEOM8N))
			if (neom8n_update() == 1) {} // SET UP DMA (DMA1_CHANNEL4_STREAM5)

		//   MOTORS   //
		if (state_periph_enabled(STATE_PERIPH_MOTOR))
			if (motor_update() == 1) {}

		//   SD CARD (DATA LOGGER)   //
		if (state_periph_enabled(STATE_PERIPH_DATALOGGER)) {
			if (imu_result != 1) {
				if (datalogger_update() == 1) {}
			}
		}

		//   WS2812 (ADDRESSABLE RGB LED STRIP)   //
		if (state_periph_enabled(STATE_PERIPH_WS2812)) {
			if (imu_result != 1) {
				if (ws2812_update() == 1) {}
			}
		}
	}
}

void error_state(uint16_t status) {
	state_reset(STATE_SYS_IDLE);

	// Error not related to a peripheral
	if (status >= 0x200)
		onboard_led_blink_forever();

	// Peripheral error
	while (1) {
		onboard_led_reset_all();

		if (status & STATE_ERROR_PERIPH_MOTOR || status & STATE_ERROR_PERIPH_BNO055)
			onboard_led_set(ONBOARD_LED_BLUE);
		if (status & STATE_ERROR_PERIPH_BMP280 || status & STATE_ERROR_PERIPH_BATTERY)
			onboard_led_set(ONBOARD_LED_ORANGE);
		if (status & STATE_ERROR_PERIPH_HCSR05 || status & STATE_ERROR_PERIPH_X8R)
			onboard_led_set(ONBOARD_LED_RED);
		if (status & STATE_ERROR_PERIPH_NEOM8N || status & STATE_ERROR_PERIPH_WS2812 || status & STATE_ERROR_PERIPH_DATALOGGER)
			onboard_led_set(ONBOARD_LED_GREEN);

		delay(500);

		uint16_t led[] = { ONBOARD_LED_RED, ONBOARD_LED_ORANGE, ONBOARD_LED_GREEN, ONBOARD_LED_BLUE };

		onboard_led_reset_all();
		for (uint8_t i = 0; i < 8; i++) {
			onboard_led_toggle(led[i % 4]);
			delay(100);
		}
	}
}
