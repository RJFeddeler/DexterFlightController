#include "datalogger.h"
// WORK IN PROGRESS!!!

/** Enable clocks and configure GPIO pins .............
  *
  * @return The status of configuration.
***/
uint16_t datalogger_config() {

	return 0;
}

/** Initialize variables and prepare to begin normal operation.
  *
  * @return The status of initialization.
***/
uint16_t datalogger_init() {

	datalogger_last_state = 0;

	return 0;
}

/** Update loop to handle state switching and ..................
  *
  * @return The status of current update iteration.
***/
int8_t datalogger_update() {
	static uint32_t last_update = 0;

	if (clocks_check_sample_rate(&last_update, DATALOGGER_SAMPLE_RATE)) {
		if (state.status != datalogger_last_state) {
			datalogger_change_state(state.status);
			datalogger_last_state = state.status;
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
void datalogger_change_state(uint8_t new_state) {
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
