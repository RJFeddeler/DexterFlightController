#include "state.h"

/** Initialize variables and prepare to begin normal operation.
  *
  * @param periphs:	Enabled peripherals.
  * @param pid:		PID gain coefficients.
  * @param rules[][]:	List of rules for the radio transmitter switch positions to be in to enable arming.
  *
  * Returns The status of initialization.
***/
uint16_t state_init(uint16_t periphs, struct PID pid[3], uint16_t rules[STATE_RX_CHANNEL_COUNT][2]) {
	if (!periphs)
		return STATE_ERROR_STATE;

	state_reset(STATE_SYS_CALIBRATING);
	waypoint_reset(&home);
	state_periphs = periphs;

	for (uint8_t i = 0; i < 3; i++)
		pid_set(&state.pid[i], pid[i].p, pid[i].i, pid[i].d);

	for (uint8_t i = 0; i < STATE_RX_CHANNEL_COUNT; i++) {
		if (rules[i][0] == 0) {
			state_rx_arming_position[i][0] = STATE_RX_RULE_NONE;
			state_rx_arming_position[i][1] = STATE_RX_VALUE_MIN;
		}
		else {
			state_rx_arming_position[i][0] = rules[i][0];
			state_rx_arming_position[i][1] = rules[i][1];
		}
	}

	return 0;
}

/** Update loop to handle state switching, failsafe and error handling,
  * and sending the Drone Dashboard packet.
  *
  * @return The status of current update iteration.
***/
int8_t state_update() {
	static uint32_t last_update = 0;
	static uint32_t arming_start = 0;

	if (!clocks_check_sample_rate(&last_update, STATE_SAMPLE_RATE))
		return 0;

	switch (state.status) {
	case STATE_SYS_CALIBRATING:
		if (state_calibrated == state_periphs)
			state.status = STATE_SYS_IDLE;

		break;
	case STATE_SYS_IDLE:
		if ((state.imu_failsafe_count == 0) && state_rx_ready_to_arm()) {
			arming_start = clocks_get_millis();
			state.status = STATE_SYS_ARMING;
		}
		else if (state_rx_get_switch_position(state.rx_channels[STATE_RX_CHANNEL_TOGGLE_L_3POS_2]) == 2) {
			uint16_t pause = 100;

			serialtx_send_dec_int32(12345);
			delay(pause);
			for (uint8_t i = 0; i < 3; i++) {
				for (uint8_t j = 0; j < STATE_AUTOTUNE_STEP_COUNT; j++) {
					serialtx_send_dec_int32(state_autotune_test_history[i][j].axis);
					delay(pause);
					serialtx_send_dec_int32(state_autotune_test_history[i][j].start_tick);
					delay(pause);
					serialtx_send_dec_int32(state_autotune_test_history[i][j].start_heading * 1000);
					delay(pause);
					serialtx_send_dec_int32(state_autotune_test_history[i][j].step);
					delay(pause);
					for (uint8_t k = 0; k < 3; k++) {
						serialtx_send_dec_int32(state_autotune_test_history[i][j].pid[k].p * 1000);
						delay(pause);
						serialtx_send_dec_int32(state_autotune_test_history[i][j].pid[k].d * 1000);
						delay(pause);
					}
					serialtx_send_dec_int32(state_autotune_test_history[i][j].ticks_to_desired_angle);
					delay(pause);
					serialtx_send_dec_int32(state_autotune_test_history[i][j].ticks_to_end_of_bounce);
					delay(pause);
					serialtx_send_dec_int32(state_autotune_test_history[i][j].overshoot * 1000);
					delay(pause);
					serialtx_send_dec_int32(state_autotune_test_history[i][j].bounce * 1000);
					delay(pause);
				}
			}
			serialtx_send_dec_int32(54321);
			delay(pause);
		}

		break;
	case STATE_SYS_ARMING:
		if ((state.imu_failsafe_count == 0) && state_rx_ready_to_arm()) {
				uint32_t millis = clocks_get_millis();
				if (millis < arming_start) {
					if ((millis + (0xFFFFFFFF - arming_start)) > STATE_ARMING_TIME)
						state.status = STATE_SYS_ARMED;
				}
				else {
					if (millis > (arming_start + STATE_ARMING_TIME))
						state.status = STATE_SYS_ARMED;
				}
		}
		else
			state.status = STATE_SYS_IDLE;

		break;
	case STATE_SYS_ARMED:
		// 	CUT OFF SWITCH	//
		if (state_rx_get_switch_position(state.rx_channels[STATE_RX_CHANNEL_TOGGLE_R_2POS]) == 3)
			state_reset(STATE_SYS_IDLE);

		//	RX FAILSAFE	//
		if (state.rx_failsafe_count > 0)
			{}
		if (state.rx_failsafe_count > STATE_RX_FAILSAFE_STAGE_1)
			state.mode_hover = true;
		if (state.rx_failsafe_count > STATE_RX_FAILSAFE_STAGE_2)
			state_reset(STATE_SYS_IDLE);

		//	IMU FAILSAFE	//
		if (state.imu_failsafe_count > 0)
			{}
		if (state.imu_failsafe_count > STATE_IMU_FAILSAFE_STAGE_1)
			state_reset(STATE_SYS_IDLE);
		if (state.imu_failsafe_count > STATE_IMU_FAILSAFE_STAGE_2)
			state_reset(STATE_SYS_IDLE);

		break;
	}

	//   PROCESSING PACKET   //
	if (state.status < STATE_SYS_ARMED) {
		if (state.rx_channels[STATE_RX_CHANNEL_TOGGLE_L_3POS_3] >= (STATE_RX_VALUE_MIDPOINT - STATE_RX_DEADZONE)) {
			static uint32_t last_packet = 0;
			if (last_packet == 0)
				last_packet = clocks_get_ticks();

			const uint16_t packet_rate = 100;
			if (clocks_check_sample_rate(&last_packet, packet_rate)) {
				uint8_t status;

				if (state.status == STATE_SYS_CALIBRATING)
					status = 0;
				else if (state.status == STATE_SYS_IDLE)
					status = 1;
				else if (state.status == STATE_SYS_ARMING)
					status = 2;
				else if (state.status == STATE_SYS_ARMED)
					status = 3;
				else
					status = 4;


				int32_t bno[] = {(int32_t)(state.pitch * 100), (int32_t)(state.roll * 100), (int32_t)(state.heading * 100),
						(int32_t)(state.gyr.x * 100), (int32_t)(state.gyr.y * 100), (int32_t)(state.gyr.z * 100),
						(int32_t)(state.acc.x * 100), (int32_t)(state.acc.y * 100), (int32_t)(state.acc.z * 100),
						(int32_t)(state.mag.x * 100), (int32_t)(state.mag.y * 100), (int32_t)(state.mag.z * 100),
						(int32_t)(state.temperature_imu * 100)
						};

				uint16_t motor[] = { state.motors[0], state.motors[1], state.motors[2], state.motors[3], state.motors[4], state.motors[5] };

				if (state.rx_channels[STATE_RX_CHANNEL_TOGGLE_L_3POS_3] < (STATE_RX_VALUE_MAX - STATE_RX_DEADZONE))
					serialtx_send_processing(status, state.rx_channels, bno, motor);
				else {
					uint8_t ws2812[252] = { 0 };
					//ws2812_get_processing(&ws2812);
					serialtx_send_processing_with_leds(status, state.rx_channels, bno, motor, ws2812);
				}
			}
		}
	}
	//   END PROCESSING PACKET   //

	return 1;
}

/** Set a peripheral as calibrated.
  *
  * @param periph:	The peripheral to mark as calibrated.
  *
  * @return Void.
***/
void state_set_calibrated(uint16_t periph) {
	state_calibrated |= periph;
}

/** Set a peripheral as NOT calibrated.
  *
  * @param periph: The peripheral to mark as uncalibrated.
  *
  * @return Void.
***/
void state_set_uncalibrated(uint16_t periph) {
	state_calibrated &= ~periph;
}

/** Determines if a specified peripheral is enabled.
  *
  * @param periph:	The peripheral to check.
  *
  * @return True if the peripheral is enabled, otherwise False.
***/
bool state_periph_enabled(uint16_t periph) {
	if (state_periphs & periph)
		return true;

	return false;
}

/** Change settings based on the toggling of switches on the radio transmitter.
  *
  * @return The status of the function.
***/
int8_t state_rx_handle_switch_changes() {
	static uint16_t last_channels[STATE_RX_CHANNEL_COUNT];

	if (!state_periph_enabled(STATE_PERIPH_X8R) || !(state_calibrated & STATE_PERIPH_X8R))
		return -1;

	// First run through, just fill up last_channels
	if (last_channels[0] == 0) {
		for (uint8_t i = 0; i < STATE_RX_CHANNEL_COUNT; i++)
			last_channels[i] = state.rx_channels[i];

		return 0;
	}

	for (uint8_t i = 0; i < STATE_RX_CHANNEL_COUNT; i++) {
		if (state.rx_channels[i] != last_channels[i]) {

		}
	}

	return 0;
}

/** Determines if the radio sticks and switches are in position to arm the copter.
  *
  * @return True if the radio transmitter's switches are in the correct positions to arm, otherwise False.
***/
bool state_rx_ready_to_arm() {
	uint16_t rule, value;

	if (state.rx_failsafe_count > 0)
		return false;

	for (uint8_t i = 0; i < 4; i++)
		if (state_rx_arming_position[i][0] == STATE_RX_RULE_NONE)
			return false;

	for (uint8_t i = 0; i < STATE_RX_CHANNEL_COUNT; i++) {
		rule = state_rx_arming_position[i][0];

		if (rule == STATE_RX_RULE_NONE)
			continue;

		value = state_rx_arming_position[i][1];

		if ((rule & STATE_RX_RULE_EQUAL_TO) && (abs((int32_t)value - state.rx_channels[i]) <= STATE_RX_DEADZONE))
			continue;
		if ((rule & STATE_RX_RULE_GREATER_THAN) && (state.rx_channels[i] > (value + STATE_RX_DEADZONE)))
			continue;
		if ((rule & STATE_RX_RULE_LESS_THAN) && (state.rx_channels[i] < (value - STATE_RX_DEADZONE)))
			continue;

		return false;
	}

	return true;
}

/** Add a radio transmitter stick or switch position rule to the set of arming rules.
  *
  * @param rules[][]:	The array of arming rules to add to.
  * @param channel:	The channel of the radio transmitter for the new rule.
  * @param rule:	The logic statement of the new rule.
  * @param value:	The value of the channel for the new rule.
  *
  * @return The status of the function.
***/
int8_t state_rx_add_arming_rule(uint16_t rules[STATE_RX_CHANNEL_COUNT][2], uint8_t channel, uint16_t rule, uint16_t value) {
	if (channel >= STATE_RX_CHANNEL_COUNT)
		return -1;

	if ((rule & 0xFFF8) > 0)
		return -1;

	if ((value < STATE_RX_VALUE_MIN) || (value > STATE_RX_VALUE_MAX))
		return -1;

	rules[channel][0] = rule;
	rules[channel][1] = value;

	return 0;
}

/** Erase all of the radio transmitter arming rules.
  *
  * @param rules[][]:	The array that holds the arming rules.
  *
  * @return Void.
***/
void state_rx_reset_arming_rules(uint16_t rules[STATE_RX_CHANNEL_COUNT][2]) {
	for (uint8_t i = 0; i < STATE_RX_CHANNEL_COUNT; i++) {
		rules[i][0] = STATE_RX_RULE_NONE;
		rules[i][1] = STATE_RX_VALUE_MIN;
	}
}

/** Get the value of a radio transmitter switch position.
  *
  * @param switch_value:	The number of the switch to get.
  *
  * @return The position of the switch (1 = LOW, 2 = MID, 3 = HIGH).
***/
uint8_t state_rx_get_switch_position(uint16_t switch_value) {
	int32_t val = abs((int32_t)switch_value - STATE_RX_POSITION_0);
	if (val < STATE_RX_DEADZONE)
		return 1;

	val = abs((int32_t)switch_value - STATE_RX_POSITION_1);
	if (val < STATE_RX_DEADZONE)
		return 2;

	val = abs((int32_t)switch_value - STATE_RX_POSITION_2);
	if (val < STATE_RX_DEADZONE)
		return 3;

	return 0;
}

/** Set the PID auto-tune parameters (EXTREMELY EXPERIMENTAL)
  *
  * @param p_range_min_pitch:
  * @param p_range_max_pitch:
  * @param p_range_min_roll:
  * @param p_range_max_roll:
  * @param p_range_min_yaw:
  * @param p_range_max_yaw:
  *
  * @return Void.
***/
void state_autotune_set(float p_range_min_pitch, float p_range_max_pitch, float p_range_min_roll, float p_range_max_roll, float p_range_min_yaw, float p_range_max_yaw) {
	state_autotune_range[STATE_AXIS_PITCH].axis.min = p_range_min_pitch;
	state_autotune_range[STATE_AXIS_PITCH].axis.max = p_range_max_pitch;
	state_autotune_range[STATE_AXIS_PITCH].current_value = p_range_min_pitch;
	state_autotune_range[STATE_AXIS_PITCH].step = (p_range_max_pitch - p_range_min_pitch) / (STATE_AUTOTUNE_STEP_COUNT - 1);

	state_autotune_range[STATE_AXIS_ROLL].axis.min = p_range_min_roll;
	state_autotune_range[STATE_AXIS_ROLL].axis.max = p_range_max_roll;
	state_autotune_range[STATE_AXIS_ROLL].current_value = p_range_min_roll;
	state_autotune_range[STATE_AXIS_ROLL].step = (p_range_max_roll - p_range_min_roll) / (STATE_AUTOTUNE_STEP_COUNT - 1);

	state_autotune_range[STATE_AXIS_YAW].axis.min = p_range_min_yaw;
	state_autotune_range[STATE_AXIS_YAW].axis.max = p_range_max_yaw;
	state_autotune_range[STATE_AXIS_YAW].current_value = p_range_min_yaw;
	state_autotune_range[STATE_AXIS_YAW].step = (p_range_max_yaw - p_range_min_yaw) / (STATE_AUTOTUNE_STEP_COUNT - 1);
}

/** Reset the variables used during the PID auto-tune procedure.
  *
  * @param *test:	Pointer to the structure that holds the test parameters.
  *
  * @return Void.
***/
void state_autotune_reset_test(struct AutotuneTest *test) {
	for (uint8_t i = 0; i < 3; i++)
		pid_zero(&test->pid[i]);

	test->axis = 0;
	test->overshoot = STATE_AUTOTUNE_MAX_OVERSHOOT;
	test->bounce = STATE_AUTOTUNE_MAX_BOUNCE;
	test->start_tick = 0;
	test->start_heading = 0.0f;
	test->ticks_to_desired_angle = 0;
	test->ticks_to_end_of_bounce = 0;
	test->step = STEP_0toNEG;
}

/** Get the current throttle value based on mode and radio transmitter stick position.
  *
  * @return The current throttle value in the range of the ESC PWM data.
***/
int32_t state_get_throttle() {
	if (state.mode_hover)	// FOR NOW, HOVER MODE = AUTO LEVELING AND THROTTLE SET AT 20% (NEEDS SMARTER METHOD)
		return scale(20, 1, 100, STATE_MOTOR_VALUE_MIN, STATE_MOTOR_VALUE_MAX);
	else if ((state.rx_channels[STATE_RX_CHANNEL_THROTTLE] - STATE_RX_VALUE_MIN) > STATE_RX_DEADZONE)
		return scale(state.rx_channels[STATE_RX_CHANNEL_THROTTLE], STATE_RX_VALUE_MIN, STATE_RX_VALUE_MAX, STATE_MOTOR_VALUE_MIN, STATE_MOTOR_VALUE_MAX);
	else
		return STATE_MOTOR_VALUE_MIN;
}

/** Reset the state parameters.
  *
  * @param status:	The status to start the state in
  * 			(Should be calibrating mode unless coming out of failsafe).
  *
  * @return Void.
***/
void state_reset(uint8_t status) {
	state.status = status;
	state.mode_hover = false;
	state.mode_autotune = AT_OFF;
	state.imu_calibration_percent = 0;

	state.voltage = 0.0f;
	state.current = 0.0f;

	vector_zero(&state.gyr);
	vector_zero(&state.acc);
	vector_zero(&state.mag);

	vector_zero(&state.gravity);
	vector_zero(&state.linear_acc);
	vector_zero(&state.linear_acc_notilt);

	state.temperature_imu = 0.0f;

	state.velocity = 0.0f;
	state.velocity_gps = 0.0f;
	state.climb_acc = 0.0f;
	state.climb_velocity = 0.0f;
	state.horizon_velocity = 0.0f;

	state.latitude = 0.0f;
	state.longitude = 0.0f;

	state.temperature_bp = 0.0f;
	state.pressure = 0.0f;
	state.altitude = 0.0f;

	state.alt_bp = 0.0f;
	state.alt_us = 0.0f;
	state.alt_gps = 0.0f;

	state.pitch = 0.0f;
	state.roll = 0.0f;
	state.heading = 0.0f;

	quaternion_zero(&state.q);

	state.imu_failsafe_count = 0;
	state.rx_failsafe_count = 0;

	for (uint8_t i = 0; i < STATE_RX_CHANNEL_COUNT; i++)
		state.rx_channels[i] = 0;

	for (uint8_t i = 0; i < STATE_MOTOR_COUNT; i++)
		state.motors[i] = 0;

	state_calibrated = 0;
	pid_reset();

	// NEED TO ADD GPS PACKET STUFF
}

/** Create a waypoint.
  *
  * @param *w:	Pointer to the waypoint structure to save the data in.
  *
  * @return Void.
***/
void waypoint_create(struct waypoint *w) {
	w->latitude = state.latitude;
	w->longitude = state.longitude;

	w->temperature = state.temperature_imu;
	w->pressure = state.pressure;
	w->altitude = state.altitude;

	w->climb_velocity = state.climb_velocity;
	w->horizon_velocity = state.horizon_velocity;

	w->pitch = state.pitch;
	w->roll = state.roll;
	w->heading = state.heading;

	w->q = state.q;
}

/** Reset a waypoint.
  *
  * @param *w:	Pointer to the waypoint structure to reset.
  *
  * @return Void.
***/
void waypoint_reset(struct waypoint *w) {
	w->latitude = 0.0f;
	w->longitude = 0.0f;

	w->temperature = 0.0f;
	w->pressure = 0.0f;
	w->altitude = 0.0f;

	w->climb_velocity = 0.0f;
	w->horizon_velocity = 0.0f;

	w->pitch = 0.0f;
	w->roll = 0.0f;
	w->heading = 0.0f;

	quaternion_zero(&w->q);
}

/** Go through an iteration of the PID control loop.
  *
  * @return The status of the function.
***/
int8_t pid_update() {
	static uint32_t last_update = 0;
	float delta_t, desired_angle[3], angle_error[3], gyro[3];
	float p_term, i_term, d_term;

	gyro[STATE_AXIS_PITCH] = state.gyr.x;
	gyro[STATE_AXIS_ROLL] = state.gyr.y;
	gyro[STATE_AXIS_YAW] = state.gyr.z;

	desired_angle[STATE_AXIS_PITCH] = 0.0f;
	desired_angle[STATE_AXIS_ROLL] = 0.0f;
	desired_angle[STATE_AXIS_YAW] = state.heading;

	if (!state.mode_hover && (state.mode_autotune != AT_RUNNING)) {
		if (abs(state.rx_channels[STATE_RX_CHANNEL_ELEVATOR] - STATE_RX_VALUE_MIDPOINT) > STATE_RX_DEADZONE) {
			desired_angle[STATE_AXIS_PITCH] =
					scalef((float)(STATE_RX_VALUE_MAX - (state.rx_channels[STATE_RX_CHANNEL_ELEVATOR] - (float)STATE_RX_VALUE_MIN)),
						(float)STATE_RX_VALUE_MIN, (float)STATE_RX_VALUE_MAX, (float)(-STATE_MAX_TILT), (float)STATE_MAX_TILT);
		}

		if (abs(state.rx_channels[STATE_RX_CHANNEL_AILERON] - STATE_RX_VALUE_MIDPOINT) > STATE_RX_DEADZONE) {
			desired_angle[STATE_AXIS_ROLL] =
					scalef(state.rx_channels[STATE_RX_CHANNEL_AILERON],
						(float)STATE_RX_VALUE_MIN, (float)STATE_RX_VALUE_MAX, (float)(-STATE_MAX_TILT), (float)STATE_MAX_TILT);
		}

		if (abs(state.rx_channels[STATE_RX_CHANNEL_RUDDER] - STATE_RX_VALUE_MIDPOINT) > STATE_RX_DEADZONE) {
			float yaw_rate =
					scalef((float)(STATE_RX_VALUE_MAX - (state.rx_channels[STATE_RX_CHANNEL_RUDDER] - (float)STATE_RX_VALUE_MIN)),
						(float)STATE_RX_VALUE_MIN, (float)STATE_RX_VALUE_MAX, -1.0f, 1.0f);

			desired_angle[STATE_AXIS_YAW] += (yaw_rate * STATE_MAX_YAW);
		}
	}
	else if (state.mode_autotune == AT_RUNNING) {
		if (state_autotune_current_test->step == STEP_0toNEG)
			desired_angle[state_autotune_current_test->axis] = -STATE_AUTOTUNE_TILT;
		else if (state_autotune_current_test->step == STEP_NEGtoPOS)
			desired_angle[state_autotune_current_test->axis] = STATE_AUTOTUNE_TILT;
		else if (state_autotune_current_test->step == STEP_POStoNEG)
			desired_angle[state_autotune_current_test->axis] = -STATE_AUTOTUNE_TILT;

		if (state_autotune_current_test->axis == STATE_AXIS_YAW) {
			int8_t direction = 0;
			if (abs_f(desired_angle[STATE_AXIS_YAW]) > 0.0f)
				direction = ((desired_angle[STATE_AXIS_YAW] < 0) ? -1 : 1);
			desired_angle[STATE_AXIS_YAW] = state_autotune_current_test->start_heading + (STATE_AUTOTUNE_TILT * (float)direction);
		}
	}

	angle_error[STATE_AXIS_PITCH] = desired_angle[STATE_AXIS_PITCH] - state.pitch;
	angle_error[STATE_AXIS_ROLL] = desired_angle[STATE_AXIS_ROLL] - state.roll;
	angle_error[STATE_AXIS_YAW] = desired_angle[STATE_AXIS_YAW] - state.heading;

	if (last_update == 0)
		last_update = clocks_get_ticks();

	uint32_t ticks = clocks_get_ticks();
	delta_t = (ticks < last_update) ? ((ticks + (0xFFFFFFFF - last_update)) / 1000000.0f) : ((ticks - last_update) / 1000000.0f);
	last_update = ticks;

	struct PID *pid;
	for (uint8_t i = 0; i < 3; i++) {
		if (state.mode_autotune != AT_RUNNING)
			pid = &state.pid[i];
		else
			pid = &state_autotune_current_test->pid[i];

		pid->i_error_sum += angle_error[i] * delta_t;

		// Clamp integral to prevent wind-up
		pid->i_error_sum = clamp(pid->i_error_sum, -PID_MAX_ERROR_SUM, PID_MAX_ERROR_SUM);

		p_term = angle_error[i] * pid->p;
		i_term = pid->i_error_sum * pid->i;
		d_term = gyro[i] * pid->d;

		state.pid[i].value = p_term + i_term - d_term;
	}

	return 0;
}

/** Set the PID gain coefficients of a PID structure.
  *
  * @param *pid:	Pointer to the PID structure to set.
  * @param p:		The proportional (P) coefficient.
  * @param i:		The integral (I) coefficient.
  * @param d:		The derivative (D) coefficient.
  *
  * @return The status of the function.
***/
int8_t pid_set(struct PID *pid, float p, float i, float d) {
	pid->p = p;
	pid->i = i;
	pid->d = d;
	pid->i_error_sum = 0.0f;
	pid->value = 0.0f;

	return 0;
}

/** Reset the PID gain coefficients and integral portion of the 3-axis PID structure to zero.
  *
  * @return The status of the function.
***/
int8_t pid_reset() {
	for (uint8_t i = 0; i < 3; i++) {
		state.pid[i].i_error_sum = 0.0f;
		state.pid[i].value = 0.0f;
	}

	return 0;
}

/** Set the PID gain coefficients of a PID structure to zero.
  *
  * @param *pid:	Pointer to the PID structure.
  *
  * @return Void.
***/
void pid_zero(struct PID *pid) {
	pid_set(pid, 0.0f, 0.0f, 0.0f);
}
