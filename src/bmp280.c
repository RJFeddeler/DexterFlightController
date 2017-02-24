#include "bmp280.h"
// WORK IN PROGRESS!!!

/** Enable clocks and configure GPIO pins B8 & B9 as Alternate Function (I2C1).
  *
  * @return The status of configuration.
***/
uint16_t bmp280_config() {
	GPIO_InitTypeDef GPIO_InitStruct;
	I2C_InitTypeDef  I2C_InitStruct;

	// Enable Clocks
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

	// Make sure I2C bus is working
	i2c_helper_unstick(GPIOB, GPIO_Pin_8, GPIO_Pin_9);

	// Enable Clocks (again)
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE);

	// Set Pins as Alternate Function: I2C
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);

	// Configure and Enable Pins B8 & B9 as Alternate Function and Open Drain
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	// Configure and Enable I2C at 400 kHz
	I2C_DeInit(I2C1);
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStruct.I2C_ClockSpeed = 400000;
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStruct.I2C_OwnAddress1 = BMP280_ADDR;
	I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_Init(I2C1, &I2C_InitStruct);

	// Enable I2C
	I2C_Cmd(I2C1, ENABLE);

	return 0;
}

/** Initialize variables and prepare to begin normal operation.
  *
  * @return The status of initialization.
***/
uint16_t bmp280_init() {
	uint8_t value = 0;
	uint32_t timedout_start;

	timed_out_init(&timedout_start);
	while (value != BMP280_ID) {
		// Read ChipID to make sure we have the correct device before we reset it
		if (i2c_helper_read_byte(I2C1, BMP280_ADDR, BMP280_CHIP_ID, &value))
			return STATE_ERROR_PERIPH_BMP280;

		// If no valid ChipID is read within 500 mS then initialization failed
		if (timed_out(timedout_start, 500))
			return STATE_ERROR_PERIPH_BMP280;
	}

	// Reset the device
	if (i2c_helper_write_byte(I2C1, BMP280_ADDR, BMP280_RESET, BMP280_RESET_CMD))
		return STATE_ERROR_PERIPH_BMP280;

	delay(100);

	timed_out_init(&timedout_start);
	while (value != BMP280_ID) {
		// Read ChipID to make sure we have the correct device (again) to make sure it reset okay
		if (i2c_helper_read_byte(I2C1, BMP280_ADDR, BMP280_CHIP_ID, &value))
			return STATE_ERROR_PERIPH_BMP280;

		// If no valid ChipID is read within 1 S then initialization failed
		if (timed_out(timedout_start, 1000))
			return STATE_ERROR_PERIPH_BMP280;
	}

	// Read Calibration offsets from device
	if (bmp280_read_calibration_data())
		return STATE_ERROR_PERIPH_BMP280;

	// Configure device settings
	if (i2c_helper_write_byte(I2C1, BMP280_ADDR, BMP280_CONFIG, BMP280_CONFIG_INACTIVE_0_5 | BMP280_CONFIG_FILTER_16))
		return STATE_ERROR_PERIPH_BMP280;
	if (i2c_helper_write_byte(I2C1, BMP280_ADDR, BMP280_CTRL_MEAS, BMP280_CTRLMEAS_MODE_NORMAL | BMP280_CTRLMEAS_OVERSAMP_P_16 | BMP280_CTRLMEAS_OVERSAMP_T_2))
		return STATE_ERROR_PERIPH_BMP280;

	// Zero out initial values
	bmp280_start_pressure = 0;
	bmp280_start_temperature = 0;
	vector_zero(&bmp280_int_sum);
	bmp280_int_count = 0;
	bmp280_kalman_last_time = 0;
	bmp280_altitude = 0.0f;
	bmp280_alt_offset = 0.0f;
	bmp280_climb_vel = 0.0f;
	bmp280_climb_acc = 0.0f;
	bmp280_avg_p_sum = 0.0f;
	bmp280_avg_t_sum = 0.0f;
	bmp280_avg_a_sum = 0.0f;
	bmp280_last_state = 0;
	bmp280_counter = 0;

	return 0;
}

/** Update loop to handle state switching and conversion from raw to cooked data.
  *
  * @return The status of current update iteration.
***/
int8_t bmp280_update() {
	static uint32_t last_update = 0;
	float alt;

	if (clocks_check_sample_rate(&last_update, BMP280_SAMPLE_RATE)) {
		if (state.status != bmp280_last_state) {
			bmp280_change_state(state.status);
			bmp280_last_state = state.status;
		}

		bmp280_read_raw_data();

		bmp280_compensate_t(bmp280_ut, &bmp280_temperature);
		bmp280_compensate_p(bmp280_up, &bmp280_pressure);
		bmp280_calculate_alt(&alt);

		if (state.status == STATE_SYS_ARMING) {
			bmp280_avg_p_sum += bmp280_pressure;
			bmp280_avg_t_sum += bmp280_temperature;
			bmp280_avg_a_sum += alt;
		}

		if (state.status == STATE_SYS_ARMED) {
			if (bmp280_int_count > 0) {
				bmp280_int_sum.x /= (float)bmp280_int_count;
				bmp280_int_sum.y /= (float)bmp280_int_count;
				bmp280_int_sum.z /= (float)bmp280_int_count;

				//m = vector_get_magnitude(bmp280_int_sum);
				//if (m > 0.2f && m < 1.5f) {  // DEFAULT: 0.5 - 1.5
				if (bmp280_int_sum.z > 0.2f) {
					bmp280_kalman_update(alt, state.linear_acc_notilt.z * 100.0f, &state.alt_bp, &state.climb_velocity);
				}
			}
			else {
				/////////////// FIGURE OUT WHAT TO DO WHEN NOT USING ACCEL
			}

			vector_zero(&bmp280_int_sum);
			bmp280_int_count = 0;
		}

		if (state.status >= STATE_SYS_ARMING)
			bmp280_counter++;

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
void bmp280_change_state(uint8_t new_state) {
	switch (new_state) {
	case STATE_SYS_CALIBRATING:
		state_set_calibrated(STATE_PERIPH_BMP280);	// Add a check here

		break;
	case STATE_SYS_IDLE:
		break;
	case STATE_SYS_ARMING:
		bmp280_counter = 0;
		bmp280_altitude = 0.0f;
		bmp280_alt_offset = 0.0f;
		bmp280_climb_vel = 0.0f;
		bmp280_climb_acc = 0.0f;

		break;
	case STATE_SYS_ARMED:
		vector_zero(&bmp280_int_sum);
		bmp280_int_count = 0;

		home.pressure = bmp280_avg_p_sum / (float)bmp280_counter;
		home.temperature = bmp280_avg_t_sum / (float)bmp280_counter;
		home.altitude = 0.0f;
		bmp280_alt_offset = bmp280_avg_a_sum / (float)bmp280_counter;

		bmp280_kalman_config(500.0f, 1.0f, 1.0f, (bmp280_avg_a_sum / (float)bmp280_counter));
		bmp280_kalman_last_time = 0;

		break;
	}
}

/** Compensate raw pressure reading with manufacturer offsets for specific sensor to increase accuracy.
  *
  * @param raw:		Raw pressure value read by sensor.
  * @param *value:	Address to store the compensated pressure value.
  *
  * @return The status of the compensation function.
***/
int8_t bmp280_compensate_p(int32_t raw, float *value) {
	float var1, var2, p;

	var1 = ((float)bmp280_tfine / 2.0f) - 64000.0f;
	var2 = var1 * var1 * ((float)bmp280_dig_p6) / 32768.0f;
	var2 = var2 + var1 * ((float)bmp280_dig_p5) * 2.0f;
	var2 = (var2 / 4.0f) + (((float)bmp280_dig_p4) * 65536.0f);
	var1 = (((float)bmp280_dig_p3) * var1 * var1 / 524288.0f + ((float)bmp280_dig_p2) * var1) / 524288.0f;
	var1 = (1.0f + var1 / 32768.0f) * ((float)bmp280_dig_p1);

	if (var1 == 0.0f)
		return -1;

	p = 1048576.0f - (float)raw;
	p = (p - (var2 / 4096.0f)) * 6250.0f / var1;
	var1 = ((float)bmp280_dig_p9) * p * p / 2147483648.0f;
	var2 = p * ((float)bmp280_dig_p8) / 32768.0f;
	p = p + (var1 + var2 + ((float)bmp280_dig_p7)) / 16.0f;

	*value = p;

	return 0;
}

/** Compensate raw temperature reading with manufacturer offsets for specific sensor to increase accuracy.
  *
  * @param raw:		Raw temperature value read by sensor.
  * @param *value:	Address to store the compensated temperature value.
  *
  * @return The status of the compensation function.
***/
int8_t bmp280_compensate_t(int32_t raw, float *value) {
	float var1, var2;

	var1 = (((float)raw) / 16384.0f - ((float)bmp280_dig_t1) / 1024.0f) * ((float)bmp280_dig_t2);
	var2 = ((((float)raw) / 131072.0f - ((float)bmp280_dig_t1) / 8192.0f) * (((float)raw) / 131072.0f -
		((float) bmp280_dig_t1) / 8192.0f)) * ((float)bmp280_dig_t3);

	bmp280_tfine = (int32_t)(var1 + var2);
	*value = (float)((var1 + var2) / 5120.0f);

	return 0;
}

/** Calculate the current altitude (in meters) based on the change in pressure.
  *
  * @param *value:	Address to store calculated altitude.
  *
  * @return The status of the calculation function.
***/
int8_t bmp280_calculate_alt(float *value) {
	*value = ft_to_m(98.57f * (bmp280_start_temperature + 273.15f) * (float)(log(bmp280_start_pressure / bmp280_pressure)));

	return 0;
}

/** Integrate accelerometer values for use in the Kalman filter.
  *
  * @param v:	Vector containing accelerometer readings in 3 dimensions.
  *
  * @return Void.
***/
void bmp280_integrate_accz(vector v) {
	bmp280_int_sum.x += v.x;
	bmp280_int_sum.y += v.y;
	bmp280_int_sum.z += v.z;

	bmp280_int_count++;
}

/** Configure the Kalman filter
  *
  * @param variance_z:
  * @param variance_zaccel:
  * @param variance_zaccelbias:
  * @param abias_initial:
  *
  * @return Void.
***/
void bmp280_kalman_config(float variance_z, float variance_zaccel, float variance_zaccelbias, float abias_initial) {
	bmp280_kalman.z = 0.0f;
	bmp280_kalman.v = 0.0f;
	bmp280_kalman.abias = abias_initial;

	bmp280_kalman.var_zaccel = variance_zaccel;
	bmp280_kalman.var_zaccelbias = variance_zaccelbias;
	bmp280_kalman.var_z = variance_z;

	bmp280_kalman.covar_zz = 1.0f;
	bmp280_kalman.covar_zv = 0.0f;
	bmp280_kalman.covar_za = 0.0f;

	bmp280_kalman.covar_vz = 0.0f;
	bmp280_kalman.covar_vv = 1.0f;
	bmp280_kalman.covar_va = 0.0f;

	bmp280_kalman.covar_az = 0.0f;
	bmp280_kalman.covar_av = 0.0f;
	bmp280_kalman.covar_aa = 100000.0f;
}

/** Use a Kalman filter to fuse pressure and accelerometer data to greatly increase
  * altitude estimation accuracy by reducing the effect of noise and drift.
  *
  * @param z:			Estimated altitude based on pressure change.
  * @param a:			Accelerometer reading perpendicular to Earth's surface.
  * @param *altitude:	Address to store filtered altitude.
  * @param *velocity:	Address to store calculated velocity perpendicular to Earth's surface.
  *
  * @return Void.
***/
void bmp280_kalman_update(float z, float a, float *altitude, float *velocity) {
	uint32_t ticks = clocks_get_ticks();
	uint32_t delta = (ticks < bmp280_kalman_last_time) ? (ticks + (0xFFFFFFFF - bmp280_kalman_last_time)) : (ticks - bmp280_kalman_last_time);
	bmp280_kalman_last_time = ticks;

	if (delta > 1000000)
		delta = 0;

	float dt = delta / 1000000.0f;

	bmp280_kalman.v += (a - bmp280_kalman.abias) * dt;
	bmp280_kalman.z += bmp280_kalman.v * dt;

	bmp280_kalman.var_zaccel = clampf((fabs((a - bmp280_kalman.abias)) / 50.0f), 1.0f, 50.0f);

	float t00,t01,t02;
	float t10,t11,t12;
	float t20,t21,t22;

	float dt2div2 = dt * dt / 2.0f;
	float dt3div2 = dt2div2 * dt;
	float dt4div4 = dt2div2 * dt2div2;

	t00 = bmp280_kalman.covar_zz + dt * bmp280_kalman.covar_vz - dt2div2 * bmp280_kalman.covar_az;
	t01 = bmp280_kalman.covar_zv + dt * bmp280_kalman.covar_vv - dt2div2 * bmp280_kalman.covar_av;
	t02 = bmp280_kalman.covar_za + dt * bmp280_kalman.covar_va - dt2div2 * bmp280_kalman.covar_aa;

	t10 = bmp280_kalman.covar_vz - dt * bmp280_kalman.covar_az;
	t11 = bmp280_kalman.covar_vv - dt * bmp280_kalman.covar_av;
	t12 = bmp280_kalman.covar_va - dt * bmp280_kalman.covar_aa;

	t20 = bmp280_kalman.covar_az;
	t21 = bmp280_kalman.covar_av;
	t22 = bmp280_kalman.covar_aa;

	bmp280_kalman.covar_zz = t00 + dt * t01 - dt2div2 * t02;
	bmp280_kalman.covar_zv = t01 - dt * t02;
	bmp280_kalman.covar_za = t02;

	bmp280_kalman.covar_vz = t10 + dt * t11 - dt2div2 * t12;
	bmp280_kalman.covar_vv = t11 - dt * t12;
	bmp280_kalman.covar_va = t12;

	bmp280_kalman.covar_az = t20 + dt * t21 - dt2div2 * t22;
	bmp280_kalman.covar_av = t21 - dt * t22;
	bmp280_kalman.covar_aa = t22;

	bmp280_kalman.covar_zz += dt4div4 * bmp280_kalman.var_zaccel;
	bmp280_kalman.covar_zv += dt3div2 * bmp280_kalman.var_zaccel;

	bmp280_kalman.covar_vz += dt3div2 * bmp280_kalman.var_zaccel;
	bmp280_kalman.covar_vv += dt * dt * bmp280_kalman.var_zaccel;

	bmp280_kalman.covar_aa += bmp280_kalman.var_zaccelbias;

	float innov = z - bmp280_kalman.z;
	float sInv = 1.0f / (bmp280_kalman.covar_zz + bmp280_kalman.var_z);

	float kz = bmp280_kalman.covar_zz * sInv;
	float kv = bmp280_kalman.covar_vz * sInv;
	float ka = bmp280_kalman.covar_az * sInv;

	bmp280_kalman.z += kz * innov;
	bmp280_kalman.v += kv * innov;
	bmp280_kalman.abias += ka * innov;

	*altitude = bmp280_kalman.z;
	*velocity = bmp280_kalman.v;

	bmp280_kalman.covar_zz -= kz * bmp280_kalman.covar_zz;
	bmp280_kalman.covar_zv -= kz * bmp280_kalman.covar_zv;
	bmp280_kalman.covar_za -= kz * bmp280_kalman.covar_za;

	bmp280_kalman.covar_vz -= kv * bmp280_kalman.covar_zz;
	bmp280_kalman.covar_vv -= kv * bmp280_kalman.covar_zv;
	bmp280_kalman.covar_va -= kv * bmp280_kalman.covar_za;

	bmp280_kalman.covar_az -= ka * bmp280_kalman.covar_zz;
	bmp280_kalman.covar_av -= ka * bmp280_kalman.covar_zv;
	bmp280_kalman.covar_aa -= ka * bmp280_kalman.covar_za;
}

/** Get the raw data values from the sensor for temperature and pressure.
  *
  * @return The status of the reading function.
***/
int8_t bmp280_read_raw_data() {
	uint8_t buffer[BMP280_DATA_LENGTH];

	if (i2c_helper_read_bytes(I2C1, BMP280_ADDR, BMP280_DATA_START, BMP280_DATA_LENGTH, buffer))
		return -1;

	bmp280_up = (int32_t)(((uint32_t)buffer[0] << 12) | ((uint16_t)buffer[1] << 4) | ((buffer[2] >> 4)));
	bmp280_ut = (int32_t)(((uint32_t)buffer[3] << 12) | ((uint16_t)buffer[4] << 4) | ((buffer[5] >> 4)));

	return 0;
}

/** Read the sensor specific offsets set by the manufacturer to be used when compensating raw values.
  *
  * @return The status of the reading function.
***/
int8_t bmp280_read_calibration_data() {
	uint8_t buffer[BMP280_COMPENSATION_LENGTH];

	if (i2c_helper_read_bytes(I2C1, BMP280_ADDR, BMP280_COMPENSATION_START, BMP280_COMPENSATION_LENGTH, buffer))
		return -1;

	bmp280_dig_t1 = (uint16_t)((uint16_t)buffer[1] << 8 | buffer[0]);
	bmp280_dig_t2 = (int16_t)((uint16_t)buffer[3] << 8 | buffer[2]);
	bmp280_dig_t3 = (int16_t)((uint16_t)buffer[5] << 8 | buffer[4]);

	bmp280_dig_p1 = (uint16_t)((uint16_t)buffer[7] << 8 | buffer[6]);
	bmp280_dig_p2 = (int16_t)((uint16_t)buffer[9] << 8 | buffer[8]);
	bmp280_dig_p3 = (int16_t)((uint16_t)buffer[11] << 8 | buffer[10]);
	bmp280_dig_p4 = (int16_t)((uint16_t)buffer[13] << 8 | buffer[12]);
	bmp280_dig_p5 = (int16_t)((uint16_t)buffer[15] << 8 | buffer[14]);
	bmp280_dig_p6 = (int16_t)((uint16_t)buffer[17] << 8 | buffer[16]);
	bmp280_dig_p7 = (int16_t)((uint16_t)buffer[19] << 8 | buffer[18]);
	bmp280_dig_p8 = (int16_t)((uint16_t)buffer[21] << 8 | buffer[20]);
	bmp280_dig_p9 = (int16_t)((uint16_t)buffer[23] << 8 | buffer[22]);

	return 0;
}

/** Get the current pressure value.
  *
  * @return The current pressure reading.
***/
float bmp280_get_pressure() { return bmp280_pressure; }

/** Get the current temperature value.
  *
  * @param unit:	The temperature scale desired.
  *
  * @return The current temperature reading in the desired scale units.
***/
float bmp280_get_temperature(uint8_t unit) {
	if (unit == BMP280_UNIT_F)
		return c_to_f(bmp280_temperature);
	else
		return bmp280_temperature;
}

/** Get the current altitude value.
  *
  * @param unit:	The distance scale desired.
  *
  * @return The current altitude in the desired scale units.
***/
float bmp280_get_altitude(uint8_t unit) {
	if (unit == BMP280_UNIT_FT)
		return m_to_ft(bmp280_altitude);
	else
		return bmp280_altitude;
}

/** Set the value of the pressure on the ground to use in altitude estimation.
  *
  * @param value:	The value to set the ground pressure at.
  *
  * @return Void.
***/
void  bmp280_set_start_pressure(float value) { bmp280_start_pressure = value; }

/** Set the value of the temperature on the ground to use in altitude estimation.
  *
  * @param value:	The value to set the ground temperature at.
  * @param unit:	The scale units being given by value.
  *
  * @return Void.
***/
void  bmp280_set_start_temperature(float value, uint8_t unit) {
	if (unit == BMP280_UNIT_F)
		bmp280_start_temperature = f_to_c(value);
	else
		bmp280_start_temperature = value;
}
