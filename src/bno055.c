#include "bno055.h"

/** The interrupt handler receives the remaining I2C data at the end of
  * the DMA transfer and resets the DMA.
  *
  * @return Void.
***/
void DMA1_Stream2_IRQHandler(void) {
	if (DMA_GetITStatus(DMA1_Stream2, DMA_IT_TCIF2) == SET) {
		DMA_ClearITPendingBit(DMA1_Stream2, DMA_IT_TCIF2);

		I2C_DMACmd(BNO055_I2C, DISABLE);

		uint32_t timedout_start;
		timed_out_init(&timedout_start);
		while (!I2C_GetFlagStatus(BNO055_I2C, I2C_FLAG_BTF))	//////////// THIS MAY NOT BE NEEDED AND CAUSE A HANG! TEST!
			if (timed_out(timedout_start, 100)) {
				bno055_error_reset(-1);
				return;
			}
		I2C_AcknowledgeConfig(BNO055_I2C, DISABLE);
		bno055_dma_buffer[BNO055_BUFFER_SIZE - 3] = I2C_ReceiveData(BNO055_I2C);
		I2C_GenerateSTOP(BNO055_I2C, ENABLE);
		bno055_dma_buffer[BNO055_BUFFER_SIZE - 2] = I2C_ReceiveData(BNO055_I2C);
		timed_out_init(&timedout_start);
		while (!I2C_CheckEvent(BNO055_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED));
		if (timed_out(timedout_start, 100)) {
			bno055_error_reset(-1);
			return;
		}
		bno055_dma_buffer[BNO055_BUFFER_SIZE - 1] = I2C_ReceiveData(BNO055_I2C);
		timed_out_init(&timedout_start);
		while (I2C_GetFlagStatus(BNO055_I2C, I2C_FLAG_STOPF));
		if (timed_out(timedout_start, 100)) {
			bno055_error_reset(-1);
			return;
		}

		state.imu_failsafe_count = 0;
		bno055_dma_running = false;
		bno055_buffer_ready = true;
	}
}

/** Enable clocks, configure GPIO pins B10 & B11 as Alternate Function (I2C2),
  * configure DMA1_Stream2_Channel7, and enable transfer complete interrupt.
  *
  * @return The status of configuration.
***/
uint16_t bno055_config() {
	GPIO_InitTypeDef GPIO_InitStruct;
	I2C_InitTypeDef  I2C_InitStruct;
	DMA_InitTypeDef	 DMA_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

	i2c_helper_unstick(GPIOB, GPIO_Pin_10, GPIO_Pin_11);

	RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C2, ENABLE);
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C2, DISABLE);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_I2C2);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	I2C_DeInit(BNO055_I2C);
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStruct.I2C_ClockSpeed = 400000;
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStruct.I2C_OwnAddress1 = BNO055_ADDR;
	I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_Init(BNO055_I2C, &I2C_InitStruct);

	I2C_Cmd(BNO055_I2C, ENABLE);

	NVIC_InitStruct.NVIC_IRQChannel = DMA1_Stream2_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);

	DMA_DeInit(DMA1_Stream2);
	DMA_InitStruct.DMA_Channel = DMA_Channel_7;
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&BNO055_I2C->DR;
	DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)bno055_dma_buffer;
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStruct.DMA_BufferSize =  BNO055_BUFFER_SIZE - 3;
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_Init(DMA1_Stream2, &DMA_InitStruct);
	DMA_ITConfig(DMA1_Stream2, DMA_IT_TC, ENABLE);

	return 0;
}

/** Initialize variables and prepare to begin normal operation.
  *
  * @param skip_phase1:		If true, a portion of the initialization code is skipped
  *  				(this option is used in special failsafe conditions when an
  *  				in-flight restart is necessary).
  *
  * @return The status of initialization.
***/
uint16_t bno055_init(bool skip_phase1) {
	uint8_t value = 0;
	uint32_t timedout_start;

	if (!skip_phase1) {
		timed_out_init(&timedout_start);
		while (value != BNO055_ID) {
			if (i2c_helper_read_byte(BNO055_I2C, BNO055_ADDR, BNO055_CHIP_ID, &value))
				return STATE_ERROR_PERIPH_BNO055;

			if (timed_out(timedout_start, 500))
				return STATE_ERROR_PERIPH_BNO055;
		}

		if (i2c_helper_write_byte(BNO055_I2C, BNO055_ADDR, BNO055_OPR_MODE, BNO055_OPERATION_MODE_CONFIG))
			return STATE_ERROR_PERIPH_BNO055;

		delay(50);

		if (i2c_helper_write_byte(BNO055_I2C, BNO055_ADDR, BNO055_PAGE_ID, 0x00))
			return STATE_ERROR_PERIPH_BNO055;
		if (i2c_helper_write_byte(BNO055_I2C, BNO055_ADDR, BNO055_SYS_TRIGGER, BNO055_TRIGGER_RST_SYS))
			return STATE_ERROR_PERIPH_BNO055;

		delay(1000);

		value = 0;
		timed_out_init(&timedout_start);
		while (value != BNO055_ID) {
			if (i2c_helper_read_byte(BNO055_I2C, BNO055_ADDR, BNO055_CHIP_ID, &value))
				return STATE_ERROR_PERIPH_BNO055;

			if (timed_out(timedout_start, 1000))
				return STATE_ERROR_PERIPH_BNO055;
		}

		if (i2c_helper_write_byte(BNO055_I2C, BNO055_ADDR, BNO055_PAGE_ID, 0x00))
			return STATE_ERROR_PERIPH_BNO055;
		if (i2c_helper_write_byte(BNO055_I2C, BNO055_ADDR, BNO055_PWR_MODE, BNO055_POWER_MODE_NORMAL))
			return STATE_ERROR_PERIPH_BNO055;

		delay(50);

		if (i2c_helper_read_byte(BNO055_I2C, BNO055_ADDR, BNO055_ST_RESULT, &value))
			return STATE_ERROR_PERIPH_BNO055;
		if (value != 0x0F)
			return STATE_ERROR_PERIPH_BNO055;
		if (i2c_helper_write_byte(BNO055_I2C, BNO055_ADDR, BNO055_SYS_TRIGGER, BNO055_TRIGGER_SELFTEST))
			return STATE_ERROR_PERIPH_BNO055;

		delay(1000);

		if (i2c_helper_read_byte(BNO055_I2C, BNO055_ADDR, BNO055_ST_RESULT, &value))
			return STATE_ERROR_PERIPH_BNO055;
		if (value != 0x0F)
			return STATE_ERROR_PERIPH_BNO055;
	}

	if (i2c_helper_write_byte(BNO055_I2C, BNO055_ADDR, BNO055_UNIT_SEL, BNO055_UNIT_ACCEL_MS2 | BNO055_UNIT_GYRO_DPS | BNO055_UNIT_EULER_DEG | BNO055_UNIT_TEMP_F))
		return STATE_ERROR_PERIPH_BNO055;

	uint8_t map = 0, sign = 0;
	bno055_axis_remap(BNO055_AXIS_X, BNO055_AXIS_Y, BNO055_AXIS_Z, &map, &sign);

	if (i2c_helper_write_byte(BNO055_I2C, BNO055_ADDR, BNO055_AXIS_MAP_CONFIG, map))
		return STATE_ERROR_PERIPH_BNO055;
	if (i2c_helper_write_byte(BNO055_I2C, BNO055_ADDR, BNO055_AXIS_MAP_SIGN, sign))
		return STATE_ERROR_PERIPH_BNO055;

	if (i2c_helper_write_byte(BNO055_I2C, BNO055_ADDR, BNO055_SYS_TRIGGER, BNO055_TRIGGER_CLK_SEL_EXT))
		return STATE_ERROR_PERIPH_BNO055;

	if (i2c_helper_write_byte(BNO055_I2C, BNO055_ADDR, BNO055_PAGE_ID, 0x01))
		return STATE_ERROR_PERIPH_BNO055;
	if (i2c_helper_write_byte(BNO055_I2C, BNO055_ADDR, BNO055_ACC_CONFIG, BNO055_ACC_OPMODE_NORMAL | BNO055_ACC_BW_62_5 | BNO055_ACC_RANGE_4G))
		return STATE_ERROR_PERIPH_BNO055;

	if (i2c_helper_write_byte(BNO055_I2C, BNO055_ADDR, BNO055_PAGE_ID, 0x00))
		return STATE_ERROR_PERIPH_BNO055;
	if (i2c_helper_write_byte(BNO055_I2C, BNO055_ADDR, BNO055_OPR_MODE, BNO055_OPERATION_MODE_NDOF))
		return STATE_ERROR_PERIPH_BNO055;

	delay(100);

	bno055_buffer_ready = false;
	bno055_dma_running = false;
	bno055_status = 0;
	state.imu_calibration_percent = 0;
	bno055_error_level = 0;

	if (!skip_phase1)
		bno055_last_state = 0;

	return 0;
}

/** Update loop to handle state switching, conversion from raw to cooked data,
  * and error detection.
  *
  * @return The status of current update iteration.
***/
int8_t bno055_update() {
	static uint32_t last_update = 0;

	if (bno055_buffer_ready == true) {
		bno055_parse_buffer();
		bno055_cook_values();

		return 1;
	}
	else if (clocks_check_sample_rate(&last_update, BNO055_SAMPLE_RATE)) {
		if (!bno055_dma_running) {
			if (!bno055_get_status(&bno055_status)) {
				if (bno055_status == BNO055_STATUS_FUSION_ON) {
					if (state.status == STATE_SYS_CALIBRATING) {
						bno055_get_calibration_data(&state.imu_calibration_percent);
						if (state.imu_calibration_percent >= 100)
							state_set_calibrated(STATE_PERIPH_BNO055);
					}

					int16_t error = bno055_start_dma();
					bno055_error_level = ((error == 0) ? 0 : bno055_error_level + error);

					if (bno055_error_level < 0)
						bno055_error_reset(bno055_error_level);
				}
			}
			else
				state.imu_failsafe_count++;
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
void bno055_change_state(uint8_t new_state) {
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

/** Begin the I2C read sequence and setup DMA to take over the rest.
  *
  * @return The status of the function.
***/
int8_t bno055_start_dma() {
	if (bno055_dma_running)
		return -1;

	bno055_dma_running = true;

	uint32_t timedout_start;
	timed_out_init(&timedout_start);
	while (I2C_GetFlagStatus(BNO055_I2C, I2C_FLAG_BUSY))
		if (timed_out(timedout_start, 100))
			return -2;
	I2C_AcknowledgeConfig(BNO055_I2C, ENABLE);
	I2C_NACKPositionConfig(BNO055_I2C, I2C_NACKPosition_Current);
	I2C_GenerateSTART(BNO055_I2C, ENABLE);
	timed_out_init(&timedout_start);
	while (!I2C_CheckEvent(BNO055_I2C, I2C_EVENT_MASTER_MODE_SELECT))
		if (timed_out(timedout_start, 100))
			return -2;
	I2C_Send7bitAddress(BNO055_I2C, (BNO055_ADDR << 1), I2C_Direction_Transmitter);
	I2C_AcknowledgeConfig(BNO055_I2C, ENABLE);

	// Send the address to start reading from
	timed_out_init(&timedout_start);
	while (!I2C_CheckEvent(BNO055_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
		if (timed_out(timedout_start, 100))
			return -2;
	I2C_SendData(BNO055_I2C, BNO055_ACCEL_DATA_X_LSB);
	timed_out_init(&timedout_start);
	while (!I2C_CheckEvent(BNO055_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		if (timed_out(timedout_start, 100))
			return -2;

	// Start the read
	I2C_GenerateSTART(BNO055_I2C, ENABLE);
	timed_out_init(&timedout_start);
	while(!I2C_CheckEvent(BNO055_I2C, I2C_EVENT_MASTER_MODE_SELECT))
		if (timed_out(timedout_start, 100))
			return -2;
	I2C_Send7bitAddress(BNO055_I2C, (BNO055_ADDR << 1), I2C_Direction_Receiver);
	timed_out_init(&timedout_start);
	while(!I2C_CheckEvent(BNO055_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
		if (timed_out(timedout_start, 100))
			return -2;

	// Enable DMA
	DMA_SetCurrDataCounter(DMA1_Stream2, BNO055_BUFFER_SIZE - 3);
	I2C_DMACmd(BNO055_I2C, ENABLE);
	DMA_Cmd(DMA1_Stream2, ENABLE);

	return 0;
}

/** Split up the raw data read by the peripheral into corresponding raw value variables.
  *
  * @return The status of the function.
***/
int8_t bno055_parse_buffer() {
	bno055_raw_values.gyr_x = ((uint16_t)(bno055_dma_buffer[BNO055_GYRO_DATA_X_MSB - BNO055_BUFFER_OFFSET]) << 8)
					| bno055_dma_buffer[BNO055_GYRO_DATA_X_LSB - BNO055_BUFFER_OFFSET];
	bno055_raw_values.gyr_y = ((uint16_t)(bno055_dma_buffer[BNO055_GYRO_DATA_Y_MSB - BNO055_BUFFER_OFFSET]) << 8)
					| bno055_dma_buffer[BNO055_GYRO_DATA_Y_LSB - BNO055_BUFFER_OFFSET];
	bno055_raw_values.gyr_z = ((uint16_t)(bno055_dma_buffer[BNO055_GYRO_DATA_Z_MSB - BNO055_BUFFER_OFFSET]) << 8)
					| bno055_dma_buffer[BNO055_GYRO_DATA_Z_LSB - BNO055_BUFFER_OFFSET];

	bno055_raw_values.acc_x = ((uint16_t)(bno055_dma_buffer[BNO055_ACCEL_DATA_X_MSB - BNO055_BUFFER_OFFSET]) << 8)
					| bno055_dma_buffer[BNO055_ACCEL_DATA_X_LSB - BNO055_BUFFER_OFFSET];
	bno055_raw_values.acc_y = ((uint16_t)(bno055_dma_buffer[BNO055_ACCEL_DATA_Y_MSB - BNO055_BUFFER_OFFSET]) << 8)
					| bno055_dma_buffer[BNO055_ACCEL_DATA_Y_LSB - BNO055_BUFFER_OFFSET];
	bno055_raw_values.acc_z = ((uint16_t)(bno055_dma_buffer[BNO055_ACCEL_DATA_Z_MSB - BNO055_BUFFER_OFFSET]) << 8)
					| bno055_dma_buffer[BNO055_ACCEL_DATA_Z_LSB - BNO055_BUFFER_OFFSET];

	bno055_raw_values.mag_x = ((uint16_t)(bno055_dma_buffer[BNO055_MAG_DATA_X_MSB - BNO055_BUFFER_OFFSET]) << 8)
					| bno055_dma_buffer[BNO055_MAG_DATA_X_LSB - BNO055_BUFFER_OFFSET];
	bno055_raw_values.mag_y = ((uint16_t)(bno055_dma_buffer[BNO055_MAG_DATA_Y_MSB - BNO055_BUFFER_OFFSET]) << 8)
					| bno055_dma_buffer[BNO055_MAG_DATA_Y_LSB - BNO055_BUFFER_OFFSET];
	bno055_raw_values.mag_z = ((uint16_t)(bno055_dma_buffer[BNO055_MAG_DATA_Z_MSB - BNO055_BUFFER_OFFSET]) << 8)
					| bno055_dma_buffer[BNO055_MAG_DATA_Z_LSB - BNO055_BUFFER_OFFSET];

	bno055_raw_values.eul_h = ((uint16_t)(bno055_dma_buffer[BNO055_EULER_H_MSB - BNO055_BUFFER_OFFSET]) << 8)
					| bno055_dma_buffer[BNO055_EULER_H_LSB - BNO055_BUFFER_OFFSET];
	bno055_raw_values.eul_p = ((uint16_t)(bno055_dma_buffer[BNO055_EULER_P_MSB - BNO055_BUFFER_OFFSET]) << 8)
					| bno055_dma_buffer[BNO055_EULER_P_LSB - BNO055_BUFFER_OFFSET];
	bno055_raw_values.eul_r = ((uint16_t)(bno055_dma_buffer[BNO055_EULER_R_MSB - BNO055_BUFFER_OFFSET]) << 8)
					| bno055_dma_buffer[BNO055_EULER_R_LSB - BNO055_BUFFER_OFFSET];

	bno055_raw_values.quat_w = ((uint16_t)(bno055_dma_buffer[BNO055_QUATERNION_DATA_W_MSB - BNO055_BUFFER_OFFSET]) << 8)
					| bno055_dma_buffer[BNO055_QUATERNION_DATA_W_LSB - BNO055_BUFFER_OFFSET];
	bno055_raw_values.quat_x = ((uint16_t)(bno055_dma_buffer[BNO055_QUATERNION_DATA_X_MSB - BNO055_BUFFER_OFFSET]) << 8)
					| bno055_dma_buffer[BNO055_QUATERNION_DATA_X_LSB - BNO055_BUFFER_OFFSET];
	bno055_raw_values.quat_y = ((uint16_t)(bno055_dma_buffer[BNO055_QUATERNION_DATA_Y_MSB - BNO055_BUFFER_OFFSET]) << 8)
					| bno055_dma_buffer[BNO055_QUATERNION_DATA_Y_LSB - BNO055_BUFFER_OFFSET];
	bno055_raw_values.quat_z = ((uint16_t)(bno055_dma_buffer[BNO055_QUATERNION_DATA_Z_MSB - BNO055_BUFFER_OFFSET]) << 8)
					| bno055_dma_buffer[BNO055_QUATERNION_DATA_Z_LSB - BNO055_BUFFER_OFFSET];

	bno055_raw_values.linear_acc_x = ((uint16_t)(bno055_dma_buffer[BNO055_LINEAR_ACCEL_DATA_X_MSB - BNO055_BUFFER_OFFSET]) << 8)
					| bno055_dma_buffer[BNO055_LINEAR_ACCEL_DATA_X_LSB - BNO055_BUFFER_OFFSET];
	bno055_raw_values.linear_acc_y = ((uint16_t)(bno055_dma_buffer[BNO055_LINEAR_ACCEL_DATA_Y_MSB - BNO055_BUFFER_OFFSET]) << 8)
					| bno055_dma_buffer[BNO055_LINEAR_ACCEL_DATA_Y_LSB - BNO055_BUFFER_OFFSET];
	bno055_raw_values.linear_acc_z = ((uint16_t)(bno055_dma_buffer[BNO055_LINEAR_ACCEL_DATA_Z_MSB - BNO055_BUFFER_OFFSET]) << 8)
					| bno055_dma_buffer[BNO055_LINEAR_ACCEL_DATA_Z_LSB - BNO055_BUFFER_OFFSET];

	bno055_raw_values.gravity_x = ((uint16_t)(bno055_dma_buffer[BNO055_GRAVITY_DATA_X_MSB - BNO055_BUFFER_OFFSET]) << 8)
					| bno055_dma_buffer[BNO055_GRAVITY_DATA_X_LSB - BNO055_BUFFER_OFFSET];
	bno055_raw_values.gravity_y = ((uint16_t)(bno055_dma_buffer[BNO055_GRAVITY_DATA_Y_MSB - BNO055_BUFFER_OFFSET]) << 8)
					| bno055_dma_buffer[BNO055_GRAVITY_DATA_Y_LSB - BNO055_BUFFER_OFFSET];
	bno055_raw_values.gravity_z = ((uint16_t)(bno055_dma_buffer[BNO055_GRAVITY_DATA_Z_MSB - BNO055_BUFFER_OFFSET]) << 8)
					| bno055_dma_buffer[BNO055_GRAVITY_DATA_Z_LSB - BNO055_BUFFER_OFFSET];

	bno055_raw_values.temperature = bno055_dma_buffer[BNO055_TEMP - BNO055_BUFFER_OFFSET];

	bno055_buffer_ready = false;

	return 0;
}

/** Compute the final cooked values from the parsed data.
  *
  * @return The status of the function.
***/
int8_t bno055_cook_values() {
	state.gyr.x = (float)bno055_raw_values.gyr_x / 16.0f;
	state.gyr.y = (float)bno055_raw_values.gyr_y / 16.0f;
	state.gyr.z = (float)bno055_raw_values.gyr_z / 16.0f;

	state.acc.x = (float)bno055_raw_values.acc_x / 100.0f;
	state.acc.y = (float)bno055_raw_values.acc_y / 100.0f;
	state.acc.z = (float)bno055_raw_values.acc_z / 100.0f;

	state.mag.x = (float)bno055_raw_values.mag_x / 16.0f;
	state.mag.y = (float)bno055_raw_values.mag_y / 16.0f;
	state.mag.z = (float)bno055_raw_values.mag_z / 16.0f;

	state.heading = (float)bno055_raw_values.eul_h / 16.0f;
	state.pitch = (float)bno055_raw_values.eul_p / 16.0f;
	state.roll = -(float)bno055_raw_values.eul_r / 16.0f;

	const double scale_factor = (1.0 / (1 << 14));
	state.q.w = (float)bno055_raw_values.quat_w * scale_factor;
	state.q.x = (float)bno055_raw_values.quat_x * scale_factor;
	state.q.y = (float)bno055_raw_values.quat_y * scale_factor;
	state.q.z = (float)bno055_raw_values.quat_z * scale_factor;

	state.linear_acc.x = (float)bno055_raw_values.linear_acc_x / 100.0f;
	state.linear_acc.y = (float)bno055_raw_values.linear_acc_y / 100.0f;
	state.linear_acc.z = (float)bno055_raw_values.linear_acc_z / 100.0f;

	state.gravity.x = (float)bno055_raw_values.gravity_x / 100.0f;
	state.gravity.y = (float)bno055_raw_values.gravity_y / 100.0f;
	state.gravity.z = (float)bno055_raw_values.gravity_z / 100.0f;

	state.temperature_imu = bno055_raw_values.temperature * 2;

	state.linear_acc_notilt = vector_rotate(state.linear_acc, state.q);
	state.climb_acc = state.linear_acc_notilt.z;

	return 0;
}

/** Reads how well calibrated the sensors of the peripheral are (0 to 100%).
  * Each of 3 sensors is scored 0 to 3 plus an overall score of 0 to 3,
  * the total calibration is represented as 0 to 12.
  *
  * @param *calibration_percent:	Pointer to the variable holding calibration
  * 					status (in 0-100% form) to be updated.
  *
  * @return The status of the function.
***/
int8_t bno055_get_calibration_data(uint8_t *calibration_percent) {
	uint8_t calibration_sys, calibration_gyr, calibration_acc, calibration_mag;
	uint8_t value, sum;

	if (i2c_helper_read_byte(BNO055_I2C, BNO055_ADDR, BNO055_CALIB_STATUS, &value))
		return -1;

	calibration_sys = (value >> 6);
	calibration_gyr = ((value & 0x30) >> 4);
	calibration_acc = ((value & 0x0C) >> 2);
	calibration_mag = (value & 0x03);

	sum = calibration_sys + calibration_gyr + calibration_acc + calibration_mag;
	*calibration_percent = (sum * 100) / 12;

	return 0;
}

/** Read the system status register.
  *
  * @param *status:	Pointer to the variable that will hold the
  * 			current system status of the peripheral.
  *
  * @return The status of the function.
***/
int8_t bno055_get_status(uint8_t *status) {
	uint8_t value;

	if (i2c_helper_read_byte(BNO055_I2C, BNO055_ADDR, BNO055_SYS_STATUS, &value))
		return -1;

	switch (value) {
	case 0:
		*status = BNO055_STATUS_SYS_IDLE;
		break;
	case 1:
		if (i2c_helper_read_byte(BNO055_I2C, BNO055_ADDR, BNO055_SYS_ERR, &value))
			return -1;

		switch (value) {
		case 0:
			*status = BNO055_STATUS_ERROR_NOERROR;
			break;
		case 1:
			*status = BNO055_STATUS_ERROR_PERIPH_INIT;
			break;
		case 2:
			*status = BNO055_STATUS_ERROR_SYS_INIT;
			break;
		case 3:
			*status = BNO055_STATUS_ERROR_ST_FAIL;
			break;
		case 4:
			*status = BNO055_STATUS_ERROR_INVALID_VALUE;
			break;
		case 5:
			*status = BNO055_STATUS_ERROR_INVALID_ADDR;
			break;
		case 6:
			*status = BNO055_STATUS_ERROR_WRITE_FAIL;
			break;
		case 7:
			*status = BNO055_STATUS_ERROR_LOWPOWERNOTAVAIL;
			break;
		case 8:
			*status = BNO055_STATUS_ERROR_ACCPOWERNOTAVAIL;
			break;
		case 9:
			*status = BNO055_STATUS_ERROR_FUSIONCONFIG_FAIL;
			break;
		case 10:
			*status = BNO055_STATUS_ERROR_SENSORCONFIG_FAIL;
			break;
		default:
			return -3;
			break;
		}
		break;
	case 2:
		*status = BNO055_STATUS_PERIPH_INIT;
		break;
	case 3:
		*status = BNO055_STATUS_SYS_INIT;
		break;
	case 4:
		*status = BNO055_STATUS_EXEC_ST;
		break;
	case 5:
		*status = BNO055_STATUS_FUSION_ON;
		break;
	case 6:
		*status = BNO055_STATUS_FUSION_OFF;
		break;
	default:
		return -2;
		break;
	}

	return 0;
}

/** Set the orientation of the sensor by swapping the x, y, and z axes if necessary.
  *
  * @param remap_x:	Set the x-axis of the sensor to x-axis (0), y-axis (1), z-axis (2).
  * @param remap_y:	Set the y-axis of the sensor to x-axis (0), y-axis (1), z-axis (2).
  * @param remap_z:	Set the z-axis of the sensor to x-axis (0), y-axis (1), z-axis (2).
  * @param *map:	Pointer to the new axis remap value.
  * @param *sign:	Pointer to the value indicating if each of the axes should be negated to
  * 			point in the correct direction.
  *
  * @return The status of the function.
***/
int8_t bno055_axis_remap(int8_t remap_x, int8_t remap_y, int8_t remap_z, uint8_t *map, uint8_t *sign) {
	uint8_t x, y, z;

	*sign = 0;

	if (remap_x < 0) {
		*sign |= (1<<2);
		x = (uint8_t)(remap_x * -1);
	}
	else
		x = (uint8_t)remap_x;

	if (remap_y < 0) {
		*sign |= (1<<1);
		y = (uint8_t)(remap_y * -1);
	}
	else
		y = (uint8_t)remap_y;

	if (remap_z < 0) {
		*sign |= (1<<0);
		z = (uint8_t)(remap_z * -1);
	}
	else
		z = (uint8_t)remap_z;

	*map = x | (y << 2) | (z << 4);

	return 0;
}

/** Attempt to recover from a problem with the peripheral by resetting certain
  * things depending on how long the problem persists.
  *
  * @param error_level:		The more negative the value, the more severe the problem (error_level is decremented).
  *
  * @return Void.
***/
void bno055_error_reset(int16_t error_level) {
	if (error_level < 0) {
		I2C_DMACmd(BNO055_I2C, DISABLE);
		DMA_Cmd(DMA1_Stream2, DISABLE);
		I2C_Cmd(BNO055_I2C, DISABLE);
		bno055_dma_running = false;
		bno055_buffer_ready = false;
		delay(10);
		I2C_Cmd(BNO055_I2C, ENABLE);
		delay(10);
	}

	if (error_level < -4) {
		/////////////// SAVE CALIB OFFSETS AND RELOAD
		I2C_Cmd(BNO055_I2C, DISABLE);
		bno055_config();
		bno055_init(true);
	}
}
