#include "i2c_helper.h"

/** Read 1 byte from an I2C peripheral.
  *
  * @param *I2Cx:	Pointer to the I2C peripheral being used (x is 1, 2, or 3).
  * @param device:	I2C peripheral device address.
  * @param reg:		I2C peripheral register address.
  * @param *data:	Pointer to the variable to store the data being read.
  *
  * @return The status of the function.
***/
int8_t i2c_helper_read_byte (I2C_TypeDef *I2Cx, uint8_t device, uint8_t reg, uint8_t  *data) {
	return i2c_helper_read_bytes(I2Cx, device, reg, 1, data);
}

/** Read a word (2 bytes) from an I2C peripheral.
  *
  * @param *I2Cx:	Pointer to the I2C peripheral being used (x is 1, 2, or 3).
  * @param device:	I2C peripheral device address.
  * @param reg:		I2C peripheral register address.
  * @param *data:	Pointer to the variable to store the data being read.
  *
  * @return The status of the function.
***/
int8_t i2c_helper_read_word (I2C_TypeDef *I2Cx, uint8_t device, uint8_t reg, uint16_t  *data) {
	uint8_t temp[2];
	int8_t ret;

	ret = i2c_helper_read_bytes(I2Cx, device, reg, 2, temp);

	if (!ret)
		*data = (((uint16_t)temp[0] << 8) | temp[1]);

	return ret;
}

/** Read a given number of bytes from an I2C peripheral.
  *
  * @param *I2Cx:	Pointer to the I2C peripheral being used (x is 1, 2, or 3).
  * @param device:	I2C peripheral device address.
  * @param reg:		I2C peripheral register address.
  * @param length:	Number of bytes to read.
  * @param *data:	Pointer to the variable to store the data being read.
  *
  * @return The status of the function.
***/
int8_t i2c_helper_read_bytes (I2C_TypeDef *I2Cx, uint8_t device, uint8_t reg, uint8_t length, uint8_t  *data) {
	uint32_t timedout_start;
	timed_out_init(&timedout_start);
	while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY))
		if (timed_out(timedout_start, 100))
			return -1;
	I2C_AcknowledgeConfig(I2Cx, ENABLE);
	I2C_NACKPositionConfig(I2Cx, I2C_NACKPosition_Current);

	I2C_GenerateSTART(I2Cx, ENABLE);
	timed_out_init(&timedout_start);
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
		if (timed_out(timedout_start, 100))
			return -1;

	// Send the address to start reading from
	I2C_Send7bitAddress(I2Cx, (device << 1), I2C_Direction_Transmitter);
	timed_out_init(&timedout_start);
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
		if (timed_out(timedout_start, 100))
			return -1;
	I2C_SendData(I2Cx, reg);
	timed_out_init(&timedout_start);
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		if (timed_out(timedout_start, 100))
			return -1;

	// Start the read
	I2C_GenerateSTART(I2Cx, ENABLE);
	timed_out_init(&timedout_start);
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
		if (timed_out(timedout_start, 100))
			return -1;
	I2C_Send7bitAddress(I2Cx, (device << 1), I2C_Direction_Receiver);
	timed_out_init(&timedout_start);
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
		if (timed_out(timedout_start, 100))
			return -1;

	if (length == 1) {
		I2C_AcknowledgeConfig(I2Cx, DISABLE);
		I2C_GenerateSTOP(I2Cx, ENABLE);

		timed_out_init(&timedout_start);
		while (!I2C_GetFlagStatus(I2Cx, I2C_FLAG_RXNE))
			if (timed_out(timedout_start, 100))
				return -1;
		data[0] = I2C_ReceiveData(I2Cx);
	}
	else if (length == 2) {
		I2C_NACKPositionConfig(I2Cx, I2C_NACKPosition_Next);
		I2C_AcknowledgeConfig(I2Cx, DISABLE);
		timed_out_init(&timedout_start);
		while (!I2C_GetFlagStatus(I2Cx, I2C_FLAG_BTF))
			if (timed_out(timedout_start, 100))
				return -1;
		I2C_GenerateSTOP(I2Cx, ENABLE);

		data[0] = I2C_ReceiveData(I2Cx);
		data[1] = I2C_ReceiveData(I2Cx);
	}
	else {
		uint16_t i = 0;
		while (length-- > 3) {
			timed_out_init(&timedout_start);
			while (!I2C_GetFlagStatus(I2Cx, I2C_FLAG_BTF))
				if (timed_out(timedout_start, 100))
					return -1;

			data[i++] = I2C_ReceiveData(I2Cx);
		}
		timed_out_init(&timedout_start);
		while (!I2C_GetFlagStatus(I2Cx, I2C_FLAG_BTF))
			if (timed_out(timedout_start, 100))
				return -1;
		I2C_AcknowledgeConfig(I2Cx, DISABLE);
		data[i++] = I2C_ReceiveData(I2Cx);
		I2C_GenerateSTOP(I2Cx, ENABLE);
		data[i++] = I2C_ReceiveData(I2Cx);
		timed_out_init(&timedout_start);
		while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED))
			if (timed_out(timedout_start, 100))
				return -1;
		data[i++] = I2C_ReceiveData(I2Cx);
	}

	timed_out_init(&timedout_start);
	while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF))
		if (timed_out(timedout_start, 100))
			return -1;

	return 0;
}

/** Write 1 byte to an I2C peripheral.
  *
  * @param *I2Cx:	Pointer to the I2C peripheral being used (x is 1, 2, or 3).
  * @param device:	I2C peripheral device address.
  * @param reg:		I2C peripheral register address.
  * @param data:	The data to be written.
  *
  * @return The status of the function.
***/
int8_t i2c_helper_write_byte (I2C_TypeDef *I2Cx, uint8_t device, uint8_t reg, uint8_t  data) {
	return i2c_helper_write_bytes(I2Cx, device, reg, 1, &data);
}

/** Write a word (2 bytes) to an I2C peripheral.
  *
  * @param *I2Cx:	Pointer to the I2C peripheral being used (x is 1, 2, or 3).
  * @param device:	I2C peripheral device address.
  * @param reg:		I2C peripheral register address.
  * @param data:	The data to be written.
  *
  * @return The status of the function.
***/
int8_t i2c_helper_write_word (I2C_TypeDef *I2Cx, uint8_t device, uint8_t reg, uint16_t  data) {
	uint8_t temp[2];

	temp[0] = (uint8_t)(data >> 8);
	temp[1] = (uint8_t)(data & 0xFF);

	return i2c_helper_write_bytes(I2Cx, device, reg, 2, temp);
}

/** Write a given number of bytes to an I2C peripheral.
  *
  * @param *I2Cx:	Pointer to the I2C peripheral being used (x is 1, 2, or 3).
  * @param device:	I2C peripheral device address.
  * @param reg:		I2C peripheral register address.
  * @param length:	Number of bytes to write.
  * @param *data:	Pointer to an array of bytes to be written.
  *
  * @return The status of the function.
***/
int8_t i2c_helper_write_bytes (I2C_TypeDef *I2Cx, uint8_t device, uint8_t reg, uint8_t length, const uint8_t  *data) {
	uint32_t timedout_start;
	timed_out_init(&timedout_start);
	while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY))
		if (timed_out(timedout_start, 100))
			return -1;
	I2C_GenerateSTART(I2Cx, ENABLE);
	timed_out_init(&timedout_start);
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
		if (timed_out(timedout_start, 100))
			return -1;
	I2C_Send7bitAddress(I2Cx, (device << 1), I2C_Direction_Transmitter);
	timed_out_init(&timedout_start);
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
		if (timed_out(timedout_start, 100))
			return -1;

	I2C_SendData(I2Cx, reg);
	timed_out_init(&timedout_start);
	while (!I2C_GetFlagStatus(I2Cx, I2C_FLAG_BTF))
		if (timed_out(timedout_start, 100))
			return -1;

	for (uint8_t i = 0; i < length; i++) {
		I2C_SendData(I2Cx, data[i]);
		timed_out_init(&timedout_start);
		while (!I2C_GetFlagStatus(I2Cx, I2C_FLAG_BTF))
			if (timed_out(timedout_start, 100))
				return -1;
	}

	I2C_GenerateSTOP(I2Cx, ENABLE);
	timed_out_init(&timedout_start);
	while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF))
		if (timed_out(timedout_start, 100))
			return -1;

	return 0;
}

/** Write 1 bit to an I2C peripheral.
  *
  * @param *I2Cx:	Pointer to the I2C peripheral being used (x is 1, 2, or 3).
  * @param device:	I2C peripheral device address.
  * @param reg:		I2C peripheral register address.
  * @param loc:		The bit location of the register to write (7 = MSB, 0 = LSB for big-endian).
  * @param data:	The data to be written.
  *
  * @return The status of the function.
***/
int8_t i2c_helper_write_bit (I2C_TypeDef *I2Cx, uint8_t device, uint8_t reg, uint8_t loc, uint8_t data) {
	return i2c_helper_write_bits(I2Cx, device, reg, loc, 1, data);
}

/** Write a given number of bits to an I2C peripheral.
  *
  * @param *I2Cx:	Pointer to the I2C peripheral being used (x is 1, 2, or 3).
  * @param device:	I2C peripheral device address.
  * @param reg:		I2C peripheral register address.
  * @param start:	The bit location of the register to write (7 = MSB, 0 = LSB for big-endian).
  * @param length:	The number of bits to write.
  * @param data:	The data to be written.
  *
  * @return The status of the function.
***/
int8_t i2c_helper_write_bits (I2C_TypeDef *I2Cx, uint8_t device, uint8_t reg, uint8_t start, uint8_t length, uint8_t data) {
	uint8_t buffer, mask;

	i2c_helper_read_byte(I2Cx, device, reg, &buffer);

	mask = ((1 << length) - 1) << (start - length + 1);
	data <<= (start - length + 1);
	data &= mask;
	buffer &= ~(mask);
	buffer |= data;

	return i2c_helper_write_byte(I2Cx, device, reg, buffer);
}

/** If an I2C bus freezes up or becomes unusable,
  * then un-sticking it may reset it to be usable again.
  *
  * @param *port:	Pointer to the GPIO port containing the I2C peripheral to un-stick.
  * @param scl_pin: 	SCL pin of the I2C peripheral.
  * @param sda_pin:	SDA pin of the I2C peripheral.
  *
  * @return The status of the function.
***/
int8_t i2c_helper_unstick(GPIO_TypeDef *port, uint16_t scl_pin, uint16_t sda_pin) {
	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_SetBits(port, scl_pin | sda_pin);

	GPIO_InitStruct.GPIO_Pin = scl_pin | sda_pin;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
	GPIO_Init(port, &GPIO_InitStruct);

	uint32_t timedout_start;
	for (uint8_t i = 0; i < 9; i++) {
		timed_out_init(&timedout_start);
		while (!GPIO_ReadInputDataBit(port, scl_pin)) {
			if (timed_out(timedout_start, 100))
				return -1;
		}

		GPIO_ResetBits(port, scl_pin);
		delay_us(10);
		GPIO_SetBits(port, scl_pin);
		delay_us(10);
	}

	// Create a Start / Stop Condition
	GPIO_ResetBits(port, sda_pin);
	delay_us(10);
	GPIO_ResetBits(port, scl_pin);
	delay_us(10);
	GPIO_SetBits(port, scl_pin);
	delay_us(10);
	GPIO_SetBits(port, sda_pin);
	delay_us(10);

	GPIO_InitStruct.GPIO_Pin = scl_pin | sda_pin;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
	GPIO_Init(port, &GPIO_InitStruct);

	return 0;
}
