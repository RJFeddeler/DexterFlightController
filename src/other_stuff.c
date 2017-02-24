#include "other_stuff.h"

/** Calculate the value of a number raised to a given power.
  *
  * @param base:	The base number.
  * @param exp:		The exponent to raise the base value to.
  *
  * @return The value of the base number raised to the exponents power.
***/
uint32_t power(uint32_t base, uint32_t exp) {
	if (exp < 1 || base < 1)
		return 1;
	if (exp == 1)
		return base;
	else
		return base * power(base, exp - 1);
}

/** Get the absolute value of a number.
  *
  * @param value:	The number to find the absolute value of.
  *
  * @return The absolute value of the number.
***/
int32_t abs(int32_t value) {
	return (value < 0) ? -value : value;
}

/** Get the absolute value of a floating-point number.
  *
  * @param value:	The number to find the absolute value of.
  *
  * @return The absolute value of the number.
***/
float abs_f(float value) {
	return value < 0 ? -value : value;
}

/** Determine the smaller of two values.
  *
  * @param value1:	The first value.
  * @param value2:	The second value.
  *
  * @return The smaller number.
***/
uint16_t min(int16_t value1, int16_t value2) {
	return (value1 < value2 ? value1 : value2);
}

/** Determine the smallest value in an array of values.
  *
  * @param *values:	Pointer to the array of values.
  * @param length:	The number of values in the array.
  *
  * @return The value of the smallest number in the array.
***/
int16_t min_array(int16_t *values, uint16_t length) {
	int16_t min = values[0];

	for (uint16_t i = 1; i < length; i++)
		if (values[i] < min)
			min = values[i];

	return min;
}

/** Determine the larger of two values.
  *
  * @param value1:	The first value.
  * @param value2:	The second value.
  *
  * @return The larger number.
***/
uint16_t max(int16_t value1, int16_t value2) {
	return (value1 > value2 ? value1 : value2);
}

/** Determine the largest value in an array of values.
  *
  * @param *values:	Pointer to the array of values.
  * @param length:	The number of values in the array.
  *
  * @return The value of the largest number in the array.
***/
int16_t max_array(int16_t *values, uint16_t length) {
	int16_t max = values[0];

	for (uint16_t i = 1; i < length; i++)
		if (values[i] > max)
			max = values[i];

	return max;
}

/** Convert a value from one scale to another.
  *
  * @param start_scale_value:	The value to change the scale of.
  * @param start_scale_min:	The minimum value of the starting scale.
  * @param start_scale_max:	The maximum value of the starting scale.
  * @param end_scale_min:	The minimum value of the converted scale.
  * @param end_scale_max:	The maximum value of the converted scale.
  *
  * @return The original value scaled to the new range of numbers.
***/
int32_t scale(int32_t start_scale_value, int32_t start_scale_min, int32_t start_scale_max, int32_t end_scale_min, int32_t end_scale_max) {
	int32_t value = (int32_t)(((end_scale_max - end_scale_min) / (float)(start_scale_max - start_scale_min)) * (float)(start_scale_value - start_scale_min) + (float)end_scale_min);

	return clamp(value, end_scale_min, end_scale_max);
}

/** Convert a floating-point value from one scale to another.
  *
  * @param start_scale_value:	The value to change the scale of.
  * @param start_scale_min:	The minimum value of the starting scale.
  * @param start_scale_max:	The maximum value of the starting scale.
  * @param end_scale_min:	The minimum value of the converted scale.
  * @param end_scale_max:	The maximum value of the converted scale.
  *
  * @return The original floating-point value scaled to the new range of numbers.
***/
float scalef(float start_scale_value, float start_scale_min, float start_scale_max, float end_scale_min, float end_scale_max) {
	float value = ((end_scale_max - end_scale_min) / (start_scale_max - start_scale_min)) * (start_scale_value - start_scale_min) + end_scale_min;

	return clampf(value, end_scale_min, end_scale_max);
}

/** Scale each element in an array by a given scalar.
  *
  * @param *values:	Pointer to the array of values to be scaled.
  * @param length:	The number of elements in the array.
  * @param scale:	The scalar to multiply each element of the array by.
  *
  * @return Void.
***/
void scale_array(int16_t *values, uint16_t length, float scale) {
	for (uint16_t i = 0; i < length; i++)
		values[i] *= scale;
}

/** Make sure a value stays within a valid range between min and max.
  *
  * @param value:	The number to be clamped.
  * @param min:		The smallest valid value that the given number can be.
  * @param max:		The largest valid value that the given number can be.
  *
  * @return The clamped value.
***/
int32_t clamp(int32_t value, int32_t min, int32_t max) {
	if (value < min)
		return min;
	else if (value > max)
		return max;
	else
		return value;
}

/** Make sure a floating-point value stays within a valid range between min and max.
  *
  * @param value:	The number to be clamped.
  * @param min:		The smallest valid value that the given number can be.
  * @param max:		The largest valid value that the given number can be.
  *
  * @return The clamped floating-point value.
***/
float clampf(float value, float min, float max) {
	if (value < min)
		return min;

	if (value > max)
		return max;

	return value;
}

/** Copy a memory location, byte by byte, of a given length from one memory location to another.
  *
  * @param *dest:	Pointer to the destination location to be copied to.
  * @param *source:	Pointer to the source data to be copied.
  * @param length:	The number of bytes to copy.
  *
  * @return Void.
***/
void mem_copy(uint8_t *dest, const uint8_t *source, uint16_t length) {
	for (uint16_t i = 0; i < length; i++)
		dest[i] = source[i];
}

/** Compare two memory locations to see if they contain the same data.
  *
  * @param *dest:	Pointer to the data being compared to the source.
  * @param *source:	Pointer to the data to compare.
  * @param length:	The number of bytes to compare.
  *
  * @return True if the data is the same, otherwise False.
***/
bool mem_cmp(const uint8_t *dest, const uint8_t *source, uint16_t length) {
	for (uint16_t i = 0; i < length; i++) {
		if (dest[i] != source[i])
			return true;
	}

	return false;
}

/** Perform a low pass filter on a changing value to reduce noise.
  *
  * @param lpf_beta:		Amount of the raw value that makes it through the
  * 				filter (1.0 = no filter, 0.0 = fully filtered).
  * @param raw_value:		The value that needs to be filtered.
  * @param *filtered_value:	Pointer to the filtered value.
  *
  * @return Void.
***/
void low_pass_filter(float lpf_beta, float raw_value, float *filtered_value) {
	float temp, delta;

	delta = raw_value - *filtered_value;
	temp = *filtered_value + (lpf_beta * delta);

	*filtered_value = temp;
}

/** Convert a temperature in Celcius to Fahrenheit.
  *
  * @param c:	The temperature in Celcius.
  *
  * @return The temperature in Fahrenheit.
***/
float c_to_f(float c) {
	return c * 1.8f + 32.0f;
}

/** Convert a temperature in Fahrenheit to Celcius.
  *
  * @param f:	The temperature in Fahrenheit.
  *
  * @return The temperature in Celcius.
***/
float f_to_c(float f) {
	return (f - 32.0f) / 1.8f;
}

/** Convert a distance in meters to feet.
  *
  * @param m:	The distance in meters.
  *
  * @return The distance in feet.
***/
float m_to_ft(float m) {
	return m * 3.2808f;
}

/** Convert a distance in feet to meters.
  *
  * @param ft:	The distance in feet.
  *
  * @return The distance in meters.
***/
float ft_to_m(float ft) {
	return ft / 3.2808f;
}
