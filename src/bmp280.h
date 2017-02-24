#ifndef _BMP280_H
#define _BMP280_H


#include <math.h>
#include "stm32f4xx.h"
#include "stm32f4xx_i2c.h"

#include "state.h"
#include "3dmath.h"
#include "clocks.h"
#include "i2c_helper.h"
#include "serialtx.h"
#include "other_stuff.h"


#define BMP280_SAMPLE_RATE			25

#define BMP280_ADDR             		0x77
#define BMP280_ID				0x58

#define BMP280_DIG_T1				0x88
#define BMP280_DIG_T2				0x8A
#define BMP280_DIG_T3				0x8C
#define BMP280_DIG_P1				0x8E
#define BMP280_DIG_P2				0x90
#define BMP280_DIG_P3				0x92
#define BMP280_DIG_P4				0x94
#define BMP280_DIG_P5				0x96
#define BMP280_DIG_P6				0x98
#define BMP280_DIG_P7				0x9A
#define BMP280_DIG_P8				0x9C
#define BMP280_DIG_P9				0x9E

#define BMP280_COMPENSATION_START		BMP280_DIG_T1
#define BMP280_COMPENSATION_LENGTH		24

#define BMP280_CHIP_ID				0xD0
#define BMP280_RESET				0xE0

#define BMP280_STATUS				0xF3
#define BMP280_CTRL_MEAS			0xF4
#define BMP280_CONFIG				0xF5
#define BMP280_PRESSUREDATA			0xF7
#define BMP280_TEMPDATA				0xFA

#define BMP280_DATA_START			BMP280_PRESSUREDATA
#define BMP280_DATA_LENGTH			6

#define BMP280_RESET_CMD			0xB6

#define BMP280_STATUS_MEASURING			0x08
#define BMP280_STATUS_IMUPDATE			0x01

#define BMP280_CTRLMEAS_MODE_SLEEP		0x00
#define BMP280_CTRLMEAS_MODE_FORCED		0x01
#define BMP280_CTRLMEAS_MODE_NORMAL		0x03

#define BMP280_CTRLMEAS_OVERSAMP_P_1		0x04
#define BMP280_CTRLMEAS_OVERSAMP_P_2		0x08
#define BMP280_CTRLMEAS_OVERSAMP_P_4		0x0C
#define BMP280_CTRLMEAS_OVERSAMP_P_8		0x10
#define BMP280_CTRLMEAS_OVERSAMP_P_16		0x14

#define BMP280_CTRLMEAS_OVERSAMP_T_1		0x20
#define BMP280_CTRLMEAS_OVERSAMP_T_2		0x40
#define BMP280_CTRLMEAS_OVERSAMP_T_4		0x60
#define BMP280_CTRLMEAS_OVERSAMP_T_8		0x80
#define BMP280_CTRLMEAS_OVERSAMP_T_16		0xA0

#define BMP280_CONFIG_3WIRESPI			0x01

#define BMP280_CONFIG_FILTER_OFF		0x00
#define BMP280_CONFIG_FILTER_2			0x04
#define BMP280_CONFIG_FILTER_4			0x08
#define BMP280_CONFIG_FILTER_8			0x0C
#define BMP280_CONFIG_FILTER_16			0x10

#define BMP280_CONFIG_INACTIVE_0_5		0x00
#define BMP280_CONFIG_INACTIVE_62_5		0x20
#define BMP280_CONFIG_INACTIVE_125		0x40
#define BMP280_CONFIG_INACTIVE_250		0x60
#define BMP280_CONFIG_INACTIVE_500		0x80
#define BMP280_CONFIG_INACTIVE_1000		0xA0
#define BMP280_CONFIG_INACTIVE_2000		0xC0
#define BMP280_CONFIG_INACTIVE_4000		0xE0

#define BMP280_UNIT_C				0
#define BMP280_UNIT_F				1
#define BMP280_UNIT_M				0
#define BMP280_UNIT_FT				1


struct KalmanStruct {
	float z, v;
	float var_z, var_zaccel, var_zaccelbias, abias;

	float covar_zz, covar_zv, covar_za;
	float covar_vz, covar_vv, covar_va;
	float covar_az, covar_av, covar_aa;
};

uint8_t			bmp280_last_state;
uint32_t		bmp280_counter;

float			bmp280_avg_p_sum, bmp280_avg_t_sum, bmp280_avg_a_sum;

int32_t			bmp280_up, bmp280_ut;
int32_t			bmp280_tfine;

struct KalmanStruct	bmp280_kalman;

float			bmp280_pressure, bmp280_start_pressure;
float			bmp280_temperature, bmp280_start_temperature;
float			bmp280_altitude, bmp280_alt_offset;
float			bmp280_climb_vel, bmp280_climb_acc;

vector			bmp280_int_sum;
uint8_t			bmp280_int_count;

uint32_t		bmp280_kalman_last_time;

uint16_t		bmp280_dig_t1, bmp280_dig_p1;
int16_t			bmp280_dig_t2, bmp280_dig_t3, bmp280_dig_p2, bmp280_dig_p3, bmp280_dig_p4;
int16_t			bmp280_dig_p5, bmp280_dig_p6, bmp280_dig_p7, bmp280_dig_p8, bmp280_dig_p9;


uint16_t	bmp280_config();
uint16_t	bmp280_init();
int8_t		bmp280_update();
void		bmp280_change_state(uint8_t new_state);
int8_t		bmp280_compensate_p(int32_t raw, float *value);
int8_t		bmp280_compensate_t(int32_t raw, float *value);
int8_t		bmp280_calculate_alt(float *value);
void		bmp280_integrate_accz(vector v);
void		bmp280_kalman_config(float variance_z, float variance_zaccel, float variance_zaccelbias, float abias_initial);
void		bmp280_kalman_update(float z, float a, float *altitude, float *velocity);
int8_t		bmp280_read_raw_data();
int8_t		bmp280_read_calibration_data();
float		bmp280_get_pressure();
float		bmp280_get_temperature(uint8_t unit);
float		bmp280_get_altitude(uint8_t unit);
void		bmp280_set_start_pressure(float value);
void		bmp280_set_start_temperature(float value, uint8_t unit);


#endif
