#ifndef _BNO055_H
#define _BNO055_H


#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_syscfg.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_dma.h"
#include "misc.h"

#include "state.h"
#include "3dmath.h"
#include "clocks.h"
#include "serialtx.h"
#include "i2c_helper.h"
#include "other_stuff.h"


#define BNO055_SAMPLE_RATE				100	// 100 Hz

#define BNO055_I2C					I2C2
#define BNO055_ADDR                    			0x28
#define BNO055_ID					0xA0

#define BNO055_STATE_WAIT_FOR_INIT			1
#define BNO055_STATE_WAIT_FOR_CALIB			2
#define BNO055_STATE_READY				3
#define BNO055_STATE_ERROR				4

#define BNO055_CHIP_ID					0x00
#define BNO055_ACCEL_REV_ID				0x01
#define BNO055_MAG_REV_ID				0x02
#define BNO055_GYRO_REV_ID				0x03
#define BNO055_SW_REV_ID_LSB				0x04
#define BNO055_SW_REV_ID_MSB				0x05
#define BNO055_BL_REV_ID				0x06
#define BNO055_PAGE_ID					0x07

#define BNO055_ACCEL_DATA_X_LSB				0x08
#define BNO055_ACCEL_DATA_X_MSB				0x09
#define BNO055_ACCEL_DATA_Y_LSB				0x0A
#define BNO055_ACCEL_DATA_Y_MSB				0x0B
#define BNO055_ACCEL_DATA_Z_LSB				0x0C
#define BNO055_ACCEL_DATA_Z_MSB				0x0D

#define BNO055_MAG_DATA_X_LSB				0x0E
#define BNO055_MAG_DATA_X_MSB				0x0F
#define BNO055_MAG_DATA_Y_LSB				0x10
#define BNO055_MAG_DATA_Y_MSB				0x11
#define BNO055_MAG_DATA_Z_LSB				0x12
#define BNO055_MAG_DATA_Z_MSB				0x13

#define BNO055_GYRO_DATA_X_LSB				0x14
#define BNO055_GYRO_DATA_X_MSB				0x15
#define BNO055_GYRO_DATA_Y_LSB				0x16
#define BNO055_GYRO_DATA_Y_MSB				0x17
#define BNO055_GYRO_DATA_Z_LSB				0x18
#define BNO055_GYRO_DATA_Z_MSB				0x19

#define BNO055_EULER_H_LSB				0x1A
#define BNO055_EULER_H_MSB				0x1B
#define BNO055_EULER_R_LSB				0x1C
#define BNO055_EULER_R_MSB				0x1D
#define BNO055_EULER_P_LSB				0x1E
#define BNO055_EULER_P_MSB				0x1F

#define BNO055_QUATERNION_DATA_W_LSB			0x20
#define BNO055_QUATERNION_DATA_W_MSB			0x21
#define BNO055_QUATERNION_DATA_X_LSB			0x22
#define BNO055_QUATERNION_DATA_X_MSB			0x23
#define BNO055_QUATERNION_DATA_Y_LSB			0x24
#define BNO055_QUATERNION_DATA_Y_MSB			0x25
#define BNO055_QUATERNION_DATA_Z_LSB			0x26
#define BNO055_QUATERNION_DATA_Z_MSB			0x27

#define BNO055_LINEAR_ACCEL_DATA_X_LSB			0x28
#define BNO055_LINEAR_ACCEL_DATA_X_MSB			0x29
#define BNO055_LINEAR_ACCEL_DATA_Y_LSB			0x2A
#define BNO055_LINEAR_ACCEL_DATA_Y_MSB			0x2B
#define BNO055_LINEAR_ACCEL_DATA_Z_LSB			0x2C
#define BNO055_LINEAR_ACCEL_DATA_Z_MSB			0x2D

#define BNO055_GRAVITY_DATA_X_LSB			0x2E
#define BNO055_GRAVITY_DATA_X_MSB			0x2F
#define BNO055_GRAVITY_DATA_Y_LSB			0x30
#define BNO055_GRAVITY_DATA_Y_MSB			0x31
#define BNO055_GRAVITY_DATA_Z_LSB			0x32
#define BNO055_GRAVITY_DATA_Z_MSB			0x33

#define BNO055_TEMP					0x34

#define BNO055_CALIB_STATUS				0x35
#define BNO055_ST_RESULT				0x36
#define BNO055_INTR_STATUS				0x37

#define BNO055_SYS_CLK_STATUS				0x38
#define BNO055_SYS_STATUS				0x39
#define BNO055_SYS_ERR					0x3A

#define BNO055_UNIT_SEL					0x3B
#define BNO055_DATA_SELECT				0x3C

#define BNO055_OPR_MODE					0x3D
#define BNO055_PWR_MODE					0x3E

#define BNO055_SYS_TRIGGER				0x3F
#define BNO055_TEMP_SOURCE				0x40

#define BNO055_AXIS_MAP_CONFIG				0x41
#define BNO055_AXIS_MAP_SIGN				0x42

#define BNO055_SIC_MATRIX_0_LSB				0x43
#define BNO055_SIC_MATRIX_0_MSB				0x44
#define BNO055_SIC_MATRIX_1_LSB				0x45
#define BNO055_SIC_MATRIX_1_MSB				0x46
#define BNO055_SIC_MATRIX_2_LSB				0x47
#define BNO055_SIC_MATRIX_2_MSB				0x48
#define BNO055_SIC_MATRIX_3_LSB				0x49
#define BNO055_SIC_MATRIX_3_MSB				0x4A
#define BNO055_SIC_MATRIX_4_LSB				0x4B
#define BNO055_SIC_MATRIX_4_MSB				0x4C
#define BNO055_SIC_MATRIX_5_LSB				0x4D
#define BNO055_SIC_MATRIX_5_MSB				0x4E
#define BNO055_SIC_MATRIX_6_LSB				0x4F
#define BNO055_SIC_MATRIX_6_MSB				0x50
#define BNO055_SIC_MATRIX_7_LSB				0x51
#define BNO055_SIC_MATRIX_7_MSB				0x52
#define BNO055_SIC_MATRIX_8_LSB				0x53
#define BNO055_SIC_MATRIX_8_MSB				0x54

#define BNO055_ACCEL_OFFSET_X_LSB			0x55
#define BNO055_ACCEL_OFFSET_X_MSB			0x56
#define BNO055_ACCEL_OFFSET_Y_LSB			0x57
#define BNO055_ACCEL_OFFSET_Y_MSB			0x58
#define BNO055_ACCEL_OFFSET_Z_LSB			0x59
#define BNO055_ACCEL_OFFSET_Z_MSB			0x5A

#define BNO055_MAG_OFFSET_X_LSB				0x5B
#define BNO055_MAG_OFFSET_X_MSB				0x5C
#define BNO055_MAG_OFFSET_Y_LSB				0x5D
#define BNO055_MAG_OFFSET_Y_MSB				0x5E
#define BNO055_MAG_OFFSET_Z_LSB				0x5F
#define BNO055_MAG_OFFSET_Z_MSB				0x60

#define BNO055_GYRO_OFFSET_X_LSB			0x61
#define BNO055_GYRO_OFFSET_X_MSB			0x62
#define BNO055_GYRO_OFFSET_Y_LSB			0x63
#define BNO055_GYRO_OFFSET_Y_MSB			0x64
#define BNO055_GYRO_OFFSET_Z_LSB			0x65
#define BNO055_GYRO_OFFSET_Z_MSB			0x66

#define BNO055_ACCEL_RADIUS_LSB				0x67
#define BNO055_ACCEL_RADIUS_MSB				0x68
#define BNO055_MAG_RADIUS_LSB				0x69
#define BNO055_MAG_RADIUS_MSB				0x6A

#define BNO055_ACC_CONFIG				0x08
#define BNO055_MAG_CONFIG				0x09
#define BNO055_GYR_CONFIG_0				0x0A
#define BNO055_GYR_CONFIG_1				0x0B

#define BNO055_POWER_MODE_NORMAL			0x00
#define BNO055_POWER_MODE_LOWPOWER			0x01
#define BNO055_POWER_MODE_SUSPEND			0x02

#define BNO055_OPERATION_MODE_CONFIG			0x00
#define BNO055_OPERATION_MODE_ACCONLY			0x01
#define BNO055_OPERATION_MODE_MAGONLY			0x02
#define BNO055_OPERATION_MODE_GYRONLY			0x03
#define BNO055_OPERATION_MODE_ACCMAG			0x04
#define BNO055_OPERATION_MODE_ACCGYRO			0x05
#define BNO055_OPERATION_MODE_MAGGYRO			0x06
#define BNO055_OPERATION_MODE_AMG			0x07
#define BNO055_OPERATION_MODE_IMUPLUS			0x08
#define BNO055_OPERATION_MODE_COMPASS			0x09
#define BNO055_OPERATION_MODE_M4G			0x0A
#define BNO055_OPERATION_MODE_NDOF_FMC_OFF		0x0B
#define BNO055_OPERATION_MODE_NDOF			0x0C

#define BNO055_TRIGGER_SELFTEST				0x01
#define BNO055_TRIGGER_RST_SYS				0x20
#define BNO055_TRIGGER_RST_INT				0x40
#define BNO055_TRIGGER_CLK_SEL_INT			0x00
#define BNO055_TRIGGER_CLK_SEL_EXT			0x80

#define BNO055_STATUS_SYS_IDLE				0
#define BNO055_STATUS_PERIPH_INIT			1
#define BNO055_STATUS_SYS_INIT				2
#define BNO055_STATUS_EXEC_ST				3
#define BNO055_STATUS_FUSION_ON				4
#define BNO055_STATUS_FUSION_OFF			5
#define BNO055_STATUS_ERROR_NOERROR			6
#define BNO055_STATUS_ERROR_PERIPH_INIT			7
#define BNO055_STATUS_ERROR_SYS_INIT			8
#define BNO055_STATUS_ERROR_ST_FAIL			9
#define BNO055_STATUS_ERROR_INVALID_VALUE		10
#define BNO055_STATUS_ERROR_INVALID_ADDR		11
#define BNO055_STATUS_ERROR_WRITE_FAIL			12
#define BNO055_STATUS_ERROR_LOWPOWERNOTAVAIL		13
#define BNO055_STATUS_ERROR_ACCPOWERNOTAVAIL		14
#define BNO055_STATUS_ERROR_FUSIONCONFIG_FAIL		15
#define BNO055_STATUS_ERROR_SENSORCONFIG_FAIL		16

#define BNO055_UNIT_ACCEL_MS2				0x00
#define BNO055_UNIT_ACCEL_MG				0x01
#define BNO055_UNIT_GYRO_DPS				0x00
#define BNO055_UNIT_GYRO_RPS				0x02
#define BNO055_UNIT_EULER_DEG				0x00
#define BNO055_UNIT_EULER_RAD				0x04
#define BNO055_UNIT_TEMP_C				0x00
#define BNO055_UNIT_TEMP_F				0x10

#define BNO055_AXIS_CONFIG_X				0x00
#define BNO055_AXIS_CONFIG_Y				0x01
#define BNO055_AXIS_CONFIG_Z				0x02

#define BNO055_AXIS_X					1
#define BNO055_AXIS_Y					2
#define BNO055_AXIS_Z					3

#define BNO055_ACC_RANGE_2G				0x00
#define BNO055_ACC_RANGE_4G				0x01
#define BNO055_ACC_RANGE_8G				0x02
#define BNO055_ACC_RANGE_16G				0x03

#define BNO055_ACC_BW_7_81				0x00
#define BNO055_ACC_BW_15_63				0x04
#define BNO055_ACC_BW_31_25				0x08
#define BNO055_ACC_BW_62_5				0x0C
#define BNO055_ACC_BW_125				0x10
#define BNO055_ACC_BW_250				0x14
#define BNO055_ACC_BW_500				0x18
#define BNO055_ACC_BW_1000				0x1C

#define BNO055_ACC_OPMODE_NORMAL			0x00
#define BNO055_ACC_OPMODE_SUSPEND			0x20
#define BNO055_ACC_OPMODE_LOWPOWER1			0x40
#define BNO055_ACC_OPMODE_STANDBY			0x60
#define BNO055_ACC_OPMODE_LOWPOWER2			0x80
#define BNO055_ACC_OPMODE_DEEPSUSPEND			0xA0

#define BNO055_GYR_RANGE_2000				0x00
#define BNO055_GYR_RANGE_1000				0x01
#define BNO055_GYR_RANGE_500				0x02
#define BNO055_GYR_RANGE_250				0x03
#define BNO055_GYR_RANGE_125				0x04

#define BNO055_GYR_BW_523				0x00
#define BNO055_GYR_BW_230				0x08
#define BNO055_GYR_BW_116				0x10
#define BNO055_GYR_BW_47				0x18
#define BNO055_GYR_BW_23				0x20
#define BNO055_GYR_BW_12				0x28
#define BNO055_GYR_BW_64				0x30
#define BNO055_GYR_BW_32				0x38

#define BNO055_GYR_OPMODE_NORMAL			0x00
#define BNO055_GYR_OPMODE_FASTPOWERUP			0x01
#define BNO055_GYR_OPMODE_DEEPSUSPEND			0x02
#define BNO055_GYR_OPMODE_SUSPEND			0x03
#define BNO055_GYR_OPMODE_ADVPOWERSAVE			0x04

#define BNO055_MAG_RATE_2HZ				0x00
#define BNO055_MAG_RATE_6HZ				0x01
#define BNO055_MAG_RATE_8HZ				0x02
#define BNO055_MAG_RATE_10HZ				0x03
#define BNO055_MAG_RATE_15HZ				0x04
#define BNO055_MAG_RATE_20HZ				0x05
#define BNO055_MAG_RATE_25HZ				0x06
#define BNO055_MAG_RATE_30HZ				0x07

#define BNO055_MAG_OPMODE_LOWPOWER			0x00
#define BNO055_MAG_OPMODE_REGULAR			0x08
#define BNO055_MAG_OPMODE_ENHANCEDREGULAR		0x10
#define BNO055_MAG_OPMODE_HIGHACCURACY			0x18

#define BNO055_MAG_PWRMODE_NORMAL			0x00
#define BNO055_MAG_PWRMODE_SLEEP			0x20
#define BNO055_MAG_PWRMODE_SUSPEND			0x40
#define BNO055_MAG_PWRMODE_FORCE			0x60

#define BNO055_BUFFER_SIZE				51	// READS 0x08 - 0x3A
#define BNO055_BUFFER_OFFSET				8


struct {
	int16_t gyr_x, gyr_y, gyr_z;
	int16_t acc_x, acc_y, acc_z;
	int16_t mag_x, mag_y, mag_z;
	int16_t eul_h, eul_p, eul_r;
	int16_t quat_w, quat_x, quat_y, quat_z;
	int16_t linear_acc_x, linear_acc_y, linear_acc_z;
	int16_t gravity_x, gravity_y, gravity_z;
	int8_t  temperature;
} bno055_raw_values;

uint8_t			bno055_last_state;

volatile bool 		bno055_buffer_ready;
volatile bool 		bno055_dma_running;

uint8_t			bno055_state;
uint8_t			bno055_status;
uint8_t			bno055_dma_buffer[BNO055_BUFFER_SIZE];
int16_t			bno055_error_level;


uint16_t	bno055_config();
uint16_t 	bno055_init(bool skip_phase1);
int8_t 		bno055_update();
void		bno055_change_state(uint8_t new_state);
int8_t		bno055_start_dma();
int8_t		bno055_parse_buffer();
int8_t		bno055_cook_values();
int8_t 		bno055_get_calibration_data(uint8_t *calibration_percent);
int8_t 		bno055_get_status(uint8_t *status);
int8_t 		bno055_axis_remap(int8_t remap_x, int8_t remap_y, int8_t remap_z, uint8_t *map, uint8_t *sign);
void		bno055_error_reset(int16_t error_level);


#endif
