#ifndef _STATE_H
#define _STATE_H


#include <stdbool.h>
#include "stm32f4xx.h"

#include "clocks.h"
#include "3dmath.h"
#include "serialtx.h"
#include "other_stuff.h"


#define STATE_SAMPLE_RATE			800

#define STATE_SYS_CALIBRATING			1
#define STATE_SYS_IDLE				2
#define STATE_SYS_ARMING			3
#define STATE_SYS_ARMED				4

#define STATE_ARMING_TIME			2000

#define STATE_MOTOR_COUNT			6
#define STATE_RX_CHANNEL_COUNT			16

#define STATE_MOTOR_1				0
#define STATE_MOTOR_2				1
#define STATE_MOTOR_3				2
#define STATE_MOTOR_4				3
#define STATE_MOTOR_5				4
#define STATE_MOTOR_6				5

#define STATE_AXIS_PITCH			0
#define STATE_AXIS_ROLL				1
#define STATE_AXIS_YAW				2

#define STATE_PERIPH_X8R			0x01
#define STATE_PERIPH_BNO055			0x02
#define STATE_PERIPH_NEOM8N			0x04
#define STATE_PERIPH_BMP280			0x08
#define STATE_PERIPH_HCSR05			0x10
#define STATE_PERIPH_BATTERY			0x20
#define STATE_PERIPH_MOTOR			0x40
#define STATE_PERIPH_WS2812			0x80
#define STATE_PERIPH_DATALOGGER			0x100

#define STATE_ERROR_CLOCKS			0x200
#define STATE_ERROR_ONBOARD_LED			0x400
#define STATE_ERROR_SERIALTX			0x800
#define STATE_ERROR_STATE			0x1000
#define STATE_ERROR_PERIPH_X8R			STATE_PERIPH_X8R
#define STATE_ERROR_PERIPH_BNO055		STATE_PERIPH_BNO055
#define STATE_ERROR_PERIPH_NEOM8N		STATE_PERIPH_NEOM8N
#define STATE_ERROR_PERIPH_BMP280		STATE_PERIPH_BMP280
#define STATE_ERROR_PERIPH_HCSR05		STATE_PERIPH_HCSR05
#define STATE_ERROR_PERIPH_BATTERY		STATE_PERIPH_BATTERY
#define STATE_ERROR_PERIPH_MOTOR		STATE_PERIPH_MOTOR
#define STATE_ERROR_PERIPH_WS2812		STATE_PERIPH_WS2812
#define STATE_ERROR_PERIPH_DATALOGGER		STATE_PERIPH_DATALOGGER

#define STATE_MAX_TILT				40
#define STATE_MAX_YAW				40

#define STATE_AUTOTUNE_RATE			100
#define STATE_AUTOTUNE_STEP_COUNT		10
#define STATE_AUTOTUNE_THROTTLE_MAX_OFFSET	100
#define STATE_AUTOTUNE_TILT			25.0f
#define STATE_AUTOTUNE_MAX_OVERSHOOT		1.0f
#define STATE_AUTOTUNE_MAX_BOUNCE		3.0f
#define STATE_AUTOTUNE_START_INCREMENT_D	0.1f

#define STATE_RX_DEADZONE			15
#define STATE_RX_VALUE_MIN			172
#define STATE_RX_VALUE_MAX			1811
#define STATE_RX_VALUE_MIDPOINT			((STATE_RX_VALUE_MIN + STATE_RX_VALUE_MAX) / 2)

#define STATE_MOTOR_VALUE_MIN			1068
#define STATE_MOTOR_VALUE_MAX			1868

#define STATE_RX_CHANNEL_THROTTLE		0
#define STATE_RX_CHANNEL_AILERON		1
#define STATE_RX_CHANNEL_ELEVATOR		2
#define STATE_RX_CHANNEL_RUDDER			3
#define STATE_RX_CHANNEL_TOGGLE_L_3POS_1	4
#define STATE_RX_CHANNEL_TOGGLE_L_3POS_2 	5
#define STATE_RX_CHANNEL_TOGGLE_L_3POS_3 	6
#define STATE_RX_CHANNEL_TOGGLE_R_3POS_1 	7
#define STATE_RX_CHANNEL_TOGGLE_R_3POS_2 	8
#define STATE_RX_CHANNEL_TOGGLE_R_3POS_3 	9
#define STATE_RX_CHANNEL_TOGGLE_L_2POS	 	10
#define STATE_RX_CHANNEL_TOGGLE_R_2POS	 	11
#define STATE_RX_CHANNEL_TOGGLE_L_KNOB	 	12
#define STATE_RX_CHANNEL_TOGGLE_R_KNOB	 	13
#define STATE_RX_CHANNEL_TOGGLE_L_SLIDER 	14
#define STATE_RX_CHANNEL_TOGGLE_R_SLIDER 	15

#define STATE_RX_RULE_NONE			0x00
#define STATE_RX_RULE_EQUAL_TO			0x01
#define STATE_RX_RULE_GREATER_THAN		0x02
#define STATE_RX_RULE_LESS_THAN			0x04

#define STATE_RX_POSITION_0			STATE_RX_VALUE_MIN
#define STATE_RX_POSITION_1			STATE_RX_VALUE_MIDPOINT
#define STATE_RX_POSITION_2			STATE_RX_VALUE_MAX

#define STATE_RX_FAILSAFE_STAGE_1		10
#define STATE_RX_FAILSAFE_STAGE_2		100

#define STATE_IMU_FAILSAFE_STAGE_1		5
#define STATE_IMU_FAILSAFE_STAGE_2		100

#define PID_MAX_ERROR_SUM			100


typedef enum {
	FSTATUS_OK = 0,
	FSTATUS_UPDATE,
	FSTATUS_ERROR1,
	FSTATUS_ERROR2,
	FSTATUS_ERROR3
} FuncStatus;

enum AutotuneState {
	AT_OFF = 0,
	AT_START,
	AT_RUNNING,
	AT_PAUSED,
	AT_DONE
};

enum AutotuneStep {
	STEP_0toNEG = 0,
	STEP_NEGtoPOS,
	STEP_POStoNEG,
	STEP_NEGto0,
	STEP_POSto0
};

struct PID {
	float p;
	float i;
	float d;
	float i_error_sum;
	float value;
};

struct Range {
	float min;
	float max;
};

struct AutotuneRange {
	struct Range axis;
	float current_value;
	float step;
};

struct AutotuneTest {
	struct PID pid[3];
	uint8_t axis;
	float overshoot;
	float bounce;
	enum AutotuneStep step;
	float start_heading;
	uint32_t start_tick;
	uint32_t ticks_to_desired_angle;
	uint32_t ticks_to_end_of_bounce;
};

struct {
	uint8_t			status;
	uint8_t			imu_calibration_percent;
	bool			mode_hover;
	enum AutotuneState	mode_autotune;

	float			voltage;
	float			current;

	vector 			gyr, acc, mag;
	vector 			gravity, linear_acc, linear_acc_notilt;

	float			velocity, velocity_gps;
	float			climb_acc, climb_velocity, horizon_velocity;

	float 			latitude, longitude;

	float			temperature_imu, temperature_bp;

	float 			pressure, altitude;

	float 			alt_bp, alt_us, alt_gps;

	float 			pitch, roll, heading;
	quaternion 		q;

	uint16_t		imu_failsafe_count;
	uint16_t		rx_failsafe_count;
	uint16_t 		rx_channels[STATE_RX_CHANNEL_COUNT];
	uint16_t		motors[STATE_MOTOR_COUNT];

	struct PID		pid[3];

	// GPS PACKET INFO (ALSO SET UP GPS TO SEND MORE PACKETS)
} state;

struct waypoint {
	float 		latitude, longitude;
	float 		temperature, pressure, altitude;
	float		climb_velocity, horizon_velocity;
	float 		pitch, roll, heading;
	quaternion 	q;
};

volatile uint16_t	state_calibrated;
uint16_t		state_periphs;
uint16_t 		state_rx_arming_position[STATE_RX_CHANNEL_COUNT][2];
struct AutotuneTest	*state_autotune_current_test;
struct AutotuneTest	state_autotune_test_history[3][STATE_AUTOTUNE_STEP_COUNT];
struct AutotuneRange	state_autotune_range[3];
uint16_t		state_autotune_start_throttle;
struct waypoint		home;
uint8_t			ws2812[252];


uint16_t	state_init(uint16_t periphs, struct PID pid[3], uint16_t rules[STATE_RX_CHANNEL_COUNT][2]);
int8_t		state_update();
void		state_set_calibrated(uint16_t periph);
void		state_set_uncalibrated(uint16_t periph);
bool 		state_periph_enabled(uint16_t periph);
int8_t		state_rx_handle_switch_changes();
bool 		state_rx_ready_to_arm();
int8_t		state_rx_add_arming_rule(uint16_t rules[STATE_RX_CHANNEL_COUNT][2], uint8_t channel, uint16_t rule, uint16_t value);
void		state_rx_reset_arming_rules(uint16_t rules[STATE_RX_CHANNEL_COUNT][2]);
uint8_t		state_rx_get_switch_position(uint16_t switch_value);
void		state_autotune_set(float p_range_min_pitch, float p_range_max_pitch, float p_range_min_roll, float p_range_max_roll, float p_range_min_yaw, float p_range_max_yaw);
void		state_autotune_reset_test(struct AutotuneTest *test);
int32_t		state_get_throttle();
void		state_reset(uint8_t status);
void		waypoint_create(struct waypoint *w);
void		waypoint_reset(struct waypoint *w);
int8_t 		pid_update();
int8_t		pid_set(struct PID *pid, float p, float i, float d);
int8_t 		pid_reset();
void 		pid_zero(struct PID *pid);


#endif
