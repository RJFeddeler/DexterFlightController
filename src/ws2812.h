#ifndef _WS2812_H
#define _WS2812_H


#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_dma.h"
#include "misc.h"

#include "state.h"
#include "clocks.h"
#include "other_stuff.h"


#define WS2812_ARM_COUNT			6
#define WS2812_LEDS_PER_ARM_TUBE		11
#define WS2812_LEDS_PER_ARM_MOUNT		3
#define WS2812_LEDS_PER_ARM_ALL			(WS2812_LEDS_PER_ARM_TUBE + WS2812_LEDS_PER_ARM_MOUNT)

#define WS2812_LED_COUNT			(WS2812_ARM_COUNT * WS2812_LEDS_PER_ARM_ALL)
#define WS2812_LAYER_COUNT			2
#define WS2812_LAYER_LENGTH_MASK		0xF000
#define WS2812_SAMPLE_RATE			200		// 200 Hz

#define WS2812_DEFAULT_SAMPLE_RATE		0		//   0 Hz (OFF)
#define WS2812_DEFAULT_BRIGHTNESS		0
#define WS2812_DEFAULT_COLOR			cp_off

#define WS2812_SIGNAL_START_DELAY		4
#define WS2812_SIGNAL_END_DELAY			42

						// 24 bits per LED + end of signal delay 50us + start of signal delay (to ensure first bit is seen)
#define WS2812_BUFFER_SIZE			(WS2812_LED_COUNT * 24 + WS2812_SIGNAL_START_DELAY + WS2812_SIGNAL_END_DELAY)
#define WS2812_BIT_1				68 		// ~800 ns
#define WS2812_BIT_0				30 		// ~350 ns
#define WS2812_BIT_Z				0  		//    0 ns

#define WS2812_GREEN				0
#define WS2812_RED				1
#define WS2812_BLUE				2
#define WS2812_BRIGHTNESS			3


struct ColorSet {
	struct Color cs_start[WS2812_ARM_COUNT];
	struct Color cs_end[WS2812_ARM_COUNT];
};

enum StripSection {
	ss_Tube,
	ss_Mount
};

enum AnimationLayer {
	al_background,
	al_layer1
};

enum ColorPalette {
	cp_off,
	cp_white,
	cp_red,
	cp_green,
	cp_blue,
	cp_magenta,
	cp_orange,
	cp_yellow,
	cp_cyan,
	cp_purple
};

enum AnimationOptions {
	opt_none			= 0x0000,	// DEFAULT : NO OPTIONS

	opt_isolate_arm			= 0x0000,	// DEFAULT
	opt_pass_to_next_arm		= 0x0001,

	opt_gradient_none		= 0x0000,	// DEFAULT
	opt_gradient_linear		= 0x0002,
	opt_gradient_center		= 0x0004,

	opt_slide_off			= 0x0000,	// DEFAULT
	opt_slide_in			= 0x0008,
	opt_slide_out			= 0x0010,
	opt_slide_bounce		= 0x0018,

	opt_blink_off			= 0x0000,	// DEFAULT
	opt_blink_on			= 0x0020,
	opt_blink_offset		= 0x0040,

	opt_anim_blend_none		= 0x0000,	// DEFAULT
	opt_anim_blend_slow		= 0x0100,
	opt_anim_blend_medium		= 0x0200,
	opt_anim_blend_fast		= 0x0300,

	opt_max_blend_none		= 0x0000,	// DEFAULT
	opt_max_blend_33		= 0x0400,
	opt_max_blend_66		= 0x0800,
	opt_max_blend_100		= 0x0C00,

	opt_layer_length_0		= 0x0000,	// DEFAULT
	opt_layer_length_1		= 0x1000,
	opt_layer_length_2		= 0x2000,
	opt_layer_length_3		= 0x3000,
	opt_layer_length_4		= 0x4000,
	opt_layer_length_5		= 0x5000,
	opt_layer_length_6		= 0x6000,
	opt_layer_length_7		= 0x7000,
	opt_layer_length_8		= 0x8000,
	opt_layer_length_9		= 0x9000,
	opt_layer_length_10		= 0xA000,
	opt_layer_length_11		= 0xB000,
	opt_layer_length_12		= 0xC000,
	opt_layer_length_13		= 0xD000,
	opt_layer_length_14		= 0xE000,
	opt_layer_length_15		= 0xF000
};

struct Animation {
	uint16_t sample_rate;
	uint32_t last_update;
	bool first_run_background;
	bool first_run_layer1;
	uint16_t frame_background;
	uint16_t frame_layer1;
	struct ColorSet cs_background;
	struct ColorSet cs_layer1;
	enum AnimationOptions ao_background;
	enum AnimationOptions ao_layer1;
};

uint8_t			ws2812_last_state;

volatile bool 		ws2812_dma_running;
uint16_t 		ws2812_dma_buffer[WS2812_BUFFER_SIZE];
struct Color		ws2812_colors[WS2812_LED_COUNT];

uint8_t			ws2812_global_brightness;
struct Animation 	ws2812_animation_tube;
struct Animation 	ws2812_animation_mount;


uint8_t		ws2812_config();
uint8_t		ws2812_init();
int8_t		ws2812_update();
void		ws2812_change_state(uint8_t new_state);
int8_t		ws2812_convert_colors();
int8_t		ws2812_push_colors();
int8_t	 	ws2812_animate(enum StripSection ss);
int8_t 		ws2812_set_color(enum StripSection section, uint8_t arm, uint8_t led, struct Color color);
struct Color	ws2812_get_color(enum StripSection section, uint8_t arm, uint8_t led);
int8_t	 	ws2812_reset_all_colors();
int8_t		ws2812_copy_color(struct Color *dest, struct Color source);
int8_t 		ws2812_set_global_brightness(uint8_t brightness);
struct Color 	ws2812_fill_color_from_palette(enum ColorPalette source, uint8_t brightness);
struct Color 	ws2812_fill_color_from_rgb(uint8_t red, uint8_t green, uint8_t blue, uint8_t brightness);
int8_t		ws2812_fill_colorset(struct ColorSet *set, uint8_t arm, enum ColorPalette start_color, uint8_t start_brightness, enum ColorPalette end_color, uint8_t end_brightness);
int8_t		ws2812_set_colorset(enum StripSection ss, enum AnimationLayer al, struct ColorSet colorset, bool reset);
int8_t		ws2812_copy_colorset(struct ColorSet *dest, struct ColorSet source);
int8_t		ws2812_reset_colorset(struct ColorSet *set);
int8_t		ws2812_set_animation_options(enum StripSection ss, enum AnimationLayer al, uint16_t rate, enum AnimationOptions options, bool reset);
int8_t		ws2812_copy_animation_options(enum AnimationOptions *dest, enum AnimationOptions source);
int8_t		ws2812_reset_animation(struct Animation *animation);
struct Color	ws2812_gradient(float progress, struct Color start, struct Color end);
bool		ws2812_is_animation(enum AnimationOptions ao, uint16_t layer_length);
void		ws2812_get_processing(uint8_t *ws2812);


#endif
