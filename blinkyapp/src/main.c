/*
 * SPDX-FileCopyrightText: Copyright 2023-2024 Andreas Sandberg <andreas@sandberg.uk>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/led.h>
#include <zephyr/random/random.h>

LOG_MODULE_REGISTER(main, CONFIG_BLINKY_LOG_LEVEL);

enum board_led_color {
	BOARD_LED_COLOR_MONO = 0,

	BOARD_LED_COLOR_RED = BOARD_LED_COLOR_MONO,
	BOARD_LED_COLOR_GREEN,
	BOARD_LED_COLOR_BLUE,

	BOARD_LED_COLOR_MAX = BOARD_LED_COLOR_BLUE,
};

struct board_led {
	const struct device *dev;
	int colors;
	uint8_t rate;
	uint8_t state;
};

#define TICK_MS 50

static struct board_led board_leds[] = {
	{
		.dev = DEVICE_DT_GET(DT_ALIAS(led_status)),
		.rate = 255,
	},
	{
		.dev = DEVICE_DT_GET(DT_ALIAS(led_rx)),
		.rate = 48,
	},
	{
		.dev = DEVICE_DT_GET(DT_ALIAS(led_tx)),
		.rate = 4,
	},
};

static uint8_t board_led_brightness = 5;

static int board_init_led(struct board_led *led)
{
	if (!device_is_ready(led->dev)) {
		return -ENODEV;
	}

	for (uint32_t i = 0; i <= BOARD_LED_COLOR_MAX; i++) {
		if (led_off(led->dev, i) != 0) {
			led->colors = i;
			return led->colors;
		}
	}
	led->colors = BOARD_LED_COLOR_MAX + 1;
	return led->colors;
}

static void board_led_on(const struct board_led *led, enum board_led_color color)
{
	if (color >= led->colors)
		return;

	led_set_brightness(led->dev, color, board_led_brightness);
}

static void board_led_off(const struct board_led *led)
{
	for (uint32_t i = 0; i <= led->colors; i++) {
		led_off(led->dev, i);
	}
}

static void board_led_next_state(struct board_led *led)
{
	const uint32_t num = sys_rand32_get();
	const enum board_led_color color = (num & 0x0f) % led->colors;
	const uint8_t rate = (num >> 8) & 0xff;

	board_led_off(led);
	if (!led->state && rate <= led->rate) {
		board_led_on(led, color);
		led->state = 1;
	} else {
		led->state = 0;
	}
}

int main()
{
	LOG_INF("RF Chameleon Blinky");

	for (int i = 0; i < ARRAY_SIZE(board_leds); i++) {
		if (board_init_led(&board_leds[i]) < 0) {
			LOG_WRN("LED %i not ready.", i);
		}
	}

	while (1) {
		for (int i = 0; i < ARRAY_SIZE(board_leds); i++) {
			board_led_next_state(&board_leds[i]);
		}
		k_sleep(K_MSEC(TICK_MS));
	};

	return 0;
}
