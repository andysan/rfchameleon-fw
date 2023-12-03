/*
 * SPDX-FileCopyrightText: Copyright 2023 Andreas Sandberg <andreas@sandberg.uk>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "board.h"

#include <stdlib.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led.h>
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log_ctrl.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(board, CONFIG_RFCH_LOG_LEVEL);

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
};

static struct board_led led_status = {
	.dev = DEVICE_DT_GET(DT_ALIAS(led_status)),
};

static struct board_led led_rx = {
	.dev = DEVICE_DT_GET(DT_ALIAS(led_rx)),
};

static struct board_led led_tx = {
	.dev = DEVICE_DT_GET(DT_ALIAS(led_tx)),
};

static const struct gpio_dt_spec gpio_variant[] = {
#if DT_NODE_EXISTS(DT_NODELABEL(sw_variant0))
	GPIO_DT_SPEC_GET(DT_NODELABEL(sw_variant0), gpios),
#endif
#if DT_NODE_EXISTS(DT_NODELABEL(sw_variant1))
	GPIO_DT_SPEC_GET(DT_NODELABEL(sw_variant1), gpios),
#endif
};

static uint8_t board_led_brightness =
	DT_PROP_OR(DT_PATH(zephyr_user), led_brightness, 100);
static uint32_t board_variant = 0;

static enum board_usb_state board_usb_state = BOARD_USB_STATE_DISCONNECTED;
static enum board_radio_state board_radio_state = BOARD_RADIO_STATE_IDLE;

static int board_usb_toggle = 0;
static int board_packet_toggle = 0;
static int board_packet_error = 0;

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

static void board_test_led(const struct board_led *led)
{
	for (int i = 0; i < led->colors; i++) {
		for (int j = 0; j <= 100; j += 10) {
			led_set_brightness(led->dev, i, j);
			k_sleep(K_MSEC(5));
		}
		for (int j = 100; j >= 0; j -= 10) {
			led_set_brightness(led->dev, i, j);
			k_sleep(K_MSEC(5));
		}
		led_off(led->dev, i);
	}
}

static void board_led_on(const struct board_led *led,
			 enum board_led_color color)
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

static void board_leds_off()
{
	board_led_off(&led_status);
	board_led_off(&led_rx);
	board_led_off(&led_tx);
}

static void board_set_error(enum board_error_codes error)
{
	board_leds_off();

	if (error & 1) {
		board_led_on(&led_tx, BOARD_LED_COLOR_MONO);
	}

	if (error & 2) {
		board_led_on(&led_rx, BOARD_LED_COLOR_MONO);
	}
}

static int board_test_quick()
{
	board_test_led(&led_status);
	board_test_led(&led_rx);
	board_test_led(&led_tx);

	return 0;
}

static void board_update_leds_rgb()
{
	enum board_led_color usb_led_color =
		board_usb_toggle ? BOARD_LED_COLOR_BLUE : BOARD_LED_COLOR_GREEN;
	enum board_led_color rf_led_color = (
		board_packet_error ? BOARD_LED_COLOR_RED :
		(board_packet_toggle ?
		 BOARD_LED_COLOR_BLUE : BOARD_LED_COLOR_GREEN));

	board_led_off(&led_status);
	board_led_off(&led_rx);
	board_led_off(&led_tx);

	switch (board_usb_state) {
	case BOARD_USB_STATE_DISCONNECTED:
		break;

	case BOARD_USB_STATE_CONNECTED:
	case BOARD_USB_STATE_DRIVER:
		board_led_on(&led_status, usb_led_color);
		break;

	default:
		LOG_ERR("Invalid USB state: %i", board_usb_state);
		abort();
	}

	switch (board_radio_state) {
	case BOARD_RADIO_STATE_IDLE:
		break;

	case BOARD_RADIO_STATE_RX:
		board_led_on(&led_rx, rf_led_color);
		break;

	case BOARD_RADIO_STATE_TX:
		board_led_on(&led_tx, rf_led_color);
		break;

	case BOARD_RADIO_STATE_ERROR:
		board_led_on(&led_tx, BOARD_LED_COLOR_RED);
		board_led_on(&led_rx, BOARD_LED_COLOR_RED);
		break;

	default:
		LOG_ERR("Invalid radio state: %i", board_usb_state);
		abort();
	};
}

static void board_update_leds_mono()
{
	switch (board_usb_state) {
	case BOARD_USB_STATE_DISCONNECTED:
		board_led_off(&led_status);
		break;

	case BOARD_USB_STATE_CONNECTED:
	case BOARD_USB_STATE_DRIVER:
		board_led_on(&led_status, BOARD_LED_COLOR_MONO);
		break;

	default:
		LOG_ERR("Invalid USB state: %i", board_usb_state);
		abort();
	}

	switch (board_radio_state) {
	case BOARD_RADIO_STATE_IDLE:
		board_led_off(&led_rx);
		board_led_off(&led_tx);
		break;

	case BOARD_RADIO_STATE_RX:
		board_led_on(&led_rx, BOARD_LED_COLOR_MONO);
		board_led_off(&led_tx);
		break;

	case BOARD_RADIO_STATE_TX:
		board_led_off(&led_rx);
		board_led_on(&led_tx, BOARD_LED_COLOR_MONO);
		break;

	case BOARD_RADIO_STATE_ERROR:
		board_led_on(&led_rx, BOARD_LED_COLOR_MONO);
		board_led_on(&led_tx, BOARD_LED_COLOR_MONO);
		break;

	default:
		LOG_ERR("Invalid radio state: %i", board_usb_state);
		abort();
	};
}

static void board_update_leds()
{
	if (led_status.colors == 3 ||
	    led_rx.colors == 3 ||
	    led_tx.colors == 3) {
		board_update_leds_rgb();
	} else {
		board_update_leds_mono();
	}
}

static int board_update_variant()
{
	int rc;
	uint32_t value = 0;

	for (int i = 0; i < ARRAY_SIZE(gpio_variant); i++) {
		if (!gpio_is_ready_dt(&gpio_variant[i])) {
			LOG_ERR("Variant %i: GPIO not ready.", i);
			return -EINVAL;
		}

		rc = gpio_pin_configure_dt(&gpio_variant[i], GPIO_INPUT);
		if (rc < 0) {
			LOG_ERR("Variant %i: Failed to configure as input: %i",
				i, rc);
			return -EINVAL;
		}

		rc = gpio_pin_get_dt(&gpio_variant[i]);
		if (rc < 0) {
			LOG_ERR("Variant %i: Failed to get value: %i", i, rc);
			return -EINVAL;
		}

		if (rc)
			value |= BIT(i);

		rc = gpio_pin_configure_dt(&gpio_variant[i], GPIO_DISCONNECTED);
		if (rc < 0) {
			LOG_WRN("Variant %i: Failed to disconnect GPIO: %i",
				i, rc);
		}
	}

	board_variant = value;

	return 0;
}

void board_set_radio_state(enum board_radio_state state)
{
	board_radio_state = state;
	board_packet_error = 0;
	board_packet_toggle = 0;
	board_update_leds();
}

void board_radio_packet()
{
	board_packet_error = 0;
	board_packet_toggle = !board_packet_toggle;
	board_update_leds();
}

void board_radio_packet_error()
{
	board_packet_toggle = 0;
	board_packet_error = 1;
	board_update_leds();
}

void board_set_usb_state(enum board_usb_state state)
{
	board_usb_toggle = 0;
	board_usb_state = state;
	board_update_leds();
}

void board_usb_activity()
{
	board_usb_toggle = !board_usb_toggle;
	board_update_leds();
}

void k_sys_fatal_error_handler(unsigned int reason, const z_arch_esf_t *esf)
{
	LOG_PANIC();
	LOG_ERR("Halting system");

	board_set_error(BOARD_ERROR_PANIC);

	while (1) {
		led_on(led_status.dev, 0);
		k_busy_wait(500000);
		led_off(led_status.dev, 0);
		k_busy_wait(500000);
	}
}

static int board_init()
{
	if (board_init_led(&led_status) < 0) {
		LOG_WRN("Status LED not ready.");
	}

	if (board_init_led(&led_rx) < 0) {
		LOG_WRN("RX LED not ready.");
	}

	if (board_init_led(&led_tx) < 0) {
		LOG_WRN("TX LED not ready.");
	}

	if (board_update_variant() < 0) {
		LOG_WRN("Failed to get board variant information.");
	} else {
		LOG_INF("Board variant: 0x%" PRIx32, board_variant);
	}

	board_test_quick();

	return 0;
}

SYS_INIT(board_init, APPLICATION, 0);

#if IS_ENABLED(CONFIG_SHELL)
static int cmd_board_test(const struct shell *sh, size_t argc, char **argv)
{
	return board_test_quick();
}

static int cmd_board_info(const struct shell *sh, size_t argc, char **argv)
{
	shell_print(sh, "Board variant bits: %i\n", ARRAY_SIZE(gpio_variant));
	shell_print(sh, "Board variant: 0x%" PRIx32 "\n", board_variant);

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_board,
	SHELL_CMD_ARG(test, NULL,
		      "Run board tests\n",
		      cmd_board_test,
		      0, 0),
	SHELL_CMD_ARG(info, NULL,
		      "Show board information\n",
		      cmd_board_info,
		      0, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(board, &sub_board, "Board specific commands", NULL);

#endif
