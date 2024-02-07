/*
 * SPDX-FileCopyrightText: Copyright 2023-2024 Andreas Sandberg <andreas@sandberg.uk>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "radio.h"

#include <zephyr/logging/log.h>

#include "board.h"
#include "uuids.h"
#include "transport.h"

LOG_MODULE_REGISTER(radio, CONFIG_RFCH_LOG_LEVEL);

#define TX_DELAY K_MSEC(100)

enum rfch_radio_test_type {
	RADIO_TEST_NULL = 0,
};

struct radio_config {
	struct rfch_radio_preset preset;
	enum rfch_radio_test_type type;
};

static const struct radio_config radio_configs[] = {
	{
		/* RF Chameleon Test - Random NULL */
		.preset = {
			.uuid = UUID_PRE_RFCH_TEST_NULL,
			.packet_size = sys_cpu_to_le16(4),
			.rx_meta_size = 0x0,
		},
		.type = RADIO_TEST_NULL,
	},
};

static const struct radio_config *radio_active_config = NULL;
static enum rfch_radio_state radio_state = RFCH_RADIO_STATE_IDLE;

int radio_validate_preset(uint16_t index)
{
	if (index < 0 || index >= ARRAY_SIZE(radio_configs)) {
		LOG_ERR("Invalid radio preset: %d", index);
		return -EINVAL;
	}

	return 0;
}

int radio_get_active_preset()
{
	if (radio_active_config)
		return radio_active_config - radio_configs;

	return -EINVAL;
}

int radio_set_active_preset(uint16_t index)
{
	int ret;

	if (radio_state != RFCH_RADIO_STATE_IDLE)
		return -EINVAL;

	ret = radio_validate_preset(index);
	if (ret < 0) {
		return ret;
	}


	LOG_INF("Activating radio preset %d", index);
	radio_active_config = &radio_configs[index];

	return 0;
}

int radio_get_preset(uint16_t index, const uint8_t **data)
{
	const struct radio_config *config;

	if (!data)
		return -EINVAL;

	if (index >= ARRAY_SIZE(radio_configs))
		return -ENOTSUP;

	config = &radio_configs[index];
	*data = (const uint8_t *)&config->preset;

	return sizeof(config->preset);
}

int radio_get_state()
{
	return radio_state;
}

int _radio_set_state(enum rfch_radio_state state)
{
	radio_state = state;

	switch (state) {
	case RFCH_RADIO_STATE_IDLE:
		board_set_radio_state(BOARD_RADIO_STATE_IDLE);
		return 0;

	case RFCH_RADIO_STATE_RX:
		board_set_radio_state(BOARD_RADIO_STATE_RX);
		return 0;

	case RFCH_RADIO_STATE_TX:
		board_set_radio_state(BOARD_RADIO_STATE_TX);
		return 0;

	case RFCH_RADIO_STATE_ERROR:
		board_set_radio_state(BOARD_RADIO_STATE_ERROR);
		return 0;

	default:
		_radio_set_state(RFCH_RADIO_STATE_ERROR);
		return -EINVAL;
	};

	return 0;
}


int radio_can_set_state(uint16_t state)
{
	switch (state) {
	case RFCH_RADIO_STATE_IDLE:
		return 0;

	case RFCH_RADIO_STATE_RX:
		switch (radio_state) {
		case RFCH_RADIO_STATE_IDLE:
		case RFCH_RADIO_STATE_RX:
			/* Only allow transitions to RX if a preset has been
			 * activated. */
			return !radio_active_config ? -EINVAL : 0;
		default:
			return -EINVAL;
		}

	case RFCH_RADIO_STATE_TX:
		/* The TX state is entered by transmitting a packet. */
		return -EINVAL;

	case RFCH_RADIO_STATE_ERROR:
		/* The error state can't be entered by software. */
		return -EINVAL;

	default:
		LOG_ERR("Unexpected state: %d", (int)state);
		return -EINVAL;
	}
}

int radio_set_state(uint16_t state)
{
	int ret = radio_can_set_state(state);

	if (ret != 0)
               return ret;

	if (radio_state != state)
		return _radio_set_state(state);

	return 0;
}

int radio_can_tx(const uint8_t *data, size_t size, int repeats)
{
	if (!radio_active_config)
		return -EINVAL;

	if (radio_state != RFCH_RADIO_STATE_IDLE)
		return -EINVAL;

	return 0;
}

int radio_tx(const uint8_t *data, size_t size, int repeats)
{
	int ret;

	ret = radio_can_tx(data, size, repeats);
	if (ret < 0)
		return ret;

	_radio_set_state(RFCH_RADIO_STATE_TX);

	LOG_INF("TX");
	k_sleep(TX_DELAY);

	_radio_set_state(RFCH_RADIO_STATE_IDLE);

	return 0;
}

int radio_init()
{
	return 0;
}

int radio_reset()
{
	radio_set_state(RFCH_RADIO_STATE_IDLE);
	radio_active_config = NULL;

	return 0;
}
