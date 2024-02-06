/*
 * SPDX-FileCopyrightText: Copyright 2023-2024 Andreas Sandberg <andreas@sandberg.uk>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "radio.h"

#include <zephyr/logging/log.h>

#include <radio/cc1101.h>

#include "board.h"
#include "uuids.h"
#include "cc1101_presets.h"
#include "transport.h"

LOG_MODULE_REGISTER(radio, CONFIG_RFCH_LOG_LEVEL);

struct radio_config {
	struct rfch_radio_preset preset;
	const struct cc1101_modem_config *cc1101;
};

static const struct device *dev_cc1101 =
	DEVICE_DT_GET(DT_COMPAT_GET_ANY_STATUS_OKAY(ti_cc1101));

static const struct radio_config radio_configs[] = {
	{
		/* RF Chameleon Chat */
		.preset = {
			.uuid = UUID_PRE_RFCH_CHAT,
			.packet_size = sys_cpu_to_le16(255),
			.rx_meta_size = 0x2,
		},
		.cc1101 = &rfcfg_cc1101_rfch_chat,
	},
};

static const struct radio_config *radio_active_config = NULL;

static void on_rx(const struct device *dev, const uint8_t *data, uint8_t size,
		  void *user)
{
	transport_on_radio_rx(data, size);
}

int radio_validate_preset(uint16_t index)
{
	const struct radio_config *config;

	if (index < 0 || index >= ARRAY_SIZE(radio_configs)) {
		LOG_ERR("Invalid radio preset: %d", index);
		return -EINVAL;
	}

	config = &radio_configs[index];
	if (!config->cc1101) {
		LOG_ERR("Radio preset has invalid CC1101 config: %d",
			index);
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
	const struct radio_config *config;
	int ret;

	ret = radio_validate_preset(index);
	if (ret < 0) {
		return ret;
	}

	config = &radio_configs[index];

	LOG_INF("Activating radio preset %d", index);
	ret = cc1101_set_modem_config(dev_cc1101, config->cc1101);
	if (ret < 0) {
		LOG_ERR("Failed to set radio modem config: %d", ret);
		radio_active_config = NULL;
		return ret;
	}

	radio_active_config = config;

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

int radio_init()
{
	int ret;

	ret = cc1101_set_recv_callback(dev_cc1101, on_rx, NULL);
	if (ret < 0) {
		LOG_ERR("Failed to set radio callback: %d", ret);
		return ret;
	}

	return 0;
}
