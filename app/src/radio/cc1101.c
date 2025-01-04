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
	struct rfch_radio_preset_info preset;
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
	{
		/* Ei RadioLINK */
		.preset = {
			.uuid = UUID_PRE_EI_RADIOLINK,
			.packet_size = sys_cpu_to_le16(20),
			.rx_meta_size = 0x2,
		},
		.cc1101 = &rfcfg_cc1101_ei_radiolink,
	},
	{
		/* Ei RadioLINK Bulk */
		.preset = {
			.uuid = UUID_PRE_EI_RADIOLINK_BULK_V1,
			.packet_size = sys_cpu_to_le16(60),
			.rx_meta_size = 0x2,
		},
		.cc1101 = &rfcfg_cc1101_ei_radiolink_bulk_v1,
	},
	{
		/* Ei RadioLINK Bulk (v2) */
		.preset = {
			.uuid = UUID_PRE_EI_RADIOLINK_BULK_V2,
			.packet_size = sys_cpu_to_le16(124),
			.rx_meta_size = 0x2,
		},
		.cc1101 = &rfcfg_cc1101_ei_radiolink_bulk_v2,
	},
	{
		/* Alert Alarm */
		.preset = {
			.uuid = UUID_PRE_ALERT_ALARM,
			.packet_size = sys_cpu_to_le16(10),
			.rx_meta_size = 0x2,
		},
		.cc1101 = &rfcfg_cc1101_alert_alarm,
	},
};

static const struct radio_config *radio_active_config = NULL;
static enum rfch_radio_state radio_state = RFCH_RADIO_STATE_IDLE;

BUILD_ASSERT(CC1101_STATUS0_RSSI_BPT <= RFCH_BULK_RX_RSSI_BPT);

static int16_t convert_rssi(int8_t rssi)
{
	static const int rf_bpt = CC1101_STATUS0_RSSI_BPT;
	static const int transport_bpt = RFCH_BULK_RX_RSSI_BPT;
	static const int16_t offset =
		CC1101_STATUS0_RSSI_OFFSET * (1 << rf_bpt);

	return ((int16_t)rssi - offset) << (transport_bpt - rf_bpt);
}

static void on_rx(const struct device *dev, const uint8_t *data, uint8_t size,
		  void *user)
{
	const struct cc1101_modem_config *rf_cfg =
		radio_active_config && radio_active_config->cc1101 ?
		radio_active_config->cc1101 : NULL;
	const int have_status = rf_cfg &&
		rf_cfg->pktctrl[0] & CC1101_PKTCTRL1_APPEND_STATUS &&
		size >= 2;
	const int crc_en = rf_cfg &&
		rf_cfg->pktctrl[1] & CC1101_PKTCTRL0_CRC_EN;
	const uint8_t *status;

	struct rfch_rx_info info = {
		.flags = 0,
	};

	info.tick = k_uptime_ticks();

	if (rf_cfg) {
		info.flags |= RFCH_BULK_RX_F_PRESET_VALID;
		info.radio_preset = radio_active_config - radio_configs;
	}

	if (have_status) {
		status = &data[size - 2];

		info.flags |= RFCH_BULK_RX_F_RSSI_VALID |
			(crc_en ? RFCH_BULK_RX_F_CRC_VALID : 0);
		if (crc_en && status[1] & CC1101_STATUS1_CRC_OK)
			info.flags |= RFCH_BULK_RX_F_CRC_OK;
		info.rssi = convert_rssi((int8_t)status[0]);
	}

	transport_on_radio_rx(&info, data, size);
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

	if (radio_state != RFCH_RADIO_STATE_IDLE)
		return -EINVAL;

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
		return -ENOENT;

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
	int ret = 0;
	const enum rfch_radio_state old_state = radio_state;

	radio_state = state;

	switch (state) {
	case RFCH_RADIO_STATE_IDLE:
		/* There is no need to explicitly force the radio into
		 * the IDLE state if we are in the TX state since this
		 * transition happens automatically. Forcing the state
		 * transition effectively acts as a barrier which
		 * introduces a delay between packets in a burst. */
		if (old_state != RFCH_RADIO_STATE_TX) {
			ret = cc1101_set_state(dev_cc1101, CC1101_STATE_IDLE);
			if (ret < 0) {
				LOG_WRN("Failed to enter IDLE state: %d", ret);
				_radio_set_state(RFCH_RADIO_STATE_ERROR);
			}
		}
		board_set_radio_state(BOARD_RADIO_STATE_IDLE);
		return ret;

	case RFCH_RADIO_STATE_RX:
		ret = cc1101_set_state(dev_cc1101, CC1101_STATE_RX);
		if (ret < 0) {
			LOG_WRN("Failed to enter RX state: %d", ret);
			_radio_set_state(RFCH_RADIO_STATE_ERROR);
		} else {
			board_set_radio_state(BOARD_RADIO_STATE_RX);
		}
		return ret;

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

int radio_can_tx(const uint8_t *data, size_t size)
{
	if (!radio_active_config)
		return -EINVAL;

	if (radio_state != RFCH_RADIO_STATE_IDLE)
		return -EINVAL;

	return 0;
}

int radio_tx(const uint8_t *data, size_t size)
{
	int ret;

	ret = radio_can_tx(data, size);
	if (ret < 0)
		return ret;

	_radio_set_state(RFCH_RADIO_STATE_TX);

	ret = cc1101_send(dev_cc1101, data, size);
	if (ret < 0) {
		LOG_WRN("Send operation failed: %d", ret);
		_radio_set_state(RFCH_RADIO_STATE_ERROR);
		return ret;
	}

	_radio_set_state(RFCH_RADIO_STATE_IDLE);

	return 0;
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

int radio_reset()
{
	LOG_DBG("Resetting radio state");

	radio_set_state(RFCH_RADIO_STATE_IDLE);
	radio_active_config = NULL;

	return 0;
}
