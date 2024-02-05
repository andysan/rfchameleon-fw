/*
 * SPDX-FileCopyrightText: Copyright 2023-2024 Andreas Sandberg <andreas@sandberg.uk>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "transport.h"

#include <zephyr/logging/log.h>
#include <zephyr/sys/reboot.h>

#include <radio/cc1101.h>

#include "uuids.h"
#include "board.h"
#include "radio.h"

LOG_MODULE_REGISTER(transport, CONFIG_RFCH_LOG_LEVEL);

static const struct device *dev_cc1101 =
	DEVICE_DT_GET(DT_COMPAT_GET_ANY_STATUS_OKAY(ti_cc1101));

static const struct rfch_fw_version_info desc_fw_version_info[] = {
	{
		.uuid = UUID_RFCH_FW,
		.major = sys_cpu_to_le16(0),
		.minor = sys_cpu_to_le16(2),
	},
};

static const struct rfch_bootloader_info desc_bootloader_info[] = {
	[ RFCH_BL_REBOOT ] = { 1, },
	[ RFCH_BL_ROM ] = { BOARD_HAVE_ROM_BOOTLOADER, },
	[ RFCH_BL_MCUBOOT ] = { 0, },
};

static int enter_bootloader(enum rfch_bootloader_type type)
{
	/* Sleep for a few milliseconds to give the driver a chance to
	 * disconnect. */
	k_sleep(K_MSEC(100));

	switch (type) {
	case RFCH_BL_REBOOT:
		sys_reboot(SYS_REBOOT_COLD);
		break;
#if BOARD_HAVE_ROM_BOOTLOADER
	case RFCH_BL_ROM:
		board_enter_rom_bootloader();
		break;
#endif
	default:
		return -EINVAL;
	}

	return 0;
}

static int get_desc(const uint8_t **data, const void *desc, size_t size)
{
	if (!data || !desc)
		return -EINVAL;

	*data = desc;
	return size;
}

int transport_handle_get(enum rfch_request req, uint16_t value,
			 const uint8_t **data, size_t size)
{
#define RETURN_DESC_ARRAY(n)						\
	return get_desc((const uint8_t **)data,				\
			value < ARRAY_SIZE((n)) ? &(n)[value] : NULL,   \
			MIN(sizeof(n[0]), size))

	switch (req) {
	case RFCH_REQ_GET_FW_VERSION:
		RETURN_DESC_ARRAY(desc_fw_version_info);

	case RFCH_REQ_BOOTLOADER:
		RETURN_DESC_ARRAY(desc_bootloader_info);

	case RFCH_REQ_GET_RADIO_PRESET:
		return radio_get_preset(value, data);

	default:
		return -ENOTSUP;
	}

#undef RETURN_DESC_ARRAY
}

int transport_validate_set(enum rfch_request req, uint16_t value,
			   const uint8_t *data, size_t size)
{
	switch (req) {
	case RFCH_REQ_BOOTLOADER:
		if (value >= ARRAY_SIZE(desc_bootloader_info) ||
		    !desc_bootloader_info[value].available)
			return -ENOTSUP;
		return 0;

	case RFCH_REQ_TX:
		return 0;

	case RFCH_REQ_SET_RX:
		if (value != 0 && value != 1)
			return -ENOTSUP;
		return 0;

	case RFCH_REQ_PRESET_RX:
	case RFCH_REQ_ACTIVATE_RADIO_PRESET:
		return radio_validate_preset(value);
		break;

	default:
		return -ENOTSUP;
	}
}

int transport_execute_set(enum rfch_request req, uint16_t value,
			  const uint8_t *data, size_t size)
{
	int ret;

	switch (req) {
	case RFCH_REQ_BOOTLOADER:
		ret = enter_bootloader(value);
		if (ret < 0) {
			LOG_ERR("Failed to enter bootloader: %d", ret);
		}
		return ret;

	case RFCH_REQ_SET_RX:
		ret = cc1101_set_state(
			dev_cc1101,
			value ? CC1101_STATE_RX : CC1101_STATE_IDLE);
		if (ret >= 0) {
			board_set_radio_state(
				value ?
				BOARD_RADIO_STATE_RX : BOARD_RADIO_STATE_IDLE);
			return 0;
		} else {
			LOG_WRN("Failed to enter state: %d", ret);
			board_set_radio_state(BOARD_RADIO_STATE_ERROR);
			return ret;
		}

	case RFCH_REQ_PRESET_RX:
		ret = radio_set_active_preset(value);
		if (ret < 0) {
			LOG_WRN("Failed to activate preset: %d", ret);
			board_set_radio_state(BOARD_RADIO_STATE_ERROR);
			return ret;
		}

		ret = cc1101_set_state(dev_cc1101, CC1101_STATE_RX);
		if (ret >= 0) {
			board_set_radio_state(BOARD_RADIO_STATE_RX);
			return 0;
		} else {
			LOG_WRN("Failed to enter state: %d", ret);
			board_set_radio_state(BOARD_RADIO_STATE_ERROR);
			return ret;
		}

	case RFCH_REQ_TX:
		board_set_radio_state(BOARD_RADIO_STATE_TX);
		ret = cc1101_send(dev_cc1101, data, size, value);
		if (ret >= 0) {
			board_set_radio_state(BOARD_RADIO_STATE_IDLE);
			return 0;
		} else {
			LOG_WRN("Send operation failed: %d", ret);
			board_set_radio_state(BOARD_RADIO_STATE_ERROR);
			return ret;
		}

	case RFCH_REQ_ACTIVATE_RADIO_PRESET:
		ret = radio_set_active_preset(value);
		if (ret < 0) {
			LOG_WRN("Failed to activate preset: %d", ret);
			board_set_radio_state(BOARD_RADIO_STATE_ERROR);
			return ret;
		}
		return 0;

	default:
		LOG_WRN("Ignoring unknown request: %" PRIu8, req);
		return -EINVAL;
	}
}
