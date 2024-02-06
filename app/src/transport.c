/*
 * SPDX-FileCopyrightText: Copyright 2023-2024 Andreas Sandberg <andreas@sandberg.uk>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "transport.h"

#include <zephyr/logging/log.h>
#include <zephyr/sys/reboot.h>

#include "uuids.h"
#include "board.h"
#include "radio.h"

LOG_MODULE_REGISTER(transport, CONFIG_RFCH_LOG_LEVEL);

#define RFCH_MAX_PACKET_SIZE (CONFIG_RFCH_CC1101_MAX_PKT_SIZE + 2)

struct rfch_packet {
	void *fifo_reserved;
	union {
		struct {
			uint8_t request;
			uint16_t value;
		} req;
		struct {
		} rx;
	} u;
	uint16_t length;
	uint8_t data[RFCH_MAX_PACKET_SIZE];
};

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

K_MEM_SLAB_DEFINE_STATIC(
	packet_slab,
	sizeof(struct rfch_packet),
	CONFIG_RFCH_TRANSPORT_SLAB,
	4);

K_FIFO_DEFINE(request_fifo);
K_FIFO_DEFINE(rx_fifo);

static int transport_execute_set(enum rfch_request req, uint16_t value,
				 const uint8_t *data, size_t size);


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

static void handle_fifo_req()
{
	struct rfch_packet *req;
	int ret;

	LOG_DBG("");
	req = k_fifo_get(&request_fifo, K_NO_WAIT);
	if (!req) {
		LOG_ERR("No pending data in request FIFO.");
		return;
	}

	LOG_DBG("Request: 0x%" PRIx8 " Value: 0x%" PRIx16,
		req->u.req.request, req->u.req.value);

	ret = transport_execute_set(req->u.req.request, req->u.req.value,
				    req->data, req->length);

	k_mem_slab_free(&packet_slab, req);
}


static void handle_fifo_rx()
{
	struct rfch_packet *req;
	int ret;

	LOG_DBG("");
	req = k_fifo_get(&rx_fifo, K_NO_WAIT);
	if (!req) {
		LOG_ERR("No pending data in RX FIFO.");
		return;
	}

	ret = transport_impl_rx(req->data, req->length);
	if (ret != req->length) {
		LOG_ERR("Transfer failure: %d", ret);
	}

	k_mem_slab_free(&packet_slab, req);
}


static void transport_main(void *, void *, void *)
{
	static struct k_poll_event events[] = {
		K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_FIFO_DATA_AVAILABLE,
						K_POLL_MODE_NOTIFY_ONLY,
						&request_fifo, 0),
		K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_FIFO_DATA_AVAILABLE,
						K_POLL_MODE_NOTIFY_ONLY,
						&rx_fifo, 0),
	};
	int ret;
	int i;

	LOG_DBG("Transport thread started");

	while (1) {
		ret = k_poll(events, ARRAY_SIZE(events), K_FOREVER);
		if (ret < 0) {
			LOG_WRN("Poll failed: %d", ret);
			continue;
		}

		if (events[0].state == K_POLL_STATE_FIFO_DATA_AVAILABLE) {
			/* Incoming request */
			handle_fifo_req();
		}

		if (events[1].state == K_POLL_STATE_FIFO_DATA_AVAILABLE) {
			/* Received packet from radio */
			handle_fifo_rx();
		}

		for (i = 0; i < ARRAY_SIZE(events); i++) {
			events[i].state = K_POLL_STATE_NOT_READY;
		}
	}
}

K_THREAD_DEFINE(transport_thread,
		CONFIG_RFCH_TRANSPORT_STACK_SIZE,
		transport_main, NULL, NULL, NULL,
		-1, K_ESSENTIAL, 0);

int transport_init()
{
	return radio_init();
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


static int transport_validate_set(enum rfch_request req, uint16_t value,
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

static int transport_execute_set(enum rfch_request req, uint16_t value,
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
		return radio_set_state(
			value ? RFCH_RADIO_STATE_RX : RFCH_RADIO_STATE_IDLE);

	case RFCH_REQ_PRESET_RX:
		ret = radio_set_active_preset(value);
		if (ret < 0) {
			LOG_WRN("Failed to activate preset: %d", ret);
			board_set_radio_state(BOARD_RADIO_STATE_ERROR);
			return ret;
		}
		return radio_set_state(RFCH_RADIO_STATE_RX);

	case RFCH_REQ_TX:
		return radio_tx(data, size, value);

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


int transport_handle_set(enum rfch_request req, uint16_t value,
			 const uint8_t *data, size_t size)
{
	struct rfch_packet *pkt = NULL;
	int ret;

	if (k_mem_slab_alloc(&packet_slab, (void **)&pkt, K_NO_WAIT)) {
		return -ENOMEM;
	}

	if (size >= sizeof(pkt->data)) {
		ret = -ENOMEM;
		goto out;
	}

	pkt->u.req.request = req;
	pkt->u.req.value = value;
	pkt->length = size;
	if (size) {
		memcpy(pkt->data, data, size);
	}

	ret = transport_validate_set(req, value, data, size);

out:
	if (ret < 0 && pkt) {
		k_mem_slab_free(&packet_slab, pkt);
	} else if (pkt) {
		k_fifo_put(&request_fifo, pkt);
	}

	return ret;
}

void transport_on_radio_rx(const uint8_t *data, size_t size)
{
	struct rfch_packet *pkt = NULL;

	LOG_DBG("size: %zd", size);
	if (size > RFCH_MAX_PACKET_SIZE) {
		LOG_ERR("Packet too large: %zd / %d",
			size, RFCH_MAX_PACKET_SIZE);
		board_radio_packet_error();
		return;
	}

	if (k_mem_slab_alloc(&packet_slab, (void **)&pkt, K_NO_WAIT)) {
		LOG_ERR("Dropping RX packet. Packet slab empty.");
		board_radio_packet_error();
		return;
	}

	pkt->length = size;
	memcpy(pkt->data, data, size);

	board_radio_packet();
	k_fifo_put(&rx_fifo, pkt);
}
