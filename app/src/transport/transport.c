/*
 * SPDX-FileCopyrightText: Copyright 2023-2025 Andreas Sandberg <andreas@sandberg.uk>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "transport.h"

#include <zephyr/logging/log.h>
#include <zephyr/sys/reboot.h>

#include <assert.h>

#include "rfchameleon/version.h"

#include "uuids.h"
#include "board.h"
#include "radio.h"

LOG_MODULE_REGISTER(transport, CONFIG_RFCH_LOG_LEVEL);


static const struct rfch_protocol_info desc_protocol_info[] = {
	{
		.uuid = UUID_RFCH_FW,
		.version = RFCH_VERSION(0, 3, 0),
		.max_payload = RFCH_MAX_PACKET_SIZE,
	},
};

static const struct rfch_firmware_info desc_firmware_info[] = {
	{
		.version = RFCH_VERSION(RFCHAMELEON_VERSION_MAJOR,
					RFCHAMELEON_VERSION_MINOR,
					RFCHAMELEON_VERSION_PATCH),
	},
};

static const struct rfch_bootloader_info desc_bootloader_info[] = {
	{
		.type = RFCH_BL_REBOOT,
	},
#if BOARD_HAVE_ROM_BOOTLOADER
	{
		.type = RFCH_BL_ROM,
	},
#endif
};

/* TODO: Initialize variant */
static struct rfch_board_info desc_board_info[] = {
	{
		.variant = 0,
		.rev = 0,
		.compatible = DT_PROP(DT_ROOT, compatible),
	},
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


static int enter_bootloader(int index)
{
	if (index < 0 || index >= ARRAY_SIZE(desc_bootloader_info))
		return -EINVAL;

	/* Sleep for a few milliseconds to give the driver a chance to
	 * disconnect. */
	k_sleep(K_MSEC(100));

	switch (desc_bootloader_info[index].type) {
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

static int convert_error(int ret)
{
	if (ret >= 0)
		return ret;

#define M(e) case -e: return -RFCH_ ## e;

	switch (ret) {
		M(ENOSYS);
		M(EINVAL);
		M(ENOMEM);
		M(ENOENT);

	default: return -RFCH_EUNKNOWN;
	};

#undef M
}

static void handle_bulk_req(union rfch_bulk_header_any *hdr_any,
			    size_t hdr_size,
			    const uint8_t *data, size_t len)
{
	struct rfch_bulk_header *hdr = &hdr_any->header;
	struct rfch_bulk_header resp_hdr = RFCH_MAKE_BULK_RESPONSE(*hdr, 0);
	const uint8_t *resp_data = NULL;
	int ret = 0;

	assert(hdr);
	assert(hdr->payload_length == 0 || data);

	if (hdr->magic != RFCH_BULK_OUT_MAGIC) {
		LOG_ERR("Ignoring OUT request with incorrect magic: 0x%08" PRIx32,
			hdr->magic);
		ret = -EINVAL;
		goto out;
	}

	if (!RFCH_BULK_TYPE_IS_OUT(hdr->type)) {
		LOG_ERR("Ignoring OUT request with IN type: 0x%04" PRIx16,
			hdr->type);
		ret = -EINVAL;
		goto out;
	}

	if (hdr->payload_length > RFCH_MAX_PACKET_SIZE) {
		LOG_ERR("OUT request exceeding maximum data length");
		ret = -ENOMEM;
		goto out;
	} else if (len < hdr->payload_length) {
		LOG_ERR("OUT request with insufficient data");
		ret = -EINVAL;
		goto out;
	} else if (len > hdr->payload_length) {
		LOG_WRN("OUT request with too big payload");
		ret = -EINVAL;
		goto out;
	}

	switch (hdr->type) {
	case RFCH_BULK_TYPE_PING:
		resp_hdr.payload_length = len;
		resp_data = data;
		break;

	case RFCH_BULK_TYPE_GET:
		ret = transport_handle_get(
			hdr->out.value >> 16,
			hdr->out.value & 0xffff,
			&resp_data, RFCH_MAX_PACKET_SIZE);
		resp_hdr.payload_length = MAX(0, ret);
		break;

	case RFCH_BULK_TYPE_SET:
		ret = transport_execute_set(
			hdr->out.value >> 16,
			hdr->out.value & 0xffff,
			data, len);
		break;

	case RFCH_BULK_TYPE_TX:
		ret = radio_tx(data, len);
		break;

	default:
		LOG_ERR("Invalid or unsupported OUT type: %d", hdr->type);
		ret = -ENOSYS;
		goto out;
	}

out:
	resp_hdr.in.ret = convert_error(ret);
	transport_impl_write(&resp_hdr, sizeof(resp_hdr), resp_data);
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

	if (req->is_bulk) {
		LOG_DBG("Bulk OUT");
		handle_bulk_req(&req->u.bulk.hdr, req->u.bulk.hdr_size,
				req->data, req->length);
	} else {
		LOG_DBG("Request: 0x%" PRIx8 " Value: 0x%" PRIx16,
			req->u.req.request, req->u.req.value);

		ret = transport_execute_set(
			req->u.req.request,
			req->u.req.value,
			req->data,
			req->length);
	}

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

	ret = transport_impl_write(&req->u.bulk.hdr.header,
				   req->u.bulk.hdr_size,
				   req->data);
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

		transport_impl_cycle();
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

static void flush_fifo(struct k_fifo *fifo)
{
	struct rfch_packet *req;

	while (1) {
		req = k_fifo_get(&request_fifo, K_NO_WAIT);
		if (!req)
			return;

		k_mem_slab_free(&packet_slab, req);
	}
}

int transport_reset()
{
	LOG_DBG("Resetting transport state");

	flush_fifo(&request_fifo);
	flush_fifo(&rx_fifo);

	return radio_reset();
}

struct rfch_packet *transport_try_alloc()
{
	struct rfch_packet *pkt;

	if (k_mem_slab_alloc(&packet_slab, (void **)&pkt, K_NO_WAIT)) {
		return NULL;
	}

	return pkt;

}

static int get_desc(const uint8_t **data, const void *desc, size_t size)
{
	if (!data)
		return -EINVAL;

	if (!desc)
		return -ENOENT;

	*data = desc;
	return size;
}

static int get_time_info(uint16_t value, const uint8_t **data) {
	static struct rfch_time_info ti = {
		.ticks_per_second = CONFIG_SYS_CLOCK_TICKS_PER_SEC,
	};

	if (!data)
		return -EINVAL;

	if (value != 0)
		return -EINVAL;

	ti.cur_tick = k_uptime_ticks();
	*data = (uint8_t *)&ti;

	return sizeof(ti);
}

int transport_handle_get(enum rfch_request req, uint16_t value,
			 const uint8_t **data, size_t size)
{
#define RETURN_DESC_ARRAY(n)						\
	return get_desc((const uint8_t **)data,				\
			value < ARRAY_SIZE((n)) ? &(n)[value] : NULL,   \
			MIN(sizeof(n[0]), size))

	if (!RFCH_REQ_IS_GET(req))
		return -EINVAL;

	switch (req) {
	case RFCH_REQ_GET_PROTOCOL_INFO:
		RETURN_DESC_ARRAY(desc_protocol_info);

	case RFCH_REQ_GET_FIRMWARE_INFO:
		RETURN_DESC_ARRAY(desc_firmware_info);

	case RFCH_REQ_GET_BOOTLOADER_INFO:
		RETURN_DESC_ARRAY(desc_bootloader_info);

	case RFCH_REQ_GET_BOARD_INFO:
		RETURN_DESC_ARRAY(desc_board_info);

	case RFCH_REQ_GET_TIME_INFO:
		return get_time_info(value, data);

	case RFCH_REQ_GET_RADIO_INFO:
		return -ENOSYS;

	case RFCH_REQ_GET_RADIO_PRESET:
		return radio_get_preset(value, data);

	case RFCH_REQ_GET_RADIO_STATE:
		return -ENOSYS;

	case RFCH_REQ_GET_RADIO_ACTIVE_PRESET:
		return -ENOSYS;

	default:
		return -ENOSYS;
	}

#undef RETURN_DESC_ARRAY
}

static int transport_validate_set(enum rfch_request req, uint16_t value,
				  const uint8_t *data, size_t size)
{
	switch (req) {
	case RFCH_REQ_SET_REBOOT:
		return value < ARRAY_SIZE(desc_bootloader_info) ? 0 : -ENOENT;

	case RFCH_REQ_SET_RADIO_STATE:
		return radio_can_set_state(value);

	case RFCH_REQ_SET_RADIO_ACTIVE_PRESET:
		return radio_validate_preset(value);

	default:
		return -ENOSYS;
	}

}

static int transport_execute_set(enum rfch_request req, uint16_t value,
				 const uint8_t *data, size_t size)
{
	int ret;

	switch (req) {
	case RFCH_REQ_SET_REBOOT:
		ret = enter_bootloader(value);
		if (ret < 0) {
			LOG_ERR("Failed to enter bootloader: %d", ret);
		}
		return ret;

	case RFCH_REQ_SET_RADIO_ACTIVE_PRESET:
		ret = radio_set_active_preset(value);
		if (ret < 0) {
			LOG_ERR("Failed to enter bootloader: %d", ret);
		}
		return ret;

	case RFCH_REQ_SET_RADIO_STATE:
		ret = radio_set_state(value);
		if (ret < 0) {
			LOG_ERR("Failed to set state: %d", ret);
		}
		return ret;

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

	if (!RFCH_REQ_IS_SET(req))
		return -EINVAL;

	if (k_mem_slab_alloc(&packet_slab, (void **)&pkt, K_NO_WAIT)) {
		return -ENOMEM;
	}

	if (size >= sizeof(pkt->data)) {
		ret = -ENOMEM;
		goto out;
	}

	pkt->is_bulk = 0;
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

int transport_handle_packet(struct rfch_packet *pkt)
{
	if (!pkt) {
		return -EINVAL;
	}

	pkt->is_bulk = 1;
	k_fifo_put(&request_fifo, pkt);

	return 0;
}

void transport_on_radio_rx(const struct rfch_rx_info *info,
			   const uint8_t *data, size_t size)
{
	struct rfch_packet *pkt = NULL;
	struct rfch_bulk_header_rx hdr = {
		.header = {
			.magic = RFCH_BULK_IN_MAGIC,
			.type = RFCH_BULK_TYPE_RX,
			.flags = 0,
			.in.ret = 0,
			.payload_length = size,
		},
		.rx = *info,
	};

	LOG_DBG("size: %" PRIu8, size);
	if (size > UINT16_MAX) {
		LOG_ERR("Packet too large: %d / %d",
			size, UINT16_MAX);
		board_radio_packet_error();
		return;
	}

	if (k_mem_slab_alloc(&packet_slab, (void **)&pkt, K_NO_WAIT)) {
		LOG_ERR("Dropping RX packet. Packet slab empty.");
		board_radio_packet_error();
		return;
	}

	pkt->u.bulk.hdr.rx = hdr;
	pkt->u.bulk.hdr_size = sizeof(hdr);
	pkt->length = size;
	memcpy(pkt->data, data, size);

	board_radio_packet();
	k_fifo_put(&rx_fifo, pkt);
}
