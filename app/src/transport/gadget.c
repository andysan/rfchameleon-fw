/*
 * SPDX-FileCopyrightText: Copyright 2023-2024 Andreas Sandberg <andreas@sandberg.uk>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "gadget.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(gadget, CONFIG_RFCH_LOG_LEVEL);

#include <zephyr/kernel.h>
#include <zephyr/usb/usb_device.h>

#include <usb_descriptor.h>

#include <assert.h>

#include "transport.h"

enum rfch_gadget_bulk_state {
	RFCH_BULK_HEADER = 0,
	RFCH_BULK_DATA,
};

#define USB_BULK_MAX_PACKET_SIZE 64

#define RFCH_EP_IN_IDX 0
#define RFCH_EP_OUT_IDX 1

USBD_CLASS_DESCR_DEFINE(primary, 0) struct {
	struct usb_if_descriptor if0;
	struct usb_ep_descriptor if0_in_ep;
	struct usb_ep_descriptor if0_out_ep;
} __packed rfch_usb_desc = {
	.if0 = {
		.bLength = sizeof(struct usb_if_descriptor),
		.bDescriptorType = USB_DESC_INTERFACE,
		.bInterfaceNumber = 0,
		.bAlternateSetting = 0,
		.bNumEndpoints = 2,
		.bInterfaceClass = USB_BCC_VENDOR,
		.bInterfaceSubClass = 0,
		.bInterfaceProtocol = 0,
		.iInterface = 0,
	},
	.if0_in_ep = {
		.bLength = sizeof(struct usb_ep_descriptor),
		.bDescriptorType = USB_DESC_ENDPOINT,
		.bEndpointAddress = AUTO_EP_IN,
		.bmAttributes = USB_DC_EP_BULK,
		.wMaxPacketSize = sys_cpu_to_le16(USB_BULK_MAX_PACKET_SIZE),
		.bInterval = 0,
	},
	.if0_out_ep = {
		.bLength = sizeof(struct usb_ep_descriptor),
		.bDescriptorType = USB_DESC_ENDPOINT,
		.bEndpointAddress = AUTO_EP_OUT,
		.bmAttributes = USB_DC_EP_BULK,
		.wMaxPacketSize = sys_cpu_to_le16(USB_BULK_MAX_PACKET_SIZE),
		.bInterval = 0,
	},
};

static void rfch_usb_bulk_out_cb(uint8_t ep,
				 enum usb_dc_ep_cb_status_code cb_status);

static int rfch_usb_vendor_handler(struct usb_setup_packet *setup,
				   int32_t *len, uint8_t **data);

static void rfch_usb_status_cb(struct usb_cfg_data *cfg,
			       enum usb_dc_status_code status,
			       const uint8_t *param);

static struct usb_ep_cfg_data rfch_usb_ep[] = {
	{ .ep_cb = usb_transfer_ep_callback, .ep_addr = AUTO_EP_IN },
	{ .ep_cb = rfch_usb_bulk_out_cb, .ep_addr = AUTO_EP_OUT },
};


USBD_DEFINE_CFG_DATA(rfch_usb_config) = {
	.usb_device_description = NULL,
	.interface_descriptor = &rfch_usb_desc.if0,
	.cb_usb_status = rfch_usb_status_cb,
	.interface = {
		.class_handler = NULL,
		.vendor_handler = rfch_usb_vendor_handler,
		.custom_handler = NULL,
	},
	.num_endpoints = ARRAY_SIZE(rfch_usb_ep),
	.endpoint = rfch_usb_ep,
};

static enum usb_dc_status_code usb_status = USB_DC_UNKNOWN;

static enum rfch_gadget_bulk_state bulk_out_state = RFCH_BULK_HEADER;
static struct rfch_packet *bulk_out_pkt = NULL;


static void rfch_usb_reset()
{
	transport_reset();

	bulk_out_state = RFCH_BULK_HEADER;
	if (!bulk_out_pkt) {
		/* This should normally always succeed since the
		 * device has just been reset and there should be no
		 * packets in flight.
		 */
		bulk_out_pkt = transport_try_alloc();
		assert(bulk_out_pkt);
	}
}

static void rfch_usb_status_cb(struct usb_cfg_data *cfg,
			       enum usb_dc_status_code status,
			       const uint8_t *param)
{
	ARG_UNUSED(param);
	ARG_UNUSED(cfg);

	/* Check the USB status and do needed action if required */
	switch (status) {
	case USB_DC_ERROR:
		LOG_DBG("USB device error");
		break;
	case USB_DC_RESET:
		LOG_DBG("USB device reset detected");
		rfch_usb_reset();
		break;
	case USB_DC_CONNECTED:
		LOG_DBG("USB device connected");
		break;
	case USB_DC_CONFIGURED:
		LOG_DBG("USB device configured");
		rfch_usb_reset();
		break;
	case USB_DC_DISCONNECTED:
		LOG_DBG("USB device disconnected");
		break;
	case USB_DC_SUSPEND:
		LOG_DBG("USB device suspended");
		break;
	case USB_DC_RESUME:
		LOG_DBG("USB device resumed");
		break;
	case USB_DC_UNKNOWN:
	default:
		LOG_DBG("USB unknown state");
		break;
	}
}

static int rfch_usb_vendor_handle_to_host(struct usb_setup_packet *setup,
					  int32_t *len, uint8_t **data)
{
	const enum rfch_request req = setup->bRequest;
	const uint16_t value = setup->wValue;
	int ret;

	ret = transport_handle_get(req, value,
				   (const uint8_t **)data, setup->wLength);
	if (ret < 0)
		return ret;

	*len = ret;
	return 0;
}

static int rfch_usb_vendor_handle_to_dev(struct usb_setup_packet *setup,
					 int32_t *len, uint8_t **data)
{
	const enum rfch_request req = setup->bRequest;
	int ret;

	if (!data || !len)
		return -EINVAL;

	ret = transport_handle_set(req, sys_le16_to_cpu(setup->wValue),
				   *data, *len);

	return ret < 0 ? ret : 0;
}

/**
 * Vendor handler is executed in the ISR context
 */
static int rfch_usb_vendor_handler(struct usb_setup_packet *setup,
				 int32_t *len, uint8_t **data)
{
	LOG_DBG("Vendor request:"
		" bRequest: 0x%" PRIx8
		" bmRequestType: 0x%" PRIx8
		" wIndex: 0x%" PRIx16
		" wValue: 0x%" PRIx16,
		setup->bRequest, setup->bmRequestType,
		setup->wIndex, setup->wValue);

	if (setup->RequestType.recipient != USB_REQTYPE_RECIPIENT_INTERFACE) {
		return -ENOTSUP;
	}

	if (usb_reqtype_is_to_host(setup)) {
		return rfch_usb_vendor_handle_to_host(setup, len, data);
	} else {
		return rfch_usb_vendor_handle_to_dev(setup, len, data);
	}
}

static void rfch_usb_bulk_out_cb(uint8_t ep,
				 enum usb_dc_ep_cb_status_code cb_status)
{
	uint32_t read_len;
	int ret;
	struct rfch_bulk_header *header;

	assert(ep == rfch_usb_config.endpoint[RFCH_EP_OUT_IDX].ep_addr);
	assert(cb_status == USB_DC_EP_DATA_OUT);
	assert(bulk_out_pkt);

	header = &bulk_out_pkt->u.bulk;

	switch (bulk_out_state) {
	case RFCH_BULK_HEADER:
		/* Initialize the header here. Note that this could be
		 * a retry.
		 */
		memset(bulk_out_pkt, 0, sizeof(*bulk_out_pkt));

		LOG_DBG("Reading bulk OUT header...");
		ret = usb_ep_read_wait(ep, (uint8_t *)header, sizeof(*header),
				       &read_len);
		if (ret < 0) {
			LOG_ERR("Header read failed: %d", ret);
			usb_ep_set_stall(ep);
			return;
		}

		if (read_len != sizeof(*header)) {
			LOG_ERR("Header too short.");
			usb_ep_set_stall(ep);
			return;
		}

		if (header->magic != RFCH_BULK_OUT_MAGIC) {
			LOG_ERR("Invalid header magic.");
			usb_ep_set_stall(ep);
			return;
		}

		LOG_DBG("Entering data phase. Expecting %" PRIu16 " bytes",
			header->payload_length);

		bulk_out_state = RFCH_BULK_DATA;
		/* We won't actually see any data packets unless there
		 * is a payload.
		 */
		if (header->payload_length > 0) {
			usb_ep_read_continue(ep);
		}
		break;

	case RFCH_BULK_DATA:
		LOG_DBG("Reading at most %d bytes of data",
			sizeof(bulk_out_pkt->data) - bulk_out_pkt->length);
		ret = usb_ep_read_wait(
			ep,
			bulk_out_pkt->data + bulk_out_pkt->length,
			sizeof(bulk_out_pkt->data) - bulk_out_pkt->length,
			&read_len);
		if (ret < 0) {
			LOG_ERR("Data read phase failed: %d", ret);
			bulk_out_state = RFCH_BULK_HEADER;
			usb_ep_read_continue(ep);
			return;
		}
		LOG_DBG("Got %" PRIu32 " bytes", read_len);
		bulk_out_pkt->length += read_len;
		if (bulk_out_pkt->length < header->payload_length) {
			usb_ep_read_continue(ep);
		}
		break;

	default:
		LOG_ERR("Invalid bulk OUT state: %d", bulk_out_state);
		return;
	}

	if (bulk_out_pkt->length >= header->payload_length) {
		if (bulk_out_pkt->length > header->payload_length) {
			LOG_WRN("Data phase returned more data than expected.");
		}

		transport_handle_packet(bulk_out_pkt);

		/* Try to allocate a new header. We'll retry in
		 * transport_impl_cycle() if this fails.
		 */
		bulk_out_pkt = transport_try_alloc();
		bulk_out_state = RFCH_BULK_HEADER;
		if (bulk_out_pkt) {
			LOG_DBG("Waiting for next OUT packet");
			usb_ep_read_continue(ep);
		} else {
			LOG_DBG("Deferring next bulk OUT packet");
		}
	}
}

static void status_cb(enum usb_dc_status_code status, const uint8_t *param)
{
	LOG_DBG("USB status change: %d -> %d", usb_status, status);
	usb_status = status;
}

int transport_impl_write(const struct rfch_bulk_header *hdr,
			 const uint8_t *data)
{
	uint8_t ep;
	size_t length;
	int ret;

	if (!hdr)
		return -EINVAL;

	length = hdr->payload_length;
	if (length && !data)
		return -EINVAL;

	ep = rfch_usb_config.endpoint[RFCH_EP_IN_IDX].ep_addr;
	ret = usb_transfer_sync(ep, (void *)hdr, sizeof(*hdr),
				USB_TRANS_WRITE);
	if (ret != sizeof(*hdr)) {
		LOG_ERR("Header transfer failure: %d", ret);
		return ret < 0 ? ret : -EPIPE;
	}

	if (!length)
		return 0;

	while (length) {
		size_t chunk_length = MIN(length, USB_BULK_MAX_PACKET_SIZE);
		ret = usb_transfer_sync(ep, (void *)data, chunk_length,
					USB_TRANS_WRITE);
		if (ret != chunk_length) {
			LOG_ERR("Transfer failure: %d", ret);
			return hdr->payload_length - length;
		}

		data += ret;
		length -= chunk_length;
	}

	return hdr->payload_length;
}

void transport_impl_cycle()
{
	if (!bulk_out_pkt) {
		assert(bulk_out_state == RFCH_BULK_HEADER);
		bulk_out_pkt = transport_try_alloc();
	}
}

int gadget_start()
{
	int ret;

	ret = transport_init();
	if (ret != 0) {
		LOG_ERR("Failed to initialize transport: %d", ret);
		return ret;
	}

	ret = usb_enable(status_cb);
	if (ret != 0) {
		LOG_ERR("Failed to enable USB: %d", ret);
		return ret;
	}

	LOG_DBG("usb initialized");

	return 0;
}
