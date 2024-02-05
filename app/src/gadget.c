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
#include <zephyr/sys/byteorder.h>

#include <usb_descriptor.h>

#include <radio/cc1101.h>

#include "board.h"
#include "transport.h"

static const struct device *dev_cc1101 =
	DEVICE_DT_GET(DT_COMPAT_GET_ANY_STATUS_OKAY(ti_cc1101));

static enum usb_dc_status_code usb_status = USB_DC_UNKNOWN;

#define USB_BULK_MAX_PACKET_SIZE 64

#define RFCH_EP_IN_IDX 0

K_MEM_SLAB_DEFINE_STATIC(
	packet_slab,
	sizeof(struct rfch_packet),
	CONFIG_RFCH_USB_PACKET_SLAB,
	4);

K_FIFO_DEFINE(request_fifo);
K_FIFO_DEFINE(rx_fifo);

USBD_CLASS_DESCR_DEFINE(primary, 0) struct {
	struct usb_if_descriptor if0;
	struct usb_ep_descriptor if0_in_ep;
} __packed rfch_usb_desc = {
	.if0 = {
		.bLength = sizeof(struct usb_if_descriptor),
		.bDescriptorType = USB_DESC_INTERFACE,
		.bInterfaceNumber = 0,
		.bAlternateSetting = 0,
		.bNumEndpoints = 1,
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
};

static struct usb_ep_cfg_data rfch_usb_ep[] = {
	{ .ep_cb = usb_transfer_ep_callback, .ep_addr = AUTO_EP_IN },
};

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
		/* TODO: Need to reset the radio here */
		break;
	case USB_DC_CONNECTED:
		LOG_DBG("USB device connected");
		break;
	case USB_DC_CONFIGURED:
		LOG_DBG("USB device configured");
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
	int ret = 0;
	struct rfch_packet *req = NULL;

	if (k_mem_slab_alloc(&packet_slab, (void **)&req, K_NO_WAIT)) {
		return -ENOMEM;
	}
	if (*len >= sizeof(req->data)) {
		ret = -ENOMEM;
		goto out;
	}

	req->u.req.request = setup->bRequest;
	req->u.req.value = sys_le16_to_cpu(setup->wValue);
	req->length = *len;
	if (*len) {
		memcpy(req->data, *data, *len);
	}

	ret = transport_validate_set(req->u.req.request, setup->wValue,
				     *data, *len);

out:
	if (ret < 0 && req) {
		k_mem_slab_free(&packet_slab, req);
	} else if (req) {
		k_fifo_put(&request_fifo, req);
	}

	return ret;
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

static void status_cb(enum usb_dc_status_code status, const uint8_t *param)
{
	LOG_DBG("USB status change: %d -> %d", usb_status, status);
	usb_status = status;
}

static void on_rx(const struct device *dev, const uint8_t *data, uint8_t size,
		  void *user)
{
	struct rfch_packet *req = NULL;

	LOG_DBG("size: %" PRIu8, size);
	if (size > RFCH_MAX_PACKET_SIZE) {
		LOG_ERR("Packet too large: %d / %d",
			size, RFCH_MAX_PACKET_SIZE);
		board_radio_packet_error();
		return;
	}

	if (k_mem_slab_alloc(&packet_slab, (void **)&req, K_NO_WAIT)) {
		LOG_ERR("Dropping RX packet. Packet slab empty.");
		board_radio_packet_error();
		return;
	}

	req->length = size;
	memcpy(req->data, data, size);

	k_fifo_put(&rx_fifo, req);
	board_radio_packet();
}

static void handle_fifo_usb_req()
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
	uint8_t ep;
	int ret;

	LOG_DBG("");
	req = k_fifo_get(&rx_fifo, K_NO_WAIT);
	if (!req) {
		LOG_ERR("No pending data in RX FIFO.");
		return;
	}

	ep = rfch_usb_config.endpoint[RFCH_EP_IN_IDX].ep_addr;
	ret = usb_transfer_sync(ep,
				(void *)req->data, req->length,
				USB_TRANS_WRITE);
	if (ret != req->length) {
		LOG_ERR("Transfer failure: %d", ret);
	}

	k_mem_slab_free(&packet_slab, req);
}

static void gadget_thread_main(void *, void *, void *)
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

	LOG_DBG("USB request thread started");

	while (1) {
		ret = k_poll(events, ARRAY_SIZE(events), K_FOREVER);
		if (ret < 0) {
			LOG_WRN("Poll failed: %d", ret);
			continue;
		}

		if (events[0].state == K_POLL_STATE_FIFO_DATA_AVAILABLE) {
			/* Incoming USB request */
			handle_fifo_usb_req();
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

K_THREAD_DEFINE(gadget_thread,
		CONFIG_RFCH_USB_THREAD_STACK_SIZE,
		gadget_thread_main, NULL, NULL, NULL,
		-1, K_ESSENTIAL, 0);

int gadget_start()
{
	int ret;

	ret = usb_enable(status_cb);
	if (ret != 0) {
		LOG_ERR("Failed to enable USB: %d", ret);
		return ret;
	}

	ret = cc1101_set_recv_callback(dev_cc1101, on_rx, NULL);
	if (ret < 0) {
		LOG_ERR("Failed to set radio callback: %d", ret);
		return ret;
	}

	LOG_DBG("usb initialized");

	return 0;
}
