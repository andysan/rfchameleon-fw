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

#include "transport.h"

static enum usb_dc_status_code usb_status = USB_DC_UNKNOWN;

#define USB_BULK_MAX_PACKET_SIZE 64

#define RFCH_EP_IN_IDX 0

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

int transport_impl_rx(const uint8_t *data, size_t size)
{
	uint8_t ep;

	ep = rfch_usb_config.endpoint[RFCH_EP_IN_IDX].ep_addr;

	return usb_transfer_sync(ep, (void *)data, size, USB_TRANS_WRITE);
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
