/*
 * SPDX-FileCopyrightText: Copyright 2023 Andreas Sandberg <andreas@sandberg.uk>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RFCHAMELEON_GADGET_H_
#define RFCHAMELEON_GADGET_H_

#include <zephyr/kernel.h>
#include <zephyr/usb/usb_device.h>

#define RFCH_MAX_PACKET_SIZE (CONFIG_RFCH_CC1101_MAX_PKT_SIZE + 2)

#define RFCH_REQ_GET_FW_VERSION 0x00
/* #define RFCH_REQ_GET_RADIO_INFO 0x01 */

#define RFCH_REQ_SET_RX 0x10
#define RFCH_REQ_PRESET_RX 0x11
#define RFCH_REQ_TX 0x20

#define RFCH_REQ_GET_RADIO_PRESET 0x30
#define RFCH_REQ_ACTIVATE_RADIO_PRESET 0x31

struct rfch_usb_fw_version_info {
	uint8_t uuid[16];
	uint16_t major;
	uint16_t minor;
} __packed;

struct rfch_usb_radio_preset {
	uint8_t uuid[16];
	uint16_t packet_size;
	uint8_t rx_meta_size;
} __packed;

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

#if IS_ENABLED(CONFIG_USB_DEVICE_STACK)
extern int gadget_start();
#else
static inline int gadget_start() { return 0; }
#endif

#endif /* RFCHAMELEON_GADGET_H_ */
