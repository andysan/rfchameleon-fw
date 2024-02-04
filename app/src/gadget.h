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

extern int gadget_start();

#endif /* RFCHAMELEON_GADGET_H_ */
