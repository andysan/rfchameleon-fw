/*
 * SPDX-FileCopyrightText: Copyright 2024 Andreas Sandberg <andreas@sandberg.uk>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RFCHAMELEON_TRANSPORT_H_
#define RFCHAMELEON_TRANSPORT_H_

#include <stdint.h>

#include <zephyr/sys/byteorder.h>

enum rfch_request {
	RFCH_REQ_GET_FW_VERSION = 0x00,
	RFCH_REQ_BOOTLOADER = 0x01,

	RFCH_REQ_SET_RX = 0x10,
	RFCH_REQ_PRESET_RX = 0x11,
	RFCH_REQ_TX = 0x20,

	RFCH_REQ_GET_RADIO_PRESET = 0x30,
	RFCH_REQ_ACTIVATE_RADIO_PRESET = 0x31,
};

enum rfch_bootloader_type {
	RFCH_BL_REBOOT = 0x00,
	RFCH_BL_ROM = 0x01,
	RFCH_BL_MCUBOOT = 0x02,
};

struct rfch_fw_version_info {
	uint8_t uuid[16];
	uint16_t major;
	uint16_t minor;
} __packed;

struct rfch_bootloader_info {
	uint8_t available;
};

struct rfch_radio_preset {
	uint8_t uuid[16];
	uint16_t packet_size;
	uint8_t rx_meta_size;
} __packed;

enum rfch_radio_state {
	RFCH_RADIO_STATE_IDLE = 0x00,
	RFCH_RADIO_STATE_RX = 0x01,
	RFCH_RADIO_STATE_TX = 0x02,

	RFCH_RADIO_STATE_ERROR = 0xff,
};

#define RFCH_BULK_IN_MAGIC sys_cpu_to_be32(0x52464349)
#define RFCH_BULK_OUT_MAGIC sys_cpu_to_be32(0x5246434F)

enum rfch_bulk_type {
	/*
	 * Host -> Device request/response messages.
	 */
	RFCH_BULK_TYPE_TX = 0x10,
	RFCH_BULK_TYPE_TX_DONE = 0x11,

	/*
	 * Device -> Host async messages
	 */
	RFCH_BULK_TYPE_RX = 0x91,
};

#define BULK_TYPE_IS_OUT(x) (((x) & 1) == 0)
#define BULK_TYPE_IS_IN(x) (((x) & 1) == 1)

/* Host initiated message with device response */
#define BULK_TYPE_IS_SYNC(x) (((x) & 0x80) == 0)
/* Device initiated message with no host response */
#define BULK_TYPE_IS_ASYNC(x) (((x) & 0x80) == 1)

struct rfch_bulk_header {
	uint32_t magic;
	uint16_t type;
	uint16_t flags;
	union {
		struct {
			int32_t errno;
		} in;
		struct {
			int32_t reserved;
		} out;
	};
	uint16_t payload_length;
} __packed;


extern int transport_init();
extern int transport_reset();

extern int transport_handle_get(enum rfch_request req, uint16_t value,
				const uint8_t **data, size_t size);

extern int transport_handle_set(enum rfch_request req, uint16_t value,
				const uint8_t *data, size_t size);

extern void transport_on_radio_rx(const uint8_t *data, size_t size);

/*
 * Transport implementation calls.
 */
extern int transport_impl_write(const struct rfch_bulk_header *hdr,
				const uint8_t *data);

#endif /* RFCHAMELEON_TRANSPORT_H_ */
