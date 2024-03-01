/*
 * SPDX-FileCopyrightText: Copyright 2024 Andreas Sandberg <andreas@sandberg.uk>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RFCHAMELEON_TRANSPORT_H_
#define RFCHAMELEON_TRANSPORT_H_

#include <stdint.h>

#include <zephyr/sys/byteorder.h>

#define RFCH_MAX_PACKET_SIZE (CONFIG_RFCH_CC1101_MAX_PKT_SIZE + 2)

enum rfch_request {
	RFCH_REQ_GET_PROTOCOL_INFO = 0x00,
	RFCH_REQ_GET_FIRMWARE_INFO = 0x02,
	RFCH_REQ_GET_BOOTLOADER_INFO = 0x04,
	RFCH_REQ_SET_REBOOT = 0x05,
	RFCH_REQ_GET_BOARD_INFO = 0x06,

	RFCH_REQ_GET_RADIO_INFO = 0x10,
	RFCH_REQ_GET_RADIO_PRESET = 0x12,
	RFCH_REQ_GET_RADIO_STATE = 0x14,
	RFCH_REQ_SET_RADIO_STATE = 0x15,
	RFCH_REQ_GET_RADIO_ACTIVE_PRESET = 0x16,
	RFCH_REQ_SET_RADIO_ACTIVE_PRESET = 0x17,
};

enum rfch_error {
	/* Unknown error occurred */
	RFCH_EUNKNOWN = 1,
	/* Invalid call type */
	RFCH_ENOSYS = 2,
	/* Invalid argument */
	RFCH_EINVAL = 3,
	/* Memory allocation failure */
	RFCH_ENOMEM = 4,
	/* Entry does not exist (e.g., valid call but invalid index). */
	RFCH_ENOENT = 5,
};

#define RFCH_REQ_IS_SET(x) (((x) & 0x01) == 1)
#define RFCH_REQ_IS_GET(x) (((x) & 0x01) == 0)

enum rfch_bootloader_type {
	RFCH_BL_REBOOT = 0x00,
	RFCH_BL_ROM = 0x01,
	RFCH_BL_MCUBOOT = 0x02,
};

#define RFCH_BL_F_HAS_VERSION 0x01

struct rfch_version {
	uint16_t major;
	uint16_t minor;
	uint16_t rev;
};

#define RFCH_VERSION(MAJ, MIN, REV) \
	{ sys_cpu_to_le16(MAJ), sys_cpu_to_le16(MIN), sys_cpu_to_le16(REV) }

struct rfch_protocol_info {
	uint8_t uuid[16];
	struct rfch_version version;
	uint16_t max_payload;
} __packed;

struct rfch_firmware_info {
	struct rfch_version version;
} __packed;

struct rfch_bootloader_info {
	uint8_t type;
	uint16_t flags;
	struct rfch_version version;
} __packed;

struct rfch_board_info {
	uint32_t variant;
	uint16_t rev;
	char compatible[40];
} __packed;

struct rfch_radio_preset_info {
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
	RFCH_BULK_TYPE_PING = 0x00,
	RFCH_BULK_TYPE_PONG = 0x01,

	RFCH_BULK_TYPE_GET = 0x02,
	RFCH_BULK_TYPE_GET_RESP = 0x03,

	RFCH_BULK_TYPE_SET = 0x04,
	RFCH_BULK_TYPE_SET_RESP = 0x05,

	RFCH_BULK_TYPE_TX = 0x10,
	RFCH_BULK_TYPE_TX_DONE = 0x11,

	/*
	 * Device -> Host async messages
	 */
	RFCH_BULK_TYPE_RX = 0x91,
};

#define RFCH_BULK_TYPE_IS_OUT(x) (((x) & 1) == 0)
#define RFCH_BULK_TYPE_IS_IN(x) (((x) & 1) == 1)

/* Host initiated message with device response */
#define RFCH_BULK_TYPE_IS_SYNC(x) (((x) & 0x80) == 0)
/* Device initiated message with no host response */
#define RFCH_BULK_TYPE_IS_ASYNC(x) (((x) & 0x80) == 1)

#define RFCH_BULK_TYPE_MAKE_RESPONSE(x) ((x) | 1)

struct rfch_bulk_header {
	uint32_t magic;
	uint16_t type;
	uint16_t flags;
	union {
		struct {
			int32_t ret;
		} in;
		struct {
			uint32_t value;
		} out;
	};
	uint16_t payload_length;
} __packed;

#define RFCH_BULK_RX_F_CRC_OK BIT(0)
#define RFCH_BULK_RX_F_CRC_VALID BIT(1)
#define RFCH_BULK_RX_F_PRESET_VALID BIT(2)
#define RFCH_BULK_RX_F_CHANNEL_VALID BIT(3)
#define RFCH_BULK_RX_F_RSSI_VALID BIT(4)

/* Number of fraction bits in RSSI */
#define RFCH_BULK_RX_RSSI_BPT 4

struct rfch_rx_info {
	uint16_t flags;
	uint16_t radio_preset;
	uint16_t channel;
	/** RSSI in dBm * 16 (i.e., 4 fraction bits) */
	int16_t rssi;
};

struct rfch_bulk_header_rx {
	struct rfch_bulk_header header;
	struct rfch_rx_info rx;
} __packed;

union rfch_bulk_header_any {
	struct rfch_bulk_header header;
	struct rfch_bulk_header_rx rx;
};

#define RFCH_MAKE_BULK_RESPONSE(x, ret_or_errno) {		\
		.magic = RFCH_BULK_IN_MAGIC,			\
		.type = RFCH_BULK_TYPE_MAKE_RESPONSE((x).type),	\
		.flags = 0,					\
		.in.ret = (ret_or_errno),			\
		.payload_length = 0,				\
	}


struct rfch_packet {
	void *fifo_reserved;
	int is_bulk;
	union {
		struct {
			uint8_t request;
			uint16_t value;
		} req;
		struct {
			union rfch_bulk_header_any hdr;
			size_t hdr_size;
		} bulk;
	} u;
	uint16_t length;
	uint8_t data[RFCH_MAX_PACKET_SIZE];
};

extern int transport_init();
extern int transport_reset();

extern struct rfch_packet *transport_try_alloc();

extern int transport_handle_get(enum rfch_request req, uint16_t value,
				const uint8_t **data, size_t size);

extern int transport_handle_set(enum rfch_request req, uint16_t value,
				const uint8_t *data, size_t size);

extern int transport_handle_packet(struct rfch_packet *pkt);

extern void transport_on_radio_rx(const struct rfch_rx_info *info,
				  const uint8_t *data, size_t size);

/*
 * Transport implementation calls.
 */
extern int transport_impl_write(const struct rfch_bulk_header *hdr,
				size_t hdr_size,
				const uint8_t *data);

extern void transport_impl_cycle();

#endif /* RFCHAMELEON_TRANSPORT_H_ */
