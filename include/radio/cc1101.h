/*
 * SPDX-FileCopyrightText: Copyright 2023 Andreas Sandberg <andreas@sandberg.uk>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RFCHAMELEON_CC1101_H_
#define RFCHAMELEON_CC1101_H_

#include <zephyr/kernel.h>
#include <zephyr/device.h>

#include <stdint.h>

#define CC1101_REG_READ 0x80
#define CC1101_REG_WRITE (0)
#define CC1101_REG_BURST 0x40
#define CC1101_REG_MASK 0x3f

#define CC1101_REG_IOCFG2 0x00
#define CC1101_REG_IOCFG1 0x01
#define CC1101_REG_IOCFG0 0x02
#define CC1101_REG_FIFOTHR 0x03
#define CC1101_REG_SYNC1 0x04
#define CC1101_REG_SYNC0 0x05
#define CC1101_REG_PKTLEN 0x06
#define CC1101_REG_PKTCTRL1 0x07
#define CC1101_REG_PKTCTRL0 0x08
#define CC1101_REG_ADDR 0x0o9
#define CC1101_REG_CHANNR 0x0a
#define CC1101_REG_FSCTRL1 0x0b
#define CC1101_REG_FSCTRL0 0x0c
#define CC1101_REG_FREQ2 0x0d
#define CC1101_REG_FREQ1 0x0e
#define CC1101_REG_FREQ0 0x0f
#define CC1101_REG_MDMCFG4 0x10
#define CC1101_REG_MDMCFG3 0x11
#define CC1101_REG_MDMCFG2 0x12
#define CC1101_REG_MDMCFG1 0x13
#define CC1101_REG_MDMCFG0 0x14
#define CC1101_REG_DEVIATN 0x15
#define CC1101_REG_MCSM2 0x16
#define CC1101_REG_MCSM1 0x17
#define CC1101_REG_MCSM0 0x18
#define CC1101_REG_FOCCFG 0x19
#define CC1101_REG_BSCFG 0x1a
#define CC1101_REG_AGCCTRL2 0x1b
#define CC1101_REG_AGCCTRL1 0x1c
#define CC1101_REG_AGCCTRL0 0x1d
#define CC1101_REG_WOREVT1 0x1e
#define CC1101_REG_WOREVT0 0x1f
#define CC1101_REG_WORCTRL 0x20
#define CC1101_REG_FREND1 0x21
#define CC1101_REG_FREND0 0x22

#define CC1101_REG_PARTNUM 0xF0
#define CC1101_REG_VERSION 0xF1
#define CC1101_REG_FREQEST 0xF2
#define CC1101_REG_LQI 0xF3
#define CC1101_REG_RSSI 0xF4
#define CC1101_REG_MARCSTATE 0xF5
#define CC1101_REG_WORTIME1 0xF6
#define CC1101_REG_WORTIME0 0xF7
#define CC1101_REG_PKTSTATUS 0xF8
#define CC1101_REG_TXBYTES 0xFA
#define CC1101_REG_RXBYTES 0xFB

#define CC1101_REG_PATABLE 0x3E
#define CC1101_REG_FIFO 0x3F

#define CC1101_CMD_SRES 0x30
#define CC1101_CMD_SFSTXON 0x31
#define CC1101_CMD_SXOFF 0x32
#define CC1101_CMD_SCAL 0x33
#define CC1101_CMD_SRX 0x34
#define CC1101_CMD_STX 0x35
#define CC1101_CMD_SIDLE 0x36
#define CC1101_CMD_SWOR 0x38
#define CC1101_CMD_SPWD 0x39
#define CC1101_CMD_SFRX 0x3a
#define CC1101_CMD_SFTX 0x3b
#define CC1101_CMD_SWORRST 0x3c
#define CC1101_CMD_SNOP 0x3d

#define CC1101_STATUS_NREADY BIT(7)
#define CC1101_STATUS_STATE_SHIFT 4
#define CC1101_STATUS_STATE_MASK 0x7
#define CC1101_STATUS_STATE(x) \
    (((x) >> CC1101_STATUS_STATE_SHIFT) & CC1101_STATUS_STATE_MASK)
#define CC1101_STATUS_STATE_IDLE 0
#define CC1101_STATUS_STATE_RX 1
#define CC1101_STATUS_STATE_TX 2
#define CC1101_STATUS_STATE_FSTXON 3
#define CC1101_STATUS_STATE_CALIBRATE 4
#define CC1101_STATUS_STATE_SETTLING 5
#define CC1101_STATUS_STATE_RXFIFO_OVERFLOW 6
#define CC1101_STATUS_STATE_TXFIFO_UNDERFLOW 7
#define CC1101_STATUS_BYTES_AVAILABLE 0xf

#define CC1101_RXBYTES_OVERFLOW BIT(7)

#define CC1101_PKTCTRL0_CRC_EN BIT(2)

#define CC1101_PKTCTRL1_APPEND_STATUS BIT(2)

#define CC1101_STATUS0_RSSI_OFFSET 74

/* Number of fraction bits in RSSI */
#define CC1101_STATUS0_RSSI_BPT 1
#define CC1101_STATUS1_CRC_OK BIT(7)
#define CC1101_STATUS1_LQI_MASK 0x7f

enum cc1101_state {
	CC1101_STATE_IDLE = 0,
	CC1101_STATE_RX,
	CC1101_STATE_TX,
	CC1101_STATE_FSTXON,
};

struct cc1101_modem_config {
	/* NOTE: This struct has the same layout as the actual
	 * registers in the CC1101. This makes it possible to transfer
	 * it to the device using a single SPI burst. */
	uint8_t sync[2];
	uint8_t pktlen;
	uint8_t pktctrl[2];
	uint8_t addr;
	uint8_t channr;
	uint8_t fsctrl[2];
	uint8_t freq[3];
	uint8_t mdmcfg[5];
	uint8_t deviatn;
} __packed;

typedef void (*cc1101_recv_callback_t)(const struct device *dev,
				       const uint8_t *data, uint8_t size,
				       void *user);

int cc1101_read_reg(const struct device *dev, uint8_t reg);
int cc1101_write_reg(const struct device *dev, uint8_t reg, uint8_t value);
int cc1101_cmd(const struct device *dev, uint8_t cmd);

int cc1101_send(const struct device *dev, const uint8_t *data, uint8_t size,
		uint8_t repeat);

int cc1101_set_state(const struct device *dev, enum cc1101_state state);

int cc1101_set_recv_callback(const struct device *dev,
			     cc1101_recv_callback_t cb, void *user);

int cc1101_set_modem_config(const struct device *dev,
			    const struct cc1101_modem_config *config);

#endif /* RFCHAMELEON_CC1101_H_ */
