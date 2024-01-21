/*
 * SPDX-FileCopyrightText: Copyright 2023-2024 Andreas Sandberg <andreas@sandberg.uk>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "cc1101_presets.h"

/*
 * RF Chameleon Chat
 *
 * Base freq: 868.1MHz (50.0 kHz channel spacing)
 * Modulation: GFSK, 38.4 kb/s, 20.6 kHz deviation, whitening
 *
 */
const struct cc1101_modem_config rfcfg_cc1101_rfch_chat = {
	.sync = {0x5f, 0xca},
	.pktlen = 0xff,
	.pktctrl = {0x04, 0x45},
	.addr = 0x00,
	.channr = 0x00,
	.fsctrl = {0x06, 0x00},
	.freq = {0x21, 0x63, 0x72},
	.mdmcfg = {0xca, 0x83, 0x17, 0x00, 0xf8 },
	.deviatn = 0x35,
};

/* Ei RadioLINK */
const struct cc1101_modem_config rfcfg_cc1101_ei_radiolink = {
	.sync = { 0xaa, 0xac },
	.pktlen = 20,
	.pktctrl = { 0x04, 0x00 },
	.fsctrl = { 0x06, 0x00 },
	/* 868.500 MHz */
	.freq =  { 0x21, 0x67, 0x62 },
	/* 38.383 kbps */
	.mdmcfg = { 0x8a, 0x83, 0x1e, 0x02, 0xf8 },
	/* 31.738 kHz */
	.deviatn = 0x42,
};
