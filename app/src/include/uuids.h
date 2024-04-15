/*
 * SPDX-FileCopyrightText: Copyright 2023-2024 Andreas Sandberg <andreas@sandberg.uk>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RFCHAMELEON_UUIDS_H_
#define RFCHAMELEON_UUIDS_H_

/* Firmware UUID (3626D5F9-8454-4519-9EEB-32DA9544D1A4) */
#define UUID_RFCH_FW {						\
		0x36, 0x26, 0xD5, 0xF9, 0x84, 0x54, 0x45, 0x19,	\
		0x9E, 0xEB, 0x32, 0xDA, 0x95, 0x44, 0xD1, 0xA4,	\
	}

/*
 * RF Presets
 */

#define _UUID_PRE_RFCH(x, y) {					\
		(x), (y), 0xE6, 0x93, 0x45, 0x18, 0x4E, 0xA6,	\
		0xB4, 0x7C, 0xCF, 0x8B, 0x5D, 0xE7, 0x43, 0xB4,	\
	}

/* RF Chameleon Chat (0000E693-4518-4EA6-B47C-CF8B5DE743B4) */
#define UUID_PRE_RFCH_CHAT _UUID_PRE_RFCH(0x00, 0x00)


#define _UUID_PRE_TEST(x, y) {					\
		(x), (y), 0x9C, 0xCB, 0x04, 0x44, 0x45, 0x34,	\
		0x89, 0x20, 0xC8, 0xF2, 0xF1, 0x7C, 0xC8, 0x22,	\
	}

/* 00009CCB-0444-4534-8920-C8F2F17CC822 */
#define UUID_PRE_RFCH_TEST_NULL _UUID_PRE_TEST(0, 0)
/* 00019CCB-0444-4534-8920-C8F2F17CC822 */
#define UUID_PRE_RFCH_TEST_LOOPBACK _UUID_PRE_TEST(0, 1)


#define _UUID_PRE_EI_RADIOLINK(x, y) {				\
		(x), (y), 0xF8, 0x0F, 0x61, 0xFD, 0x46, 0xD5,	\
		0x9B, 0xC8, 0x94, 0xAE, 0x07, 0x73, 0xFB, 0xA0,	\
	}

/* Ei RadioLINK (0000F80F-61FD-46D5-9BC8-94AE0773FBA0) */
#define UUID_PRE_EI_RADIOLINK _UUID_PRE_EI_RADIOLINK(0x00, 0x00)
/* Ei RadioLINK Bulk v1 (0100F80F-61FD-46D5-9BC8-94AE0773FBA0) */
#define UUID_PRE_EI_RADIOLINK_BULK_V1 _UUID_PRE_EI_RADIOLINK(0x01, 0x00)
/* Ei RadioLINK Bulk v2 (0101F80F-61FD-46D5-9BC8-94AE0773FBA0) */
#define UUID_PRE_EI_RADIOLINK_BULK_V2 _UUID_PRE_EI_RADIOLINK(0x01, 0x01)

/* Alert Alarm (00003637-087D-41AC-A143-EE6423D40DFC) */
#define UUID_PRE_ALERT_ALARM {			\
	0x00, 0x00, 0x36, 0x37, 0x08, 0x7D, 0x41, 0xAC,		\
	0xA1, 0x43, 0xEE, 0x64, 0x23, 0xD4, 0x0D, 0xFC,		\
	}

#endif /* RFCHAMELEON_CC1101_PRESETS_H_ */
