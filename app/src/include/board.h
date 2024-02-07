/*
 * SPDX-FileCopyrightText: Copyright 2023-2024 Andreas Sandberg <andreas@sandberg.uk>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RFCHAMELEON_BOARD_H_
#define RFCHAMELEON_BOARD_H_

#include <zephyr/kernel.h>

#define BOARD_HAVE_ROM_BOOTLOADER IS_ENABLED(CONFIG_STM32_BOOTLOADER)

enum board_radio_state {
	BOARD_RADIO_STATE_IDLE = 0,
	BOARD_RADIO_STATE_RX,
	BOARD_RADIO_STATE_TX,
	BOARD_RADIO_STATE_ERROR,
};

enum board_usb_state {
	BOARD_USB_STATE_DISCONNECTED = 0,
	BOARD_USB_STATE_CONNECTED,
	BOARD_USB_STATE_DRIVER,
};

enum board_error_codes {
	BOARD_ERROR_PANIC = 0,
};

extern void board_set_radio_state(enum board_radio_state state);
extern void board_radio_packet();
extern void board_radio_packet_error();
extern void board_set_usb_state(enum board_usb_state state);
extern void board_usb_activity();

#if BOARD_HAVE_ROM_BOOTLOADER
extern FUNC_NORETURN void board_enter_rom_bootloader();
#endif

#endif /* RFCHAMELEON_GADGET_H_ */
