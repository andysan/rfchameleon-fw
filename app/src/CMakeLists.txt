#
# SPDX-FileCopyrightText: Copyright 2023-2024 Andreas Sandberg <andreas@sandberg.uk>
#
# SPDX-License-Identifier: Apache-2.0
#

target_include_directories(app PRIVATE include)

target_sources(app PRIVATE main.c)
target_sources(app PRIVATE board.c)

target_sources_ifdef(CONFIG_RFCH_BACKEND_CC1101 app PRIVATE
	radio/cc1101_presets.c
	radio/cc1101.c
)

target_sources(app PRIVATE transport/transport.c)
target_sources_ifdef(CONFIG_RFCH_TRANSPORT_USB app PRIVATE transport/gadget.c)

target_sources_ifdef(CONFIG_STM32_BOOTLOADER app PRIVATE stm32_bootloader.c)
