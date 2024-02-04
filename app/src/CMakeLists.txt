#
# SPDX-FileCopyrightText: Copyright 2023-2024 Andreas Sandberg <andreas@sandberg.uk>
#
# SPDX-License-Identifier: Apache-2.0
#

target_sources(app PRIVATE main.c)
target_sources(app PRIVATE board.c)
target_sources(app PRIVATE cc1101_presets.c)
target_sources(app PRIVATE transport.c)

target_sources_ifdef(CONFIG_RFCH_TRANSPORT_USB app PRIVATE gadget.c)

target_sources_ifdef(CONFIG_STM32_BOOTLOADER app PRIVATE stm32_bootloader.c)
