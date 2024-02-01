<!--
SPDX-FileCopyrightText: Copyright 2023-2024 Andreas Sandberg <andreas@sandberg.uk>

SPDX-License-Identifier: Apache-2.0
-->

# What is RF Chameleon

RF Chameleon is a set of tools to interface with ISM-band radios. It's
inspired by [RFQuack](https://rfquack.org/) but provides a more
abstract interface to the radio which implies that it needs to make
more assumptions about the underlying protocol. Unlike RFQuack, RF
Chameleon is not intended as an RF exploration tool. RF Chameleon is
intended to be used in the phase after basic exploration when the
underlying radio parameters are known.

# Building and flashing

RF Chameleon uses Zephyr. Follow (Zephyr's getting started
guide)[https://docs.zephyrproject.org/3.5.0/develop/getting_started/index.html]
to setup an Arm (cross) compiler and the west build tool. Once west
has been installed, clone RF Chameleon using the following command:

    west init -m https://github.com/rfchameleon/rfchameleon-fw.git rfchameleon-fw
    cd rfchameleon-fw
    west update

Once the repository has been initialized, follow the build
instructions for your board.

# Supported radios

## Texas Instruments CC1101

The CC1101 radio is supported out of the box. This radio supports the
315 MHz, 433 MHz, 868 MHz, and 915 MHz bands. RF Chameleon currently
uses a custom driver with its own
[device tree bindings](dts/bindings/ti,cc1101.yaml). Other radios in
the same family (e.g., CC2500) will likely work but are untested.

# Supported boards

## RF Chameleon CC1101

The RF Chameleon CC1101 board is a custom board based on an ST
STM32F401 MCU and a Texas Instruments CC1101 radio.

Build the firmware using the following command:

    west build -b rfccc1101 -d build.rfccc1101 rfchameleon/app

To flash the board, connect USB while simultaneously pressing the boot
button. This will activate the chip's built-in ROM bootloader. The
firmware can then be flashed with the default flasher (USB DFU):

    west flash -d build.nrf52

The board has a Tag Connect TC2050 debug header that can be used with
common off-the-shelf debuggers. The board support files contain
presets for ST-Link (OpenOCD), BlackMagic Probe, and J-Link.

## Nordic Semiconductor nRF52840 DK with TI CC1101

The exact board configuration is specified in
[app/boards/nrf52840dk_nrf52840.overlay](app/boards/nrf52840dk_nrf52840.overlay). By
default, the firmware will use the following radio pinout:

| Board GPIO  | CC1101 pin  |
| ----------- | ----------- |
| P1.7        | GDO2        |
| P1.8        | GDO0        |
| P1.12       | CSn         |
| P1.13       | SCLK        |
| P1.14       | MISO / GDO1 |
| P1.15       | MOSI        |

Build the firmware using the following command:

    west build -b nrf52840dk_nrf52840 -d build.nrf52 rfchameleon/app

The firmware can be flashed with the default flasher:

    west flash -d build.nrf52

# Adding support for new boards

Any board with USB support that is supported by
 [Zephyr](https://docs.zephyrproject.org/latest/boards/index.html) can
 be used with RF Chameleon. To support a new board, simply create a
 new device tree overlay that tells the operating system how to access
 a supported radio. The NRF52840DK
 [device tree overlay](app/boards/nrf52840dk_nrf52840.overlay) and
 [configuration](app/boards/nrf52840dk_nrf52840.conf) are a good
 starting points.
