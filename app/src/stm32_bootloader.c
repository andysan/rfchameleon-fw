/*
 * SPDX-FileCopyrightText: Copyright 2024 Andreas Sandberg <andreas@sandberg.uk>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/sys/reboot.h>

#include "stm32_bootloader.h"

struct boot_vtab {
	uint32_t initial_sp;
	void (*reset)();
};

#if IS_ENABLED(CONFIG_SOC_STM32F401XC)
#define BOOTLOADER_MAGIC 0xB350C929UL
#else
#error Unsupported SoC
#endif

static uint32_t bootloader_flag __attribute__((noinit));

static void enter_bootloader()
{
	const struct boot_vtab *sysmem_vtab = (const struct boot_vtab *)0x1FFF0000;

	/*
	 * Zephyr sets the BASEPRI register early at boot. Reset it to
	 * 0 to avoid interrupt issues in the bootloader (USB won't
	 * work without this).
	 */
	__set_BASEPRI(0);

	__set_MSP(sysmem_vtab->initial_sp);

	__DSB();

	__enable_irq();
	sysmem_vtab->reset();
}

static int early_bootloader()
{
	if (bootloader_flag == BOOTLOADER_MAGIC) {
		bootloader_flag = 0;
		enter_bootloader();
	}

	return 0;
}

FUNC_NORETURN void stm32_reboot_bootloader()
{
	bootloader_flag = BOOTLOADER_MAGIC;
	sys_reboot(SYS_REBOOT_COLD);
}

SYS_INIT(early_bootloader, EARLY, 0);
