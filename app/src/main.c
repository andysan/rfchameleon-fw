/*
 * SPDX-FileCopyrightText: Copyright 2023 Andreas Sandberg <andreas@sandberg.uk>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "gadget.h"
#include "rfchameleon/version.h"

LOG_MODULE_REGISTER(main, CONFIG_RFCH_LOG_LEVEL);

int main()
{
	int ret;

	LOG_INF("Starting gadget interface...");
	ret = gadget_start();
	if (ret != 0) {
		LOG_ERR("Failed to start USB gadget interface: %d", ret);
	}

	LOG_INF("RF Chameleon " RFCHAMELEON_VERSION_STR
		" started on " CONFIG_ARCH);

	return 0;
}
