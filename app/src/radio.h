/*
 * SPDX-FileCopyrightText: Copyright 2023-2024 Andreas Sandberg <andreas@sandberg.uk>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RFCHAMELEON_RADIO_H_
#define RFCHAMELEON_RADIO_H_

#include <stdint.h>

extern int radio_init();

extern int radio_get_preset(uint16_t index, const uint8_t **data);
extern int radio_validate_preset(uint16_t index);

extern int radio_get_active_preset();
extern int radio_set_active_preset(uint16_t index);

#endif /* RFCHAMELEON_RADIO_H_ */
