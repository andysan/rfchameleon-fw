/*
 * SPDX-FileCopyrightText: Copyright 2024 Andreas Sandberg <andreas@sandberg.uk>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RFCHAMELEON_CC1101_PRESETS_H_
#define RFCHAMELEON_CC1101_PRESETS_H_

#include <radio/cc1101.h>

extern const struct cc1101_modem_config rfcfg_cc1101_rfch_chat;

extern const struct cc1101_modem_config rfcfg_cc1101_ei_radiolink;
extern const struct cc1101_modem_config rfcfg_cc1101_ei_radiolink_bulk_v1;
extern const struct cc1101_modem_config rfcfg_cc1101_ei_radiolink_bulk_v2;

extern const struct cc1101_modem_config rfcfg_cc1101_alert_alarm;

#endif /* RFCHAMELEON_CC1101_PRESETS_H_ */
