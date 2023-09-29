#
# SPDX-FileCopyrightText: Copyright 2023 Andreas Sandberg <andreas@sandberg.uk>
#
# SPDX-License-Identifier: Apache-2.0
#

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_RFCH_CC1101 cc1101.c)
