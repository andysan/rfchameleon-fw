#
# SPDX-FileCopyrightText: Copyright 2023 Andreas Sandberg <andreas@sandberg.uk>
#
# SPDX-License-Identifier: Apache-2.0
#

cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(rfchameleon LANGUAGES C VERSION 0.1.0)

target_include_directories(app PRIVATE ${CMAKE_BINARY_DIR}/rfchameleon/include)
configure_file(version.h.in ${CMAKE_BINARY_DIR}/rfchameleon/include/rfchameleon/version.h)

add_subdirectory(src)
