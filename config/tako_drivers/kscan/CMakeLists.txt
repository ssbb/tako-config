# Copyright (c) 2023 Sviatoslav Bulbakha
# SPDX-License-Identifier: MIT

zephyr_library_named(zmk__hhkb_drivers__kscan_ec)
zephyr_library_include_directories(${CMAKE_SOURCE_DIR}/include)
zephyr_library_include_directories(${CMAKE_SOURCE_DIR}/module/include/zmk)
zephyr_library_include_directories(${CMAKE_SOURCE_DIR}/module/drivers/kscan)

zephyr_library_sources(kscan_gpio_ec.c)
