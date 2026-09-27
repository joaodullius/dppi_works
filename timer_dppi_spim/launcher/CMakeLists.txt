# SPDX-License-Identifier: Apache-2.0
#
# Application-core image used when timer_dppi_spim runs on the FLPR: it starts the
# FLPR (nordic-flpr snippet, like Zephyr's default vpr_launcher) and, unlike
# the default one, requests the HFXO so the TIMER on the FLPR side runs from
# the crystal.

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(hfxo_launcher)

target_sources(app PRIVATE src/main.c)
