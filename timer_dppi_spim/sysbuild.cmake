# SPDX-License-Identifier: Apache-2.0
#
# When timer_dppi_spim is built for a FLPR target, replace Zephyr's default
# vpr_launcher (samples/basic/minimal) by launcher/, which also requests the
# HFXO. Build with -DSB_CONFIG_VPR_LAUNCHER=n so the default one is not added.

if(BOARD_QUALIFIERS MATCHES "cpuflpr" AND NOT SB_CONFIG_VPR_LAUNCHER)
  string(REPLACE "/" ";" quals ${BOARD_QUALIFIERS})
  list(GET quals 0 launcher_soc)
  string(CONCAT launcher_board ${BOARD} "/" ${launcher_soc} "/cpuapp")

  ExternalZephyrProject_Add(
    APPLICATION hfxo_launcher
    SOURCE_DIR ${APP_DIR}/launcher
    BOARD ${launcher_board}
  )
  sysbuild_cache_set(VAR hfxo_launcher_SNIPPET APPEND REMOVE_DUPLICATES nordic-flpr)
endif()
