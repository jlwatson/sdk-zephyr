# SPDX-License-Identifier: Apache-2.0

if(CONFIG_LIVE_UPDATE)
  zephyr_interface_library_named(LIVE_UPDATE)

  zephyr_library()
  zephyr_library_include_directories(${CMAKE_CURRENT_SOURCE_DIR})
  zephyr_library_sources(lu_init.c lu_idle.c lu_uart.c)
endif()

