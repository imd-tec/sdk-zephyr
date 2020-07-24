# Copyright (c) 2020 IMD Technologies
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_SHTC3 shtc3.c)
zephyr_library_sources_ifdef(CONFIG_SHTC3 __temphum9_driver.c)
