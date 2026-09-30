# SPDX-License-Identifier: Apache-2.0

zephyr_library()

# 协议无关的公共逻辑，DT7/VT13 两个驱动都要用
zephyr_library_sources(rc_common.c rc_thread.c)

zephyr_library_sources_ifdef(CONFIG_DT7_REMOTE dt7_remote.c)
zephyr_library_sources_ifdef(CONFIG_VT13_REMOTE vt13_remote.c)