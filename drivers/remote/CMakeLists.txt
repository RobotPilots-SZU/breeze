# SPDX-License-Identifier: Apache-2.0

zephyr_library()

# 协议无关的公共逻辑，DT7/VT13 两个驱动都要用。
# 组帧 + 解析线程不在这里：驱动只吐字节（remote_set_rx_cb）并说明本协议帧的
# 格式（remote_get_parser_ops），拼帧和解析都是应用的事，
# 参考实现见 samples/boards/dm_mc02/remote/src/rc_thread.c。
zephyr_library_sources(rc_common.c)

zephyr_library_sources_ifdef(CONFIG_DT7_REMOTE dt7_remote.c)
zephyr_library_sources_ifdef(CONFIG_VT13_REMOTE vt13_remote.c)