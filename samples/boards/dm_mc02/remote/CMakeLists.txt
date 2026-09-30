cmake_minimum_required(VERSION 3.20)

set(BOARD damiao_mc02)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(remote)

list(APPEND DTC_BINDINGS_DIRS
    ${bindings_DIR}/remote
)

# rc_thread.c 以前在 drivers/remote 里，但组帧 + 解析线程属于应用职责，不是驱动该做的：
# 驱动只吐字节（remote_set_rx_cb）并说明本协议帧的格式（remote_get_parser_ops），
# 字节怎么拼成帧、在哪个线程解析都在这里。
target_sources(app PRIVATE
    src/main.c
    src/rc_thread.c
)

target_include_directories(app PRIVATE include)
