# Copyright (c) 2025 RobotPilots-SZU

# keep first
board_runner_args(stm32cubeprogrammer "--port=swd" "--reset-mode=hw")
board_runner_args(openocd --target-handle=_CHIPNAME.cpu0)
board_runner_args(jlink "--device=STM32H723VG" "--speed=10000")
board_runner_args(pyocd "--target=stm32h723vgtx")

# keep first
include(${ZEPHYR_BASE}/boards/common/stm32cubeprogrammer.board.cmake)
include(${ZEPHYR_BASE}/boards/common/openocd-stm32.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
include(${ZEPHYR_BASE}/boards/common/pyocd.board.cmake)
