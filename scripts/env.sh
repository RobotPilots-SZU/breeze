#!/bin/bash

CUR_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# enable env
echo "[breeze-env] Enableing breeze environment at: $CUR_PATH"

if [ -d "$CUR_PATH/.venv" ]; then
    source "$CUR_PATH/.venv/bin/activate"
    echo "[breeze-env] Found python venv: $CUR_PATH/.venv"
else
    echo "[breeze-env] Python venv not found, is your workspace initialized correctly?"
fi

if [ -d "$CUR_PATH/breeze-sdk/zephyr" ]; then
    source "$CUR_PATH/breeze-sdk/zephyr/zephyr-env.sh"
    echo "[breeze-env] Found zephyr-rtos repo: $CUR_PATH/breeze-sdk/zephyr"
else
    echo "[breeze-env] Zephyr-rtos repo not found, please run \"west update\" to fix it."
fi

if [ -d "$CUR_PATH/breeze-sdk/toolchain" ]; then
    export ZEPHYR_SDK_INSTALL_DIR="$CUR_PATH/breeze-sdk/toolchain"
    echo "[breeze-env] Found built-in toolchain: $ZEPHYR_SDK_INSTALL_DIR"
else
    echo "[breeze-env] Built-in toolchain not found, run \"west sdk\" to check it."
fi

echo "[breeze-env] Done."
