# Breeze

深圳大学RobotPilots战队嵌入式开发框架. 基于Zephyr-RTOS.

## 前言

> 在没有任何信息及日志的情况下诊断问题无异于闭眼开车.

## 快速开始

该小节简单讲述如何开始使用breeze进行开发.

建议在Linux环境下进行开发, Windows用户可安装wsl/虚拟机或双系统.

### 自动挡

1. 使用如下脚本可自动初始化开发环境, 并安装编译工具链.

    ```
    curl https://raw.githubusercontent.com/RobotPilots-SZU/breeze/refs/heads/main/scripts/breeze-init.sh | bash
    ```

2. 启用breeze-workspace内的虚拟环境 `source env.sh` , 后续每次重新启动终端时都需使用本命令配置虚拟环境.

3. 启动VSCode `code breeze-app/[app-folder]` , 将`[app-folder]`替换为需要打开的项目, 开始进行App开发.

### 手动挡

参考文献: [Getting Started Guide - Zephyr Project Documentation](https://docs.zephyrproject.org/latest/develop/getting_started/index.html)

1. 在合适的路径下使用 `mkdir breeze-workspace && cd breeze-workspace` 创建并进入工作区.

2. 更新apt软件包 `sudo apt update && sudo apt upgrade` .

3. 安装apt包依赖

    ```
    sudo apt install --no-install-recommends git cmake ninja-build gperf ccache dfu-util device-tree-compiler wget \
        python3-dev python3-venv python3-tk python-is-python3 xz-utils file make gcc g++ gdb-multiarch \
        gcc-multilib g++-multilib libsdl2-dev libmagic1 openocd -y
    ```

4. 创建并启用Python虚拟环境 `python -m venv .venv && source .venv/bin/activate` .

5. 在虚拟环境中安装West `pip install west` .

6. 在当前位置下, 使用Breeze作为清单仓库初始化West `west init -m https://github.com/RobotPilots-SZU/breeze.git .` .

7. 使用West拉取Breeze的剩余部分 `west update` , 仓库数量较多, 建议多次运行该命令保证拉取成功.

8. 使用West安装Python包依赖 `west packages pip --install` , 包数量较多/依赖关系复杂, 同样建议多次运行该命令保证所有包都成功安装.

9. 使用West安装编译工具链 `west sdk install --install-dir breeze-sdk/toolchain --toolchains arm-zephyr-eabi` .

10. 拷贝环境初始化脚本 `cp breeze/scripts/env.sh env.sh` .

11. 启用breeze-workspace内的虚拟环境 `source env.sh` , 后续每次重新启动终端时都需使用本命令配置虚拟环境.

12. 启动VSCode `code breeze-app/[app-folder]` , 将 `[app-folder]` 替换为需要打开的项目, 开始进行App开发.

## 文件结构

如下所示为一个breeze工作区的文件目录结构:

```
breeze-workspace            工作区，用于隔离开发环境 (如Python的venv)
    |
    |-  breeze              Breeze主清单仓库，拥有额外添加的主板移植以及驱动程序等
    |
    |-  breeze-sdk          SDK目录，主要为Zephyr及其附属模块
    |       | - zephyr      Zephyr-RTOS主仓库
    |       | - modules     Zephyr子模块仓库存放目录
    |
    |-  breeze-app          APP目录，用户进行开发时存放相关仓库的位置
    |       | - app1        用户app1仓库
    |       | - app2        用户app2仓库
    |       | - ...         更多用户app仓库
```
