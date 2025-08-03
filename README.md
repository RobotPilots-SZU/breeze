# Breeze

深圳大学RobotPilots战队嵌入式开发框架. 基于Zephyr-RTOS.

## 前言

> 在没有任何信息及日志的情况下诊断问题无异于闭眼开车.

## 快速开始

该小节简单讲述如何开始使用breeze进行开发.

建议在Linux环境下进行开发, Windows用户可安装wsl/虚拟机或双系统.

### 捷径

1. 使用如下脚本可自动初始化开发环境.

    ```
    curl https://raw.githubusercontent.com/RobotPilots-SZU/breeze/refs/heads/main/scripts/breeze-init.sh | bash
    ```

2. 安装Zephyr-SDK，用于编译项目.

    参考官方文档 [Zephyr SDK - Zephyr Project Documentation](https://docs.zephyrproject.org/latest/develop/toolchains/zephyr_sdk.html).

    建议将SDK安装至`/opt/zephyr-sdk`目录下, 完整的SDK体积较大, 建议下载minimal版本，后续使用其自带的配置脚本安装`arm-zephyr-eabi`以及`host-tools`即可满足使用.

### 参考

[Getting Started Guide - Zephyr Project Documentation](https://docs.zephyrproject.org/latest/develop/getting_started/index.html)

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
