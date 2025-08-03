# breeze

深圳大学RobotPilots战队嵌入式开发框架.

基于Zephyr-RTOS.

## 快速开始

该小节简单讲述如何开始使用breeze进行开发

### 捷径

1. 使用如下脚本可自动初始化开发环境.

    ```
    curl https://raw.githubusercontent.com/RobotPilots-SZU/breeze/refs/heads/main/scripts/breeze-init.sh | bash
    ```

2. 安装Zephyr-SDK，用于编译项目.

    参考官方文档 [Zephyr SDK - Zephyr Project Documentation](https://docs.zephyrproject.org/latest/develop/toolchains/zephyr_sdk.html).

    建议将SDK安装至`/opt`目录下, 完整的SDK体积较大, 建议下载minimal版本，后续使用其自带的配置脚本安装`arm-zephyr-eabi`以及`host-tools`即可满足使用.

### 参考

[Getting Started Guide - Zephyr Project Documentation](https://docs.zephyrproject.org/latest/develop/getting_started/index.html)
