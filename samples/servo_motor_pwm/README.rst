.. zephyr:code-sample:: breeze-servo-motor-pwm
   :name: PWM 舵机角度控制
   :relevant-api: pwm_interface

   用 PWM 驱动舵机，从 0° 到 180° 来回扫。

概述
****

舵机吃的是 50Hz PWM（周期 20ms），脉宽 0.5ms–2.5ms 对应 0°–180°。
本 sample 走 breeze 的 ``rp,servo-motor-pwm`` 驱动，设备由 ``DT_ALIAS(servo0)`` 取得：

* ``servo_set_mechanical_limit_angle()`` 设机械行程上下限
* ``servo_set_offset_angle()`` 设零位偏移
* ``servo_set_angle()`` 设目标角度（主循环里 0→180→0 每 10° 停 1 秒）

脉宽范围、最大角度这些都在设备树里，不在代码里——换舵机只改 overlay。

板级支持
********

**本 sample 只支持 ``rm_typec``（DJI C 板）。**

====================== ==================================================================
板子                    说明
====================== ==================================================================
``rm_typec``            ``boards/rm_typec.overlay`` 打开 ``&timers8`` 与 ``pwm8``（ch1 → ``PC6``，
                        预分频 167 得 1MHz 计数），定义 ``servo_180`` 节点并挂 ``servo0`` 别名
``damiao_mc02``         **不支持**。不存在的 pinctrl label
                        ``&tim1_ch1_pe09``，
====================== ==================================================================

在喵板上构建会在下面这一行报错，这是预期的、代表硬件/设备树没有支持：

.. code-block:: c

   static const struct device *servo = DEVICE_DT_GET(DT_ALIAS(servo0));
   /* error: '__device_dts_ord_DT_N_ALIAS_servo0_ORD' was not declared in this scope */

将来要让喵板也能用，移植时注意两点（原先那份 overlay 就栽在第一点上）：

1. H723 的 pinctrl label 是 **``tim1_ch1_pe9``**，不是 ``tim1_ch1_pe09``——Zephyr 的引脚命名
   不做零填充（``PE9`` → ``pe9``，``PB15`` → ``pb15``）。可以在
   ``breeze-sdk/modules/hal/stm32/dts/st/h7/stm32h723vgtx-pinctrl.dtsi`` 里搜确认
2. 喵板的 PWM 输出需要 5V 供电，原 overlay 用 ``&gpioc`` 的 ``pc15`` 做 gpio-hog 拉高使能，
   这块要一起带上

构建和运行
**********

.. zephyr-app-commands::
   :zephyr-app: samples/servo_motor_pwm
   :board: rm_typec
   :goals: build flash

设备树属性
==========

``servo_180`` 节点的属性（binding 见 ``dts/bindings/servo-motor-pwm/rp,servo-motor-pwm.yaml``）：

================== =================================================
属性                说明
================== =================================================
``pwms``            必填。PWM 控制器 + 通道 + 周期，例：``<&pwm8 1 PWM_MSEC(20)>``
``min-pulse``       必填。最小脉宽，单位 ns（``PWM_USEC(500)``）
``max-pulse``       必填。最大脉宽，单位 ns（``PWM_USEC(2500)``）
``max-angle``       必填。最大角度（度）
``min-angle``       可选。最小角度，不写按 0 处理
================== =================================================

本 sample 的 ``prj.conf`` 里还有两项跟驱动相关：

* ``CONFIG_SERVO_MOTOR_PWM=y`` —— 打开驱动
* ``CONFIG_SERVO_INIT_PRIORITY=92`` —— 驱动初始化优先级

输出
====

RTT 控制台可以按 ``LOG_LEVEL_DBG`` 打每个角度，默认只在 ``LOG_LEVEL_INF`` 输出错误。
舵机应当从一头匀速扫到另一头，再扫回来。
