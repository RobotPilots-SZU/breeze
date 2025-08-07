.. zephyr:code-sample:: can-dual-controller
   :name: 控制器局域网(CAN)双控制器LED控制
   :relevant-api: can_interface

   双CAN控制器通信与LED控制演示。

概述
****

此示例演示如何使用双控制器局域网(CAN)控制器进行交叉通信和LED控制。示例特性包括：

* 两个独立的CAN控制器(CAN1和CAN2)
* 交叉通信：CAN1消息控制LED1，CAN2消息控制LED0
* 实时CAN状态监控和错误计数
* 通过Kconfig配置消息ID和时序
* k_msgq用于消息发送队列
* 支持总线过载测试

示例在两个CAN控制器之间发送交替的LED控制命令，
演示双向通信和LED状态指示。

构建和运行
**********
对于C板：

.. zephyr-app-commands::
   :zephyr-app: samples/boards/rm_typec/can
   :board: rm_typec

配置选项
========

示例支持以下Kconfig选项：

* ``CONFIG_CAN1_MSG_ID``：CAN1消息ID（默认：0x11）
* ``CONFIG_CAN2_MSG_ID``：CAN2消息ID（默认：0x22）
* ``CONFIG_SLEEP_TIME_MS``：消息传输间隔，单位毫秒（默认：500）
* ``CONFIG_CAN_TEST_OVERLOAD``：使能CAN总线过载测试（默认关闭）。开启后，消息将尽可能快地发送，用于测试总线负载和容错能力。

示例输出
========
CAN1和CAN2接好线后，led开始闪烁。
.. code-block:: console
   *** Booting Zephyr OS build v4.2.0-1258-g810ccf123e81 ***
   [00:00:00.000,000] <inf> can_sample: Configuring CAN RX filters and callbacks
   [00:00:00.000,000] <inf> can_sample: CAN1 RX filter added successfully
   [00:00:00.000,000] <inf> can_sample: CAN2 RX filter added successfully
   [00:00:00.000,000] <inf> can_sample: LED0 initialized successfully
   [00:00:00.000,000] <inf> can_sample: LED1 initialized successfully
   [00:00:00.000,000] <inf> can_sample: Initialization complete. Starting CAN communication test...
   [00:00:00.000,000] <inf> can_sample: CAN2 received message: ID=0x11, data[0]=0x01
   [00:00:00.000,000] <inf> can_sample: CAN1 received message: ID=0x22, data[0]=0x01
   [00:00:00.500,000] <inf> can_sample: CAN2 received message: ID=0x11, data[0]=0x00
   [00:00:00.500,000] <inf> can_sample: CAN1 received message: ID=0x22, data[0]=0x00
   [00:00:01.000,000] <inf> can_sample: CAN2 received message: ID=0x11, data[0]=0x01
   [00:00:01.000,000] <inf> can_sample: CAN1 received message: ID=0x22, data[0]=0x01
   [00:00:01.500,000] <inf> can_sample: CAN2 received message: ID=0x11, data[0]=0x00
   [00:00:01.500,000] <inf> can_sample: CAN1 received message: ID=0x22, data[0]=0x00
   [00:00:02.000,000] <inf> can_sample: CAN2 received message: ID=0x11, data[0]=0x01
   [00:00:02.000,000] <inf> can_sample: CAN1 received message: ID=0x22, data[0]=0x01

拔掉CAN1或CAN2。输出发送错误。
.. code-block:: console
   [00:00:38.508,000] <inf> can_sample: CAN2 received message: ID=0x11, data[0]=0x00
   [00:00:38.508,000] <inf> can_sample: CAN1 received message: ID=0x22, data[0]=0x00
   [00:00:39.008,000] <err> can_sample: State change ISR - State: error-active, RX error count: 0, TX error count: 8
   [00:00:39.008,000] <err> can_sample: State change ISR - State: error-warning, RX error count: 0, TX error count: 96
   [00:00:39.009,000] <err> can_sample: State change ISR - State: error-passive, RX error count: 0, TX error count: 128
   [00:00:39.039,000] <wrn> can_sample: CAN state: error-passive, RX error count: 0, TX error count: 128

重新插回后，输出恢复正常。
.. code-block:: console
   [00:01:33.695,000] <inf> can_sample: CAN1 received message: ID=0x22, data[0]=0x01
   [00:01:33.794,000] <wrn> can_sample: CAN state: error-warning, RX error count: 0, TX error count: 121
   [00:01:34.195,000] <inf> can_sample: CAN2 received message: ID=0x11, data[0]=0x00
   [00:01:34.195,000] <inf> can_sample: CAN1 received message: ID=0x22, data[0]=0x00
   [00:01:34.295,000] <wrn> can_sample: CAN state: error-warning, RX error count: 0, TX error count: 120
   [00:01:34.695,000] <inf> can_sample: CAN2 received message: ID=0x11, data[0]=0x01
   [00:01:34.695,000] <inf> can_sample: CAN1 received message: ID=0x22, data[0]=0x01
   [00:01:34.795,000] <wrn> can_sample: CAN state: error-warning, RX error count: 0, TX error count: 119
   [00:01:35.195,000] <inf> can_sample: CAN2 received message: ID=0x11, data[0]=0x00
   [00:01:35.195,000] <inf> can_sample: CAN1 received message: ID=0x22, data[0]=0x00
   [00:01:35.195,000] <wrn> can_sample: CAN state: error-warning, RX error count: 0, TX error count: 118
   [00:01:35.695,000] <inf> can_sample: CAN2 received message: ID=0x11, data[0]=0x01
   [00:01:35.695,000] <inf> can_sample: CAN1 received message: ID=0x22, data[0]=0x01
   [00:01:35.696,000] <wrn> can_sample: CAN state: error-warning, RX error count: 0, TX error count: 117
   [00:01:36.195,000] <inf> can_sample: CAN2 received message: ID=0x11, data[0]=0x00
   [00:01:36.195,000] <inf> can_sample: CAN1 received message: ID=0x22, data[0]=0x00
   [00:01:36.196,000] <wrn> can_sample: CAN state: error-warning, RX error count: 0, TX error count: 116

使能总线过载测试后，看到队列满和消息丢弃的情况。
.. code-block:: console
   [00:00:06.734,000] <wrn> can_sample: CAN2 send queue full, drop LED1 OFF frame
   [00:00:06.750,000] <inf> can_sample: CAN1 received message: ID=0x22, data[0]=0x01
   --- 549 messages dropped ---
   [00:00:06.766,000] <inf> can_sample: CAN2 received message: ID=0x11, data[0]=0x00
   --- 320 messages dropped ---
   [00:00:06.782,000] <inf> can_sample: CAN1 received message: ID=0x22, data[0]=0x00
   --- 327 messages dropped ---
   [00:00:06.798,000] <inf> can_sample: CAN1 received message: ID=0x22, data[0]=0x00
   --- 322 messages dropped ---
   [00:00:06.814,000] <inf> can_sample: CAN1 received message: ID=0x22, data[0]=0x00
   --- 321 messages dropped ---
   [00:00:06.831,000] <inf> can_sample: CAN1 received message: ID=0x22, data[0]=0x00
   --- 324 messages dropped ---
   [00:00:06.847,000] <inf> can_sample: CAN2 received message: ID=0x11, data[0]=0x00
   --- 322 messages dropped ---
   [00:00:06.863,000] <inf> can_sample: CAN2 received message: ID=0x11, data[0]=0x00
   --- 324 messages dropped ---
   [00:00:06.879,000] <inf> can_sample: CAN1 received message: ID=0x22, data[0]=0x01
   --- 321 messages dropped ---
   [00:00:06.895,000] <inf> can_sample: CAN1 received message: ID=0x22, data[0]=0x00
   --- 321 messages dropped ---
   [00:00:06.912,000] <wrn> can_sample: CAN1 send queue full, drop LED0 OFF frame
   --- 328 messages dropped ---
   [00:00:06.928,000] <inf> can_sample: CAN1 received message: ID=0x22, data[0]=0x01
