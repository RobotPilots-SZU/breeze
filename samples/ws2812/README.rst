.. zephyr:code-sample:: breeze-ws2812
   :name: WS2812 灯珠控制
   :relevant-api: led_strip_interface

   用 SPI 驱动单颗 WS2812 灯珠，循环切换颜色。

概述
****

WS2812 不是普通 PWM 灯，而是靠 SPI（或 PWM+DMA）模拟单总线时序的单线灯珠。
本 sample 走 Zephyr 的 ``led-strip`` 子系统：

* ``led_strip_length()`` 取灯珠数量
* 在 ``struct led_rgb`` 数组里填颜色，用 ``led_strip_update_rgb()`` 一次性刷出去

``rgb_led`` 设备上电后由 ``Init_ws2812()`` 初始化，随后 ``led_task_test`` 线程每 500 ms
换一种颜色（黄 → 紫 → 橙）循环。

板级支持
********

**本 sample 只支持 ``damiao_mc02``（妙板）。**

``damiao_mc02`` 板级 dts 在 ``&spi6`` 上挂了灯珠节点：

.. code-block:: dts

   rgb_led: ws2812@0 {
       compatible = "worldsemi,ws2812-spi";
       spi-max-frequency = <DT_FREQ_M(4)>;
       frame-format = <SPI_FRAME_FORMAT_TI>;
       chain-length = <1>;
       spi-one-frame = <0x70>;
       spi-zero-frame = <0x40>;
       color-mapping = <LED_COLOR_ID_GREEN>, <LED_COLOR_ID_RED>, <LED_COLOR_ID_BLUE>;
   };

``rm_typec`` 上**既没有灯带硬件，也没有对应的设备树节点**，所以不支持本 sample
（``DT_NODELABEL(rgb_led)`` 会直接编译失败）。

沿用这块板时注意几点：

* 板级已经开了 ``CONFIG_LED_STRIP=y``，本 sample 的 ``prj.conf`` 又显式写了一遍，
  因为 sample 应当自证依赖
* 用的是 C++（``new led_rgb[]``），所以 ``prj.conf`` 里 ``CONFIG_CPP=y``
* 换灯珠长度时改板级 dts 的 ``chain-length``，代码会自动跟着变（不写死）

构建和运行
**********

.. zephyr-app-commands::
   :zephyr-app: samples/ws2812
   :board: damiao_mc02
   :goals: build flash
