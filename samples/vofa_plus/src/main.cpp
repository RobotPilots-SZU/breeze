/**  示例程序：向 SEGGER RTT 上报浮点数据帧，使用JustFloat格式（用于 Vofa+ 读取并绘制可视化曲线）
  *  说明：
  *  - 使用 `SEGGER_RTT` 将原始二进制 float 数据写入上行缓冲区
  *  - 每帧发送 3 个 float 值：timestamp(s)[作为时间戳]、value[数据1]、value*2[数据2]；随后写入帧尾作为帧结束符
  */

#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <SEGGER_RTT.h>

LOG_MODULE_REGISTER(vofa_plus, LOG_LEVEL_DBG);

// RTT 通道索引
#define RTT_CH_LOG 0        // RTT 默认日志通道（Zephyr 的日志系统会使用这个通道输出日志）
#define RTT_CH_VOFA_1 1
#define RTT_CH_VOFA_2 2

// 上行通道环形缓冲区（示例中使用 4KB）
static uint8_t vofa_buf_1[4096];
static uint8_t vofa_buf_2[4096];

// VOFA+ JustFloat模式 帧尾
static const uint8_t vofa_tail[4] = {
    0x00, 0x00, 0x80, 0x7F
};

// 配置 RTT 通道
void vofa_rtt_init(void)
{
    // 上行通道
    SEGGER_RTT_ConfigUpBuffer(
        RTT_CH_VOFA_1,                      // 通道索引
        "vofa_channel_1",                   // 通道名称（仅供调试器显示）
        vofa_buf_1,                         // 缓冲区地址
        sizeof(vofa_buf_1),                 // 缓冲区大小
        SEGGER_RTT_MODE_NO_BLOCK_SKIP       // 无阻塞模式：空间不足时丢弃新数据，不等待
    );
    SEGGER_RTT_ConfigUpBuffer(
        RTT_CH_VOFA_2,
        "vofa_channel_2",
        vofa_buf_2,
        sizeof(vofa_buf_2),
        SEGGER_RTT_MODE_NO_BLOCK_SKIP
    );

    LOG_INF("VOFA RTT channel configured\r\n");
}

int main(void)
{
    // 初始化 RTT 上行通道
    vofa_rtt_init();

    int i = 0;  // 示例数据

    while (1) 
    {
        // 打包一帧 JustFloat 数据：3 个 float（时间戳、数据1、数据2）
        float vofa_data[] = {
            (float)k_uptime_get_32() / 1000.0f, // 时间戳（秒）
            (float)i,                           // 示例值
            (float)(2 * i)                      // 示例值的两倍
        };

        // 将数据包写入 RTT 上行通道
        SEGGER_RTT_Write(RTT_CH_VOFA_1, vofa_data, sizeof(vofa_data));
        // 写入帧尾
        SEGGER_RTT_Write(RTT_CH_VOFA_1, vofa_tail, sizeof(vofa_tail));
        LOG_DBG("VOFA RTT: Sent data");

        i++;    // 模拟数据变化

        k_sleep(K_MSEC(200));
    }
}