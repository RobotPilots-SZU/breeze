#include "song_lib.h"
#include <drivers/buzzer.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(buzzer, LOG_LEVEL_INF);

// 使用别名，指向当前 DTS 的蜂鸣器节点（rp,buzzer 或 pwm-leds 子节点的 alias）
#define BUZZER_NODE DT_ALIAS(buzzer0)
#if !DT_NODE_HAS_STATUS_OKAY(BUZZER_NODE)
#error "DT alias 'buzzer0' 未定义或禁用"
#endif

static const struct device *buzzer;

int buzzer_init(void)
{
    buzzer = DEVICE_DT_GET(BUZZER_NODE);
    if (!device_is_ready(buzzer)) {
        __ASSERT(false, "Buzzer device not ready");
        return ENODEV;
    }
    LOG_INF("Buzzer is ready!");
    return 0;
}

int main(void)
{
    buzzer_init();

    while (1)
    {
        buzzer_play_song(buzzer, &song_YOU);
        k_sleep(K_SECONDS(5));
    }
    return 0;
}
