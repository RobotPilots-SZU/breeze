#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/watchdog.h>

/* 
*   日志模块注册
*   改成LOG_LEVEL_DBG即可输出DBG级别的日志
*   输出DBG级别日志需要注意 CONFIG_LOG_PROCESS_THREAD_STACK_SIZE 配置，防止日志过多导致栈溢出
*/
LOG_MODULE_REGISTER(wdg_sample, LOG_LEVEL_INF);
       
/* 在调试暂停的时候暂停看门狗计数 */
// 实测用处不大，在看门狗处同时打两个断点gdb调试就会崩溃
#define DEBUG
#ifndef DEBUG
    int flags = 0;
#else
    int flags = WDT_OPT_PAUSE_HALTED_BY_DBG;
#endif


static const struct device *iwdg = DEVICE_DT_GET(DT_ALIAS(watchdog0));
static const struct device *wwdg = DEVICE_DT_GET(DT_ALIAS(watchdog1));
static int iwdg_channel_id;
static int wwdg_channel_id;
static struct wdt_timeout_cfg iwdg_config;
static struct wdt_timeout_cfg wwdg_config;

static void wdg_timeout_callback(const struct device *dev, int channel_id)
{
    LOG_WRN("Watchdog timeout occurred on device: %s, channel: %d", dev->name, channel_id);
}


int main() 
{
    printk("main started\n"); 

    /* 检查看门狗设备是否就绪 */
    if (!device_is_ready(iwdg)) {
        LOG_ERR("Watchdog device %s is not ready", iwdg->name);
        return -1;
    }
    if (!device_is_ready(wwdg)) {
        LOG_ERR("Watchdog device %s is not ready", wwdg->name);
        return -1;
    }

    /* 配置看门狗超时参数 */
    iwdg_config = (struct wdt_timeout_cfg){
        .window = {
            .max = 2000U,                   /* 超时时间: 2000 毫秒 */
        },
        .callback = NULL,                   /* STM32 IWDG 不支持中断回调，仅支持超时复位 */
        .flags = WDT_FLAG_RESET_SOC,        /* 超时后执行系统复位 */
    };
    wwdg_config = (struct wdt_timeout_cfg){
        .window = {
            .max = 20U,                     /* 窗口最大值（ms） 最大约24.97ms */
        },
        .callback = wdg_timeout_callback,   /* 超时触发时执行回调 */
        .flags = WDT_FLAG_RESET_SOC,        /* 超时后执行系统复位 */
    };

    /* 安装超时配置，返回通道ID */
    iwdg_channel_id = wdt_install_timeout(iwdg, &iwdg_config);
    if (iwdg_channel_id < 0) {
        LOG_ERR("Failed to install iwdg timeout: %d", iwdg_channel_id);
        return -1;
    }
    wwdg_channel_id = wdt_install_timeout(wwdg, &wwdg_config);
    if (wwdg_channel_id < 0) {
        LOG_ERR("Failed to install wwdg timeout: %d", wwdg_channel_id);
        return -1;
    }

    /* 启动看门狗 */
    if (wdt_setup(iwdg, flags) < 0) {
        LOG_ERR("Failed to setup watchdog: %s", iwdg->name);
        return -1;
    }
    if (wdt_setup(wwdg, flags) < 0) {
        LOG_ERR("Failed to setup watchdog: %s", wwdg->name);
        return -1;
    }

    LOG_INF("Watchdog configured and started.");


    while(1)
    {
        /* 喂狗，防止系统复位 */
        if (wdt_feed(iwdg, iwdg_channel_id)) {
            LOG_ERR("Failed to feed watchdog: %s", iwdg->name);
        } else {
            LOG_DBG("Watchdog fed successfully: %s", iwdg->name);
        }
        if (wdt_feed(wwdg, wwdg_channel_id)) {
            LOG_ERR("Failed to feed watchdog: %s", wwdg->name);
        } else {
            LOG_DBG("Watchdog fed successfully: %s", wwdg->name);
        }
        
        k_sleep(K_MSEC(15));

    }

}
